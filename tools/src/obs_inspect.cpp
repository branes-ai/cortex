// SPDX-License-Identifier: MIT
//
// obs_inspect — real-data observability / null-space-leak diagnostic (#212, #337).
//
// The synthetic observability_probe (sdk/eval/observability_probe.hpp, #337)
// established the MECHANISM behind the EuRoC over-confidence: a monocular VIO has
// a 4-DoF unobservable gauge (global translation x3, yaw x1); a consistent filter
// must never gain information along it, i.e. the stacked camera Jacobian must
// annihilate the gauge, H*N ~ 0. That holds only at a SINGLE consistent
// linearization; a standard EKF re-linearizes each clone at its own drifted
// estimate, so across a window H*N != 0 and the filter fabricates information --
// over-confidence. The synthetic probe showed translation stays ~0 while YAW
// leaks and grows with the perturbation sigma.
//
// This tool MEASURES that leak on REAL data. It runs the shipped estimator over a
// real EuRoC sequence and, for every MSCKF update, drives the SHIPPED Jacobian
// (CameraUpdater::projection_jacobians, real extrinsic) on the LIVE clone window +
// the live triangulated feature, and forms the stacked H exactly as the updater
// does. Two leaks per update:
//
//   * consistent leak -- H and N both at the current clone estimates => the
//     gauge-annihilation sanity check on real geometry (must be ~ machine-eps; a
//     nonzero value would mean the production Jacobian itself is wrong, not the
//     linearization-point story);
//   * real leak -- H at the clone window PERTURBED by the FILTER'S OWN claimed
//     per-clone sigma (read from P), N at the unperturbed gauge, averaged over
//     several seeded draws. This is the synthetic sigma-sweep evaluated at the
//     uncertainty the filter actually reports on this window: given that
//     uncertainty, how much yaw/translation information does the shipped H
//     spuriously inject?
//   * R-IEKF leak -- the SAME perturbed windows run through the shipped
//     right-invariant Jacobian (build_invariant_measurement) + the invariant gauge,
//     whose clone directions are estimate-independent constants. On V2_03 this is
//     ~1e-16 where the standard leak is ~0.4 -- the candidate fix, validated on
//     real data.
//
// Output: per-update JSONL (window size, mean clone sigma, consistent/real yaw &
// translation leak, NIS) + a summary localizing the leak to yaw vs translation on
// the real sequence. Read-only -- no filter change. EuRoC (~1.5 GB) is not
// vendored, so this is a developer diagnostic.
//
// Usage:
//   obs_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N] [--draws K]
//   Then inspect <out>/observability.jsonl.

#include <branes/cv/image_io.hpp>
#include <branes/math/cameras.hpp>
#include <branes/math/lie/detail.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/sfm/init_window.hpp>
#include <branes/sdk/vio_estimator.hpp>
#include <branes/tools/observability_inspect.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace bs = branes::sdk;
namespace bt = branes::tools;
namespace cv = branes::cv;
namespace ms = branes::sdk::msckf;
namespace ld = branes::math::lie::detail;
using json = nlohmann::json;
using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T, Backend>;
using DVec3 = ld::Vec<T, 3>;
using DMat3 = ld::Mat<T, 3, 3>;
using DSO3 = branes::math::lie::SO3<T>;

namespace {

struct Args {
    std::string dataset;
    std::string out = "build/stage_probes/OBS";
    std::uint64_t max_frames = std::numeric_limits<std::uint64_t>::max();
    int draws = 8;  // seeded perturbation draws to average the real leak
    bool help = false;
};

std::uint64_t to_u64(const std::string& raw) {
    std::size_t pos = 0;
    unsigned long long v = std::stoull(raw, &pos);
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("obs_inspect: invalid integer '" + raw + "'");
    return v;
}

Args parse(int argc, char** argv) {
    Args a;
    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("obs_inspect: missing value after " + std::string(argv[i]));
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        std::string_view v = argv[i];
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--dataset")
            a.dataset = next(i);
        else if (v == "--out")
            a.out = next(i);
        else if (v == "--max-frames")
            a.max_frames = to_u64(next(i));
        else if (v == "--draws")
            a.draws = static_cast<int>(to_u64(next(i)));
        else
            throw std::runtime_error("obs_inspect: unknown argument '" + std::string(v) + "'");
    }
    return a;
}

// EuRoC MAV cam0 calibration (intrinsics + extrinsic), matching s6_inspect/vio_pipeline.
Backend::CameraCalibration euroc_cam0() {
    Backend::CameraCalibration cal;
    cal.intrinsics =
        Backend::Camera(458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    DMat3 R{};
    R(0, 0) = 0.0148655429818;
    R(0, 1) = -0.999880929698;
    R(0, 2) = 0.00414029679422;
    R(1, 0) = 0.999557249008;
    R(1, 1) = 0.0149672133247;
    R(1, 2) = 0.025715529948;
    R(2, 0) = -0.0257744366974;
    R(2, 1) = 0.00375618835797;
    R(2, 2) = 0.999660727178;
    cal.extrinsics.R_imu_cam = bs::sfm::so3_from_matrix<T>(R);
    cal.extrinsics.p_imu_cam = DVec3{{-0.0216401454975, -0.064676986768, 0.00981073058949}};
    return cal;
}

using bt::ObsPose;

void usage() {
    std::cout << "obs_inspect -- real-data observability / null-space-leak diagnostic (#212, #337)\n\n"
                 "  obs_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N] [--draws K]\n";
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        args = parse(argc, argv);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 2;
    }
    if (args.help) {
        usage();
        return 0;
    }
    if (args.dataset.empty()) {
        std::cerr << "obs_inspect: --dataset <EuRoC mav0 dir> is required.\n\n";
        usage();
        return 2;
    }

    std::vector<bs::ImuMeasurement<T>> imu;
    std::vector<bs::euroc::ImageEntry> images;
    try {
        imu = bs::euroc::parse_imu<T>(args.dataset);
        images = bs::euroc::parse_images(args.dataset);
    } catch (const std::exception& ex) {
        std::cerr << "obs_inspect: " << ex.what() << "\n";
        return 1;
    }
    if (images.empty() || imu.empty()) {
        std::cerr << "obs_inspect: empty EuRoC streams\n";
        return 1;
    }

    const auto cal = euroc_cam0();
    const ms::CameraUpdater<T> updater(std::vector<ms::CameraExtrinsics<T>>{cal.extrinsics});

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    std::ofstream os(args.out + "/observability.jsonl");
    if (!os) {
        std::cerr << "obs_inspect: cannot write " << args.out << "/observability.jsonl\n";
        return 1;
    }

    Estimator est(Backend(std::vector<Backend::CameraCalibration>{cal}));
    bs::VioConfig cfg;
    est.configure(cfg);
    est.activate();

    const DVec3 g{{T{0}, T{0}, T{1}}};  // yaw axis = world up (config gravity is -z)
    double frame_t = 0.0;
    std::uint64_t n_updates = 0, n_measured = 0;
    double sum_cons_yaw = 0.0, sum_cons_tr = 0.0;
    double sum_real_yaw = 0.0, sum_real_tr = 0.0, max_real_yaw = 0.0;
    double sum_inv_yaw = 0.0, max_inv_yaw = 0.0;  // R-IEKF leak at the SAME perturbations

    est.backend().set_update_observer([&](const ms::State<T>& s,
                                          const ms::FeatureTrack<T>& track,
                                          const ms::NisSample<T>& nis,
                                          bool accepted,
                                          const ms::DynMat<T>&) {
        ++n_updates;
        const std::size_t m = track.observations.size();
        if (m < 2)
            return;

        DVec3 p_f{};
        if (!updater.triangulate(s, track.observations, p_f))
            return;

        std::vector<ObsPose<T>> cl;
        cl.reserve(m);
        const ms::DynMat<T> P = s.covariance();
        std::vector<T> sth(m, T{0}), sp(m, T{0});  // per-clone theta / p sigma from P
        for (std::size_t i = 0; i < m; ++i) {
            const std::size_t ci = track.observations[i].clone_index;
            if (ci >= s.clones.size())
                return;
            cl.push_back(ObsPose<T>{s.clones[ci].R, s.clones[ci].p});
            const std::size_t off = s.clone_offset(ci);
            T vth = T{0}, vp = T{0};
            for (std::size_t k = 0; k < 3; ++k) {
                vth += P(off + k, off + k);
                vp += P(off + 3 + k, off + 3 + k);
            }
            sth[i] = std::sqrt(vth / T{3});
            sp[i] = std::sqrt(vp / T{3});
        }

        // (1) Consistent leak: H and N at the same current estimate => must be ~0.
        const ms::DynMat<T> N = bt::obs_build_N<T>(cl, p_f, g);
        const auto [cons_tr, cons_yaw] = bt::obs_leak<T>(bt::obs_build_H<T>(updater, cl, p_f), N);

        // (2) Real leak: perturb the clone window by the filter's OWN per-clone
        // sigma, N at the unperturbed gauge, averaged over seeded draws. For each
        // perturbed window also measure the RIGHT-INVARIANT (R-IEKF) leak — the
        // candidate fix — on the SAME geometry: its gauge directions are constants,
        // so it should stay ~0 where the standard one leaks.
        const ms::DynMat<T> Ninv = bt::obs_build_N_invariant<T>(cl, p_f, g);
        double dyaw = 0.0, dtr = 0.0, dyaw_inv = 0.0;
        std::mt19937_64 rng(0x0B5E11ull ^ (n_updates * 0x9E3779B97F4A7C15ull));
        for (int d = 0; d < args.draws; ++d) {
            std::vector<ObsPose<T>> pe = cl;
            for (std::size_t i = 0; i < m; ++i) {
                pe[i].R = pe[i].R * DSO3::exp(DVec3{{sth[i] * bt::obs_urand<T>(rng),
                                                     sth[i] * bt::obs_urand<T>(rng),
                                                     sth[i] * bt::obs_urand<T>(rng)}});
                pe[i].p = DVec3{{pe[i].p[0] + sp[i] * bt::obs_urand<T>(rng),
                                 pe[i].p[1] + sp[i] * bt::obs_urand<T>(rng),
                                 pe[i].p[2] + sp[i] * bt::obs_urand<T>(rng)}};
            }
            const auto [tr, yaw] = bt::obs_leak<T>(bt::obs_build_H<T>(updater, pe, p_f), N);
            dyaw += static_cast<double>(yaw);
            dtr += static_cast<double>(tr);
            const auto inv = bt::obs_leak<T>(bt::obs_build_H_invariant<T>(pe, p_f, cal.extrinsics), Ninv);
            dyaw_inv += static_cast<double>(inv.second);
        }
        dyaw /= args.draws;
        dtr /= args.draws;
        dyaw_inv /= args.draws;

        T mean_sth = T{0}, mean_sp = T{0};
        for (std::size_t i = 0; i < m; ++i) {
            mean_sth += sth[i];
            mean_sp += sp[i];
        }
        mean_sth /= static_cast<T>(m);
        mean_sp /= static_cast<T>(m);

        ++n_measured;
        sum_cons_yaw += static_cast<double>(cons_yaw);
        sum_cons_tr += static_cast<double>(cons_tr);
        sum_real_yaw += dyaw;
        sum_real_tr += dtr;
        sum_inv_yaw += dyaw_inv;
        if (dyaw > max_real_yaw)
            max_real_yaw = dyaw;
        if (dyaw_inv > max_inv_yaw)
            max_inv_yaw = dyaw_inv;

        json j{{"index", n_updates - 1},
               {"t", frame_t},
               {"n_obs", m},
               {"accepted", accepted},
               {"nis_over_dof", nis.valid && nis.dof > 0 ? static_cast<double>(nis.value) / nis.dof : 0.0},
               {"clone_sigma_theta_deg", static_cast<double>(mean_sth) * 180.0 / 3.14159265358979323846},
               {"clone_sigma_p_mm", static_cast<double>(mean_sp) * 1000.0},
               {"leak_yaw_consistent", static_cast<double>(cons_yaw)},
               {"leak_trans_consistent", static_cast<double>(cons_tr)},
               {"leak_yaw_real", dyaw},
               {"leak_trans_real", dtr},
               {"leak_yaw_invariant", dyaw_inv}};
        os << j.dump() << '\n';
    });

    std::size_t imu_idx = 0;
    std::uint64_t processed = 0;
    for (const auto& frame : images) {
        if (processed >= args.max_frames)
            break;
        std::size_t end = imu_idx;
        while (end < imu.size() && imu[end].timestamp_s <= frame.t_s)
            ++end;
        if (end > imu_idx) {
            est.feed_imu(std::span<const bs::ImuMeasurement<T>>{imu.data() + imu_idx, end - imu_idx});
            imu_idx = end;
        }
        cv::OwnedImage<std::uint8_t> img;
        try {
            img = cv::read_png(frame.path);
        } catch (const std::exception&) {
            continue;
        }
        frame_t = frame.t_s;
        est.feed_image(frame.t_s, std::as_const(img).view());
        ++processed;
    }
    os.flush();

    const double nm = n_measured ? static_cast<double>(n_measured) : 1.0;
    const double ry = sum_real_yaw / nm, rt = sum_real_tr / nm, iy = sum_inv_yaw / nm;
    std::cout << "obs_inspect: processed " << processed << " frames, " << n_updates << " updates (" << n_measured
              << " measured)\n"
              << "  consistent leak (sanity, want ~0):     yaw " << sum_cons_yaw / nm << "  trans " << sum_cons_tr / nm
              << "\n"
              << "  STANDARD leak @ filter's own clone sigma:  yaw " << ry << "  trans " << rt << "  (max yaw "
              << max_real_yaw << ")\n"
              << "  R-IEKF leak @ the SAME perturbations:      yaw " << iy << "  (max yaw " << max_inv_yaw << ")\n"
              << "  -> "
              << (ry > 4.0 * rt + 1e-12 ? "YAW dominates the standard filter; " : "no clear standard yaw dominance; ")
              << (iy < ry / 10.0 + 1e-12 ? "R-IEKF flattens it (observable by construction)"
                                         : "R-IEKF does NOT flatten it")
              << "\n  records: " << args.out << "/observability.jsonl\n";
    return 0;
}
