// SPDX-License-Identifier: MIT
//
// s6_inspect — the S6 (MSCKF measurement update) stage inspector (epic #371,
// issue #380). The real-data companion to the synthetic S6 probe
// (s6_msckf_update.cpp); establishes the filter-internal renderer tier, built on
// the S4 inspector template (s4_inspect.cpp).
//
// It runs the REAL estimator (VioEstimator + MsckfBackend) over a real EuRoC
// sequence and taps EVERY MSCKF camera update through the backend's update
// observer (#380's decouple): for each update it records the NIS / dof, the
// χ²-gate decision (accept / reject), the per-observation reprojection residual,
// and the covariance trace before / after — the local mirror of the #212
// over-confidence, measured on real data. The figures
// (docs-site/scripts/gen-update-figures.mjs) draw the NIS-over-updates curve with
// its χ² band, the gate-decision strip, and the covariance before/after heatmap.
//
// Usage:
//   s6_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N]
//              [--camera-noise σ] [--calib-rot-deg D] [--min-parallax DEG]
//              [--dump-update K]
//
//   --dataset       sequence mav0 directory (cam0/, imu0/). Required.
//   --out           output dir (default build/stage_probes/S6)
//   --max-frames    cap frames processed
//   --camera-noise  visual measurement σ (normalized) — the modeled R (default 0.01)
//   --calib-rot-deg S10 extrinsic-rotation σ folded into R (deg, default 0)
//   --min-parallax  S5 parallax gate (deg, default 0 = off)
//   --dump-update   update index whose full covariance is written (default: first accepted)
//
// Then render:
//   node docs-site/scripts/gen-update-figures.mjs <out>
//
// EuRoC (~1.5 GB) is not vendored, so this is a developer tool; the inspector
// logic is gated by tests/tools/s6_update_inspect.cpp.

#include <branes/cv/image_io.hpp>
#include <branes/math/cameras.hpp>
#include <branes/math/lie/detail.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/sfm/init_window.hpp>  // so3_from_matrix
#include <branes/sdk/vio_estimator.hpp>
#include <branes/tools/s6_update_inspect.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bs = branes::sdk;
namespace cam = branes::math::cameras;
namespace cv = branes::cv;
namespace bt = branes::tools;
namespace ms = branes::sdk::msckf;
using json = nlohmann::json;
using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T, Backend>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using Mat3 = branes::math::lie::detail::Mat<T, 3, 3>;

namespace {

struct Args {
    std::string dataset;
    std::string out = "build/stage_probes/S6";
    std::uint64_t max_frames = std::numeric_limits<std::uint64_t>::max();
    double camera_noise = 0.01;
    double calib_rot_deg = 0.0;
    double min_parallax = 0.0;
    std::int64_t dump_update = -1;  // -1 ⇒ first accepted
    bool help = false;
};

std::uint64_t to_u64(const std::string& raw) {
    if (!raw.empty() && (raw.front() == '-' || raw.front() == '+'))
        throw std::runtime_error("s6_inspect: expected a non-negative integer, got '" + raw + "'");
    std::size_t pos = 0;
    unsigned long long v = 0;
    try {
        v = std::stoull(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s6_inspect: invalid integer value '" + raw + "'");
    return v;
}
double to_f64(const std::string& raw) {
    std::size_t pos = 0;
    double v = 0;
    try {
        v = std::stod(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s6_inspect: invalid number value '" + raw + "'");
    if (!std::isfinite(v))  // std::stod accepts "nan"/"inf" — reject them for σ/threshold flags
        throw std::runtime_error("s6_inspect: non-finite number value '" + raw + "'");
    return v;
}

Args parse(int argc, char** argv) {
    Args a;
    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("s6_inspect: missing value after " + std::string(argv[i]));
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
        else if (v == "--camera-noise") {
            a.camera_noise = to_f64(next(i));
            if (!(a.camera_noise > 0.0))
                throw std::runtime_error("s6_inspect: --camera-noise must be > 0 (it is the modeled σ)");
        } else if (v == "--calib-rot-deg") {
            a.calib_rot_deg = to_f64(next(i));
            if (a.calib_rot_deg < 0.0)
                throw std::runtime_error("s6_inspect: --calib-rot-deg must be >= 0");
        } else if (v == "--min-parallax") {
            a.min_parallax = to_f64(next(i));
            if (a.min_parallax < 0.0)
                throw std::runtime_error("s6_inspect: --min-parallax must be >= 0");
        } else if (v == "--dump-update") {
            const std::uint64_t k = to_u64(next(i));
            if (k > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
                throw std::runtime_error("s6_inspect: --dump-update value out of range");
            a.dump_update = static_cast<std::int64_t>(k);
        } else
            throw std::runtime_error("s6_inspect: unknown argument '" + std::string(v) + "'");
    }
    return a;
}

void usage() {
    std::cout << "s6_inspect — S6 MSCKF-update inspector (real EuRoC run, per-update covariance/NIS/gate)\n\n"
                 "  s6_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N]\n"
                 "             [--camera-noise σ] [--calib-rot-deg D] [--min-parallax DEG] [--dump-update K]\n\n"
                 "  Then: node docs-site/scripts/gen-update-figures.mjs <out>\n";
}

// EuRoC MAV cam0 calibration (intrinsics + extrinsic), matching vio_pipeline.
Backend::CameraCalibration euroc_cam0() {
    Backend::CameraCalibration cal;
    cal.intrinsics =
        Backend::Camera(458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    Mat3 R{};
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
    cal.extrinsics.p_imu_cam = Vec3{{-0.0216401454975, -0.064676986768, 0.00981073058949}};
    return cal;
}

json dump_matrix(const ms::DynMat<T>& m) {
    json rows = json::array();
    for (std::size_t i = 0; i < m.rows; ++i) {
        json row = json::array();
        for (std::size_t j = 0; j < m.cols; ++j)
            row.push_back(m(i, j));
        rows.push_back(std::move(row));
    }
    return rows;
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
        std::cerr << "s6_inspect: --dataset <EuRoC mav0 dir> is required.\n\n";
        usage();
        return 2;
    }

    std::vector<bs::ImuMeasurement<T>> imu;
    std::vector<bs::euroc::ImageEntry> images;
    try {
        imu = bs::euroc::parse_imu<T>(args.dataset);
        images = bs::euroc::parse_images(args.dataset);
    } catch (const std::exception& ex) {
        std::cerr << "s6_inspect: " << ex.what() << "\n";
        return 1;
    }
    if (images.empty() || imu.empty()) {
        std::cerr << "s6_inspect: empty EuRoC streams (" << images.size() << " images, " << imu.size() << " IMU)\n";
        return 1;
    }

    const auto cal = euroc_cam0();

    bs::VioConfig cfg;
    cfg.camera_noise_normalized = args.camera_noise;
    cfg.calib_ext_rot_sigma_deg = args.calib_rot_deg;
    cfg.min_parallax_deg = args.min_parallax;

    // The inspector mirrors the backend's updater options so the gate threshold and
    // residual triangulation match the real path exactly (chi2_per_dof defaults to
    // the backend's; gating on).
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    ms::CameraUpdaterOptions<T> uopts;
    uopts.normalized_sigma = args.camera_noise;
    uopts.min_parallax_deg = args.min_parallax;
    uopts.calib_rot_sigma = args.calib_rot_deg * kDegToRad;
    bt::S6UpdateInspector inspector(std::vector<ms::CameraExtrinsics<T>>{cal.extrinsics}, uopts);

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    const std::string upd_path = args.out + "/updates.jsonl";
    std::ofstream upd_os(upd_path);
    if (!upd_os) {
        std::cerr << "s6_inspect: cannot write " << upd_path << "\n";
        return 1;
    }

    // ── Install the per-update observer before any frame is fed. ───────────────
    Estimator est(Backend(std::vector<Backend::CameraCalibration>{cal}));
    est.configure(cfg);
    est.activate();

    double frame_t = 0.0;
    std::uint64_t n_updates = 0, n_accepted = 0, n_gated = 0, n_psd_fail = 0;
    double sum_nis_over_dof = 0.0;
    std::size_t nis_count = 0;
    bool dumped = false;
    json cov_dump;

    est.backend().set_update_observer([&](const ms::State<T>& state_after,
                                          const ms::FeatureTrack<T>& track,
                                          const ms::NisSample<T>& nis,
                                          bool accepted,
                                          const ms::DynMat<T>& cov_before) {
        const auto rec = inspector.record(state_after, track, nis, accepted, cov_before);
        json j = bt::to_json(rec);
        j["index"] = n_updates;
        j["t"] = frame_t;
        upd_os << j.dump() << '\n';

        ++n_updates;
        if (rec.accepted)
            ++n_accepted;
        if (rec.gated)
            ++n_gated;
        if (!rec.psd_after)
            ++n_psd_fail;
        if (rec.valid && rec.dof > 0) {
            sum_nis_over_dof += rec.nis_over_dof;
            ++nis_count;
        }

        // Dump one update's full covariance for the before/after heatmap.
        const bool pick = args.dump_update >= 0 ? (static_cast<std::int64_t>(n_updates - 1) == args.dump_update)
                                                : (!dumped && rec.accepted);
        if (pick && !dumped) {
            cov_dump = json{{"update_index", n_updates - 1},
                            {"t", frame_t},
                            {"dim", cov_before.rows},
                            {"imu_dim", ms::State<T>::kImuDim},
                            {"clone_dim", ms::State<T>::kCloneDim},
                            {"accepted", rec.accepted},
                            {"before", dump_matrix(cov_before)},
                            {"after", dump_matrix(state_after.covariance())}};
            dumped = true;
        }
    });

    // ── Replay: feed IMU up to each frame, then the frame. ────────────────────
    std::uint64_t processed = 0, skipped = 0;
    std::size_t imu_idx = 0;
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
            ++skipped;
            continue;
        }
        frame_t = frame.t_s;
        est.feed_image(frame.t_s, std::as_const(img).view());
        ++processed;
    }

    // Fail fast on a truncated record stream (disk-full / late I/O error): the
    // observer's per-update writes set failbit silently, so check once at the end.
    upd_os.flush();
    if (!upd_os) {
        std::cerr << "s6_inspect: error writing " << upd_path << " (output may be truncated)\n";
        return 1;
    }
    if (dumped) {
        const std::string cov_path = args.out + "/covariance.json";
        std::ofstream cov_os(cov_path);
        cov_os << cov_dump.dump(0) << '\n';
        if (!cov_os) {
            std::cerr << "s6_inspect: error writing " << cov_path << "\n";
            return 1;
        }
    }

    const double mean_nod = nis_count ? sum_nis_over_dof / static_cast<double>(nis_count) : 0.0;
    std::cout << "s6_inspect: processed " << processed << " frames";
    if (skipped)
        std::cout << " (" << skipped << " unreadable, skipped)";
    std::cout << "\n  updates:  " << n_updates << " (" << n_accepted << " accepted, " << n_gated << " χ²-gated)\n"
              << "  NIS/dof:  mean " << mean_nod << " over " << nis_count << " valid updates"
              << (mean_nod > 1.2 ? "  → over-confident (R under-modeled)" : "") << "\n";
    if (n_psd_fail)
        std::cout << "  WARNING:  " << n_psd_fail << " updates left P non-PSD\n";
    std::cout << "  records:  " << upd_path << "\n";
    if (dumped)
        std::cout << "  cov:      " << args.out << "/covariance.json   (before/after heatmap)\n";
    std::cout << "  render:   node docs-site/scripts/gen-update-figures.mjs " << args.out << "\n";
    return 0;
}
