// SPDX-License-Identifier: MIT
//
// vio_trajectory_report — trajectory-level consistency of the MSCKF on one
// EuRoC sequence (issue #447, epic #444 §C).
//
// The stage benches check one transformation; the composition bench checks the
// loop on a few cycles. This report checks a whole run against ground truth:
//
//   • accuracy:     ATE (Horn-aligned) and RPE (translation RMSE over 1 s segments);
//   • NEES:         the 15-dof nav error against the filter's covariance, the
//                   unobservable gauge (position + yaw) anchored at the first
//                   post-init frame (as tests/sdk/vio_euroc.cpp) — overall,
//                   per block, and in windows;
//   • NIS:          every camera update, overall and in windows;
//   • observability: per applied update, the share of the information it adds
//                   that lies along the four unobservable directions,
//                   ‖H₀N̂‖²/‖H₀‖² at its linearization point (the composition
//                   bench's unobservable_leak). A consistent update adds none;
//   • covariance growth: position σ at the start and end (global position is
//                   unobservable, so it must grow), and how often the position
//                   error exceeds 3σ.
//
// The S6 boundaries come from the stage tap (#446); everything else from the
// estimator's public state. Writes report.json and windows.csv to --out.
//
//   ./vio_trajectory_report --dataset .../V1_01_easy/mav0 --label V1_01 --out DIR [--window 10]

#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/eval/consistency.hpp>
#include <branes/sdk/eval/nav_consistency.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/vio_estimator.hpp>
#include <branes/tools/bench/c_filter_loop_bench.hpp>
#include <branes/tools/euroc_cam0.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ev = branes::sdk::eval;
namespace msckf = branes::sdk::msckf;
using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T, Backend>;
using SE3 = branes::math::lie::SE3<T>;
using json = nlohmann::json;

struct Args {
    std::string dataset, label = "sequence", out;
    double window_s = 10.0;
    bool help = false;
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string_view v = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc)
                throw std::invalid_argument(std::string(v) + " needs a value");
            return argv[++i];
        };
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--dataset")
            a.dataset = next();
        else if (v == "--label")
            a.label = next();
        else if (v == "--out")
            a.out = next();
        else if (v == "--window") {
            a.window_s = std::stod(next());
            if (!(a.window_s > 0.0))
                throw std::invalid_argument("--window must be > 0 s");
        } else
            throw std::invalid_argument("unknown flag " + std::string(v));
    }
    return a;
}

/// Per S6 update: NIS, and the share of its information along the unobservable directions.
class UpdateTap final : public msckf::StageTap<T, msckf::FullCovariance<T>> {
public:
    struct Sample {
        double t = 0.0;
        double nis = 0.0;
        std::size_t dof = 0;
        bool applied = false;
        double leak = 0.0;  ///< ‖H₀N̂‖² / ‖H₀‖² (applied updates)
    };
    std::vector<Sample> samples;

    [[nodiscard]] bool wants(msckf::TapStage s) const override {
        return s == msckf::TapStage::S6_msckf_update;
    }
    void on_s6(const St& before,
               const msckf::CameraUpdater<T>& upd,
               std::uint64_t,
               const msckf::FeatureTrack<T>& track,
               const msckf::stages::s6_msckf_update::Diagnostics<T>& d,
               const St& after) override {
        if (!d.nis.valid)
            return;
        Sample s;
        s.t = after.timestamp;
        s.nis = d.nis.value;
        s.dof = d.nis.dof;
        s.applied = d.accepted();
        if (s.applied) {
            // H₀ at the update's linearization point: the shipped S5 → S6b on the state before.
            namespace st = msckf::stages;
            const auto tri = st::s5_triangulation::apply(before, upd, track);
            const auto jac = st::s6a_jacobians::apply(before, upd, track, tri.p_f);
            const auto proj = st::s6b_nullspace_projection::apply(jac.system);
            if (tri.ok && jac.ok && proj.ok)
                s.leak = branes::tools::bench::CFilterLoopBench::unobservable_leak(before, proj.projected);
        }
        samples.push_back(s);
    }
};

struct Window {
    double t0 = 0.0;
    ev::ConsistencyAccumulator nees, nis;
    double leak_sum = 0.0;
    std::size_t leak_n = 0, frames = 0, beyond_3sigma = 0;
};

double normalized(const ev::ConsistencyAccumulator& a) {
    return a.samples() > 0 ? a.report().normalized : std::nan("");
}

json band(const ev::ConsistencyAccumulator& a) {
    if (a.samples() == 0)
        return json{{"samples", 0}};
    const auto r = a.report();
    return json{{"samples", r.samples},
                {"normalized", r.normalized},
                {"lower", r.lower},
                {"upper", r.upper},
                {"verdict", r.consistent() ? "consistent" : (r.overconfident ? "over-confident" : "under-confident")}};
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        args = parse(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "vio_trajectory_report: " << e.what() << "\n";
        return 2;
    }
    if (args.help || args.dataset.empty()) {
        std::cout << "usage: vio_trajectory_report --dataset <EuRoC mav0> [--label NAME] [--out DIR] [--window S]\n";
        return args.help ? 0 : 2;
    }

    const auto cam0 = branes::tools::euroc_cam0<T>();
    Backend::CameraCalibration cal;
    cal.intrinsics = cam0.intrinsics;
    cal.extrinsics.R_imu_cam = cam0.R_imu_cam;
    cal.extrinsics.p_imu_cam = cam0.p_imu_cam;
    Estimator est(Backend(std::vector<Backend::CameraCalibration>{cal}));
    UpdateTap tap;
    est.backend().set_stage_tap(&tap);

    std::vector<bs::euroc::GroundTruthState<T>> gt;
    std::vector<ev::StampedPose<T>> gt_poses;
    try {
        gt = bs::euroc::parse_groundtruth_states<T>(args.dataset);
        gt_poses = bs::euroc::parse_groundtruth<T>(args.dataset);
    } catch (const std::exception& e) {
        std::cerr << "vio_trajectory_report: " << e.what() << "\n";
        return 1;
    }

    // Per frame: the gauge-anchored nav error against its covariance.
    ev::ConsistencyAccumulator nees_all;
    std::array<ev::ConsistencyAccumulator, ev::kNumNavBlocks> nees_block;
    std::vector<Window> windows;
    std::vector<ev::StampedPose<T>> traj;
    std::size_t gi = 0, frames = 0, beyond = 0, not_pd = 0;
    bool anchored = false;
    SE3 anchor;
    double sigma_first = std::nan(""), sigma_last = std::nan(""), t_first = 0.0;
    auto window_at = [&](double t) -> Window& {
        if (windows.empty() || t >= windows.back().t0 + args.window_s) {
            Window w;
            w.t0 = windows.empty()
                       ? t
                       : windows.back().t0 + args.window_s * std::floor((t - windows.back().t0) / args.window_s);
            windows.push_back(w);
        }
        return windows.back();
    };
    const auto on_frame = [&](double t, const Estimator& e) {
        if (!e.backend().initialized())
            return;
        ev::StampedPose<T> sp;
        sp.t_s = t;
        sp.pose = e.current_pose();
        traj.push_back(sp);
        if (gt.empty())
            return;
        while (gi + 1 < gt.size() && std::abs(gt[gi + 1].t_s - t) <= std::abs(gt[gi].t_s - t))
            ++gi;
        if (std::abs(gt[gi].t_s - t) > 0.01)
            return;
        const auto& st = e.backend().state();
        if (!anchored) {
            anchor = ev::gauge_align<T>(SE3(st.R, st.p), SE3(gt[gi].nav.R, gt[gi].nav.p));
            anchored = true;
        }
        const auto truth = ev::align_truth<T>(anchor, gt[gi].nav);
        const auto err = ev::nav_error<T>(ev::NavSample<T>{st.R, st.p, st.v, st.bg, st.ba}, truth);
        const auto P = st.covariance();
        const double var_p = P(msckf::State<T>::kPos, msckf::State<T>::kPos) +
                             P(msckf::State<T>::kPos + 1, msckf::State<T>::kPos + 1) +
                             P(msckf::State<T>::kPos + 2, msckf::State<T>::kPos + 2);
        const double sigma = std::sqrt(var_p);
        const double ep = std::sqrt(err[3] * err[3] + err[4] * err[4] + err[5] * err[5]);
        if (std::isnan(sigma_first)) {
            sigma_first = sigma;
            t_first = t;
        }
        sigma_last = sigma;
        auto& w = window_at(t);
        ++frames;
        ++w.frames;
        if (ep > 3.0 * sigma) {
            ++beyond;
            ++w.beyond_3sigma;
        }
        try {
            const auto core = ev::core_covariance<T>(P);
            const double n = ev::nees<T>(err, core);
            nees_all.add(n, ev::kNavErrorDim);
            w.nees.add(n, ev::kNavErrorDim);
            const auto blk = ev::nav_block_nees<T>(err, core);
            for (std::size_t b = 0; b < ev::kNumNavBlocks; ++b)
                nees_block[b].add(blk[b], 3);
        } catch (const std::domain_error&) {
            ++not_pd;
        }
    };

    try {
        (void)bs::euroc::replay(args.dataset, est, bs::VioConfig{}, on_frame);  // poses come from on_frame
    } catch (const std::exception& e) {
        std::cerr << "vio_trajectory_report: " << e.what() << "\n";
        return 1;
    }
    if (traj.empty()) {
        std::cerr << "vio_trajectory_report: the filter never initialized\n";
        return 1;
    }

    // Updates: NIS, and the share of their information along the unobservable directions.
    ev::ConsistencyAccumulator nis_all;
    double leak_sum = 0.0, leak_max = 0.0;
    std::size_t applied = 0, leaking = 0;
    std::vector<double> leaks;
    for (const auto& s : tap.samples) {
        if (s.t < t_first)
            continue;
        nis_all.add(s.nis, static_cast<int>(s.dof));
        auto& w = window_at(s.t);  // samples arrive in time order, after the frame's NEES
        w.nis.add(s.nis, static_cast<int>(s.dof));
        if (s.applied) {
            ++applied;
            leak_sum += s.leak;
            leak_max = std::max(leak_max, s.leak);
            leaking += s.leak > 1e-6 ? 1 : 0;
            leaks.push_back(s.leak);
            w.leak_sum += s.leak;
            ++w.leak_n;
        }
    }

    // Accuracy.
    const auto assoc = ev::associate<T>(traj, gt_poses, 0.02);
    const double ate = assoc.estimated.empty() ? std::nan("") : ev::ate_rmse<T>(assoc.estimated, assoc.reference);
    const std::size_t seg = 20;  // 1 s at 20 Hz
    double rpe = std::nan("");
    if (assoc.estimated.size() > seg)
        rpe = ev::rpe_translation_rmse<T>(assoc.estimated, assoc.reference, seg);
    double leak_median = std::nan("");
    if (!leaks.empty()) {
        std::nth_element(leaks.begin(), leaks.begin() + static_cast<std::ptrdiff_t>(leaks.size() / 2), leaks.end());
        leak_median = leaks[leaks.size() / 2];
    }
    const double leak_mean = applied ? leak_sum / static_cast<double>(applied) : std::nan("");

    const auto& init = est.backend().init_diagnostics();
    json report{{"sequence", args.label},
                {"dataset", args.dataset},
                {"init_method", std::string(to_string(init.method))},
                {"frames_tracked", traj.size()},
                {"frames_with_truth", frames},
                {"ate_m", ate},
                {"rpe_1s_m", rpe},
                {"nees", band(nees_all)},
                {"nees_frames_not_pd", not_pd},
                {"nis", band(nis_all)},
                {"updates_applied", applied},
                {"unobservable_leak_mean", leak_mean},
                {"unobservable_leak_median", leak_median},
                {"unobservable_leak_max", leak_max},
                {"updates_with_leak_above_1e-6", leaking},
                {"position_sigma_start_m", sigma_first},
                {"position_sigma_end_m", sigma_last},
                {"position_sigma_growth", sigma_last / sigma_first},
                {"position_error_beyond_3sigma_fraction", frames ? static_cast<double>(beyond) / frames : 0.0}};
    json blocks;
    for (std::size_t b = 0; b < ev::kNumNavBlocks; ++b)
        blocks[ev::nav_block_name(b)] = normalized(nees_block[b]);
    report["nees_per_block"] = blocks;

    std::cout << std::setprecision(4) << "  " << args.label << " (init " << report["init_method"].get<std::string>()
              << ", " << traj.size() << " frames)\n"
              << "    ATE " << ate << " m   RPE(1 s) " << rpe << " m\n"
              << "    NEES/dof " << normalized(nees_all) << "  [" << report["nees"].value("verdict", "-")
              << "]   per block:";
    for (std::size_t b = 0; b < ev::kNumNavBlocks; ++b)
        std::cout << ' ' << ev::nav_block_name(b) << '=' << normalized(nees_block[b]);
    std::cout << "\n    NIS/dof " << normalized(nis_all) << "  [" << report["nis"].value("verdict", "-")
              << "]   updates " << applied << "\n    unobservable leak |H0 N|^2/|H0|^2: mean " << leak_mean
              << ", median " << leak_median << ", max " << leak_max << "; above 1e-6 in " << leaking << " of "
              << applied << " updates\n"
              << "    position sigma " << sigma_first << " m -> " << sigma_last << " m (x" << sigma_last / sigma_first
              << "); error beyond 3 sigma in " << 100.0 * report["position_error_beyond_3sigma_fraction"].get<double>()
              << " % of frames\n";

    if (!args.out.empty()) {
        std::filesystem::create_directories(args.out);
        std::ofstream(std::filesystem::path(args.out) / (args.label + "_report.json")) << report.dump(2) << "\n";
        std::ofstream csv(std::filesystem::path(args.out) / (args.label + "_windows.csv"));
        csv << "t0_s,frames,nees_per_dof,nis_per_dof,unobservable_leak_mean,beyond_3sigma_fraction\n";
        for (const auto& w : windows)
            csv << w.t0 - t_first << ',' << w.frames << ',' << normalized(w.nees) << ',' << normalized(w.nis) << ','
                << (w.leak_n ? w.leak_sum / static_cast<double>(w.leak_n) : 0.0) << ','
                << (w.frames ? static_cast<double>(w.beyond_3sigma) / w.frames : 0.0) << '\n';
        std::cout << "    wrote " << args.out << "/" << args.label << "_{report.json,windows.csv}\n";
    }
    return 0;
}
