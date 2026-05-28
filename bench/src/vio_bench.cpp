// SPDX-License-Identifier: MIT
//
// vio_bench — run one EuRoC sequence through the VIO estimator and report
// the industry accuracy/performance metrics plus empirical (RAPL) energy:
// ATE, RPE (RMSE, % and deg/m), per-frame latency p50/p99, real-time
// factor, energy/frame, average power, and accuracy-bounded fps/W. Also
// emits a first-order per-operator profile (the Phase-2 graphs energy-model
// input). Accuracy on real data is dataset-driven, so this is a local tool,
// not a CI gate.
//
//   vio_bench <V1_01_easy/mav0> [--out report] [--ate-gate 0.5] [--max-clones 11]

#include <branes/bench/energy_backend.hpp>
#include <branes/bench/operator_profile.hpp>
#include <branes/bench/report.hpp>
#include <branes/bench/tegrastats.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/eval/latency_budget.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>
#include <branes/sdk/vio_estimator.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ev = branes::sdk::eval;
namespace bb = branes::bench;
using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T, Backend>;

double ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// Strict full-token numeric parse: reject trailing characters (std::stod
// /std::stoi otherwise silently accept "0.5foo" / "11x").
double parse_double_strict(const std::string& v) {
    std::size_t pos = 0;
    const double d = std::stod(v, &pos);
    if (pos != v.size())
        throw std::invalid_argument("trailing characters in number");
    return d;
}
int parse_int_strict(const std::string& v) {
    std::size_t pos = 0;
    const int n = std::stoi(v, &pos);
    if (pos != v.size())
        throw std::invalid_argument("trailing characters in number");
    return n;
}

}  // namespace

int main(int argc, char** argv) {
    std::string root, out_prefix, energy_kind_str = "rapl", energy_source;
    double ate_gate = 0.5;  // EuRoC V1_01_easy gate (see benchmarks/accuracy)
    int max_clones = 11;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto take = [&](std::string& dst) -> bool {
            if (i + 1 >= argc) {
                std::cerr << "vio_bench: missing value for " << a << "\n";
                return false;
            }
            dst = argv[++i];
            return true;
        };
        std::string v;
        try {
            if (a == "--out") {
                if (!take(out_prefix))
                    return 2;
            } else if (a == "--ate-gate") {
                if (!take(v))
                    return 2;
                ate_gate = parse_double_strict(v);
            } else if (a == "--max-clones") {
                if (!take(v))
                    return 2;
                max_clones = parse_int_strict(v);
            } else if (a == "--energy") {
                if (!take(energy_kind_str))
                    return 2;
            } else if (a == "--energy-source") {
                if (!take(energy_source))
                    return 2;
            } else if (a.rfind("--", 0) == 0) {
                std::cerr << "vio_bench: unknown flag " << a << "\n";
                return 2;
            } else if (root.empty()) {
                root = a;
            } else {
                std::cerr << "vio_bench: unexpected argument " << a << "\n";
                return 2;
            }
        } catch (const std::exception&) {
            std::cerr << "vio_bench: invalid value for " << a << "\n";
            return 2;
        }
    }
    if (root.empty()) {
        std::cerr << "usage: vio_bench <mav0-dir> [--out prefix] [--ate-gate X] [--max-clones N]\n"
                  << "                 [--energy rapl|tegrastats|external] [--energy-source <path|interval-ms>]\n";
        return 2;
    }
    bb::EnergyKind energy_kind{};
    if (!bb::energy_kind_from_string(energy_kind_str, energy_kind)) {
        std::cerr << "vio_bench: unknown --energy '" << energy_kind_str << "' (rapl|tegrastats|external)\n";
        return 2;
    }
    if (energy_kind == bb::EnergyKind::External && energy_source.empty()) {
        std::cerr << "vio_bench: --energy external requires --energy-source <cumulative-uJ counter file>\n";
        return 2;
    }

    // EuRoC cam0 (pinhole-radtan) intrinsics from the published calibration.
    Backend::CameraCalibration cal;
    cal.intrinsics =
        Backend::Camera(458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    Estimator est(Backend(std::vector<Backend::CameraCalibration>{cal}));
    bs::VioConfig cfg;
    cfg.max_clones = max_clones;

    std::vector<bs::ImuMeasurement<T>> imu;
    std::vector<bs::euroc::ImageEntry> images;
    try {
        imu = bs::euroc::parse_imu<T>(root);
        images = bs::euroc::parse_images(root);
    } catch (const std::exception& e) {
        std::cerr << "vio_bench: failed to load dataset at " << root << ": " << e.what() << "\n";
        return 1;
    }
    if (images.size() < 2) {
        std::cerr << "vio_bench: too few frames in " << root << "\n";
        return 1;
    }

    est.configure(cfg);
    est.activate();

    // ── Run the sequence, timing each frame and accumulating counts ──────
    std::vector<ev::StampedPose<T>> traj;
    traj.reserve(images.size());
    std::vector<double> latencies_ms;
    double feat_sum = 0.0, dim_sum = 0.0;
    std::size_t width = 0, height = 0;
    std::size_t imu_idx = 0;

    // Start the selected energy backend before the run (counter semantics).
    std::unique_ptr<bb::EnergyBackend> meter;
    switch (energy_kind) {
    case bb::EnergyKind::Rapl:
        meter = std::make_unique<bb::RaplBackend>();
        break;
    case bb::EnergyKind::Tegrastats: {
        int interval_ms = 100;
        if (!energy_source.empty()) {
            try {
                interval_ms = std::max(1, parse_int_strict(energy_source));
            } catch (const std::exception&) {
                std::cerr << "vio_bench: --energy-source must be an integer interval (ms) for tegrastats\n";
                return 2;
            }
        }
        meter = std::make_unique<bb::tegrastats::TegrastatsBackend>(interval_ms);
        break;
    }
    case bb::EnergyKind::External:
        meter = std::make_unique<bb::CounterFileBackend>(energy_source);
        break;
    }
    const auto wall0 = std::chrono::steady_clock::now();

    for (const auto& frame : images) {
        std::size_t end = imu_idx;
        while (end < imu.size() && imu[end].timestamp_s <= frame.t_s)
            ++end;
        if (end > imu_idx) {
            est.feed_imu(std::span<const bs::ImuMeasurement<T>>{imu.data() + imu_idx, end - imu_idx});
            imu_idx = end;
        }
        branes::cv::OwnedImage<std::uint8_t> img;
        try {
            img = branes::cv::read_png(frame.path);
        } catch (const std::exception&) {
            continue;  // skip an unreadable frame
        }
        if (img.view().empty())
            continue;
        if (width == 0) {
            width = img.view().width();
            height = img.view().height();
        }
        const auto t0 = std::chrono::steady_clock::now();
        est.feed_image(frame.t_s, img.view());
        latencies_ms.push_back(ms_since(t0));

        feat_sum += static_cast<double>(est.num_tracked_features());
        dim_sum += static_cast<double>(est.backend().state().dim());
        traj.push_back(ev::StampedPose<T>{frame.t_s, est.current_pose()});
    }

    const double processing_s = ms_since(wall0) / 1000.0;
    const double energy_j = meter->joules();
    const bool rapl_ok = meter->available();

    // ── Metrics ──────────────────────────────────────────────────────────
    bb::BenchReport r;
    r.sequence = root;
    const std::size_t nframes = traj.size();
    const double seq_duration = images.back().t_s - images.front().t_s;

    // Ground truth is optional: a sequence without it (or a malformed file)
    // still yields the performance/energy report — accuracy is left at 0.
    std::vector<ev::StampedPose<T>> gt;
    try {
        gt = bs::euroc::parse_groundtruth<T>(root);
    } catch (const std::exception& e) {
        std::cerr << "vio_bench: warning — no usable ground truth (" << e.what() << "); accuracy left at 0\n";
    }
    const auto matched = ev::associate(traj, gt, 0.01);
    if (matched.estimated.size() >= 2) {
        r.ate_m = ev::ate_rmse(matched.estimated, matched.reference);
        const std::size_t delta = std::max<std::size_t>(1, matched.estimated.size() / 20);
        r.rpe_rmse_m = ev::rpe_translation_rmse(matched.estimated, matched.reference, delta);
        const auto drift = ev::rpe_drift(matched.estimated, matched.reference, delta);
        r.rpe_translation_pct = drift.translation_pct;
        r.rpe_rotation_deg_per_m = drift.rotation_deg_per_m;
    } else {
        std::cerr << "vio_bench: warning — too few ground-truth matches; accuracy left at 0\n";
    }
    r.ate_gate_m = ate_gate;

    r.sequence_duration_s = seq_duration;
    r.processing_s = processing_s;
    r.real_time_factor = processing_s > 0.0 ? seq_duration / processing_s : 0.0;
    r.fps = processing_s > 0.0 ? static_cast<double>(nframes) / processing_s : 0.0;
    if (!latencies_ms.empty()) {
        r.latency_p50_ms = ev::percentile(latencies_ms, 0.5);
        r.latency_p99_ms = ev::percentile(latencies_ms, 0.99);
    }

    r.energy_backend = meter->name();
    r.rapl_available = rapl_ok;
    if (rapl_ok && nframes > 0) {
        r.energy_j = energy_j;
        r.energy_per_frame_mj = energy_j / static_cast<double>(nframes) * 1000.0;
        r.avg_power_w = processing_s > 0.0 ? energy_j / processing_s : 0.0;
    }

    r.accuracy_within_gate = matched.estimated.size() >= 2 && r.ate_m <= ate_gate;
    if (rapl_ok && r.accuracy_within_gate && r.avg_power_w > 0.0)
        r.fps_per_watt = r.fps / r.avg_power_w;

    // ── Counts + operator profile (Phase-2 graphs input) ─────────────────
    r.counts.frames = nframes;
    r.counts.width = width;
    r.counts.height = height;
    r.counts.pyramid_levels = 3;
    r.counts.avg_tracked_features = nframes ? feat_sum / static_cast<double>(nframes) : 0.0;
    r.counts.avg_state_dim = nframes ? dim_sum / static_cast<double>(nframes) : 15.0;
    r.counts.imu_samples = imu_idx;
    // First-order: one camera update per frame; null-space leaves ~2·features rows.
    r.counts.camera_updates = nframes;
    r.counts.avg_update_rows = 2.0 * r.counts.avg_tracked_features;
    r.profile = bb::build_pipeline_profile(r.counts);

    // ── Output ───────────────────────────────────────────────────────────
    bb::to_markdown(std::cout, r);
    if (!out_prefix.empty()) {
        bool ok = true;
        const std::string jpath = out_prefix + ".json", mpath = out_prefix + ".md";
        {
            std::ofstream js(jpath);
            bb::to_json(js, r);
            if (!js) {
                std::cerr << "vio_bench: failed to write " << jpath << "\n";
                ok = false;
            }
        }
        {
            std::ofstream md(mpath);
            bb::to_markdown(md, r);
            if (!md) {
                std::cerr << "vio_bench: failed to write " << mpath << "\n";
                ok = false;
            }
        }
        if (ok)
            std::cerr << "vio_bench: wrote " << jpath << " and " << mpath << "\n";
        else
            return 1;
    }
    return 0;
}
