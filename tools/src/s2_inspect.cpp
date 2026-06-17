// SPDX-License-Identifier: MIT
//
// s2_inspect — the S2 (IMU propagation) stage inspector (epic #371, issue #377).
// The real-data companion to the synthetic S2 probe (s2_propagation.cpp); renders
// on the filter-internal tier, built on the S4 inspector template (s4_inspect.cpp).
//
// It takes a real EuRoC IMU window, seeds the prior pose+velocity from ground
// truth at the window start (plus the filter's initial-P seed), and runs the REAL
// IMU propagator (msckf::Propagator, via branes/tools/s2_propagation_inspect.hpp)
// step by step — recording the covariance GROWTH (per-block σ, full diagonal) and
// the propagated mean. It then compares the propagated pose against ground truth
// across the window: does the filter's own growing ±σ envelope actually contain
// the dead-reckoning drift? The figures (docs-site/scripts/gen-propagation-
// figures.mjs) draw the σ-growth heatmap and the drift-vs-envelope plot.
//
// No SDK change: the propagator is already standalone (propagate(state,…) → mean +
// state.covariance()), so this just drives it on a real window.
//
// Usage:
//   s2_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--start-t T] [--window-s S]
//              [--sigma0 σ] [--max-samples N]
//
//   --dataset     mav0 directory (imu0/, state_groundtruth_estimate0/). Required.
//   --out         output dir (default build/stage_probes/S2)
//   --start-t     window start (seconds, dataset clock; default: first GT sample)
//   --window-s    propagation window length (default 0.5 s — an inter-keyframe gap is ~0.05 s)
//   --sigma0      initial-P seed σ (isotropic, default 0.1)
//   --max-samples cap recorded steps (default 120)
//
// Then render:
//   node docs-site/scripts/gen-propagation-figures.mjs <out>
//
// EuRoC (~1.5 GB) is not vendored, so this is a developer tool; the inspector
// logic is gated by tests/tools/s2_propagation_inspect.cpp.

#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/vio_backend.hpp>
#include <branes/tools/s2_propagation_inspect.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bs = branes::sdk;
namespace ms = branes::sdk::msckf;
namespace ev = branes::sdk::eval;
namespace bt = branes::tools;
namespace lie = branes::math::lie;
using json = nlohmann::json;
using T = double;
using Vec3 = lie::detail::Vec<T, 3>;
using SO3 = lie::SO3<T>;

namespace {

struct Args {
    std::string dataset;
    std::string out = "build/stage_probes/S2";
    double start_t = -1.0;  // <0 ⇒ first GT sample
    double window_s = 0.5;
    double sigma0 = 0.1;
    std::size_t max_samples = 120;
    bool help = false;
};

double to_f64(const std::string& raw) {
    std::size_t pos = 0;
    double v = 0;
    try {
        v = std::stod(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s2_inspect: invalid number value '" + raw + "'");
    if (!std::isfinite(v))
        throw std::runtime_error("s2_inspect: non-finite number value '" + raw + "'");
    return v;
}
std::uint64_t to_u64(const std::string& raw) {
    if (!raw.empty() && (raw.front() == '-' || raw.front() == '+'))
        throw std::runtime_error("s2_inspect: expected a non-negative integer, got '" + raw + "'");
    std::size_t pos = 0;
    unsigned long long v = 0;
    try {
        v = std::stoull(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s2_inspect: invalid integer value '" + raw + "'");
    return v;
}

Args parse(int argc, char** argv) {
    Args a;
    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("s2_inspect: missing value after " + std::string(argv[i]));
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
        else if (v == "--start-t")
            a.start_t = to_f64(next(i));
        else if (v == "--window-s") {
            a.window_s = to_f64(next(i));
            if (!(a.window_s > 0.0))
                throw std::runtime_error("s2_inspect: --window-s must be > 0");
        } else if (v == "--sigma0") {
            a.sigma0 = to_f64(next(i));
            if (!(a.sigma0 > 0.0))
                throw std::runtime_error("s2_inspect: --sigma0 must be > 0");
        } else if (v == "--max-samples")
            a.max_samples = static_cast<std::size_t>(to_u64(next(i)));
        else
            throw std::runtime_error("s2_inspect: unknown argument '" + std::string(v) + "'");
    }
    return a;
}

void usage() {
    std::cout << "s2_inspect — S2 IMU-propagation inspector (real EuRoC IMU window → covariance growth + drift)\n\n"
                 "  s2_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--start-t T] [--window-s S]\n"
                 "             [--sigma0 σ] [--max-samples N]\n\n"
                 "  Then: node docs-site/scripts/gen-propagation-figures.mjs <out>\n";
}

const ev::NavSample<T>* nearest_nav(const std::vector<bs::euroc::GroundTruthState<T>>& gt, double t_s) {
    if (gt.empty())
        return nullptr;
    std::size_t best = 0;
    double bestd = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < gt.size(); ++i) {
        const double d = gt[i].t_s > t_s ? gt[i].t_s - t_s : t_s - gt[i].t_s;
        if (d < bestd) {
            bestd = d;
            best = i;
        }
    }
    return &gt[best].nav;
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
        std::cerr << "s2_inspect: --dataset <EuRoC mav0 dir> is required.\n\n";
        usage();
        return 2;
    }

    std::vector<bs::ImuMeasurement<T>> imu;
    std::vector<bs::euroc::GroundTruthState<T>> gt;
    try {
        imu = bs::euroc::parse_imu<T>(args.dataset);
        gt = bs::euroc::parse_groundtruth_states<T>(args.dataset);
    } catch (const std::exception& ex) {
        std::cerr << "s2_inspect: " << ex.what() << "\n";
        return 1;
    }
    if (imu.size() < 2 || gt.empty()) {
        std::cerr << "s2_inspect: need IMU + ground truth (got " << imu.size() << " IMU, " << gt.size() << " GT)\n";
        return 1;
    }

    const double start_t = args.start_t >= 0.0 ? args.start_t : gt.front().t_s;
    const double end_t = start_t + args.window_s;

    // Gather the IMU window [start_t, end_t].
    bt::S2PropagationInspector::S2Input in;
    in.max_samples = args.max_samples;
    in.noise = ms::ImuNoise<T>{};  // default densities ≈ the VioConfig defaults
    bs::VioConfig cfg;
    in.noise.gyro = cfg.gyro_noise_density;
    in.noise.accel = cfg.accel_noise_density;
    in.noise.gyro_bias = cfg.gyro_bias_random_walk;
    in.noise.accel_bias = cfg.accel_bias_random_walk;
    in.gravity = Vec3{{0.0, 0.0, -cfg.gravity_magnitude}};
    for (const auto& m : imu)
        if (m.timestamp_s >= start_t && m.timestamp_s <= end_t)
            in.imu.push_back(m);
    if (in.imu.size() < 2) {
        std::cerr << "s2_inspect: only " << in.imu.size() << " IMU samples in [" << start_t << ", " << end_t
                  << "] — widen --window-s or move --start-t\n";
        return 1;
    }

    // Seed the prior from ground truth at the window start + the isotropic P seed.
    const ev::NavSample<T>* gt0 = nearest_nav(gt, start_t);
    in.prior = ms::State<T>(args.sigma0);
    in.prior.R = gt0->R;
    in.prior.p = gt0->p;
    in.prior.v = gt0->v;
    in.prior.bg = gt0->bg;
    in.prior.ba = gt0->ba;

    const bt::S2PropagationInspector inspector;
    auto rec = inspector.run(in);

    // Attach drift vs ground truth at each recorded step.
    std::size_t within = 0, gtsteps = 0;
    for (auto& st : rec.steps) {
        const ev::NavSample<T>* g = nearest_nav(gt, start_t + st.t);
        if (!g)
            continue;
        const Vec3 p_est{{st.p[0], st.p[1], st.p[2]}};
        const SO3 R_est(SO3::Quaternion{{st.q[0], st.q[1], st.q[2], st.q[3]}});
        const auto d = bt::S2PropagationInspector::drift(p_est, R_est, g->p, g->R);
        st.have_gt = true;
        st.pos_drift_mm = d.pos_mm;
        st.att_drift_deg = d.att_deg;
        st.pos_within_3sigma = d.pos_mm <= 3.0 * st.pos_sigma_mm;
        if (st.pos_within_3sigma)
            ++within;
        ++gtsteps;
    }

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    const std::string out_path = args.out + "/propagation.json";
    std::ofstream os(out_path);
    if (!os) {
        std::cerr << "s2_inspect: cannot write " << out_path << "\n";
        return 1;
    }
    os << bt::to_json(rec).dump(1) << '\n';
    if (!os) {
        std::cerr << "s2_inspect: error writing " << out_path << "\n";
        return 1;
    }

    const auto& last = rec.steps.back();
    std::cout << "s2_inspect: window [" << start_t << ", " << end_t << "] — " << in.imu.size() << " IMU samples @ "
              << rec.rate_hz << " Hz\n"
              << "  cov growth: pos σ " << rec.steps.front().pos_sigma_mm << " → " << last.pos_sigma_mm << " mm, att σ "
              << rec.steps.front().att_sigma_deg << " → " << last.att_sigma_deg << " deg\n";
    if (gtsteps)
        std::cout << "  vs GT:      final drift " << last.pos_drift_mm << " mm / " << last.att_drift_deg << " deg; "
                  << within << "/" << gtsteps << " steps within 3σ"
                  << (within < gtsteps ? "  → envelope under-covers the drift (Q too small?)" : "") << "\n";
    std::cout << "  steps:      " << out_path << "\n"
              << "  render:     node docs-site/scripts/gen-propagation-figures.mjs " << args.out << "\n";
    return 0;
}
