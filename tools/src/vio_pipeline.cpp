// SPDX-License-Identifier: MIT
//
// vio_pipeline — the end-to-end noise→robustness demo. The whole VIO premise is
// additive noise: hand the filter noisy camera + IMU streams and it must still
// deliver a robust metric estimate. This tool runs a synthetic world with EXACT
// ground truth through the real MSCKF backend, adds a controllable amount of
// additive noise to the two sensor streams, and measures how well the filter
// cleans it up — trajectory error (ATE) and consistency (NIS) versus the noise
// level — writing both the saveable per-frame stream (JSONL) and CSVs the figure
// generator renders.
//
//   ./vio_pipeline --out DIR              # one run at matched noise, write streams
//   ./vio_pipeline --noise 3 --out DIR    # 3× the assumed sensor noise
//   ./vio_pipeline --sweep --out DIR      # sweep the noise level → robustness curve
//   ./vio_pipeline --robot drone          # aggressive motion preset
//
// Source: synthetic (default, exact GT). EuRoC replay is a planned second source.

#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf_backend.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
using T = double;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using branes::sdk::FrontendObservation;
using branes::sdk::ImuMeasurement;
using branes::sdk::MsckfBackend;
using branes::sdk::VioConfig;

struct Args {
    std::string out;
    double noise = 1.0;
    bool sweep = false;
    std::string robot = "default";
    bool help = false;
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string_view v = argv[i];
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--sweep")
            a.sweep = true;
        else if (v == "--out" && i + 1 < argc && std::string_view(argv[i + 1]).substr(0, 2) != "--")
            a.out = argv[++i];
        else if (v == "--noise" && i + 1 < argc)
            a.noise = std::stod(argv[++i]);
        else if (v == "--robot" && i + 1 < argc)
            a.robot = argv[++i];
    }
    return a;
}

ev::SyntheticConfig<T> robot_preset(const std::string& r) {
    ev::SyntheticConfig<T> c;
    if (r == "ground") {  // slow, gentle — a ground robot
        c.trans_amp = 0.5;
        c.motion_rate = 0.6;
        c.yaw_amp = 0.3;
    } else if (r == "drone") {  // aggressive — a drone
        c.trans_amp = 1.2;
        c.motion_rate = 2.2;
        c.yaw_amp = 0.9;
    }
    return c;
}

double norm3(const Vec3& a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

// Result of one run at a given noise scale.
struct RunResult {
    double ate_rms_m = 0;       ///< RMS position error vs ground truth
    double final_err_m = 0;     ///< final-frame position error
    double nis_normalized = 0;  ///< (Σ NIS)/(Σ dof): 1 ≈ consistent
    std::size_t frames = 0;
    double mean_features = 0;
};

// Run the synthetic world through the backend at noise scale `ns`. When
// `traj`/`stream` are open, write the per-frame trajectory CSV and JSONL stream.
RunResult run(const ev::SyntheticData<T>& w,
              const VioConfig& cfg,
              double ns,
              std::uint64_t seed,
              std::ofstream* traj,
              std::ofstream* stream) {
    using Cal = MsckfBackend<T>::CameraCalibration;
    Cal cal;
    cal.intrinsics = w.camera;
    cal.extrinsics.R_imu_cam = w.R_imu_cam;
    cal.extrinsics.p_imu_cam = w.p_imu_cam;
    MsckfBackend<T> backend(std::vector<Cal>{cal});
    backend.initialize(cfg);

    // Additive-noise std devs matched to the filter's assumed densities × scale.
    const double imu_dt = 1.0 / 200.0;
    const double sg = cfg.gyro_noise_density / std::sqrt(imu_dt) * ns;
    const double sa = cfg.accel_noise_density / std::sqrt(imu_dt) * ns;
    const double spx = cfg.camera_noise_normalized * 458.0 * ns;  // normalized σ → pixels (≈fx)
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> N01(0.0, 1.0);

    RunResult r;
    std::size_t imu_idx = 0;
    double sq_sum = 0;
    std::size_t feat_total = 0;
    auto qstr = [](const branes::math::lie::SO3<T>& R) {
        const auto& q = R.quaternion();
        std::ostringstream o;
        o << '[' << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3] << ']';
        return o.str();
    };
    auto v3 = [](const Vec3& p) {
        std::ostringstream o;
        o << '[' << p[0] << ',' << p[1] << ',' << p[2] << ']';
        return o.str();
    };

    for (std::size_t f = 0; f < w.frames.size(); ++f) {
        const double t = w.frames[f].t;
        // Feed all IMU up to this frame, with additive noise.
        for (; imu_idx < w.imu.size() && w.imu[imu_idx].timestamp_s <= t; ++imu_idx) {
            ImuMeasurement<T> m = w.imu[imu_idx];  // angular_velocity / linear_acceleration are std::array
            for (int c = 0; c < 3; ++c) {
                m.angular_velocity[c] += sg * N01(rng);
                m.linear_acceleration[c] += sa * N01(rng);
            }
            backend.process_imu(m);
        }
        // Feed the frame's observations with additive pixel noise.
        std::vector<FrontendObservation<T>> obs = w.frames[f].obs;
        for (auto& o : obs) {
            o.u += spx * N01(rng);
            o.v += spx * N01(rng);
        }
        backend.process_camera(t, std::span<const FrontendObservation<T>>{obs});
        feat_total += obs.size();

        if (!backend.initialized())
            continue;
        const auto est = backend.current_state();
        const Vec3 ep = est.T_world_imu.translation();
        const Vec3& gp = w.gt[f].p;
        const double err = norm3(Vec3{{ep[0] - gp[0], ep[1] - gp[1], ep[2] - gp[2]}});
        sq_sum += err * err;
        r.final_err_m = err;
        ++r.frames;

        if (traj && traj->is_open())
            *traj << t << ',' << gp[0] << ',' << gp[1] << ',' << gp[2] << ',' << ep[0] << ',' << ep[1] << ',' << ep[2]
                  << ',' << err << ',' << obs.size() << '\n';
        if (stream && stream->is_open()) {
            *stream << "{\"type\":\"gt\",\"t\":" << t << ",\"q\":" << qstr(w.gt[f].R) << ",\"p\":" << v3(gp) << "}\n";
            *stream << "{\"type\":\"est\",\"t\":" << t << ",\"q\":" << qstr(est.T_world_imu.rotation())
                    << ",\"p\":" << v3(ep) << ",\"pos_err\":" << err << "}\n";
        }
    }
    r.ate_rms_m = r.frames ? std::sqrt(sq_sum / static_cast<double>(r.frames)) : 0;
    r.nis_normalized = backend.nis_consistency().report().normalized;
    r.mean_features = w.frames.empty() ? 0 : static_cast<double>(feat_total) / static_cast<double>(w.frames.size());
    r.frames = r.frames;
    return r;
}

}  // namespace

int main(int argc, char** argv) {
    const Args args = parse(argc, argv);
    if (args.help) {
        std::cout << "vio_pipeline — end-to-end noise->robustness demo (synthetic GT)\n"
                     "  --out DIR     write JSONL stream + CSVs\n"
                     "  --noise S     additive sensor-noise scale (1 = matches filter)\n"
                     "  --sweep       sweep the noise level -> robustness curve\n"
                     "  --robot R     ground | drone | default (motion aggressiveness)\n";
        return 0;
    }

    const ev::SyntheticConfig<T> scfg = robot_preset(args.robot);
    const ev::SyntheticData<T> world = ev::generate_world<T>(scfg);
    VioConfig cfg;  // the filter's assumed densities (the noise scale=1 baseline)

    std::cout << "synthetic world: " << world.imu.size() << " IMU, " << world.frames.size()
              << " frames, robot=" << args.robot << "\n";

    auto open = [&](const std::string& name) {
        std::ofstream f;
        if (!args.out.empty())
            f.open(args.out + "/" + name);
        return f;
    };

    if (args.sweep) {
        auto sweep = open("noise_sweep.csv");
        if (sweep.is_open())
            sweep << "noise_scale,ate_rms_m,final_err_m,nis_normalized,mean_features\n";
        std::cout << "\n  noise   ATE(m)   final(m)   NIS   features\n";
        std::cout << "  ------------------------------------------------\n";
        for (const double ns : {0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0}) {
            const RunResult r = run(world, cfg, ns, 0xC0FFEE, nullptr, nullptr);
            std::cout << "  " << std::left << std::setw(7) << ns << std::setw(9) << std::setprecision(3) << r.ate_rms_m
                      << std::setw(11) << r.final_err_m << std::setw(7) << r.nis_normalized << r.mean_features << "\n";
            if (sweep.is_open())
                sweep << ns << ',' << r.ate_rms_m << ',' << r.final_err_m << ',' << r.nis_normalized << ','
                      << r.mean_features << '\n';
        }
    } else {
        auto traj = open("trajectory.csv");
        if (traj.is_open())
            traj << "t,gt_x,gt_y,gt_z,est_x,est_y,est_z,pos_err,n_feat\n";
        auto stream = open("run.jsonl");
        const RunResult r = run(world, cfg, args.noise, 0xC0FFEE, &traj, &stream);
        std::cout << "\n  noise scale     : " << args.noise << "\n"
                  << "  frames tracked  : " << r.frames << "\n"
                  << "  mean features   : " << r.mean_features << "\n"
                  << "  ATE (RMS pos)   : " << r.ate_rms_m << " m\n"
                  << "  final pos error : " << r.final_err_m << " m\n"
                  << "  NIS (normalized): " << r.nis_normalized << "  (1 = consistent)\n";
    }
    if (!args.out.empty())
        std::cout << "\n  artifacts in " << args.out
                  << "/  — render: node docs-site/scripts/gen-sensor-model-figures.mjs " << args.out
                  << " docs/assessments/figures/pipeline\n";
    return 0;
}
