// SPDX-License-Identifier: MIT
//
// s1_initialization — study the S1 stage (initialization: static & dynamic
// bootstrap) in isolation. Drives the real ImuInitializer (+ ImuPreintegrator
// for the dynamic keyframes) to measure the S1 contract in native units: the
// static gravity-leveling residual, the dynamic scale-observability cliff, and
// what the filter's isotropic initial covariance implies per state.
//
//   ./s1_initialization --help / --list / --out DIR / --no-out

#include <branes/sdk/eval/initialization_probe.hpp>
#include <branes/tools/vio_stage_contracts.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
namespace tl = branes::tools;
using T = double;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;

template <class V>
std::string fmt(V v, int prec = 4) {
    std::ostringstream o;
    o.precision(prec);
    o << v;
    return o.str();
}

}  // namespace

int main(int argc, char** argv) {
    const tl::StageInfo& S = tl::kS1;
    tl::ProbeArgs args = tl::parse_args(argc, argv, S);
    if (args.list) {
        const auto pl = tl::pipeline();
        return tl::run_scaffold(argc, argv, S, &pl);
    }
    tl::print_contract(S);
    if (args.help)
        return 0;

    std::vector<tl::ResultRow> rows;

    // ── S1.1 static init (gravity leveling) ─────────────────────────────────
    {
        const auto r = ev::static_init<T>(/*tilt_deg=*/8.0, Vec3{{0.01, -0.005, 0.008}});
        rows.push_back({"static init gate (stationary window)",
                        r.success ? "accept" : "reject",
                        "-",
                        r.success ? "PASS" : "FAIL"});
        rows.push_back(
            {"gravity-leveling residual", fmt(r.gravity_residual_ms2), "m/s2", "recovered R levels gravity (~0)"});
        rows.push_back({"recovered gravity magnitude", fmt(r.recovered_g_ms2), "m/s2", "~9.81"});
        rows.push_back(
            {"roll/pitch leveling error", fmt(r.rollpitch_error_deg), "deg", "attitude error from accel noise"});
        rows.push_back({"gyro-bias recovery error", fmt(r.gyro_bias_error), "rad/s", "static window pins gyro bias"});
        auto f = tl::open_artifact(args, "init_static_sweep.csv");
        if (f.is_open())
            f << "accel_noise_std,rollpitch_err_deg,gate_accept\n";
        for (const auto& p : ev::static_noise_sweep<T>()) {
            if (f.is_open())
                f << p.accel_noise_std << ',' << p.rollpitch_error_deg << ',' << (p.success ? 1 : 0) << '\n';
        }
    }

    // ── S1.2 dynamic init (scale observability) ─────────────────────────────
    {
        const auto r = ev::dynamic_init<T>();
        rows.push_back({"dynamic init gate (scale observable)",
                        r.success ? "accept" : "reject",
                        "-",
                        r.success ? "PASS" : "declined"});
        rows.push_back(
            {"scale recovery error (excited)", fmt(r.scale_error_pct), "%", "metric scale from VI alignment"});
        rows.push_back({"recovered vs true scale",
                        fmt(r.scale_recovered) + " / " + fmt(r.scale_true),
                        "-",
                        "vision->metric factor"});
        rows.push_back({"gravity direction error", fmt(r.gravity_dir_error_deg), "deg", "alignment recovers gravity"});
        rows.push_back({"gravity magnitude error", fmt(r.gravity_mag_error_ms2), "m/s2", "~0"});
        rows.push_back({"velocity recovery error", fmt(r.velocity_error_ms), "m/s", "per-keyframe velocities"});
        auto f = tl::open_artifact(args, "init_excitation_sweep.csv");
        if (f.is_open())
            f << "excitation_ms2,scale_err_pct,resolved_motion_m,gate_accept\n";
        for (const auto& p : ev::dynamic_excitation_sweep<T>()) {
            rows.push_back({"  excitation " + fmt(p.excitation_ms2, 2) + " m/s2",
                            p.success ? fmt(p.scale_error_pct, 3) + "% (accept)" : "declined (unobservable)",
                            "%",
                            "the observability cliff"});
            if (f.is_open())
                f << p.excitation_ms2 << ',' << p.scale_error_pct << ',' << p.resolved_motion_m << ','
                  << (p.success ? 1 : 0) << '\n';
        }
    }

    // ── S1.3 initial covariance sizing ──────────────────────────────────────
    {
        const auto p = ev::initial_p_sizing<T>();
        rows.push_back(
            {"initial-P seed (isotropic sigma)", fmt(p.sigma), "-", p.isotropic ? "ISOTROPIC seed" : "structured"});
        rows.push_back(
            {"  -> yaw sigma (unobservable dir)", fmt(p.yaw_sigma_deg), "deg", "same as roll/pitch (not larger)"});
        rows.push_back({"  -> position sigma", fmt(p.position_sigma_m), "m", "global position unobs."});
        rows.push_back({"  -> accel-bias sigma", fmt(p.accel_bias_sigma_ms2), "m/s2", "weakly observable"});
        if (auto f = tl::open_artifact(args, "init_p_sizing.csv"); f.is_open()) {
            f << "block,sigma_native,unit\n";
            f << "yaw," << p.yaw_sigma_deg << ",deg\n";
            f << "rollpitch," << p.rollpitch_sigma_deg << ",deg\n";
            f << "position," << p.position_sigma_m << ",m\n";
            f << "velocity," << p.velocity_sigma_ms << ",m/s\n";
            f << "accel_bias," << p.accel_bias_sigma_ms2 << ",m/s2\n";
        }
    }

    tl::print_results("S1 native-unit assessment", rows);
    tl::note_artifacts(args, S);

    std::cout << "\n  Reading: static init levels gravity to a small residual and the\n"
                 "  stationarity gate refuses a noisy 'static' window (see the noise sweep).\n"
                 "  Dynamic init recovers the metric scale under excitation and the\n"
                 "  observability gate DECLINES (rather than returning garbage) as excitation\n"
                 "  drops toward a hover — the dynamic-init cliff. The initial covariance is an\n"
                 "  ISOTROPIC sigma*I: it is not enlarged on the unobservable yaw/scale or the\n"
                 "  weakly-observable accel bias, which a structured seed would be.\n";
    return 0;
}
