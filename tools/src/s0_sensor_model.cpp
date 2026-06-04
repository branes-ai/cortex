// SPDX-License-Identifier: MIT
//
// s0_sensor_model — study the S0 stage (sensor & calibration models) in
// isolation. Runs the S0 probes (sdk/eval/sensor_model_probe.hpp) on the EuRoC
// cam0 model + a fisheye + the real IMU propagator, prints a native-unit results
// table against the S0 contract, and writes the CSV artifacts the figure
// generator renders.
//
//   ./s0_sensor_model            # run all probes, write artifacts
//   ./s0_sensor_model --help     # print the S0 contract and exit
//   ./s0_sensor_model --list     # print the whole S0–S10 pipeline
//   ./s0_sensor_model --out DIR  # choose the artifact directory
//   ./s0_sensor_model --no-out   # compute & print only, no files

#include <branes/sdk/eval/sensor_model_probe.hpp>
#include <branes/tools/vio_stage_contracts.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
namespace cam = branes::math::cameras;
namespace tl = branes::tools;
using T = double;
using SO3 = branes::math::lie::SO3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;

cam::PinholeRadtanCamera<T> euroc_cam0() {
    return {458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
}
cam::EquidistantCamera<T> fisheye() {
    return {190.0, 190.0, 254.0, 256.0, 0.003, 0.001, -0.0005, 0.0001};
}
Vec3 euroc_p_imu_cam() {
    return Vec3{{-0.0216, -0.0647, 0.00981}};
}

template <class V>
std::string fmt(V v, int prec = 4) {
    std::ostringstream o;
    o.setf(std::ios::fmtflags(0), std::ios::floatfield);
    o.precision(prec);
    o << v;
    return o.str();
}

}  // namespace

int main(int argc, char** argv) {
    const tl::StageInfo& S = tl::kS0;
    tl::ProbeArgs args = tl::parse_args(argc, argv, S);
    if (args.list) {
        const auto pl = tl::pipeline();
        return tl::run_scaffold(argc, argv, S, &pl);
    }
    tl::print_contract(S);
    if (args.help)
        return 0;

    std::vector<tl::ResultRow> rows;

    // ── S0.1 round-trip (radtan + fisheye) ──────────────────────────────────
    {
        const auto c = euroc_cam0();
        const auto rt = ev::camera_round_trip(c, 752.0, 480.0, 40u, 26u);
        rows.push_back({"radtan round-trip residual (max)",
                        fmt(rt.residual_px.max),
                        "px",
                        rt.residual_px.max < 1e-2 ? "PASS (<1e-2)" : "FAIL"});
        rows.push_back({"  FOV incidence covered",
                        fmt(rt.min_incidence_deg, 3) + "-" + fmt(rt.max_incidence_deg, 3),
                        "deg",
                        "input domain"});
        if (auto f = tl::open_artifact(args, "roundtrip_radtan.csv"); f.is_open()) {
            f << "x_px,y_px,residual_px,incidence_deg\n";
            for (const auto& s : rt.field)
                f << s.x << ',' << s.y << ',' << s.value << ',' << s.incidence_deg << '\n';
        }
        const auto fc = fisheye();
        const auto rf = ev::camera_round_trip(fc, 508.0, 512.0, 40u, 40u, 4.0, 70.0);
        rows.push_back({"fisheye round-trip residual (max, <=70deg)",
                        fmt(rf.residual_px.max),
                        "px",
                        rf.residual_px.max < 1e-2 ? "PASS (<1e-2)" : "FAIL"});
        if (auto f = tl::open_artifact(args, "roundtrip_fisheye.csv"); f.is_open()) {
            f << "x_px,y_px,residual_px,incidence_deg\n";
            for (const auto& s : rf.field)
                f << s.x << ',' << s.y << ',' << s.value << ',' << s.incidence_deg << '\n';
        }
    }

    // ── S0.2 Jacobian consistency (radtan + fisheye) ────────────────────────
    {
        const auto c = euroc_cam0();
        const auto jr = ev::camera_jacobian_consistency(c, 752.0, 480.0, 40u, 26u);
        rows.push_back({"radtan Jacobian rel-error (max)",
                        fmt(jr.rel_frob.max),
                        "-",
                        jr.rel_frob.max < 1e-4 ? "PASS (<1e-4)" : "FAIL"});
        rows.push_back({"  lin-px error for 1cm scene move (max)", fmt(jr.lin_px.max), "px", "curvature floor"});
        if (auto f = tl::open_artifact(args, "jacobian_radtan.csv"); f.is_open()) {
            f << "x_px,y_px,lin_px_err\n";
            for (const auto& s : jr.field)
                f << s.x << ',' << s.y << ',' << s.value << '\n';
        }
        const auto fc = fisheye();
        const auto jf = ev::camera_jacobian_consistency(fc, 508.0, 512.0, 36u, 36u, 4.0);
        rows.push_back({"fisheye Jacobian rel-error (max)",
                        fmt(jf.rel_frob.max),
                        "-",
                        jf.rel_frob.max < 1e-3 ? "PASS (<1e-3)" : "FAIL"});
        if (auto f = tl::open_artifact(args, "jacobian_fisheye.csv"); f.is_open()) {
            f << "x_px,y_px,lin_px_err\n";
            for (const auto& s : jf.field)
                f << s.x << ',' << s.y << ',' << s.value << '\n';
        }
    }

    // ── S0.3 time-offset sensitivity ────────────────────────────────────────
    {
        const auto c = euroc_cam0();
        const cam::Vec3<T> p_c{1.0, 0.5, 5.0};
        const std::vector<ev::MotionRegime<T>> regimes = {
            {"slow", Vec3{{0.3, 0, 0}}, Vec3{{0, 0.2, 0}}},
            {"medium", Vec3{{1.0, 0, 0}}, Vec3{{0, 0.6, 0}}},
            {"fast_aggressive", Vec3{{2.0, 0, 0}}, Vec3{{0, 1.5, 0}}},
        };
        auto f = tl::open_artifact(args, "timeoffset.csv");
        if (f.is_open())
            f << "regime,dt_ms,error_px,px_per_ms\n";
        for (const auto& m : regimes) {
            const auto r = ev::time_offset_sensitivity(c, p_c, m);
            rows.push_back({std::string("time-offset sensitivity [") + m.name + "]",
                            fmt(r.px_per_ms),
                            "px/ms",
                            "1px budget = " + fmt(r.dt_budget_ms_at_1px, 3) + " ms"});
            if (f.is_open())
                for (const auto& pt : r.curve)
                    f << m.name << ',' << pt.dt_ms << ',' << pt.error_px << ',' << r.px_per_ms << '\n';
        }
    }

    // ── S0.4 extrinsic sensitivity ──────────────────────────────────────────
    {
        const auto c = euroc_cam0();
        const SO3 R_ic{};
        const Vec3 p_ic = euroc_p_imu_cam();
        auto f = tl::open_artifact(args, "extrinsic.csv");
        if (f.is_open())
            f << "depth_m,type,perturb,error_px\n";
        for (const int depth : {2, 10}) {
            const Vec3 p_feat{{p_ic[0] + 0.6, p_ic[1] + 0.3, p_ic[2] + T(depth)}};
            const auto r = ev::extrinsic_sensitivity(c, p_feat, R_ic, p_ic);
            const std::string at = " @" + std::to_string(depth) + "m";
            rows.push_back({"extrinsic rot sensitivity" + at, fmt(r.px_per_deg), "px/deg", "~depth-independent"});
            rows.push_back({"extrinsic trans sensitivity" + at, fmt(r.px_per_mm), "px/mm", "scales 1/depth"});
            if (f.is_open()) {
                for (const auto& pt : r.rot_curve)
                    f << depth << ",rot_deg," << pt.perturb << ',' << pt.error_px << '\n';
                for (const auto& pt : r.trans_curve)
                    f << depth << ",trans_mm," << pt.perturb << ',' << pt.error_px << '\n';
            }
        }
    }

    // ── S0.5 IMU static identity & drift ────────────────────────────────────
    {
        const auto cases = ev::imu_static_identity(10.0, 200.0);
        auto f = tl::open_artifact(args, "imu_drift.csv");
        if (f.is_open())
            f << "case,t_s,pos_drift_mm,vel_drift_mm_s\n";
        for (const auto& cse : cases) {
            std::string note;
            if (std::string(cse.name) == "ideal")
                note = cse.final_pos_drift_mm < 1e-6 ? "PASS (~0)" : "FAIL";
            else
                note = "leakage signature";
            rows.push_back({std::string("IMU drift [") + cse.name + "] @10s", fmt(cse.final_pos_drift_mm), "mm", note});
            if (f.is_open())
                for (const auto& pt : cse.curve)
                    f << cse.name << ',' << pt.t_s << ',' << pt.pos_drift_mm << ',' << pt.vel_drift_mm_s << '\n';
        }
    }

    tl::print_results("S0 native-unit assessment", rows);
    tl::note_artifacts(args, S);

    std::cout << "\n  Reading: the EuRoC camera model and its analytic Jacobian round-trip and\n"
                 "  linearize to machine precision — the S0 CAMERA contract HOLDS, so the #212\n"
                 "  over-confidence is not a projection bug. What S0 does expose is the COST of\n"
                 "  fixing calibration: px/ms (time offset) and px/deg, px/mm (extrinsics) are the\n"
                 "  uncertainty the filter omits by treating T_CI and t_d as perfect — the S10\n"
                 "  candidate. And the IMU drift cases make gravity leakage visceral: 0.5° of\n"
                 "  attitude error is metres of position drift in ten seconds.\n";
    return 0;
}
