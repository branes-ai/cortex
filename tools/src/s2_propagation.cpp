// SPDX-License-Identifier: MIT
//
// s2_propagation — study the S2 stage (IMU propagation: mean + covariance) in
// isolation. Drives the real Propagator (sdk/eval/propagation_probe.hpp) to
// measure the S2 contract in native units, with a focus on the #212 candidate:
// the cortex propagator injects a DIAGONAL Q_d and no position-block term — does
// that materially under-state the propagated covariance? The probe answers in
// millimetres of position σ, not opinion.
//
//   ./s2_propagation --help / --list / --out DIR / --no-out

#include <branes/sdk/eval/propagation_probe.hpp>
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
    const tl::StageInfo& S = tl::kS2;
    tl::ProbeArgs args = tl::parse_args(argc, argv, S);
    if (args.list) {
        const auto pl = tl::pipeline();
        return tl::run_scaffold(argc, argv, S, &pl);
    }
    tl::print_contract(S);
    if (args.help)
        return 0;

    const branes::sdk::msckf::ImuNoise<T> noise{};  // cortex defaults
    const Vec3 gyro{{0.2, -0.1, 0.3}};              // exciting body rate (rad/s)
    const Vec3 accel{{0.5, -0.3, 9.81}};            // gravity + small specific force
    std::vector<tl::ResultRow> rows;

    // ── S2.1 Q_d structure ──────────────────────────────────────────────────
    {
        const auto q = ev::q_structure(noise, 1.0 / 200.0);
        rows.push_back({"Q theta sigma (cortex vs canon)",
                        fmt(q.theta_sigma_cortex) + " / " + fmt(q.theta_sigma_canon),
                        "rad",
                        "match"});
        rows.push_back({"Q vel sigma (cortex vs canon)",
                        fmt(q.vel_sigma_cortex) + " / " + fmt(q.vel_sigma_canon),
                        "m/s",
                        "match"});
        rows.push_back({"Q pos sigma (cortex vs canon)",
                        fmt(q.pos_sigma_cortex) + " / " + fmt(q.pos_sigma_canon),
                        "m",
                        "cortex DROPS pos term"});
        rows.push_back({"Q v-p cross term (canon; cortex=0)", fmt(q.vp_cross_canon), "m2/s", "cortex DROPS cross"});
        rows.push_back({"Q rel-Frobenius gap (canon vs cortex)",
                        fmt(q.rel_frobenius_gap),
                        "-",
                        "fraction of Q the diagonal drops"});
        if (auto f = tl::open_artifact(args, "prop_q_structure.csv"); f.is_open()) {
            f << "block,sigma_cortex,sigma_canon\n";
            f << "theta," << q.theta_sigma_cortex << ',' << q.theta_sigma_canon << '\n';
            f << "vel," << q.vel_sigma_cortex << ',' << q.vel_sigma_canon << '\n';
            f << "pos," << q.pos_sigma_cortex << ',' << q.pos_sigma_canon << '\n';
        }
    }

    // ── S2.2 covariance growth (the decisive native comparison) ─────────────
    {
        const auto g = ev::cov_growth(noise, gyro, accel);
        rows.push_back({"Phi-reconstruction validation",
                        fmt(g.F_validation_residual),
                        "-",
                        g.F_validation_residual < 1e-10 ? "PASS (real==ref)" : "CHECK"});
        if (!g.curve.empty()) {
            const auto& last = g.curve.back();
            rows.push_back({"pos sigma @0.5s (cortex vs canon)",
                            fmt(last.pos_sigma_mm_cortex) + " / " + fmt(last.pos_sigma_mm_canon),
                            "mm",
                            "the over-confidence, if any"});
            rows.push_back({"pos sigma under-report @0.05s",
                            fmt(g.pos_underreport_pct_interframe),
                            "%",
                            "diagonal-Q deficit, inter-frame"});
            rows.push_back(
                {"pos sigma under-report @0.5s", fmt(g.pos_underreport_pct_final), "%", "diagonal-Q deficit, 0.5s"});
            rows.push_back({"R orthonormality residual @0.5s",
                            fmt(last.R_ortho_residual),
                            "-",
                            last.R_ortho_residual < 1e-10 ? "PASS (on SO3)" : "FAIL"});
            rows.push_back({"P min eigenvalue @0.5s",
                            fmt(last.min_eig_cortex),
                            "-",
                            last.min_eig_cortex >= -1e-12 ? "PASS (PSD)" : "FAIL"});
        }
        if (auto f = tl::open_artifact(args, "prop_growth.csv"); f.is_open()) {
            f << "t_s,pos_cortex_mm,pos_canon_mm,vel_cortex_mms,vel_canon_mms,att_cortex_deg,att_"
                 "canon_deg,min_eig,R_ortho\n";
            for (const auto& p : g.curve)
                f << p.t_s << ',' << p.pos_sigma_mm_cortex << ',' << p.pos_sigma_mm_canon << ','
                  << p.vel_sigma_mm_s_cortex << ',' << p.vel_sigma_mm_s_canon << ',' << p.att_sigma_deg_cortex << ','
                  << p.att_sigma_deg_canon << ',' << p.min_eig_cortex << ',' << p.R_ortho_residual << '\n';
        }
    }

    // ── S2.3 GT-injection (integration accuracy vs dt) ──────────────────────
    {
        const auto sweep = ev::gt_injection_dt_sweep<T>();
        auto f = tl::open_artifact(args, "prop_gt_injection.csv");
        if (f.is_open())
            f << "dt_s,pos_error_mm,att_error_deg,vel_error_mms\n";
        for (const auto& p : sweep) {
            rows.push_back({"GT-injection pos error @" + fmt(1.0 / p.dt_s, 0) + "Hz",
                            fmt(p.pos_error_mm),
                            "mm",
                            "discretization error"});
            if (f.is_open())
                f << p.dt_s << ',' << p.pos_error_mm << ',' << p.att_error_deg << ',' << p.vel_error_mm_s << '\n';
        }
        if (sweep.size() >= 2) {
            const T ratio = sweep[sweep.size() - 2].pos_error_mm / sweep.back().pos_error_mm;
            rows.push_back({"GT-injection error order (halving dt)",
                            fmt(ratio, 3),
                            "x",
                            ratio > 1.6 ? "~O(dt) integrator" : "check"});
        }
    }

    // ── S2.4 propagation-only NEES vs Q scale (#212 lever) ──────────────────
    {
        const auto n = ev::nees_vs_qscale(noise);
        auto f = tl::open_artifact(args, "prop_nees.csv");
        if (f.is_open())
            f << "q_scale,nees_pose,dof\n";
        for (const auto& p : n.curve) {
            rows.push_back(
                {"NEES (6-DoF pose) @ Q-scale " + fmt(p.q_scale, 2), fmt(p.nees_pose, 3), "-", "target = 6"});
            if (f.is_open())
                f << p.q_scale << ',' << p.nees_pose << ',' << n.dof << '\n';
        }
    }

    // ── S2.5 nullspace preservation ─────────────────────────────────────────
    {
        const auto ns = ev::nullspace_position(gyro, accel);
        rows.push_back({"global-position nullspace leak",
                        fmt(ns.position_leak),
                        "-",
                        ns.position_leak < 1e-12 ? "PASS (preserved)" : "leak"});
    }

    tl::print_results("S2 native-unit assessment", rows);
    tl::note_artifacts(args, S);

    std::cout << "\n  Reading: the Phi-reconstruction validation confirms the probe's reference\n"
                 "  matches the shipped Propagator, so the cortex-vs-canonical comparison is\n"
                 "  apples-to-apples. The cortex Q_d is diagonal and drops the canonical\n"
                 "  position-block and v-p cross terms; the 'pos sigma under-report' numbers say\n"
                 "  in MILLIMETRES how much that actually costs over a propagation window — the\n"
                 "  decisive test of #212 candidate #2. NEES vs Q-scale is the consistency lever:\n"
                 "  NEES at scale 1 above the 6-DoF target means the propagated covariance is\n"
                 "  already over-confident before any camera update.\n";
    return 0;
}
