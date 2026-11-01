// SPDX-License-Identifier: MIT
//
// s10_online_calibration — study the S10 stage (online calibration) in
// isolation, and test the leading #212 hypothesis: the filter treats the
// camera↔IMU calibration as perfectly known, so unmodeled calibration
// uncertainty shows up as measurement over-confidence (the empirical "R×4 fixes
// NEES"). Drives sdk/eval/calibration_probe.hpp:
//   (1) the analytic calibration noise budget → R-inflation factor (vs R×4);
//   (2) an end-to-end R-inflation sweep on the perfect-calibration world (find
//       the R-scale, if any, that brings NEES to dof, and compare it to the 1deg
//       calibration budget). The sensors are exact, so its NEES below dof says
//       nothing about consistency under noise (#551);
//   (3) the matched-noise control (#551): camera and IMU noise at the σ's the
//       filter assumes, the pose NEES split into roll/pitch, yaw and position,
//       and the yaw σ at the first and last frame (the yaw-leak signature).
//
//   ./s10_online_calibration --help / --list / --out DIR / --no-out

#include <branes/sdk/eval/calibration_probe.hpp>
#include <branes/tools/vio_stage_contracts.hpp>

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace {
namespace ev = branes::sdk::eval;
namespace tl = branes::tools;
using T = double;

template <class V>
std::string fmt(V v, int prec = 3) {
    std::ostringstream o;
    o.precision(prec);
    o << v;
    return o.str();
}
}  // namespace

int main(int argc, char** argv) {
    const tl::StageInfo& S = tl::kS10;
    tl::ProbeArgs args = tl::parse_args(argc, argv, S);
    if (args.list) {
        const auto pl = tl::pipeline();
        return tl::run_scaffold(argc, argv, S, &pl);
    }
    tl::print_contract(S);
    if (args.help)
        return 0;

    std::vector<tl::ResultRow> rows;

    // ── S10.1 analytic noise budget: does realistic calib uncertainty ⇒ R×4? ──
    {
        auto f = tl::open_artifact(args, "calib_budget.csv");
        if (f.is_open())
            f << "ext_rot_deg,motion,total_calib_px,assumed_px,effective_px,r_inflation\n";
        struct Motion {
            const char* name;
            T px_per_ms;
        };
        for (const Motion m : {Motion{"slow", 0.12}, Motion{"aggressive", 0.87}}) {
            for (const T deg : {0.5, 1.0, 2.0}) {
                const auto b = ev::noise_budget<T>(deg, /*trans_mm=*/10, /*t_off_ms=*/5, m.px_per_ms);
                rows.push_back(
                    {std::string("R-inflation @ ") + fmt(deg, 1) + "deg ext, " + m.name,
                     fmt(b.r_inflation_factor),
                     "x",
                     "induced " + fmt(b.total_calib_px, 2) + "px vs assumed " + fmt(b.assumed_px, 2) + "px"});
                if (f.is_open())
                    f << deg << ',' << m.name << ',' << b.total_calib_px << ',' << b.assumed_px << ',' << b.effective_px
                      << ',' << b.r_inflation_factor << '\n';
            }
        }
    }

    // ── S10.2 R-inflation sweep: the empirical fix vs the calibration budget ──
    {
        const auto sw = ev::r_inflation_sweep<T>();
        auto f = tl::open_artifact(args, "calib_r_sweep.csv");
        if (f.is_open())
            f << "r_scale,nees,ate\n";
        for (const auto& p : sw.curve) {
            rows.push_back({"pose NEES @ Rx" + std::to_string(static_cast<int>(p.r_scale)),
                            fmt(p.nees),
                            "-",
                            p.r_scale == 1 ? "Rx1 baseline (target dof=6)" : "more R => less confident"});
            if (f.is_open())
                f << p.r_scale << ',' << p.nees << ',' << p.ate << '\n';
        }
        rows.push_back({"R-scale that restores NEES≈dof (empirical)",
                        fmt(sw.r_scale_for_consistency, 2),
                        "x",
                        "0 = none needed (NEES already <= dof at Rx1)"});
        rows.push_back({"R-inflation from 1deg calib (analytic)",
                        fmt(sw.budget_r_inflation_1deg, 2),
                        "x",
                        "what unmodeled extrinsic uncertainty would induce"});
    }

    // ── S10.3 matched-noise control: is the backend consistent under noise? ──
    {
        auto f = tl::open_artifact(args, "calib_noise_control.csv");
        if (f.is_open())
            f << "seed,nees,nees_rp,nees_yaw,nees_pos,yaw_sigma_first,yaw_sigma_last,ate\n";
        for (const std::uint32_t seed : {1u, 2u, 3u}) {
            const auto r = ev::matched_noise_control<T>(seed);
            rows.push_back({"matched-noise pose NEES, seed " + std::to_string(seed),
                            fmt(r.nees),
                            "-",
                            "rp " + fmt(r.nees_rp) + " (2), yaw " + fmt(r.nees_yaw) + " (1), pos " + fmt(r.nees_pos) +
                                " (3)"});
            rows.push_back({"matched-noise yaw sigma, seed " + std::to_string(seed),
                            fmt(r.yaw_sigma_last, 2),
                            "rad",
                            "from " + fmt(r.yaw_sigma_first, 2) + " rad: unobservable, must not fall"});
            if (f.is_open())
                f << seed << ',' << r.nees << ',' << r.nees_rp << ',' << r.nees_yaw << ',' << r.nees_pos << ','
                  << r.yaw_sigma_first << ',' << r.yaw_sigma_last << ',' << r.ate << '\n';
        }
    }

    tl::print_results("S10 calibration assessment", rows);
    tl::note_artifacts(args, S);

    std::cout << "\n  Reading (honest, three lines of evidence):\n"
                 "  1. ANALYTIC budget: a realistic ~1deg extrinsic uncertainty induces a\n"
                 "     reprojection error (~8 px) LARGER than the filter's assumed pixel noise\n"
                 "     (~4.6 px) — an R-inflation of ~4x, quantitatively matching the empirical\n"
                 "     R×4 component of the #212 fix.\n"
                 "  2. EXACT-SENSOR CONTROL: with perfect calibration and exact sensors the\n"
                 "     pose NEES is below dof=6 at R×1 (ATE ~2 cm). The assumed R is\n"
                 "     pessimistic against zero noise, so this checks the world and the\n"
                 "     plumbing, not consistency. The NEES ~43 this once showed was the\n"
                 "     world's warm-up velocity step (#537), not the filter.\n"
                 "  3. MATCHED-NOISE CONTROL (#551): with noise the filter models exactly,\n"
                 "     the body-frame backend IS over-confident (pose NEES well above 6).\n"
                 "     Yaw is unobservable, yet its σ falls from the prior while the yaw\n"
                 "     error grows: the #212 yaw leak, reproduced with perfect calibration.\n"
                 "     So calibration uncertainty is not needed to explain over-confidence;\n"
                 "     the R-IEKF (#347) is consistent on the same kind of noise.\n"
                 "  Principled fix for (1): MODEL calibration uncertainty (estimate T_CI / t_d\n"
                 "  online, OpenVINS/MINS-style, or a calibration term in R) rather than a\n"
                 "  blunt global R-scale.\n";
    return 0;
}
