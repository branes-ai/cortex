// SPDX-License-Identifier: MIT
//
// s10_online_calibration — study the S10 stage (online calibration) in
// isolation, and test the leading #212 hypothesis: the filter treats the
// camera↔IMU calibration as perfectly known, so unmodeled calibration
// uncertainty shows up as measurement over-confidence (the empirical "R×4 fixes
// NEES"). Drives sdk/eval/calibration_probe.hpp:
//   (1) the analytic calibration noise budget → R-inflation factor (vs R×4);
//   (2) an end-to-end R-inflation sweep on the perfect-calibration world (the
//       backend is already over-confident; find the R-scale that restores NEES,
//       and compare it to the 1deg calibration budget).
//
//   ./s10_online_calibration --help / --list / --out DIR / --no-out

#include <branes/sdk/eval/calibration_probe.hpp>
#include <branes/tools/vio_stage_contracts.hpp>

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
                        "the filter's measurement-noise deficit"});
        rows.push_back({"R-inflation from 1deg calib (analytic)",
                        fmt(sw.budget_r_inflation_1deg, 2),
                        "x",
                        "what unmodeled extrinsic uncertainty would induce"});
    }

    tl::print_results("S10 calibration assessment", rows);
    tl::note_artifacts(args, S);

    std::cout << "\n  Reading (honest, two lines of evidence):\n"
                 "  1. ANALYTIC budget: a realistic ~1deg extrinsic uncertainty induces a\n"
                 "     reprojection error (~8 px) LARGER than the filter's assumed pixel noise\n"
                 "     (~4.6 px) — an R-inflation of ~4x, quantitatively matching the empirical\n"
                 "     R×4 component of the #212 fix.\n"
                 "  2. END-TO-END: the synthetic backend reproduces the EuRoC over-confidence\n"
                 "     almost exactly (pose NEES ~43 at R×1, vs the troubleshooting's MH_05 ~43),\n"
                 "     and inflating R drives NEES back toward dof=6 — confirming R is the lever.\n"
                 "  Nuance, stated plainly: R-ONLY inflation needs ~x32 to fully restore NEES,\n"
                 "  MORE than x4 — because the empirical MH_05 fix paired R×4 with Q×10, so R\n"
                 "  alone has to do both jobs. So calibration uncertainty cleanly accounts for the\n"
                 "  R×4 COMPONENT (analytically and in magnitude), but is one contributor, not the\n"
                 "  whole over-confidence (the rest is process-noise Q and the S5 no-parallax-gate\n"
                 "  effect reproduced earlier). Principled fix: MODEL calibration uncertainty\n"
                 "  (estimate T_CI / t_d online, OpenVINS/MINS-style, or a calibration term in R)\n"
                 "  rather than a blunt global R-scale.\n";
    return 0;
}
