// SPDX-License-Identifier: MIT
//
// s6_msckf_update — study the S6 stage (MSCKF measurement update) in isolation.
// Drives the SHIPPED CameraUpdater::update (sdk/eval/update_probe.hpp) on a
// well-conditioned, self-consistent mono scene and measures the consistency-
// critical contract: the projected innovation NIS vs its dof (the LOCAL mirror of
// the global #212 NEES≫dim), the left-null-space algebra (NᵀH_f≈0, NᵀN=I), and the
// Joseph-form PSD guarantee. Then sweeps measurement-noise mismatch to reproduce
// over-confidence at the innovation — the same lever S10 quantifies as "R×4".
//
//   ./s6_msckf_update            # run the probe, write artifacts
//   ./s6_msckf_update --help     # print the S6 contract and exit
//   ./s6_msckf_update --list     # print the whole S0–S10 pipeline
//   ./s6_msckf_update --out DIR  # choose the artifact directory
//   ./s6_msckf_update --no-out   # compute & print only, no files

#include <branes/sdk/eval/update_probe.hpp>
#include <branes/tools/vio_stage_contracts.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace {
namespace ev = branes::sdk::eval;
namespace tl = branes::tools;
using T = double;

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
    const tl::StageInfo& S = tl::kS6;
    tl::ProbeArgs args = tl::parse_args(argc, argv, S);
    if (args.list) {
        const auto pl = tl::pipeline();
        return tl::run_scaffold(argc, argv, S, &pl);
    }
    tl::print_contract(S);
    if (args.help)
        return 0;

    const std::size_t m = 4;  // clones per feature ⇒ dof = 2m−3 = 5
    const auto probe = ev::update_probe<T>(m, /*trials=*/4000);

    std::vector<tl::ResultRow> rows;

    // ── S6b/S6e  null-space + noise-preservation algebra (deterministic) ─────
    {
        const auto& ns = probe.nullspace;
        rows.push_back({"null-space marginalizes feature (NᵀH_f)",
                        fmt(ns.ntHf_max),
                        "-",
                        ns.ntHf_max < 1e-10 ? "PASS (feature eliminated)" : "FAIL"});
        rows.push_back({"null-space orthonormal (NᵀN−I)",
                        fmt(ns.orth_max),
                        "-",
                        ns.orth_max < 1e-10 ? "PASS (projected noise stays σ²I)" : "FAIL"});
        rows.push_back({"projected residual dimension",
                        fmt(ns.rows_out) + " (== 2m−3 = " + fmt(ns.rows_expected) + ")",
                        "rows",
                        ns.rows_out == ns.rows_expected ? "PASS" : "FAIL"});
    }

    // ── S6d  NIS consistency at matched (P, R) — the headline ────────────────
    {
        const auto& mt = probe.matched;
        rows.push_back({"NIS / dof @ matched noise",
                        fmt(mt.nis_over_dof, 4),
                        "-",
                        mt.consistent ? "PASS (NIS≈dof — update consistent)" : "OFF TARGET"});
        rows.push_back({"  χ² consistency band",
                        "[" + fmt(mt.band_lo, 4) + ", " + fmt(mt.band_hi, 4) + "]",
                        "-",
                        "target 1.0; band from " + fmt(mt.samples) + " updates"});
        rows.push_back({"  per-update dof (2m−3)", fmt(mt.dof), "-", "projected residual dimension"});
        rows.push_back({"Joseph keeps P PSD (every update)",
                        mt.joseph_pd_all ? "yes" : "no",
                        "-",
                        mt.joseph_pd_all ? "PASS (P⁺ positive-definite)" : "FAIL"});
    }

    // ── S6d  NIS vs measurement-noise mismatch (the over-confidence lever) ───
    {
        auto f = tl::open_artifact(args, "update_nis.csv");
        if (f.is_open())
            f << "noise_scale,nis_over_dof,dof,samples\n";
        for (const auto& p : probe.sweep) {
            rows.push_back({"NIS / dof @ injected noise ×" + fmt(p.noise_scale, 2),
                            fmt(p.nis_over_dof, 3),
                            "-",
                            p.noise_scale == 1.0
                                ? "matched"
                                : (p.noise_scale > 1.0 ? "under-modeled R ⇒ NIS↑" : "over-modeled R ⇒ NIS↓")});
            if (f.is_open())
                f << p.noise_scale << ',' << p.nis_over_dof << ',' << p.dof << ',' << p.samples << '\n';
        }
    }
    if (auto f = tl::open_artifact(args, "update_nullspace.csv"); f.is_open()) {
        f << "ntHf_max,orth_max,rows_out,rows_expected\n";
        f << probe.nullspace.ntHf_max << ',' << probe.nullspace.orth_max << ',' << probe.nullspace.rows_out << ','
          << probe.nullspace.rows_expected << '\n';
    }

    tl::print_results("S6 native-unit assessment", rows);
    tl::note_artifacts(args, S);

    std::cout << "\n  Reading: S6 is where the filter's consistency is won or lost. Driven in\n"
                 "  isolation on a self-consistent (P, R) scene — clone poses perturbed by a draw\n"
                 "  from the filter's OWN covariance, image noise at exactly the assumed σ — a\n"
                 "  correct update makes the projected innovation χ²(2m−3), so NIS/dof → 1. The\n"
                 "  matched reading above is the verdict on the update ALGEBRA (triangulation,\n"
                 "  Jacobians, null-space, S formation): near 1 ⇒ the algebra is consistent and the\n"
                 "  #212 over-confidence enters via the INPUTS (the noise budget P⁻/R — i.e. the S2\n"
                 "  process noise and the S10 unmodeled calibration), not the update math. The\n"
                 "  noise-mismatch sweep is the local lever: NIS/dof climbs above 1 exactly as the\n"
                 "  true measurement noise outgrows the modeled R — the innovation-level image of\n"
                 "  the empirical 'R×4'. (Not yet instrumented here: analytic-vs-numeric Jacobian\n"
                 "  and FEJ divergence — the current updater linearizes at the live poses.)\n";

    // Usable as a CI gate: the algebra must hold and the matched update must be
    // consistent.
    const auto& ns = probe.nullspace;
    const auto& mt = probe.matched;
    const bool ok = ns.ntHf_max < 1e-10 && ns.orth_max < 1e-10 && ns.rows_out == ns.rows_expected && mt.joseph_pd_all &&
                    mt.consistent;
    return ok ? 0 : 1;
}
