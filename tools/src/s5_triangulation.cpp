// SPDX-License-Identifier: MIT
//
// s5_triangulation — study the S5 stage (feature triangulation) in isolation.
// Drives the SHIPPED triangulator (CameraUpdater::triangulate) across an exact
// two-view parallax sweep (sdk/eval/triangulation_probe.hpp), prints a
// native-unit results table against the S5 contract, and writes the CSV
// artifacts the figure generator renders.
//
//   ./s5_triangulation            # run the probe, write artifacts
//   ./s5_triangulation --help     # print the S5 contract and exit
//   ./s5_triangulation --list     # print the whole S0–S10 pipeline
//   ./s5_triangulation --out DIR  # choose the artifact directory
//   ./s5_triangulation --no-out   # compute & print only, no files

#include <branes/sdk/eval/triangulation_probe.hpp>
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
    const tl::StageInfo& S = tl::kS5;
    tl::ProbeArgs args = tl::parse_args(argc, argv, S);
    if (args.list) {
        const auto pl = tl::pipeline();
        return tl::run_scaffold(argc, argv, S, &pl);
    }
    tl::print_contract(S);
    if (args.help)
        return 0;

    const T depth = 5.0;
    const auto sweep = ev::triangulation_parallax_sweep<T>(depth, /*px_noise=*/1.0, /*mc=*/400);

    std::vector<tl::ResultRow> rows;
    for (const auto& p : sweep.curve) {
        const std::string at = " @" + fmt(p.parallax_deg, 3) + "deg";
        const T sigma_pct = 100.0 * p.depth_sigma_mm / (depth * 1000.0);
        rows.push_back({"triangulate status" + at,
                        p.status_ok ? "ok" : "FAIL",
                        "-",
                        p.status_ok ? "shipped triangulate() succeeds" : "Cholesky breakdown (degenerate)"});
        rows.push_back({"  condition number" + at,
                        fmt(p.condition_number, 4),
                        "-",
                        p.condition_number > 1e4 ? "ill-conditioned" : "well-conditioned"});
        rows.push_back(
            {"  depth sigma (1px noise)" + at, fmt(p.depth_sigma_mm, 4), "mm", fmt(sigma_pct, 3) + "% of depth"});
    }

    if (auto f = tl::open_artifact(args, "triang_parallax_sweep.csv"); f.is_open()) {
        f << "parallax_deg,status_ok,condition_number,depth_error_mm,depth_sigma_mm,reproj_rms_px,mc_failures\n";
        for (const auto& p : sweep.curve)
            f << p.parallax_deg << ',' << (p.status_ok ? 1 : 0) << ',' << p.condition_number << ',' << p.depth_error_mm
              << ',' << p.depth_sigma_mm << ',' << p.reproj_rms_px << ',' << p.mc_failures << '\n';
    }
    if (auto f = tl::open_artifact(args, "triang_reproj.csv"); f.is_open()) {
        f << "parallax_deg,reproj_rms_px,depth_error_mm\n";
        for (const auto& p : sweep.curve)
            f << p.parallax_deg << ',' << p.reproj_rms_px << ',' << p.depth_error_mm << '\n';
    }

    tl::print_results("S5 native-unit assessment", rows);
    tl::note_artifacts(args, S);

    std::cout << "\n  Reading: the shipped triangulator has only a HARD guard — the Cholesky\n"
                 "  breakdown at near-parallel rays. Across the danger band the linear system is\n"
                 "  already badly conditioned and the true depth uncertainty (Monte-Carlo sigma\n"
                 "  under 1 px noise) is large, yet triangulate() still returns success and the\n"
                 "  feature is admitted to the update at full weight. The suggested SOFT parallax\n"
                 "  gate (sigma <= 5% of depth) sits at ~"
              << fmt(sweep.gate_parallax_deg, 3)
              << " deg. Admitting sub-gate features\n"
                 "  injects optimistic information — the S5 #212 candidate for the V2_03 divergence.\n";
    return 0;
}
