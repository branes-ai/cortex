// SPDX-License-Identifier: MIT
//
// s4_frontend — study the S4 stage (visual frontend / track generation) in
// isolation. Drives the SHIPPED FAST detector + pyramidal KLT tracker
// (sdk/eval/frontend_probe.hpp) on synthetic textured images warped by a known
// translation, prints a native-unit results table, and writes the CSV artifacts.
//
//   ./s4_frontend            # run the probe, write artifacts
//   ./s4_frontend --help     # print the S4 contract and exit
//   ./s4_frontend --list     # print the whole S0–S10 pipeline
//   ./s4_frontend --out DIR  # choose the artifact directory
//   ./s4_frontend --no-out   # compute & print only, no files

#include <branes/sdk/eval/frontend_probe.hpp>
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
    const tl::StageInfo& S = tl::kS4;
    tl::ProbeArgs args = tl::parse_args(argc, argv, S);
    if (args.list) {
        const auto pl = tl::pipeline();
        return tl::run_scaffold(argc, argv, S, &pl);
    }
    tl::print_contract(S);
    if (args.help)
        return 0;

    const auto r = ev::frontend_probe<T>();

    std::vector<tl::ResultRow> rows;
    rows.push_back({"features detected (FAST)", fmt(r.detected), "-", "level-0 corners"});
    rows.push_back({"forward-tracked (KLT)", fmt(r.tracked) + "/" + fmt(r.detected), "-", "KLT status = Tracked"});
    rows.push_back({"forward-backward residual (median)",
                    fmt(r.fb_residual_median),
                    "px",
                    r.fb_residual_median < 0.5 ? "self-consistent" : "drift"});
    rows.push_back({"forward-backward residual (max)",
                    fmt(r.fb_residual_max),
                    "px",
                    "the un-gated outliers (no FB check shipped)"});
    rows.push_back(
        {"FB-gate inliers (<1px)", fmt(r.fb_inliers) + "/" + fmt(r.tracked), "-", "what a 1px FB gate WOULD keep"});
    rows.push_back(
        {"RANSAC inlier ratio", "n/a", "%", "RANSAC not shipped (nor is the FB gate) — no geometric gate applied"});
    rows.push_back({"endpoint RMS (all tracked)",
                    fmt(r.endpoint_rms_all_px),
                    "px",
                    "what the un-gated frontend actually feeds the backend"});
    rows.push_back({"endpoint RMS (FB-gated <1px)",
                    fmt(r.endpoint_rms_gated_px),
                    "px",
                    "what a forward-backward gate WOULD keep"});
    rows.push_back({"grid coverage occupancy",
                    fmt(r.grid_occupancy_pct, 3),
                    "%",
                    fmt(r.grid_cols) + "x" + fmt(r.grid_rows) + " grid"});
    rows.push_back({"mean track length", fmt(r.mean_track_len), "frames", "over a 12-frame translating sequence"});
    for (const auto& np : r.noise_curve)
        rows.push_back({"  endpoint RMS @ noise " + fmt(np.sigma_px, 2) + "px",
                        fmt(np.endpoint_rms),
                        "px",
                        fmt(np.survival_pct, 3) + "% survive"});

    if (auto f = tl::open_artifact(args, "frontend_fb_residual.csv"); f.is_open()) {
        f << "feature,fb_residual_px,endpoint_err_px\n";
        for (std::size_t i = 0; i < r.fb_residuals.size(); ++i)
            f << i << ',' << r.fb_residuals[i] << ',' << (i < r.endpoint_errors.size() ? r.endpoint_errors[i] : T{0})
              << '\n';
    }
    if (auto f = tl::open_artifact(args, "frontend_coverage.csv"); f.is_open()) {
        f << "grid_cols,grid_rows,occupancy_pct,detected\n";
        f << r.grid_cols << ',' << r.grid_rows << ',' << r.grid_occupancy_pct << ',' << r.detected << '\n';
    }
    if (auto f = tl::open_artifact(args, "frontend_tracklen.csv"); f.is_open()) {
        f << "track_length_frames,count\n";
        for (std::size_t l = 0; l < r.track_len_hist.size(); ++l)
            f << l << ',' << r.track_len_hist[l] << '\n';
    }
    if (auto f = tl::open_artifact(args, "frontend_noise_sweep.csv"); f.is_open()) {
        f << "sigma_px,endpoint_rms_px,fb_residual_median_px,survival_pct\n";
        for (const auto& np : r.noise_curve)
            f << np.sigma_px << ',' << np.endpoint_rms << ',' << np.fb_residual_median << ',' << np.survival_pct
              << '\n';
    }

    tl::print_results("S4 native-unit assessment", rows);
    tl::note_artifacts(args, S);

    std::cout << "\n  Reading: the shipped primitives (FAST + pyramidal KLT) track the textured\n"
                 "  pattern well — sub-pixel endpoint error and a near-zero forward-backward\n"
                 "  residual on the good tracks. But the frontend ships only KLT's Lost/OutOfBounds\n"
                 "  status filter: there is NO forward-backward check and NO RANSAC, so a track that\n"
                 "  drifts to a confidently-wrong location (large FB residual, see the max) still\n"
                 "  reaches the backend at full weight. The pixel-noise sweep gives the TRUE track\n"
                 "  endpoint noise per px of image noise — the measurement sigma the backend should\n"
                 "  use, vs the optimistic assumed value (the S4 link to the #212 over-confidence).\n";
    return 0;
}
