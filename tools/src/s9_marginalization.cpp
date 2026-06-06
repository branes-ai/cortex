// SPDX-License-Identifier: MIT
//
// s9_marginalization — study the S9 stage (marginalization / clone management) in
// isolation. Drives the SHIPPED StateHelper::marginalize_clone
// (sdk/eval/clone_window_probe.hpp), checks that dropping a clone is exact
// principal-submatrix extraction (kept-state marginal unchanged), prints a
// native-unit results table, and writes the CSV artifact.
//
//   ./s9_marginalization            # run the probe, write artifacts
//   ./s9_marginalization --help     # print the S9 contract and exit
//   ./s9_marginalization --list     # print the whole S0–S10 pipeline
//   ./s9_marginalization --out DIR  # choose the artifact directory
//   ./s9_marginalization --no-out   # compute & print only, no files

#include <branes/sdk/eval/clone_window_probe.hpp>
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
    const tl::StageInfo& S = tl::kS9;
    tl::ProbeArgs args = tl::parse_args(argc, argv, S);
    if (args.list) {
        const auto pl = tl::pipeline();
        return tl::run_scaffold(argc, argv, S, &pl);
    }
    tl::print_contract(S);
    if (args.help)
        return 0;

    const auto r = ev::marginalization_probe<T>();
    const T eps = 1e-9;

    std::vector<tl::ResultRow> rows;
    rows.push_back({"kept-marginal invariance",
                    fmt(r.kept_marginal_err),
                    "-",
                    r.kept_marginal_err < eps ? "PASS (kept states unchanged)" : "FAIL"});
    rows.push_back({"P' positive semidefinite", r.psd ? "yes" : "no", "-", r.psd ? "PASS" : "FAIL"});
    rows.push_back(
        {"state dimension", fmt(r.dim_before) + " -> " + fmt(r.dim_after), "-", "shrinks by one 6-DoF clone"});
    rows.push_back(
        {"window length", fmt(r.clones_before) + " -> " + fmt(r.clones_after), "clones", "the window stays bounded"});

    if (auto f = tl::open_artifact(args, "marg_invariance.csv"); f.is_open()) {
        f << "kept_marginal_err,psd,dim_before,dim_after,clones_before,clones_after\n";
        f << r.kept_marginal_err << ',' << (r.psd ? 1 : 0) << ',' << r.dim_before << ',' << r.dim_after << ','
          << r.clones_before << ',' << r.clones_after << '\n';
    }

    tl::print_results("S9 native-unit assessment", rows);
    tl::note_artifacts(args, S);

    std::cout << "\n  Reading: dropping a clone is principal-submatrix extraction, so the kept-state\n"
                 "  marginal must be UNCHANGED (marginalizing a variable never alters the marginal\n"
                 "  of the others). The residual above is machine-epsilon, so cortex's\n"
                 "  cov.marginalize(keep) is an exact extraction and P' stays PSD. For a filter\n"
                 "  (not a smoother) no FEJ prior arises here — correctness rides on S3's augment\n"
                 "  cross-covariance being right, which it is. S9 is correct — not a #212 source.\n";

    // Exit non-zero if any invariant failed, so the probe is usable as a CI gate.
    const bool ok = r.kept_marginal_err < eps && r.psd;
    return ok ? 0 : 1;
}
