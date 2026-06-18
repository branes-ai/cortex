// SPDX-License-Identifier: MIT
//
// branes/tools/s9_marginalization_inspect.hpp — the S9 (marginalization / clone
// management) inspector (epic #371, issue #383). The real-data companion to the
// synthetic S9 probe (sdk/eval/clone_window_probe.hpp); renders on the
// filter-internal tier, built on the S4 inspector template (issue #374). It is
// the inverse of the S3 augmentation inspector.
//
// When a clone leaves the sliding window the filter MARGINALIZES it. Dropping a
// variable is principal-submatrix extraction, so the contract is exact: the
// kept-state marginal must be UNCHANGED (marginalizing a variable never alters the
// marginal of the others). The real operator is the shipped
// `StateHelper::marginalize_clone`; this inspector runs it on a real correlated
// covariance and measures the kept-marginal residual, the information dropped (the
// clone's own uncertainty + its correlations with the kept states), PSD, and the
// before/after covariance for the heatmap that shows the block disappear.
//
// Decoupled behind the S9 I/O struct (`S9Input{state, idx}` → `S9Record`). `run()`
// does not mutate the caller's state (it marginalizes a copy) and is a pure
// function of the input covariance — unit-testable on a synthetic correlated state
// without the ~1.5 GB EuRoC dataset.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_S9_MARGINALIZATION_INSPECT_HPP
#define BRANES_TOOLS_S9_MARGINALIZATION_INSPECT_HPP

#include <branes/sdk/msckf/dense.hpp>  // is_positive_semidefinite
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/msckf/state_helper.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace branes::tools {

/// Everything one clone marginalization produced — the inspector's record.
struct S9MarginalizeRecord {
    bool valid =
        false;  ///< the marginalization ran and its postcondition held; residuals below are meaningful only if true
    std::size_t dim_before = 0, dim_after = 0;
    std::size_t dropped_offset = 0;  ///< where the dropped clone block sat in P_before
    std::size_t clone_dim = 0;       ///< == State::kCloneDim (set by run)
    std::size_t dropped_index = 0;   ///< which clone (by window index) was dropped
    double kept_marginal_err = 0.0;  ///< ‖P'[kept] − P[kept,kept]‖max (≈ 0: exact extraction)
    double dropped_sigma = 0.0;      ///< √trace of the dropped clone's 6×6 block (uncertainty no longer tracked)
    double max_cross_dropped =
        0.0;           ///< max |cross-cov| between the dropped clone and the kept states (correlations discarded)
    bool psd = false;  ///< the reduced covariance stays positive-semidefinite
    std::size_t n_clones_before = 0, n_clones_after = 0;
    std::vector<double> cov_before;  ///< row-major dim_before²
    std::vector<double> cov_after;   ///< row-major dim_after²
};

[[nodiscard]] inline nlohmann::json to_json(const S9MarginalizeRecord& r) {
    using nlohmann::json;
    return json{{"valid", r.valid},
                {"dim_before", r.dim_before},
                {"dim_after", r.dim_after},
                {"dropped_offset", r.dropped_offset},
                {"clone_dim", r.clone_dim},
                {"dropped_index", r.dropped_index},
                {"kept_marginal_err", r.kept_marginal_err},
                {"dropped_sigma", r.dropped_sigma},
                {"max_cross_dropped", r.max_cross_dropped},
                {"psd", r.psd},
                {"n_clones_before", r.n_clones_before},
                {"n_clones_after", r.n_clones_after},
                {"cov_before", r.cov_before},
                {"cov_after", r.cov_after}};
}

/// Runs the shipped clone marginalization and characterizes what was dropped.
class S9MarginalizationInspector {
public:
    using State = branes::sdk::msckf::State<double>;
    using StateHelper = branes::sdk::msckf::StateHelper<double>;
    using DynMat = branes::sdk::msckf::DynMat<double>;

    struct S9Input {
        State state;          ///< the keyframe state (mean + covariance) before the drop
        std::size_t idx = 0;  ///< which clone (window index) to marginalize; default = oldest
    };

    /// Marginalize a COPY of `in.state` and report. The caller's state is untouched.
    [[nodiscard]] S9MarginalizeRecord run(const S9Input& in) const {
        namespace ms = branes::sdk::msckf;
        S9MarginalizeRecord r;

        r.clone_dim = State::kCloneDim;  // bind the reported block size to the SDK constant
        const DynMat P = in.state.covariance();
        const std::size_t d = in.state.dim();
        r.dim_before = d;
        r.dropped_index = in.idx;
        r.n_clones_before = in.state.clones.size();
        r.cov_before = flatten(P);
        if (in.idx >= in.state.clones.size())
            return r;  // invalid index — valid stays false, residuals at defaults

        const std::size_t off = in.state.clone_offset(in.idx);
        r.dropped_offset = off;

        // The kept indices: everything except the dropped clone's 6×6 block.
        std::vector<std::size_t> keep;
        keep.reserve(d - State::kCloneDim);
        for (std::size_t i = 0; i < d; ++i)
            if (i < off || i >= off + State::kCloneDim)
                keep.push_back(i);

        State s = in.state;  // copy — marginalize mutates
        StateHelper::marginalize_clone(s, in.idx);
        const DynMat Pp = s.covariance();
        r.dim_after = s.dim();
        r.n_clones_after = s.clones.size();
        r.cov_after = flatten(Pp);
        r.psd = ms::is_positive_semidefinite(Pp);

        // Postcondition: exactly one clone block removed; marginalization compacts the
        // kept states in order, so keep[i] in P maps to position i in P'. If it ever
        // doesn't hold, FAIL CLOSED (valid stays false) so a broken contract is never
        // serialized/printed as a clean pass.
        if (r.dim_after != keep.size())
            return r;

        // Kept-state marginal must be UNCHANGED (exact principal-submatrix extraction).
        double kept = 0.0;
        for (std::size_t i = 0; i < keep.size(); ++i)
            for (std::size_t j = 0; j < keep.size(); ++j)
                kept = std::max(kept, std::abs(Pp(i, j) - P(keep[i], keep[j])));
        r.kept_marginal_err = kept;

        // Information dropped: the clone's own σ + its correlations with the kept states.
        double tr = 0.0;
        for (std::size_t a = 0; a < State::kCloneDim; ++a)
            tr += P(off + a, off + a);
        r.dropped_sigma = std::sqrt(tr < 0.0 ? 0.0 : tr);
        double cross = 0.0;
        for (std::size_t a = 0; a < State::kCloneDim; ++a)
            for (std::size_t k : keep)
                cross = std::max(cross, std::abs(P(off + a, k)));
        r.max_cross_dropped = cross;
        r.valid = true;  // ran end-to-end with the postcondition satisfied
        return r;
    }

private:
    [[nodiscard]] static std::vector<double> flatten(const DynMat& M) {
        std::vector<double> v;
        v.reserve(M.rows * M.cols);
        for (std::size_t i = 0; i < M.rows; ++i)
            for (std::size_t j = 0; j < M.cols; ++j)
                v.push_back(M(i, j));
        return v;
    }
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_S9_MARGINALIZATION_INSPECT_HPP
