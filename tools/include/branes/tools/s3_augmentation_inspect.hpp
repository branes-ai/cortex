// SPDX-License-Identifier: MIT
//
// branes/tools/s3_augmentation_inspect.hpp — the S3 (state augmentation /
// stochastic cloning) inspector (epic #371, issue #378). The real-data companion
// to the synthetic S3 probe (sdk/eval/clone_window_probe.hpp); renders on the
// filter-internal tier, built on the S4 inspector template (issue #374).
//
// At a keyframe the filter CLONES the current IMU pose (θ, p) into a new state
// block so future measurements can relate it to later poses. A clone is a
// DETERMINISTIC COPY, so the contract is exact: the new block's marginal must
// equal the cloned pose's marginal, and its cross-covariance with every existing
// state must equal the pose's — "just add a zero/block-diagonal block" is the
// classic stochastic-cloning inconsistency. The real operator is the shipped
// `StateHelper::augment_clone`; this inspector runs it on a real correlated
// covariance and measures those residuals + the before/after covariance for the
// heatmap that shows the cloned block appear.
//
// Decoupled behind the S3 I/O struct (`S3Input{state}` → `S3Record`). `run()` does
// not mutate the caller's state (it augments a copy), and is a pure function of
// the input covariance — unit-testable on a synthetic correlated state without the
// ~1.5 GB EuRoC dataset.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_S3_AUGMENTATION_INSPECT_HPP
#define BRANES_TOOLS_S3_AUGMENTATION_INSPECT_HPP

#include <branes/sdk/msckf/dense.hpp>  // is_positive_semidefinite
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/msckf/state_helper.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace branes::tools {

/// Everything one clone augmentation produced — the inspector's record.
struct S3AugmentRecord {
    std::size_t dim_before = 0, dim_after = 0;
    std::size_t clone_offset = 0;  ///< where the new 6×6 clone block sits in P_after
    std::size_t clone_dim = 6;
    double clone_marginal_err = 0.0;  ///< ‖P'[clone,clone] − P[pose,pose]‖max (≈ 0: faithful copy)
    double clone_cross_err = 0.0;     ///< ‖P'[clone,*] − P[pose,*]‖max over existing states (≈ 0)
    bool psd = false;                 ///< the augmented covariance stays positive-semidefinite
    std::size_t n_clones_after = 0;
    std::vector<double> cov_before;  ///< row-major dim_before²
    std::vector<double> cov_after;   ///< row-major dim_after²
};

[[nodiscard]] inline nlohmann::json to_json(const S3AugmentRecord& r) {
    using nlohmann::json;
    return json{{"dim_before", r.dim_before},
                {"dim_after", r.dim_after},
                {"clone_offset", r.clone_offset},
                {"clone_dim", r.clone_dim},
                {"clone_marginal_err", r.clone_marginal_err},
                {"clone_cross_err", r.clone_cross_err},
                {"psd", r.psd},
                {"n_clones_after", r.n_clones_after},
                {"cov_before", r.cov_before},
                {"cov_after", r.cov_after}};
}

/// Runs the shipped clone augmentation and characterizes the new block.
class S3AugmentationInspector {
public:
    using State = branes::sdk::msckf::State<double>;
    using StateHelper = branes::sdk::msckf::StateHelper<double>;
    using DynMat = branes::sdk::msckf::DynMat<double>;

    struct S3Input {
        State state;  ///< the keyframe state (mean + covariance) to clone
    };

    /// Augment a COPY of `in.state` and report. The caller's state is untouched.
    [[nodiscard]] S3AugmentRecord run(const S3Input& in) const {
        namespace ms = branes::sdk::msckf;
        S3AugmentRecord r;

        const DynMat P = in.state.covariance();
        const std::size_t d = in.state.dim();
        r.dim_before = d;
        r.cov_before = flatten(P);

        // The cloned pose is the IMU (θ, p) block.
        const std::vector<std::size_t> pose{
            State::kTheta + 0, State::kTheta + 1, State::kTheta + 2, State::kPos + 0, State::kPos + 1, State::kPos + 2};

        State s = in.state;  // copy — augment mutates
        StateHelper::augment_clone(s);
        const DynMat Pp = s.covariance();
        r.dim_after = s.dim();
        r.clone_offset = d;  // the new clone is appended at the end
        r.n_clones_after = s.clones.size();
        r.cov_after = flatten(Pp);

        const std::vector<std::size_t> clone{d + 0, d + 1, d + 2, d + 3, d + 4, d + 5};
        // marginal: P'[clone,clone] must equal P[pose,pose].
        r.clone_marginal_err = block_diff(Pp, clone, clone, P, pose, pose);
        // cross: P'[clone, existing k] must equal P[pose, k] for every prior state k.
        double cross = 0.0;
        for (std::size_t i = 0; i < clone.size(); ++i)
            for (std::size_t k = 0; k < d; ++k)
                cross = std::max(cross, std::abs(Pp(clone[i], k) - P(pose[i], k)));
        r.clone_cross_err = cross;
        r.psd = ms::is_positive_semidefinite(Pp);
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

    [[nodiscard]] static double block_diff(const DynMat& A,
                                           const std::vector<std::size_t>& ra,
                                           const std::vector<std::size_t>& ca,
                                           const DynMat& B,
                                           const std::vector<std::size_t>& rb,
                                           const std::vector<std::size_t>& cb) {
        double m = 0.0;
        for (std::size_t i = 0; i < ra.size(); ++i)
            for (std::size_t j = 0; j < ca.size(); ++j)
                m = std::max(m, std::abs(A(ra[i], ca[j]) - B(rb[i], cb[j])));
        return m;
    }
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_S3_AUGMENTATION_INSPECT_HPP
