// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/innovation_whiteness.hpp — zero-mean and whiteness tests on the
// filter's innovation sequence.
//
// NIS (consistency.hpp) tests the innovation *magnitude* (is the covariance the
// right size?) but is blind to two failure modes that a magnitude cannot see:
//
//   • a *biased* innovation — a non-zero mean — which is the signature of a
//     SYSTEMATIC error (camera/IMU extrinsics, feature-triangulation bias, a
//     time-sync offset), not a covariance mistune.
//   • a *correlated* innovation — temporally non-white — which means the filter
//     is leaving exploitable structure in the residuals (unmodelled dynamics, or
//     an observability/linearization inconsistency).
//
// This is the discriminator the #212 consistency-sweep diagnosis left open
// (docs/assessments/vio-consistency-sweep-diagnosis.md): the NIS over-confidence
// scaled away cleanly with R (1/R²), arguing "zero-mean but under-sized R"
// (→ observability/Jacobian) rather than "biased" (→ systematic). This test
// confirms or refutes that directly.
//
// Per update the MSCKF emits the signed statistic S_t = Σ_k r_k/σ over the
// projected residual and its dimension dof_t. Under a consistent zero-mean
// filter S_t ~ N(0, dof_t), so the standardized u_t = S_t/√dof_t ~ N(0,1) i.i.d.
//   – zero-mean test: mean of all components, scaled to a z-score.
//   – whiteness test: lag-1 autocorrelation of the u_t sequence, scaled to a
//     z-score (a one-lag Box–Pierce / Durbin–Watson-style check).
//
// Header-only, C++20.

#ifndef BRANES_SDK_EVAL_INNOVATION_WHITENESS_HPP
#define BRANES_SDK_EVAL_INNOVATION_WHITENESS_HPP

#include <branes/sdk/eval/consistency.hpp>  // detail::inverse_normal_cdf

#include <cmath>
#include <cstddef>

namespace branes::sdk::eval {

struct InnovationWhitenessReport {
    std::size_t updates = 0;    ///< number of updates accumulated
    std::size_t dof_total = 0;  ///< total residual components seen
    double mean = 0.0;          ///< mean normalized innovation component
    double mean_z = 0.0;        ///< zero-mean z-score: mean·√dof_total (~N(0,1) under H0)
    double lag1 = 0.0;          ///< lag-1 autocorrelation of the standardized per-update sums
    double lag1_z = 0.0;        ///< whiteness z-score: lag1·√(updates−1)
    double z_crit = 0.0;        ///< two-sided |z| threshold for the chosen α
    bool biased = false;        ///< |mean_z| > z_crit ⇒ systematic (non-zero-mean) innovation
    bool correlated = false;    ///< |lag1_z| > z_crit ⇒ temporally correlated (not white)
};

/// Accumulates the per-update signed innovation statistic and tests the
/// innovation sequence for zero mean and whiteness. Mirrors
/// `ConsistencyAccumulator`: feed it from each MSCKF update, then `report()`.
class InnovationWhitenessAccumulator {
public:
    /// Add one update's signed normalized-innovation sum `S_t = Σ_k r_k/σ` and
    /// its residual dimension `dof` (≥ 1; a zero-dof update is ignored).
    void add(double innov_sum, std::size_t dof) {
        if (dof == 0)
            return;
        sum_ += innov_sum;
        dof_total_ += dof;
        ++updates_;
        const double u = innov_sum / std::sqrt(static_cast<double>(dof));
        if (have_prev_) {
            cross_ += prev_u_ * u;
            ++cross_n_;
        }
        usq_ += u * u;
        prev_u_ = u;
        have_prev_ = true;
    }

    [[nodiscard]] std::size_t updates() const noexcept {
        return updates_;
    }

    /// Zero-mean (`biased`) and whiteness (`correlated`) verdicts at significance
    /// `alpha` (default 1%). The z-scores are standard-normal under the null of a
    /// consistent, zero-mean, white innovation sequence.
    [[nodiscard]] InnovationWhitenessReport report(double alpha = 0.01) const {
        InnovationWhitenessReport rep;
        rep.updates = updates_;
        rep.dof_total = dof_total_;
        if (dof_total_ == 0)
            return rep;
        rep.mean = sum_ / static_cast<double>(dof_total_);
        rep.mean_z = rep.mean * std::sqrt(static_cast<double>(dof_total_));
        if (cross_n_ > 0 && usq_ > 0.0) {
            rep.lag1 = cross_ / usq_;  // Σ u_t u_{t−1} / Σ u_t²
            rep.lag1_z = rep.lag1 * std::sqrt(static_cast<double>(cross_n_));
        }
        rep.z_crit = detail::inverse_normal_cdf(1.0 - alpha / 2.0);
        rep.biased = std::abs(rep.mean_z) > rep.z_crit;
        rep.correlated = std::abs(rep.lag1_z) > rep.z_crit;
        return rep;
    }

private:
    double sum_ = 0.0;  ///< Σ_t S_t  (= Σ over all components of r_k/σ)
    std::size_t dof_total_ = 0;
    std::size_t updates_ = 0;
    double prev_u_ = 0.0;
    bool have_prev_ = false;
    double cross_ = 0.0;  ///< Σ u_t u_{t−1}
    std::size_t cross_n_ = 0;
    double usq_ = 0.0;  ///< Σ u_t²
};

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_INNOVATION_WHITENESS_HPP
