// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/consistency.hpp — filter-consistency statistics for VIO
// evaluation: NEES (normalized estimation error squared) and NIS (normalized
// innovation squared), plus a chi-square run-consistency band.
//
// The keystone diagnostic instrument from docs/assessments/
// vio-diagnostic-methodology.md: a *consistent* (correctly-tuned, unbiased)
// estimator satisfies
//   - NEES:  e = x̂ ⊟ x_true,  E[eᵀ P⁻¹ e]  = dim(state)
//   - NIS:   ν = z − ẑ,        E[νᵀ S⁻¹ ν]  = dim(measurement)
// because a quadratic form of a zero-mean Gaussian with its own covariance is
// chi-square distributed with that many degrees of freedom. Over a run the
// (dof-)normalized average should sit in a chi-square confidence band; a value
// persistently > 1 means the filter is OVER-confident (covariance too small /
// a wrong Jacobian / a biased residual), < 1 means UNDER-confident (inflated
// noise). Per-state-block NEES localizes *which* state is overconfident.
//
// This file is the pure statistic — it has no estimator coupling, so it can be
// (and is) verified in isolation against the chi-square distribution itself, an
// oracle independent of any VIO code. The error/innovation vectors are supplied
// by the caller (the error must already be in the tangent space, e.g. SO3 via
// boxminus). Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_CONSISTENCY_HPP
#define BRANES_SDK_EVAL_CONSISTENCY_HPP

#include <branes/sdk/msckf/dense.hpp>  // DynMat, cholesky, cholesky_solve

#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>

namespace branes::sdk::eval {

using msckf::DynMat;

namespace detail {

/// Inverse standard-normal CDF (probit) Φ⁻¹(p), p ∈ (0,1). Acklam's rational
/// approximation (max abs error ~1.15e-9), used to turn a confidence level into
/// the z-multiplier of the chi-square run-consistency band. Clean-room from the
/// published algorithm.
[[nodiscard]] inline double inverse_normal_cdf(double p) {
    if (!(p > 0.0 && p < 1.0))
        throw std::invalid_argument("inverse_normal_cdf: p must be in (0,1)");
    static constexpr double a[] = {-3.969683028665376e+01,
                                   2.209460984245205e+02,
                                   -2.759285104469687e+02,
                                   1.383577518672690e+02,
                                   -3.066479806614716e+01,
                                   2.506628277459239e+00};
    static constexpr double b[] = {-5.447609879822406e+01,
                                   1.615858368580409e+02,
                                   -1.556989798598866e+02,
                                   6.680131188771972e+01,
                                   -1.328068155288572e+01};
    static constexpr double c[] = {-7.784894002430293e-03,
                                   -3.223964580411365e-01,
                                   -2.400758277161838e+00,
                                   -2.549732539343734e+00,
                                   4.374664141464968e+00,
                                   2.938163982698783e+00};
    static constexpr double d[] = {
        7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00, 3.754408661907416e+00};
    static constexpr double plow = 0.02425, phigh = 1.0 - 0.02425;
    if (p < plow) {
        const double q = std::sqrt(-2.0 * std::log(p));
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
    }
    if (p <= phigh) {
        const double q = p - 0.5, r = q * q;
        return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q /
               (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0);
    }
    const double q = std::sqrt(-2.0 * std::log(1.0 - p));
    return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) /
           ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
}

}  // namespace detail

/// `eᵀ P⁻¹ e` via a Cholesky solve (no explicit inverse). The shared core of
/// NEES and NIS. PRE: `P` is square and `e.size() == P.rows`. Throws
/// `std::invalid_argument` on a shape mismatch and `std::domain_error` if `P`
/// is not positive-definite — a degenerate covariance is a hard refusal, not a
/// silently-wrong number.
template <math::Scalar T>
[[nodiscard]] T normalized_squared(std::span<const T> e, const DynMat<T>& P) {
    const std::size_t n = P.rows;
    if (P.cols != n || e.size() != n)
        throw std::invalid_argument("normalized_squared: shape mismatch");
    if (n == 0)
        return T{0};
    DynMat<T> L;
    if (!msckf::cholesky(P, L))
        throw std::domain_error("normalized_squared: covariance is not positive-definite");
    DynMat<T> b(n, 1);
    for (std::size_t i = 0; i < n; ++i)
        b(i, 0) = e[i];
    const DynMat<T> x = msckf::cholesky_solve(L, b);  // x = P⁻¹ e
    T q{0};
    for (std::size_t i = 0; i < n; ++i)
        q += e[i] * x(i, 0);
    return q;
}

/// Normalized Innovation Squared: `νᵀ S⁻¹ ν`, with `S = H P Hᵀ + R` the
/// innovation covariance. Expected value = dim(ν) for a consistent filter.
template <math::Scalar T>
[[nodiscard]] T nis(std::span<const T> innovation, const DynMat<T>& S) {
    return normalized_squared<T>(innovation, S);
}

/// Normalized Estimation Error Squared: `eᵀ P⁻¹ e`, with `e = x̂ ⊟ x_true` the
/// tangent-space estimation error. Expected value = dim(state) when consistent.
template <math::Scalar T>
[[nodiscard]] T nees(std::span<const T> error, const DynMat<T>& P) {
    return normalized_squared<T>(error, P);
}

/// Verdict of a run-consistency test (see ConsistencyAccumulator).
struct ConsistencyReport {
    std::size_t samples = 0;          ///< number of statistics accumulated
    double total_dof = 0.0;           ///< Σ dofᵢ (sum is χ²(Σ dof) by additivity)
    double sum_statistic = 0.0;       ///< Σ statisticᵢ
    double normalized = 0.0;          ///< (Σ stat) / (Σ dof): 1 ≈ consistent
    double lower = 0.0, upper = 0.0;  ///< acceptance band on `normalized`
    bool overconfident = false;       ///< normalized > upper (covariance too small)
    bool underconfident = false;      ///< normalized < lower (covariance too large)
    [[nodiscard]] bool consistent() const noexcept {
        return !overconfident && !underconfident;
    }
};

/// Accumulates NEES or NIS samples and applies the large-sample chi-square
/// consistency test (Bar-Shalom). By chi-square additivity Σᵢ statᵢ ~ χ²(Σᵢ
/// dofᵢ); for a large total dof D, χ²(D) ≈ N(D, 2D), so the (1−α) two-sided
/// acceptance band on the dof-normalized average S/D is `1 ± z·√(2/D)`,
/// `z = Φ⁻¹(1−α/2)`. Per-sample dof may vary (e.g. NIS over frames with
/// different feature counts) — that is handled by summing dof.
class ConsistencyAccumulator {
public:
    /// Add one statistic with its degrees of freedom (state dim for NEES,
    /// measurement dim for NIS). `dof` must be positive.
    void add(double statistic, int dof) {
        if (dof <= 0)
            throw std::invalid_argument("ConsistencyAccumulator::add: dof must be positive");
        sum_ += statistic;
        dof_ += static_cast<double>(dof);
        ++n_;
    }

    [[nodiscard]] std::size_t samples() const noexcept {
        return n_;
    }

    /// Consistency verdict at significance `alpha` (default 5%). Requires at
    /// least one sample; the normal approximation is meant for a large total
    /// dof (a real run of hundreds–thousands of updates).
    [[nodiscard]] ConsistencyReport report(double alpha = 0.05) const {
        if (n_ == 0)
            throw std::domain_error("ConsistencyAccumulator::report: no samples");
        ConsistencyReport r;
        r.samples = n_;
        r.total_dof = dof_;
        r.sum_statistic = sum_;
        r.normalized = sum_ / dof_;
        const double z = detail::inverse_normal_cdf(1.0 - alpha / 2.0);
        const double half = z * std::sqrt(2.0 / dof_);  // half-width on S/D
        r.lower = 1.0 - half;
        r.upper = 1.0 + half;
        r.overconfident = r.normalized > r.upper;
        r.underconfident = r.normalized < r.lower;
        return r;
    }

private:
    double sum_ = 0.0;
    double dof_ = 0.0;
    std::size_t n_ = 0;
};

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_CONSISTENCY_HPP
