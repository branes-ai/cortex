// SPDX-License-Identifier: MIT
//
// branes/math/stats.hpp — the few distribution functions the estimators need
// to turn a significance level into a threshold: the standard-normal quantile
// (probit), the regularized lower incomplete gamma function, and the χ²
// quantile built on it.
//
// These are *configuration-time* numbers (a gate line, a consistency band), so
// they are computed in `double` whatever arithmetic the filter runs in; the
// caller casts the threshold to its own scalar type.
//
// Clean-room from the published formulas: Acklam's rational approximation for
// the probit; the power series and Legendre continued fraction for the
// incomplete gamma function (Abramowitz & Stegun 6.5.29 and 6.5.31); the
// Wilson–Hilferty cube-root approximation as the starting point of a Newton
// solve for the χ² quantile.
//
// Header-only, C++20.

#ifndef BRANES_MATH_STATS_HPP
#define BRANES_MATH_STATS_HPP

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace branes::math {

/// Inverse standard-normal CDF (probit) Φ⁻¹(p), p ∈ (0,1). Acklam's rational
/// approximation, max abs error ~1.15e-9.
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

/// Regularized lower incomplete gamma function P(a, x) = γ(a, x) / Γ(a), for
/// a > 0 and x ≥ 0. The power series converges fast for x < a + 1; above that
/// the continued fraction for Q = 1 − P does (evaluated by the modified Lentz
/// method). Relative accuracy ~1e-14.
[[nodiscard]] inline double regularized_gamma_p(double a, double x) {
    if (!(a > 0.0) || !(x >= 0.0))
        throw std::invalid_argument("regularized_gamma_p: need a > 0 and x >= 0");
    if (x == 0.0)
        return 0.0;
    constexpr double eps = 1e-15;
    constexpr int max_iter = 1000;
    const double log_prefix = -x + a * std::log(x) - std::lgamma(a);
    if (x < a + 1.0) {
        // P = e^{-x} x^a / Γ(a) · Σ_{n≥0} x^n / (a (a+1) … (a+n)).
        double ap = a, term = 1.0 / a, sum = term;
        for (int n = 0; n < max_iter; ++n) {
            ap += 1.0;
            term *= x / ap;
            sum += term;
            if (std::abs(term) < std::abs(sum) * eps)
                break;
        }
        return sum * std::exp(log_prefix);
    }
    // Q = e^{-x} x^a / Γ(a) · 1/(x+1−a − 1·(1−a)/(x+3−a − 2·(2−a)/(x+5−a − …))).
    constexpr double tiny = std::numeric_limits<double>::min() / eps;
    double bb = x + 1.0 - a;
    double cc = 1.0 / tiny;
    double dd = 1.0 / bb;
    double h = dd;
    for (int i = 1; i < max_iter; ++i) {
        const double an = -static_cast<double>(i) * (static_cast<double>(i) - a);
        bb += 2.0;
        dd = an * dd + bb;
        if (std::abs(dd) < tiny)
            dd = tiny;
        cc = bb + an / cc;
        if (std::abs(cc) < tiny)
            cc = tiny;
        dd = 1.0 / dd;
        const double del = dd * cc;
        h *= del;
        if (std::abs(del - 1.0) < eps)
            break;
    }
    return 1.0 - std::exp(log_prefix) * h;
}

/// The p-quantile of the χ² distribution with k degrees of freedom: the x with
/// P(χ²_k ≤ x) = p, for p ∈ (0,1) and k ≥ 1. Newton's method on the CDF
/// P(k/2, x/2), started from the Wilson–Hilferty approximation; converges to
/// ~1e-12 relative in a handful of steps. Example: chi_square_quantile(0.95, 1)
/// = 3.841459, chi_square_quantile(0.95, 9) = 16.918978.
[[nodiscard]] inline double chi_square_quantile(double p, std::size_t k) {
    if (!(p > 0.0 && p < 1.0))
        throw std::invalid_argument("chi_square_quantile: p must be in (0,1)");
    if (k == 0)
        throw std::invalid_argument("chi_square_quantile: k must be at least 1");
    const double kk = static_cast<double>(k), half_k = 0.5 * kk;
    const double w = 2.0 / (9.0 * kk);
    const double c = 1.0 - w + inverse_normal_cdf(p) * std::sqrt(w);
    double x = c > 0.0 ? kk * c * c * c : 0.5 * kk * p;  // W-H fails far in the low tail
    if (!(x > 0.0))
        x = std::numeric_limits<double>::min();
    const double log_norm = -half_k * std::log(2.0) - std::lgamma(half_k);
    for (int it = 0; it < 100; ++it) {
        const double f = regularized_gamma_p(half_k, 0.5 * x) - p;
        const double pdf = std::exp(log_norm + (half_k - 1.0) * std::log(x) - 0.5 * x);
        if (!(pdf > 0.0))
            break;
        double next = x - f / pdf;
        if (!(next > 0.0))
            next = 0.5 * x;  // keep the iterate in the support
        const bool done = std::abs(next - x) <= 1e-12 * x;
        x = next;
        if (done)
            break;
    }
    return x;
}

}  // namespace branes::math

#endif  // BRANES_MATH_STATS_HPP
