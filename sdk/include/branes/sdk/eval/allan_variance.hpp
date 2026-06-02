// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/allan_variance.hpp — Allan variance / deviation for IMU noise
// characterization.
//
// The first stage of CHARACTERIZE in the diagnostic methodology
// (docs/assessments/vio-diagnostic-methodology.md): measure what the sensor
// stream actually contains before blaming the estimator. The Allan deviation
// σ_A(τ) of a stationary IMU axis, plotted against the averaging time τ on
// log-log axes, separates the noise processes by their slope:
//
//   slope −1/2  white noise (angle/velocity random walk) — the continuous-time
//               noise density N the filter's measurement noise needs. In this
//               region σ_A(τ) = N/√τ, so N = σ_A(τ)·√τ is constant.
//   slope  0    bias instability — the flat floor (the irreducible drift).
//   slope +1/2  rate random walk — the bias random-walk the filter's process
//               noise needs.
//
// So Allan variance answers, quantitatively: "are we running the filter on the
// right Q?" — compare the measured N against `VioConfig::{gyro,accel}_noise_density`.
//
// Caveat: bias-instability and random-walk (large τ) want a long STATIC log; on
// a moving sequence platform dynamics add low-frequency power that contaminates
// large τ. The white-noise density (small τ) is broadband and largely
// motion-independent, so it is usable even on a flight sequence — which is why
// `white_noise_density` reads only the smallest averaging factors.
//
// Non-overlapping estimator (Allan's original): simple and unbiased; the
// overlapping variant uses more of the data but is not needed here. Header-only,
// C++20, type-generic.

#ifndef BRANES_SDK_EVAL_ALLAN_VARIANCE_HPP
#define BRANES_SDK_EVAL_ALLAN_VARIANCE_HPP

#include <branes/math/arithmetic.hpp>
#include <branes/math/lie/detail.hpp>

#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

namespace branes::sdk::eval {

/// Non-overlapping Allan deviation `σ_A(τ)` of a scalar series for averaging
/// factor `m` (so `τ = m·dt`): bin the series into groups of `m`, average each
/// group, and take `√( ½ · mean( (avgₖ₊₁ − avgₖ)² ) )`. Returns 0 when there are
/// fewer than two full bins. `m` must be ≥ 1.
template <math::Scalar T>
[[nodiscard]] T allan_deviation_at(std::span<const T> x, std::size_t m) {
    if (m == 0)
        throw std::invalid_argument("allan_deviation_at: m must be >= 1");
    const std::size_t k = x.size() / m;  // number of full bins
    if (k < 2)
        return T{0};
    std::vector<T> avg(k);
    for (std::size_t b = 0; b < k; ++b) {
        T s{0};
        for (std::size_t i = 0; i < m; ++i)
            s += x[b * m + i];
        avg[b] = s / static_cast<T>(m);
    }
    T acc{0};
    for (std::size_t b = 0; b + 1 < k; ++b) {
        const T d = avg[b + 1] - avg[b];
        acc += d * d;
    }
    return math::lie::detail::sqrt_(acc / (T{2} * static_cast<T>(k - 1)));
}

/// Octave-spaced averaging times `τ = (2^j)·dt` while at least two bins remain,
/// for plotting the Allan-deviation curve.
template <math::Scalar T>
[[nodiscard]] std::vector<T> octave_taus(std::size_t n_samples, T dt) {
    std::vector<T> taus;
    for (std::size_t m = 1; n_samples / m >= 2; m <<= 1)
        taus.push_back(static_cast<T>(m) * dt);
    return taus;
}

/// `σ_A` at each averaging time in `taus` (each rounded to ≥ 1 sample by
/// `m = round(τ/dt)`). Pairs with `octave_taus` for the log-log curve.
template <math::Scalar T>
[[nodiscard]] std::vector<T> allan_deviation(std::span<const T> x, T dt, std::span<const T> taus) {
    if (!(dt > T{0}))
        throw std::invalid_argument("allan_deviation: dt must be positive");
    std::vector<T> out;
    out.reserve(taus.size());
    for (const T tau : taus) {
        const auto m = static_cast<std::size_t>(tau / dt + T{1} / T{2});
        out.push_back(allan_deviation_at<T>(x, m == 0 ? 1 : m));
    }
    return out;
}

/// The white-noise (continuous-time) density `N` of the series — the quantity
/// the filter's measurement noise needs (e.g. rad/s/√Hz for a gyro axis). In
/// the white-noise region `σ_A(τ) = N/√τ`, so `σ_A(τ)·√τ = N` is constant there;
/// estimate `N` by averaging `σ_A·√τ` over the smallest `n_small` octave
/// averaging factors (the region least contaminated by bias instability or
/// platform motion). Returns 0 if the series is too short.
template <math::Scalar T>
[[nodiscard]] T white_noise_density(std::span<const T> x, T dt, std::size_t n_small = 4) {
    if (!(dt > T{0}))
        throw std::invalid_argument("white_noise_density: dt must be positive");
    T sum{0};
    std::size_t cnt = 0;
    for (std::size_t j = 0; j < n_small; ++j) {
        const std::size_t m = std::size_t{1} << j;  // 1, 2, 4, ...
        if (x.size() / m < 2)
            break;
        const T s = allan_deviation_at<T>(x, m);
        sum += s * math::lie::detail::sqrt_(static_cast<T>(m) * dt);
        ++cnt;
    }
    return cnt == 0 ? T{0} : sum / static_cast<T>(cnt);
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_ALLAN_VARIANCE_HPP
