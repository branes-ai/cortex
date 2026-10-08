// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/invariants/stream_checks.hpp — invariants over sensor STREAMS
// and over statistics accumulated across a window (epic #444 §B, issue #445):
// timestamp monotonicity and Δt bounds, the static-window ‖a‖ ≈ g check, and
// NEES/NIS χ² consistency over a window.
//
// Timestamps stay `double` by design (the pipeline never carries time in the
// estimation type T), so the timing checks are not templated. The χ² window
// checks wrap the ConsistencyAccumulator of consistency.hpp, which is the
// `double` telemetry accumulator by design.
//
// Header-only, C++20.

#ifndef BRANES_SDK_EVAL_INVARIANTS_STREAM_CHECKS_HPP
#define BRANES_SDK_EVAL_INVARIANTS_STREAM_CHECKS_HPP

#include <branes/sdk/eval/consistency.hpp>
#include <branes/sdk/eval/invariants/core.hpp>
#include <branes/sdk/eval/invariants/linalg.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace branes::sdk::eval::inv {

/// Timestamps strictly increasing (inputs: IMU and frame streams). Value: the
/// count of non-increasing consecutive pairs (Δt ≤ 0, or a NaN); threshold 0.
[[nodiscard]] inline InvariantResult check_timestamps_increasing(
    std::span<const double> t, Stage stage = Stage::Inputs, std::string_view name = "timestamps.strictly_increasing") {
    std::size_t bad = 0;
    for (std::size_t i = 1; i < t.size(); ++i)
        bad += (t[i] - t[i - 1] > 0.0) ? 0 : 1;
    return detail::make(name, "count", stage, Bound::Upper, static_cast<double>(bad), 0.0);
}

/// Δt within the sensor-rate bounds (inputs): two results, min Δt ≥ dt_min and
/// max Δt ≤ dt_max, in seconds. The bounds are physical (nominal period ±
/// jitter, a dropout limit), so they are the caller's.
[[nodiscard]] inline std::vector<InvariantResult>
check_dt_bounds(std::span<const double> t, double dt_min, double dt_max, Stage stage = Stage::Inputs) {
    double lo = detail::kInf, hi = -detail::kInf;
    bool finite = true;
    for (std::size_t i = 1; i < t.size(); ++i) {
        const double dt = t[i] - t[i - 1];
        finite = finite && std::isfinite(dt);
        lo = std::min(lo, dt);
        hi = std::max(hi, dt);
    }
    if (t.size() < 2)
        lo = hi = 0.5 * (dt_min + dt_max);  // no interval to violate
    else if (!finite)                       // std::min/max drop NaN; a non-finite interval fails both bounds
        return {detail::nonfinite_violation("timestamps.dt_min", stage, Bound::Lower),
                detail::nonfinite_violation("timestamps.dt_max", stage)};
    return {detail::make("timestamps.dt_min", "s", stage, Bound::Lower, lo, dt_min),
            detail::make("timestamps.dt_max", "s", stage, Bound::Upper, hi, dt_max)};
}

/// Static window: the mean specific-force magnitude equals gravity (inputs,
/// before S1). Value: | mean ‖aᵢ‖ − g₀ | in m/s²; threshold `tol` (physical:
/// accelerometer noise/√N plus bias and scale-factor error).
template <math::Scalar T>
[[nodiscard]] InvariantResult check_static_gravity(std::span<const std::array<T, 3>> accel,
                                                   T g_nominal,
                                                   T tol,
                                                   Stage stage = Stage::Inputs,
                                                   std::string_view name = "imu.static_accel_norm") {
    if (accel.empty())
        return detail::make(name, "m/s^2", stage, Bound::Upper, detail::kInf, detail::to_double(tol));
    T sum{0};
    for (const auto& a : accel)
        sum += la::sqrt_(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const T mean = sum / T(static_cast<double>(accel.size()));
    return detail::make(
        name, "m/s^2", stage, Bound::Upper, detail::to_double(la::abs_(mean - g_nominal)), detail::to_double(tol));
}

/// NEES or NIS consistent over a window (S6d for NIS, end to end for NEES): the
/// dof-normalized sum Σstat / Σdof inside the (1 − α) χ² band (Bar-Shalom; see
/// ConsistencyAccumulator). Value dimensionless, target 1; Band bounds. An empty
/// window is a violation (nothing was measured).
[[nodiscard]] inline InvariantResult
check_chi2_window(const ConsistencyAccumulator& acc, Stage stage, std::string_view name, double alpha = 0.05) {
    if (acc.samples() == 0)
        return detail::make(name, "dimensionless", stage, Bound::Upper, detail::kInf, 0.0);
    const ConsistencyReport rep = acc.report(alpha);
    return detail::make(name, "dimensionless", stage, Bound::Band, rep.normalized, rep.lower, rep.upper);
}

}  // namespace branes::sdk::eval::inv

#endif  // BRANES_SDK_EVAL_INVARIANTS_STREAM_CHECKS_HPP
