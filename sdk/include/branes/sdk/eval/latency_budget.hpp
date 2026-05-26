// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/latency_budget.hpp — the per-frame VIO latency budget
// and the percentile helper used to enforce it.
//
// The budget is the wall-clock upper bound on one `VioEstimator::feed_image`
// call (front-end tracking + one backend update step) on the host CPU. It
// is a *tunable knob*: tighten these numbers as the SDK gets faster — the
// latency test will then hold the implementation to the new bound. Float
// is budgeted tighter than double (half the arithmetic width).
//
// Header-only, C++20.

#ifndef BRANES_SDK_EVAL_LATENCY_BUDGET_HPP
#define BRANES_SDK_EVAL_LATENCY_BUDGET_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace branes::sdk::eval {

/// Per-frame `feed_image` latency budget (milliseconds), keyed by the
/// estimator scalar. Both the median and the 99th percentile of measured
/// frame latencies must stay at or under `per_frame_ms`.
///
/// TUNABLE — RATCHET DOWN. The real-time target is ~30 ms (≈33 Hz). The
/// MVP front end re-runs full-frame FAST detection every frame, which on a
/// reference host (x86, -O2) measures ~40 ms median at EuRoC's 752×480, so
/// the *recorded* gate starts loose (with headroom for slower hosts) and
/// is meant to be tightened toward 30 ms as the front end gains
/// incremental detection / SIMD and the backend is optimized. Float is
/// budgeted tighter than double (half the arithmetic width).
///
/// Enforced only in optimized (NDEBUG) builds — a real-time budget is
/// meaningless under -O0.
template <class T>
struct FrameLatencyBudget;

template <>
struct FrameLatencyBudget<double> {
    static constexpr double per_frame_ms = 75.0;
};

template <>
struct FrameLatencyBudget<float> {
    static constexpr double per_frame_ms = 50.0;
};

/// Nearest-rank percentile of `samples` (p in [0, 1]); e.g. p = 0.5 is the
/// median, p = 0.99 the 99th percentile. Uses the nearest-rank definition
/// rank = ⌈p·n⌉ (so the tail isn't underestimated: p99 of 60 samples picks
/// the 60th, not the 59th). Copies and sorts. Throws if `samples` is empty.
[[nodiscard]] inline double percentile(std::vector<double> samples, double p) {
    if (samples.empty())
        throw std::invalid_argument("percentile: empty sample set");
    // NaN slips through the p<=0 / p>=1 guards below, and casting ceil(NaN)
    // to size_t is undefined behavior — reject it up front.
    if (std::isnan(p))
        throw std::invalid_argument("percentile: p is NaN");
    std::sort(samples.begin(), samples.end());
    const auto n = samples.size();
    if (p <= 0.0)
        return samples.front();
    if (p >= 1.0)
        return samples.back();
    auto rank = static_cast<std::size_t>(std::ceil(p * static_cast<double>(n)));
    if (rank < 1)
        rank = 1;
    if (rank > n)
        rank = n;
    return samples[rank - 1];
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_LATENCY_BUDGET_HPP
