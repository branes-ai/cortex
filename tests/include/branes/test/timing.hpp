// branes/test/timing.hpp — latency-measurement helpers for regression tests.
//
// Pattern (used together with branes/test/budgets.hpp):
//
//     #include <branes/test/budgets.hpp>
//     #include <branes/test/timing.hpp>
//
//     TEST_CASE("fast_detect latency", "[perf][cv][fast]") {
//         auto stats = branes::test::measure_n(50, [] {
//             // ... call the operator under test
//         });
//         REQUIRE(stats.p99 < branes::test::budgets::fast_detect_p99);
//     }
//
// Use `measure_once` for a single-shot timing (cheap, low confidence).
// Use `measure_n` when the budget targets a percentile — N >= 50 keeps
// the p99 estimator stable across runs.

#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <utility>
#include <vector>

namespace branes::test {

using microseconds = std::chrono::microseconds;

/// Time a single invocation. Returns elapsed time in microseconds.
template <typename F>
[[nodiscard]] inline microseconds measure_once(F&& f) {
    const auto t0 = std::chrono::steady_clock::now();
    std::forward<F>(f)();
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<microseconds>(t1 - t0);
}

struct LatencyStats {
    microseconds min;
    microseconds median;
    microseconds p99;
    microseconds max;
    std::size_t samples;
};

/// Run `f` `n` times and return min / median / p99 / max in microseconds.
/// `n` should be >= 50 for stable p99 estimates; smaller `n` is fine for
/// smoke-level checks but the p99 will be noisy.
template <typename F>
[[nodiscard]] inline LatencyStats measure_n(std::size_t n, F&& f) {
    std::vector<long long> us;
    us.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        us.push_back(measure_once(f).count());
    }
    std::sort(us.begin(), us.end());

    // Guard against n==0 to keep callers honest, but treat it as a
    // programmer error rather than papering over it with a default.
    const std::size_t last_idx = (n == 0) ? 0 : n - 1;
    const std::size_t median_idx = n / 2;
    const std::size_t p99_idx = (n * 99) / 100;

    return LatencyStats{
        microseconds(n == 0 ? 0 : us.front()),
        microseconds(n == 0 ? 0 : us[median_idx]),
        microseconds(n == 0 ? 0 : us[p99_idx]),
        microseconds(n == 0 ? 0 : us[last_idx]),
        n,
    };
}

}  // namespace branes::test
