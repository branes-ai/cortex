// branes/test/budgets.hpp — per-operator latency budgets.
//
// Each constant is the upper bound on the named metric. Tests use them
// via branes::test::measure_n (see branes/test/timing.hpp):
//
//     auto stats = branes::test::measure_n(50, [] { ... });
//     REQUIRE(stats.p99 < branes::test::budgets::fast_detect_p99);
//
// Budgets are intentionally tight. If a regression bumps you over, the
// expectation is "fix the regression". Bumping the budget itself is
// allowed but must be explicit in the same PR and called out in the
// commit message, so the velocity loss shows up in the CHANGELOG.
//
// Naming convention:
//   <operator>_<percentile>   — e.g. fast_detect_p99, vio_feed_image_median
//
// Initial state: empty. Phase 3+ issues populate this as operators land.
// First entry is expected with issue #37 (FAST corner detector).

#pragma once

#include <chrono>

namespace branes::test::budgets {

/// Sentinel for "no budget set yet — placeholder accepts any timing".
/// Use this when scaffolding a test before the operator is benchmarked
/// (so the test still runs and validates correctness, just not perf).
inline constexpr auto unset = std::chrono::microseconds::max();

// Write entries as `std::chrono::microseconds(123)` / `milliseconds(5)`.
// Avoid the `123us` / `5ms` literals here — they require
// `using namespace std::chrono_literals`, which has no business
// polluting an `inline` header (it would leak through every TU that
// includes budgets.hpp).
//
// CV front-end (cv/) — populated by Phase 3, issues #37, #38, #48, #49.
// inline constexpr auto fast_detect_p99   = std::chrono::microseconds(...);  // #37
// inline constexpr auto klt_track_p99     = std::chrono::microseconds(...);  // #38
// inline constexpr auto pyramid_build_p99 = std::chrono::microseconds(...);  // #49
//
// MSCKF backend (sdk/) — populated by Phase 3, issues #40, #43, #44.
// inline constexpr auto imu_preintegration_p99 = std::chrono::microseconds(...);  // #40
// inline constexpr auto state_propagator_p99   = std::chrono::microseconds(...);  // #43
// inline constexpr auto camera_update_p99      = std::chrono::microseconds(...);  // #44
//
// Top-level VIO API — populated by Phase 3, issue #47.
// inline constexpr auto vio_feed_image_median = std::chrono::milliseconds(...);  // #47
// inline constexpr auto vio_feed_image_p99    = std::chrono::milliseconds(...);  // #47

}  // namespace branes::test::budgets
