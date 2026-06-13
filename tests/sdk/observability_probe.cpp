// VIO observability null-space audit (issue #337).
//
// Locks the measured root cause of the structural NEES over-confidence: the
// camera measurement Jacobian preserves the 4-DoF unobservable null space
// (global translation ×3 + gravity-yaw ×1) ONLY when linearized at a single
// consistent point. Linearized at the evolving estimate — as the shipped EKF
// does, with no FEJ anchor — it leaks information into the YAW gauge direction,
// the mechanism behind the EuRoC attitude-NEES ≈ 993. Translation is preserved
// regardless (the clone-δp and feature columns are ±Hf and cancel).
//
// Measured leak (yaw, ‖H·N‖): consistent 2e-16 → estimate-point 0.15 at σ=2 cm,
// rising monotonically with the linearization error.

#include <branes/sdk/eval/observability_probe.hpp>

#include <catch2/catch_test_macros.hpp>

namespace {
namespace ev = branes::sdk::eval;
using T = double;
}  // namespace

TEST_CASE("observability: the camera Jacobian annihilates the 4-DoF gauge when consistently linearized",
          "[sdk][observability][s6]") {
    const auto p = ev::observability_probe<T>();
    // A single, honest linearization point ⇒ H·N ≈ 0 for both gauge directions:
    // the measurement model is gauge-correct (validates the analytic null space).
    REQUIRE(p.trans_leak_consistent < 1e-10);
    REQUIRE(p.yaw_leak_consistent < 1e-10);
}

TEST_CASE("observability: estimate-point linearization leaks information into YAW (the #212 mechanism)",
          "[sdk][observability][s6]") {
    const auto p = ev::observability_probe<T>();
    // Translation is preserved even at the perturbed estimate (the −Hf / +Hf
    // columns cancel for any single H), so it is NOT the leak.
    REQUIRE(p.trans_leak_inconsistent < 1e-10);
    // YAW is not: linearizing at the (wrong) estimate fabricates information about
    // rotation-about-gravity — the filter becomes over-confident in attitude.
    REQUIRE(p.yaw_leak_inconsistent > 1e-3);
    REQUIRE(p.yaw_leak_inconsistent > 1e6 * p.yaw_leak_consistent);
}

TEST_CASE("observability: the yaw leak grows with the linearization error (FEJ acceptance curve)",
          "[sdk][observability][s6]") {
    const auto p = ev::observability_probe<T>();
    REQUIRE(p.sweep.size() >= 4);
    // σ = 0 is the consistent case ⇒ ~0; the leak rises monotonically with the
    // estimate error. A correct FEJ / OC-EKF fix must drive the whole curve to ~0.
    REQUIRE(p.sweep.front().first == T{0});
    REQUIRE(p.sweep.front().second < 1e-10);
    for (std::size_t i = 1; i < p.sweep.size(); ++i)
        REQUIRE(p.sweep[i].second > p.sweep[i - 1].second);
}
