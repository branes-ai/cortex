// S2 IMU-propagation probe (docs/arch/vio-pipeline-canonical.md).
//
// Locks the S2 contract findings against independent oracles: the Φ
// reconstruction is validated against the shipped Propagator's own covariance;
// the mean stays on SO(3); the covariance stays PSD; the analytic
// global-position nullspace is preserved; the GT-injection error converges as
// O(dt) (coarse-vs-fine reference); and the propagation-only NEES tracks the
// 1/Q lever with NEES≈dof at the cortex default noise.
//
// The headline #212 reading these assertions pin: the diagonal-Q omission costs
// only single-digit-% position σ inter-frame, and propagation-only NEES is ≈
// consistent at the default Q — so the over-confidence is not an S2 fault.

#include <branes/sdk/eval/propagation_probe.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace {
namespace ev = branes::sdk::eval;
using T = double;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
const branes::sdk::msckf::ImuNoise<T> kNoise{};
const Vec3 kGyro{{0.2, -0.1, 0.3}};
const Vec3 kAccel{{0.5, -0.3, 9.81}};
}  // namespace

TEST_CASE("S2 cortex diagonal Q drops the canonical position-block & v-p terms", "[sdk][s2][propagation]") {
    const auto q = ev::q_structure(kNoise, 1.0 / 200.0);
    // θ and v blocks match the canonical leading order.
    REQUIRE_THAT(q.theta_sigma_cortex, Catch::Matchers::WithinRel(q.theta_sigma_canon, 1e-12));
    REQUIRE_THAT(q.vel_sigma_cortex, Catch::Matchers::WithinRel(q.vel_sigma_canon, 1e-12));
    // The position block the diagonal injection drops is real but small.
    REQUIRE(q.pos_sigma_cortex == 0.0);
    REQUIRE(q.pos_sigma_canon > 0.0);
    REQUIRE(q.vp_cross_canon > 0.0);
    REQUIRE(q.rel_frobenius_gap < 0.05);  // <5% of Q dropped — minor
}

TEST_CASE("S2 propagation keeps the mean on SO(3), the covariance PSD, and the probe's "
          "Phi matches the shipped Propagator",
          "[sdk][s2][propagation]") {
    const auto g = ev::cov_growth(kNoise, kGyro, kAccel);
    REQUIRE(!g.curve.empty());
    // The reference Φ·P·Φᵀ+Q with cortex-Q reproduces the real Propagator's P.
    REQUIRE(g.F_validation_residual < 1e-10);
    for (const auto& p : g.curve) {
        REQUIRE(p.R_ortho_residual < 1e-10);  // R stays orthonormal
        REQUIRE(p.min_eig_cortex >= -1e-9);   // P stays PSD
    }
    // The diagonal-Q deficit is a single-digit-% effect, not orders of magnitude.
    REQUIRE(g.pos_underreport_pct_interframe > 0.0);
    REQUIRE(g.pos_underreport_pct_interframe < 25.0);
}

TEST_CASE("S2 mean integration converges as O(dt) against a fine reference", "[sdk][s2][propagation]") {
    const auto sweep = ev::gt_injection_dt_sweep<T>();
    REQUIRE(sweep.size() >= 2);
    // Error must shrink monotonically as dt shrinks, ~halving each time (O(dt)).
    for (std::size_t i = 1; i < sweep.size(); ++i)
        REQUIRE(sweep[i].pos_error_mm < sweep[i - 1].pos_error_mm);
    const T ratio = sweep[sweep.size() - 2].pos_error_mm / sweep.back().pos_error_mm;
    REQUIRE(ratio > 1.8);
    REQUIRE(ratio < 2.2);
}

TEST_CASE("S2 propagation-only NEES tracks the 1/Q lever and is ~consistent at default Q", "[sdk][s2][propagation]") {
    const auto n = ev::nees_vs_qscale(kNoise);
    REQUIRE(n.curve.size() == 5);
    // NEES monotonically decreases as Q grows (more process noise ⇒ less confident).
    for (std::size_t i = 1; i < n.curve.size(); ++i)
        REQUIRE(n.curve[i].nees_pose < n.curve[i - 1].nees_pose);
    // At the cortex default noise (q_scale = 1), NEES is near the 6-DoF target —
    // propagation alone is approximately consistent, so #212 is not an S2 fault.
    T nees_at_1 = 0;
    for (const auto& p : n.curve)
        if (p.q_scale == 1.0)
            nees_at_1 = p.nees_pose;
    REQUIRE(nees_at_1 > 4.0);
    REQUIRE(nees_at_1 < 9.0);
}

TEST_CASE("S2 propagation preserves the global-position nullspace", "[sdk][s2][propagation]") {
    const auto ns = ev::nullspace_position(kGyro, kAccel);
    REQUIRE(ns.position_leak < 1e-12);
}
