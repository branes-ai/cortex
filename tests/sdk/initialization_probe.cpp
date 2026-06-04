// S1 initialization probe (docs/arch/vio-pipeline-canonical.md).
//
// Locks the S1 contract findings against independent oracles: static init
// levels a synthetic tilted gravity to machine precision and recovers |g| and
// the gyro bias; the stationarity gate rejects an over-noisy window; dynamic
// init recovers the injected metric scale under excitation and the
// observability gate DECLINES (rather than guessing) as excitation drops below
// the floor; and the filter's initial covariance is reported as the isotropic
// σ·I it actually is.

#include <branes/sdk/eval/initialization_probe.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace {
namespace ev = branes::sdk::eval;
using T = double;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
}  // namespace

TEST_CASE("S1 static init levels gravity and recovers |g| and gyro bias", "[sdk][s1][init]") {
    const auto r = ev::static_init<T>(/*tilt_deg=*/8.0, Vec3{{0.01, -0.005, 0.008}});
    REQUIRE(r.success);
    // The recovered frame levels the measured gravity to (numerical) zero.
    REQUIRE(r.gravity_residual_ms2 < 1e-9);
    // |g| is sane and within the static gate's 5% gravity tolerance.
    REQUIRE(r.recovered_g_ms2 > 9.3);
    REQUIRE(r.recovered_g_ms2 < 10.3);
    // Sub-degree leveling and small gyro-bias error at the default noise.
    REQUIRE(r.rollpitch_error_deg < 0.2);
    REQUIRE(r.gyro_bias_error < 5e-3);
}

TEST_CASE("S1 stationarity gate rejects an over-noisy 'static' window", "[sdk][s1][init]") {
    const auto sweep = ev::static_noise_sweep<T>();
    REQUIRE(sweep.size() >= 4);
    // Leveling error grows with accel noise across the accepted windows.
    T prev = -1;
    bool saw_reject = false;
    for (const auto& p : sweep) {
        if (p.success) {
            REQUIRE(p.rollpitch_error_deg >= prev);  // monotone non-decreasing
            prev = p.rollpitch_error_deg;
        } else {
            saw_reject = true;  // the gate refuses the noisiest window
        }
    }
    REQUIRE(saw_reject);
}

TEST_CASE("S1 dynamic init recovers the metric scale under excitation", "[sdk][s1][init]") {
    const auto r = ev::dynamic_init<T>(/*excitation=*/1.5, /*scale_true=*/2.0);
    REQUIRE(r.success);
    REQUIRE(r.scale_error_pct < 1.0);        // metric scale recovered to <1%
    REQUIRE(r.gravity_dir_error_deg < 1.0);  // gravity direction recovered
    REQUIRE(r.gravity_mag_error_ms2 < 1e-3);
}

TEST_CASE("S1 scale-observability gate declines at low excitation (the cliff)", "[sdk][s1][init]") {
    const auto sweep = ev::dynamic_excitation_sweep<T>();
    REQUIRE(sweep.size() >= 4);
    // Lowest excitation: resolved motion below the floor ⇒ the gate declines.
    REQUIRE_FALSE(sweep.front().success);
    REQUIRE(sweep.front().resolved_motion_m < 0.05);
    // Highest excitation: well-observed ⇒ accept with a small scale error.
    REQUIRE(sweep.back().success);
    REQUIRE(sweep.back().resolved_motion_m > 0.05);
    REQUIRE(sweep.back().scale_error_pct < 1.0);
}

TEST_CASE("S1 reports the initial covariance as the isotropic seed it is", "[sdk][s1][init]") {
    const auto p = ev::initial_p_sizing<T>();
    REQUIRE(p.isotropic);
    REQUIRE_THAT(p.sigma, Catch::Matchers::WithinRel(0.1, 1e-9));
    // 0.1 rad on yaw ≈ 5.73°, identical to the (well-observed) roll/pitch seed —
    // i.e. the seed is NOT enlarged on the unobservable direction.
    REQUIRE_THAT(p.yaw_sigma_deg, Catch::Matchers::WithinAbs(5.7296, 1e-2));
    REQUIRE_THAT(p.yaw_sigma_deg, Catch::Matchers::WithinRel(p.rollpitch_sigma_deg, 1e-12));
}
