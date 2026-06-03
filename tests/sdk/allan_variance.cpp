// Allan variance / deviation for IMU noise characterization (issue #268).
//
// Verified against an independent oracle — the known Allan signatures of
// synthetic noise processes (white noise has σ_A(τ)=N/√τ, a −1/2 log-log slope;
// a random walk has a +1/2 slope) — not against the implementation's own
// assumptions. The dataset-gated case reports the EuRoC IMU's measured noise
// densities against the filter's configured Q (the #212 first probe), skipped in
// CI.

#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/eval/allan_variance.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <random>
#include <span>
#include <string>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
namespace bs = branes::sdk;
using T = double;

}  // namespace

TEST_CASE("Allan deviation is ~zero for a constant series", "[sdk][eval][allan]") {
    std::vector<T> x(1000, 5.0);
    REQUIRE_THAT(ev::allan_deviation_at<T>(x, 1), Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(ev::allan_deviation_at<T>(x, 8), Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THROWS_AS(ev::allan_deviation_at<T>(x, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(ev::white_noise_density<T>(x, -1.0), std::invalid_argument);
}

// NOTE: keep TEST_CASE names ASCII-only — Unicode in a case name survives Linux
// ctest but the MSVC console codepage mangles it (σ→s, √τ→vt), so ctest's
// name filter matches nothing and the test reports as failed.
TEST_CASE("white noise has the N/sqrt(tau) (minus-half slope) signature", "[sdk][eval][allan]") {
    const T sigma_w = 0.01;  // per-sample standard deviation
    const T dt = 0.005;      // 200 Hz, EuRoC-like
    std::mt19937 gen(20240602u);
    std::normal_distribution<T> nrm(0.0, sigma_w);
    std::vector<T> x(200000);
    for (auto& v : x)
        v = nrm(gen);

    // At m=1 the Allan deviation equals the per-sample std.
    REQUIRE_THAT(ev::allan_deviation_at<T>(x, 1), Catch::Matchers::WithinRel(sigma_w, 0.02));
    // −1/2 slope: quadrupling the averaging factor halves σ_A.
    const T s1 = ev::allan_deviation_at<T>(x, 1);
    const T s4 = ev::allan_deviation_at<T>(x, 4);
    REQUIRE_THAT(s4 / s1, Catch::Matchers::WithinAbs(0.5, 0.03));
    // The recovered noise density N = σ_w·√dt (the −1/2 asymptote at τ=1 s).
    REQUIRE_THAT(ev::white_noise_density<T>(x, dt), Catch::Matchers::WithinRel(sigma_w * std::sqrt(dt), 0.03));
}

TEST_CASE("a random walk has the +1/2 slope signature", "[sdk][eval][allan]") {
    const T step = 0.01;
    std::mt19937 gen(777u);
    std::normal_distribution<T> nrm(0.0, step);
    std::vector<T> rw(200000);
    T acc = 0.0;
    for (auto& v : rw) {
        acc += nrm(gen);
        v = acc;
    }
    // +1/2 slope: quadrupling τ doubles σ_A (σ_A ∝ √τ).
    const T s2 = ev::allan_deviation_at<T>(rw, 2);
    const T s8 = ev::allan_deviation_at<T>(rw, 8);
    REQUIRE(s8 > s2);  // grows with τ (opposite of white noise)
    REQUIRE_THAT(s8 / s2, Catch::Matchers::WithinAbs(2.0, 0.4));
}

TEST_CASE("octave_taus spans the series", "[sdk][eval][allan]") {
    const auto taus = ev::octave_taus<T>(1000, 0.01);
    REQUIRE(taus.front() == 0.01);  // m=1
    REQUIRE(taus.size() == 9);      // m = 1,2,…,256 (512 would leave <2 bins)
    REQUIRE(taus.back() == 0.01 * 256.0);
}

// Dataset-gated: characterize the real EuRoC IMU and compare to the filter's Q.
TEST_CASE("EuRoC IMU noise density vs the filter's configured Q", "[sdk][eval][allan][dataset]") {
    const char* env = std::getenv("CORTEX_EUROC_V101");
    if (env == nullptr || std::string(env).empty())
        SKIP("set CORTEX_EUROC_V101 to the V1_01_easy/mav0 directory to characterize its IMU");

    const auto imu = bs::euroc::parse_imu<T>(std::string(env));
    REQUIRE(imu.size() > 1000);
    const T dt = (imu.back().timestamp_s - imu.front().timestamp_s) / static_cast<T>(imu.size() - 1);

    auto axis_density = [&](auto pick) {
        std::vector<T> a(imu.size());
        for (std::size_t i = 0; i < imu.size(); ++i)
            a[i] = pick(imu[i]);
        return ev::white_noise_density<T>(a, dt);
    };
    T gyro_n = 0, accel_n = 0;
    for (std::size_t k = 0; k < 3; ++k) {
        gyro_n += axis_density([k](const auto& m) { return m.angular_velocity[k]; });
        accel_n += axis_density([k](const auto& m) { return m.linear_acceleration[k]; });
    }
    gyro_n /= 3.0;
    accel_n /= 3.0;

    const bs::VioConfig cfg;  // the filter's configured noise densities
    // CAVEAT: this runs on a FLIGHT sequence, so the small-tau Allan deviation is
    // contaminated by the platform's high-frequency motion (sigma_A^2 ~= sigma_w^2
    // + 0.5*mean((alpha*dt)^2)). The reported N is therefore an UPPER BOUND on the
    // true sensor white-noise density, badly inflated on the aggressive
    // sequences. A clean measurement of Q needs a long STATIC IMU log; EuRoC's
    // published static value (gyro ~1.7e-4) already matches cfg.gyro_noise_density,
    // so treat N>>Q here as "needs a static log to confirm", not "retune Q".
    WARN("EuRoC IMU @ " << 1.0 / dt << " Hz (FLIGHT, motion-contaminated — upper bound): gyro N=" << gyro_n
                        << " rad/s/sqrt(Hz) (filter Q " << cfg.gyro_noise_density << "), accel N=" << accel_n
                        << " m/s^2/sqrt(Hz) (filter Q " << cfg.accel_noise_density << ")");
    // Sanity only (the measurement runs on a flight sequence, so the white-noise
    // density is the meaningful number; bias terms need a static log): the
    // densities are positive and within a few orders of magnitude of the config.
    REQUIRE(gyro_n > 0.0);
    REQUIRE(accel_n > 0.0);
    REQUIRE(gyro_n < 100.0 * cfg.gyro_noise_density);
    REQUIRE(accel_n < 100.0 * cfg.accel_noise_density);
}
