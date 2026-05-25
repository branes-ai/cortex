// IMU initialization tests (issue #42).
//
// Static init: a stationary, tilted IMU with a gyro bias — checks that
// the recovered gravity direction is within 0.5° of truth (the EuRoC
// acceptance bar) and that the gyro bias is recovered. Dynamic init: a
// synthetic moving trajectory whose preintegration deltas are built to be
// exactly consistent with a known gravity, velocities, and gyro bias —
// checks that the linear alignment recovers all three.

#include <branes/sdk/imu_init.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace {

namespace bs = branes::sdk;
using T = double;
using Vec3 = bs::ImuInitializer<T>::Vec3;
using SO3 = bs::ImuInitializer<T>::SO3;

// Angle (radians) between two non-zero vectors.
T angle_between(const Vec3& a, const Vec3& b) {
    const T num = branes::math::lie::detail::dot(a, b);
    const T den = branes::math::lie::detail::norm(a) * branes::math::lie::detail::norm(b);
    T c = num / den;
    c = std::max(T{-1}, std::min(T{1}, c));
    return std::acos(c);
}

}  // namespace

TEST_CASE("static init recovers gravity within 0.5 deg and the gyro bias", "[sdk][imu_init]") {
    // Truth: tilted ~25° about x, ~10° about y, with a gyro bias.
    const SO3 R_true = SO3::exp(Vec3{{0.43, 0.17, 0.0}});
    const Vec3 g_world{{0.0, 0.0, -9.81}};
    const Vec3 bias{{0.01, -0.005, 0.008}};

    // Accelerometer at rest reads Rᵀ(−g) = "up" in the body frame.
    const Vec3 up_world{{0.0, 0.0, 9.81}};
    const Vec3 a_m = R_true.inverse() * up_world;

    std::vector<Vec3> gyro, accel;
    for (int k = 0; k < 200; ++k) {
        // Tiny deterministic dither so the std is non-zero but sub-gate.
        const T n = 1e-4 * std::sin(0.3 * k);
        gyro.push_back(Vec3{{bias[0] + n, bias[1] - n, bias[2] + 0.5 * n}});
        accel.push_back(Vec3{{a_m[0] + n, a_m[1] + n, a_m[2] - n}});
    }

    bs::ImuInitializer<T> init;
    const auto r = init.try_static(gyro, accel);
    REQUIRE(r.success);

    // Gravity-in-body from the estimate vs. truth.
    const Vec3 g_body_true = R_true.inverse() * g_world;
    const Vec3 g_body_est = r.R_world_imu.inverse() * r.gravity_world;
    const T deg = angle_between(g_body_true, g_body_est) * 180.0 / M_PI;
    REQUIRE(deg < 0.5);

    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(r.gyro_bias[i], Catch::Matchers::WithinAbs(bias[i], 2e-4));
}

TEST_CASE("static init rejects a non-stationary window", "[sdk][imu_init]") {
    std::vector<Vec3> gyro, accel;
    for (int k = 0; k < 100; ++k) {
        // Large swings — clearly moving, not at rest.
        gyro.push_back(Vec3{{0.5 * std::sin(0.2 * k), 0.0, 0.0}});
        accel.push_back(Vec3{{2.0 * std::cos(0.2 * k), 0.0, 9.81}});
    }
    bs::ImuInitializer<T> init;
    REQUIRE_FALSE(init.try_static(gyro, accel).success);
}

TEST_CASE("dynamic init recovers gravity, velocities, and gyro bias", "[sdk][imu_init]") {
    using KF = bs::DynInitKeyframe<T>;
    using Mat3 = bs::ImuInitializer<T>::Mat3;

    const Vec3 g_world{{0.0, 0.0, -9.81}};
    const Vec3 bg_true{{0.004, -0.002, 0.006}};
    const T dt = 0.1;
    const std::size_t n = 6;

    // A known moving trajectory: per-keyframe world pose and velocity.
    std::vector<SO3> R(n);
    std::vector<Vec3> p(n), v(n);
    for (std::size_t i = 0; i < n; ++i) {
        const T s = static_cast<T>(i);
        R[i] = SO3::exp(Vec3{{0.05 * s, -0.03 * s, 0.02 * s}});
        p[i] = Vec3{{0.4 * s, 0.1 * s * s, -0.05 * s}};
        v[i] = Vec3{{0.4 + 0.1 * s, 0.2 * s, 0.05}};
    }

    // Build preintegration deltas exactly consistent with truth, so the
    // alignment has the truth as its exact solution.
    std::vector<KF> kfs(n);
    kfs[0].R_world_imu = R[0];
    kfs[0].p_world_imu = p[0];
    for (std::size_t i = 1; i < n; ++i) {
        const Mat3 Rit = R[i - 1].inverse().matrix();
        kfs[i].R_world_imu = R[i];
        kfs[i].p_world_imu = p[i];
        kfs[i].dt = dt;
        kfs[i].dR_dbg = Mat3::identity();
        // ΔR carries a −bias twist so the GN step recovers bg_true.
        kfs[i].dR = (R[i - 1].inverse() * R[i]) * SO3::exp(bg_true * (-T{1}));
        kfs[i].dv = Rit * (v[i] - v[i - 1] + g_world * dt);
        kfs[i].dp = Rit * (p[i] - p[i - 1] - v[i - 1] * dt + g_world * (T{0.5} * dt * dt));
    }

    bs::ImuInitializer<T> init;
    const auto r = init.try_dynamic(kfs);
    REQUIRE(r.success);

    REQUIRE(angle_between(r.gravity_world, g_world) * 180.0 / M_PI < 0.5);
    REQUIRE_THAT(branes::math::lie::detail::norm(r.gravity_world), Catch::Matchers::WithinAbs(9.81, 1e-3));
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(r.gyro_bias[i], Catch::Matchers::WithinAbs(bg_true[i], 1e-6));

    REQUIRE(r.velocities_world.size() == n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            REQUIRE_THAT(r.velocities_world[i][j], Catch::Matchers::WithinAbs(v[i][j], 1e-3));
}

TEST_CASE("dynamic init fails with too few keyframes", "[sdk][imu_init]") {
    std::vector<bs::DynInitKeyframe<T>> one(1);
    bs::ImuInitializer<T> init;
    REQUIRE_FALSE(init.try_dynamic(one).success);
}
