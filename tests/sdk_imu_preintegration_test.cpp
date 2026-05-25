// IMU preintegration tests (issue #40).
//
// Acceptance: a synthetic noiseless IMU stream over a known trajectory
// recovers ΔR/Δv/Δp to numerical precision. We forward-integrate a
// reference nav-state with the same per-step convention the
// preintegrator implies, then check that preintegrate→predict reproduces
// it. Also verifies the bias-correction Jacobians against finite
// differences and the zero-motion identity.

#include <branes/sdk/imu_preintegration.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

namespace lie = branes::math::lie;
using T = double;
using Vec3 = lie::detail::Vec<T, 3>;
using SO3 = lie::SO3<T>;

struct Sample {
    Vec3 gyro, accel;
};

// A deterministic, time-varying noiseless IMU stream.
std::vector<Sample> make_stream(int n, T dt) {
    std::vector<Sample> s;
    for (int k = 0; k < n; ++k) {
        const T t = k * dt;
        s.push_back({Vec3{{0.1 * std::sin(0.7 * t), 0.05, -0.12 * std::cos(0.4 * t)}},
                     Vec3{{0.3, -0.2 + 0.05 * t, 9.6 * std::cos(0.2 * t)}}});
    }
    return s;
}

void require_vec3_close(const Vec3& a, const Vec3& b, double tol) {
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE(std::abs(a[i] - b[i]) < tol);
}

}  // namespace

TEST_CASE("preintegration predicts the forward-integrated state exactly", "[sdk][imu]") {
    const T dt = 0.004;
    const int n = 250;  // 1 second
    const auto stream = make_stream(n, dt);
    const Vec3 g{{0.0, 0.0, -9.81}};

    // Reference forward integration (same per-step convention).
    SO3 R = SO3::exp(Vec3{{0.2, -0.1, 0.3}});
    Vec3 v{{0.5, 0.0, -0.2}};
    Vec3 p{{1.0, 2.0, 3.0}};
    const SO3 R0 = R;
    const Vec3 v0 = v, p0 = p;
    for (const auto& s : stream) {
        const Vec3 Ra = R.matrix() * s.accel;
        p = p + v * dt + Ra * (0.5 * dt * dt) + g * (0.5 * dt * dt);
        v = v + Ra * dt + g * dt;
        R = R * SO3::exp(s.gyro * dt);
    }

    // Preintegrate then predict from the initial state.
    branes::sdk::ImuPreintegrator<T> pre;
    for (const auto& s : stream)
        pre.integrate(s.gyro, s.accel, dt);
    SO3 Rp;
    Vec3 vp, pp;
    pre.predict(R0, v0, p0, g, Rp, vp, pp);

    REQUIRE(std::abs(pre.delta_time() - n * dt) < 1e-9);
    // Rotation: compare via the relative-rotation log (≈ 0).
    require_vec3_close((Rp.inverse() * R).log(), Vec3{{0, 0, 0}}, 1e-9);
    require_vec3_close(vp, v, 1e-9);
    require_vec3_close(pp, p, 1e-8);
}

TEST_CASE("zero motion gives identity preintegration", "[sdk][imu]") {
    branes::sdk::ImuPreintegrator<T> pre;
    for (int k = 0; k < 50; ++k)
        pre.integrate(Vec3{{0, 0, 0}}, Vec3{{0, 0, 0}}, 0.01);
    require_vec3_close(pre.delta_rotation().log(), Vec3{{0, 0, 0}}, 1e-12);
    require_vec3_close(pre.delta_velocity(), Vec3{{0, 0, 0}}, 1e-12);
    require_vec3_close(pre.delta_position(), Vec3{{0, 0, 0}}, 1e-12);
    REQUIRE(std::abs(pre.delta_time() - 0.5) < 1e-9);
}

TEST_CASE("integrate ignores non-positive dt", "[sdk][imu]") {
    branes::sdk::ImuPreintegrator<T> pre;
    pre.integrate(Vec3{{0.1, 0.2, 0.3}}, Vec3{{1, 2, 3}}, 0.0);    // no-op
    pre.integrate(Vec3{{0.1, 0.2, 0.3}}, Vec3{{1, 2, 3}}, -0.01);  // no-op
    REQUIRE(pre.delta_time() == 0.0);
    require_vec3_close(pre.delta_rotation().log(), Vec3{{0, 0, 0}}, 1e-12);
    require_vec3_close(pre.delta_velocity(), Vec3{{0, 0, 0}}, 1e-12);
    pre.integrate(Vec3{{0.1, 0.2, 0.3}}, Vec3{{1, 2, 3}}, 0.01);  // applied
    REQUIRE(std::abs(pre.delta_time() - 0.01) < 1e-12);
}

TEST_CASE("bias-correction Jacobians match finite differences", "[sdk][imu]") {
    const T dt = 0.005;
    const int n = 120;
    const auto stream = make_stream(n, dt);

    branes::sdk::ImuPreintegrator<T> base;  // linearized at zero bias
    for (const auto& s : stream)
        base.integrate(s.gyro, s.accel, dt);

    const Vec3 dbg{{1e-3, -8e-4, 6e-4}};
    const Vec3 dba{{-7e-4, 5e-4, 9e-4}};

    // Re-integrate with the perturbed bias as the ground truth.
    branes::sdk::ImuPreintegrator<T> pert(dbg, dba);
    for (const auto& s : stream)
        pert.integrate(s.gyro, s.accel, dt);

    // First-order correction from base should match the perturbed run.
    require_vec3_close(
        (pert.delta_rotation().inverse() * base.corrected_delta_rotation(dbg)).log(), Vec3{{0, 0, 0}}, 1e-5);
    require_vec3_close(base.corrected_delta_velocity(dbg, dba), pert.delta_velocity(), 1e-4);
    require_vec3_close(base.corrected_delta_position(dbg, dba), pert.delta_position(), 1e-4);
}
