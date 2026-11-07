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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
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
    const T deg = angle_between(g_body_true, g_body_est) * T(180) / std::numbers::pi_v<T>;
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

namespace {

// A still, tilted platform with rotor vibration (#591), over the backend's
// 50-sample static window: the gyro reads the bias plus two high-frequency
// tones (0.43 and 0.29 cycles/sample, rotor harmonics aliased at 200 Hz), a
// per-sample std of ~0.03 rad/s around a zero mean rate. The accelerometer
// vibrates too, within the static gate. `slow` replaces the tones with a rate
// swing of the same size slow enough to be real motion (0.01 cycles/sample).
struct VibratingStart {
    Vec3 bias{{-0.003, 0.021, 0.077}};  // EuRoC-like, as measured on MH_03
    SO3 R_true = SO3::exp(Vec3{{0.43, 0.17, 0.0}});
    std::vector<Vec3> gyro, accel;

    explicit VibratingStart(bool slow = false, T accel_scale = 1.0, std::size_t n = 50) {
        const Vec3 a_rest = R_true.inverse() * Vec3{{0.0, 0.0, 9.81 * accel_scale}};
        const T two_pi = 2.0 * std::numbers::pi_v<T>;
        for (std::size_t k = 0; k < n; ++k) {
            const T x = static_cast<T>(k);
            Vec3 w;
            for (std::size_t i = 0; i < 3; ++i) {
                const T ph = 1.3 * static_cast<T>(i);
                w[i] = slow ? 0.03 * std::sin(two_pi * 0.01 * x + ph)
                            : 0.04 * std::sin(two_pi * 0.43 * x + ph) + 0.02 * std::sin(two_pi * 0.29 * x + 2.0 * ph);
            }
            gyro.push_back(bias + w);
            const T j = 0.3 * std::sin(two_pi * 0.37 * x);
            accel.push_back(Vec3{{a_rest[0] + j, a_rest[1] - j, a_rest[2] + 0.5 * j}});
        }
    }
};

// The per-sample gyro std, max over axes: what the static gate tests.
T per_sample_std(const std::vector<Vec3>& v) {
    T worst = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
        T m = 0.0, q = 0.0;
        for (const auto& x : v)
            m += x[i];
        m /= static_cast<T>(v.size());
        for (const auto& x : v)
            q += (x[i] - m) * (x[i] - m);
        worst = std::max(worst, std::sqrt(q / static_cast<T>(v.size())));
    }
    return worst;
}

}  // namespace

TEST_CASE("a vibrating start at rest yields its gyro mean as the bias (#591)", "[sdk][imu_init]") {
    const VibratingStart w;
    REQUIRE(per_sample_std(w.gyro) > 0.025);
    bs::ImuInitializer<T> init;

    // try_static keeps the per-sample test, so it rejects the window...
    REQUIRE_FALSE(init.try_static(w.gyro, w.accel).success);
    // ...but the low-passed test sees a constant rate, and the mean is the bias.
    const auto b = init.vibrating_rest_bias(w.gyro, w.accel);
    REQUIRE(b.has_value());
    REQUIRE(b->rate_variation < 0.01);
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(b->gyro_bias[i], Catch::Matchers::WithinAbs(w.bias[i], 2e-3));

    // With the low-pass off it is the per-sample test, and the window fails.
    bs::ImuInitConfig<T> raw;
    raw.gyro_lowpass_samples = 1;
    REQUIRE_FALSE(bs::ImuInitializer<T>{raw}.vibrating_rest_bias(w.gyro, w.accel).has_value());
}

TEST_CASE("a slow rate change as large as the vibration is not at rest", "[sdk][imu_init]") {
    const VibratingStart w(/*slow=*/true);
    REQUIRE(per_sample_std(w.gyro) < 0.025);  // smaller per-sample spread than the vibration
    bs::ImuInitializer<T> init;
    REQUIRE_FALSE(init.vibrating_rest_bias(w.gyro, w.accel).has_value());
    REQUIRE_FALSE(init.try_static(w.gyro, w.accel).success);
}

TEST_CASE("vibrating_rest_bias keeps the static gravity gate", "[sdk][imu_init]") {
    const VibratingStart w(/*slow=*/false, /*accel_scale=*/1.08);  // |a| 8 % off: not at rest
    REQUIRE_FALSE(bs::ImuInitializer<T>{}.vibrating_rest_bias(w.gyro, w.accel).has_value());
}

TEST_CASE("gravity-align seeds a rest bias instead of zero (#591)", "[sdk][imu_init]") {
    // The timeout window itself is turning slowly, so its own mean is rate,
    // not bias: without a hint the bias stays zero, as before.
    const VibratingStart turning(/*slow=*/true, /*accel_scale=*/1.08);
    bs::ImuInitializer<T> init;
    const auto plain = init.try_gravity_align(turning.gyro, turning.accel);
    REQUIRE(plain.success);
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE(plain.gyro_bias[i] == T{0});

    // With the bias from an earlier window at rest, that bias is seeded.
    const Vec3 rest{{-0.003, 0.021, 0.077}};
    const auto hinted = init.try_gravity_align(turning.gyro, turning.accel, rest);
    REQUIRE(hinted.success);
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE(hinted.gyro_bias[i] == rest[i]);

    // A vibrating window that is itself at rest seeds its own mean.
    const VibratingStart still(/*slow=*/false, /*accel_scale=*/1.08);
    const auto own = init.try_gravity_align(still.gyro, still.accel);
    REQUIRE(own.success);
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(own.gyro_bias[i], Catch::Matchers::WithinAbs(still.bias[i], 2e-3));
}

TEST_CASE("gravity-align recovers roll/pitch on a moving start static init rejects", "[sdk][imu_init]") {
    // A genuinely tilted platform that is also rotating + jostling: the mean
    // specific force is still gravity-dominated, so roll/pitch is recoverable
    // even though the window is far from at rest.
    const SO3 R_true = SO3::exp(Vec3{{0.43, 0.17, 0.0}});  // ~25° / ~10° tilt
    const Vec3 up_world{{0.0, 0.0, 9.81}};
    const Vec3 a_rest = R_true.inverse() * up_world;
    const Vec3 g_world{{0.0, 0.0, -9.81}};

    std::vector<Vec3> gyro, accel;
    for (int k = 0; k < 120; ++k) {
        // Real angular rate (~0.2 rad/s swings) and ±0.4 m/s² translational
        // jostle on top of the gravity term — clearly not stationary.
        gyro.push_back(Vec3{{0.2 * std::sin(0.2 * k), 0.15 * std::cos(0.13 * k), 0.1 * std::sin(0.07 * k)}});
        const T j = 0.4 * std::sin(0.3 * k);
        accel.push_back(Vec3{{a_rest[0] + j, a_rest[1] - j, a_rest[2] + 0.5 * j}});
    }

    bs::ImuInitializer<T> init;
    REQUIRE_FALSE(init.try_static(gyro, accel).success);  // too lively for static

    const auto r = init.try_gravity_align(gyro, accel);
    REQUIRE(r.success);
    // Roll/pitch should be close to truth (the jostle adds a few degrees);
    // yaw is unobservable, so compare the recovered gravity directions.
    const Vec3 g_body_true = R_true.inverse() * g_world;
    const Vec3 g_body_est = r.R_world_imu.inverse() * r.gravity_world;
    const T deg = angle_between(g_body_true, g_body_est) * T(180) / std::numbers::pi_v<T>;
    REQUIRE(deg < 5.0);
    // The window is rotating, so the gyro mean is NOT trusted as bias.
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE(r.gyro_bias[i] == T{0});
}

TEST_CASE("gravity-align rejects a window whose mean force isn't gravity", "[sdk][imu_init]") {
    // Sustained free-fall-ish / high-acceleration: mean specific force is far
    // from g, so its direction is meaningless and alignment must decline.
    std::vector<Vec3> gyro, accel;
    for (int k = 0; k < 80; ++k) {
        gyro.push_back(Vec3{{0.0, 0.0, 0.0}});
        accel.push_back(Vec3{{0.0, 0.0, 2.0}});  // |mean| ≈ 2 m/s², nowhere near 9.81
    }
    bs::ImuInitializer<T> init;
    REQUIRE_FALSE(init.try_gravity_align(gyro, accel).success);
}

TEST_CASE("dynamic init recovers gravity, velocities, and gyro bias", "[sdk][imu_init]") {
    using KF = bs::DynInitKeyframe<T>;
    using Mat3 = bs::ImuInitializer<T>::Mat3;

    const Vec3 g_world{{0.0, 0.0, -9.81}};
    const Vec3 bg_true{{0.004, -0.002, 0.006}};
    const T dt = 0.1;
    const std::size_t n = 8;

    // A known, *well-excited* moving trajectory (varied rotation and a curved
    // translation) so gravity, scale, and the velocities are all observable.
    std::vector<SO3> R(n);
    std::vector<Vec3> p(n), v(n);
    for (std::size_t i = 0; i < n; ++i) {
        const T s = static_cast<T>(i);
        R[i] = SO3::exp(Vec3{{0.3 * std::sin(0.7 * s), 0.25 * std::cos(0.5 * s), 0.2 * s}});
        p[i] = Vec3{{0.5 * s + 0.3 * std::sin(s), 0.08 * s * s, 0.2 * std::cos(0.6 * s) + 0.1 * s}};
        v[i] = Vec3{{0.5 + 0.2 * std::cos(s), 0.16 * s, -0.12 * std::sin(0.6 * s) + 0.1}};
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
        kfs[i].dv = Rit * (v[i] - v[i - 1] - g_world * dt);
        kfs[i].dp = Rit * (p[i] - p[i - 1] - v[i - 1] * dt - g_world * (T{0.5} * dt * dt));
    }

    bs::ImuInitializer<T> init;
    const auto r = init.try_dynamic(kfs);
    REQUIRE(r.success);

    REQUIRE(angle_between(r.gravity_world, g_world) * T(180) / std::numbers::pi_v<T> < 0.5);
    REQUIRE_THAT(branes::math::lie::detail::norm(r.gravity_world), Catch::Matchers::WithinAbs(9.81, 1e-3));
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(r.gyro_bias[i], Catch::Matchers::WithinAbs(bg_true[i], 1e-6));

    // The keyframe positions here are already metric, so the recovered scale is 1.
    REQUIRE_THAT(r.scale, Catch::Matchers::WithinAbs(1.0, 1e-3));

    REQUIRE(r.velocities_world.size() == n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            REQUIRE_THAT(r.velocities_world[i][j], Catch::Matchers::WithinAbs(v[i][j], 1e-3));
}

TEST_CASE("dynamic init recovers metric scale from up-to-scale vision poses", "[sdk][imu_init]") {
    using KF = bs::DynInitKeyframe<T>;
    using Mat3 = bs::ImuInitializer<T>::Mat3;

    const Vec3 g_world{{0.0, 0.0, -9.81}};
    const T dt = 0.1;
    const std::size_t n = 8;
    const T scale_true = 4.0;  // the vision baseline is 1/4 the metric size

    // Same well-excited trajectory; the IMU preintegration is metric, the
    // vision keyframe positions are supplied only up to scale (÷ scale_true).
    std::vector<SO3> R(n);
    std::vector<Vec3> p(n), v(n);
    for (std::size_t i = 0; i < n; ++i) {
        const T s = static_cast<T>(i);
        R[i] = SO3::exp(Vec3{{0.3 * std::sin(0.7 * s), 0.25 * std::cos(0.5 * s), 0.2 * s}});
        p[i] = Vec3{{0.5 * s + 0.3 * std::sin(s), 0.08 * s * s, 0.2 * std::cos(0.6 * s) + 0.1 * s}};
        v[i] = Vec3{{0.5 + 0.2 * std::cos(s), 0.16 * s, -0.12 * std::sin(0.6 * s) + 0.1}};
    }

    std::vector<KF> kfs(n);
    kfs[0].R_world_imu = R[0];
    kfs[0].p_world_imu = p[0] * (1.0 / scale_true);
    for (std::size_t i = 1; i < n; ++i) {
        const Mat3 Rit = R[i - 1].inverse().matrix();
        kfs[i].R_world_imu = R[i];
        kfs[i].p_world_imu = p[i] * (1.0 / scale_true);  // up-to-scale vision position
        kfs[i].dt = dt;
        kfs[i].dR_dbg = Mat3::identity();
        kfs[i].dR = R[i - 1].inverse() * R[i];  // no gyro bias here
        kfs[i].dv = Rit * (v[i] - v[i - 1] - g_world * dt);
        kfs[i].dp = Rit * (p[i] - p[i - 1] - v[i - 1] * dt - g_world * (T{0.5} * dt * dt));
    }

    bs::ImuInitializer<T> init;
    const auto r = init.try_dynamic(kfs);
    REQUIRE(r.success);

    // The metric scale of the vision poses is recovered: scale · p_vision = p_metric.
    REQUIRE_THAT(r.scale, Catch::Matchers::WithinAbs(scale_true, 1e-3));
    // Gravity and velocities come out metric regardless of the vision scaling.
    REQUIRE_THAT(branes::math::lie::detail::norm(r.gravity_world), Catch::Matchers::WithinAbs(9.81, 1e-3));
    REQUIRE(angle_between(r.gravity_world, g_world) * T(180) / std::numbers::pi_v<T> < 0.5);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            REQUIRE_THAT(r.velocities_world[i][j], Catch::Matchers::WithinAbs(v[i][j], 1e-2));
}

namespace {

// The well-excited up-to-scale window of the metric-scale test, with the vision
// scale optionally drifting along the window: keyframe i's position is supplied
// at 1 / (scale_true · (1 + drift · i)) of its metric size. A drifting scale is
// what an unsettled short-baseline SfM track looks like to the alignment (#592).
std::vector<bs::DynInitKeyframe<T>> scaled_window(std::size_t n, T scale_true, T drift) {
    const Vec3 g_world{{0.0, 0.0, -9.81}};
    const T dt = 0.1;
    std::vector<SO3> R(n);
    std::vector<Vec3> p(n), v(n);
    for (std::size_t i = 0; i < n; ++i) {
        const T s = static_cast<T>(i);
        R[i] = SO3::exp(Vec3{{0.3 * std::sin(0.7 * s), 0.25 * std::cos(0.5 * s), 0.2 * s}});
        p[i] = Vec3{{0.5 * s + 0.3 * std::sin(s), 0.08 * s * s, 0.2 * std::cos(0.6 * s) + 0.1 * s}};
        v[i] = Vec3{{0.5 + 0.2 * std::cos(s), 0.16 * s, -0.12 * std::sin(0.6 * s) + 0.1}};
    }
    std::vector<bs::DynInitKeyframe<T>> kfs(n);
    for (std::size_t i = 0; i < n; ++i) {
        kfs[i].R_world_imu = R[i];
        kfs[i].p_world_imu = p[i] * (1.0 / (scale_true * (1.0 + drift * static_cast<T>(i))));
        if (i == 0)
            continue;
        const auto Rit = R[i - 1].inverse().matrix();
        kfs[i].dt = dt;
        kfs[i].dR_dbg = bs::ImuInitializer<T>::Mat3::identity();
        kfs[i].dR = R[i - 1].inverse() * R[i];
        kfs[i].dv = Rit * (v[i] - v[i - 1] - g_world * dt);
        kfs[i].dp = Rit * (p[i] - p[i - 1] - v[i - 1] * dt - g_world * (T{0.5} * dt * dt));
    }
    return kfs;
}

}  // namespace

TEST_CASE("dynamic init: a settled scale passes the settling gate", "[sdk][imu_init]") {
    const auto kfs = scaled_window(9, 4.0, 0.0);
    const auto r = bs::ImuInitializer<T>{}.try_dynamic(kfs);
    REQUIRE(r.success);
    REQUIRE_FALSE(r.scale_unsettled);
    // The prefix (the first 6 of 9 keyframes) recovers the same scale.
    REQUIRE_THAT(r.prefix_scale, Catch::Matchers::WithinRel(r.scale, 1e-3));
    REQUIRE_THAT(r.scale, Catch::Matchers::WithinAbs(4.0, 1e-3));
}

TEST_CASE("dynamic init declines a scale that has not settled", "[sdk][imu_init]") {
    // The vision scale drifts 4 % per keyframe, as MH_01's did over its first
    // windows: the prefix and the full window disagree on the scale.
    const auto kfs = scaled_window(9, 4.0, 0.04);
    const auto r = bs::ImuInitializer<T>{}.try_dynamic(kfs);
    REQUIRE_FALSE(r.success);
    REQUIRE(r.scale_unsettled);
    REQUIRE(r.prefix_scale > 0.0);
    REQUIRE(std::abs(r.scale / r.prefix_scale - 1.0) > 0.05);
    // The decline is the settling gate's: the window resolved enough motion.
    REQUIRE(r.resolved_motion >= bs::ImuInitConfig<T>{}.min_dynamic_motion);

    // With the gate opened up, the same window aligns.
    bs::ImuInitConfig<T> loose;
    loose.max_scale_change = 1.0;
    const auto open = bs::ImuInitializer<T>{loose}.try_dynamic(kfs);
    REQUIRE(open.success);
    REQUIRE_FALSE(open.scale_unsettled);
}

TEST_CASE("dynamic init skips the settling gate below 4 keyframes", "[sdk][imu_init]") {
    const auto kfs = scaled_window(3, 4.0, 0.0);
    const auto r = bs::ImuInitializer<T>{}.try_dynamic(kfs);
    REQUIRE(r.prefix_scale == 0.0);
    REQUIRE_FALSE(r.scale_unsettled);
}

TEST_CASE("dynamic init recovers the gyro bias with a non-identity dR_dbg", "[sdk][imu_init]") {
    using KF = bs::DynInitKeyframe<T>;
    using Mat3 = bs::ImuInitializer<T>::Mat3;

    const Vec3 g_world{{0.0, 0.0, -9.81}};
    const Vec3 bg_true{{0.003, -0.004, 0.005}};
    const T dt = 0.1;
    const std::size_t n = 5;

    // A non-trivial bias Jacobian, varied per keyframe, exercises the JᵀJ
    // normal-equations accumulation (not the J = I shortcut).
    auto jac = [](std::size_t i) {
        Mat3 J{};
        J(0, 0) = 2.0 - 0.1 * static_cast<T>(i);
        J(1, 1) = 0.5 + 0.05 * static_cast<T>(i);
        J(2, 2) = 1.5;
        J(0, 1) = 0.2;  // off-diagonal coupling
        return J;
    };

    std::vector<SO3> R(n);
    std::vector<Vec3> p(n), v(n);
    for (std::size_t i = 0; i < n; ++i) {
        const T s = static_cast<T>(i);
        R[i] = SO3::exp(Vec3{{0.04 * s, -0.02 * s, 0.03 * s}});
        p[i] = Vec3{{0.3 * s, 0.05 * s * s, 0.1 * s}};
        v[i] = Vec3{{0.3, 0.1 * s, -0.05}};
    }

    std::vector<KF> kfs(n);
    kfs[0].R_world_imu = R[0];
    kfs[0].p_world_imu = p[0];
    for (std::size_t i = 1; i < n; ++i) {
        const Mat3 Rit = R[i - 1].inverse().matrix();
        const Mat3 J = jac(i);
        kfs[i].R_world_imu = R[i];
        kfs[i].p_world_imu = p[i];
        kfs[i].dt = dt;
        kfs[i].dR_dbg = J;
        // ΔR = (Rᵢᵀ Rⱼ)·Exp(−J·bg) ⇒ residual Log(ΔRᵀ Rᵢᵀ Rⱼ) = J·bg,
        // so the GN step solves (Σ JᵀJ) δ = (Σ JᵀJ) bg ⇒ δ = bg_true.
        kfs[i].dR = (R[i - 1].inverse() * R[i]) * SO3::exp(J * bg_true * (-T{1}));
        kfs[i].dv = Rit * (v[i] - v[i - 1] - g_world * dt);
        kfs[i].dp = Rit * (p[i] - p[i - 1] - v[i - 1] * dt - g_world * (T{0.5} * dt * dt));
    }

    bs::ImuInitializer<T> init;
    const auto r = init.try_dynamic(kfs);
    REQUIRE(r.success);
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(r.gyro_bias[i], Catch::Matchers::WithinAbs(bg_true[i], 1e-6));
}

TEST_CASE("dynamic init fails with too few keyframes", "[sdk][imu_init]") {
    std::vector<bs::DynInitKeyframe<T>> one(1);
    bs::ImuInitializer<T> init;
    REQUIRE_FALSE(init.try_dynamic(one).success);
}
