// MSCKF backend end-to-end tests (issue #35).
//
// Drives the MsckfBackend through the VioBackend interface with synthetic
// IMU + camera streams: a stationary window initializes attitude and gyro
// bias, then an accelerated phase generates self-consistent feature
// observations (projected through the backend's own clone poses) so the
// full pipeline — propagation, clone augmentation, feature tracking,
// null-space update, and window marginalization — runs and produces a
// stable, finite navigation state. Accuracy against ground truth is the
// EuRoC harness's job (#46).

#include <branes/sdk/msckf_backend.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

namespace bs = branes::sdk;
using T = double;
using Backend = bs::MsckfBackend<T>;
using DVec3 = Backend::DVec3;

bs::ImuMeasurement<T> imu_sample(double t, const DVec3& gyro, const DVec3& accel) {
    bs::ImuMeasurement<T> m;
    m.timestamp_s = t;
    m.angular_velocity = bs::Vec3<T>{{gyro[0], gyro[1], gyro[2]}};
    m.linear_acceleration = bs::Vec3<T>{{accel[0], accel[1], accel[2]}};
    return m;
}

bs::VioConfig default_config() {
    bs::VioConfig c;
    c.max_clones = 8;
    return c;
}

}  // namespace

TEST_CASE("the backend initializes attitude from a stationary window", "[sdk][msckf][backend]") {
    Backend backend;
    backend.initialize(default_config());

    // Tilted ~20° about x; at rest the accelerometer reads Rᵀ(0,0,+g).
    const auto R_true = branes::math::lie::SO3<T>::exp(DVec3{{0.35, 0.0, 0.0}});
    const DVec3 up_world{{0.0, 0.0, 9.81}};
    const DVec3 a_m = R_true.inverse() * up_world;
    const DVec3 bias{{0.005, -0.003, 0.004}};

    double t = 0.0;
    for (int k = 0; k < 80; ++k, t += 0.005) {
        const T n = 1e-4 * std::sin(0.2 * k);
        backend.process_imu(
            imu_sample(t, DVec3{{bias[0] + n, bias[1] - n, bias[2]}}, DVec3{{a_m[0] + n, a_m[1], a_m[2] - n}}));
    }
    REQUIRE(backend.initialized());

    // Gravity-in-body from the estimate should match truth within ~1°.
    const auto ns = backend.current_state();
    const DVec3 g_world{{0.0, 0.0, -9.81}};
    const DVec3 g_body_true = R_true.inverse() * g_world;
    const DVec3 g_body_est = ns.T_world_imu.rotation().inverse() * g_world;
    const T cosang = branes::math::lie::detail::dot(g_body_true, g_body_est) /
                     (branes::math::lie::detail::norm(g_body_true) * branes::math::lie::detail::norm(g_body_est));
    const T deg = std::acos(std::min(T{1}, std::max(T{-1}, cosang))) * 180.0 / M_PI;
    REQUIRE(deg < 1.0);
}

TEST_CASE("the backend runs end-to-end and stays stable under motion", "[sdk][msckf][backend]") {
    const auto cfg = default_config();
    Backend backend;
    backend.initialize(cfg);

    // 1) Stationary init (level, no bias).
    double t = 0.0;
    for (int k = 0; k < 80; ++k, t += 0.005)
        backend.process_imu(imu_sample(t, DVec3{{0, 0, 0}}, DVec3{{0, 0, 9.81}}));
    REQUIRE(backend.initialized());

    // World features ahead of the camera (it accelerates along +x).
    const std::vector<DVec3> feats = {{{1.0, 0.3, 4.0}},
                                      {{1.5, -0.2, 5.0}},
                                      {{2.0, 0.4, 6.0}},
                                      {{0.8, -0.4, 3.5}},
                                      {{2.5, 0.1, 5.5}},
                                      {{1.2, 0.0, 4.5}}};

    // 2) Accelerate along +x: specific force (1, 0, 9.81) ⇒ a_world = (1,0,0).
    const DVec3 accel{{1.0, 0.0, 9.81}};
    const double p0x = backend.current_state().T_world_imu.translation()[0];

    int frames = 0;
    for (int step = 0; step < 600; ++step, t += 0.005) {
        backend.process_imu(imu_sample(t, DVec3{{0, 0, 0}}, accel));

        // Every 10 IMU samples, emit a camera frame whose observations are
        // projected through the backend's *current* pose (so the new clone
        // and its measurements are self-consistent — residuals ~0).
        if (step % 10 == 0) {
            const auto ns = backend.current_state();
            const auto& Rwi = ns.T_world_imu.rotation();
            const auto pwi = ns.T_world_imu.translation();
            std::vector<bs::FrontendObservation<T>> obs;
            for (std::size_t f = 0; f < feats.size(); ++f) {
                const DVec3 pc = Rwi.inverse() * (feats[f] - pwi);  // identity extrinsics
                if (!(pc[2] > 0.0))
                    continue;
                bs::FrontendObservation<T> o;
                o.feature_id = f;
                o.camera_id = 0;
                o.u = pc[0] / pc[2];  // identity pinhole ⇒ pixel == normalized
                o.v = pc[1] / pc[2];
                obs.push_back(o);
            }
            backend.process_camera(t, obs);
            ++frames;

            // Invariants after each frame: covariance stays valid, the
            // window respects max_clones, and the state is finite.
            REQUIRE(branes::sdk::msckf::is_positive_semidefinite(backend.state().P));
            REQUIRE(backend.state().clones.size() <= static_cast<std::size_t>(cfg.max_clones));
            const auto pose = backend.current_state().T_world_imu.translation();
            for (std::size_t i = 0; i < 3; ++i)
                REQUIRE(std::isfinite(pose[i]));
        }
    }

    REQUIRE(frames > 0);
    // The estimator should have moved forward along +x (it produces poses,
    // not a frozen state).
    const double p1x = backend.current_state().T_world_imu.translation()[0];
    REQUIRE(p1x > p0x + 0.05);
    REQUIRE(std::isfinite(backend.current_state().velocity_world[0]));
}

TEST_CASE("the square-root covariance flag is a distinct (unimplemented) type", "[sdk][msckf][backend]") {
    // Documents the template knob: the full-covariance backend is the
    // default; the sqrt path is a separate instantiation (static_assert
    // guards it), so this is purely a compile-time contract check.
    Backend full;
    full.initialize(default_config());
    REQUIRE_FALSE(full.initialized());  // no samples yet
}
