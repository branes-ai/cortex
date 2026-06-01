// MSCKF backend dynamic-init end-to-end test (issue #230, epic #211).
//
// Drive the backend through a *moving* start (no stationary window) with a
// self-consistent IMU + camera stream and confirm it bootstraps via the full
// visual-inertial alignment (InitMethod::Dynamic) rather than the gravity-only
// fallback, leaving a finite, positive-semidefinite, advancing state.

#include <branes/sdk/msckf_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ld = branes::math::lie::detail;
using T = double;
using Backend = bs::MsckfBackend<T>;
using DVec3 = Backend::DVec3;
using SO3 = branes::math::lie::SO3<T>;
using Mat3 = ld::Mat<T, 3, 3>;

bs::ImuMeasurement<T> imu(double t, const DVec3& g, const DVec3& a) {
    bs::ImuMeasurement<T> m;
    m.timestamp_s = t;
    m.angular_velocity = bs::Vec3<T>{{g[0], g[1], g[2]}};
    m.linear_acceleration = bs::Vec3<T>{{a[0], a[1], a[2]}};
    return m;
}

std::vector<DVec3> landmarks() {
    std::vector<DVec3> pts;
    for (int i = -2; i <= 2; ++i)
        for (int j = -2; j <= 2; ++j) {
            const T z = 5.0 + 0.8 * static_cast<T>((i * 3 + j * 5 + 7) % 4) + 0.3 * (i + 2);
            pts.push_back(DVec3{{0.9 * i, 0.8 * j, z}});
        }
    return pts;
}

}  // namespace

TEST_CASE("the backend bootstraps a moving start via dynamic VI init", "[sdk][msckf][dynamic-init]") {
    Backend backend;  // single identity-pinhole camera at the IMU
    bs::VioConfig cfg;
    cfg.max_clones = 8;
    backend.initialize(cfg);

    const DVec3 g_world{{0.0, 0.0, -9.81}};
    const T dt = 0.005;                // 200 Hz IMU
    const std::size_t cam_every = 20;  // 10 Hz camera
    const std::size_t steps = 240;
    const auto X = landmarks();

    // Forward-integrate a self-consistent moving trajectory from analytic IMU
    // inputs (so the backend's preintegration reconstructs exactly this motion),
    // emitting camera frames whose observations project through the true pose.
    SO3 R;  // R[0] = identity
    DVec3 p{}, v{{0.3, 0.1, 0.0}};
    std::size_t frames = 0;
    for (std::size_t k = 0; k < steps; ++k) {
        const T t = static_cast<T>(k) * dt;
        const DVec3 gyro{{0.3 * std::sin(1.5 * t), 0.25 * std::cos(1.2 * t), 0.4}};
        const DVec3 a_world{{0.5 * std::cos(0.8 * t), 0.4 * std::sin(t), 0.3 * std::cos(1.3 * t)}};
        const DVec3 accel_body = R.inverse() * (a_world - g_world);  // specific force

        backend.process_imu(imu(t, gyro, accel_body));

        if (k % cam_every == 0) {
            const Mat3 Rt = R.inverse().matrix();
            std::vector<bs::FrontendObservation<T>> obs;
            for (std::size_t m = 0; m < X.size(); ++m) {
                const DVec3 Xc = Rt * (X[m] - p);
                if (!(Xc[2] > 0.0))
                    continue;
                bs::FrontendObservation<T> o;
                o.feature_id = static_cast<std::uint64_t>(m);
                o.camera_id = 0;
                o.u = Xc[0] / Xc[2];  // identity pinhole ⇒ pixel == normalized
                o.v = Xc[1] / Xc[2];
                obs.push_back(o);
            }
            backend.process_camera(t, obs);
            ++frames;
        }

        // Strapdown integration of the trajectory (matches the IMU stream).
        p = p + v * dt + a_world * (T{0.5} * dt * dt);
        v = v + a_world * dt;
        R = R * SO3::exp(gyro * dt);
    }

    REQUIRE(frames > 5);
    REQUIRE(backend.initialized());
    // Bootstrapped by the full visual-inertial alignment, not gravity-only.
    REQUIRE(backend.init_diagnostics().method == bs::InitMethod::Dynamic);

    // The recovered gravity magnitude is sane.
    REQUIRE(std::abs(backend.init_diagnostics().gravity_residual) < 0.1);

    // The state is finite and the covariance valid.
    const auto ns = backend.current_state();
    for (std::size_t i = 0; i < 3; ++i) {
        REQUIRE(std::isfinite(ns.T_world_imu.translation()[i]));
        REQUIRE(std::isfinite(ns.velocity_world[i]));
    }
    REQUIRE(branes::sdk::msckf::is_positive_semidefinite(backend.state().covariance()));
}
