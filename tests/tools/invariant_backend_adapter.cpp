// SPDX-License-Identifier: MIT
//
// Gate for the R-IEKF EuRoC adapter (branes/tools/invariant_backend_adapter.hpp,
// #347/#348). The full end-to-end comparison runs in vio_pipeline over EuRoC
// (not vendored in CI); here we pin the adapter's contract bridges that the
// harness relies on, on synthetic input:
//
//   * it bootstraps from a static IMU window (gravity-aligned) — initialized()
//     flips true only after enough samples, and the seeded attitude levels the
//     measured gravity (roll/pitch recovered, zero velocity);
//   * it satisfies the VioEstimator<T, Backend> duck-typed contract (process_imu /
//     process_camera / current_state) and maps the SE2(3) nav back to a NavState.
//
// ASCII-only TEST_CASE names (the Windows MSVC ctest job rejects non-ASCII).

#include <branes/math/cameras/pinhole_radtan.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/vio_backend.hpp>
#include <branes/tools/invariant_backend_adapter.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <span>
#include <vector>

namespace bt = branes::tools;
namespace bs = branes::sdk;
using Catch::Matchers::WithinAbs;

namespace {
using T = double;
using Adapter = bt::InvariantBackendAdapter<T>;
using SO3 = branes::math::lie::SO3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;

Adapter make_adapter() {
    // EuRoC-like intrinsics; identity extrinsic keeps the test about the bridge.
    Adapter::Camera cam(458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    return Adapter(cam, SO3{}, Vec3{});
}

bs::ImuMeasurement<T> stationary_sample(double t) {
    bs::ImuMeasurement<T> m;
    m.timestamp_s = t;
    m.angular_velocity = {0.0, 0.0, 0.0};
    m.linear_acceleration = {0.0, 0.0, 9.81};  // at rest: specific force = +g (world up)
    return m;
}
}  // namespace

TEST_CASE("invariant adapter: bootstraps from a static IMU window", "[tools][invariant_adapter]") {
    Adapter a = make_adapter();
    bs::VioConfig cfg;
    a.initialize(cfg);
    REQUIRE_FALSE(a.initialized());

    // Pin the bootstrap boundary exactly: one sample short of the window
    // (kInitSamples = 150 in the adapter) it must still be down, and it must come
    // up as soon as the window fills. Asserting at the off-by-one catches both an
    // early bootstrap and a silently-lowered threshold.
    for (int i = 0; i < 149; ++i)
        a.process_imu(stationary_sample(0.005 * i));
    REQUIRE_FALSE(a.initialized());
    for (int i = 149; i < 200; ++i)
        a.process_imu(stationary_sample(0.005 * i));
    REQUIRE(a.initialized());

    const auto ns = a.current_state();
    // Zero-velocity seed.
    REQUIRE_THAT(ns.velocity_world[0], WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(ns.velocity_world[1], WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(ns.velocity_world[2], WithinAbs(0.0, 1e-9));
    // Gravity-aligned attitude: rotating the measured specific force (+z body) into
    // world should land on world up (+z), i.e. the seed levels roll/pitch.
    const Vec3 up_body{{0.0, 0.0, 1.0}};
    const Vec3 up_world = ns.T_world_imu.rotation() * up_body;
    REQUIRE_THAT(up_world[2], WithinAbs(1.0, 1e-6));
}

TEST_CASE("invariant adapter: satisfies the VioEstimator backend contract", "[tools][invariant_adapter]") {
    Adapter a = make_adapter();
    a.initialize(bs::VioConfig{});
    for (int i = 0; i < 200; ++i)
        a.process_imu(stationary_sample(0.005 * i));
    REQUIRE(a.initialized());

    // process_camera takes pixel FrontendObservations; with no real parallax the
    // update may decline, but the call path (unprojection → backend) must be safe.
    std::vector<bs::FrontendObservation<T>> obs;
    for (int k = 0; k < 4; ++k) {
        bs::FrontendObservation<T> o;
        o.feature_id = static_cast<std::uint64_t>(k);
        o.camera_id = 0;
        o.u = 300.0 + 40.0 * k;
        o.v = 240.0;
        obs.push_back(o);
    }
    const auto before_t = a.current_state().timestamp_s;
    a.process_camera(1.0, std::span<const bs::FrontendObservation<T>>{obs});

    // covariance() is a square PSD-sized matrix; and process_camera must have
    // advanced the state stamp (propagation to the frame time), not merely left
    // the positive bootstrap stamp in place.
    const auto P = a.covariance();
    REQUIRE(P.rows == P.cols);
    REQUIRE(P.rows >= 15);
    REQUIRE(a.current_state().timestamp_s > before_t);
}
