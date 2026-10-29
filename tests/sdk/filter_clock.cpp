// The filter clock stays double in every arithmetic type (issue #449).
//
// Timestamps are double by design; T may be a float or a posit. Propagation
// advances the clock by T(dt), which a narrow T rounds (float(0.0025) is off by
// about 6e-11 s), so a backend that only accumulated Σ T(dt) would report pose
// times that drift from the sensor clock. Both backends reset the clock to the
// double sample time instead: the MSCKF backend after each IMU sample and in S3
// augmentation (which stamps the frame time), the invariant backend after each
// propagation. The reported time is then the sample time exactly, in float as in
// double.
//
// ASCII-only TEST_CASE names (the Windows MSVC ctest job rejects non-ASCII).

#include <branes/sdk/msckf/invariant_vio_backend.hpp>
#include <branes/sdk/msckf_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ms = branes::sdk::msckf;
namespace lie = branes::math::lie;
using T = float;

constexpr double kT0 = 1000.0;  // a sensor clock well away from zero
constexpr double kDt = 0.005;   // 200 Hz IMU

}  // namespace

TEST_CASE("MSCKF backend in float: a frame between IMU samples stamps the frame time exactly",
          "[sdk][msckf][backend][clock]") {
    using Backend = bs::MsckfBackend<T>;
    Backend backend;
    bs::VioConfig cfg;
    cfg.max_clones = 8;
    backend.initialize(cfg);

    bs::ImuMeasurement<T> m;
    m.angular_velocity = {T{0}, T{0}, T{0}};
    m.linear_acceleration = {T{0}, T{0}, T{9.81f}};  // at rest, level
    double t = kT0;
    for (int k = 0; k < 120; ++k) {
        t = kT0 + kDt * k;
        m.timestamp_s = t;
        backend.process_imu(m);
    }
    REQUIRE(backend.initialized());
    REQUIRE(backend.current_state().timestamp_s == t);

    // Frames half-way between samples: propagation to the frame runs with
    // T(0.0025), which float rounds.
    const std::vector<bs::FrontendObservation<T>> none;
    for (int f = 0; f < 5; ++f) {
        const double t_frame = t + kDt / 2;
        backend.process_camera(t_frame, std::span<const bs::FrontendObservation<T>>{none});
        REQUIRE(backend.current_state().timestamp_s == t_frame);
        t += kDt;
        m.timestamp_s = t;
        backend.process_imu(m);
        REQUIRE(backend.current_state().timestamp_s == t);
    }
}

TEST_CASE("invariant VIO backend in float: the nav clock is the sample time, not a sum of T(dt)",
          "[sdk][riekf][backend][clock]") {
    using Vio = ms::InvariantVioBackend<T>;
    using Vec3 = Vio::Vec3;
    Vio vio;
    vio.set_nav(Vio::SE23{}, Vec3{}, Vec3{}, kT0);

    const Vec3 gyro{{T{0}, T{0}, T{0}}};
    const Vec3 accel{{T{0}, T{0}, T{9.81f}}};
    const std::vector<ms::NormalizedObs<T>> none;
    double t = kT0;
    for (int k = 1; k <= 400; ++k) {
        t = kT0 + kDt * k;
        vio.process_imu(gyro, accel, t);
        if (k % 10 == 5) {
            const double t_frame = t + kDt / 2;
            vio.process_camera(t_frame, std::span<const ms::NormalizedObs<T>>{none});
            REQUIRE(vio.nav().timestamp == t_frame);
        }
    }
    // The last sample's propagation ran from the previous sample (or frame);
    // the clock is that sample's time exactly.
    vio.process_imu(gyro, accel, t + kDt);
    REQUIRE(vio.nav().timestamp == t + kDt);
}
