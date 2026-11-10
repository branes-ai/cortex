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

#include <cmath>
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

// An image that reaches the clock before its coincident IMU sample (#618
// review). The image holds the previous sample up to the clock; the sample that
// then arrives at the clock is new and must start the next interval, not be
// dropped as a duplicate. The gyro is w0 up to and at the tie and w1 after it,
// so the hold over the gap equals the interpolated interval, and image-first
// must match IMU-first to roundoff. Dropping the tie sample would start the next
// interval from the midpoint of w0 and w1, about 1e-3 rad off.
namespace {

template <typename Feed>
double msckf_yaw_after_tie(Feed feed) {
    using D = double;
    bs::MsckfBackend<D> backend;
    bs::VioConfig cfg;
    cfg.max_clones = 8;
    backend.initialize(cfg);
    bs::ImuMeasurement<D> m;
    m.angular_velocity = {0.0, 0.0, 0.0};
    m.linear_acceleration = {0.0, 0.0, 9.81};
    double t = kT0;
    for (int k = 0; k < 120; ++k) {
        t = kT0 + kDt * k;
        m.timestamp_s = t;
        backend.process_imu(m);
    }
    REQUIRE(backend.initialized());
    feed(backend, m, t + kDt);             // the tie, with the gyro still w0 = 0
    m.angular_velocity = {0.0, 0.0, 1.0};  // w1
    m.timestamp_s = t + 2 * kDt;
    backend.process_imu(m);
    return backend.current_state().T_world_imu.rotation().log()[2];
}

}  // namespace

TEST_CASE("MSCKF backend: an IMU sample that arrives after an image at its time is kept",
          "[sdk][msckf][backend][clock]") {
    const std::vector<bs::FrontendObservation<double>> none;
    const auto camera = [&](bs::MsckfBackend<double>& b, double t) {
        b.process_camera(t, std::span<const bs::FrontendObservation<double>>{none});
    };
    const double imu_first = msckf_yaw_after_tie([&](auto& b, auto m, double t) {
        m.timestamp_s = t;
        b.process_imu(m);
        camera(b, t);
    });
    const double image_first = msckf_yaw_after_tie([&](auto& b, auto m, double t) {
        camera(b, t);
        m.timestamp_s = t;
        b.process_imu(m);
    });
    REQUIRE(imu_first > 1e-3);  // the step rotated: the test can see a dropped sample
    REQUIRE(std::abs(image_first - imu_first) < 1e-12);
}

TEST_CASE("invariant VIO backend: an IMU sample that arrives after an image at its time is kept",
          "[sdk][riekf][backend][clock]") {
    using Vio = ms::InvariantVioBackend<double>;
    using Vec3 = Vio::Vec3;
    const Vec3 accel{{0.0, 0.0, 9.81}};
    const Vec3 w0{{0.0, 0.0, 0.0}};
    const Vec3 w1{{0.0, 0.0, 1.0}};
    const std::vector<ms::NormalizedObs<double>> none;
    const auto run = [&](bool image_first) {
        Vio vio;
        vio.set_nav(Vio::SE23{}, Vec3{}, Vec3{}, kT0);
        vio.process_imu(w0, accel, kT0 + kDt);
        const double t_tie = kT0 + 2 * kDt;
        const auto camera = [&] { vio.process_camera(t_tie, std::span<const ms::NormalizedObs<double>>{none}); };
        if (image_first)
            camera();
        vio.process_imu(w0, accel, t_tie);
        if (!image_first)
            camera();
        vio.process_imu(w1, accel, t_tie + kDt);
        return vio.nav().X.rotation().log()[2];
    };
    const double imu_first = run(false);
    const double image_first = run(true);
    REQUIRE(imu_first > 1e-3);
    REQUIRE(std::abs(image_first - imu_first) < 1e-12);
}
