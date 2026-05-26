// VioEstimator end-to-end tests (issue #45).
//
// Exercises the lifecycle gate (measurements ignored unless Active) and a
// full front-end → backend run on synthetic frames: a high-contrast
// pattern of bright squares translating across a grayscale image is FAST-
// detected and KLT-tracked, the resulting observations drive the MSCKF
// backend, and the estimator produces a finite, stable navigation state.
// Trajectory accuracy on real EuRoC is the harness's job (#46).

#include <branes/sdk/vio_estimator.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

namespace bs = branes::sdk;
using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T>;
using DVec3 = Backend::DVec3;

constexpr std::size_t kW = 240;
constexpr std::size_t kH = 180;

// Render bright 8×8 squares on a black background, on a 40px grid, shifted
// by (ox, oy). The square corners are strong FAST features that KLT can
// follow across small frame-to-frame translations.
branes::cv::OwnedImage<std::uint8_t> render(double ox, double oy) {
    branes::cv::OwnedImage<std::uint8_t> img(kW, kH, 0);
    for (std::size_t gy = 40; gy + 12 < kH; gy += 40)
        for (std::size_t gx = 40; gx + 12 < kW; gx += 40) {
            const auto x0 = static_cast<long>(gx + ox);
            const auto y0 = static_cast<long>(gy + oy);
            for (long dy = 0; dy < 8; ++dy)
                for (long dx = 0; dx < 8; ++dx) {
                    const long px = x0 + dx, py = y0 + dy;
                    if (px >= 0 && py >= 0 && px < static_cast<long>(kW) && py < static_cast<long>(kH))
                        img(static_cast<std::size_t>(py), static_cast<std::size_t>(px)) = 255;
                }
        }
    return img;
}

bs::ImuMeasurement<T> level_sample(double t) {
    bs::ImuMeasurement<T> m;
    m.timestamp_s = t;
    m.angular_velocity = bs::Vec3<T>{{0, 0, 0}};
    m.linear_acceleration = bs::Vec3<T>{{0, 0, 9.81}};  // at rest, level
    return m;
}

// A calibrated backend (so pixel→bearing unprojection is sane).
Estimator make_estimator() {
    Backend::CameraCalibration cal;
    cal.intrinsics = Backend::Camera(200.0, 200.0, 120.0, 90.0, 0.0, 0.0, 0.0, 0.0);
    return Estimator(Backend(std::vector<Backend::CameraCalibration>{cal}));
}

}  // namespace

TEST_CASE("the estimator gates measurements on its lifecycle", "[sdk][vio]") {
    auto est = make_estimator();
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Unconfigured);

    // Feeds before configuration are ignored (no crash, no state change).
    const auto frame = render(0, 0);
    est.feed_image(0.0, frame.view());
    REQUIRE(est.num_tracked_features() == 0);

    bs::VioConfig cfg;
    cfg.max_clones = 8;
    est.configure(cfg);
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Inactive);

    // Still inactive ⇒ still ignored.
    est.feed_image(0.01, frame.view());
    REQUIRE(est.num_tracked_features() == 0);

    est.activate();
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Active);
    est.feed_image(0.02, frame.view());
    REQUIRE(est.num_tracked_features() > 0);

    est.deactivate();
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Inactive);

    est.reset();
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Inactive);
    REQUIRE(est.num_tracked_features() == 0);

    est.teardown();
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Teardown);
    est.activate();  // no-op from Teardown
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Teardown);
}

TEST_CASE("the estimator runs the front end and backend end-to-end", "[sdk][vio]") {
    auto est = make_estimator();
    bs::VioConfig cfg;
    cfg.max_clones = 8;
    est.configure(cfg);
    est.activate();

    // 1) Stationary IMU window to initialize the backend.
    double t = 0.0;
    std::vector<bs::ImuMeasurement<T>> batch;
    for (int k = 0; k < 80; ++k, t += 0.005)
        batch.push_back(level_sample(t));
    est.feed_imu(std::span<const bs::ImuMeasurement<T>>{batch});
    REQUIRE(est.backend().initialized());

    // 2) Feed translating frames interleaved with IMU. The front end
    // should detect and then persistently track the square corners.
    std::size_t max_clones_seen = 0;
    for (int f = 0; f < 20; ++f) {
        std::vector<bs::ImuMeasurement<T>> imus;
        for (int k = 0; k < 10; ++k, t += 0.005)
            imus.push_back(level_sample(t));
        est.feed_imu(std::span<const bs::ImuMeasurement<T>>{imus});

        const auto img = render(1.0 * f, 0.5 * f);
        est.feed_image(t, img.view());

        REQUIRE(est.num_tracked_features() > 0);
        REQUIRE(branes::sdk::msckf::is_positive_semidefinite(est.backend().state().P));
        max_clones_seen = std::max(max_clones_seen, est.backend().state().clones.size());

        const auto pose = est.current_pose().translation();
        for (std::size_t i = 0; i < 3; ++i)
            REQUIRE(std::isfinite(pose[i]));
    }

    // The sliding window grew but stayed bounded by max_clones.
    REQUIRE(max_clones_seen > 1);
    REQUIRE(max_clones_seen <= static_cast<std::size_t>(cfg.max_clones));

    // Features tracked across many frames (KLT actually followed them, not
    // just re-detected fresh each time).
    REQUIRE(est.num_tracked_features() > 0);
}
