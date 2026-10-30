// branes/test/synthetic_frames.hpp — a synthetic grayscale scene for driving the
// VIO front end in tests.
//
// Bright 8×8 squares on a black background, on a 40 px grid, shifted by
// (ox, oy). The square corners are strong FAST features that KLT can follow
// across small frame-to-frame translations.
//
// `drive_estimator` runs a VioEstimator over a scene that is the same picture
// seen by a moving camera: 80 stationary IMU samples to initialize, then the
// body accelerates along +x at kDriveAccel, with ten 200 Hz IMU samples between
// frames. The camera is the IMU (identity extrinsic, so it looks along world +z
// when level); the squares' corners are landmarks on the plane z = kDriveDepth,
// projected through the estimator's pinhole (f = 200, c = (120, 90)). IMU and
// image agree, so the backend's triangulation sees real parallax and its S6
// updates pass the gate. Generic in the arithmetic type, so the same run can
// be compared across types.

#pragma once

#include <branes/cv/image.hpp>
#include <branes/sdk/vio_estimator.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace branes::test {

inline constexpr std::size_t kFrameWidth = 240;
inline constexpr std::size_t kFrameHeight = 180;

[[nodiscard]] inline branes::cv::OwnedImage<std::uint8_t> render_squares(double ox, double oy) {
    branes::cv::OwnedImage<std::uint8_t> img(kFrameWidth, kFrameHeight, 0);
    for (std::size_t gy = 40; gy + 12 < kFrameHeight; gy += 40)
        for (std::size_t gx = 40; gx + 12 < kFrameWidth; gx += 40) {
            const auto x0 = static_cast<long>(gx + ox);
            const auto y0 = static_cast<long>(gy + oy);
            for (long dy = 0; dy < 8; ++dy)
                for (long dx = 0; dx < 8; ++dx) {
                    const long px = x0 + dx, py = y0 + dy;
                    if (px >= 0 && py >= 0 && px < static_cast<long>(kFrameWidth) &&
                        py < static_cast<long>(kFrameHeight))
                        img(static_cast<std::size_t>(py), static_cast<std::size_t>(px)) = 255;
                }
        }
    return img;
}

inline constexpr double kDriveAccel = 2.0;  // m/s², along world +x after the stationary window
inline constexpr double kDriveDepth = 3.0;  // m, the landmark plane above the camera
inline constexpr double kDriveFocal = 200.0, kDriveCx = 120.0, kDriveCy = 90.0;

/// The squares as the camera sees them from body position (px, 0, 0): each
/// corner is a landmark at depth kDriveDepth, placed so that px = 0 renders the
/// grid of `render_squares(0, 0)`.
[[nodiscard]] inline branes::cv::OwnedImage<std::uint8_t> render_from(double px) {
    return render_squares(-kDriveFocal * px / kDriveDepth, 0.0);
}

/// A level IMU sample at time t with specific force (ax, 0, g).
template <class S>
[[nodiscard]] branes::sdk::ImuMeasurement<S> level_sample(double t, double ax) {
    branes::sdk::ImuMeasurement<S> m;
    m.timestamp_s = t;
    m.angular_velocity = branes::sdk::Vec3<S>{{S{0}, S{0}, S{0}}};
    m.linear_acceleration = branes::sdk::Vec3<S>{{S(ax), S{0}, S{9.81}}};
    return m;
}

/// A level, stationary IMU sample at time t.
template <class S>
[[nodiscard]] branes::sdk::ImuMeasurement<S> level_sample(double t) {
    branes::sdk::ImuMeasurement<S> m;
    m.timestamp_s = t;
    m.angular_velocity = branes::sdk::Vec3<S>{{S{0}, S{0}, S{0}}};
    m.linear_acceleration = branes::sdk::Vec3<S>{{S{0}, S{0}, S{9.81}}};  // at rest, level
    return m;
}

/// An estimator with a calibrated pinhole camera (so pixel→bearing unprojection
/// is sane), on backend B (the default MSCKF backend, or one with per-stage types).
template <class S, class B = branes::sdk::MsckfBackend<S>>
[[nodiscard]] branes::sdk::VioEstimator<S, B> make_estimator() {
    typename B::CameraCalibration cal;
    cal.intrinsics = typename B::Camera(S{200}, S{200}, S{120}, S{90}, S{0}, S{0}, S{0}, S{0});
    return branes::sdk::VioEstimator<S, B>(B(std::vector<typename B::CameraCalibration>{cal}));
}

inline constexpr double kDriveDt = 0.005;             // 200 Hz IMU
inline constexpr double kDriveTMove = kDriveDt * 79;  // the last stationary sample

/// The true body x position at time t (y = z = 0, attitude level throughout).
[[nodiscard]] inline double drive_truth_x(double t) {
    const double tau = t > kDriveTMove ? t - kDriveTMove : 0.0;
    return 0.5 * kDriveAccel * tau * tau;
}

/// Initialize `est` from 80 stationary samples, then accelerate and feed
/// `frames` frames, calling `per_frame(f, t)` after frame f is fed at time t.
template <class S, class B, class PerFrame>
void drive_estimator(branes::sdk::VioEstimator<S, B>& est, int frames, PerFrame&& per_frame) {
    constexpr double dt = kDriveDt;
    std::vector<branes::sdk::ImuMeasurement<S>> batch;
    for (int k = 0; k < 80; ++k)
        batch.push_back(level_sample<S>(dt * k));
    est.feed_imu(std::span<const branes::sdk::ImuMeasurement<S>>{batch});
    // The backend integrates each interval with the sample that ends it, so the
    // motion starts at the last stationary sample (kDriveTMove).
    int k = 80;
    for (int f = 0; f < frames; ++f) {
        std::vector<branes::sdk::ImuMeasurement<S>> imus;
        for (int j = 0; j < 10; ++j, ++k)
            imus.push_back(level_sample<S>(dt * k, kDriveAccel));
        est.feed_imu(std::span<const branes::sdk::ImuMeasurement<S>>{imus});
        const double t = dt * (k - 1);
        const auto img = render_from(drive_truth_x(t));
        est.feed_image(t, img.view());
        per_frame(f, t);
    }
}

}  // namespace branes::test
