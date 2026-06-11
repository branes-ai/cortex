// SPDX-License-Identifier: MIT
//
// branes/sdk/vio_backend.hpp — the VIO backend abstraction.
//
// This is the seam between the visual-inertial *front end* (feature
// tracking, IMU buffering) and the *estimator backend* (MSCKF today,
// sliding-window optimization later). The front end produces
// backend-agnostic measurements — IMU samples and per-frame feature
// observations — and the backend consumes them through this interface
// and exposes the current navigation state. Swapping MSCKF for SW-opt
// must require no front-end change; see #33 (E3) for the topology.
//
// The interface is templated on the scalar so the whole estimator can
// be instantiated in double, float, or a Universal type (the math layer
// is already type-generic). Measurement and state structs are POD-ish
// value types — no ownership, no middleware (no Zenoh/ROS here; that
// lives in daemons/).
//
// Header-only, C++20.

#ifndef BRANES_SDK_VIO_BACKEND_HPP
#define BRANES_SDK_VIO_BACKEND_HPP

#include <branes/math/arithmetic.hpp>
#include <branes/math/lie/se3.hpp>

#include <array>
#include <cstdint>
#include <span>

namespace branes::sdk {

template <class T>
using Vec3 = std::array<T, 3>;

/// Minimal estimator configuration. This is a deliberately small,
/// typed POD seed — the unified YAML→POD config (E6, #69) will replace
/// and extend it. Backends read what they need and ignore the rest.
struct VioConfig {
    double gravity_magnitude = 9.81;      ///< m/s² (world −z is "down")
    double accel_noise_density = 2.0e-3;  ///< continuous-time σ_a
    double gyro_noise_density = 1.6e-4;   ///< continuous-time σ_g
    double accel_bias_random_walk = 3.0e-3;
    double gyro_bias_random_walk = 1.9e-5;
    /// Visual measurement σ in normalized image coordinates — the MSCKF camera
    /// update's R. Default matches the updater's historical hardcoded value.
    double camera_noise_normalized = 1.0e-2;
    /// S5 parallax gate: reject a triangulation whose maximum inter-view parallax
    /// is below this many degrees (degenerate depth — see the S5 probe). Default 0
    /// = disabled; ~2–5° trades feature coverage for depth quality and must be
    /// validated end-to-end before being turned on (docs/arch/vio-pipeline-canonical.md).
    double min_parallax_deg = 0.0;
    /// S10 calibration-uncertainty term: a camera↔IMU **extrinsic-rotation** σ (in
    /// degrees) folded into the visual measurement noise `R`, so the filter stops
    /// treating the calibration as perfectly known (the #212 over-confidence; the
    /// empirical "R×4 restores NEES" — see the S10 probe). Default 0 = disabled;
    /// ~1° (≈ the EuRoC extrinsic uncertainty) reproduces the R×4 variance
    /// inflation. Validate end-to-end before turning on
    /// (docs/arch/vio-pipeline-canonical.md §S10).
    double calib_ext_rot_sigma_deg = 0.0;
    int num_cameras = 1;  ///< 1 = mono, 2 = stereo
    int max_clones = 11;  ///< MSCKF sliding-window length
    /// Skip the stationary-window static init and bootstrap from the dynamic
    /// visual-inertial alignment instead. For a known moving start (the drone
    /// is already in motion at the first sample) the static check would only
    /// burn samples before timing out; preferring dynamic also lets the
    /// dynamic-init path be exercised on sequences that happen to contain an
    /// early quiet moment. The gravity-only timeout fallback still applies.
    bool prefer_dynamic_init = false;
};

/// A single inertial sample, in the IMU frame. Timestamps are seconds.
template <math::Scalar T>
struct ImuMeasurement {
    double timestamp_s = 0.0;
    Vec3<T> angular_velocity{};     ///< rad/s (gyroscope)
    Vec3<T> linear_acceleration{};  ///< m/s² (accelerometer, incl. gravity)
};

/// One feature observation from the front end. Backend-agnostic: it
/// carries a tracker-assigned feature id (stable across frames), the
/// camera it was seen in, and the *pixel* coordinates. Whether the
/// backend triangulates, anchors, or null-space-projects the feature is
/// entirely its own concern — the front end never sees that choice.
template <math::Scalar T>
struct FrontendObservation {
    std::uint64_t feature_id = 0;
    std::uint32_t camera_id = 0;
    T u = T{0};  ///< pixel column
    T v = T{0};  ///< pixel row
};

/// Estimated navigation state: the body/IMU pose in the world frame plus
/// world-frame velocity, stamped. `T_world_imu` maps a point in the IMU
/// frame to the world frame.
template <math::Scalar T>
struct NavState {
    double timestamp_s = 0.0;
    math::lie::SE3<T> T_world_imu{};
    Vec3<T> velocity_world{};
};

/// Abstract VIO estimator backend. Implementations consume the
/// front end's IMU and camera streams and maintain the navigation state.
///
/// Lifecycle: `initialize` once, then an interleaved stream of
/// `process_imu` (high rate) and `process_camera` (frame rate);
/// `current_state` is callable at any time after initialization.
template <math::Scalar T>
class VioBackend {
public:
    using Scalar = T;

    VioBackend() = default;
    VioBackend(const VioBackend&) = default;
    VioBackend(VioBackend&&) = default;
    VioBackend& operator=(const VioBackend&) = default;
    VioBackend& operator=(VioBackend&&) = default;
    virtual ~VioBackend() = default;

    /// Configure the backend before any measurement is processed.
    virtual void initialize(const VioConfig& config) = 0;

    /// Ingest one inertial sample (propagation / preintegration input).
    virtual void process_imu(const ImuMeasurement<T>& imu) = 0;

    /// Ingest one camera frame's feature observations at `timestamp_s`.
    /// The span is borrowed for the duration of the call only.
    virtual void process_camera(double timestamp_s, std::span<const FrontendObservation<T>> obs) = 0;

    /// The most recent navigation-state estimate.
    [[nodiscard]] virtual NavState<T> current_state() const = 0;
};

}  // namespace branes::sdk

#endif  // BRANES_SDK_VIO_BACKEND_HPP
