// SPDX-License-Identifier: MIT
//
// branes/sdk/vio_estimator.hpp — the top-level VIO estimator.
//
// VioEstimator is the public façade the daemons drive: it owns the visual
// front end (FAST detection + pyramidal KLT tracking, producing stable
// per-feature observations) and the selected estimator backend (MSCKF by
// default; any VioBackend works), and exposes a small lifecycle-gated API:
//
//     configure → activate → (feed_imu / feed_image)* → deactivate / reset
//
// The lifecycle mirrors the Rust Resource Manager's managed states
// (Unconfigured → Inactive → Active → Teardown); measurements are only
// consumed while Active, matching the "no dynamic reconfiguration on the
// hot path" rule — parameters are fixed at `configure`.
//
// Inputs are non-owning views (an Image view for frames, a span for IMU
// batches); the estimator copies nothing it does not need to retain.
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_VIO_ESTIMATOR_HPP
#define BRANES_SDK_VIO_ESTIMATOR_HPP

#include <branes/cv/fast.hpp>
#include <branes/cv/image.hpp>
#include <branes/cv/klt.hpp>
#include <branes/cv/pyramid.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace branes::sdk {

// FrontendParams: msckf/stages/s4_frontend.hpp (the S4 front-end stage, #456).

template <math::Scalar T, class Backend = MsckfBackend<T>>
class VioEstimator {
public:
    using Scalar = T;
    using Pose = math::lie::SE3<T>;
    using Pixel = std::uint8_t;

    /// Managed lifecycle, mirroring the Rust RM's states.
    enum class Lifecycle { Unconfigured, Inactive, Active, Teardown };

    VioEstimator() = default;

    /// Construct with a pre-built backend (e.g. one carrying real camera
    /// calibration). The backend is still (re)initialized at `configure`.
    explicit VioEstimator(Backend backend, const FrontendParams& fe = {}) : backend_(std::move(backend)), fe_(fe) {}

    /// Unconfigured/Inactive → Inactive. Fixes parameters and initializes
    /// the backend. From Inactive it acts as a reconfigure, which also clears
    /// runtime state. Ignored while Active (parameters are immutable while
    /// processing — deactivate() first; ADR-0005) and after Teardown.
    void configure(const VioConfig& config) {
        if (state_ == Lifecycle::Teardown || state_ == Lifecycle::Active)
            return;
        config_ = config;
        backend_.initialize(config);
        reset_frontend();
        state_ = Lifecycle::Inactive;
    }

    /// Inactive → Active. Begins consuming measurements.
    void activate() {
        if (state_ == Lifecycle::Inactive)
            state_ = Lifecycle::Active;
    }

    /// Active → Inactive. Stops consuming measurements; keeps the estimate.
    void deactivate() {
        if (state_ == Lifecycle::Active)
            state_ = Lifecycle::Inactive;
    }

    /// → Teardown (terminal). No further measurements are accepted.
    void teardown() {
        state_ = Lifecycle::Teardown;
    }

    [[nodiscard]] Lifecycle lifecycle() const noexcept {
        return state_;
    }

    /// Feed a batch of inertial samples (ascending timestamps). Ignored
    /// unless Active.
    void feed_imu(std::span<const ImuMeasurement<T>> samples) {
        if (state_ != Lifecycle::Active)
            return;
        for (const auto& s : samples)
            backend_.process_imu(s);
    }

    /// Feed one grayscale frame at `timestamp_s`. Runs the front end and
    /// hands the resulting observations to the backend. Ignored unless
    /// Active. The image is borrowed for the call only.
    void feed_image(double timestamp_s, cv::Image<const Pixel> image) {
        if (state_ != Lifecycle::Active || image.empty())
            return;
        // S4: the image-domain front end, then the backend.
        const auto tr = msckf::stages::s4_frontend::track<T>(frontend_, image, fe_);
        backend_.process_camera(timestamp_s, std::span<const FrontendObservation<T>>{tr.observations});
    }

    /// Current body pose in the world frame (T_world_imu).
    [[nodiscard]] Pose current_pose() const {
        return backend_.current_state().T_world_imu;
    }

    /// Full navigation state (pose + velocity + stamp).
    [[nodiscard]] NavState<T> current_state() const {
        return backend_.current_state();
    }

    /// Clear the estimate and the front end, keeping the configuration.
    /// Returns to Inactive (re-activate to resume).
    void reset() {
        if (state_ == Lifecycle::Unconfigured || state_ == Lifecycle::Teardown)
            return;
        backend_.initialize(config_);
        reset_frontend();
        state_ = Lifecycle::Inactive;
    }

    [[nodiscard]] const Backend& backend() const noexcept {
        return backend_;
    }

    /// Mutable backend access — for installing inspection hooks (e.g. the S6
    /// update observer, #380) before a replay. Not for use on the hot path.
    [[nodiscard]] Backend& backend() noexcept {
        return backend_;
    }

    /// Number of features the front end is currently tracking (telemetry).
    [[nodiscard]] std::size_t num_tracked_features() const noexcept {
        return frontend_.tracks.size();
    }

    /// One tracked feature for visualization: stable id + current pixel position.
    struct TrackedFeature {
        std::uint64_t id = 0;
        float x = 0.0f;
        float y = 0.0f;
    };

    /// The front end's current tracked features (id + pixel), valid after the
    /// last `feed_image`. For overlaying the live tracks on the scene video.
    [[nodiscard]] std::vector<TrackedFeature> tracked_features() const {
        std::vector<TrackedFeature> out;
        out.reserve(frontend_.tracks.size());
        for (const auto& t : frontend_.tracks)
            out.push_back(TrackedFeature{t.id, t.x, t.y});
        return out;
    }

private:
    void reset_frontend() {
        frontend_ = msckf::stages::s4_frontend::FrontendState{};
    }

    Backend backend_{};
    FrontendParams fe_{};
    VioConfig config_{};
    Lifecycle state_ = Lifecycle::Unconfigured;

    msckf::stages::s4_frontend::FrontendState frontend_{};  ///< S4 state: previous pyramid + live tracks
};

}  // namespace branes::sdk

#endif  // BRANES_SDK_VIO_ESTIMATOR_HPP
