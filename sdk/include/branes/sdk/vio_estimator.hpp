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

/// Front-end tuning. Defaults target a ~480p grayscale stream.
struct FrontendParams {
    int pyramid_levels = 3;
    double fast_threshold = 20.0;        ///< FAST-9 contrast threshold
    std::size_t target_features = 150;   ///< re-detect when tracked count drops below
    double min_feature_distance = 15.0;  ///< px; suppress detections near existing tracks
    /// S4 forward-backward gate: re-track each survivor back to the previous frame
    /// and drop it if the round-trip error exceeds this (px). Rejects tracks that
    /// drifted to a confidently-wrong location before they reach the backend.
    /// **Default 0 (disabled)**: enabling it (e.g. ~1 px) doubles the KLT cost and
    /// should be validated end-to-end before being turned on.
    double fb_max_residual = 0.0;
    cv::KltParams klt{};
};

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
    /// the backend. Allowed from any non-teardown state (acts as a
    /// reconfigure, which also clears runtime state).
    void configure(const VioConfig& config) {
        if (state_ == Lifecycle::Teardown)
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
        const auto obs = track_frame(image);
        backend_.process_camera(timestamp_s, std::span<const FrontendObservation<T>>{obs});
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
        return tracks_.size();
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
        out.reserve(tracks_.size());
        for (const auto& t : tracks_)
            out.push_back(TrackedFeature{t.id, t.x, t.y});
        return out;
    }

private:
    struct Track {
        std::uint64_t id = 0;
        float x = 0.0f;
        float y = 0.0f;
    };

    void reset_frontend() {
        prev_pyramid_ = cv::Pyramid<Pixel>{};
        have_prev_ = false;
        tracks_.clear();
        next_id_ = 0;
    }

    // Track existing features into `image`, replenish with new FAST
    // detections, and return one observation per surviving track.
    std::vector<FrontendObservation<T>> track_frame(cv::Image<const Pixel> image) {
        // At least one level, so level(0) is always valid even if a caller
        // sets a non-positive pyramid_levels.
        cv::Pyramid<Pixel> next(image, std::max(1, fe_.pyramid_levels));

        if (have_prev_ && !tracks_.empty()) {
            std::vector<cv::KeyPoint> pts;
            pts.reserve(tracks_.size());
            for (const auto& t : tracks_)
                pts.push_back(cv::KeyPoint{t.x, t.y, 0.0f});
            const auto res = cv::track_klt_pyramidal(prev_pyramid_, next, pts, fe_.klt);

            // Forward-backward consistency: re-track the forward survivors back to
            // the previous frame; a self-consistent track returns to its origin.
            std::vector<cv::KeyPoint> back_pts;
            std::vector<cv::KltResult> back;
            if (fe_.fb_max_residual > 0.0) {
                back_pts.reserve(res.size());
                for (const auto& rr : res)
                    back_pts.push_back(cv::KeyPoint{rr.x, rr.y, 0.0f});
                back = cv::track_klt_pyramidal(next, prev_pyramid_, back_pts, fe_.klt);
            }
            const double fb2 = fe_.fb_max_residual * fe_.fb_max_residual;

            std::vector<Track> kept;
            kept.reserve(tracks_.size());
            for (std::size_t i = 0; i < res.size(); ++i) {
                if (res[i].status != cv::TrackStatus::Tracked)
                    continue;
                if (fe_.fb_max_residual > 0.0) {
                    if (back[i].status != cv::TrackStatus::Tracked)
                        continue;  // can't verify the round trip → drop
                    const double dx = static_cast<double>(back[i].x) - pts[i].x;
                    const double dy = static_cast<double>(back[i].y) - pts[i].y;
                    if (dx * dx + dy * dy > fb2)
                        continue;  // failed forward-backward → drop the outlier
                }
                Track t = tracks_[i];
                t.x = res[i].x;
                t.y = res[i].y;
                kept.push_back(t);
            }
            tracks_ = std::move(kept);
        }

        if (tracks_.size() < fe_.target_features)
            detect_new(next.level(0));

        std::vector<FrontendObservation<T>> obs;
        obs.reserve(tracks_.size());
        for (const auto& t : tracks_) {
            FrontendObservation<T> o;
            o.feature_id = t.id;
            o.camera_id = 0;
            o.u = static_cast<T>(t.x);
            o.v = static_cast<T>(t.y);
            obs.push_back(o);
        }

        prev_pyramid_ = std::move(next);
        have_prev_ = true;
        return obs;
    }

    // Add the strongest FAST corners that are far enough from every
    // existing track, up to the target count.
    void detect_new(cv::Image<const Pixel> level0) {
        auto kps = cv::detect_fast(level0, fe_.fast_threshold);
        std::sort(kps.begin(), kps.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
            return a.response > b.response;
        });
        const double min_d2 = fe_.min_feature_distance * fe_.min_feature_distance;
        for (const auto& kp : kps) {
            if (tracks_.size() >= fe_.target_features)
                break;
            bool too_close = false;
            for (const auto& t : tracks_) {
                const double dx = static_cast<double>(kp.x) - t.x;
                const double dy = static_cast<double>(kp.y) - t.y;
                if (dx * dx + dy * dy < min_d2) {
                    too_close = true;
                    break;
                }
            }
            if (!too_close)
                tracks_.push_back(Track{next_id_++, kp.x, kp.y});
        }
    }

    Backend backend_{};
    FrontendParams fe_{};
    VioConfig config_{};
    Lifecycle state_ = Lifecycle::Unconfigured;

    cv::Pyramid<Pixel> prev_pyramid_{};
    bool have_prev_ = false;
    std::vector<Track> tracks_;
    std::uint64_t next_id_ = 0;
};

}  // namespace branes::sdk

#endif  // BRANES_SDK_VIO_ESTIMATOR_HPP
