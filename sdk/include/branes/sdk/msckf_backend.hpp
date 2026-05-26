// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf_backend.hpp — the MSCKF VioBackend implementation.
//
// Wires the Phase-3 pieces into one estimator: IMU-based static
// initialization (ImuInitializer), per-sample mean+covariance propagation
// (Propagator), a sliding window of cloned poses (StateHelper), and the
// null-space camera measurement update (CameraUpdater). The front end
// feeds IMU and per-frame feature observations through the VioBackend
// interface; this class maintains the navigation state.
//
// Feature management is the standard MSCKF policy: every camera frame
// augments a clone; a feature is fed to the EKF update when its track
// ends (it is no longer observed) or when the oldest clone it touches is
// about to be marginalized to keep the window at `max_clones`.
//
// This is the full-covariance backend. A square-root covariance variant
// is a separate, larger effort tracked as a follow-up to issue #35.
//
// `process_camera` propagates the state to the frame timestamp (zero-order
// hold on the last IMU sample) before cloning, so each clone is anchored
// at exactly the image time even on an asynchronous IMU/camera feed.
//
// Observations are pixel coordinates per the VioBackend contract; each
// camera's model unprojects them to bearings, keeping the estimator core
// intrinsics-agnostic. Header-only, C++20.

#ifndef BRANES_SDK_MSCKF_BACKEND_HPP
#define BRANES_SDK_MSCKF_BACKEND_HPP

#include <branes/math/cameras/pinhole_radtan.hpp>
#include <branes/sdk/imu_init.hpp>
#include <branes/sdk/msckf.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace branes::sdk {

template <math::Scalar T>
class MsckfBackend final : public VioBackend<T> {
public:
    using Scalar = T;
    using Camera = math::cameras::PinholeRadtanCamera<T>;
    using Extrinsics = msckf::CameraExtrinsics<T>;
    using DVec3 = math::lie::detail::Vec<T, 3>;
    using DVec2 = math::lie::detail::Vec<T, 2>;

    /// Advertises that this backend performs real estimation (contrast
    /// with the SlidingWindowBackend skeleton, which is false).
    static constexpr bool kImplemented = true;

    /// A camera's calibration: its projection model (for pixel→bearing
    /// unprojection) and its pose in the IMU frame.
    struct CameraCalibration {
        Camera intrinsics{};
        Extrinsics extrinsics{};
    };

    /// Default: a single identity-pinhole camera coincident with the IMU.
    /// This is only correct when the observations are already normalized /
    /// rectified (tests, or a front end that pre-normalizes); production
    /// code must construct with the real per-camera `CameraCalibration`.
    MsckfBackend() : MsckfBackend(std::vector<CameraCalibration>{CameraCalibration{}}) {}

    /// Precondition: `cameras` is non-empty.
    explicit MsckfBackend(std::vector<CameraCalibration> cameras)
        : cameras_(std::move(cameras)), updater_(extrinsics_of(cameras_)) {
        if (cameras_.empty())
            throw std::invalid_argument("MsckfBackend: at least one camera calibration is required");
    }

    void initialize(const VioConfig& config) override {
        config_ = config;
        max_clones_ = config.max_clones > 0 ? static_cast<std::size_t>(config.max_clones) : 1;

        const msckf::ImuNoise<T> noise{static_cast<T>(config.gyro_noise_density),
                                       static_cast<T>(config.accel_noise_density),
                                       static_cast<T>(config.gyro_bias_random_walk),
                                       static_cast<T>(config.accel_bias_random_walk)};
        const DVec3 gravity{{T{0}, T{0}, -static_cast<T>(config.gravity_magnitude)}};
        prop_ = msckf::Propagator<T>(noise, gravity);

        ImuInitConfig<T> icfg;
        icfg.gravity_magnitude = static_cast<T>(config.gravity_magnitude);
        initializer_ = ImuInitializer<T>(icfg);

        state_ = msckf::State<T>(kInitialSigma());
        initialized_ = false;
        have_last_time_ = false;
        last_imu_time_ = 0.0;
        init_gyro_.clear();
        init_accel_.clear();
        tracks_.clear();
    }

    void process_imu(const ImuMeasurement<T>& imu) override {
        const DVec3 gyro = to_dvec(imu.angular_velocity);
        const DVec3 accel = to_dvec(imu.linear_acceleration);

        if (!initialized_) {
            try_initialize(gyro, accel, imu.timestamp_s);
            return;
        }
        if (have_last_time_) {
            const double dt = imu.timestamp_s - last_imu_time_;
            if (!(dt > 0.0))
                return;  // drop out-of-order / duplicate samples — don't touch filter time
            prop_.propagate(state_, gyro, accel, static_cast<T>(dt));
        }
        last_imu_time_ = imu.timestamp_s;
        have_last_time_ = true;
        state_.timestamp = imu.timestamp_s;
        last_gyro_ = gyro;  // held over to the next frame's image time
        last_accel_ = accel;
    }

    void process_camera(double timestamp_s, std::span<const FrontendObservation<T>> obs) override {
        if (!initialized_)
            return;

        // Propagate to the image time (zero-order hold on the last IMU
        // sample) so the clone is anchored at exactly `timestamp_s`, even
        // when the frame falls between IMU samples. Out-of-order frames are
        // ignored.
        if (have_last_time_) {
            const double dt = timestamp_s - last_imu_time_;
            if (dt < 0.0)
                return;
            if (dt > 0.0) {
                prop_.propagate(state_, last_gyro_, last_accel_, static_cast<T>(dt));
                last_imu_time_ = timestamp_s;
            }
        }

        // Keep the window bounded: marginalize (after updating) the oldest
        // clone(s) before adding the new one.
        while (state_.clones.size() >= max_clones_)
            marginalize_oldest();

        // Augment a clone at the (now frame-time) pose.
        state_.timestamp = timestamp_s;
        msckf::StateHelper<T>::augment_clone(state_);

        std::unordered_set<std::uint64_t> seen;
        seen.reserve(obs.size());
        for (const auto& o : obs) {
            DVec2 xy;
            if (!to_normalized(o.camera_id, o.u, o.v, xy))
                continue;  // behind the camera / bad calibration index
            tracks_[o.feature_id].push_back(ObsRec{timestamp_s, o.camera_id, xy});
            seen.insert(o.feature_id);
        }

        // Features not observed this frame have ended: feed them to the
        // filter and drop them.
        std::vector<std::uint64_t> ended;
        for (const auto& [id, recs] : tracks_)
            if (seen.find(id) == seen.end())
                ended.push_back(id);
        for (const std::uint64_t id : ended)
            update_and_erase(id);
    }

    [[nodiscard]] NavState<T> current_state() const override {
        NavState<T> ns;
        ns.timestamp_s = state_.timestamp;
        ns.T_world_imu = math::lie::SE3<T>(state_.R, state_.p);
        ns.velocity_world = Vec3<T>{{state_.v[0], state_.v[1], state_.v[2]}};
        return ns;
    }

    /// Whether initialization has completed (gravity/bias resolved).
    [[nodiscard]] bool initialized() const noexcept {
        return initialized_;
    }

    /// Read-only view of the full filter state (covariance, clone window,
    /// biases). For inspection, logging, and the replay harness (#46).
    [[nodiscard]] const msckf::State<T>& state() const noexcept {
        return state_;
    }

private:
    // One stored observation of a feature, tagged by the clone time it was
    // taken at (clone indices shift on marginalization; times do not).
    struct ObsRec {
        double clone_time = 0.0;
        std::uint32_t camera_id = 0;
        DVec2 xy{};
    };

    static constexpr T kInitialSigma() {
        return T{1} / T{10};
    }
    static constexpr std::size_t kInitSamples = 50;      ///< static-init window
    static constexpr std::size_t kInitMaxSamples = 400;  ///< fall back past this

    static std::vector<Extrinsics> extrinsics_of(const std::vector<CameraCalibration>& cams) {
        std::vector<Extrinsics> e;
        e.reserve(cams.size());
        for (const auto& c : cams)
            e.push_back(c.extrinsics);
        return e;
    }

    static DVec3 to_dvec(const Vec3<T>& a) {
        return DVec3{{a[0], a[1], a[2]}};
    }

    void try_initialize(const DVec3& gyro, const DVec3& accel, double t) {
        init_gyro_.push_back(gyro);
        init_accel_.push_back(accel);
        if (init_gyro_.size() < kInitSamples)
            return;

        const auto r = initializer_.try_static(std::span<const DVec3>{init_gyro_}, std::span<const DVec3>{init_accel_});
        if (r.success) {
            state_ = msckf::State<T>(kInitialSigma());
            state_.R = r.R_world_imu;
            state_.bg = r.gyro_bias;
            state_.ba = r.accel_bias;
            state_.timestamp = t;
            finish_init(t);
            return;
        }
        // Not stationary yet. Slide the window; if we never settle, fall
        // back to an identity attitude (a moving start needs dynamic VI
        // init, which the front end can drive via ImuInitializer).
        if (init_gyro_.size() >= kInitMaxSamples) {
            state_ = msckf::State<T>(kInitialSigma());
            state_.timestamp = t;
            finish_init(t);
        } else {
            init_gyro_.erase(init_gyro_.begin());
            init_accel_.erase(init_accel_.begin());
        }
    }

    void finish_init(double t) {
        initialized_ = true;
        have_last_time_ = true;
        last_imu_time_ = t;
        init_gyro_.clear();
        init_accel_.clear();
    }

    bool to_normalized(std::uint32_t cam_id, T u, T v, DVec2& out) const {
        if (cam_id >= cameras_.size())
            return false;  // unknown camera — reject rather than alias to camera 0
        const auto bearing = cameras_[cam_id].intrinsics.unproject(math::cameras::Vec2<T>{{u, v}});
        // unproject returns a +Z-forward unit bearing; normalize to the
        // image plane. A non-positive Z means the ray is not in front.
        if (!(bearing[2] > T{0}))
            return false;
        out = DVec2{{bearing[0] / bearing[2], bearing[1] / bearing[2]}};
        return true;
    }

    // Index of the clone taken at time `t`, or npos if it is gone.
    std::size_t clone_index_at(double t) const {
        for (std::size_t i = 0; i < state_.clones.size(); ++i)
            if (state_.clones[i].timestamp == t)
                return i;
        return kNoClone;
    }

    // Build the updater's track for feature `id` from the observations
    // whose clone is still in the window, run the EKF update if it has
    // enough of them, then forget the feature.
    void update_and_erase(std::uint64_t id) {
        auto it = tracks_.find(id);
        if (it == tracks_.end())
            return;
        msckf::FeatureTrack<T> track;
        track.observations.reserve(it->second.size());
        for (const ObsRec& rec : it->second) {
            const std::size_t ci = clone_index_at(rec.clone_time);
            if (ci != kNoClone)
                track.observations.push_back({ci, rec.camera_id, rec.xy});
        }
        if (track.observations.size() >= 2)
            updater_.update(state_, track);
        tracks_.erase(it);
    }

    // Update with every feature observed at the oldest clone (so its
    // information is used), drop that clone, and purge its stale rows.
    void marginalize_oldest() {
        if (state_.clones.empty())
            return;
        const double old_time = state_.clones.front().timestamp;

        std::vector<std::uint64_t> touching;
        for (const auto& [id, recs] : tracks_)
            for (const ObsRec& rec : recs)
                if (rec.clone_time == old_time) {
                    touching.push_back(id);
                    break;
                }
        for (const std::uint64_t id : touching)
            update_and_erase(id);

        msckf::StateHelper<T>::marginalize_clone(state_, 0);

        // Any surviving track that still references the dropped clone loses
        // that observation; remove now-empty tracks.
        for (auto it = tracks_.begin(); it != tracks_.end();) {
            auto& recs = it->second;
            for (auto r = recs.begin(); r != recs.end();)
                r = (r->clone_time == old_time) ? recs.erase(r) : r + 1;
            it = recs.empty() ? tracks_.erase(it) : std::next(it);
        }
    }

    static constexpr std::size_t kNoClone = static_cast<std::size_t>(-1);

    std::vector<CameraCalibration> cameras_;
    msckf::CameraUpdater<T> updater_;
    msckf::Propagator<T> prop_{};
    ImuInitializer<T> initializer_{};
    msckf::State<T> state_{kInitialSigma()};
    VioConfig config_{};
    std::size_t max_clones_ = 11;

    bool initialized_ = false;
    bool have_last_time_ = false;
    double last_imu_time_ = 0.0;
    DVec3 last_gyro_{};  ///< most recent IMU sample, held over to the image time
    DVec3 last_accel_{};
    std::vector<DVec3> init_gyro_;
    std::vector<DVec3> init_accel_;
    std::unordered_map<std::uint64_t, std::vector<ObsRec>> tracks_;
};

}  // namespace branes::sdk

#endif  // BRANES_SDK_MSCKF_BACKEND_HPP
