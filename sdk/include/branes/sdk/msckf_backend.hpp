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
#include <branes/sdk/eval/consistency.hpp>
#include <branes/sdk/eval/innovation_whiteness.hpp>
#include <branes/sdk/imu_init.hpp>
#include <branes/sdk/imu_preintegration.hpp>
#include <branes/sdk/msckf.hpp>
#include <branes/sdk/sfm/init_window.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace branes::sdk {

/// How the backend resolved its initial attitude/bias. Surfacing this is the
/// single most useful divergence-triage signal: a tilted real-world start that
/// silently lands on `Identity` injects a phantom ~g acceleration and the
/// filter diverges within seconds.
enum class InitMethod {
    None,          ///< not initialized yet
    Static,        ///< stationary window: full roll/pitch + gyro bias
    Dynamic,       ///< moving start: full visual-inertial alignment (yaw + scale + velocity)
    GravityAlign,  ///< moving start: roll/pitch from mean specific force only
    Identity,      ///< last resort: mean force wasn't even gravity-like
};

[[nodiscard]] inline const char* to_string(InitMethod m) noexcept {
    switch (m) {
    case InitMethod::Static:
        return "static";
    case InitMethod::Dynamic:
        return "dynamic";
    case InitMethod::GravityAlign:
        return "gravity_align";
    case InitMethod::Identity:
        return "identity";
    case InitMethod::None:
        break;
    }
    return "none";
}

/// Read-only record of how initialization went (telemetry, like `initialized`).
template <math::Scalar T>
struct InitDiagnostics {
    using DVec3 = math::lie::detail::Vec<T, 3>;
    InitMethod method = InitMethod::None;
    double t_s = 0.0;               ///< filter time at which init completed
    DVec3 up_body{};                ///< measured "up" (specific-force) direction in the body frame
    double gravity_residual = 0.0;  ///< ||mean accel| − g| / g over the init window
    DVec3 gyro_bias{};
    DVec3 accel_bias{};
    // Dynamic-path diagnostics (only meaningful when method == Dynamic): the
    // recovered metric scale of the vision poses, the speed the filter is
    // seeded with (|v_world| of the last keyframe), and the number of SfM
    // keyframes the alignment used. Surfaced to localize a bad dynamic seed.
    double dyn_scale = 0.0;
    double dyn_seed_speed = 0.0;
    int dyn_keyframes = 0;
    // Dynamic-init attempt accounting, accumulated across every try while
    // uninitialized — populated even when dynamic never fires, to localize why
    // (#247). dyn_attempts: tries past kMinDynFrames; dyn_window_builds: how
    // many produced a valid SfM window (else the two-view/PnP bootstrap is
    // failing on real tracks); dyn_best_keyframes / dyn_best_metric_motion: the
    // best window size and resolved metric motion (scale · displacement) seen —
    // if builds succeed but the motion stays below min_dynamic_motion, the
    // vision trajectory is inconsistent with the IMU (scale collapses).
    int dyn_attempts = 0;
    int dyn_window_builds = 0;
    int dyn_best_keyframes = 0;
    double dyn_best_metric_motion = 0.0;
    // Roll/pitch sanity for a dynamic seed (#247): angle (deg) between the
    // seed's world-up expressed in the body frame and the accelerometer's
    // mean specific-force direction over the init window. Large ⇒ the
    // VI-solved gravity DIRECTION is wrong (under-observable on a low-excitation
    // window), injecting phantom horizontal gravity → divergence.
    double dyn_tilt_vs_accel_deg = 0.0;
};

/// FEJ engagement telemetry (#280): how far each clone's mean pose drifts from
/// its frozen first-estimate (R_fej, p_fej) over its lifetime — the magnitude the
/// First-Estimates Jacobian actually linearizes away from. Sampled when a clone is
/// marginalized (its full age). A tiny divergence with a large effect on
/// consistency would point at a bug; a large divergence makes the FEJ linearization
/// gap real.
struct FejDivergence {
    double rot_deg_mean = 0.0;
    double rot_deg_max = 0.0;
    double trans_mean = 0.0;
    double trans_max = 0.0;
    std::size_t samples = 0;
};

/// The MSCKF backend, generic over the covariance representation `Cov`
/// (`FullCovariance` — Joseph-form, the default; or `SqrtCovariance` — the
/// QR array update). The mean, IMU init, feature management, and window
/// policy are identical for either; only the covariance algebra differs.
/// Use the `MsckfBackend<T>` / `MsckfSqrtBackend<T>` aliases below.
template <math::Scalar T, msckf::CovarianceModel<T> Cov = msckf::FullCovariance<T>>
class MsckfBackendT final : public VioBackend<T> {
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
    MsckfBackendT() : MsckfBackendT(std::vector<CameraCalibration>{CameraCalibration{}}) {}

    /// Precondition: `cameras` is non-empty.
    explicit MsckfBackendT(std::vector<CameraCalibration> cameras)
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

        // Wire the visual measurement noise R from config so it is tunable
        // alongside the IMU process noise (the camera update's normalized σ).
        typename msckf::CameraUpdater<T>::Options uopts;
        uopts.normalized_sigma = static_cast<T>(config.camera_noise_normalized);
        uopts.first_estimates_jacobians = config.use_fej;
        updater_ = msckf::CameraUpdater<T>(extrinsics_of(cameras_), uopts);

        ImuInitConfig<T> icfg;
        icfg.gravity_magnitude = static_cast<T>(config.gravity_magnitude);
        initializer_ = ImuInitializer<T>(icfg);

        state_ = msckf::State<T, Cov>(kInitialSigma());
        init_diag_ = InitDiagnostics<T>{};
        initialized_ = false;
        have_last_time_ = false;
        last_imu_time_ = 0.0;
        init_gyro_.clear();
        init_accel_.clear();
        init_samples_seen_ = 0;
        init_frames_.clear();
        init_preint_ = ImuPreintegrator<T>{};
        init_have_imu_t_ = false;
        init_last_imu_t_ = 0.0;
        tracks_.clear();
        nis_ = eval::ConsistencyAccumulator{};
        innov_ = eval::InnovationWhitenessAccumulator{};
        fej_rot_sum_ = fej_rot_max_ = fej_trans_sum_ = fej_trans_max_ = 0.0;
        fej_n_ = 0;
    }

    void process_imu(const ImuMeasurement<T>& imu) override {
        const DVec3 gyro = to_dvec(imu.angular_velocity);
        const DVec3 accel = to_dvec(imu.linear_acceleration);

        if (!initialized_) {
            // Accumulate IMU preintegration for the dynamic-init window (the
            // deltas between successive camera frames). Bias linearization at
            // zero; try_dynamic corrects via dR_dbg.
            if (init_have_imu_t_) {
                const double dt = imu.timestamp_s - init_last_imu_t_;
                if (dt > 0.0)
                    init_preint_.integrate(gyro, accel, static_cast<T>(dt));
            }
            init_last_imu_t_ = imu.timestamp_s;
            init_have_imu_t_ = true;
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
        if (!initialized_) {
            // Drop duplicate / out-of-order frames: a non-increasing timestamp
            // would corrupt the SfM + preintegration window (the post-init path
            // guards this too).
            if (!init_frames_.empty() && !(timestamp_s > init_frames_.back().timestamp))
                return;
            // Buffer the frame for the dynamic-init SfM window and attempt the
            // visual-inertial alignment once enough frames have accumulated.
            buffer_init_frame(timestamp_s, obs);
            try_dynamic_init(timestamp_s);
            return;
        }

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
        // Process in a deterministic (feature-id) order: tracks_ is an
        // unordered_map, so its iteration order is hash-driven and varies across
        // STL implementations. Sorting keeps the EKF update sequence — and the
        // order-dependent innovation-whiteness lag-1 telemetry — reproducible.
        std::sort(ended.begin(), ended.end());
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

    /// How initialization resolved the initial attitude/bias. `method` is
    /// `None` until `initialized()` is true. The prime divergence-triage
    /// signal: `Identity` (or a large `gravity_residual`) on real data means
    /// the attitude seed is wrong and the filter will diverge.
    [[nodiscard]] const InitDiagnostics<T>& init_diagnostics() const noexcept {
        return init_diag_;
    }

    /// Filter-consistency telemetry: every camera update's NIS (νᵀS⁻¹ν) is
    /// accumulated here. `nis_consistency().report()` gives the chi-square
    /// run-consistency verdict — a normalized average persistently > 1 means the
    /// filter is over-confident (covariance too small / a wrong Jacobian / a
    /// biased residual); < 1 means under-confident. The always-on, ground-truth-
    /// free half of the consistency instrument (#264).
    [[nodiscard]] const eval::ConsistencyAccumulator& nis_consistency() const noexcept {
        return nis_;
    }

    /// Innovation zero-mean + whiteness telemetry (#280 discriminator): tests the
    /// *direction* and *temporal structure* of the innovation, which NIS (a
    /// magnitude) cannot. A biased mean ⇒ a systematic error (extrinsics /
    /// triangulation / time-sync); temporal correlation ⇒ unmodelled dynamics or
    /// an observability/linearization inconsistency.
    [[nodiscard]] const eval::InnovationWhitenessAccumulator& innovation_whiteness() const noexcept {
        return innov_;
    }

    /// FEJ engagement telemetry (#280): the run's clone mean-vs-first-estimate
    /// pose divergence, averaged and peaked over marginalized clones.
    [[nodiscard]] FejDivergence fej_divergence() const noexcept {
        FejDivergence d;
        d.samples = fej_n_;
        if (fej_n_ > 0) {
            d.rot_deg_mean = fej_rot_sum_ / static_cast<double>(fej_n_);
            d.trans_mean = fej_trans_sum_ / static_cast<double>(fej_n_);
        }
        d.rot_deg_max = fej_rot_max_;
        d.trans_max = fej_trans_max_;
        return d;
    }

    /// Read-only view of the full filter state (covariance, clone window,
    /// biases). For inspection, logging, and the replay harness (#46).
    [[nodiscard]] const msckf::State<T, Cov>& state() const noexcept {
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
    static constexpr std::size_t kInitMaxSamples = 400;  ///< fall back past this (~2 s @ 200 Hz)
    /// Longer runway before the gravity-only fallback when the caller prefers
    /// the dynamic path: a moving start may hover briefly before the motion
    /// that makes the metric scale observable, so give dynamic init time to
    /// fire on a later excited window instead of preempting it at 2 s.
    static constexpr std::size_t kInitMaxSamplesDynamic = 4000;  ///< ~20 s @ 200 Hz
    static constexpr std::size_t kMinDynFrames = 5;              ///< camera frames before attempting dynamic init
    // Bounded init-window frame buffer. Wider spans more time, so even a slow
    // moving start accumulates enough translational baseline for the two-view
    // gravity/scale solve to be well-observable: a 12-frame (0.55 s @ 20 Hz)
    // window on MH_05's ~0.1 m/s start gave only ~5 cm of motion and an
    // under-observable gravity direction → divergence (#247). Bounded at 20 so
    // the per-frame init retry (a two-view solve + one PnP per later frame)
    // stays cheap if dynamic init keeps declining until the timeout.
    static constexpr std::size_t kMaxDynFrames = 20;  ///< ~1 s @ 20 Hz

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
        ++init_samples_seen_;
        // Slide a fixed window of the most recent kInitSamples: the static
        // check wants the latest stretch (most likely to catch a quiet
        // moment), while init_samples_seen_ tracks total elapsed samples for
        // the timeout below. (Trimming on every miss instead — the previous
        // behaviour — pinned the window at kInitSamples so it never reached
        // kInitMaxSamples and the fallback was unreachable.)
        if (init_gyro_.size() > kInitSamples) {
            init_gyro_.erase(init_gyro_.begin());
            init_accel_.erase(init_accel_.begin());
        }
        if (init_gyro_.size() < kInitSamples)
            return;

        const std::span<const DVec3> g{init_gyro_}, a{init_accel_};
        // prefer_dynamic_init suppresses the static path so the dynamic VI
        // alignment (driven from process_camera) bootstraps the filter even
        // when an early quiet window exists. The gravity-only timeout fallback
        // below still applies as a safety net if dynamic never fires.
        if (!config_.prefer_dynamic_init) {
            const auto stat = initializer_.try_static(g, a);
            if (stat.success) {
                apply_init(stat, InitMethod::Static, t);
                return;
            }
        }
        // No stationary window after kInitMaxSamples of trying: bootstrap from
        // the most recent window without waiting for a rest that may never
        // come (e.g. a sequence airborne from the first sample).
        //
        // The proper estimator for a moving start is full visual-inertial
        // alignment (ImuInitializer::try_dynamic), which also recovers yaw,
        // velocity, and scale — but it needs vision-estimated keyframe poses
        // that require an SfM/two-view bootstrap the front end does not
        // produce yet. Until that lands, gravity-only alignment fixes the
        // dominant error (roll/pitch), which is enough to keep the filter
        // from diverging. See the dynamic-VI-init follow-up.
        const std::size_t init_max = config_.prefer_dynamic_init ? kInitMaxSamplesDynamic : kInitMaxSamples;
        if (init_samples_seen_ >= init_max) {
            // Prefer gravity-only alignment (roll/pitch from the mean
            // specific force) over a blind identity attitude: on a tilted
            // mount, identity leaves gravity uncancelled in propagation and
            // the position diverges quadratically. Identity is the last
            // resort only when the mean force isn't even gravity-like.
            const auto grav = initializer_.try_gravity_align(g, a);
            if (grav.success)
                apply_init(grav, InitMethod::GravityAlign, t);
            else
                apply_init(ImuInitResult<T>{}, InitMethod::Identity, t);
        }
    }

    // Seed the state from an init result, record the diagnostics (measured
    // up-direction + gravity residual over the window), and finish.
    void apply_init(const ImuInitResult<T>& res, InitMethod method, double t) {
        state_ = msckf::State<T, Cov>(kInitialSigma());
        state_.R = res.R_world_imu;  // identity for a default-constructed result
        state_.bg = res.gyro_bias;
        state_.ba = res.accel_bias;
        state_.timestamp = t;

        DVec3 am{};
        for (const DVec3& s : init_accel_)
            am = am + s;
        if (!init_accel_.empty())
            am = am * (T{1} / static_cast<T>(init_accel_.size()));
        const T g_meas = math::lie::detail::norm(am);
        const T g_cfg = static_cast<T>(config_.gravity_magnitude);
        init_diag_.method = method;
        init_diag_.t_s = t;
        init_diag_.up_body = g_meas > T{0} ? am * (T{1} / g_meas) : DVec3{};
        init_diag_.gravity_residual =
            (g_meas > T{0} && g_cfg > T{0}) ? math::lie::detail::abs_(g_meas - g_cfg) / g_cfg : T{1};
        init_diag_.gyro_bias = res.gyro_bias;
        init_diag_.accel_bias = res.accel_bias;
        finish_init(t);
    }

    void finish_init(double t) {
        initialized_ = true;
        have_last_time_ = true;
        last_imu_time_ = t;
        init_gyro_.clear();
        init_accel_.clear();
        init_frames_.clear();
    }

    // Record one init-window camera frame (camera 0 only — the bootstrap is
    // single-camera) with its normalized observations and the IMU
    // preintegration accumulated since the previous frame, then reset the
    // preintegrator. A bounded sliding buffer; dropping the oldest is harmless
    // because the SfM window re-anchors on its first frame.
    void buffer_init_frame(double t, std::span<const FrontendObservation<T>> obs) {
        sfm::InitFrame<T> frame;
        frame.timestamp = t;
        for (const auto& o : obs) {
            if (o.camera_id != 0)
                continue;
            DVec2 xy;
            if (!to_normalized(o.camera_id, o.u, o.v, xy))
                continue;
            frame.ids.push_back(o.feature_id);
            frame.obs.push_back(sfm::Vec2<T>{{xy[0], xy[1]}});
        }
        if (!init_frames_.empty()) {  // the first frame has no preintegration
            frame.dR = init_preint_.delta_rotation();
            frame.dv = init_preint_.delta_velocity();
            frame.dp = init_preint_.delta_position();
            frame.dR_dbg = init_preint_.d_rotation_d_gyro_bias();
            frame.dt = init_preint_.delta_time();
        }
        init_preint_ = ImuPreintegrator<T>{};  // start the next interval
        init_frames_.push_back(std::move(frame));
        while (init_frames_.size() > kMaxDynFrames)
            init_frames_.erase(init_frames_.begin());
    }

    // Once enough frames have accumulated, build the SfM window and run the
    // visual-inertial alignment; on success seed the state from it. Preferred
    // over the gravity-only fallback, which only fires later (kInitMaxSamples)
    // and only while still uninitialized.
    void try_dynamic_init(double t) {
        if (init_frames_.size() < kMinDynFrames)
            return;
        ++init_diag_.dyn_attempts;
        const auto win =
            sfm::build_init_window<T>(std::span<const sfm::InitFrame<T>>{init_frames_}, cameras_[0].extrinsics);
        if (!win.success)
            return;  // two-view/PnP bootstrap failed on these tracks
        ++init_diag_.dyn_window_builds;
        if (static_cast<int>(win.keyframes.size()) > init_diag_.dyn_best_keyframes)
            init_diag_.dyn_best_keyframes = static_cast<int>(win.keyframes.size());
        const auto r = initializer_.try_dynamic(win.keyframes);
        // Record the resolved metric motion even on decline, so a run shows how
        // close the best window came to the observability floor (#247).
        if (static_cast<double>(r.resolved_motion) > init_diag_.dyn_best_metric_motion)
            init_diag_.dyn_best_metric_motion = static_cast<double>(r.resolved_motion);
        if (!r.success)
            return;
        apply_dynamic_init(r, win, t);
    }

    // Seed the state from a successful dynamic alignment. try_dynamic resolves
    // gravity, metric per-keyframe velocity, scale, and the gravity-aligned
    // attitude of the *first* keyframe; carry the vision relative rotation to
    // the last keyframe (the current pose) and take its velocity.
    void apply_dynamic_init(const ImuInitResult<T>& r, const sfm::InitWindowResult<T>& win, double t) {
        state_ = msckf::State<T, Cov>(kInitialSigma());
        const auto rel = win.keyframes.front().R_world_imu.inverse() * win.keyframes.back().R_world_imu;
        state_.R = r.R_world_imu * rel;
        state_.v = r.velocities_world.empty() ? DVec3{} : r.velocities_world.back();
        state_.bg = r.gyro_bias;
        state_.ba = DVec3{};  // accel bias is not observable from the dynamic alignment
        state_.timestamp = t;

        const T g_norm = math::lie::detail::norm(r.gravity_world);
        const T g_cfg = static_cast<T>(config_.gravity_magnitude);
        const DVec3 up_world = g_norm > T{0} ? r.gravity_world * (-T{1} / g_norm) : DVec3{};
        init_diag_.method = InitMethod::Dynamic;
        init_diag_.t_s = t;
        init_diag_.up_body = state_.R.inverse() * up_world;  // "up" in the current body frame
        init_diag_.gravity_residual =
            (g_norm > T{0} && g_cfg > T{0}) ? math::lie::detail::abs_(g_norm - g_cfg) / g_cfg : T{1};
        init_diag_.gyro_bias = r.gyro_bias;
        init_diag_.accel_bias = DVec3{};
        // Roll/pitch sanity: how far the seed's up disagrees with the mean
        // accelerometer up over the init window (gravity-dominated). Large ⇒
        // the VI-solved gravity direction is wrong → divergence (#247).
        if (!init_accel_.empty()) {
            namespace ld = math::lie::detail;
            DVec3 am{};
            for (const auto& a : init_accel_)
                am = am + a;
            const T amn = ld::norm(am);
            const T ubn = ld::norm(init_diag_.up_body);
            if (amn > T{0} && ubn > T{0}) {
                const DVec3 au = am * (T{1} / amn);                  // accelerometer up (body)
                const DVec3 su = init_diag_.up_body * (T{1} / ubn);  // seed up (body)
                // angle(u,v) = atan2(|u×v|, u·v) — robust near 0 and π.
                const T ang = ld::atan2_(ld::norm(ld::cross(au, su)), ld::dot(au, su));
                init_diag_.dyn_tilt_vs_accel_deg = static_cast<double>(ang) * 180.0 / 3.14159265358979323846;
            }
        }
        init_diag_.dyn_scale = static_cast<double>(r.scale);
        init_diag_.dyn_seed_speed = static_cast<double>(math::lie::detail::norm(state_.v));
        init_diag_.dyn_keyframes = static_cast<int>(win.keyframes.size());
        finish_init(t);
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
        if (track.observations.size() >= 2) {
            msckf::NisSample<T> ns;
            updater_.update(state_, track, &ns);
            if (ns.valid) {  // record the innovation NIS even if it was gated out
                nis_.add(static_cast<double>(ns.value), static_cast<int>(ns.dof));
                innov_.add(static_cast<double>(ns.innov_sum), ns.dof);
            }
        }
        tracks_.erase(it);
    }

    // Update with every feature observed at the oldest clone (so its
    // information is used), drop that clone, and purge its stale rows.
    // Accumulate one clone's mean-vs-first-estimate divergence (#280).
    void sample_fej_divergence(const typename msckf::State<T, Cov>::Clone& c) {
        const double ang_deg = static_cast<double>(math::lie::detail::norm((c.R_fej.inverse() * c.R).log())) * 180.0 /
                               3.14159265358979323846;
        const double trans = static_cast<double>(math::lie::detail::norm(c.p - c.p_fej));
        fej_rot_sum_ += ang_deg;
        fej_trans_sum_ += trans;
        fej_rot_max_ = std::max(fej_rot_max_, ang_deg);
        fej_trans_max_ = std::max(fej_trans_max_, trans);
        ++fej_n_;
    }

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
        std::sort(touching.begin(), touching.end());  // deterministic update order (see process_camera)
        for (const std::uint64_t id : touching)
            update_and_erase(id);

        sample_fej_divergence(state_.clones.front());  // FEJ engagement, at full clone age (#280)
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
    eval::ConsistencyAccumulator nis_{};            // per-update NIS, accumulated over the run
    eval::InnovationWhitenessAccumulator innov_{};  // per-update innovation mean/whiteness
    // FEJ clone divergence accumulators (#280): rotation in degrees, translation in metres.
    double fej_rot_sum_ = 0.0;
    double fej_rot_max_ = 0.0;
    double fej_trans_sum_ = 0.0;
    double fej_trans_max_ = 0.0;
    std::size_t fej_n_ = 0;
    msckf::Propagator<T> prop_{};
    ImuInitializer<T> initializer_{};
    msckf::State<T, Cov> state_{kInitialSigma()};
    VioConfig config_{};
    std::size_t max_clones_ = 11;

    InitDiagnostics<T> init_diag_{};
    bool initialized_ = false;
    bool have_last_time_ = false;
    double last_imu_time_ = 0.0;
    DVec3 last_gyro_{};  ///< most recent IMU sample, held over to the image time
    DVec3 last_accel_{};
    std::vector<DVec3> init_gyro_;
    std::vector<DVec3> init_accel_;
    std::size_t init_samples_seen_ = 0;  ///< total IMU samples seen during init (for the timeout)

    // Dynamic-init state: a bounded buffer of init-window camera frames and the
    // IMU preintegrator accumulating the deltas between them.
    std::vector<sfm::InitFrame<T>> init_frames_;
    ImuPreintegrator<T> init_preint_{};
    bool init_have_imu_t_ = false;
    double init_last_imu_t_ = 0.0;

    std::unordered_map<std::uint64_t, std::vector<ObsRec>> tracks_;
};

/// Full-covariance MSCKF backend (Joseph-form update). The default backend.
template <math::Scalar T>
using MsckfBackend = MsckfBackendT<T, msckf::FullCovariance<T>>;

/// Square-root-covariance MSCKF backend: carries the Cholesky factor of the
/// covariance and updates it with a QR (Householder) array algorithm instead
/// of the Joseph form — numerically superior conditioning over long runs.
/// Same VioBackend interface and feature-management policy as MsckfBackend<T>,
/// so it drops in without changing the front end (issue #187).
template <math::Scalar T>
using MsckfSqrtBackend = MsckfBackendT<T, msckf::SqrtCovariance<T>>;

}  // namespace branes::sdk

#endif  // BRANES_SDK_MSCKF_BACKEND_HPP
