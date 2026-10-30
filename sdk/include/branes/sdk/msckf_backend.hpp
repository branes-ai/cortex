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
// The backend is the call sequence of the stage transformations in
// msckf/stages/ (#452): S2_propagation per IMU sample; per frame, S6 + S9 to
// slide the window, S3_augmentation, S0_sensor_model + S4_frontend to ingest the
// observations, then S5 → S6e for every ended track. S1_initialization seeds the
// state and S10_online_calibration adds the calibration states. What remains here
// is policy: initialization timing, the window size, and the feature schedule.
//
// Each stage computes in the type a stage-type policy names for it
// (msckf/stage_types.hpp); the default policy is every stage in T. A stage in
// another type runs on the state rebound into that type (msckf/rebind.hpp) and
// returns it rounded to T, so the state is always stored in T (#450).
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
#include <branes/sdk/msckf/rebind.hpp>
#include <branes/sdk/msckf/stage_tap.hpp>
#include <branes/sdk/msckf/stage_types.hpp>
#include <branes/sdk/msckf/stages.hpp>
#include <branes/sdk/sfm/init_window.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace branes::sdk {

// InitMethod / InitDiagnostics: msckf/stages/s1_initialization.hpp.

/// The MSCKF backend, generic over the covariance representation `Cov`
/// (`FullCovariance` — Joseph-form, the default; or `SqrtCovariance` — the
/// QR array update). The mean, IMU init, feature management, and window
/// policy are identical for either; only the covariance algebra differs.
/// `Stages` names the type each stage computes in (msckf/stage_types.hpp); the
/// state is stored in T. Use the `MsckfBackend<T>` / `MsckfSqrtBackend<T>` /
/// `MsckfMixedBackend<T, Stages>` aliases below.
template <math::Scalar T,
          msckf::CovarianceModel<T> Cov = msckf::FullCovariance<T>,
          msckf::StageTypePolicy Stages = msckf::UniformStageTypes<T>>
class MsckfBackendT final : public VioBackend<T> {
public:
    using Scalar = T;
    using StageTypes = Stages;
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
        rebind_operators();
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
        uopts.min_parallax_deg = static_cast<T>(config.min_parallax_deg);
        // S10 calibration-uncertainty term: deg → rad, folded into R by the updater.
        constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
        uopts.calib_rot_sigma = static_cast<T>(config.calib_ext_rot_sigma_deg * kDegToRad);
        updater_ = msckf::CameraUpdater<T>(extrinsics_of(cameras_), uopts);

        rebind_operators();

        ImuInitConfig<T> icfg;
        icfg.gravity_magnitude = static_cast<T>(config.gravity_magnitude);
        initializer_ = ImuInitializer<T>(icfg);

        state_ = fresh_state();
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
            propagate(gyro, accel, static_cast<T>(dt));
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

        // S2: propagate to the image time (zero-order hold on the last IMU
        // sample) so the clone is anchored at exactly `timestamp_s`, even when
        // the frame falls between IMU samples. Out-of-order frames are ignored.
        if (have_last_time_) {
            const double dt = timestamp_s - last_imu_time_;
            if (dt < 0.0)
                return;
            if (dt > 0.0) {
                propagate(last_gyro_, last_accel_, static_cast<T>(dt));
                last_imu_time_ = timestamp_s;
            }
        }

        if (stage_tap_)
            stage_tap_->on_frame(timestamp_s);

        // S6 + S9: keep the window bounded — update with, then marginalize, the
        // oldest clone(s) before adding the new one.
        while (state_.clones.size() >= max_clones_)
            marginalize_oldest();

        // S3: clone the (now frame-time) pose into the window.
        if (wants(msckf::TapStage::S3_augmentation)) {
            const auto before = state_;
            const auto d = augment(timestamp_s);
            stage_tap_->on_s3(before, timestamp_s, d, state_);
        } else {
            augment(timestamp_s);
        }

        // S0 + S4: normalize this frame's observations onto their tracks.
        const auto ingest = msckf::stages::s4_frontend::apply<T>(
            tracks_, timestamp_s, obs, [this](std::uint32_t cam, T u, T v, DVec2& xy) {
                return to_normalized(cam, u, v, xy);
            });

        // S5 → S6e: features not observed this frame have ended; feed them to the
        // filter (ascending id) and drop them.
        for (const std::uint64_t id : ingest.ended)
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

    /// Read-only view of the full filter state (covariance, clone window,
    /// biases). For inspection, logging, and the replay harness (#46).
    [[nodiscard]] const msckf::State<T, Cov>& state() const noexcept {
        return state_;
    }

    /// Per-update observation hook (S6 inspector, #380). Invoked once per MSCKF
    /// camera update with the covariance snapshot taken *before* the update, the
    /// feature track consumed, the resulting NIS sample, whether the update was
    /// applied (false ⇒ χ²-gated or degenerate), and — implicitly — the state
    /// *after* (the first argument is the live, updated state). It decouples the
    /// update from the monolithic backend for study without touching the hot path:
    /// unset (the default) costs only a branch and no covariance copy. SDK-typed,
    /// so it adds no dependency on the tools layer.
    using UpdateObserver = std::function<void(const msckf::State<T, Cov>& state_after,
                                              const msckf::FeatureTrack<T>& track,
                                              const msckf::NisSample<T>& nis,
                                              bool accepted,
                                              const msckf::DynMat<T>& cov_before)>;
    void set_update_observer(UpdateObserver observer) {
        update_observer_ = std::move(observer);
    }

    /// The stage-boundary tap (#446): the boundaries it wants (S0, S2, S3, S6,
    /// S9) are reported with the state before and after. Non-owning — the tap
    /// must outlive its use; nullptr (the default) turns it off, at the cost of
    /// one null test per boundary.
    using StageTap = msckf::StageTap<T, Cov>;
    void set_stage_tap(StageTap* tap) noexcept {
        stage_tap_ = tap;
    }

private:
    using T0 = typename Stages::sensor_model;
    using T2 = typename Stages::propagation;
    using T3 = typename Stages::augmentation;
    using T6 = typename Stages::update;
    using T6e = msckf::ekf_update_t<Stages>;
    using T9 = typename Stages::marginalization;
    using T10 = typename Stages::calibration;
    template <class U>
    static constexpr bool kStorage = std::is_same_v<U, T>;

    /// Run `stage` on `s` in U: on `s` itself when U is the storage type, else on
    /// `s` rebound to U, with the result rounded back into `s`. Returns what the
    /// stage returns.
    template <math::Scalar U, class Stage>
    static auto in_type(msckf::State<T, Cov>& s, Stage&& stage) {
        if constexpr (kStorage<U>) {
            return stage(s);
        } else {
            auto su = msckf::rebind<U>(s);
            auto r = stage(su);
            s = msckf::rebind<T>(su);
            return r;
        }
    }

    /// The operators of the stages that compute in another type, rebound from
    /// the storage-type ones. Re-run whenever those change (construction and
    /// `initialize`); nothing to do for a stage in the storage type.
    void rebind_operators() {
        if constexpr (!kStorage<T0>) {
            cameras0_.clear();
            for (const auto& c : cameras_)
                cameras0_.push_back(msckf::rebind<T0>(c.intrinsics));
        }
        if constexpr (!kStorage<T2>)
            prop2_ = msckf::rebind<T2>(prop_);
        if constexpr (!kStorage<T6>)
            updater6_.emplace(msckf::rebind<T6>(updater_));
        if constexpr (!kStorage<T6e> && !std::is_same_v<T6e, T6>)
            updater6e_.emplace(msckf::rebind<T6e>(updater_));
    }
    [[nodiscard]] const auto& propagator2() const {
        if constexpr (kStorage<T2>)
            return prop_;
        else
            return prop2_;
    }
    [[nodiscard]] const auto& updater6() const {
        if constexpr (kStorage<T6>)
            return updater_;
        else
            return *updater6_;
    }
    [[nodiscard]] const auto& updater6e() const {
        if constexpr (std::is_same_v<T6e, T6>)
            return updater6();
        else if constexpr (kStorage<T6e>)
            return updater_;
        else
            return *updater6e_;
    }

    // S3 in its stage type.
    msckf::stages::s3_augmentation::Diagnostics augment(double t) {
        return in_type<T3>(state_, [t](auto& s) { return msckf::stages::s3_augmentation::apply(s, t); });
    }

    // S9 in its stage type.
    msckf::stages::s9_marginalization::Diagnostics marginalize(std::size_t idx) {
        return in_type<T9>(state_, [idx](auto& s) { return msckf::stages::s9_marginalization::apply(s, idx); });
    }

    // S5 → S6e in its stage type; the diagnostics come back in T. With a separate
    // `ekf_update` type, S5 → S6d run in T6 (they only read the state) and S6e
    // in T6e, on the measurement rebound into it.
    msckf::stages::s6_msckf_update::Diagnostics<T> update(const msckf::FeatureTrack<T>& track) {
        namespace s6 = msckf::stages;
        if constexpr (!std::is_same_v<T6e, T6>) {
            const auto prep = [&] {
                if constexpr (kStorage<T6>)
                    return s6::s6_msckf_update::prepare(state_, updater_, track);
                else
                    return s6::s6_msckf_update::prepare(
                        msckf::rebind<T6>(state_), updater6(), msckf::rebind<T6>(track));
            }();
            s6::s6_msckf_update::Diagnostics<T> d{prep.diag.outcome, msckf::rebind<T>(prep.diag.nis), prep.diag.rows};
            if (!prep.ready)
                return d;
            // The gate ran in T6. S6e solves with the innovation covariance in T6e,
            // so it must factor there too, or the solve has no gain to apply: reject
            // the track as an ill-conditioned innovation and leave the state as it was.
            const auto m = msckf::rebind<T6e>(prep.measurement);
            const auto& ue = updater6e();
            const auto factors = [&](const auto& s) {
                using U = T6e;
                U gamma{0};
                return s.cov.mahalanobis(m.H, std::span<const U>{m.r}, ue.measurement_variance(), gamma);
            };
            if constexpr (kStorage<T6e>) {
                if (!factors(state_)) {
                    d.outcome = s6::s6_msckf_update::Outcome::Gated;
                    return d;
                }
                (void)s6::s6e_ekf_update::apply(state_, ue, m);
            } else {
                auto su = msckf::rebind<T6e>(state_);
                if (!factors(su)) {
                    d.outcome = s6::s6_msckf_update::Outcome::Gated;
                    return d;
                }
                (void)s6::s6e_ekf_update::apply(su, ue, m);
                state_ = msckf::rebind<T>(su);
            }
            d.outcome = s6::s6_msckf_update::Outcome::Applied;
            return d;
        } else if constexpr (kStorage<T6>) {
            return msckf::stages::s6_msckf_update::apply(state_, updater_, track);
        } else {
            const auto tu = msckf::rebind<T6>(track);
            const auto d =
                in_type<T6>(state_, [&](auto& s) { return msckf::stages::s6_msckf_update::apply(s, updater6(), tu); });
            return {d.outcome, msckf::rebind<T>(d.nis), d.rows};
        }
    }

    static constexpr T kInitialSigma() {
        return T{1} / T{10};
    }

    /// A fresh uninitialized state: isotropic IMU covariance, plus — when online
    /// extrinsic calibration is enabled (#332) — a per-camera calibration block
    /// seeded from the configured extrinsics and the configured prior σ. Used at
    /// every state reset (initialize and both init-completion paths) so the
    /// calibration block survives re-seeding. Inert when the flag is off.
    [[nodiscard]] msckf::State<T, Cov> fresh_state() const {
        msckf::State<T, Cov> s(kInitialSigma());
        if (config_.estimate_extrinsics && !cameras_.empty()) {
            using Calib = typename msckf::State<T, Cov>::CalibState;
            std::vector<Calib> init;
            init.reserve(cameras_.size());
            for (const auto& cam : cameras_)
                init.push_back(Calib{cam.extrinsics.R_imu_cam, cam.extrinsics.p_imu_cam});
            constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
            const T rot_sigma = static_cast<T>(config_.calib_ext_rot_prior_deg * kDegToRad);
            const T trans_sigma = static_cast<T>(config_.calib_ext_trans_prior_mm / 1000.0);
            in_type<T10>(s, [&](auto& sx) {
                using Calib10 = typename std::remove_cvref_t<decltype(sx)>::CalibState;
                std::vector<Calib10> init10;
                init10.reserve(init.size());
                for (const auto& c : init)
                    init10.push_back({msckf::rebind<T10>(c.R_imu_cam), msckf::rebind<T10>(c.p_imu_cam)});
                return msckf::stages::s10_online_calibration::apply(
                    sx, std::move(init10), msckf::scalar_cast<T10>(rot_sigma), msckf::scalar_cast<T10>(trans_sigma));
            });
        }
        return s;
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

    // S1: seed a fresh state from an IMU-only init result, record how it went,
    // and finish.
    void apply_init(const ImuInitResult<T>& res, InitMethod method, double t) {
        state_ = fresh_state();
        msckf::stages::s1_initialization::seed_from_imu(state_,
                                                        res,
                                                        method,
                                                        t,
                                                        std::span<const DVec3>{init_accel_},
                                                        static_cast<T>(config_.gravity_magnitude),
                                                        init_diag_);
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
        // The attempt, as the S1 tap sees it: reported before a successful
        // alignment seeds the filter (which clears the init buffers).
        const auto report = [&](bool aligned) {
            if (wants(msckf::TapStage::S1_initialization))
                stage_tap_->on_s1(std::span<const sfm::InitFrame<T>>{init_frames_},
                                  cameras_[0].extrinsics,
                                  std::span<const DVec3>{init_gyro_},
                                  std::span<const DVec3>{init_accel_},
                                  static_cast<T>(config_.gravity_magnitude),
                                  t,
                                  win,
                                  aligned);
        };
        if (!win.success) {
            report(false);
            return;  // two-view/PnP bootstrap failed on these tracks
        }
        ++init_diag_.dyn_window_builds;
        if (static_cast<int>(win.keyframes.size()) > init_diag_.dyn_best_keyframes)
            init_diag_.dyn_best_keyframes = static_cast<int>(win.keyframes.size());
        const auto r = initializer_.try_dynamic(win.keyframes);
        // Record the resolved metric motion even on decline, so a run shows how
        // close the best window came to the observability floor (#247).
        if (static_cast<double>(r.resolved_motion) > init_diag_.dyn_best_metric_motion)
            init_diag_.dyn_best_metric_motion = static_cast<double>(r.resolved_motion);
        report(r.success);
        if (!r.success)
            return;
        apply_dynamic_init(r, win, t);
    }

    // S1: seed a fresh state from a successful dynamic visual-inertial
    // alignment, record how it went, and finish.
    void apply_dynamic_init(const ImuInitResult<T>& r, const sfm::InitWindowResult<T>& win, double t) {
        state_ = fresh_state();
        msckf::stages::s1_initialization::seed_from_alignment(state_,
                                                              r,
                                                              win,
                                                              t,
                                                              std::span<const DVec3>{init_accel_},
                                                              static_cast<T>(config_.gravity_magnitude),
                                                              init_diag_);
        finish_init(t);
    }

    bool to_normalized(std::uint32_t cam_id, T u, T v, DVec2& out) const {
        if (cam_id >= cameras_.size())
            return false;  // unknown camera — reject rather than alias to camera 0
        const auto n = sensor_model(cam_id, u, v);
        if (wants(msckf::TapStage::S0_sensor_model))
            stage_tap_->on_s0(cam_id, u, v, n);
        if (!n.valid)
            return false;  // the ray is not in front of the camera
        out = n.xy;
        return true;
    }

    // S0 in its stage type; the result comes back in T.
    [[nodiscard]] msckf::stages::s0_sensor_model::Result<T> sensor_model(std::uint32_t cam_id, T u, T v) const {
        if constexpr (kStorage<T0>) {
            return msckf::stages::s0_sensor_model::apply(cameras_[cam_id].intrinsics, u, v);
        } else {
            const auto n = msckf::stages::s0_sensor_model::apply(
                cameras0_[cam_id], msckf::scalar_cast<T0>(u), msckf::scalar_cast<T0>(v));
            return {n.valid, msckf::rebind<T>(n.xy)};
        }
    }

    [[nodiscard]] bool wants(msckf::TapStage stage) const {
        return stage_tap_ != nullptr && stage_tap_->wants(stage);
    }

    // S2: one propagation step, reported to the tap when it asks.
    void propagate(const DVec3& gyro, const DVec3& accel, T dt) {
        if (wants(msckf::TapStage::S2_propagation)) {
            const auto before = state_;
            propagate_step(gyro, accel, dt);
            stage_tap_->on_s2(before, prop_, gyro, accel, dt, state_);
            return;
        }
        propagate_step(gyro, accel, dt);
    }

    // S2 in its stage type.
    void propagate_step(const DVec3& gyro, const DVec3& accel, T dt) {
        if constexpr (kStorage<T2>) {
            msckf::stages::s2_propagation::apply(state_, prop_, gyro, accel, dt);
        } else {
            const auto g = msckf::rebind<T2>(gyro), a = msckf::rebind<T2>(accel);
            const T2 h = msckf::scalar_cast<T2>(dt);
            in_type<T2>(state_,
                        [&](auto& s) { return msckf::stages::s2_propagation::apply(s, propagator2(), g, a, h); });
        }
    }

    // S4 → S5 → S6e: assemble feature `id`'s track from the observations whose
    // clone is still in the window, run the MSCKF update if it has enough of
    // them, record the innovation telemetry, then forget the feature.
    void update_and_erase(std::uint64_t id) {
        auto it = tracks_.find(id);
        if (it == tracks_.end())
            return;
        const msckf::FeatureTrack<T> track = msckf::stages::s4_frontend::assemble(it->second, state_);
        if (track.observations.size() >= 2) {
            // Snapshot the pre-update covariance only when someone is watching.
            msckf::DynMat<T> cov_before;
            if (update_observer_)
                cov_before = state_.covariance();
            std::optional<msckf::State<T, Cov>> before;  // the tap's copy, only when it asks
            if (wants(msckf::TapStage::S6_msckf_update))
                before.emplace(state_);
            const auto upd = update(track);
            if (before)
                stage_tap_->on_s6(*before, updater_, id, track, upd, state_);
            if (upd.nis.valid) {  // record the innovation NIS even if it was gated out
                nis_.add(static_cast<double>(upd.nis.value), static_cast<int>(upd.nis.dof));
                innov_.add(static_cast<double>(upd.nis.innov_sum), upd.nis.dof);
            }
            if (update_observer_)
                update_observer_(state_, track, upd.nis, upd.accepted(), cov_before);
        }
        tracks_.erase(it);
    }

    // S6 then S9: update with every feature observed at the oldest clone (so its
    // information is used), drop that clone, and purge its stale observations.
    void marginalize_oldest() {
        if (state_.clones.empty())
            return;
        const double old_time = state_.clones.front().timestamp;
        for (const std::uint64_t id : msckf::stages::s9_marginalization::tracks_touching(tracks_, old_time))
            update_and_erase(id);
        if (wants(msckf::TapStage::S9_marginalization)) {
            const auto before = state_;
            const auto d = marginalize(0);
            stage_tap_->on_s9(before, 0, d, state_);
        } else {
            marginalize(0);
        }
        msckf::stages::s9_marginalization::purge(tracks_, old_time);
    }

    std::vector<CameraCalibration> cameras_;
    msckf::CameraUpdater<T> updater_;
    eval::ConsistencyAccumulator nis_{};            // per-update NIS, accumulated over the run
    eval::InnovationWhitenessAccumulator innov_{};  // per-update innovation mean/whiteness
    UpdateObserver update_observer_{};              // optional per-update inspector hook (#380); unset in production
    StageTap* stage_tap_ = nullptr;                 // optional stage-boundary tap (#446); unset in production
    msckf::Propagator<T> prop_{};
    // The rebound operators of the stages that compute in another type (unused,
    // and left empty, for a stage in T).
    std::vector<math::cameras::PinholeRadtanCamera<T0>> cameras0_;
    msckf::Propagator<T2> prop2_{};
    std::optional<msckf::CameraUpdater<T6>> updater6_;
    std::optional<msckf::CameraUpdater<T6e>> updater6e_;
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

    msckf::stages::s4_frontend::TrackTable<T> tracks_;
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

/// Full-covariance MSCKF backend storing its state in T, each stage computing in
/// the type `Stages` names for it (msckf/stage_types.hpp, #450). With
/// `UniformStageTypes<T>` it is `MsckfBackend<T>`.
template <math::Scalar T, msckf::StageTypePolicy Stages>
using MsckfMixedBackend = MsckfBackendT<T, msckf::FullCovariance<T>, Stages>;

}  // namespace branes::sdk

#endif  // BRANES_SDK_MSCKF_BACKEND_HPP
