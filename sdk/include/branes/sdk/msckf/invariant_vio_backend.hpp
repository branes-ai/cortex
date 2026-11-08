// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/invariant_vio_backend.hpp — drives MsckfInvariantBackend with
// the SAME feature-track lifecycle as the validated body-frame
// MsckfBackend::process_camera (issue #365). The system-level R-IEKF verdict (#347)
// was confounded by a hand-rolled Monte-Carlo loop (#360) that over-fused — the
// attitude split (yaw ≈ 7× vs roll/pitch ≈ 3400×) proved the cause was the track
// management, not the unit-proven observability fix. This adapter ports the
// validated lifecycle (update-a-feature-when-it-is-LOST, plus
// marginalize-feeds-the-oldest-clone-before-dropping-it) over the invariant state
// and update, so the verdict is measured through correct fusion.
//
// Scope (first increment): normalized observations + identity camera↔IMU extrinsic
// (the invariant measurement assumes clone == camera). Pixel/intrinsics/extrinsic
// plumbing for a full VioBackend + real-EuRoC run is a follow-up.
//
// Header-only, C++20.

#ifndef BRANES_SDK_MSCKF_INVARIANT_VIO_BACKEND_HPP
#define BRANES_SDK_MSCKF_INVARIANT_VIO_BACKEND_HPP

#include <branes/sdk/eval/consistency.hpp>
#include <branes/sdk/eval/innovation_whiteness.hpp>
#include <branes/sdk/msckf/msckf_invariant_backend.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace branes::sdk::msckf {

/// One normalized feature observation from the front end: which landmark, and its
/// position in normalized image coordinates (identity extrinsic ⇒ clone == camera).
template <math::Scalar T>
struct NormalizedObs {
    std::uint64_t feature_id = 0;
    std::size_t camera_index = 0;
    branes::math::lie::detail::Vec<T, 2> xy{};
};

/// Wraps `MsckfInvariantBackend` and feeds it through the body-frame backend's
/// feature-track lifecycle. The backend's own sliding-window cap is set effectively
/// unbounded; THIS class owns the window so a clone is never dropped before the
/// tracks it anchors have been fused.
template <math::Scalar T, class Cov = FullCovariance<T>>
class InvariantVioBackend {
public:
    using Backend = MsckfInvariantBackend<T, Cov>;
    using Calib = InvariantCalib<T>;
    using Vec3 = branes::math::lie::detail::Vec<T, 3>;
    using Vec2 = branes::math::lie::detail::Vec<T, 2>;
    using SE23 = branes::math::lie::SE23<T>;
    using SO3 = branes::math::lie::SO3<T>;

    struct Config {
        typename Backend::Config backend{};  ///< noise, gravity, σ's, gating
        std::size_t max_clones = 11;         ///< sliding-window size (owned here)
    };

    explicit InvariantVioBackend(const Config& cfg = {})
        : be_(with_unbounded_window(cfg.backend)), max_clones_(cfg.max_clones) {
        // A window of < 2 clones is degenerate: 0 spins the marginalization loop and
        // 1 never gives the MSCKF update a second view to triangulate against.
        if (cfg.max_clones < 2)
            throw std::invalid_argument("InvariantVioBackend: max_clones must be >= 2");
    }

    /// Seed the nav state — e.g. from a static/dynamic init or ground truth at t0.
    void set_nav(const SE23& X, const Vec3& bg, const Vec3& ba, double t) {
        be_.set_nav(X, bg, ba, t);
        last_imu_time_ = t;
        have_last_ = false;
    }

    /// Seed the nav state at `t` and hold the IMU sample in effect there (the
    /// last one at or before `t`), so propagation resumes from `t` without
    /// waiting for the next sample: an initializer that fires at an image time,
    /// between IMU samples, otherwise loses that gap (#608).
    void
    set_nav(const SE23& X, const Vec3& bg, const Vec3& ba, double t, const Vec3& hold_gyro, const Vec3& hold_accel) {
        be_.set_nav(X, bg, ba, t);
        last_imu_time_ = t;
        have_last_ = true;
        last_gyro_ = hold_gyro;
        last_accel_ = hold_accel;
    }

    /// Replace the nav covariance with G·Gᵀ (MsckfInvariantBackend::seed_covariance).
    void seed_covariance(const DynMat<T>& G) {
        be_.seed_covariance(G);
    }

    /// Turn on online extrinsic calibration: append a calibration block.
    void enable_calibration(std::vector<Calib> init, T rot_sigma, T trans_sigma) {
        be_.enable_calibration(std::move(init), rot_sigma, trans_sigma);
    }

    /// IMU sample: propagate from the previous sample (zero-order hold held over to
    /// the next image time). Drops out-of-order / duplicate timestamps, and any
    /// sample older than the time `set_nav` seeded: integrating those would run
    /// the seeded state forward over an interval that precedes it.
    void process_imu(const Vec3& gyro, const Vec3& accel, double t) {
        if (have_last_ ? t <= last_imu_time_ : t < last_imu_time_)
            return;
        if (have_last_) {
            const double dt = t - last_imu_time_;
            if (dt > 0.0) {
                be_.propagate(last_gyro_, last_accel_, static_cast<T>(dt));
                be_.set_timestamp(t);  // the clock stays the double sample time, not Σ T(dt)
            }
        }
        last_imu_time_ = t;
        have_last_ = true;
        last_gyro_ = gyro;
        last_accel_ = accel;
    }

    /// Image frame: propagate to frame time, bound the window, clone, ingest the
    /// observations, and fuse every feature whose track ended this frame.
    void process_camera(double t, std::span<const NormalizedObs<T>> obs) {
        if (!propagate_to(t))
            return;
        // Bound the window: marginalize (after fusing) the oldest clone(s) first.
        while (clone_times_.size() >= max_clones_)
            marginalize_oldest();

        be_.augment_clone();
        clone_times_.push_back(t);

        std::unordered_set<std::uint64_t> seen;
        seen.reserve(obs.size());
        for (const auto& o : obs) {
            tracks_[o.feature_id].push_back(ObsRec{t, o.camera_index, o.xy});
            seen.insert(o.feature_id);
        }
        // Features not observed this frame have ENDED — fuse their full track, drop.
        std::vector<std::uint64_t> ended;
        for (const auto& [id, recs] : tracks_)
            if (seen.find(id) == seen.end())
                ended.push_back(id);
        std::sort(ended.begin(), ended.end());  // deterministic EKF update order
        for (const std::uint64_t id : ended)
            update_and_erase(id);
    }

    /// S8: an image frame at rest. Propagate to its time and apply the v = 0
    /// pseudo-measurement; the frame adds no clone, as in the body-frame
    /// backend: without parallax its features would triangulate on a zero
    /// baseline (#600). Returns true iff the state was updated.
    bool process_still_frame(double t, T velocity_sigma) {
        if (!propagate_to(t))
            return false;
        return be_.zero_velocity_update(velocity_sigma);
    }

    /// S8: the gyro bias at rest is the mean angular rate (σ per axis).
    bool gyro_bias_update(const Vec3& mean_rate, T sigma) {
        return be_.gyro_bias_update(mean_rate, sigma);
    }

    // ── accessors (for the consistency harness) ──────────────────────────────
    [[nodiscard]] const typename Backend::Nav& nav() const noexcept {
        return be_.nav();
    }
    [[nodiscard]] const std::vector<Calib>& calib() const noexcept {
        return be_.calib();
    }
    [[nodiscard]] DynMat<T> covariance() const {
        return be_.covariance();
    }
    [[nodiscard]] std::size_t num_clones() const noexcept {
        return be_.num_clones();
    }
    /// Every update's NIS, accumulated over the run (as MsckfBackend's, #264).
    [[nodiscard]] const eval::ConsistencyAccumulator& nis_consistency() const noexcept {
        return nis_;
    }
    /// Every update's signed innovation sum: zero-mean and whiteness (#280).
    [[nodiscard]] const eval::InnovationWhitenessAccumulator& innovation_whiteness() const noexcept {
        return innov_;
    }

private:
    static constexpr std::size_t kNoClone = static_cast<std::size_t>(-1);
    struct ObsRec {
        double clone_time = 0.0;
        std::size_t camera_index = 0;
        Vec2 xy{};
    };

    static typename Backend::Config with_unbounded_window(typename Backend::Config c) {
        c.max_clones = 1000000;  // we own the window; never let the backend self-drop
        return c;
    }

    [[nodiscard]] std::size_t clone_index_at(double time) const {
        for (std::size_t i = 0; i < clone_times_.size(); ++i)
            if (clone_times_[i] == time)
                return i;
        return kNoClone;
    }

    // Propagate to an image time on the held IMU sample (zero-order hold).
    // False for a frame older than the filter clock.
    bool propagate_to(double t) {
        if (have_last_) {
            const double dt = t - last_imu_time_;
            if (dt < 0.0)
                return false;
            if (dt > 0.0) {
                be_.propagate(last_gyro_, last_accel_, static_cast<T>(dt));
                be_.set_timestamp(t);
                last_imu_time_ = t;
            }
        }
        return true;
    }

    void update_and_erase(std::uint64_t id) {
        auto it = tracks_.find(id);
        if (it == tracks_.end())
            return;
        std::vector<InvariantObs<T>> track;
        track.reserve(it->second.size());
        for (const ObsRec& rec : it->second) {
            const std::size_t ci = clone_index_at(rec.clone_time);
            if (ci != kNoClone) {  // clone still in the window
                InvariantObs<T> io;
                io.clone_index = ci;
                io.camera_index = rec.camera_index;
                io.xy = rec.xy;
                track.push_back(io);
            }
        }
        if (track.size() >= 2) {
            (void)be_.update(track);
            const auto& n = be_.last_nis();
            if (n.valid) {  // recorded whether or not the gate accepted the track
                nis_.add(static_cast<double>(n.value), static_cast<int>(n.dof));
                innov_.add(static_cast<double>(n.innov_sum), n.dof);
            }
        }
        tracks_.erase(it);
    }

    void marginalize_oldest() {
        if (clone_times_.empty())
            return;
        const double old_time = clone_times_.front();
        // Fuse the information from every track touching the oldest clone first.
        std::vector<std::uint64_t> touching;
        for (const auto& [id, recs] : tracks_)
            for (const ObsRec& rec : recs)
                if (rec.clone_time == old_time) {
                    touching.push_back(id);
                    break;
                }
        std::sort(touching.begin(), touching.end());
        for (const std::uint64_t id : touching)
            update_and_erase(id);

        be_.marginalize_clone(0);
        clone_times_.erase(clone_times_.begin());

        // Purge the dropped clone's observations from surviving tracks.
        for (auto it = tracks_.begin(); it != tracks_.end();) {
            auto& recs = it->second;
            for (auto r = recs.begin(); r != recs.end();)
                r = (r->clone_time == old_time) ? recs.erase(r) : std::next(r);
            it = recs.empty() ? tracks_.erase(it) : std::next(it);
        }
    }

    Backend be_;
    eval::ConsistencyAccumulator nis_{};
    eval::InnovationWhitenessAccumulator innov_{};
    std::size_t max_clones_;
    std::vector<double> clone_times_;  // parallel to be_.clones()
    std::unordered_map<std::uint64_t, std::vector<ObsRec>> tracks_;
    Vec3 last_gyro_{};
    Vec3 last_accel_{};
    double last_imu_time_ = 0.0;
    bool have_last_ = false;
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_INVARIANT_VIO_BACKEND_HPP
