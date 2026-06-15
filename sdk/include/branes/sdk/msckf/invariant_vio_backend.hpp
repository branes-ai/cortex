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

#include <branes/sdk/msckf/msckf_invariant_backend.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace branes::sdk::msckf {

/// One normalized feature observation from the front end: which landmark, and its
/// position in normalized image coordinates (identity extrinsic ⇒ clone == camera).
template <math::Scalar T>
struct NormalizedObs {
    std::uint64_t feature_id = 0;
    math::lie::detail::Vec<T, 2> xy{};
};

/// Wraps `MsckfInvariantBackend` and feeds it through the body-frame backend's
/// feature-track lifecycle. The backend's own sliding-window cap is set effectively
/// unbounded; THIS class owns the window so a clone is never dropped before the
/// tracks it anchors have been fused.
template <math::Scalar T, class Cov = FullCovariance<T>>
class InvariantVioBackend {
public:
    using Backend = MsckfInvariantBackend<T, Cov>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Vec2 = math::lie::detail::Vec<T, 2>;
    using SE23 = math::lie::SE23<T>;
    using SO3 = math::lie::SO3<T>;

    struct Config {
        typename Backend::Config backend{};  ///< noise, gravity, σ's, gating
        std::size_t max_clones = 11;         ///< sliding-window size (owned here)
    };

    explicit InvariantVioBackend(const Config& cfg = {})
        : be_(with_unbounded_window(cfg.backend)), max_clones_(cfg.max_clones) {}

    /// Seed the nav state — e.g. from a static/dynamic init or ground truth at t0.
    void set_nav(const SE23& X, const Vec3& bg, const Vec3& ba, double t) {
        be_.set_nav(X, bg, ba, t);
        last_imu_time_ = t;
        have_last_ = true;
    }

    /// IMU sample: propagate from the previous sample (zero-order hold held over to
    /// the next image time). Drops out-of-order / duplicate timestamps.
    void process_imu(const Vec3& gyro, const Vec3& accel, double t) {
        if (have_last_) {
            const double dt = t - last_imu_time_;
            if (!(dt > 0.0))
                return;
            be_.propagate(gyro, accel, static_cast<T>(dt));
        }
        last_imu_time_ = t;
        have_last_ = true;
        last_gyro_ = gyro;
        last_accel_ = accel;
    }

    /// Image frame: propagate to frame time, bound the window, clone, ingest the
    /// observations, and fuse every feature whose track ended this frame.
    void process_camera(double t, std::span<const NormalizedObs<T>> obs) {
        if (have_last_) {
            const double dt = t - last_imu_time_;
            if (dt < 0.0)
                return;
            if (dt > 0.0) {
                be_.propagate(last_gyro_, last_accel_, static_cast<T>(dt));
                last_imu_time_ = t;
            }
        }
        // Bound the window: marginalize (after fusing) the oldest clone(s) first.
        while (clone_times_.size() >= max_clones_)
            marginalize_oldest();

        be_.augment_clone();
        clone_times_.push_back(t);

        std::unordered_set<std::uint64_t> seen;
        seen.reserve(obs.size());
        for (const auto& o : obs) {
            tracks_[o.feature_id].push_back(ObsRec{t, o.xy});
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

    // ── accessors (for the consistency harness) ──────────────────────────────
    [[nodiscard]] const typename Backend::Nav& nav() const noexcept {
        return be_.nav();
    }
    [[nodiscard]] DynMat<T> covariance() const {
        return be_.covariance();
    }
    [[nodiscard]] std::size_t num_clones() const noexcept {
        return be_.num_clones();
    }

private:
    static constexpr std::size_t kNoClone = static_cast<std::size_t>(-1);
    struct ObsRec {
        double clone_time = 0.0;
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

    void update_and_erase(std::uint64_t id) {
        auto it = tracks_.find(id);
        if (it == tracks_.end())
            return;
        InvariantTrack<T> track;
        track.reserve(it->second.size());
        for (const ObsRec& rec : it->second) {
            const std::size_t ci = clone_index_at(rec.clone_time);
            if (ci != kNoClone)  // clone still in the window
                track.push_back(InvariantObs<T>{ci, rec.xy});
        }
        if (track.size() >= 2)
            be_.update(track);
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
