// SPDX-License-Identifier: MIT
//
// branes/tools/ideal_tracks.hpp — feature tracks synthesized from ground truth
// (issue #602).
//
// The real EuRoC IMU and timestamps go to the backend unchanged; the KLT front
// end is replaced by observations of synthetic landmarks projected through the
// ground-truth pose and the same cam0 calibration the backend uses, with
// Gaussian pixel noise. A run on ideal tracks bounds what the estimator can do
// with this IMU when the front end is perfect, which splits the end-to-end
// error between the front end and the estimator:
//
//   - landmarks are spawned in the world frame along the ray of a random pixel
//     at a random depth whenever fewer than `target_features` are visible, at
//     least `min_feature_distance` px from the visible ones (as S4 detects);
//   - each frame projects every live landmark through T_world_imu (ground
//     truth, interpolated to the image stamp) and T_imu_cam;
//   - a track ends when its landmark leaves the image or falls behind the
//     camera, and is never re-acquired.
//
// What ideal tracks also remove: calibration error (both sides share the
// calibration), outliers, track drift and churn. The ground truth carries its
// own millimetre-level error and time alignment to the IMU.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_IDEAL_TRACKS_HPP
#define BRANES_TOOLS_IDEAL_TRACKS_HPP

#include <branes/math/cameras/pinhole_radtan.hpp>
#include <branes/math/lie/se3.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <span>
#include <utility>
#include <vector>

namespace branes::tools {

struct IdealTrackOptions {
    double sigma_px = 1.0;  ///< Gaussian pixel noise per axis
    /// Draw the noise in undistorted normalized coordinates with σ = sigma_px / fx
    /// (the filter's own pixel-to-normalized conversion), then project through
    /// the lens: the noise the filter is told is then exactly the noise it
    /// gets. Off (the default), the noise is N(0, σ²) in distorted pixels, which
    /// undistortion stretches by up to ~1.35× near EuRoC cam0's edges (#613).
    bool noise_normalized = false;
    std::size_t target_features = 150;   ///< spawn while fewer are visible (S4's default)
    double min_feature_distance = 15.0;  ///< px between a new landmark and the visible ones (S4's default)
    double min_depth_m = 2.0;            ///< spawn depth range along the ray (log-uniform)
    double max_depth_m = 8.0;
    double border_px = 10.0;  ///< spawn and keep only this far inside the image
    int width = 752;          ///< EuRoC cam0
    int height = 480;
    double max_gt_gap_s = 0.02;  ///< no observations where the ground truth has a longer gap
    std::uint64_t seed = 1;
};

/// Salt for the pixel-noise stream: seeded seed ^ salt, apart from the landmark
/// geometry's stream. A consumer drawing other noise for the same run (the
/// synthesized IMU, gt_spline.hpp) must use a different salt (#613).
inline constexpr std::uint64_t kTrackNoiseSeedSalt = 0x9E3779B97F4A7C15ULL;

template <class T>
class IdealTracks {
public:
    using Vec3 = math::lie::detail::Vec<T, 3>;

    IdealTracks(std::vector<sdk::eval::StampedPose<T>> gt,
                const math::cameras::PinholeRadtanCamera<T>& camera,
                const math::lie::SE3<T>& T_imu_cam,
                const IdealTrackOptions& opt = {})
        : gt_(std::move(gt)), cam_(camera), T_imu_cam_(T_imu_cam), opt_(opt), rng_(opt.seed),
          noise_rng_(opt.seed ^ kTrackNoiseSeedSalt) {}

    /// The observations at image stamp `t_s`: empty where the ground truth does
    /// not cover `t_s`.
    std::vector<sdk::FrontendObservation<T>> observe(double t_s) {
        std::vector<sdk::FrontendObservation<T>> obs;
        const auto T_wi = pose_at(t_s);
        if (!T_wi)
            return obs;
        const math::lie::SE3<T> T_cw = (*T_wi * T_imu_cam_).inverse();

        std::vector<std::pair<T, T>> px;  // the clean pixels of the visible landmarks
        std::vector<Landmark> live;
        live.reserve(live_.size());
        for (const auto& l : live_) {
            const auto p = project(T_cw * l.p_world);
            if (!p)
                continue;  // left the view: the track ends
            live.push_back(l);
            px.push_back(*p);
        }
        live_ = std::move(live);

        std::uniform_real_distribution<double> uu(opt_.border_px, opt_.width - 1 - opt_.border_px);
        std::uniform_real_distribution<double> vv(opt_.border_px, opt_.height - 1 - opt_.border_px);
        std::uniform_real_distribution<double> ld(std::log(opt_.min_depth_m), std::log(opt_.max_depth_m));
        const math::lie::SE3<T> T_wc = T_cw.inverse();
        const double d2 = opt_.min_feature_distance * opt_.min_feature_distance;
        for (std::size_t tries = 0; live_.size() < opt_.target_features && tries < 20 * opt_.target_features; ++tries) {
            const T u = static_cast<T>(uu(rng_)), v = static_cast<T>(vv(rng_));
            const bool crowded = std::any_of(px.begin(), px.end(), [&](const auto& q) {
                const double du = static_cast<double>(q.first - u), dv = static_cast<double>(q.second - v);
                return du * du + dv * dv < d2;
            });
            if (crowded)
                continue;
            const auto b = cam_.unproject({u, v});
            const T z = static_cast<T>(std::exp(ld(rng_)));
            const Vec3 p_cam{{b[0] / b[2] * z, b[1] / b[2] * z, z}};
            live_.push_back(Landmark{next_id_++, T_wc * p_cam});
            px.emplace_back(u, v);
        }

        // A unit normal scaled by σ: normal_distribution needs a positive σ,
        // and σ = 0 draws nothing.
        obs.reserve(live_.size());
        for (std::size_t i = 0; i < live_.size(); ++i) {
            sdk::FrontendObservation<T> o;
            o.feature_id = live_[i].id;
            o.camera_id = 0;
            o.u = px[i].first;
            o.v = px[i].second;
            if (opt_.sigma_px > 0 && opt_.noise_normalized) {
                const auto b = cam_.unproject({o.u, o.v});
                const T sn = static_cast<T>(opt_.sigma_px) / cam_.fx();
                const T x = b[0] / b[2] + sn * static_cast<T>(unit_(noise_rng_));
                const T y = b[1] / b[2] + sn * static_cast<T>(unit_(noise_rng_));
                const auto q = cam_.project({x, y, T(1)});
                o.u = q[0];
                o.v = q[1];
            } else if (opt_.sigma_px > 0) {
                o.u += static_cast<T>(opt_.sigma_px * unit_(noise_rng_));
                o.v += static_cast<T>(opt_.sigma_px * unit_(noise_rng_));
            }
            obs.push_back(o);
        }
        return obs;
    }

    /// Landmarks spawned so far (the number of distinct tracks).
    [[nodiscard]] std::uint64_t tracks() const noexcept {
        return next_id_;
    }

    /// The ground-truth T_world_imu at `t_s`, interpolated between the two
    /// bracketing samples; none outside the ground truth or across a gap.
    [[nodiscard]] std::optional<math::lie::SE3<T>> pose_at(double t_s) const {
        const auto it = std::lower_bound(
            gt_.begin(), gt_.end(), t_s, [](const sdk::eval::StampedPose<T>& a, double t) { return a.t_s < t; });
        if (it == gt_.end() || it == gt_.begin()) {
            if (it != gt_.end() && it->t_s == t_s)
                return it->pose;
            return std::nullopt;
        }
        const auto& b = *it;
        const auto& a = *(it - 1);
        if (b.t_s - a.t_s > opt_.max_gt_gap_s)
            return std::nullopt;
        const T s = static_cast<T>((t_s - a.t_s) / (b.t_s - a.t_s));
        const auto& Ra = a.pose.rotation();
        const auto dR = (Ra.inverse() * b.pose.rotation()).log();
        const auto R = Ra * math::lie::SO3<T>::exp({dR[0] * s, dR[1] * s, dR[2] * s});
        const auto& pa = a.pose.translation();
        const auto& pb = b.pose.translation();
        const Vec3 p{{pa[0] + s * (pb[0] - pa[0]), pa[1] + s * (pb[1] - pa[1]), pa[2] + s * (pb[2] - pa[2])}};
        return math::lie::SE3<T>(R, p);
    }

private:
    struct Landmark {
        std::uint64_t id;
        Vec3 p_world;
    };

    /// The clean pixel of a camera-frame point, if it is in front of the camera
    /// and inside the image's border.
    [[nodiscard]] std::optional<std::pair<T, T>> project(const Vec3& p) const {
        if (!(p[2] > T(0.1)))
            return std::nullopt;
        // Far outside the field of view the radtan polynomial folds back into
        // the image; EuRoC cam0's corners are at a normalized radius of ~1.
        const T x = p[0] / p[2], y = p[1] / p[2];
        if (!(x * x + y * y < T(1.5)))
            return std::nullopt;
        const auto q = cam_.project({p[0], p[1], p[2]});
        const T lo = static_cast<T>(opt_.border_px);
        if (!(q[0] >= lo && q[1] >= lo && q[0] <= T(opt_.width - 1) - lo && q[1] <= T(opt_.height - 1) - lo))
            return std::nullopt;
        return std::pair<T, T>{q[0], q[1]};
    }

    std::vector<sdk::eval::StampedPose<T>> gt_;
    math::cameras::PinholeRadtanCamera<T> cam_;
    math::lie::SE3<T> T_imu_cam_;
    IdealTrackOptions opt_;
    std::mt19937_64 rng_;        ///< landmark geometry only
    std::mt19937_64 noise_rng_;  ///< pixel noise only, so σ does not change the layout of a seed
    std::normal_distribution<double> unit_{0.0, 1.0};
    std::vector<Landmark> live_;
    std::uint64_t next_id_ = 0;
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_IDEAL_TRACKS_HPP
