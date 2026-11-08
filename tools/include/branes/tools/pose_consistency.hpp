// SPDX-License-Identifier: MIT
//
// branes/tools/pose_consistency.hpp — the pose NEES of a real-data run against
// its ground truth, split into roll/pitch, yaw and position (issue #605).
//
// A VIO cannot observe its global position or its yaw about gravity, so the
// estimate's world frame differs from the ground truth's by those four
// directions. The accumulator fixes that gauge once, at the first estimate that
// has a ground-truth match: it rotates the truth about +z and shifts it so that
// the two agree in yaw and position there (sdk::eval::gauge_align, the
// Zhang–Scaramuzza "posyaw" anchor). Roll and pitch are observable and are not
// aligned, so their error stays in the NEES. From then on each frame's error
// is the truth's drift away from the estimate, and the NEES asks whether the
// filter's covariance covers it:
//
//   - attitude is expressed in the WORLD frame, so yaw (about gravity) separates
//     from roll/pitch;
//   - each block's NEES is eᵀ P⁻¹ e over its own components (target: its number
//     of components; roll/pitch 2, yaw 1, position 3, pose 6), averaged over the
//     frames (the ANEES);
//   - the world-yaw σ is tracked: a consistent filter never lowers it below
//     its value at the anchor, since nothing observes yaw.
//
// The error is taken in the filter's own error-state convention, so its own
// covariance applies without a change of coordinates:
//
//   - body_right (MsckfBackend):     R_true = R̂·Exp(θ), p_true = p̂ + δp;
//                                    world attitude error R̂·θ.
//   - right_invariant (R-IEKF):      R_true = Exp(φ)·R̂, p_true = Exp(φ)·p̂ + ρ;
//                                    φ is already in the world frame.
//
// The anchor frame itself has zero yaw and position error by construction.
// The yaw (and position) ANEES is therefore only approximate. Its error is the
// change since the anchor, but its denominator is the current variance alone.
// The variance of that change also needs the anchor's variance and the
// covariance between the two times, which neither filter exposes. With a large
// yaw prior that the filter never reduces, the current variance mostly
// reflects the prior and the ANEES reads low. The world-yaw σ is the sound
// signal: nothing observes yaw, so it should not fall below its value at the
// anchor (yaw_sigma_min against yaw_sigma_first). The EuRoC gate
// (scripts/euroc-consistency.sh) gates on that ratio and reports the ANEES for
// information only.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_POSE_CONSISTENCY_HPP
#define BRANES_TOOLS_POSE_CONSISTENCY_HPP

#include <branes/math/lie/se3.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/eval/nav_consistency.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>
#include <branes/sdk/msckf/dense.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

namespace branes::tools {

/// Which error-state convention the covariance is written in.
enum class PoseErrorConvention { body_right, right_invariant };

/// Per-run summary: ANEES by block, and the world-yaw σ over the run.
struct PoseConsistencySummary {
    std::size_t frames = 0;        ///< frames scored (a ground-truth match and a PD covariance)
    std::size_t not_pd = 0;        ///< frames skipped because a block's covariance was not PD or not finite
    std::size_t unmatched = 0;     ///< frames with no ground truth within max_dt
    double nees_pose = 0.0;        ///< 6-DoF attitude + position, target 6
    double nees_rp = 0.0;          ///< world roll/pitch, target 2
    double nees_yaw = 0.0;         ///< world yaw, target 1
    double nees_pos = 0.0;         ///< position, target 3
    double yaw_sigma_first = 0.0;  ///< world-yaw σ at the anchor (rad)
    double yaw_sigma_min = 0.0;    ///< smallest world-yaw σ over the run (rad)
    double yaw_sigma_last = 0.0;   ///< world-yaw σ at the last scored frame (rad)
    double yaw_err_last = 0.0;     ///< |world-yaw error| at the last scored frame (rad)
    double yaw_err_max = 0.0;      ///< largest |world-yaw error| over the run (rad)

    /// One-line JSON object (no trailing newline), full precision.
    void write_json(std::ostream& os) const {
        const auto flags = os.flags();
        const auto prec = os.precision(std::numeric_limits<double>::max_digits10);
        os << std::defaultfloat;
        const auto num = [&](double v) -> std::ostream& {
            if (std::isfinite(v))
                os << v;
            else
                os << "null";  // JSON has no inf/nan
            return os;
        };
        os << "{\"frames\":" << frames << ",\"not_pd\":" << not_pd << ",\"unmatched\":" << unmatched;
        os << ",\"nees_pose\":";
        num(nees_pose) << ",\"nees_rp\":";
        num(nees_rp) << ",\"nees_yaw\":";
        num(nees_yaw) << ",\"nees_pos\":";
        num(nees_pos) << ",\"yaw_sigma_first\":";
        num(yaw_sigma_first) << ",\"yaw_sigma_min\":";
        num(yaw_sigma_min) << ",\"yaw_sigma_last\":";
        num(yaw_sigma_last) << ",\"yaw_err_last\":";
        num(yaw_err_last) << ",\"yaw_err_max\":";
        num(yaw_err_max) << '}';
        os.precision(prec);
        os.flags(flags);
    }
};

namespace pose_consistency_detail {

/// eᵀ A⁻¹ e for a small symmetric A (n ≤ 6) by Cholesky; nullopt when A is
/// not positive-definite or not finite.
template <std::size_t N>
[[nodiscard]] std::optional<double> quad_inv(const std::array<std::array<double, 6>, 6>& A,
                                             const std::array<double, 6>& e,
                                             const std::array<std::size_t, N>& c) {
    std::array<std::array<double, N>, N> L{};
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j <= i; ++j) {
            double s = A[c[i]][c[j]];
            for (std::size_t k = 0; k < j; ++k)
                s -= L[i][k] * L[j][k];
            if (i == j) {
                if (!(s > 0.0) || !std::isfinite(s))
                    return std::nullopt;
                L[i][i] = std::sqrt(s);
            } else {
                L[i][j] = s / L[j][j];
            }
        }
    }
    // Forward-substitute L y = e; then eᵀ A⁻¹ e = ‖y‖².
    std::array<double, N> y{};
    double q = 0.0;
    for (std::size_t i = 0; i < N; ++i) {
        double s = e[c[i]];
        for (std::size_t k = 0; k < i; ++k)
            s -= L[i][k] * y[k];
        y[i] = s / L[i][i];
        q += y[i] * y[i];
    }
    if (!std::isfinite(q))
        return std::nullopt;
    return q;
}

}  // namespace pose_consistency_detail

/// Accumulates the gauge-anchored, world-frame pose NEES of a run against a
/// time-sorted ground-truth trajectory (T_world_imu).
template <class T>
class PoseConsistency {
public:
    using SE3 = math::lie::SE3<T>;
    using SO3 = math::lie::SO3<T>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Mat = sdk::msckf::DynMat<T>;

    explicit PoseConsistency(std::vector<sdk::eval::StampedPose<T>> gt, double max_dt = 0.02)
        : gt_(std::move(gt)), max_dt_(max_dt) {}

    /// Score one frame. `P` is the filter's full covariance; its attitude block
    /// starts at `theta` and its position block at `pos`. Returns false when the
    /// frame was not scored (no ground truth within max_dt, or a block not PD).
    bool add(double t_s, const SE3& est, const Mat& P, std::size_t theta, std::size_t pos, PoseErrorConvention c) {
        const auto truth_raw = truth_at(t_s);
        if (!truth_raw) {
            ++sum_.unmatched;
            return false;
        }
        if (!anchor_)
            anchor_ = sdk::eval::gauge_align<T>(est, *truth_raw);
        const SE3 truth = *anchor_ * *truth_raw;

        const SO3& Rh = est.rotation();
        const Vec3 ph = est.translation();
        const SO3& Rt = truth.rotation();
        const Vec3 pt = truth.translation();

        // Error in the filter's convention, and the Jacobian taking it to the world frame.
        Vec3 th{}, dp{};
        std::array<std::array<double, 3>, 3> Jrot{};  // world attitude = Jrot · θ
        if (c == PoseErrorConvention::body_right) {
            th = (Rh.inverse() * Rt).log();
            dp = Vec3{{pt[0] - ph[0], pt[1] - ph[1], pt[2] - ph[2]}};
            const auto M = Rh.matrix();
            for (std::size_t i = 0; i < 3; ++i)
                for (std::size_t j = 0; j < 3; ++j)
                    Jrot[i][j] = static_cast<double>(M(i, j));
        } else {
            const SO3 Re = Rt * Rh.inverse();
            th = Re.log();
            const Vec3 rp = Re * ph;
            dp = Vec3{{pt[0] - rp[0], pt[1] - rp[1], pt[2] - rp[2]}};
            for (std::size_t i = 0; i < 3; ++i)
                Jrot[i][i] = 1.0;
        }

        // e_w = J e, P_w = J P6 Jᵀ with J = diag(Jrot, I).
        std::array<double, 6> e{};
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j)
                e[i] += Jrot[i][j] * static_cast<double>(th[j]);
            e[3 + i] = static_cast<double>(dp[i]);
        }
        const std::array<std::size_t, 6> idx{theta, theta + 1, theta + 2, pos, pos + 1, pos + 2};
        std::array<std::array<double, 6>, 6> P6{}, J{}, JP{}, Pw{};
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                P6[i][j] = static_cast<double>(P(idx[i], idx[j]));
        for (std::size_t i = 0; i < 3; ++i) {
            J[3 + i][3 + i] = 1.0;
            for (std::size_t j = 0; j < 3; ++j)
                J[i][j] = Jrot[i][j];
        }
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                for (std::size_t k = 0; k < 6; ++k)
                    JP[i][j] += J[i][k] * P6[k][j];
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                for (std::size_t k = 0; k < 6; ++k)
                    Pw[i][j] += JP[i][k] * J[j][k];

        using pose_consistency_detail::quad_inv;
        const auto all = quad_inv(Pw, e, std::array<std::size_t, 6>{0, 1, 2, 3, 4, 5});
        const auto rp = quad_inv(Pw, e, std::array<std::size_t, 2>{0, 1});
        const auto yaw = quad_inv(Pw, e, std::array<std::size_t, 1>{2});
        const auto ps = quad_inv(Pw, e, std::array<std::size_t, 3>{3, 4, 5});
        if (!all || !rp || !yaw || !ps) {
            ++sum_.not_pd;
            return false;
        }
        nees_[0] += *all;
        nees_[1] += *rp;
        nees_[2] += *yaw;
        nees_[3] += *ps;

        const double ys = std::sqrt(Pw[2][2]);
        const double ye = std::abs(e[2]);
        if (sum_.frames == 0) {
            sum_.yaw_sigma_first = ys;
            sum_.yaw_sigma_min = ys;
        }
        sum_.yaw_sigma_min = std::min(sum_.yaw_sigma_min, ys);
        sum_.yaw_sigma_last = ys;
        sum_.yaw_err_last = ye;
        sum_.yaw_err_max = std::max(sum_.yaw_err_max, ye);
        ++sum_.frames;
        return true;
    }

    [[nodiscard]] PoseConsistencySummary summary() const {
        PoseConsistencySummary s = sum_;
        if (s.frames) {
            const double n = static_cast<double>(s.frames);
            s.nees_pose = nees_[0] / n;
            s.nees_rp = nees_[1] / n;
            s.nees_yaw = nees_[2] / n;
            s.nees_pos = nees_[3] / n;
        }
        return s;
    }

    /// The truth→estimate gauge fixed at the first scored frame, if any.
    [[nodiscard]] const std::optional<SE3>& anchor() const {
        return anchor_;
    }

private:
    /// The ground-truth pose nearest `t_s`, if one lies within max_dt.
    [[nodiscard]] std::optional<SE3> truth_at(double t_s) const {
        if (gt_.empty())
            return std::nullopt;
        const auto it = std::lower_bound(
            gt_.begin(), gt_.end(), t_s, [](const sdk::eval::StampedPose<T>& p, double t) { return p.t_s < t; });
        const sdk::eval::StampedPose<T>* best = nullptr;
        if (it != gt_.end())
            best = &*it;
        if (it != gt_.begin()) {
            const auto& prev = *std::prev(it);
            if (!best || std::abs(prev.t_s - t_s) < std::abs(best->t_s - t_s))
                best = &prev;
        }
        if (!best || std::abs(best->t_s - t_s) > max_dt_)
            return std::nullopt;
        return best->pose;
    }

    std::vector<sdk::eval::StampedPose<T>> gt_;
    double max_dt_;
    std::optional<SE3> anchor_;
    std::array<double, 4> nees_{};
    PoseConsistencySummary sum_;
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_POSE_CONSISTENCY_HPP
