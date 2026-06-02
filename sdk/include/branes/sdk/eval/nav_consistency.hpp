// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/nav_consistency.hpp — NEES of a VIO nav-state estimate
// against ground truth.
//
// The ground-truth half of the consistency instrument (#264): given the
// estimated navigation state (R, p, v, bg, ba) with its 15-dim error-state
// covariance and the true state, form the tangent-space error and feed it to
// `nees` / `ConsistencyAccumulator` (consistency.hpp). NEES tests whether the
// covariance is consistent with the *actual* error — the run average is dim
// (15) when consistent, > 1·dim when over-confident, < 1·dim when under.
//
// Two VIO-specific details this header handles:
//   1. Error-state convention — must match `msckf::State`'s layout
//      [δθ δp δv δbg δba], world-centric, right perturbation R_true = R̂·Exp(δθ),
//      so δθ = Log(R̂⁻¹ R_true) and the rest are additive.
//   2. Gauge — a VIO solution is only fixed up to a global yaw + position
//      (the 4 unobservable DoF). The estimate and truth live in different world
//      frames, so the gauge must be anchored before differencing. `gauge_align`
//      builds the yaw-only (position + yaw, the 4-DoF VIO gauge) transform from
//      one matched pose (typically the first evaluated frame) — roll/pitch are
//      observable and must NOT be absorbed; `align_truth` brings a truth sample
//      into the estimate
//      frame. NEES then measures covariance-vs-error consistency relative to
//      that anchor — including whether the filter grows its yaw uncertainty
//      correctly (yaw being unobservable, the error AND the covariance should
//      grow; NEES stays consistent iff they grow together).
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_NAV_CONSISTENCY_HPP
#define BRANES_SDK_EVAL_NAV_CONSISTENCY_HPP

#include <branes/math/lie/se3.hpp>
#include <branes/sdk/eval/consistency.hpp>

#include <array>
#include <cstddef>
#include <stdexcept>

namespace branes::sdk::eval {

template <math::Scalar T>
using Vec3 = math::lie::detail::Vec<T, 3>;

/// Full IMU navigation state for consistency evaluation, matching the core of
/// `msckf::State`: world←imu rotation, world position/velocity, gyro/accel bias.
template <math::Scalar T>
struct NavSample {
    math::lie::SO3<T> R{};
    Vec3<T> p{};
    Vec3<T> v{};
    Vec3<T> bg{};
    Vec3<T> ba{};
};

/// Number of core error-state dimensions: [δθ δp δv δbg δba].
inline constexpr std::size_t kNavErrorDim = 15;

/// The 15-vector error-state of `est` from `truth`, in `msckf::State`'s
/// convention: `[δθ δp δv δbg δba]` with `δθ = Log(R̂⁻¹ R_true)` (right
/// perturbation) and the remaining blocks additive (`truth − est`). Both states
/// must be in the SAME world frame — anchor the gauge with `align_truth` first.
template <math::Scalar T>
[[nodiscard]] std::array<T, kNavErrorDim> nav_error(const NavSample<T>& est, const NavSample<T>& truth) {
    std::array<T, kNavErrorDim> e{};
    const Vec3<T> dtheta = (est.R.inverse() * truth.R).log();
    const Vec3<T> dp = truth.p - est.p;
    const Vec3<T> dv = truth.v - est.v;
    const Vec3<T> dbg = truth.bg - est.bg;
    const Vec3<T> dba = truth.ba - est.ba;
    for (std::size_t k = 0; k < 3; ++k) {
        e[k] = dtheta[k];
        e[3 + k] = dp[k];
        e[6 + k] = dv[k];
        e[9 + k] = dbg[k];
        e[12 + k] = dba[k];
    }
    return e;
}

/// Transform mapping the truth world into the estimate world, fixing the VIO
/// gauge from one matched pose pair. The gauge is **4-DoF** — global position
/// plus **yaw about gravity** — because roll and pitch are *observable* (the
/// accelerometer measures the gravity direction). So the rotation is yaw-only
/// (about world up, +z); a full-SE3 anchor would wrongly absorb the observable
/// roll/pitch error and report an optimistically small NEES (Zhang &
/// Scaramuzza, "A Tutorial on Quantitative Trajectory Evaluation for VIO",
/// IROS 2018 — the `posyaw` alignment). Applying the result to the truth pose
/// reproduces the estimate's position and yaw at that frame, while leaving any
/// roll/pitch error in the NEES.
template <math::Scalar T>
[[nodiscard]] math::lie::SE3<T> gauge_align(const math::lie::SE3<T>& est_pose, const math::lie::SE3<T>& truth_pose) {
    const auto R_rel = (est_pose.rotation() * truth_pose.rotation().inverse()).matrix();
    const T yaw = math::lie::detail::atan2_(R_rel(1, 0), R_rel(0, 0));  // yaw of R_rel about +z
    const math::lie::SO3<T> R_align = math::lie::SO3<T>::exp(Vec3<T>{{T{0}, T{0}, yaw}});
    return math::lie::SE3<T>(R_align, est_pose.translation() - R_align * truth_pose.translation());
}

/// Bring a truth nav sample into the estimate frame via `T_align`: rotation and
/// position transform rigidly, velocity rotates (no translation), and the
/// biases are IMU-frame quantities, unchanged by a world-frame gauge.
template <math::Scalar T>
[[nodiscard]] NavSample<T> align_truth(const math::lie::SE3<T>& T_align, const NavSample<T>& truth) {
    NavSample<T> out;
    out.R = T_align.rotation() * truth.R;
    out.p = T_align * truth.p;
    out.v = T_align.rotation() * truth.v;
    out.bg = truth.bg;
    out.ba = truth.ba;
    return out;
}

/// Top-left `k×k` (default 15) block of a full error-state covariance — the core
/// nav-state covariance, excluding the clone window. Throws if `full` is smaller
/// than `k×k`.
template <math::Scalar T>
[[nodiscard]] DynMat<T> core_covariance(const DynMat<T>& full, std::size_t k = kNavErrorDim) {
    if (full.rows < k || full.cols < k)
        throw std::invalid_argument("core_covariance: covariance smaller than the requested core block");
    DynMat<T> c(k, k);
    for (std::size_t i = 0; i < k; ++i)
        for (std::size_t j = 0; j < k; ++j)
            c(i, j) = full(i, j);
    return c;
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_NAV_CONSISTENCY_HPP
