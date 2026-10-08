// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/invariants/geometry_checks.hpp — the manifold, sensor-model,
// and measurement-geometry invariants of the VIO stages (epic #444 §B, issue
// #445): SO(3) membership, unit quaternions, camera intrinsics, gravity, positive
// depth, image bounds, bounded residuals, round trips, and analytic Jacobians
// against central finite differences.
//
// Manifold and Jacobian checks take arithmetic tolerances (core.hpp). Checks whose
// bound is physical — a pixel margin, a reprojection bound, a gravity magnitude
// window — take that bound as an explicit parameter.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_INVARIANTS_GEOMETRY_CHECKS_HPP
#define BRANES_SDK_EVAL_INVARIANTS_GEOMETRY_CHECKS_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/eval/invariants/core.hpp>
#include <branes/sdk/eval/invariants/linalg.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

namespace branes::sdk::eval::inv {

template <math::Scalar T>
using Mat3 = math::lie::detail::Mat<T, 3, 3>;
template <math::Scalar T>
using Vec3 = math::lie::detail::Vec<T, 3>;

// ── Manifold membership ─────────────────────────────────────────────────────

/// R ∈ SO(3): two results, orthogonality max |RᵀR − I| and |det R − 1|, each held
/// to safety·3·ε_T. Use for the propagated attitude (S2), the initial attitude
/// (S1), and the camera–IMU extrinsic (inputs, S10). A reflection (det = −1)
/// passes orthogonality and fails the determinant.
template <math::Scalar T>
[[nodiscard]] std::vector<InvariantResult> check_so3(const Mat3<T>& r,
                                                     Stage stage,
                                                     std::string_view orth_name = "so3.orthogonality",
                                                     std::string_view det_name = "so3.determinant",
                                                     double safety = kDefaultSafety) {
    if (!detail::all_finite<T>(std::span<const T>(r.e)))
        return {detail::nonfinite_violation(orth_name, stage), detail::nonfinite_violation(det_name, stage)};
    T worst{0};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) {
            T s{0};
            for (std::size_t k = 0; k < 3; ++k)
                s += r(k, i) * r(k, j);
            worst = std::max(worst, la::abs_(s - (i == j ? T{1} : T{0})));
        }
    const T det = r(0, 0) * (r(1, 1) * r(2, 2) - r(1, 2) * r(2, 1)) -
                  r(0, 1) * (r(1, 0) * r(2, 2) - r(1, 2) * r(2, 0)) + r(0, 2) * (r(1, 0) * r(2, 1) - r(1, 1) * r(2, 0));
    const double tol = arithmetic_tolerance<T>(3, 1.0, safety);
    return {detail::make(orth_name, "dimensionless", stage, Bound::Upper, detail::to_double(worst), tol),
            detail::make(det_name, "dimensionless", stage, Bound::Upper, detail::to_double(la::abs_(det - T{1})), tol)};
}

template <math::Scalar T>
[[nodiscard]] std::vector<InvariantResult> check_so3(const math::lie::SO3<T>& r,
                                                     Stage stage,
                                                     std::string_view orth_name = "so3.orthogonality",
                                                     std::string_view det_name = "so3.determinant",
                                                     double safety = kDefaultSafety) {
    return check_so3<T>(r.matrix(), stage, orth_name, det_name, safety);
}

/// ‖q‖ = 1 for a raw quaternion (w, x, y, z). Value: |‖q‖ − 1|; threshold
/// safety·4·ε_T. (math::lie::SO3 renormalizes on construction; this is for
/// quaternions carried outside it — captured state, an external estimator.)
template <math::Scalar T>
[[nodiscard]] InvariantResult check_unit_quaternion(std::span<const T, 4> q,
                                                    Stage stage,
                                                    std::string_view name = "quaternion.unit_norm",
                                                    double safety = kDefaultSafety) {
    const T n = la::sqrt_(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    return detail::make(name,
                        "dimensionless",
                        stage,
                        Bound::Upper,
                        detail::to_double(la::abs_(n - T{1})),
                        arithmetic_tolerance<T>(4, 1.0, safety));
}

// ── Camera intrinsics ───────────────────────────────────────────────────────

/// Pinhole intrinsics are physical (inputs): two results, min(fx, fy) > 0 (px;
/// lower bound one pixel, a focal length below that is a units error) and the
/// principal point inside the image — distance outside [0, w] × [0, h] in px,
/// threshold 0.
template <math::Scalar T>
[[nodiscard]] std::vector<InvariantResult>
check_intrinsics(T fx, T fy, T cx, T cy, T width, T height, Stage stage = Stage::Inputs) {
    if (!detail::all_finite<T>(std::array<T, 6>{fx, fy, cx, cy, width, height}))
        return {detail::nonfinite_violation("intrinsics.focal_positive", stage, Bound::Lower),
                detail::nonfinite_violation("intrinsics.principal_point_in_image", stage)};
    const T fmin = std::min(fx, fy);
    const T ox = cx < T{0} ? -cx : (cx > width ? cx - width : T{0});
    const T oy = cy < T{0} ? -cy : (cy > height ? cy - height : T{0});
    return {detail::make("intrinsics.focal_positive", "px", stage, Bound::Lower, detail::to_double(fmin), 1.0),
            detail::make("intrinsics.principal_point_in_image",
                         "px",
                         stage,
                         Bound::Upper,
                         detail::to_double(std::max(ox, oy)),
                         0.0)};
}

// ── Gravity (S1, the #262 contract) ─────────────────────────────────────────

/// The world gravity vector points DOWN with the nominal magnitude: two results,
/// | ‖g‖ − g₀ | ≤ `magnitude_tol` (m/s²) and the angle between g and −ẑ ≤
/// `direction_tol` (rad). A sign error (+ẑ) shows as an angle of π.
template <math::Scalar T>
[[nodiscard]] std::vector<InvariantResult> check_gravity(
    const Vec3<T>& g_world, T g_nominal, T magnitude_tol, T direction_tol, Stage stage = Stage::S1_initialization) {
    const T n = la::sqrt_(g_world[0] * g_world[0] + g_world[1] * g_world[1] + g_world[2] * g_world[2]);
    double angle = 3.141592653589793;
    if (n > T{0}) {
        const double c = std::clamp(detail::to_double(-g_world[2] / n), -1.0, 1.0);
        angle = std::acos(c);
    }
    return {detail::make("gravity.magnitude",
                         "m/s^2",
                         stage,
                         Bound::Upper,
                         detail::to_double(la::abs_(n - g_nominal)),
                         detail::to_double(magnitude_tol)),
            detail::make("gravity.direction", "rad", stage, Bound::Upper, angle, detail::to_double(direction_tol))};
}

// ── Bounds on samples and residuals ─────────────────────────────────────────

/// max |xᵢ| ≤ bound — sensor-range limits (inputs), bias bounds (S1), the KLT
/// forward–backward error (S4), the reprojection error (S5), an ATE bound (end to
/// end). `unit` names what the values are in.
template <math::Scalar T>
[[nodiscard]] InvariantResult
check_bounded(std::span<const T> x, T bound, Stage stage, std::string_view name, std::string_view unit) {
    if (!detail::all_finite(x))
        return detail::nonfinite_violation(name, stage);
    T worst{0};
    for (const T& v : x)
        worst = std::max(worst, la::abs_(v));
    return detail::make(name, unit, stage, Bound::Upper, detail::to_double(worst), detail::to_double(bound));
}

/// min xᵢ ≥ floor — positive depth of a triangulated feature in every observing
/// camera (S5, floor > 0), positive variances, …
template <math::Scalar T>
[[nodiscard]] InvariantResult
check_at_least(std::span<const T> x, T floor, Stage stage, std::string_view name, std::string_view unit) {
    if (x.empty())
        return detail::make(name, unit, stage, Bound::Lower, detail::to_double(floor), detail::to_double(floor));
    if (!detail::all_finite(x))
        return detail::nonfinite_violation(name, stage, Bound::Lower);
    T lo = x[0];
    for (const T& v : x)
        lo = std::min(lo, v);
    return detail::make(name, unit, stage, Bound::Lower, detail::to_double(lo), detail::to_double(floor));
}

/// Pixel tracks inside the image (S4): value = the count of points outside
/// [margin, w − margin] × [margin, h − margin]; threshold 0.
template <math::Scalar T>
[[nodiscard]] InvariantResult check_points_in_image(std::span<const std::array<T, 2>> pts,
                                                    T width,
                                                    T height,
                                                    T margin = T{0},
                                                    Stage stage = Stage::S4_frontend,
                                                    std::string_view name = "tracks.in_image") {
    std::size_t out = 0;
    for (const auto& p : pts) {
        const bool inside = p[0] >= margin && p[0] <= width - margin && p[1] >= margin && p[1] <= height - margin;
        out += inside ? 0 : 1;
    }
    return detail::make(name, "count", stage, Bound::Upper, static_cast<double>(out), 0.0);
}

/// Track bookkeeping (S4 ↔ backend): no track is longer than the clone window
/// that has to hold its observations, and none is empty. Value: the count of
/// inconsistent tracks; threshold 0.
[[nodiscard]] inline InvariantResult check_track_lengths(std::span<const std::size_t> lengths,
                                                         std::size_t window_clones,
                                                         Stage stage = Stage::S4_frontend,
                                                         std::string_view name = "tracks.length_vs_clones") {
    std::size_t bad = 0;
    for (const std::size_t l : lengths)
        bad += (l == 0 || l > window_clones) ? 1 : 0;
    return detail::make(name, "count", stage, Bound::Upper, static_cast<double>(bad), 0.0);
}

/// A measured statistic held to a fixed physical bound (ATE, RPE, a probe's
/// headline number). `bound` direction as given.
[[nodiscard]] inline InvariantResult
check_scalar(double value, double threshold, Bound bound, Stage stage, std::string_view name, std::string_view unit) {
    return detail::make(name, unit, stage, bound, value, threshold);
}

// ── Round trips and Jacobians (S0, S6a) ─────────────────────────────────────

/// a ≈ b after a round trip (project ∘ unproject, distort ∘ undistort). Value:
/// max |aᵢ − bᵢ|; threshold safety·n·ε_T·max(1, ‖a‖_max). Iterative inverses
/// (undistortion) converge to ~100·ε_T, so pass their own safety if needed.
template <math::Scalar T>
[[nodiscard]] InvariantResult check_round_trip(std::span<const T> a,
                                               std::span<const T> b,
                                               Stage stage,
                                               std::string_view name,
                                               std::string_view unit,
                                               double safety = kDefaultSafety) {
    if (a.size() != b.size())
        return detail::make(name, "shape", stage, Bound::Upper, detail::kInf, 0.0);
    if (!detail::all_finite(a) || !detail::all_finite(b))
        return detail::nonfinite_violation(name, stage);
    T worst{0}, scale{1};
    for (std::size_t i = 0; i < a.size(); ++i) {
        worst = std::max(worst, la::abs_(a[i] - b[i]));
        scale = std::max(scale, la::abs_(a[i]));
    }
    return detail::make(name,
                        unit,
                        stage,
                        Bound::Upper,
                        detail::to_double(worst),
                        arithmetic_tolerance<T>(a.size(), detail::to_double(scale), safety));
}

/// An analytic Jacobian J (m×n, row-major) of f: Tⁿ → Tᵐ at x matches central
/// finite differences. The step h_j = ε_T^{1/3}·max(1, |x_j|) balances the O(h²)
/// truncation error against the O(ε_T/h) roundoff, so the FD Jacobian itself is
/// good to ~ε_T^{2/3}: value max |J − J_fd|; threshold safety·ε_T^{2/3}·max(1,
/// ‖f(x)‖_max, ‖J‖_max). `f` is callable as `std::vector<T>(std::span<const T>)`.
template <math::Scalar T, class F>
[[nodiscard]] InvariantResult check_jacobian_fd(F&& f,
                                                std::span<const T> x,
                                                std::span<const T> j_analytic,
                                                Stage stage,
                                                std::string_view name = "jacobian_vs_fd",
                                                double safety = kDefaultSafety) {
    const std::vector<T> f0 = f(x);
    const std::size_t m = f0.size(), n = x.size();
    if (j_analytic.size() != m * n)
        return detail::make(name, "shape", stage, Bound::Upper, detail::kInf, 0.0);
    if (!detail::all_finite(x) || !detail::all_finite(j_analytic) || !detail::all_finite<T>(f0))
        return detail::nonfinite_violation(name, stage);
    const double eps = detail::to_double(epsilon<T>());
    const T cbrt_eps = T(std::cbrt(eps));
    std::vector<T> xp(x.begin(), x.end());
    T worst{0}, scale{1};
    for (const T& v : f0)
        scale = std::max(scale, la::abs_(v));
    for (const T& v : j_analytic)
        scale = std::max(scale, la::abs_(v));
    for (std::size_t c = 0; c < n; ++c) {
        const T h = cbrt_eps * std::max(T{1}, la::abs_(x[c]));
        const T x0 = xp[c];
        xp[c] = x0 + h;
        const std::vector<T> fp = f(std::span<const T>(xp));
        xp[c] = x0 - h;
        const std::vector<T> fm = f(std::span<const T>(xp));
        xp[c] = x0;
        if (!detail::all_finite<T>(fp) || !detail::all_finite<T>(fm))
            return detail::nonfinite_violation(name, stage);
        const T two_h = (x0 + h) - (x0 - h);  // the step actually taken
        for (std::size_t r = 0; r < m; ++r) {
            const T fd = (fp[r] - fm[r]) / two_h;
            worst = std::max(worst, la::abs_(j_analytic[r * n + c] - fd));
        }
    }
    const double tol = safety * std::cbrt(eps * eps) * detail::to_double(scale);
    return detail::make(name, "jacobian units", stage, Bound::Upper, detail::to_double(worst), tol);
}

}  // namespace branes::sdk::eval::inv

#endif  // BRANES_SDK_EVAL_INVARIANTS_GEOMETRY_CHECKS_HPP
