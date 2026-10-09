// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s5_triangulation.hpp — stage S5_triangulation as an
// explicit transformation (issue #452, docs/arch/vio-pipeline-canonical.md §S5).
//
//   apply(state, updater, track) → (p_f ∈ ℝ³ | rejected)
//
// Validates the track (≥ 2 observations, clone and camera indices in the
// window) and triangulates the feature from the clone poses: the linear
// ray-perpendicular solve, the optional parallax gate, then Gauss-Newton
// reprojection refinement. Reads the state; never mutates it.
//
//   apply(state, updater, track, method) → the same, by an alternative method
//
// The alternatives are candidates the S5 bench measures against the shipped
// solve (#444); VioEstimator does not use them. They share the updater's camera
// model (`camera_pose`, `projection_jacobians`) and ignore the parallax gate:
//
//   Midpoint      the widest-parallax pair of rays; the midpoint of their common
//                 perpendicular. Two views, closed form.
//   Dlt           inhomogeneous DLT: the algebraic error x·(r₃·X + t₃) − (r₁·X + t₁)
//                 over every view, solved in least squares (X with W = 1).
//   InverseDepth  anchored inverse depth (α, β, ρ) in the first observing camera,
//                 seeded by the DLT, refined by Gauss-Newton on the reprojection
//                 error — the parameterization OpenVINS-class filters use, which
//                 stays well-posed as parallax → 0 (ρ → 0, not depth → ∞).
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S5_TRIANGULATION_HPP
#define BRANES_SDK_MSCKF_STAGES_S5_TRIANGULATION_HPP

#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/dense.hpp>

#include <cstddef>
#include <vector>

namespace branes::sdk::msckf::stages::s5_triangulation {

template <math::Scalar T>
struct Result {
    bool ok = false;                     ///< false: invalid track, degenerate geometry, or gated parallax
    math::lie::detail::Vec<T, 3> p_f{};  ///< feature position in the world frame
};

template <math::Scalar T, class Cov>
[[nodiscard]] Result<T> apply(const State<T, Cov>& s, const CameraUpdater<T>& updater, const FeatureTrack<T>& track) {
    Result<T> out;
    if (!updater.validate_track(s, track))
        return out;
    out.ok = updater.triangulate(s, track.observations, out.p_f);
    return out;
}

/// The triangulation method: the shipped solve, or an alternative under study.
enum class Method { Shipped, Midpoint, Dlt, InverseDepth };

namespace detail {

template <math::Scalar T>
using V3 = math::lie::detail::Vec<T, 3>;

template <math::Scalar T>
T dot(const V3<T>& a, const V3<T>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/// Solve the 3×3 symmetric system A·x = b by Cholesky; false if not SPD.
template <math::Scalar T>
bool solve3(const DynMat<T>& A, const DynMat<T>& b, V3<T>& x) {
    DynMat<T> L;
    if (!cholesky(A, L))
        return false;
    const DynMat<T> s = cholesky_solve(L, b);
    x = V3<T>{{s(0, 0), s(1, 0), s(2, 0)}};
    return true;
}

/// True when `p_f` projects in front of every observing camera.
template <math::Scalar T, class Cov>
bool in_front(const State<T, Cov>& s, const CameraUpdater<T>& upd, const FeatureTrack<T>& track, const V3<T>& p_f) {
    for (const auto& o : track.observations)
        if (!upd.projection_jacobians(s, o, p_f).valid)
            return false;
    return true;
}

template <math::Scalar T, class Cov>
bool midpoint(const State<T, Cov>& s, const CameraUpdater<T>& upd, const FeatureTrack<T>& track, V3<T>& p_f) {
    std::vector<typename CameraUpdater<T>::CameraPose> poses;
    std::vector<V3<T>> rays;
    for (const auto& o : track.observations) {
        poses.push_back(upd.camera_pose(s, o));
        V3<T> d = poses.back().R_wc * V3<T>{{o.xy[0], o.xy[1], T{1}}};
        const T n = math::lie::detail::norm(d);
        if (!(n > T{0}))
            return false;
        rays.push_back(d * (T{1} / n));
    }
    // The widest pair (smallest cosine) carries the most depth information.
    std::size_t bi = 0, bj = 1;
    T best = T{2};
    for (std::size_t i = 0; i < rays.size(); ++i)
        for (std::size_t j = i + 1; j < rays.size(); ++j)
            if (const T c = dot(rays[i], rays[j]); c < best) {
                best = c;
                bi = i;
                bj = j;
            }
    // Closest points c₁ + s·d₁ and c₂ + t·d₂ (unit d): [1 −c; c −1]·[s t]ᵀ = [d₁·w; d₂·w].
    const V3<T> w = poses[bj].center - poses[bi].center;
    const T c = best, den = T{1} - c * c;
    if (!(den > T{0}))
        return false;  // parallel rays
    const T e1 = dot(rays[bi], w), e2 = dot(rays[bj], w);
    const T sp = (e1 - c * e2) / den, tp = (c * e1 - e2) / den;
    p_f = (poses[bi].center + rays[bi] * sp + poses[bj].center + rays[bj] * tp) * T(0.5);
    return in_front(s, upd, track, p_f);
}

template <math::Scalar T, class Cov>
bool dlt(const State<T, Cov>& s, const CameraUpdater<T>& upd, const FeatureTrack<T>& track, V3<T>& p_f) {
    DynMat<T> A(3, 3), b(3, 1);
    for (const auto& o : track.observations) {
        const auto pose = upd.camera_pose(s, o);
        // Camera ← world: R = R_wcᵀ, t = −R·c. Rows r_k and offsets t_k.
        V3<T> r[3];
        T t[3];
        for (std::size_t k = 0; k < 3; ++k) {
            r[k] = V3<T>{{pose.R_wc(0, k), pose.R_wc(1, k), pose.R_wc(2, k)}};
            t[k] = -dot(r[k], pose.center);
        }
        for (std::size_t m = 0; m < 2; ++m) {
            // (x_m·r₃ − r_m)·X = t_m − x_m·t₃
            const V3<T> a = r[2] * o.xy[m] - r[m];
            const T rhs = t[m] - o.xy[m] * t[2];
            for (std::size_t i = 0; i < 3; ++i) {
                for (std::size_t j = 0; j < 3; ++j)
                    A(i, j) += a[i] * a[j];
                b(i, 0) += a[i] * rhs;
            }
        }
    }
    return solve3(A, b, p_f) && in_front(s, upd, track, p_f);
}

template <math::Scalar T, class Cov>
bool inverse_depth(const State<T, Cov>& s, const CameraUpdater<T>& upd, const FeatureTrack<T>& track, V3<T>& p_f) {
    if (!dlt(s, upd, track, p_f))
        return false;
    const auto anchor = upd.camera_pose(s, track.observations.front());
    // Seed (α, β, ρ) from the DLT point in the anchor camera.
    const V3<T> pa = math::lie::detail::transpose(anchor.R_wc) * (p_f - anchor.center);
    if (!(pa[2] > T{0}))
        return false;
    T al = pa[0] / pa[2], be = pa[1] / pa[2], rho = T{1} / pa[2];
    auto point = [&](T a, T bb, T r) { return anchor.center + anchor.R_wc * V3<T>{{a, bb, T{1}}} * (T{1} / r); };
    const std::size_t iters = std::max<std::size_t>(upd.options().max_triangulation_iters, 5);
    for (std::size_t it = 0; it < iters; ++it) {
        p_f = point(al, be, rho);
        // ∂p_f/∂(α, β, ρ) = R_wa·[e₁/ρ, e₂/ρ, −(α, β, 1)/ρ²].
        const T ir = T{1} / rho;
        const V3<T> col[3] = {anchor.R_wc * V3<T>{{ir, T{0}, T{0}}},
                              anchor.R_wc * V3<T>{{T{0}, ir, T{0}}},
                              anchor.R_wc * V3<T>{{-al * ir * ir, -be * ir * ir, -ir * ir}}};
        DynMat<T> H(3, 3), g(3, 1);
        for (const auto& o : track.observations) {
            const auto J = upd.projection_jacobians(s, o, p_f);
            if (!J.valid)
                return false;
            for (std::size_t k = 0; k < 2; ++k) {
                const T res = o.xy[k] - J.h[k];
                T Jk[3];
                for (std::size_t c = 0; c < 3; ++c)
                    Jk[c] = J.Hf(k, 0) * col[c][0] + J.Hf(k, 1) * col[c][1] + J.Hf(k, 2) * col[c][2];
                for (std::size_t i = 0; i < 3; ++i) {
                    for (std::size_t j = 0; j < 3; ++j)
                        H(i, j) += Jk[i] * Jk[j];
                    g(i, 0) += Jk[i] * res;
                }
            }
        }
        V3<T> dx;
        if (!solve3(H, g, dx))
            break;  // keep the last point
        al += dx[0];
        be += dx[1];
        rho += dx[2];
        if (!(rho > T{0}))
            return false;  // stepped behind the anchor
    }
    p_f = point(al, be, rho);
    return in_front(s, upd, track, p_f);
}

}  // namespace detail

/// Triangulate by `method`. `Method::Shipped` is `apply(s, updater, track)`.
template <math::Scalar T, class Cov>
[[nodiscard]] Result<T>
apply(const State<T, Cov>& s, const CameraUpdater<T>& updater, const FeatureTrack<T>& track, Method method) {
    if (method == Method::Shipped)
        return apply(s, updater, track);
    Result<T> out;
    if (!updater.validate_track(s, track))
        return out;
    switch (method) {
    case Method::Midpoint:
        out.ok = detail::midpoint(s, updater, track, out.p_f);
        break;
    case Method::Dlt:
        out.ok = detail::dlt(s, updater, track, out.p_f);
        break;
    case Method::InverseDepth:
        out.ok = detail::inverse_depth(s, updater, track, out.p_f);
        break;
    case Method::Shipped:
        break;
    }
    if (!out.ok)
        out.p_f = {};
    return out;
}

}  // namespace branes::sdk::msckf::stages::s5_triangulation

#endif  // BRANES_SDK_MSCKF_STAGES_S5_TRIANGULATION_HPP
