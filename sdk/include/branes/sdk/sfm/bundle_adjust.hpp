// SPDX-License-Identifier: MIT
//
// branes/sdk/sfm/bundle_adjust.hpp — bundle adjustment of the dynamic-init SfM
// window (issue #501). The window's poses come from linear solves (an 8-point
// essential matrix and DLT resections) on a short baseline, and its landmarks
// from the bootstrap pair alone. Their position noise biases the IMU alignment's
// scale low: the alignment fits accelerations, second differences of the
// positions, and noise in a regressor attenuates the fit (#501). This refines the
// poses and points jointly by minimizing the reprojection error.
//
// Levenberg–Marquardt over world→camera poses (rotation perturbed on the left,
// R ← Exp(δθ)·R) and points, with Marquardt's diagonal damping and Huber
// weights. Each point's 3×3 block is eliminated by the Schur complement, so the
// dense solve is 6 × (free frames). Frames marked fixed hold the gauge; the
// scale gauge is left free and the damping keeps that direction bounded.
// Plus a linear multi-view triangulation (the N-view DLT).
//
// Clean-room from Triggs, McLauchlan, Hartley & Fitzgibbon, "Bundle Adjustment
// — A Modern Synthesis" (1999) §4, §6.1 (Schur complement), and Hartley &
// Zisserman, "Multiple View Geometry" (2nd ed.) §12.2 (DLT triangulation),
// §18.1, A6. Header-only, C++20, type-generic.

#ifndef BRANES_SDK_SFM_BUNDLE_ADJUST_HPP
#define BRANES_SDK_SFM_BUNDLE_ADJUST_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/dense.hpp>   // DynMat, cholesky
#include <branes/sdk/sfm/two_view.hpp>  // Vec2/Vec3/Mat3, jacobi_eigh

#include <array>
#include <cstddef>
#include <span>
#include <vector>

namespace branes::sdk::sfm {

/// One image observation: `point` seen by `frame` at normalized `x`.
template <math::Scalar T>
struct BaObservation {
    std::size_t frame = 0;
    std::size_t point = 0;
    Vec2<T> x{};
};

template <math::Scalar T>
struct BundleAdjustOptions {
    std::size_t max_iterations = 20;  ///< accepted LM steps
    T huber = T{0};  ///< residual norm (normalized) above which a residual counts linearly; 0 = least squares
    T initial_lambda = T{1} / T{1000};  ///< Marquardt damping to start from
    T min_relative_decrease = T(1e-6);  ///< stop when a step lowers the cost by less than this fraction
};

template <math::Scalar T>
struct BundleAdjustResult {
    bool success = false;  ///< the cost did not increase (false: degenerate input)
    std::size_t iterations = 0;
    T initial_rms = T{0};  ///< RMS reprojection error (normalized) before
    T final_rms = T{0};    ///< …and after
};

namespace detail {

/// Inverse of a symmetric 3×3 by its adjugate; false if near-singular.
template <math::Scalar T>
[[nodiscard]] bool inverse3(const std::array<T, 9>& a, std::array<T, 9>& inv) {
    const T c00 = a[4] * a[8] - a[5] * a[7];
    const T c01 = a[5] * a[6] - a[3] * a[8];
    const T c02 = a[3] * a[7] - a[4] * a[6];
    const T det = a[0] * c00 + a[1] * c01 + a[2] * c02;
    const T scale = ld::abs_(a[0]) + ld::abs_(a[4]) + ld::abs_(a[8]);
    if (!(scale > T{0}) || !(ld::abs_(det) > T(1e-12) * scale * scale * scale))
        return false;
    const T id = T{1} / det;
    inv = {c00 * id,
           (a[2] * a[7] - a[1] * a[8]) * id,
           (a[1] * a[5] - a[2] * a[4]) * id,
           c01 * id,
           (a[0] * a[8] - a[2] * a[6]) * id,
           (a[2] * a[3] - a[0] * a[5]) * id,
           c02 * id,
           (a[1] * a[6] - a[0] * a[7]) * id,
           (a[0] * a[4] - a[1] * a[3]) * id};
    return true;
}

/// Residual π(R·X + t) − x and, when `J` is given, its Jacobians: `Jc` (2×6,
/// w.r.t. [δθ, δt]) and `Jp` (2×3, w.r.t. δX). False if the point is not in
/// front of the camera.
template <math::Scalar T>
[[nodiscard]] bool reproject(const Mat3<T>& R,
                             const Vec3<T>& t,
                             const Vec3<T>& X,
                             const Vec2<T>& x,
                             Vec2<T>& r,
                             std::array<T, 12>* Jc = nullptr,
                             std::array<T, 6>* Jp = nullptr) {
    const Vec3<T> RX = R * X;
    const Vec3<T> c = RX + t;
    if (!(c[2] > T(1e-9)))
        return false;
    const T iz = T{1} / c[2];
    r = Vec2<T>{{c[0] * iz - x[0], c[1] * iz - x[1]}};
    if (Jc == nullptr)
        return true;
    // dπ/dc = [[1/z, 0, −x/z²], [0, 1/z, −y/z²]].
    const T a0 = iz, a2 = -c[0] * iz * iz, b1 = iz, b2 = -c[1] * iz * iz;
    const std::array<T, 6> P{a0, T{0}, a2, T{0}, b1, b2};
    // dc/dδθ = −[RX]×, dc/dδt = I, dc/dδX = R.
    const std::array<T, 9> Sk{T{0}, RX[2], -RX[1], -RX[2], T{0}, RX[0], RX[1], -RX[0], T{0}};  // −[RX]×
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 3; ++j) {
            T s = T{0}, u = T{0};
            for (std::size_t k = 0; k < 3; ++k) {
                s += P[i * 3 + k] * Sk[k * 3 + j];
                u += P[i * 3 + k] * R(k, j);
            }
            (*Jc)[i * 6 + j] = s;
            (*Jc)[i * 6 + 3 + j] = P[i * 3 + j];
            (*Jp)[i * 3 + j] = u;
        }
    return true;
}

/// Huber cost of a squared residual norm, and its IRLS weight.
template <math::Scalar T>
[[nodiscard]] T huber_cost(T sq, T k, T* w = nullptr) {
    if (!(k > T{0}) || sq <= k * k) {
        if (w)
            *w = T{1};
        return sq;
    }
    const T n = ld::sqrt_(sq);
    if (w)
        *w = k / n;
    return T{2} * k * n - k * k;
}

}  // namespace detail

/// Linear N-view triangulation of one point from world→camera poses (R, t) and
/// its normalized observations: the null vector of the stacked DLT rows
/// x·(r₃·X + t₃) − (r₁·X + t₁) and y·(r₃·X + t₃) − (r₂·X + t₂). False if
/// degenerate or behind any camera.
template <math::Scalar T>
[[nodiscard]] bool
triangulate_views(std::span<const Mat3<T>> R, std::span<const Vec3<T>> t, std::span<const Vec2<T>> x, Vec3<T>& X) {
    const std::size_t n = x.size();
    if (n < 2 || R.size() != n || t.size() != n)
        return false;
    ld::Mat<T, 4, 4> AtA{};
    for (std::size_t v = 0; v < n; ++v)
        for (std::size_t c = 0; c < 2; ++c) {
            std::array<T, 4> row{};
            for (std::size_t j = 0; j < 3; ++j)
                row[j] = x[v][c] * R[v](2, j) - R[v](c, j);
            row[3] = x[v][c] * t[v][2] - t[v][c];
            for (std::size_t i = 0; i < 4; ++i)
                for (std::size_t j = 0; j < 4; ++j)
                    AtA(i, j) += row[i] * row[j];
        }
    ld::Mat<T, 4, 4> V;
    ld::Vec<T, 4> w;
    jacobi_eigh<T, 4>(AtA, V, w);
    std::size_t jmin = 0;
    for (std::size_t j = 1; j < 4; ++j)
        if (w[j] < w[jmin])
            jmin = j;
    const T hw = V(3, jmin);
    if (hw == T{0})
        return false;
    X = Vec3<T>{{V(0, jmin) / hw, V(1, jmin) / hw, V(2, jmin) / hw}};
    for (std::size_t v = 0; v < n; ++v)
        if (!((R[v] * X + t[v])[2] > T{0}))
            return false;
    return true;
}

/// Refine world→camera poses (`R`, `t`; X_cam = R·X + t) and `points` against
/// `obs`. Frames with `fixed[f] != 0` are held; a point with fewer than two
/// observations in front of the cameras is held. Observations whose point is
/// behind the camera contribute nothing.
template <math::Scalar T>
BundleAdjustResult<T> bundle_adjust(std::vector<Mat3<T>>& R,
                                    std::vector<Vec3<T>>& t,
                                    std::vector<Vec3<T>>& points,
                                    std::span<const BaObservation<T>> obs,
                                    std::span<const char> fixed,
                                    const BundleAdjustOptions<T>& opt = {}) {
    using msckf::DynMat;
    BundleAdjustResult<T> out;
    const std::size_t nf = R.size(), np = points.size();
    if (t.size() != nf || fixed.size() != nf || obs.empty())
        return out;
    for (const auto& o : obs)
        if (o.frame >= nf || o.point >= np)
            return out;

    std::vector<std::size_t> col(nf, 0);  // free frame → its block, in 6-column units
    std::size_t m = 0;
    for (std::size_t f = 0; f < nf; ++f)
        if (!fixed[f])
            col[f] = m++;

    const auto cost = [&](const std::vector<Mat3<T>>& Rs,
                          const std::vector<Vec3<T>>& ts,
                          const std::vector<Vec3<T>>& Xs,
                          T* sq_sum,
                          std::size_t* used) {
        T c = T{0}, s = T{0};
        std::size_t u = 0;
        for (const auto& o : obs) {
            Vec2<T> r;
            if (!detail::reproject<T>(Rs[o.frame], ts[o.frame], Xs[o.point], o.x, r))
                continue;
            const T sq = r[0] * r[0] + r[1] * r[1];
            c += detail::huber_cost<T>(sq, opt.huber);
            s += sq;
            ++u;
        }
        if (sq_sum)
            *sq_sum = s;
        if (used)
            *used = u;
        return c;
    };
    T sq0 = T{0};
    std::size_t used0 = 0;
    T current = cost(R, t, points, &sq0, &used0);
    if (used0 == 0)
        return out;
    out.initial_rms = ld::sqrt_(sq0 / static_cast<T>(used0));
    out.final_rms = out.initial_rms;
    out.success = true;
    if (m == 0)
        return out;

    T lambda = opt.initial_lambda;
    for (std::size_t it = 0; it < opt.max_iterations; ++it) {
        // Normal equations at the current state, by blocks.
        DynMat<T> Hcc(6 * m, 6 * m);
        std::vector<T> gc(6 * m, T{0});
        std::vector<std::array<T, 9>> Hpp(np, std::array<T, 9>{});
        std::vector<Vec3<T>> gp(np, Vec3<T>{});
        std::vector<std::size_t> nobs(np, 0);
        struct Couple {
            std::size_t point, frame;
            std::array<T, 18> H;  // 6×3: J_cᵀ W J_p
        };
        std::vector<Couple> cp;
        cp.reserve(obs.size());
        for (const auto& o : obs) {
            Vec2<T> r;
            std::array<T, 12> Jc{};
            std::array<T, 6> Jp{};
            if (!detail::reproject<T>(R[o.frame], t[o.frame], points[o.point], o.x, r, &Jc, &Jp))
                continue;
            T w = T{1};
            (void)detail::huber_cost<T>(r[0] * r[0] + r[1] * r[1], opt.huber, &w);
            ++nobs[o.point];
            for (std::size_t i = 0; i < 3; ++i) {
                for (std::size_t j = 0; j < 3; ++j)
                    Hpp[o.point][i * 3 + j] += w * (Jp[i] * Jp[j] + Jp[3 + i] * Jp[3 + j]);
                gp[o.point][i] += w * (Jp[i] * r[0] + Jp[3 + i] * r[1]);
            }
            if (fixed[o.frame])
                continue;
            const std::size_t c0 = 6 * col[o.frame];
            for (std::size_t i = 0; i < 6; ++i) {
                for (std::size_t j = 0; j < 6; ++j)
                    Hcc(c0 + i, c0 + j) += w * (Jc[i] * Jc[j] + Jc[6 + i] * Jc[6 + j]);
                gc[c0 + i] += w * (Jc[i] * r[0] + Jc[6 + i] * r[1]);
            }
            Couple k{o.point, o.frame, {}};
            for (std::size_t i = 0; i < 6; ++i)
                for (std::size_t j = 0; j < 3; ++j)
                    k.H[i * 3 + j] = w * (Jc[i] * Jp[j] + Jc[6 + i] * Jp[3 + j]);
            cp.push_back(k);
        }
        // The couplings of each point, grouped.
        std::vector<std::vector<std::size_t>> by_point(np);
        for (std::size_t k = 0; k < cp.size(); ++k)
            by_point[cp[k].point].push_back(k);

        bool stepped = false;
        for (int attempt = 0; attempt < 10 && !stepped; ++attempt) {
            // Damped reduced camera system S δc = −b.
            DynMat<T> S = Hcc;
            DynMat<T> b(6 * m, 1);
            for (std::size_t i = 0; i < 6 * m; ++i) {
                S(i, i) += lambda * (Hcc(i, i) > T(1e-12) ? Hcc(i, i) : T(1e-12));
                b(i, 0) = gc[i];
            }
            std::vector<std::array<T, 9>> Hinv(np);
            std::vector<char> active(np, 0);
            for (std::size_t p = 0; p < np; ++p) {
                if (nobs[p] < 2)
                    continue;
                auto Hd = Hpp[p];
                for (std::size_t i = 0; i < 3; ++i)
                    Hd[i * 4] += lambda * (Hpp[p][i * 4] > T(1e-12) ? Hpp[p][i * 4] : T(1e-12));
                if (!detail::inverse3<T>(Hd, Hinv[p]))
                    continue;
                active[p] = 1;
                const auto& ks = by_point[p];
                // Hcpᵢ·Hpp⁻¹ for each coupling, then subtract its outer products.
                std::vector<std::array<T, 18>> W(ks.size());
                for (std::size_t a = 0; a < ks.size(); ++a)
                    for (std::size_t i = 0; i < 6; ++i)
                        for (std::size_t j = 0; j < 3; ++j) {
                            T s = T{0};
                            for (std::size_t k = 0; k < 3; ++k)
                                s += cp[ks[a]].H[i * 3 + k] * Hinv[p][k * 3 + j];
                            W[a][i * 3 + j] = s;
                        }
                for (std::size_t a = 0; a < ks.size(); ++a) {
                    const std::size_t ca = 6 * col[cp[ks[a]].frame];
                    for (std::size_t i = 0; i < 6; ++i) {
                        T s = T{0};
                        for (std::size_t k = 0; k < 3; ++k)
                            s += W[a][i * 3 + k] * gp[p][k];
                        b(ca + i, 0) -= s;
                    }
                    for (std::size_t c = 0; c < ks.size(); ++c) {
                        const std::size_t cb = 6 * col[cp[ks[c]].frame];
                        for (std::size_t i = 0; i < 6; ++i)
                            for (std::size_t j = 0; j < 6; ++j) {
                                T s = T{0};
                                for (std::size_t k = 0; k < 3; ++k)
                                    s += W[a][i * 3 + k] * cp[ks[c]].H[j * 3 + k];
                                S(ca + i, cb + j) -= s;
                            }
                    }
                }
            }
            DynMat<T> L;
            if (!msckf::cholesky(S, L)) {
                lambda *= T{10};
                continue;
            }
            DynMat<T> dc = msckf::cholesky_solve(L, b);  // solves S·dc = b; the step is −dc
            // Candidate state.
            auto R2 = R;
            auto t2 = t;
            auto X2 = points;
            for (std::size_t f = 0; f < nf; ++f) {
                if (fixed[f])
                    continue;
                const std::size_t c0 = 6 * col[f];
                const Vec3<T> dth{{-dc(c0, 0), -dc(c0 + 1, 0), -dc(c0 + 2, 0)}};
                R2[f] = math::lie::SO3<T>::exp(dth).matrix() * R[f];
                t2[f] = t[f] + Vec3<T>{{-dc(c0 + 3, 0), -dc(c0 + 4, 0), -dc(c0 + 5, 0)}};
            }
            for (std::size_t p = 0; p < np; ++p) {
                if (!active[p])
                    continue;
                // δX = −Hpp⁻¹ (gp + Σ Hcpᵀ δc), with δc = −dc.
                Vec3<T> rhs = gp[p];
                for (const std::size_t k : by_point[p]) {
                    const std::size_t c0 = 6 * col[cp[k].frame];
                    for (std::size_t j = 0; j < 3; ++j)
                        for (std::size_t i = 0; i < 6; ++i)
                            rhs[j] -= cp[k].H[i * 3 + j] * dc(c0 + i, 0);
                }
                for (std::size_t i = 0; i < 3; ++i) {
                    T s = T{0};
                    for (std::size_t j = 0; j < 3; ++j)
                        s += Hinv[p][i * 3 + j] * rhs[j];
                    X2[p][i] -= s;
                }
            }
            T sq = T{0};
            std::size_t used = 0;
            const T next = cost(R2, t2, X2, &sq, &used);
            if (used == used0 && next < current) {
                const T decrease = (current - next) / current;
                R = std::move(R2);
                t = std::move(t2);
                points = std::move(X2);
                current = next;
                out.final_rms = ld::sqrt_(sq / static_cast<T>(used));
                out.iterations = it + 1;
                lambda = lambda / T{10} > T(1e-9) ? lambda / T{10} : T(1e-9);
                stepped = true;
                if (decrease < opt.min_relative_decrease)
                    return out;
            } else {
                lambda *= T{10};
            }
        }
        if (!stepped)
            break;  // no damping lowered the cost: at a minimum to working precision
    }
    return out;
}

}  // namespace branes::sdk::sfm

#endif  // BRANES_SDK_SFM_BUNDLE_ADJUST_HPP
