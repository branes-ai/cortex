// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/sqrt_covariance.hpp — the square-root covariance policy
// for the MSCKF State. Carries the Cholesky factor S (P = S·Sᵀ) instead of
// P, so the covariance is represented with half the dynamic range and stays
// positive semidefinite by construction. Every operation re-triangularizes a
// stacked factor with one Householder QR (qr.hpp); the measurement step is
// the array update, which produces the innovation factor, Kalman gain, and
// updated factor in a single triangularization.
//
// Clean-room from the textbook square-root / array Kalman algorithms
// (Kailath, Sayed & Hassibi, "Linear Estimation", §array algorithms; Maybeck,
// "Stochastic Models, Estimation and Control" §square-root filtering). Mirrors
// the FullCovariance interface so it is a drop-in policy.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_SQRT_COVARIANCE_HPP
#define BRANES_SDK_MSCKF_SQRT_COVARIANCE_HPP

#include <branes/math/lie/detail.hpp>  // detail::sqrt_
#include <branes/sdk/msckf/covariance.hpp>
#include <branes/sdk/msckf/qr.hpp>

#include <mtl/mat/view/map_view.hpp>
#include <mtl/operation/mult.hpp>
#include <mtl/operation/trsm.hpp>

#include <cstddef>
#include <numeric>
#include <span>
#include <utility>
#include <vector>

namespace branes::sdk::msckf {

namespace sqrt_detail {
/// The block of `m` at rows [r0, r0 + nr) and columns [c0, c0 + nc), as an MTL5
/// view (no copy). `m` must outlive the view.
template <math::Scalar T>
[[nodiscard]] mtl::mat::view::map_view<mtl_view::Dense<T>> block(const mtl_view::Dense<T>& m, std::size_t r0,
                                                                  std::size_t nr, std::size_t c0, std::size_t nc) {
    using Index = typename mtl_view::Dense<T>::size_type;
    std::vector<Index> rows(nr), cols(nc);
    std::iota(rows.begin(), rows.end(), static_cast<Index>(r0));
    std::iota(cols.begin(), cols.end(), static_cast<Index>(c0));
    return {m, rows, cols};
}

/// Forward-solve L y = b with L the leading n×n lower triangle of `l`, up to its
/// first zero pivot. Returns the number of rows solved: n, or the index of that
/// pivot, past which y keeps b. MTL5's trsm subtracts L(i,0)·y(0), L(i,1)·y(1), …
/// from b(i) in turn and divides by the pivot, as the hand-rolled substitution
/// did, so y is the same to the bit.
template <math::Scalar T>
std::size_t forward_solve(const DynMat<T>& l, std::size_t n, DynMat<T>& y) {
    std::size_t m = 0;
    while (m < n && !(l(m, m) == T{0}))
        ++m;
    if (m == 0)
        return 0;
    const auto L = mtl_view::of(l);
    DynMat<T> ym(m, 1);
    for (std::size_t i = 0; i < m; ++i)
        ym(i, 0) = y(i, 0);
    auto Y = mtl_view::of(ym);
    mtl::trsm(T{1}, block(L, 0, m, 0, m), Y, /*upper=*/false);
    for (std::size_t i = 0; i < m; ++i)
        y(i, 0) = ym(i, 0);
    return m;
}
}  // namespace sqrt_detail

/// Square-root error-state covariance: stores S with P = S·Sᵀ. S is square
/// lower-triangular when full rank; clone augmentation transiently produces a
/// rank-minimal rectangular factor, which the next prediction re-squares.
template <math::Scalar T>
struct SqrtCovariance {
    DynMat<T> S;  ///< covariance factor, P = S·Sᵀ

    SqrtCovariance() = default;
    /// Initial IMU covariance P = sigma²·I ⇒ S = sigma·I over `dim` states.
    SqrtCovariance(T sigma, std::size_t dim) : S(dim, dim) {
        for (std::size_t i = 0; i < dim; ++i)
            S(i, i) = sigma;
    }

    [[nodiscard]] std::size_t dim() const {
        return S.rows;
    }

    /// P ← G P Gᵀ = (G S)(G S)ᵀ, re-triangularized.
    void transform(const DynMat<T>& G) {
        S = retriangularize(mul(G, S));
    }

    /// Keep only the listed error-state rows of S (the kept-index principal
    /// submatrix of P = S Sᵀ is (S_keep)(S_keep)ᵀ), then re-triangularize.
    void marginalize(const std::vector<std::size_t>& keep) {
        DynMat<T> Sk(keep.size(), S.cols);
        for (std::size_t a = 0; a < keep.size(); ++a)
            for (std::size_t j = 0; j < S.cols; ++j)
                Sk(a, j) = S(keep[a], j);
        S = retriangularize(Sk);
    }

    /// Predict P ← F P Fᵀ + Q = [F S | √Q][F S | √Q]ᵀ, re-triangularized.
    void predict(const DynMat<T>& F, std::span<const NoiseTerm<T>> q) {
        const DynMat<T> FS = mul(F, S);
        const std::size_t d = FS.rows;
        const std::size_t nq = q.size();
        DynMat<T> B(d, FS.cols + nq);
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < FS.cols; ++j)
                B(i, j) = FS(i, j);
        for (std::size_t c = 0; c < nq; ++c)
            B(q[c].first, FS.cols + c) = math::lie::detail::sqrt_(q[c].second);
        S = retriangularize(B);
    }

    /// Array measurement update. Triangularizing the pre-array
    ///   [ √R   H S ]            [ Re    0  ]
    ///   [  0    S  ]  ──QR──▶   [ Kbar  S' ]
    /// gives Re·Reᵀ = H P Hᵀ + R (innovation factor), S'·S'ᵀ = P⁺ (updated
    /// factor), and gain K = Kbar·Re⁻¹. The correction is δx = Kbar·(Re⁻¹ r).
    [[nodiscard]] std::vector<T> update(const DynMat<T>& H, std::span<const T> r, std::span<const T> R_diag) {
        const std::size_t d = S.rows;
        const std::size_t k = H.rows;
        const DynMat<T> HS = mul(H, S);  // k×d

        DynMat<T> pre(k + d, k + d);
        for (std::size_t i = 0; i < k; ++i) {
            pre(i, i) = math::lie::detail::sqrt_(R_diag[i]);  // √R (diagonal)
            for (std::size_t j = 0; j < d; ++j)
                pre(i, k + j) = HS(i, j);  // H S
        }
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < d; ++j)
                pre(k + i, k + j) = S(i, j);  // S

        // LQ(pre) = (QR(preᵀ))ᵀ → block-lower-triangular post.
        const DynMat<T> post = retriangularize(pre);

        // δx = Kbar·(Re⁻¹ r): forward-solve Re y = r (Re = post[0:k,0:k],
        // lower-triangular). A zero pivot means the innovation is singular in
        // that direction — the same condition mahalanobis() rejects on — so
        // skip the update entirely (leave S and the mean unchanged) rather
        // than apply a δx from an unobservable innovation. This mirrors the
        // full-covariance path, whose spd_solve no-ops on a non-PD innovation.
        // With gating enabled (the default) the gate already rejects this
        // before update() runs; the guard covers the gating-disabled path.
        DynMat<T> y(k, 1);
        for (std::size_t i = 0; i < k; ++i)
            y(i, 0) = r[i];
        if (sqrt_detail::forward_solve(post, k, y) < k)
            return std::vector<T>(d, T{0});  // singular innovation ⇒ no-op

        // Commit the updated factor S' = post[k:, k:] and form the correction
        // δx = Kbar y (Kbar = post[k:, 0:k]) now that the solve has succeeded.
        DynMat<T> Sn(d, d);
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < d; ++j)
                Sn(i, j) = post(k + i, k + j);
        // Each δx(i) is 0 + Kbar(i,0)·y(0) + Kbar(i,1)·y(1) + … in ascending j,
        // MTL5's generic product, which is the order of the loop it replaced.
        DynMat<T> dx(d, 1);
        if (d > 0 && k > 0) {
            auto DX = mtl_view::of(dx);
            mtl::mult(sqrt_detail::block(mtl_view::of(post), k, d, 0, k), mtl_view::of(y), DX);
        }
        S = std::move(Sn);
        return std::move(dx.d);
    }

    /// Mahalanobis distance γ = rᵀ (H P Hᵀ + r2·I)⁻¹ r = ‖Re⁻¹ r‖², with
    /// Re the lower-triangular factor of the innovation covariance. Returns
    /// false (reject) if Re is singular.
    [[nodiscard]] bool mahalanobis(const DynMat<T>& H, std::span<const T> r, T r2, T& gamma) const {
        const std::size_t k = H.rows;
        const DynMat<T> HS = mul(H, S);  // k×d
        DynMat<T> B(k, HS.cols + k);
        for (std::size_t i = 0; i < k; ++i) {
            for (std::size_t j = 0; j < HS.cols; ++j)
                B(i, j) = HS(i, j);
            B(i, HS.cols + i) = math::lie::detail::sqrt_(r2);
        }
        const DynMat<T> Re = retriangularize(B);  // k×k lower-triangular

        // On a zero pivot γ holds the rows solved before it (NIS records it).
        DynMat<T> y(k, 1);
        for (std::size_t i = 0; i < k; ++i)
            y(i, 0) = r[i];
        const std::size_t solved = sqrt_detail::forward_solve(Re, k, y);
        gamma = T{0};
        for (std::size_t i = 0; i < solved; ++i)
            gamma += y(i, 0) * y(i, 0);
        return solved == k;  // a zero pivot is a singular innovation ⇒ reject
    }

    /// Dense covariance P = S·Sᵀ (for inspection and PSD checks).
    [[nodiscard]] DynMat<T> covariance() const {
        DynMat<T> P = mul(S, transpose(S));
        symmetrize(P);
        return P;
    }
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_SQRT_COVARIANCE_HPP
