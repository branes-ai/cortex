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

#include <branes/sdk/msckf/covariance.hpp>
#include <branes/sdk/msckf/qr.hpp>

#include <cstddef>
#include <span>
#include <utility>
#include <vector>

namespace branes::sdk::msckf {

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

        // S' = post[k:, k:].
        DynMat<T> Sn(d, d);
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < d; ++j)
                Sn(i, j) = post(k + i, k + j);

        // δx = Kbar·(Re⁻¹ r): forward-solve Re y = r (Re = post[0:k,0:k]
        // lower-triangular), then δx = Kbar y (Kbar = post[k:, 0:k]).
        std::vector<T> y(k, T{0});
        for (std::size_t i = 0; i < k; ++i) {
            T s = r[i];
            for (std::size_t j = 0; j < i; ++j)
                s -= post(i, j) * y[j];
            const T diag = post(i, i);
            y[i] = diag != T{0} ? s / diag : T{0};
        }
        std::vector<T> dx(d, T{0});
        for (std::size_t i = 0; i < d; ++i) {
            T acc = T{0};
            for (std::size_t j = 0; j < k; ++j)
                acc += post(k + i, j) * y[j];
            dx[i] = acc;
        }

        S = std::move(Sn);
        return dx;
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

        gamma = T{0};
        std::vector<T> y(k, T{0});
        for (std::size_t i = 0; i < k; ++i) {
            T s = r[i];
            for (std::size_t j = 0; j < i; ++j)
                s -= Re(i, j) * y[j];
            const T diag = Re(i, i);
            if (diag == T{0})
                return false;  // singular innovation ⇒ reject
            y[i] = s / diag;
            gamma += y[i] * y[i];
        }
        return true;
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
