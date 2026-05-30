// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/covariance.hpp — the covariance-representation policy for
// the MSCKF State. The estimator's mean, feature management, triangulation,
// and Jacobians are independent of *how* the error-state covariance is
// stored; only a handful of operations (clone augmentation, marginalization,
// prediction, the measurement update, and the Mahalanobis gate) touch the
// representation. Factoring those behind a policy lets the same backend run
// either the full-covariance form (this file, `FullCovariance`) or the
// square-root form (`sqrt_covariance.hpp`, `SqrtCovariance`).
//
// `FullCovariance` carries the dense covariance P directly and is the exact
// math previously inlined in StateHelper/Propagator — Joseph-form update,
// F P Fᵀ + Q propagation, principal-submatrix marginalization — relocated
// verbatim so the full-covariance path is behavior-identical.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_COVARIANCE_HPP
#define BRANES_SDK_MSCKF_COVARIANCE_HPP

#include <branes/sdk/msckf/dense.hpp>

#include <concepts>
#include <cstddef>
#include <span>
#include <utility>
#include <vector>

namespace branes::sdk::msckf {

/// One diagonal process-noise injection: (error-state index, variance).
template <math::Scalar T>
using NoiseTerm = std::pair<std::size_t, T>;

/// The operations a covariance representation must provide for the MSCKF
/// State. `transform`/`marginalize`/`predict` mutate the covariance under a
/// structural or dynamic map; `update` applies a measurement and returns the
/// error-state correction δx for the caller to box-plus onto the manifold
/// mean; `mahalanobis` computes the innovation-gated distance; `covariance`
/// materializes the dense P for inspection and PSD checks.
template <class C, class T>
concept CovarianceModel = math::Scalar<T> && requires(C c,
                                                      const C cc,
                                                      const DynMat<T>& M,
                                                      std::span<const T> r,
                                                      T scalar,
                                                      T& gamma,
                                                      std::span<const NoiseTerm<T>> q,
                                                      const std::vector<std::size_t>& keep) {
    { cc.dim() } -> std::convertible_to<std::size_t>;
    c.transform(M);
    c.marginalize(keep);
    c.predict(M, q);
    { c.update(M, r, r) } -> std::convertible_to<std::vector<T>>;
    { cc.mahalanobis(M, r, scalar, gamma) } -> std::convertible_to<bool>;
    { cc.covariance() } -> std::convertible_to<DynMat<T>>;
};

/// Full (dense) error-state covariance P. The reference representation.
template <math::Scalar T>
struct FullCovariance {
    DynMat<T> P;

    FullCovariance() = default;
    /// Initial IMU covariance P = sigma²·I over `dim` error states.
    FullCovariance(T sigma, std::size_t dim) : P(dim, dim) {
        for (std::size_t i = 0; i < dim; ++i)
            P(i, i) = sigma * sigma;
    }

    [[nodiscard]] std::size_t dim() const {
        return P.rows;
    }

    /// Linear covariance map P ← G P Gᵀ (G is d'×d). Clone augmentation
    /// uses G = [I; J]; any structural reshuffle is the same operation.
    void transform(const DynMat<T>& G) {
        DynMat<T> Pn = mul(mul(G, P), transpose(G));
        symmetrize(Pn);
        P = std::move(Pn);
    }

    /// Keep only the listed error-state indices (principal submatrix). A
    /// principal submatrix of an SPD matrix stays SPD.
    void marginalize(const std::vector<std::size_t>& keep) {
        DynMat<T> Pn(keep.size(), keep.size());
        for (std::size_t a = 0; a < keep.size(); ++a)
            for (std::size_t b = 0; b < keep.size(); ++b)
                Pn(a, b) = P(keep[a], keep[b]);
        P = std::move(Pn);
    }

    /// Predict P ← F P Fᵀ + Q with Q a diagonal injection.
    void predict(const DynMat<T>& F, std::span<const NoiseTerm<T>> q) {
        DynMat<T> Pn = mul(mul(F, P), transpose(F));
        for (const auto& [i, var] : q)
            Pn(i, i) += var;
        symmetrize(Pn);
        P = std::move(Pn);
    }

    /// Measurement update with Jacobian H (k×d), residual r (k), and
    /// diagonal noise `R_diag` (k). Joseph form keeps P symmetric positive
    /// definite; returns δx = K r for the caller to apply to the mean.
    [[nodiscard]] std::vector<T> update(const DynMat<T>& H, std::span<const T> r, std::span<const T> R_diag) {
        const std::size_t d = P.rows;
        const std::size_t k = H.rows;
        const DynMat<T> Ht = transpose(H);
        const DynMat<T> PHt = mul(P, Ht);  // d×k
        DynMat<T> S = mul(H, PHt);         // k×k
        for (std::size_t i = 0; i < k; ++i)
            S(i, i) += R_diag[i];

        const DynMat<T> Kt = spd_solve(S, transpose(PHt));  // k×d
        const DynMat<T> K = transpose(Kt);                  // d×k

        std::vector<T> dx(d, T{0});
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < k; ++j)
                dx[i] += K(i, j) * r[j];

        // Joseph form: P = (I−KH) P (I−KH)ᵀ + K diag(R) Kᵀ.
        const DynMat<T> KH = mul(K, H);  // d×d
        DynMat<T> ImKH = DynMat<T>::identity(d);
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < d; ++j)
                ImKH(i, j) -= KH(i, j);
        DynMat<T> P1 = mul(mul(ImKH, P), transpose(ImKH));
        DynMat<T> KR = K;  // K with columns scaled by R_diag
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < k; ++j)
                KR(i, j) *= R_diag[j];
        DynMat<T> Pn = add(P1, mul(KR, Kt));
        symmetrize(Pn);
        P = std::move(Pn);
        return dx;
    }

    /// Mahalanobis distance γ = rᵀ (H P Hᵀ + r2·I)⁻¹ r. Returns false (the
    /// caller should reject) if the innovation covariance is not positive
    /// definite.
    [[nodiscard]] bool mahalanobis(const DynMat<T>& H, std::span<const T> r, T r2, T& gamma) const {
        const std::size_t k = H.rows;
        const DynMat<T> Ht = transpose(H);
        const DynMat<T> PHt = mul(P, Ht);
        DynMat<T> S = mul(H, PHt);
        for (std::size_t i = 0; i < k; ++i)
            S(i, i) += r2;
        DynMat<T> rv(k, 1);
        for (std::size_t i = 0; i < k; ++i)
            rv(i, 0) = r[i];
        DynMat<T> L;
        if (!cholesky(S, L))
            return false;  // ill-conditioned innovation
        const DynMat<T> Sinv_r = cholesky_solve(L, rv);
        gamma = T{0};
        for (std::size_t i = 0; i < k; ++i)
            gamma += r[i] * Sinv_r(i, 0);
        return true;
    }

    /// Dense covariance (already dense here).
    [[nodiscard]] DynMat<T> covariance() const {
        return P;
    }
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_COVARIANCE_HPP
