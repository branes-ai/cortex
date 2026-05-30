// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/qr.hpp — a Householder QR for the square-root MSCKF
// covariance. Returns only the upper-triangular factor R (the economy
// min(m,n)×n block); the orthogonal Q is never formed. This is the one
// primitive every square-root covariance operation is built on:
// propagation, clone augmentation, marginalization, and the array
// measurement update all reduce to re-triangularizing a stacked factor.
//
// Clean-room from the textbook Householder QR (Golub & Van Loan, "Matrix
// Computations"; Kailath, Sayed & Hassibi, "Linear Estimation" §array
// algorithms). Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_QR_HPP
#define BRANES_SDK_MSCKF_QR_HPP

#include <branes/sdk/msckf/dense.hpp>

#include <cstddef>
#include <vector>

namespace branes::sdk::msckf {

/// Upper-triangular factor R of A (m×n) such that A = Q·R for some
/// orthonormal Q. Returns the *economy* R: its leading min(m,n) rows
/// (a square upper-triangular block when m ≥ n, upper-trapezoidal when
/// m < n). Q is not accumulated — the square-root recursions only need R.
///
/// The diagonal of R may carry either sign (the reflector sign is chosen
/// for numerical stability, not positivity); this is immaterial when R is
/// used as a covariance factor, since R·Rᵀ is sign-invariant per column.
template <math::Scalar T>
[[nodiscard]] DynMat<T> householder_qr_r(const DynMat<T>& A) {
    const std::size_t m = A.rows;
    const std::size_t n = A.cols;
    const std::size_t p = m < n ? m : n;
    DynMat<T> R = A;  // triangularized in place, then the top p rows are kept

    std::vector<T> v(m, T{0});
    for (std::size_t k = 0; k < p; ++k) {
        // Householder reflector that zeroes column k below the diagonal.
        T sigma = T{0};
        for (std::size_t i = k; i < m; ++i)
            sigma += R(i, k) * R(i, k);
        const T norm_x = math::lie::detail::sqrt_(sigma);
        if (!(norm_x > T{0}))
            continue;  // column already zero below the pivot
        // Reflect onto −sign(x_k)·‖x‖ to avoid cancellation in v_k.
        const T alpha = R(k, k) >= T{0} ? -norm_x : norm_x;
        v[k] = R(k, k) - alpha;
        for (std::size_t i = k + 1; i < m; ++i)
            v[i] = R(i, k);
        T vtv = T{0};
        for (std::size_t i = k; i < m; ++i)
            vtv += v[i] * v[i];
        if (!(vtv > T{0}))
            continue;

        // Apply H = I − 2·v·vᵀ/(vᵀv) to the trailing columns k..n−1.
        for (std::size_t j = k; j < n; ++j) {
            T dot = T{0};
            for (std::size_t i = k; i < m; ++i)
                dot += v[i] * R(i, j);
            const T beta = T{2} * dot / vtv;
            for (std::size_t i = k; i < m; ++i)
                R(i, j) -= beta * v[i];
        }
        R(k, k) = alpha;
        for (std::size_t i = k + 1; i < m; ++i)
            R(i, k) = T{0};
    }

    DynMat<T> Rtop(p, n);
    for (std::size_t i = 0; i < p; ++i)
        for (std::size_t j = 0; j < n; ++j)
            Rtop(i, j) = R(i, j);
    return Rtop;
}

/// Re-triangularize a covariance factor: given any B with B·Bᵀ = P,
/// return a (lower-triangular when B is tall, otherwise rank-minimal)
/// factor S with S·Sᵀ = P. Since B·Bᵀ = (Bᵀ)ᵀ·Bᵀ and QR gives
/// Bᵀ = Q·R, we have B·Bᵀ = Rᵀ·R, so S = Rᵀ.
template <math::Scalar T>
[[nodiscard]] DynMat<T> retriangularize(const DynMat<T>& B) {
    return transpose(householder_qr_r(transpose(B)));
}

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_QR_HPP
