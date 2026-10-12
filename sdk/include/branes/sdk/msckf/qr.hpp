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

#include <mtl/operation/qr.hpp>
#include <mtl/tag/reflector_form.hpp>
#include <mtl/vec/dense_vector.hpp>

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
///
/// MTL5's `qr_factor` with the textbook reflector (#448, stillwater-sc/mtl5#559):
/// unscaled v = x − α·e₁ with α = −sign(x_k)·‖x‖, applied as
/// A(:,j) −= ((2·vᵀA(:,j))/vᵀv)·v, a column that is zero below the pivot (or
/// NaN) skipped, so R is the hand-written textbook QR's, bit for bit. MTL5
/// stores its packed reflectors below the diagonal; only R is kept.
template <math::Scalar T>
[[nodiscard]] DynMat<T> householder_qr_r(const DynMat<T>& A) {
    const std::size_t m = A.rows;
    const std::size_t n = A.cols;
    const std::size_t p = m < n ? m : n;
    DynMat<T> R = A;  // triangularized in place, then the top p rows are kept
    if (p > 0) {
        auto V = mtl_view::of(R);
        mtl::vec::dense_vector<T> tau;
        (void)mtl::qr_factor(V, tau, mtl::tag::textbook_reflector{});
    }

    DynMat<T> Rtop(p, n);
    for (std::size_t i = 0; i < p; ++i)
        for (std::size_t j = i; j < n; ++j)
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
