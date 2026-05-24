// SPDX-License-Identifier: MIT
//
// branes/math/nls/dense_linalg.hpp — small dense linear-algebra kernels
// for the non-linear least-squares trust-region core.
//
// The NLS problems this layer targets (VIO/SLAM residual blocks, the
// per-iteration reduced systems) form modest dense normal equations
// JᵀJ Δ = −Jᵀr. These helpers operate on std::vector-backed row-major
// matrices and are type-generic over the scalar (double, float, Universal
// posit, ...). The sparse/iterative paths use MTL5 (see krylov.hpp /
// sparse_direct.hpp); this header is the self-contained dense core.
//
// Header-only, C++20.

#ifndef BRANES_MATH_NLS_DENSE_LINALG_HPP
#define BRANES_MATH_NLS_DENSE_LINALG_HPP

#include <branes/math/arithmetic.hpp>
#include <branes/math/lie/detail.hpp>  // scalar sqrt_/abs_ helpers

#include <cstddef>
#include <span>
#include <vector>

namespace branes::math::nls::detail {

using branes::math::lie::detail::abs_;
using branes::math::lie::detail::sqrt_;

template <Scalar T>
using Vec = std::vector<T>;

/// Dot product.
template <Scalar T>
[[nodiscard]] T dot(std::span<const T> a, std::span<const T> b) {
    T s{0};
    for (std::size_t i = 0; i < a.size(); ++i)
        s = s + a[i] * b[i];
    return s;
}

/// Euclidean norm.
template <Scalar T>
[[nodiscard]] T norm(std::span<const T> a) {
    return sqrt_(dot<T>(a, a));
}

/// Form the normal equations from a residual r (length m) and a row-major
/// Jacobian J (m×n): `JtJ` (n×n, row-major) = JᵀJ and `Jtr` (n) = Jᵀr.
template <Scalar T>
void normal_equations(
    std::span<const T> J, std::span<const T> r, std::size_t m, std::size_t n, Vec<T>& JtJ, Vec<T>& Jtr) {
    JtJ.assign(n * n, T{0});
    Jtr.assign(n, T{0});
    for (std::size_t k = 0; k < m; ++k) {
        const T rk = r[k];
        const std::size_t row = k * n;
        for (std::size_t i = 0; i < n; ++i) {
            const T Jki = J[row + i];
            Jtr[i] = Jtr[i] + Jki * rk;
            for (std::size_t j = 0; j < n; ++j) {
                JtJ[i * n + j] = JtJ[i * n + j] + Jki * J[row + j];
            }
        }
    }
}

/// y = A x for a row-major n×n matrix A.
template <Scalar T>
void matvec(std::span<const T> A, std::span<const T> x, std::size_t n, Vec<T>& y) {
    y.assign(n, T{0});
    for (std::size_t i = 0; i < n; ++i) {
        T s{0};
        for (std::size_t j = 0; j < n; ++j)
            s = s + A[i * n + j] * x[j];
        y[i] = s;
    }
}

/// Solve a symmetric positive-definite system A x = b (A row-major n×n)
/// by dense Cholesky. Returns false if a non-positive pivot is hit (A not
/// PD to working precision), leaving `x` unspecified.
template <Scalar T>
[[nodiscard]] bool cholesky_solve(Vec<T> A, std::size_t n, std::span<const T> b, Vec<T>& x) {
    Vec<T> L(n * n, T{0});
    auto Lat = [&](std::size_t r, std::size_t c) -> T& { return L[r * n + c]; };
    auto Aat = [&](std::size_t r, std::size_t c) -> T& { return A[r * n + c]; };
    for (std::size_t j = 0; j < n; ++j) {
        T sum = Aat(j, j);
        for (std::size_t k = 0; k < j; ++k)
            sum = sum - Lat(j, k) * Lat(j, k);
        if (!(sum > T{0}))
            return false;
        const T Ljj = sqrt_(sum);
        Lat(j, j) = Ljj;
        for (std::size_t i = j + 1; i < n; ++i) {
            T s = Aat(i, j);
            for (std::size_t k = 0; k < j; ++k)
                s = s - Lat(i, k) * Lat(j, k);
            Lat(i, j) = s / Ljj;
        }
    }
    // Forward solve L y = b (reuse x as scratch).
    x.assign(n, T{0});
    for (std::size_t i = 0; i < n; ++i) {
        T s = b[i];
        for (std::size_t k = 0; k < i; ++k)
            s = s - Lat(i, k) * x[k];
        x[i] = s / Lat(i, i);
    }
    // Back solve Lᵀ x = y.
    for (std::size_t ii = n; ii-- > 0;) {
        T s = x[ii];
        for (std::size_t k = ii + 1; k < n; ++k)
            s = s - Lat(k, ii) * x[k];
        x[ii] = s / Lat(ii, ii);
    }
    return true;
}

}  // namespace branes::math::nls::detail

#endif  // BRANES_MATH_NLS_DENSE_LINALG_HPP
