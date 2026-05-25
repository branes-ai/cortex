// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/dense.hpp — a minimal dynamic dense matrix for the
// MSCKF covariance bookkeeping (the IMU+clone error-state covariance and
// the EKF update grow/shrink at runtime, so the fixed-size math-layer
// matrices don't fit). Row-major, type-generic, header-only. Not a
// general BLAS — just what the State/Propagator/StateHelper need.

#ifndef BRANES_SDK_MSCKF_DENSE_HPP
#define BRANES_SDK_MSCKF_DENSE_HPP

#include <branes/math/lie/detail.hpp>  // scalar sqrt_

#include <cstddef>
#include <vector>

namespace branes::sdk::msckf {

template <math::Scalar T>
struct DynMat {
    std::size_t rows = 0;
    std::size_t cols = 0;
    std::vector<T> d;

    DynMat() = default;
    DynMat(std::size_t r, std::size_t c) : rows(r), cols(c), d(r * c, T{0}) {}

    [[nodiscard]] T& operator()(std::size_t i, std::size_t j) {
        return d[i * cols + j];
    }
    [[nodiscard]] const T& operator()(std::size_t i, std::size_t j) const {
        return d[i * cols + j];
    }

    [[nodiscard]] static DynMat identity(std::size_t n) {
        DynMat m(n, n);
        for (std::size_t i = 0; i < n; ++i)
            m(i, i) = T{1};
        return m;
    }
};

template <math::Scalar T>
[[nodiscard]] DynMat<T> mul(const DynMat<T>& a, const DynMat<T>& b) {
    DynMat<T> r(a.rows, b.cols);
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t k = 0; k < a.cols; ++k) {
            const T aik = a(i, k);
            if (aik == T{0})
                continue;
            for (std::size_t j = 0; j < b.cols; ++j)
                r(i, j) += aik * b(k, j);
        }
    return r;
}

template <math::Scalar T>
[[nodiscard]] DynMat<T> transpose(const DynMat<T>& a) {
    DynMat<T> r(a.cols, a.rows);
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t j = 0; j < a.cols; ++j)
            r(j, i) = a(i, j);
    return r;
}

template <math::Scalar T>
[[nodiscard]] DynMat<T> add(const DynMat<T>& a, const DynMat<T>& b) {
    DynMat<T> r(a.rows, a.cols);
    for (std::size_t i = 0; i < a.d.size(); ++i)
        r.d[i] = a.d[i] + b.d[i];
    return r;
}

/// Symmetrize in place: A ← (A + Aᵀ)/2. Counters drift that would
/// otherwise creep into the covariance over many updates.
template <math::Scalar T>
void symmetrize(DynMat<T>& a) {
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t j = i + 1; j < a.cols; ++j) {
            const T avg = (a(i, j) + a(j, i)) / T{2};
            a(i, j) = avg;
            a(j, i) = avg;
        }
}

/// Cholesky factor (lower L with A = L Lᵀ). Returns false if A is not
/// positive-definite to working precision.
template <math::Scalar T>
[[nodiscard]] bool cholesky(const DynMat<T>& a, DynMat<T>& L) {
    const std::size_t n = a.rows;
    L = DynMat<T>(n, n);
    for (std::size_t j = 0; j < n; ++j) {
        T sum = a(j, j);
        for (std::size_t k = 0; k < j; ++k)
            sum -= L(j, k) * L(j, k);
        if (!(sum > T{0}))
            return false;
        const T Ljj = math::lie::detail::sqrt_(sum);
        L(j, j) = Ljj;
        for (std::size_t i = j + 1; i < n; ++i) {
            T s = a(i, j);
            for (std::size_t k = 0; k < j; ++k)
                s -= L(i, k) * L(j, k);
            L(i, j) = s / Ljj;
        }
    }
    return true;
}

/// True iff `a` is symmetric positive-definite to working precision.
template <math::Scalar T>
[[nodiscard]] bool is_positive_definite(const DynMat<T>& a) {
    DynMat<T> L;
    return cholesky(a, L);
}

/// True iff `a` is symmetric positive-*semi*-definite (no negative
/// eigenvalues) to within a small ridge. MSCKF clone augmentation makes
/// the covariance singular at that instant (the clone is an exact copy of
/// the IMU pose), so PSD — not strict PD — is the invariant there.
template <math::Scalar T>
[[nodiscard]] bool is_positive_semidefinite(const DynMat<T>& a, T ridge = T(1e-9)) {
    DynMat<T> b = a;
    for (std::size_t i = 0; i < b.rows; ++i)
        b(i, i) += ridge;
    DynMat<T> L;
    return cholesky(b, L);
}

/// Solve the SPD system A X = B (A n×n, B n×k) via the Cholesky factor.
/// Precondition: A is positive-definite.
template <math::Scalar T>
[[nodiscard]] DynMat<T> spd_solve(const DynMat<T>& a, const DynMat<T>& b) {
    DynMat<T> L;
    if (!cholesky(a, L))
        return DynMat<T>(a.rows, b.cols);  // not PD
    const std::size_t n = a.rows;
    const std::size_t k = b.cols;
    DynMat<T> x(n, k);
    // Forward (L y = b) then back (Lᵀ x = y), per RHS column.
    for (std::size_t c = 0; c < k; ++c) {
        for (std::size_t i = 0; i < n; ++i) {
            T s = b(i, c);
            for (std::size_t j = 0; j < i; ++j)
                s -= L(i, j) * x(j, c);
            x(i, c) = s / L(i, i);
        }
        for (std::size_t ii = n; ii-- > 0;) {
            T s = x(ii, c);
            for (std::size_t j = ii + 1; j < n; ++j)
                s -= L(j, ii) * x(j, c);
            x(ii, c) = s / L(ii, ii);
        }
    }
    return x;
}

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_DENSE_HPP
