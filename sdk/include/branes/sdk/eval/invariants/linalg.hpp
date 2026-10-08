// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/invariants/linalg.hpp — the small, accuracy-first dense
// kernels the invariant checks measure with: norms, products, the symmetric
// eigenvalues (cyclic Jacobi), the singular values (one-sided Jacobi), and an
// orthonormal column basis (modified Gram–Schmidt with re-orthogonalization).
//
// Jacobi methods are chosen over QR-iteration because they are short, need no
// shifts or deflation logic, and compute eigen/singular values to high RELATIVE
// accuracy — the property an oracle needs, since the definiteness and rank
// verdicts hinge on the smallest values. They are O(n³) per sweep, which is fine
// for bench-sized matrices (the MSCKF covariance is ≲ 150×150).
//
// Clean-room from Golub & Van Loan, "Matrix Computations", §8.5 (symmetric
// Jacobi) and §8.6.3 (one-sided Jacobi SVD). Computes in T, so the measurement
// carries the arithmetic of the type under test. Header-only, C++20.

#ifndef BRANES_SDK_EVAL_INVARIANTS_LINALG_HPP
#define BRANES_SDK_EVAL_INVARIANTS_LINALG_HPP

#include <branes/math/lie/detail.hpp>  // sqrt_, abs_
#include <branes/sdk/eval/invariants/core.hpp>
#include <branes/sdk/msckf/dense.hpp>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace branes::sdk::eval::inv::la {

using math::lie::detail::abs_;
using math::lie::detail::sqrt_;
using msckf::DynMat;

template <math::Scalar T>
[[nodiscard]] T max_abs(const DynMat<T>& a) {
    T m{0};
    for (const T& x : a.d)
        m = std::max(m, abs_(x));
    return m;
}

template <math::Scalar T>
[[nodiscard]] T frobenius(const DynMat<T>& a) {
    T s{0};
    for (const T& x : a.d)
        s += x * x;
    return sqrt_(s);
}

template <math::Scalar T>
[[nodiscard]] DynMat<T> sub(const DynMat<T>& a, const DynMat<T>& b) {
    DynMat<T> r(a.rows, a.cols);
    for (std::size_t i = 0; i < a.d.size(); ++i)
        r.d[i] = a.d[i] - b.d[i];
    return r;
}

template <math::Scalar T>
[[nodiscard]] bool same_shape(const DynMat<T>& a, const DynMat<T>& b) noexcept {
    return a.rows == b.rows && a.cols == b.cols;
}

/// A ← (A + Aᵀ)/2, returned as a copy (the eigen solver needs exact symmetry).
template <math::Scalar T>
[[nodiscard]] DynMat<T> symmetrized(const DynMat<T>& a) {
    DynMat<T> s = a;
    msckf::symmetrize(s);
    return s;
}

/// Eigenvalues of a symmetric matrix, ascending. Cyclic Jacobi: sweep every
/// (p, q) pair with the rotation that zeroes a_pq until the off-diagonal mass is
/// below ε_T·‖A‖_F. The input is symmetrized first.
template <math::Scalar T>
[[nodiscard]] std::vector<T> symmetric_eigenvalues(const DynMat<T>& a_in, int max_sweeps = 60) {
    DynMat<T> a = symmetrized(a_in);
    const std::size_t n = a.rows;
    const T tol = epsilon<T>() * frobenius(a);
    for (int sweep = 0; sweep < max_sweeps; ++sweep) {
        T off{0};
        for (std::size_t p = 0; p < n; ++p)
            for (std::size_t q = p + 1; q < n; ++q)
                off += a(p, q) * a(p, q);
        if (!(sqrt_(off) > tol))
            break;
        for (std::size_t p = 0; p < n; ++p)
            for (std::size_t q = p + 1; q < n; ++q) {
                const T apq = a(p, q);
                if (apq == T{0})
                    continue;
                const T theta = (a(q, q) - a(p, p)) / (T{2} * apq);
                const T sgn = theta >= T{0} ? T{1} : T{-1};
                const T t = sgn / (abs_(theta) + sqrt_(T{1} + theta * theta));
                const T c = T{1} / sqrt_(T{1} + t * t);
                const T s = t * c;
                // A ← Jᵀ A J with J = [[c, s], [−s, c]] in the (p, q) plane.
                for (std::size_t k = 0; k < n; ++k) {
                    const T akp = a(k, p), akq = a(k, q);
                    a(k, p) = c * akp - s * akq;
                    a(k, q) = s * akp + c * akq;
                }
                for (std::size_t k = 0; k < n; ++k) {
                    const T apk = a(p, k), aqk = a(q, k);
                    a(p, k) = c * apk - s * aqk;
                    a(q, k) = s * apk + c * aqk;
                }
            }
    }
    std::vector<T> ev(n);
    for (std::size_t i = 0; i < n; ++i)
        ev[i] = a(i, i);
    std::sort(ev.begin(), ev.end());
    return ev;
}

/// Singular values of A (m×n), descending. One-sided (Hestenes) Jacobi on the
/// columns of A — or of Aᵀ when m < n, so the working matrix is tall.
template <math::Scalar T>
[[nodiscard]] std::vector<T> singular_values(const DynMat<T>& a_in, int max_sweeps = 60) {
    DynMat<T> u = a_in.rows >= a_in.cols ? a_in : msckf::transpose(a_in);
    const std::size_t m = u.rows, n = u.cols;
    const T eps = epsilon<T>();
    for (int sweep = 0; sweep < max_sweeps; ++sweep) {
        bool rotated = false;
        for (std::size_t p = 0; p < n; ++p)
            for (std::size_t q = p + 1; q < n; ++q) {
                T alpha{0}, beta{0}, gamma{0};
                for (std::size_t i = 0; i < m; ++i) {
                    alpha += u(i, p) * u(i, p);
                    beta += u(i, q) * u(i, q);
                    gamma += u(i, p) * u(i, q);
                }
                if (!(abs_(gamma) > eps * sqrt_(alpha * beta)))
                    continue;
                rotated = true;
                const T zeta = (beta - alpha) / (T{2} * gamma);
                const T sgn = zeta >= T{0} ? T{1} : T{-1};
                const T t = sgn / (abs_(zeta) + sqrt_(T{1} + zeta * zeta));
                const T c = T{1} / sqrt_(T{1} + t * t);
                const T s = c * t;
                for (std::size_t i = 0; i < m; ++i) {
                    const T up = u(i, p), uq = u(i, q);
                    u(i, p) = c * up - s * uq;
                    u(i, q) = s * up + c * uq;
                }
            }
        if (!rotated)
            break;
    }
    std::vector<T> sv(n);
    for (std::size_t j = 0; j < n; ++j) {
        T s{0};
        for (std::size_t i = 0; i < m; ++i)
            s += u(i, j) * u(i, j);
        sv[j] = sqrt_(s);
    }
    std::sort(sv.begin(), sv.end(), [](const T& x, const T& y) { return x > y; });
    return sv;
}

/// Numerical rank: the count of singular values above n·ε_T·σ_max·safety.
template <math::Scalar T>
[[nodiscard]] std::size_t numerical_rank(const DynMat<T>& a, double safety = kDefaultSafety) {
    const auto sv = singular_values(a);
    if (sv.empty() || !(sv.front() > T{0}))
        return 0;
    const std::size_t big = std::max(a.rows, a.cols);
    const T cut = T(safety) * T(static_cast<double>(big)) * epsilon<T>() * sv.front();
    std::size_t r = 0;
    for (const T& s : sv)
        r += s > cut ? 1 : 0;
    return r;
}

/// Orthonormal basis (n×k', k' ≤ k) of the column span of N (n×k). Modified
/// Gram–Schmidt applied twice ("twice is enough"); columns that vanish relative
/// to the largest input column are dropped as linearly dependent.
template <math::Scalar T>
[[nodiscard]] DynMat<T> orthonormal_columns(const DynMat<T>& N, double safety = kDefaultSafety) {
    const std::size_t n = N.rows, k = N.cols;
    T scale{0};
    for (std::size_t j = 0; j < k; ++j) {
        T s{0};
        for (std::size_t i = 0; i < n; ++i)
            s += N(i, j) * N(i, j);
        scale = std::max(scale, sqrt_(s));
    }
    const T drop = T(safety) * T(static_cast<double>(n)) * epsilon<T>() * scale;
    std::vector<std::vector<T>> basis;
    for (std::size_t j = 0; j < k; ++j) {
        std::vector<T> v(n);
        for (std::size_t i = 0; i < n; ++i)
            v[i] = N(i, j);
        for (int pass = 0; pass < 2; ++pass)
            for (const auto& q : basis) {
                T d{0};
                for (std::size_t i = 0; i < n; ++i)
                    d += q[i] * v[i];
                for (std::size_t i = 0; i < n; ++i)
                    v[i] -= d * q[i];
            }
        T nv{0};
        for (std::size_t i = 0; i < n; ++i)
            nv += v[i] * v[i];
        nv = sqrt_(nv);
        if (!(nv > drop))
            continue;
        for (std::size_t i = 0; i < n; ++i)
            v[i] /= nv;
        basis.push_back(std::move(v));
    }
    DynMat<T> Q(n, basis.size());
    for (std::size_t j = 0; j < basis.size(); ++j)
        for (std::size_t i = 0; i < n; ++i)
            Q(i, j) = basis[j][i];
    return Q;
}

}  // namespace branes::sdk::eval::inv::la

#endif  // BRANES_SDK_EVAL_INVARIANTS_LINALG_HPP
