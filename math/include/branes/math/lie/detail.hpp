// SPDX-License-Identifier: MIT
//
// branes/math/lie/detail.hpp — minimal fixed-size linear algebra for the
// Lie-group implementations (SO3, SE3, Sim3).
//
// These groups operate on 3-vectors, 3x3 / 4x4 matrices, quaternions,
// and 6x6 / 7x7 adjoints. Rather than pull a dynamic matrix library into
// these tiny, compile-time-sized operations, we use std::array-backed
// fixed types with constexpr-friendly free functions. Everything is
// type-generic over the scalar (double, float, Universal posit, ...).
//
// Scalar transcendentals are reached through ADL so the same code binds
// std::sqrt/sin/... for built-in floats and sw::universal::sqrt/sin/...
// for the Universal number types (callers include the relevant Universal
// mathlib header; this file only needs <cmath> for the built-ins).
//
// Header-only, C++20.

#ifndef BRANES_MATH_LIE_DETAIL_HPP
#define BRANES_MATH_LIE_DETAIL_HPP

#include <branes/math/arithmetic.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

namespace branes::math::lie::detail {

// ── Scalar helpers (ADL dispatch to std:: or sw::universal::) ────────

template <Scalar T>
[[nodiscard]] T sqrt_(const T& x) {
    using std::sqrt;
    return sqrt(x);
}
template <Scalar T>
[[nodiscard]] T sin_(const T& x) {
    using std::sin;
    return sin(x);
}
template <Scalar T>
[[nodiscard]] T cos_(const T& x) {
    using std::cos;
    return cos(x);
}
template <Scalar T>
[[nodiscard]] T atan2_(const T& y, const T& x) {
    using std::atan2;
    return atan2(y, x);
}
template <Scalar T>
[[nodiscard]] T abs_(const T& x) {
    using std::abs;
    return abs(x);
}
template <Scalar T>
[[nodiscard]] T exp_(const T& x) {
    using std::exp;
    return exp(x);
}
template <Scalar T>
[[nodiscard]] T log_(const T& x) {
    using std::log;
    return log(x);
}

// ── Fixed-size column vector ─────────────────────────────────────────

template <Scalar T, std::size_t N>
struct Vec {
    std::array<T, N> e{};

    [[nodiscard]] constexpr T& operator[](std::size_t i) noexcept {
        return e[i];
    }
    [[nodiscard]] constexpr const T& operator[](std::size_t i) const noexcept {
        return e[i];
    }
    static constexpr std::size_t size() noexcept {
        return N;
    }

    [[nodiscard]] constexpr Vec operator+(const Vec& o) const noexcept {
        Vec r;
        for (std::size_t i = 0; i < N; ++i)
            r[i] = e[i] + o[i];
        return r;
    }
    [[nodiscard]] constexpr Vec operator-(const Vec& o) const noexcept {
        Vec r;
        for (std::size_t i = 0; i < N; ++i)
            r[i] = e[i] - o[i];
        return r;
    }
    [[nodiscard]] constexpr Vec operator-() const noexcept {
        Vec r;
        for (std::size_t i = 0; i < N; ++i)
            r[i] = -e[i];
        return r;
    }
    [[nodiscard]] constexpr Vec operator*(const T& s) const noexcept {
        Vec r;
        for (std::size_t i = 0; i < N; ++i)
            r[i] = e[i] * s;
        return r;
    }
};

template <Scalar T, std::size_t N>
[[nodiscard]] constexpr T dot(const Vec<T, N>& a, const Vec<T, N>& b) noexcept {
    T s{0};
    for (std::size_t i = 0; i < N; ++i)
        s = s + a[i] * b[i];
    return s;
}

template <Scalar T, std::size_t N>
[[nodiscard]] T norm(const Vec<T, N>& a) {
    return sqrt_(dot(a, a));
}

template <Scalar T>
[[nodiscard]] constexpr Vec<T, 3> cross(const Vec<T, 3>& a, const Vec<T, 3>& b) noexcept {
    return Vec<T, 3>{{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}};
}

// ── Fixed-size row-major matrix ──────────────────────────────────────

template <Scalar T, std::size_t R, std::size_t C>
struct Mat {
    std::array<T, R * C> e{};

    [[nodiscard]] constexpr T& operator()(std::size_t i, std::size_t j) noexcept {
        return e[i * C + j];
    }
    [[nodiscard]] constexpr const T& operator()(std::size_t i, std::size_t j) const noexcept {
        return e[i * C + j];
    }
    static constexpr std::size_t rows() noexcept {
        return R;
    }
    static constexpr std::size_t cols() noexcept {
        return C;
    }

    [[nodiscard]] static constexpr Mat zero() noexcept {
        return Mat{};
    }

    [[nodiscard]] static constexpr Mat identity() noexcept {
        Mat m{};
        for (std::size_t i = 0; i < (R < C ? R : C); ++i)
            m(i, i) = T{1};
        return m;
    }

    [[nodiscard]] constexpr Mat operator+(const Mat& o) const noexcept {
        Mat r;
        for (std::size_t i = 0; i < R * C; ++i)
            r.e[i] = e[i] + o.e[i];
        return r;
    }
    [[nodiscard]] constexpr Mat operator-(const Mat& o) const noexcept {
        Mat r;
        for (std::size_t i = 0; i < R * C; ++i)
            r.e[i] = e[i] - o.e[i];
        return r;
    }
    [[nodiscard]] constexpr Mat operator*(const T& s) const noexcept {
        Mat r;
        for (std::size_t i = 0; i < R * C; ++i)
            r.e[i] = e[i] * s;
        return r;
    }
};

template <Scalar T, std::size_t R, std::size_t K, std::size_t C>
[[nodiscard]] constexpr Mat<T, R, C> operator*(const Mat<T, R, K>& a, const Mat<T, K, C>& b) noexcept {
    Mat<T, R, C> r{};
    for (std::size_t i = 0; i < R; ++i) {
        for (std::size_t j = 0; j < C; ++j) {
            T s{0};
            for (std::size_t k = 0; k < K; ++k)
                s = s + a(i, k) * b(k, j);
            r(i, j) = s;
        }
    }
    return r;
}

template <Scalar T, std::size_t R, std::size_t C>
[[nodiscard]] constexpr Vec<T, R> operator*(const Mat<T, R, C>& a, const Vec<T, C>& v) noexcept {
    Vec<T, R> r{};
    for (std::size_t i = 0; i < R; ++i) {
        T s{0};
        for (std::size_t j = 0; j < C; ++j)
            s = s + a(i, j) * v[j];
        r[i] = s;
    }
    return r;
}

template <Scalar T, std::size_t R, std::size_t C>
[[nodiscard]] constexpr Mat<T, C, R> transpose(const Mat<T, R, C>& a) noexcept {
    Mat<T, C, R> r{};
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < C; ++j)
            r(j, i) = a(i, j);
    return r;
}

/// Skew-symmetric "hat" of a 3-vector: hat(w) * v == cross(w, v).
template <Scalar T>
[[nodiscard]] constexpr Mat<T, 3, 3> hat(const Vec<T, 3>& w) noexcept {
    Mat<T, 3, 3> m{};
    m(0, 1) = -w[2];
    m(0, 2) = w[1];
    m(1, 0) = w[2];
    m(1, 2) = -w[0];
    m(2, 0) = -w[1];
    m(2, 1) = w[0];
    return m;
}

/// Inverse of hat: extract the 3-vector from a skew-symmetric matrix.
template <Scalar T>
[[nodiscard]] constexpr Vec<T, 3> vee(const Mat<T, 3, 3>& m) noexcept {
    return Vec<T, 3>{{m(2, 1), m(0, 2), m(1, 0)}};
}

/// Dense inverse of a small square matrix via Gauss–Jordan elimination
/// with partial pivoting. Used for the modest fixed sizes (3x3 W block,
/// 7x7 Sim3 Jacobian) where a generic routine is clearer than per-size
/// closed forms. Assumes the matrix is invertible.
template <Scalar T, std::size_t N>
[[nodiscard]] Mat<T, N, N> inverse(const Mat<T, N, N>& a) {
    Mat<T, N, N> m = a;
    Mat<T, N, N> inv = Mat<T, N, N>::identity();
    for (std::size_t col = 0; col < N; ++col) {
        // Partial pivot: pick the row with the largest |m(row,col)|.
        std::size_t pivot = col;
        T best = abs_(m(col, col));
        for (std::size_t r = col + 1; r < N; ++r) {
            const T v = abs_(m(r, col));
            if (v > best) {
                best = v;
                pivot = r;
            }
        }
        if (pivot != col) {
            for (std::size_t j = 0; j < N; ++j) {
                std::swap(m(col, j), m(pivot, j));
                std::swap(inv(col, j), inv(pivot, j));
            }
        }
        const T diag = m(col, col);
        const T inv_diag = T{1} / diag;
        for (std::size_t j = 0; j < N; ++j) {
            m(col, j) = m(col, j) * inv_diag;
            inv(col, j) = inv(col, j) * inv_diag;
        }
        for (std::size_t r = 0; r < N; ++r) {
            if (r == col)
                continue;
            const T factor = m(r, col);
            for (std::size_t j = 0; j < N; ++j) {
                m(r, j) = m(r, j) - factor * m(col, j);
                inv(r, j) = inv(r, j) - factor * inv(col, j);
            }
        }
    }
    return inv;
}

}  // namespace branes::math::lie::detail

#endif  // BRANES_MATH_LIE_DETAIL_HPP
