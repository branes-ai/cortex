// SPDX-License-Identifier: MIT
//
// branes/math/cameras/detail.hpp — shared primitives for the camera
// models: small fixed-size vector aliases, ADL scalar transcendentals
// (std:: for built-in floats, sw::universal:: for Universal types), and
// the radial-tangential (Brown–Conrady) distortion shared by the pinhole
// and unified (Mei) models.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_MATH_CAMERAS_DETAIL_HPP
#define BRANES_MATH_CAMERAS_DETAIL_HPP

#include <branes/math/arithmetic.hpp>

#include <array>
#include <cmath>
#include <cstddef>

namespace branes::math::cameras {

template <class T>
using Vec2 = std::array<T, 2>;
template <class T>
using Vec3 = std::array<T, 3>;
/// Row-major 2x2 Jacobian: [∂f0/∂x0, ∂f0/∂x1, ∂f1/∂x0, ∂f1/∂x1].
template <class T>
using Mat22 = std::array<T, 4>;
/// Row-major 2x3 Jacobian (d pixel / d point).
template <class T>
using Mat23 = std::array<T, 6>;

namespace detail {

template <Scalar T>
[[nodiscard]] T sqrt_(const T& x) {
    using std::sqrt;
    return sqrt(x);
}
template <Scalar T>
[[nodiscard]] T atan_(const T& x) {
    using std::atan;
    return atan(x);
}
template <Scalar T>
[[nodiscard]] T atan2_(const T& y, const T& x) {
    using std::atan2;
    return atan2(y, x);
}
template <Scalar T>
[[nodiscard]] T tan_(const T& x) {
    using std::tan;
    return tan(x);
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
[[nodiscard]] T abs_(const T& x) {
    using std::abs;
    return abs(x);
}

/// Multiply a 2x2 (row-major) by a 2x3 (row-major) → 2x3 (row-major).
template <Scalar T>
[[nodiscard]] constexpr Mat23<T> mul_22_23(const Mat22<T>& a, const Mat23<T>& b) {
    Mat23<T> r{};
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            r[i * 3 + j] = a[i * 2 + 0] * b[0 * 3 + j] + a[i * 2 + 1] * b[1 * 3 + j];
    return r;
}

// ── Radial-tangential (Brown–Conrady) distortion on normalized coords ──
//
// Maps a normalized image point n = (x, y) = (X/Z, Y/Z) to its distorted
// counterpart using radial (k1,k2,k3) and tangential (p1,p2) coefficients.

template <Scalar T>
struct RadTan {
    T k1{0}, k2{0}, p1{0}, p2{0}, k3{0};

    [[nodiscard]] Vec2<T> distort(const Vec2<T>& n) const {
        const T x = n[0], y = n[1];
        const T r2 = x * x + y * y;
        const T radial = T{1} + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        const T xy = x * y;
        const T xd = x * radial + T{2} * p1 * xy + p2 * (r2 + T{2} * x * x);
        const T yd = y * radial + p1 * (r2 + T{2} * y * y) + T{2} * p2 * xy;
        return {xd, yd};
    }

    /// 2x2 Jacobian d(distort)/d(x,y), row-major.
    [[nodiscard]] Mat22<T> jacobian(const Vec2<T>& n) const {
        const T x = n[0], y = n[1];
        const T r2 = x * x + y * y;
        const T radial = T{1} + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
        // d(radial)/d(r2) = k1 + 2 k2 r2 + 3 k3 r2^2; d(r2)/dx = 2x, /dy = 2y.
        const T dradial_dr2 = k1 + T{2} * k2 * r2 + T{3} * k3 * r2 * r2;
        const T drad_dx = T{2} * x * dradial_dr2;
        const T drad_dy = T{2} * y * dradial_dr2;
        // xd = x*radial + 2 p1 x y + p2 (r2 + 2 x^2)
        const T dxd_dx = radial + x * drad_dx + T{2} * p1 * y + p2 * (T{2} * x + T{4} * x);
        const T dxd_dy = x * drad_dy + T{2} * p1 * x + p2 * (T{2} * y);
        // yd = y*radial + p1 (r2 + 2 y^2) + 2 p2 x y
        const T dyd_dx = y * drad_dx + p1 * (T{2} * x) + T{2} * p2 * y;
        const T dyd_dy = radial + y * drad_dy + p1 * (T{2} * y + T{4} * y) + T{2} * p2 * x;
        return {dxd_dx, dxd_dy, dyd_dx, dyd_dy};
    }

    /// Invert the distortion by fixed-point iteration (Newton-free; the
    /// standard OpenCV-style undistort iteration), recovering normalized
    /// coords from distorted ones.
    [[nodiscard]] Vec2<T> undistort(const Vec2<T>& d) const {
        Vec2<T> n = d;  // initial guess
        for (int iter = 0; iter < 20; ++iter) {
            const Vec2<T> f = distort(n);
            const Mat22<T> J = jacobian(n);
            const T rx = f[0] - d[0];
            const T ry = f[1] - d[1];
            const T det = J[0] * J[3] - J[1] * J[2];
            if (abs_(det) < T(1e-12))
                break;  // parens: 1e12 not float-exact
            const T inv = T{1} / det;
            // Newton step: n -= J^{-1} r.
            const T dx = inv * (J[3] * rx - J[1] * ry);
            const T dy = inv * (-J[2] * rx + J[0] * ry);
            n[0] = n[0] - dx;
            n[1] = n[1] - dy;
            if (abs_(dx) + abs_(dy) < T(1e-11))
                break;
        }
        return n;
    }
};

}  // namespace detail
}  // namespace branes::math::cameras

#endif  // BRANES_MATH_CAMERAS_DETAIL_HPP
