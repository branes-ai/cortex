// SPDX-License-Identifier: MIT
//
// branes/math/cameras/unified.hpp — unified omnidirectional camera
// (Mei–Rives): a point is lifted to a unit sphere, re-projected from a
// center offset by ξ, then radtan-distorted. Clean-room from the
// published model (C. Mei & P. Rives, 2007). Header-only, type-generic.

#ifndef BRANES_MATH_CAMERAS_UNIFIED_HPP
#define BRANES_MATH_CAMERAS_UNIFIED_HPP

#include <branes/math/cameras/detail.hpp>

namespace branes::math::cameras {

/// Pinhole intrinsics (fx, fy, cx, cy) + unified mirror parameter ξ +
/// radtan distortion (k1, k2, p1, p2).
template <Scalar T>
class OmnidirectionalCamera {
public:
    OmnidirectionalCamera() = default;
    OmnidirectionalCamera(T fx, T fy, T cx, T cy, T xi, T k1 = T{0}, T k2 = T{0}, T p1 = T{0}, T p2 = T{0})
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), xi_(xi), dist_{k1, k2, p1, p2, T{0}} {}

    [[nodiscard]] Vec2<T> project(const Vec3<T>& p) const {
        const T d = detail::sqrt_(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        const T w = p[2] + xi_ * d;
        const Vec2<T> n{p[0] / w, p[1] / w};
        const Vec2<T> dd = dist_.distort(n);
        return {fx_ * dd[0] + cx_, fy_ * dd[1] + cy_};
    }

    [[nodiscard]] Vec3<T> unproject(const Vec2<T>& px) const {
        const Vec2<T> nd{(px[0] - cx_) / fx_, (px[1] - cy_) / fy_};
        const Vec2<T> n = dist_.undistort(nd);
        // Lift normalized coords to the unit sphere (UCM inverse).
        const T rho2 = n[0] * n[0] + n[1] * n[1];
        const T factor = (xi_ + detail::sqrt_(T{1} + (T{1} - xi_ * xi_) * rho2)) / (rho2 + T{1});
        return {factor * n[0], factor * n[1], factor - xi_};
    }

    [[nodiscard]] Vec2<T> distort(const Vec2<T>& n) const {
        return dist_.distort(n);
    }
    [[nodiscard]] Vec2<T> undistort(const Vec2<T>& d) const {
        return dist_.undistort(d);
    }

    /// Analytical d(pixel)/d(point), 2x3 row-major.
    [[nodiscard]] Mat23<T> project_jacobian(const Vec3<T>& p) const {
        const T X = p[0], Y = p[1], Z = p[2];
        const T d = detail::sqrt_(X * X + Y * Y + Z * Z);
        const T w = Z + xi_ * d;
        const T iw2 = T{1} / (w * w);
        // d(sphere-projected normalized)/d(point).
        const Mat23<T> Js{(w - xi_ * X * X / d) * iw2,
                          -xi_ * X * Y / d * iw2,
                          -X * (T{1} + xi_ * Z / d) * iw2,
                          -xi_ * X * Y / d * iw2,
                          (w - xi_ * Y * Y / d) * iw2,
                          -Y * (T{1} + xi_ * Z / d) * iw2};
        const Vec2<T> n{X / w, Y / w};
        const Mat22<T> Jd = dist_.jacobian(n);
        Mat23<T> J = detail::mul_22_23(Jd, Js);
        for (std::size_t j = 0; j < 3; ++j) {
            J[0 * 3 + j] *= fx_;
            J[1 * 3 + j] *= fy_;
        }
        return J;
    }

private:
    T fx_{1}, fy_{1}, cx_{0}, cy_{0}, xi_{0};
    detail::RadTan<T> dist_{};
};

}  // namespace branes::math::cameras

#endif  // BRANES_MATH_CAMERAS_UNIFIED_HPP
