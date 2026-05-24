// SPDX-License-Identifier: MIT
//
// branes/math/cameras/pinhole_radtan.hpp — pinhole camera with
// radial-tangential (Brown–Conrady) distortion. The standard EuRoC MAV
// projection model. Clean-room from the published model. Header-only,
// type-generic.

#ifndef BRANES_MATH_CAMERAS_PINHOLE_RADTAN_HPP
#define BRANES_MATH_CAMERAS_PINHOLE_RADTAN_HPP

#include <branes/math/cameras/detail.hpp>

namespace branes::math::cameras {

/// Pinhole intrinsics (fx, fy, cx, cy) + 5-parameter radtan distortion
/// (k1, k2, p1, p2, k3). Points are in the camera frame with +Z forward.
template <Scalar T>
class PinholeRadtanCamera {
public:
    PinholeRadtanCamera() = default;
    PinholeRadtanCamera(T fx, T fy, T cx, T cy, T k1, T k2, T p1, T p2, T k3 = T{0})
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), dist_{k1, k2, p1, p2, k3} {}

    /// Project a 3D camera-frame point to a pixel.
    [[nodiscard]] Vec2<T> project(const Vec3<T>& p) const {
        const Vec2<T> n{p[0] / p[2], p[1] / p[2]};
        const Vec2<T> d = dist_.distort(n);
        return {fx_ * d[0] + cx_, fy_ * d[1] + cy_};
    }

    /// Unproject a pixel to a unit bearing (camera frame, +Z forward).
    [[nodiscard]] Vec3<T> unproject(const Vec2<T>& px) const {
        const Vec2<T> nd{(px[0] - cx_) / fx_, (px[1] - cy_) / fy_};
        const Vec2<T> n = dist_.undistort(nd);
        const T inv = T{1} / detail::sqrt_(n[0] * n[0] + n[1] * n[1] + T{1});
        return {n[0] * inv, n[1] * inv, inv};
    }

    /// Radtan distortion of a normalized point, and its inverse.
    [[nodiscard]] Vec2<T> distort(const Vec2<T>& n) const {
        return dist_.distort(n);
    }
    [[nodiscard]] Vec2<T> undistort(const Vec2<T>& d) const {
        return dist_.undistort(d);
    }

    /// Analytical d(pixel)/d(point), 2x3 row-major.
    [[nodiscard]] Mat23<T> project_jacobian(const Vec3<T>& p) const {
        const T X = p[0], Y = p[1], Z = p[2];
        const T iz = T{1} / Z;
        const Vec2<T> n{X * iz, Y * iz};
        // d(normalized)/d(point): [[1/Z, 0, -X/Z^2], [0, 1/Z, -Y/Z^2]].
        const Mat23<T> Jn{iz, T{0}, -X * iz * iz, T{0}, iz, -Y * iz * iz};
        const Mat22<T> Jd = dist_.jacobian(n);
        Mat23<T> Jdn = detail::mul_22_23(Jd, Jn);
        // Apply diag(fx, fy) on the left (scale rows).
        for (std::size_t j = 0; j < 3; ++j) {
            Jdn[0 * 3 + j] *= fx_;
            Jdn[1 * 3 + j] *= fy_;
        }
        return Jdn;
    }

private:
    T fx_{1}, fy_{1}, cx_{0}, cy_{0};
    detail::RadTan<T> dist_{};
};

}  // namespace branes::math::cameras

#endif  // BRANES_MATH_CAMERAS_PINHOLE_RADTAN_HPP
