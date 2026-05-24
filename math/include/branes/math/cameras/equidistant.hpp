// SPDX-License-Identifier: MIT
//
// branes/math/cameras/equidistant.hpp — equidistant (Kannala–Brandt)
// fisheye camera. Clean-room from the published KB model (J. Kannala &
// S. Brandt, 2006). Header-only, type-generic.

#ifndef BRANES_MATH_CAMERAS_EQUIDISTANT_HPP
#define BRANES_MATH_CAMERAS_EQUIDISTANT_HPP

#include <branes/math/cameras/detail.hpp>

namespace branes::math::cameras {

/// Pinhole intrinsics (fx, fy, cx, cy) + 4-parameter equidistant fisheye
/// distortion (k1..k4) on the incidence angle θ = atan(‖n‖).
template <Scalar T>
class EquidistantCamera {
public:
    EquidistantCamera() = default;
    EquidistantCamera(T fx, T fy, T cx, T cy, T k1, T k2, T k3, T k4)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), k1_(k1), k2_(k2), k3_(k3), k4_(k4) {}

    [[nodiscard]] Vec2<T> project(const Vec3<T>& p) const {
        const Vec2<T> n{p[0] / p[2], p[1] / p[2]};
        const Vec2<T> d = distort(n);
        return {fx_ * d[0] + cx_, fy_ * d[1] + cy_};
    }

    [[nodiscard]] Vec3<T> unproject(const Vec2<T>& px) const {
        const Vec2<T> d{(px[0] - cx_) / fx_, (px[1] - cy_) / fy_};
        const Vec2<T> n = undistort(d);
        const T inv = T{1} / detail::sqrt_(n[0] * n[0] + n[1] * n[1] + T{1});
        return {n[0] * inv, n[1] * inv, inv};
    }

    /// Fisheye distortion of a normalized point.
    [[nodiscard]] Vec2<T> distort(const Vec2<T>& n) const {
        const T r = detail::sqrt_(n[0] * n[0] + n[1] * n[1]);
        if (r < detail::solver_eps<T>())
            return n;  // limit: scale → 1 at the optical axis
        const T theta = detail::atan_(r);
        const T td = theta_d(theta);
        const T scale = td / r;
        return {scale * n[0], scale * n[1]};
    }

    /// Inverse fisheye distortion (Newton solve for θ).
    [[nodiscard]] Vec2<T> undistort(const Vec2<T>& d) const {
        const T rd = detail::sqrt_(d[0] * d[0] + d[1] * d[1]);
        if (rd < detail::solver_eps<T>())
            return d;
        T theta = rd;  // initial guess
        for (int i = 0; i < 20; ++i) {
            const T f = theta_d(theta) - rd;
            const T fp = dtheta_d_dtheta(theta);
            if (detail::abs_(fp) < detail::solver_eps<T>())
                break;  // flat derivative: avoid divide-by-near-zero
            const T step = f / fp;
            theta -= step;
            if (detail::abs_(step) < detail::solver_eps<T>())
                break;
        }
        const T r = detail::tan_(theta);
        const T scale = r / rd;
        return {scale * d[0], scale * d[1]};
    }

    /// Analytical d(pixel)/d(point), 2x3 row-major.
    [[nodiscard]] Mat23<T> project_jacobian(const Vec3<T>& p) const {
        const T X = p[0], Y = p[1], Z = p[2];
        const T iz = T{1} / Z;
        const Mat23<T> Jn{iz, T{0}, -X * iz * iz, T{0}, iz, -Y * iz * iz};
        const Mat22<T> Jd = distort_jacobian({X * iz, Y * iz});
        Mat23<T> J = detail::mul_22_23(Jd, Jn);
        for (std::size_t j = 0; j < 3; ++j) {
            J[0 * 3 + j] *= fx_;
            J[1 * 3 + j] *= fy_;
        }
        return J;
    }

private:
    [[nodiscard]] T theta_d(T theta) const {
        const T t2 = theta * theta;
        return theta * (T{1} + t2 * (k1_ + t2 * (k2_ + t2 * (k3_ + t2 * k4_))));
    }
    [[nodiscard]] T dtheta_d_dtheta(T theta) const {
        const T t2 = theta * theta;
        return T{1} + t2 * (T{3} * k1_ + t2 * (T{5} * k2_ + t2 * (T{7} * k3_ + t2 * T{9} * k4_)));
    }

    /// 2x2 Jacobian d(distort)/d(a,b). With x_d = f(r)·a, y_d = f(r)·b
    /// where f(r) = θ_d(atan r)/r:
    ///   ∂x_d/∂a = f + f'·a²/r,  ∂x_d/∂b = f'·a·b/r,  (symmetric for y).
    [[nodiscard]] Mat22<T> distort_jacobian(const Vec2<T>& n) const {
        const T a = n[0], b = n[1];
        const T r = detail::sqrt_(a * a + b * b);
        if (r < detail::solver_eps<T>())
            return {T{1}, T{0}, T{0}, T{1}};
        const T theta = detail::atan_(r);
        const T td = theta_d(theta);
        const T f = td / r;
        // f'(r) = (θ_d'(r)·r − θ_d)/r², with θ_d'(r) = (dθ_d/dθ)/(1+r²).
        const T td_prime_r = dtheta_d_dtheta(theta) / (T{1} + r * r);
        const T fp = (td_prime_r * r - td) / (r * r);
        const T fp_over_r = fp / r;
        return {f + fp_over_r * a * a, fp_over_r * a * b, fp_over_r * a * b, f + fp_over_r * b * b};
    }

    T fx_{1}, fy_{1}, cx_{0}, cy_{0};
    T k1_{0}, k2_{0}, k3_{0}, k4_{0};
};

}  // namespace branes::math::cameras

#endif  // BRANES_MATH_CAMERAS_EQUIDISTANT_HPP
