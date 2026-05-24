// SPDX-License-Identifier: MIT
//
// branes/math/lie/so3.hpp — SO(3), the group of 3D rotations.
//
// Clean-room implementation from standard Lie theory (J. Solà et al.,
// "A micro Lie theory for state estimation in robotics", 2018; T.
// Barfoot, "State Estimation for Robotics", 2017). No third-party source
// was consulted. Type-generic over the scalar.
//
// Storage is a unit quaternion (w, x, y, z), canonicalized to w >= 0.
// The Lie algebra so(3) is identified with R^3 (rotation vectors); `hat`
// maps a rotation vector to a skew-symmetric matrix. Convention: the
// exponential is the right-handed axis-angle rotation, and `adjoint()`
// equals the rotation matrix.
//
// Header-only, C++20.

#ifndef BRANES_MATH_LIE_SO3_HPP
#define BRANES_MATH_LIE_SO3_HPP

#include <branes/math/lie/detail.hpp>

namespace branes::math::lie {

template <Scalar T>
class SO3 {
public:
    using Scalar_t = T;
    using Tangent = detail::Vec<T, 3>;  ///< so(3) ~ R^3
    using Vector3 = detail::Vec<T, 3>;
    using Matrix3 = detail::Mat<T, 3, 3>;
    using Quaternion = detail::Vec<T, 4>;  ///< (w, x, y, z)

    /// Identity rotation.
    constexpr SO3() noexcept : q_{{T{1}, T{0}, T{0}, T{0}}} {}

    /// Construct from a (not necessarily normalized) quaternion (w,x,y,z).
    explicit SO3(const Quaternion& q) : q_(q) {
        normalize();
    }

    [[nodiscard]] const Quaternion& quaternion() const noexcept {
        return q_;
    }

    // ── exp / log ────────────────────────────────────────────────────

    /// Exponential map so(3) -> SO(3): rotation vector to rotation.
    [[nodiscard]] static SO3 exp(const Tangent& phi) {
        const T theta2 = detail::dot(phi, phi);
        const T theta = detail::sqrt_(theta2);
        T real, imag;  // quaternion real part, and sin(theta/2)/theta
        if (theta < kSmall) {
            // Series: cos(theta/2) and sin(theta/2)/theta about theta=0.
            real = T{1} - theta2 * T{0.125};
            imag = T{0.5} - theta2 * (T{1} / T{48});
        } else {
            const T half = theta * T{0.5};
            real = detail::cos_(half);
            imag = detail::sin_(half) / theta;
        }
        SO3 r;
        r.q_ = Quaternion{{real, imag * phi[0], imag * phi[1], imag * phi[2]}};
        r.normalize();
        return r;
    }

    /// Logarithm map SO(3) -> so(3): rotation to rotation vector.
    [[nodiscard]] Tangent log() const {
        const Vector3 v{{q_[1], q_[2], q_[3]}};
        const T nv = detail::norm(v);
        const T w = q_[0];  // canonicalized w >= 0
        T scale;
        if (nv < kSmall) {
            // theta ~ 0: scale = 2/w * (1 + (nv/w)^2/3 + ...) ~ 2 (w~1).
            scale = T{2} / w - (T{2} / T{3}) * (nv * nv) / (w * w * w);
        } else {
            scale = T{2} * detail::atan2_(nv, w) / nv;
        }
        return Vector3{{v[0] * scale, v[1] * scale, v[2] * scale}};
    }

    // ── group operations ─────────────────────────────────────────────

    /// Composition (Hamilton quaternion product).
    [[nodiscard]] SO3 operator*(const SO3& o) const {
        const auto& a = q_;
        const auto& b = o.q_;
        SO3 r;
        r.q_ = Quaternion{{
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
        }};
        r.normalize();
        return r;
    }

    /// Rotate a 3-vector.
    [[nodiscard]] Vector3 operator*(const Vector3& v) const {
        return matrix() * v;
    }

    [[nodiscard]] SO3 inverse() const {
        SO3 r;
        r.q_ = Quaternion{{q_[0], -q_[1], -q_[2], -q_[3]}};
        return r;  // already unit; conjugate of canonicalized stays w>=0
    }

    /// 3x3 rotation matrix.
    [[nodiscard]] Matrix3 matrix() const {
        const T w = q_[0], x = q_[1], y = q_[2], z = q_[3];
        Matrix3 R{};
        R(0, 0) = T{1} - T{2} * (y * y + z * z);
        R(0, 1) = T{2} * (x * y - w * z);
        R(0, 2) = T{2} * (x * z + w * y);
        R(1, 0) = T{2} * (x * y + w * z);
        R(1, 1) = T{1} - T{2} * (x * x + z * z);
        R(1, 2) = T{2} * (y * z - w * x);
        R(2, 0) = T{2} * (x * z - w * y);
        R(2, 1) = T{2} * (y * z + w * x);
        R(2, 2) = T{1} - T{2} * (x * x + y * y);
        return R;
    }

    /// Adjoint of SO(3) is the rotation matrix itself.
    [[nodiscard]] Matrix3 adjoint() const {
        return matrix();
    }

    // ── Jacobians of the exponential map ─────────────────────────────

    /// Left Jacobian Jl(phi): exp(phi + dphi) ~ exp(Jl dphi) exp(phi)
    /// to first order (left-perturbation convention).
    [[nodiscard]] static Matrix3 left_jacobian(const Tangent& phi) {
        const Matrix3 W = detail::hat(phi);
        const Matrix3 W2 = W * W;
        const T theta2 = detail::dot(phi, phi);
        const Matrix3 I = Matrix3::identity();
        if (theta2 < kSmall * kSmall) {
            // Jl ~ I + 1/2 W + 1/6 W^2.
            return I + W * T{0.5} + W2 * (T{1} / T{6});
        }
        const T theta = detail::sqrt_(theta2);
        const T a = (T{1} - detail::cos_(theta)) / theta2;
        const T b = (theta - detail::sin_(theta)) / (theta2 * theta);
        return I + W * a + W2 * b;
    }

    /// Inverse of the left Jacobian.
    [[nodiscard]] static Matrix3 left_jacobian_inverse(const Tangent& phi) {
        const Matrix3 W = detail::hat(phi);
        const Matrix3 W2 = W * W;
        const T theta2 = detail::dot(phi, phi);
        const Matrix3 I = Matrix3::identity();
        if (theta2 < kSmall * kSmall) {
            // Jl^{-1} ~ I - 1/2 W + 1/12 W^2.
            return I - W * T{0.5} + W2 * (T{1} / T{12});
        }
        const T theta = detail::sqrt_(theta2);
        const T half = theta * T{0.5};
        // c = 1/theta^2 - (1 + cos)/(2 theta sin) = 1/theta^2 (1 - (theta/2) cot(theta/2))
        const T c = T{1} / theta2 - (T{1} + detail::cos_(theta)) / (T{2} * theta * detail::sin_(theta));
        (void)half;
        return I - W * T{0.5} + W2 * c;
    }

    /// Right Jacobian Jr(phi) = Jl(-phi).
    [[nodiscard]] static Matrix3 right_jacobian(const Tangent& phi) {
        return left_jacobian(-phi);
    }
    /// Inverse of the right Jacobian.
    [[nodiscard]] static Matrix3 right_jacobian_inverse(const Tangent& phi) {
        return left_jacobian_inverse(-phi);
    }

private:
    // Small-angle threshold; series expansions kick in below this.
    static constexpr T kSmall = T{1} / T{100000};  // 1e-5

    void normalize() {
        T n = detail::sqrt_(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
        const T inv = T{1} / n;
        for (std::size_t i = 0; i < 4; ++i)
            q_[i] = q_[i] * inv;
        if (q_[0] < T{0}) {
            for (std::size_t i = 0; i < 4; ++i)
                q_[i] = -q_[i];
        }
    }

    Quaternion q_;
};

}  // namespace branes::math::lie

#endif  // BRANES_MATH_LIE_SO3_HPP
