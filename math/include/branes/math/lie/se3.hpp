// SPDX-License-Identifier: MIT
//
// branes/math/lie/se3.hpp — SE(3), the group of 3D rigid-body motions.
//
// Clean-room implementation from standard Lie theory (J. Solà et al.,
// "A micro Lie theory", 2018; T. Barfoot, "State Estimation for
// Robotics", 2017 — the SE(3) left-Jacobian Q block uses the closed
// form of Barfoot eq. 7.86). No third-party source consulted.
//
// Stored as an SO3 rotation plus a translation 3-vector. The Lie algebra
// se(3) is a 6-vector ξ = [ρ; φ] with the *translation part first*
// (ρ ∈ R^3) and the rotation part second (φ ∈ R^3) — the Solà / Barfoot
// convention. With this ordering the adjoint is [[R, [t]× R], [0, R]].
//
// Header-only, C++20.

#ifndef BRANES_MATH_LIE_SE3_HPP
#define BRANES_MATH_LIE_SE3_HPP

#include <branes/math/lie/so3.hpp>

namespace branes::math::lie {

/// SE(3) — the group of 3D rigid-body motions (rotation + translation).
/// Provides exp/log, composition, inverse, point action, homogeneous
/// matrix, 6×6 adjoint, and the full left/right exponential Jacobians.
template <Scalar T>
class SE3 {
public:
    using Scalar_t = T;
    using Tangent = detail::Vec<T, 6>;  ///< se(3): [ρ (trans); φ (rot)]
    using Vector3 = detail::Vec<T, 3>;
    using Matrix3 = detail::Mat<T, 3, 3>;
    using Matrix4 = detail::Mat<T, 4, 4>;
    using Matrix6 = detail::Mat<T, 6, 6>;

    /// Identity transform.
    constexpr SE3() noexcept = default;

    SE3(const SO3<T>& rotation, const Vector3& translation) : R_(rotation), t_(translation) {}

    [[nodiscard]] const SO3<T>& rotation() const noexcept {
        return R_;
    }
    [[nodiscard]] const Vector3& translation() const noexcept {
        return t_;
    }

    // ── exp / log ────────────────────────────────────────────────────

    [[nodiscard]] static SE3 exp(const Tangent& xi) {
        const Vector3 rho{{xi[0], xi[1], xi[2]}};
        const Vector3 phi{{xi[3], xi[4], xi[5]}};
        const SO3<T> R = SO3<T>::exp(phi);
        const Matrix3 V = SO3<T>::left_jacobian(phi);
        return SE3(R, V * rho);
    }

    [[nodiscard]] Tangent log() const {
        const Vector3 phi = R_.log();
        const Matrix3 Vinv = SO3<T>::left_jacobian_inverse(phi);
        const Vector3 rho = Vinv * t_;
        return Tangent{{rho[0], rho[1], rho[2], phi[0], phi[1], phi[2]}};
    }

    // ── group operations ─────────────────────────────────────────────

    [[nodiscard]] SE3 operator*(const SE3& o) const {
        return SE3(R_ * o.R_, R_ * o.t_ + t_);
    }

    /// Act on a point: R p + t.
    [[nodiscard]] Vector3 operator*(const Vector3& p) const {
        return R_ * p + t_;
    }

    [[nodiscard]] SE3 inverse() const {
        const SO3<T> Ri = R_.inverse();
        return SE3(Ri, -(Ri * t_));
    }

    /// 4x4 homogeneous transform.
    [[nodiscard]] Matrix4 matrix() const {
        const Matrix3 R = R_.matrix();
        Matrix4 M = Matrix4::identity();
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j)
                M(i, j) = R(i, j);
            M(i, 3) = t_[i];
        }
        return M;
    }

    /// 6x6 adjoint in the [ρ; φ] ordering: [[R, [t]× R], [0, R]].
    [[nodiscard]] Matrix6 adjoint() const {
        const Matrix3 R = R_.matrix();
        const Matrix3 TR = detail::hat(t_) * R;
        Matrix6 A{};
        place(A, 0, 0, R);
        place(A, 0, 3, TR);
        place(A, 3, 3, R);
        return A;
    }

    // ── Jacobians ────────────────────────────────────────────────────

    /// Left Jacobian of the exponential, 6x6: [[Jl, Q], [0, Jl]].
    [[nodiscard]] static Matrix6 left_jacobian(const Tangent& xi) {
        const Vector3 rho{{xi[0], xi[1], xi[2]}};
        const Vector3 phi{{xi[3], xi[4], xi[5]}};
        const Matrix3 Jl = SO3<T>::left_jacobian(phi);
        const Matrix3 Q = q_matrix(rho, phi);
        Matrix6 J{};
        place(J, 0, 0, Jl);
        place(J, 0, 3, Q);
        place(J, 3, 3, Jl);
        return J;
    }

    /// Inverse of the left Jacobian: [[Jl⁻¹, −Jl⁻¹ Q Jl⁻¹], [0, Jl⁻¹]].
    [[nodiscard]] static Matrix6 left_jacobian_inverse(const Tangent& xi) {
        const Vector3 rho{{xi[0], xi[1], xi[2]}};
        const Vector3 phi{{xi[3], xi[4], xi[5]}};
        const Matrix3 Jli = SO3<T>::left_jacobian_inverse(phi);
        const Matrix3 Q = q_matrix(rho, phi);
        const Matrix3 top_right = (Jli * Q * Jli) * T{-1};
        Matrix6 J{};
        place(J, 0, 0, Jli);
        place(J, 0, 3, top_right);
        place(J, 3, 3, Jli);
        return J;
    }

    [[nodiscard]] static Matrix6 right_jacobian(const Tangent& xi) {
        return left_jacobian(neg(xi));
    }
    [[nodiscard]] static Matrix6 right_jacobian_inverse(const Tangent& xi) {
        return left_jacobian_inverse(neg(xi));
    }

private:
    static constexpr T kSmall = T{1} / T{100000};  // 1e-5

    static Tangent neg(const Tangent& xi) {
        Tangent r;
        for (std::size_t i = 0; i < 6; ++i)
            r[i] = -xi[i];
        return r;
    }

    static void place(Matrix6& M, std::size_t r0, std::size_t c0, const Matrix3& B) {
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                M(r0 + i, c0 + j) = B(i, j);
    }

    /// SE(3) left-Jacobian Q block (Barfoot eq. 7.86).
    static Matrix3 q_matrix(const Vector3& rho, const Vector3& phi) {
        const Matrix3 P = detail::hat(rho);
        const Matrix3 W = detail::hat(phi);
        const T theta2 = detail::dot(phi, phi);

        T c1, c2, c3;
        if (theta2 < kSmall * kSmall) {
            c1 = T{1} / T{6};
            c2 = T{1} / T{24};
            c3 = T{1} / T{120};
        } else {
            const T theta = detail::sqrt_(theta2);
            const T s = detail::sin_(theta);
            const T c = detail::cos_(theta);
            const T t4 = theta2 * theta2;
            c1 = (theta - s) / (theta2 * theta);
            c2 = (theta2 + T{2} * c - T{2}) / (T{2} * t4);
            c3 = (T{2} * theta - T{3} * s + theta * c) / (T{2} * t4 * theta);
        }

        const Matrix3 WP = W * P;
        const Matrix3 PW = P * W;
        const Matrix3 WPW = WP * W;
        const Matrix3 WW = W * W;
        const Matrix3 WWP = WW * P;
        const Matrix3 PWW = P * WW;
        const Matrix3 WPWW = WPW * W;
        const Matrix3 WWPW = WW * PW;

        Matrix3 Q = P * T{0.5};
        Q = Q + (WP + PW + WPW) * c1;
        Q = Q + (WWP + PWW + WPW * T{-3}) * c2;
        Q = Q + (WPWW + WWPW) * c3;
        return Q;
    }

    SO3<T> R_{};
    Vector3 t_{};
};

}  // namespace branes::math::lie

#endif  // BRANES_MATH_LIE_SE3_HPP
