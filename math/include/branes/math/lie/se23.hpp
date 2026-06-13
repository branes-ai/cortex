// SPDX-License-Identifier: MIT
//
// branes/math/lie/se23.hpp — SE₂(3), the "extended pose" / double-direct-
// isometry group: a rotation plus TWO coupled translations (velocity and
// position). It is the natural state manifold for inertial navigation and the
// foundation of the Invariant EKF (Barrau & Bonnabel, 2017): writing the IMU
// state on SE₂(3) makes the dynamics group-affine, so the right-invariant error
// Jacobians become state-independent and the unobservable yaw/position subspace
// is preserved by construction (issue #347; the cure for the #212 over-confidence
// that no FEJ variant could fix — see eval/observability_probe.hpp).
//
// An element is (R, v, p), R ∈ SO(3), v, p ∈ R³, with the 5×5 matrix
//   X = [ R v p ; 0 1 0 ; 0 0 1 ].
// The Lie algebra se₂(3) is a 9-vector ξ = [φ (rot); ν (vel); ρ (pos)] — rotation
// FIRST (the IEKF convention; note se3.hpp uses translation-first for its 6-vector).
// Both translation components share the SO(3) left Jacobian: exp gives v = Jl(φ)·ν,
// p = Jl(φ)·ρ. Clean-room from standard Lie theory (Solà 2018; Barrau-Bonnabel 2017).
//
// Header-only, C++20.

#ifndef BRANES_MATH_LIE_SE23_HPP
#define BRANES_MATH_LIE_SE23_HPP

#include <branes/math/lie/so3.hpp>

namespace branes::math::lie {

/// SE₂(3) — extended pose (rotation + velocity + position). exp/log, composition,
/// inverse, 5×5 matrix, and the 9×9 adjoint.
template <Scalar T>
class SE23 {
public:
    using Scalar_t = T;
    using Tangent = detail::Vec<T, 9>;  ///< se₂(3): [φ (rot); ν (vel); ρ (pos)]
    using Vector3 = detail::Vec<T, 3>;
    using Matrix3 = detail::Mat<T, 3, 3>;
    using Matrix5 = detail::Mat<T, 5, 5>;
    using Matrix9 = detail::Mat<T, 9, 9>;

    constexpr SE23() noexcept = default;
    SE23(const SO3<T>& rotation, const Vector3& velocity, const Vector3& position)
        : R_(rotation), v_(velocity), p_(position) {}

    [[nodiscard]] const SO3<T>& rotation() const noexcept {
        return R_;
    }
    [[nodiscard]] const Vector3& velocity() const noexcept {
        return v_;
    }
    [[nodiscard]] const Vector3& position() const noexcept {
        return p_;
    }

    // ── exp / log ────────────────────────────────────────────────────
    [[nodiscard]] static SE23 exp(const Tangent& xi) {
        const Vector3 phi{{xi[0], xi[1], xi[2]}};
        const Vector3 nu{{xi[3], xi[4], xi[5]}};
        const Vector3 rho{{xi[6], xi[7], xi[8]}};
        const SO3<T> R = SO3<T>::exp(phi);
        const Matrix3 Jl = SO3<T>::left_jacobian(phi);
        return SE23(R, Jl * nu, Jl * rho);
    }

    [[nodiscard]] Tangent log() const {
        const Vector3 phi = R_.log();
        const Matrix3 Jli = SO3<T>::left_jacobian_inverse(phi);
        const Vector3 nu = Jli * v_;
        const Vector3 rho = Jli * p_;
        return Tangent{{phi[0], phi[1], phi[2], nu[0], nu[1], nu[2], rho[0], rho[1], rho[2]}};
    }

    // ── group operations ─────────────────────────────────────────────
    [[nodiscard]] SE23 operator*(const SE23& o) const {
        return SE23(R_ * o.R_, R_ * o.v_ + v_, R_ * o.p_ + p_);
    }

    [[nodiscard]] SE23 inverse() const {
        const SO3<T> Ri = R_.inverse();
        return SE23(Ri, -(Ri * v_), -(Ri * p_));
    }

    /// 5×5 matrix representation [ R v p ; 0 1 0 ; 0 0 1 ].
    [[nodiscard]] Matrix5 matrix() const {
        const Matrix3 R = R_.matrix();
        Matrix5 M = Matrix5::identity();
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j)
                M(i, j) = R(i, j);
            M(i, 3) = v_[i];
            M(i, 4) = p_[i];
        }
        return M;
    }

    /// 9×9 adjoint in the [φ; ν; ρ] ordering:
    ///   [[R, 0, 0], [[v]× R, R, 0], [[p]× R, 0, R]].
    [[nodiscard]] Matrix9 adjoint() const {
        const Matrix3 R = R_.matrix();
        Matrix9 A{};
        place(A, 0, 0, R);
        place(A, 3, 0, detail::hat(v_) * R);
        place(A, 3, 3, R);
        place(A, 6, 0, detail::hat(p_) * R);
        place(A, 6, 6, R);
        return A;
    }

private:
    static void place(Matrix9& M, std::size_t r0, std::size_t c0, const Matrix3& B) {
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                M(r0 + i, c0 + j) = B(i, j);
    }

    SO3<T> R_{};
    Vector3 v_{};
    Vector3 p_{};
};

}  // namespace branes::math::lie

#endif  // BRANES_MATH_LIE_SE23_HPP
