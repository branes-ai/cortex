// SPDX-License-Identifier: MIT
//
// branes/math/lie/sim3.hpp — Sim(3), 3D similarity transforms
// (scale · rotation · translation).
//
// Clean-room implementation from first principles. An element is
// (s, R, t) acting on a point as p ↦ s R p + t, s > 0. The Lie algebra
// sim(3) is a 7-vector τ = [ρ; φ; σ] (translation, rotation, log-scale).
//
// The exponential's translation block uses the closed-form integral
//   W = ∫₀¹ e^{σu} exp(u φ^) du = A·I + (B/θ)·φ^ + (C/θ²)·(φ^)²,
// derived directly (no third-party source). The 7×7 exp Jacobians use
// the exact, group-agnostic identity Jl = Σ_{n≥0} ad^n / (n+1)!, where
// ad(τ) is the sim(3) algebra adjoint — this avoids a bespoke (and
// error-prone) closed form and is validated against finite differences.
//
// Header-only, C++20.

#ifndef BRANES_MATH_LIE_SIM3_HPP
#define BRANES_MATH_LIE_SIM3_HPP

#include <branes/math/lie/so3.hpp>

namespace branes::math::lie {

template <Scalar T>
class Sim3 {
public:
    using Scalar_t = T;
    using Tangent = detail::Vec<T, 7>;  ///< sim(3): [ρ; φ; σ]
    using Vector3 = detail::Vec<T, 3>;
    using Matrix3 = detail::Mat<T, 3, 3>;
    using Matrix4 = detail::Mat<T, 4, 4>;
    using Matrix7 = detail::Mat<T, 7, 7>;

    /// Identity (scale 1, no rotation, no translation).
    constexpr Sim3() noexcept = default;

    Sim3(const T& scale, const SO3<T>& rotation, const Vector3& translation)
        : R_(rotation), t_(translation), s_(scale) {}

    [[nodiscard]] const T& scale() const noexcept {
        return s_;
    }
    [[nodiscard]] const SO3<T>& rotation() const noexcept {
        return R_;
    }
    [[nodiscard]] const Vector3& translation() const noexcept {
        return t_;
    }

    // ── exp / log ────────────────────────────────────────────────────

    [[nodiscard]] static Sim3 exp(const Tangent& tau) {
        const Vector3 rho{{tau[0], tau[1], tau[2]}};
        const Vector3 phi{{tau[3], tau[4], tau[5]}};
        const T sigma = tau[6];
        const SO3<T> R = SO3<T>::exp(phi);
        const T s = detail::exp_(sigma);
        const Matrix3 W = w_matrix(phi, sigma);
        return Sim3(s, R, W * rho);
    }

    [[nodiscard]] Tangent log() const {
        const Vector3 phi = R_.log();
        const T sigma = detail::log_(s_);
        const Matrix3 Winv = detail::inverse(w_matrix(phi, sigma));
        const Vector3 rho = Winv * t_;
        return Tangent{{rho[0], rho[1], rho[2], phi[0], phi[1], phi[2], sigma}};
    }

    // ── group operations ─────────────────────────────────────────────

    [[nodiscard]] Sim3 operator*(const Sim3& o) const {
        // (s1 R1, t1)·(s2 R2, t2): scale s1 s2, rot R1 R2,
        // trans s1 R1 t2 + t1.
        return Sim3(s_ * o.s_, R_ * o.R_, (R_ * o.t_) * s_ + t_);
    }

    /// Act on a point: s R p + t.
    [[nodiscard]] Vector3 operator*(const Vector3& p) const {
        return (R_ * p) * s_ + t_;
    }

    [[nodiscard]] Sim3 inverse() const {
        const T si = T{1} / s_;
        const SO3<T> Ri = R_.inverse();
        return Sim3(si, Ri, (Ri * t_) * (-si));
    }

    /// 4x4 homogeneous matrix (upper-left block is s·R).
    [[nodiscard]] Matrix4 matrix() const {
        const Matrix3 sR = R_.matrix() * s_;
        Matrix4 M = Matrix4::identity();
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j)
                M(i, j) = sR(i, j);
            M(i, 3) = t_[i];
        }
        return M;
    }

    /// 7x7 adjoint. Blocks (rows ρ,φ,σ × cols ρ,φ,σ):
    ///   [ sR   [t]× R   -t ]
    ///   [ 0    R         0 ]
    ///   [ 0    0         1 ]
    [[nodiscard]] Matrix7 adjoint() const {
        const Matrix3 R = R_.matrix();
        const Matrix3 sR = R * s_;
        const Matrix3 TR = detail::hat(t_) * R;
        Matrix7 A{};
        place3(A, 0, 0, sR);
        place3(A, 0, 3, TR);
        place3(A, 3, 3, R);
        for (std::size_t i = 0; i < 3; ++i)
            A(i, 6) = -t_[i];
        A(6, 6) = T{1};
        return A;
    }

    // ── Jacobians (exact series Jl = Σ ad^n/(n+1)!) ──────────────────

    [[nodiscard]] static Matrix7 left_jacobian(const Tangent& tau) {
        return jl_series(small_adjoint(tau));
    }
    [[nodiscard]] static Matrix7 left_jacobian_inverse(const Tangent& tau) {
        return detail::inverse(left_jacobian(tau));
    }
    [[nodiscard]] static Matrix7 right_jacobian(const Tangent& tau) {
        return jl_series(small_adjoint(neg(tau)));
    }
    [[nodiscard]] static Matrix7 right_jacobian_inverse(const Tangent& tau) {
        return detail::inverse(right_jacobian(tau));
    }

    /// sim(3) algebra adjoint ad(τ) (7x7). Blocks:
    ///   [ σI + φ^   ρ^   -ρ ]
    ///   [ 0         φ^    0 ]
    ///   [ 0         0     0 ]
    [[nodiscard]] static Matrix7 small_adjoint(const Tangent& tau) {
        const Vector3 rho{{tau[0], tau[1], tau[2]}};
        const Vector3 phi{{tau[3], tau[4], tau[5]}};
        const T sigma = tau[6];
        const Matrix3 Px = detail::hat(rho);
        const Matrix3 Fx = detail::hat(phi);
        Matrix7 ad{};
        place3(ad, 0, 0, Fx);
        for (std::size_t i = 0; i < 3; ++i)
            ad(i, i) += sigma;
        place3(ad, 0, 3, Px);
        place3(ad, 3, 3, Fx);
        for (std::size_t i = 0; i < 3; ++i)
            ad(i, 6) = -rho[i];
        return ad;
    }

private:
    static constexpr T kSmall = T{1} / T{100000};  // 1e-5

    static Tangent neg(const Tangent& tau) {
        Tangent r;
        for (std::size_t i = 0; i < 7; ++i)
            r[i] = -tau[i];
        return r;
    }

    static void place3(Matrix7& M, std::size_t r0, std::size_t c0, const Matrix3& B) {
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                M(r0 + i, c0 + j) = B(i, j);
    }

    /// Jl = Σ_{n=0}^{Nterms} ad^n / (n+1)!. Converges rapidly for the
    /// tangent magnitudes seen in practice; 20 terms is exact to working
    /// precision for ‖τ‖ ≲ a few.
    static Matrix7 jl_series(const Matrix7& ad) {
        Matrix7 term = Matrix7::identity();  // ad^0 / 1!
        Matrix7 sum = term;
        T factorial = T{1};
        constexpr int Nterms = 20;
        for (int n = 1; n <= Nterms; ++n) {
            factorial = factorial * T(n + 1);  // (n+1)!
            term = term * ad;                  // ad^n (numerator part)
            sum = sum + term * (T{1} / factorial);
        }
        return sum;
    }

    /// Translation integral W(φ, σ) = ∫₀¹ e^{σu} exp(u φ^) du
    ///   = A·I + (B/θ)·φ^ + (C/θ²)·(φ^)², with the scalar integrals
    /// evaluated in closed form and small-argument limits applied.
    static Matrix3 w_matrix(const Vector3& phi, const T& sigma) {
        const Matrix3 Fx = detail::hat(phi);
        const Matrix3 Fx2 = Fx * Fx;
        const T theta2 = detail::dot(phi, phi);
        const T theta = detail::sqrt_(theta2);
        const Matrix3 I = Matrix3::identity();

        // A = ∫₀¹ e^{σu} du.
        const T A = scalar_A(sigma);

        // For θ→0, (B/θ) and (C/θ²) have finite limits; compute via series.
        T coefF, coefF2;
        if (theta2 < kSmall * kSmall) {
            // exp(uφ^) ≈ I + uφ^ + u²/2 (φ^)²; integrate e^{σu}·u and ·u²/2.
            coefF = scalar_Au(sigma);             // ∫ e^{σu} u du
            coefF2 = scalar_Au2(sigma) * T{0.5};  // ½ ∫ e^{σu} u² du
        } else {
            const T denom = sigma * sigma + theta2;
            const T es = detail::exp_(sigma);
            const T s = detail::sin_(theta);
            const T c = detail::cos_(theta);
            // B = ∫ e^{σu} sin(uθ) du ; Bc = ∫ e^{σu} cos(uθ) du.
            const T B = (es * (sigma * s - theta * c) + theta) / denom;
            const T Bc = (es * (sigma * c + theta * s) - sigma) / denom;
            const T C = A - Bc;  // ∫ e^{σu}(1 - cos(uθ)) du
            coefF = B / theta;
            coefF2 = C / theta2;
        }
        return I * A + Fx * coefF + Fx2 * coefF2;
    }

    // Scalar integrals over u∈[0,1] with small-σ series fallbacks.
    static T scalar_A(const T& sigma) {  // ∫ e^{σu} du
        if (detail::abs_(sigma) < kSmall) {
            return T{1} + sigma * T{0.5} + sigma * sigma * (T{1} / T{6});
        }
        return (detail::exp_(sigma) - T{1}) / sigma;
    }
    static T scalar_Au(const T& sigma) {  // ∫ e^{σu} u du = (e^σ(σ-1)+1)/σ²
        if (detail::abs_(sigma) < kSmall) {
            return T{0.5} + sigma * (T{1} / T{3}) + sigma * sigma * (T{1} / T{8});
        }
        const T es = detail::exp_(sigma);
        return (es * (sigma - T{1}) + T{1}) / (sigma * sigma);
    }
    static T scalar_Au2(const T& sigma) {  // ∫ e^{σu} u² du
        if (detail::abs_(sigma) < kSmall) {
            return T{1} / T{3} + sigma * T{0.25} + sigma * sigma * (T{1} / T{10});
        }
        const T es = detail::exp_(sigma);
        const T s2 = sigma * sigma;
        // = (e^σ(σ² - 2σ + 2) - 2)/σ³
        return (es * (s2 - T{2} * sigma + T{2}) - T{2}) / (s2 * sigma);
    }

    SO3<T> R_{};
    Vector3 t_{};
    T s_ = T{1};
};

}  // namespace branes::math::lie

#endif  // BRANES_MATH_LIE_SIM3_HPP
