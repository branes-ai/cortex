// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/propagator.hpp — IMU mean + covariance propagation for
// the MSCKF state. Clean-room from the standard world-centric error-state
// EKF (Trawny & Roumeliotis, "Indirect Kalman Filter"; Forster et al.).
//
// Mean: discrete strapdown integration. Covariance: P ← F P Fᵀ + Q, with
// F = I + A·dt the first-order error-state transition on the IMU block
// (clones are static during propagation) and Q the discrete process-noise
// injection. Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_PROPAGATOR_HPP
#define BRANES_SDK_MSCKF_PROPAGATOR_HPP

#include <branes/sdk/msckf/state.hpp>

namespace branes::sdk::msckf {

/// Continuous-time IMU noise densities.
template <math::Scalar T>
struct ImuNoise {
    T gyro = T{1} / T{1000};        ///< σ_g  (rad/s/√Hz)
    T accel = T{1} / T{100};        ///< σ_a  (m/s²/√Hz)
    T gyro_bias = T{1} / T{10000};  ///< σ_bg random walk
    T accel_bias = T{1} / T{1000};  ///< σ_ba random walk
};

template <math::Scalar T>
class Propagator {
public:
    using Vec3 = typename State<T>::Vec3;
    using SO3 = typename State<T>::SO3;
    using Mat3 = math::lie::detail::Mat<T, 3, 3>;

    explicit Propagator(const ImuNoise<T>& noise = {}, const Vec3& gravity = {{T{0}, T{0}, T{-9.81}}})
        : noise_(noise), gravity_(gravity) {}

    /// Propagate one IMU sample (gyro, accel) over `dt`. Ignores
    /// non-positive dt.
    void propagate(State<T>& s, const Vec3& gyro, const Vec3& accel, T dt) {
        if (!(dt > T{0}))
            return;
        const Vec3 w = gyro - s.bg;
        const Vec3 a = accel - s.ba;
        const Mat3 R = s.R.matrix();

        // ── Covariance: build F = I + A·dt on the 15×15 IMU block ──────
        const std::size_t d = s.dim();
        DynMat<T> F = DynMat<T>::identity(d);
        const Mat3 negR_dt = R * (-dt);
        const Mat3 negRahat_dt = (R * math::lie::detail::hat(a)) * (-dt);
        // δθ̇ = −R δbg  ⇒  F[θ, bg] = −R dt
        place3(F, State<T>::kTheta, State<T>::kBg, negR_dt);
        // δṗ = δv       ⇒  F[p, v] = I·dt
        for (std::size_t i = 0; i < 3; ++i)
            F(State<T>::kPos + i, State<T>::kVel + i) += dt;
        // δv̇ = −R[a]ₓ δθ − R δba
        place3(F, State<T>::kVel, State<T>::kTheta, negRahat_dt);
        place3(F, State<T>::kVel, State<T>::kBa, negR_dt);

        DynMat<T> FP = mul(F, s.P);
        DynMat<T> P = mul(FP, transpose(F));
        // Discrete process noise on the IMU block (diagonal injection).
        add_diag(P, State<T>::kTheta, noise_.gyro * noise_.gyro * dt);
        add_diag(P, State<T>::kVel, noise_.accel * noise_.accel * dt);
        add_diag(P, State<T>::kBg, noise_.gyro_bias * noise_.gyro_bias * dt);
        add_diag(P, State<T>::kBa, noise_.accel_bias * noise_.accel_bias * dt);
        symmetrize(P);
        s.P = std::move(P);

        // ── Mean: strapdown integration ────────────────────────────────
        const Vec3 a_world = R * a + gravity_;
        s.p = s.p + s.v * dt + a_world * (T{0.5} * dt * dt);
        s.v = s.v + a_world * dt;
        s.R = s.R * SO3::exp(w * dt);
        s.timestamp += dt;
    }

private:
    static void place3(DynMat<T>& M, std::size_t r0, std::size_t c0, const Mat3& B) {
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                M(r0 + i, c0 + j) += B(i, j);
    }
    static void add_diag(DynMat<T>& M, std::size_t off, T val) {
        for (std::size_t i = 0; i < 3; ++i)
            M(off + i, off + i) += val;
    }

    ImuNoise<T> noise_;
    Vec3 gravity_;
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_PROPAGATOR_HPP
