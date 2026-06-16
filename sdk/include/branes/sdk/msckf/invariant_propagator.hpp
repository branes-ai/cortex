// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/invariant_propagator.hpp — the IMU mean + covariance
// propagation for the Right-Invariant EKF (R-IEKF) backend (issue #347, Phase B).
// The mean lives on SE₂(3) (extended pose, R/v/p) plus the body-frame biases.
//
// Clean-room implementation from the R-IEKF paper (Barrau & Bonnabel, 2017)
// and the "Imperfect IEKF" bias coupling (van Goor et al., 2020).
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_INVARIANT_PROPAGATOR_HPP
#define BRANES_SDK_MSCKF_INVARIANT_PROPAGATOR_HPP

#include <branes/math/lie/se23.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/propagator.hpp>  // ImuNoise, NoiseTerm

#include <array>
#include <cstddef>
#include <span>

namespace branes::sdk::msckf {

/// The R-IEKF inertial state: the extended pose on SE₂(3) plus the IMU biases.
template <math::Scalar T>
struct InvariantNavState {
    using SE23 = math::lie::SE23<T>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    SE23 X{};   ///< (R = world←imu, v = world velocity, p = world position)
    Vec3 bg{};  ///< gyro bias
    Vec3 ba{};  ///< accel bias
    double timestamp = 0.0;

    static constexpr std::size_t kTheta = 0;  // left-invariant error-state offsets
    static constexpr std::size_t kVel = 3;
    static constexpr std::size_t kPos = 6;
    static constexpr std::size_t kBg = 9;
    static constexpr std::size_t kBa = 12;
    static constexpr std::size_t kDim = 15;
};

/// Propagates an `InvariantNavState` and its left-invariant error covariance.
template <math::Scalar T>
class InvariantPropagator {
public:
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using SO3 = math::lie::SO3<T>;
    using Mat3 = math::lie::detail::Mat<T, 3, 3>;
    using SE23 = math::lie::SE23<T>;
    using State = InvariantNavState<T>;

    explicit InvariantPropagator(const ImuNoise<T>& noise = {}, const Vec3& gravity = {{T{0}, T{0}, T{-981} / T{100}}})
        : noise_(noise), gravity_(gravity) {}

    /// The 15×15 left-invariant error-transition Φ at the extended pose (R̂, v̂, p̂).
    /// Built as the discrete Φ = I + A·dt + ½(A·dt)² from the continuous left-invariant
    /// generator A, so it is CONSISTENT with the exact SE₂(3) mean strapdown (a plain
    /// Euler I + A·dt is not — issue #366).
    ///
    /// The continuous generator (ordering [δθ; δv; δp; δbg; δba]):
    ///   δθ̇ = −R̂ δbg
    ///   δv̇ = [g]× δθ − [v̂]× R̂ δbg − R̂ δba
    ///   δṗ = δv        − [p̂]× R̂ δbg
    /// The (θ,v,p) sub-block is STATE-INDEPENDENT (only the constant [g]×); the gyro-bias
    /// column carries the world-frame lever arms −[v̂]× R̂ and −[p̂]× R̂ — the standard
    /// left-invariant ("imperfect IEKF") bias coupling. Those lever-arm terms were the
    /// #366 divergence: omitting them left the velocity↔attitude observability chain
    /// inconsistent, so the camera update injected error through the bias cross-covariance
    /// (RMS pos err 162.9 m → 3.44 m once restored). Two discretization details matter too:
    /// the ½(A·dt)² term carries the only δθ → δv → δρ path (roll/pitch observability),
    /// and A is nilpotent on the nav block (A³ = 0 there) so 2nd order is exact for it.
    ///
    /// Public so the observability test can confirm Φ·N = N at any state — preserved
    /// because every bias term multiplies a bias perturbation, which is zero on all four
    /// gauge directions (global translation + gravity-yaw), and [g]×·ẑ = 0.
    [[nodiscard]] DynMat<T> phi(const SO3& R, const Vec3& v, const Vec3& p, T dt) const {
        const Mat3 R_m = R.matrix();
        const Mat3 gx = math::lie::detail::hat(gravity_);
        const Mat3 vx = math::lie::detail::hat(v);
        const Mat3 px = math::lie::detail::hat(p);
        // Continuous generator A (no dt yet).
        DynMat<T> A(State::kDim, State::kDim);
        place3(A, State::kTheta, State::kBg, R_m * T{-1});       // δθ̇ ← −R̂ δbg
        place3(A, State::kVel, State::kTheta, gx);               // δv̇ ← [g]× δθ
        place3(A, State::kVel, State::kBg, (vx * R_m) * T{-1});  // δv̇ ← −[v̂]× R̂ δbg
        place3(A, State::kVel, State::kBa, R_m * T{-1});         // δv̇ ← −R̂ δba
        for (std::size_t i = 0; i < 3; ++i)
            A(State::kPos + i, State::kVel + i) += T{1};         // δṗ ← δv
        place3(A, State::kPos, State::kBg, (px * R_m) * T{-1});  // δṗ ← −[p̂]× R̂ δbg

        // Φ = I + A·dt + ½(A·dt)².
        const std::size_t d = State::kDim;
        DynMat<T> F = DynMat<T>::identity(d);
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < d; ++j)
                F(i, j) += A(i, j) * dt;
        const T half_dt2 = dt * dt / T{2};
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < d; ++j) {
                T a2 = T{0};
                for (std::size_t k = 0; k < d; ++k)
                    a2 += A(i, k) * A(k, j);
                F(i, j) += a2 * half_dt2;
            }
        return F;
    }

    /// Propagate the mean (SE₂(3) strapdown) and the left-invariant covariance
    /// P ← Φ P Φᵀ + Q over one IMU sample. Ignores non-positive dt.
    template <class Cov>
    void propagate(State& s, Cov& cov, const Vec3& gyro, const Vec3& accel, T dt) const {
        if (!(dt > T{0}))
            return;
        const Vec3 w = gyro - s.bg;
        const Vec3 a = accel - s.ba;
        const SO3& R = s.X.rotation();

        // Covariance in the left-invariant error coordinates.
        const DynMat<T> F = phi(R, s.X.velocity(), s.X.position(), dt);
        std::array<NoiseTerm<T>, 12> q;
        push_diag(q, 0, State::kTheta, noise_.gyro * noise_.gyro * dt);
        push_diag(q, 3, State::kVel, noise_.accel * noise_.accel * dt);
        push_diag(q, 6, State::kBg, noise_.gyro_bias * noise_.gyro_bias * dt);
        push_diag(q, 9, State::kBa, noise_.accel_bias * noise_.accel_bias * dt);
        cov.predict(F, std::span<const NoiseTerm<T>>{q});
        propagate_mean(s, gyro, accel, dt);
    }

    /// Propagate ONLY the SE₂(3) mean (no covariance) — the same strapdown
    /// integration as the body-frame filter (identical physics). Exposed so a
    /// backend that carries a joint covariance can advance the mean without
    /// touching a nav-only covariance. Ignores non-positive dt.
    void propagate_mean(State& s, const Vec3& gyro, const Vec3& accel, T dt) const {
        if (!(dt > T{0}))
            return;
        const Vec3 w = gyro - s.bg;
        const Vec3 a = accel - s.ba;
        const SO3& R = s.X.rotation();
        const Vec3 v = s.X.velocity();
        const Vec3 p = s.X.position();
        const Vec3 a_world = R * a + gravity_;
        const Vec3 p_next{{p[0] + v[0] * dt + a_world[0] * (T{1} / T{2} * dt * dt),
                           p[1] + v[1] * dt + a_world[1] * (T{1} / T{2} * dt * dt),
                           p[2] + v[2] * dt + a_world[2] * (T{1} / T{2} * dt * dt)}};
        const Vec3 v_next{{v[0] + a_world[0] * dt, v[1] + a_world[1] * dt, v[2] + a_world[2] * dt}};
        s.X = SE23(R * SO3::exp(w * dt), v_next, p_next);
    }

private:
    static void place3(DynMat<T>& M, std::size_t r0, std::size_t c0, const Mat3& B) {
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                M(r0 + i, c0 + j) += B(i, j);
    }

    static void push_diag(std::array<NoiseTerm<T>, 12>& q, std::size_t at, std::size_t off, T val) {
        for (std::size_t i = 0; i < 3; ++i)
            q[at + i] = NoiseTerm<T>{off + i, val};
    }

    ImuNoise<T> noise_;
    Vec3 gravity_;
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_INVARIANT_PROPAGATOR_HPP
