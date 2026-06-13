// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/invariant_propagator.hpp — the IMU propagation for the
// Right-Invariant EKF (R-IEKF) backend (issue #347, Phase B). The mean lives on
// SE₂(3) (extended pose, R/v/p) plus the IMU biases; the covariance is carried in
// the RIGHT-INVARIANT error state [δθ; δv; δp; δbg; δba].
//
// Why this exists: the shipped body-frame propagator (propagator.hpp) couples
// velocity to attitude through −R̂[a]×·dt — dependent on the rotation ESTIMATE —
// which is the propagation source of the #212 yaw observability leak (no FEJ
// variant could freeze it; both diverged). In the right-invariant error the
// specific force cancels and only GRAVITY survives, so the nav block of the
// error-transition Φ becomes the constant [g]×·dt and the gyro self-term −[ω]×
// drops out: the nav Φ depends only on (g, dt), and the unobservable yaw/position
// gauge is preserved through propagation BY CONSTRUCTION. The bias columns retain
// an R̂ term — the standard "imperfect IEKF" coupling (biases are body-frame).
// Phase A measured both halves (flat yaw leak + state-independent nav Φ).
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_INVARIANT_PROPAGATOR_HPP
#define BRANES_SDK_MSCKF_INVARIANT_PROPAGATOR_HPP

#include <branes/math/lie/se23.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/propagator.hpp>  // ImuNoise, NoiseTerm

#include <array>
#include <cstddef>

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

    static constexpr std::size_t kTheta = 0;  // right-invariant error-state offsets
    static constexpr std::size_t kVel = 3;
    static constexpr std::size_t kPos = 6;
    static constexpr std::size_t kBg = 9;
    static constexpr std::size_t kBa = 12;
    static constexpr std::size_t kDim = 15;
};

/// Propagates an `InvariantNavState` and its right-invariant error covariance.
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

    /// The 15×15 right-invariant error-transition Φ at rotation `R̂`. The nav block
    /// (θ,v,p) is STATE-INDEPENDENT — the only coupling is the constant [g]×; the
    /// bias columns carry R̂ (imperfect IEKF). Public so the observability test can
    /// confirm it preserves the gauge null space at any state.
    [[nodiscard]] DynMat<T> phi(const SO3& R, T dt) const {
        DynMat<T> F = DynMat<T>::identity(State::kDim);
        const Mat3 R_m = R.matrix();
        // δθ̇ = −R̂ δbg            (no [ω]× self-term)
        place3(F, State::kTheta, State::kBg, R_m * (-dt));
        // δv̇ = [g]× δθ − R̂ δba   ([g]× constant — state-independent)
        place3(F, State::kVel, State::kTheta, math::lie::detail::hat(gravity_) * dt);
        place3(F, State::kVel, State::kBa, R_m * (-dt));
        // δṗ = δv
        for (std::size_t i = 0; i < 3; ++i)
            F(State::kPos + i, State::kVel + i) += dt;
        return F;
    }

    /// Propagate the mean (SE₂(3) strapdown) and the right-invariant covariance
    /// P ← Φ P Φᵀ + Q over one IMU sample. Ignores non-positive dt.
    template <class Cov>
    void propagate(State& s, Cov& cov, const Vec3& gyro, const Vec3& accel, T dt) const {
        if (!(dt > T{0}))
            return;
        const Vec3 w = gyro - s.bg;
        const Vec3 a = accel - s.ba;
        const SO3& R = s.X.rotation();

        // Covariance in the right-invariant error coordinates.
        const DynMat<T> F = phi(R, dt);
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
        s.timestamp += dt;
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
