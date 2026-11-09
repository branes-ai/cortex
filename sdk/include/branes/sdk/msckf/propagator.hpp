// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/propagator.hpp — IMU mean + covariance propagation for
// the MSCKF state. Clean-room from the standard world-centric error-state
// EKF (Trawny & Roumeliotis, "Indirect Kalman Filter"; Forster et al.).
//
// The input is an IMU interval: the samples at both of its ends (#616). A
// sample is the rate at its timestamp, so the input over [t_{k−1}, t_k] is the
// line between them; holding either sample over the interval integrates the
// IMU half a sample early or late against the camera (a 2.5 ms skew at 200 Hz,
// docs/first-principles/sample-alignment.mdx).
//
// Mean: RK4 on the interpolated input (OpenVINS, Geneva et al. 2020). With ω
// linear in time the attitude has a closed form to fourth order, the rate's
// integral plus the coning term (the second Magnus term); velocity and
// position are then quadratures, which RK4 evaluates at the start, middle and
// end of the interval. Covariance: P ← F P Fᵀ + Q, with F the exact Jacobian
// of that mean on the IMU block (clones are static during propagation), so Φ
// preserves the unobservable directions the mean does, and Q the discrete
// process-noise injection. Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_PROPAGATOR_HPP
#define BRANES_SDK_MSCKF_PROPAGATOR_HPP

#include <branes/sdk/msckf/state.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>

namespace branes::sdk::msckf {

/// Continuous-time IMU noise densities.
template <math::Scalar T>
struct ImuNoise {
    T gyro = T{1} / T{1000};        ///< σ_g  (rad/s/√Hz)
    T accel = T{1} / T{100};        ///< σ_a  (m/s²/√Hz)
    T gyro_bias = T{1} / T{10000};  ///< σ_bg random walk
    T accel_bias = T{1} / T{1000};  ///< σ_ba random walk
};

/// One IMU interval [t_{k−1}, t_k]: the samples at both of its ends and its
/// length (#616). S2 interpolates the input linearly between the two samples.
template <math::Scalar T>
struct ImuInterval {
    using Vec3 = math::lie::detail::Vec<T, 3>;
    Vec3 gyro0{}, accel0{};  ///< the sample at the start of the interval
    Vec3 gyro1{}, accel1{};  ///< the sample at the end of the interval
    T dt{0};

    /// A constant input over `dt`: both ends the same sample.
    [[nodiscard]] static ImuInterval held(const Vec3& gyro, const Vec3& accel, T dt) {
        return ImuInterval{gyro, accel, gyro, accel, dt};
    }
    /// The interval's mean input, (start + end) / 2.
    [[nodiscard]] Vec3 mean_gyro() const {
        return (gyro0 + gyro1) * (T{1} / T{2});
    }
    [[nodiscard]] Vec3 mean_accel() const {
        return (accel0 + accel1) * (T{1} / T{2});
    }
};

/// The line through two samples (t_a, ω_a) and (t_b, ω_b) evaluated at t,
/// clamped to [t_a, t_b]; ω_b when the two times coincide. Used to start an
/// interval at a time between samples (an image time, a seed time).
template <math::Scalar T>
[[nodiscard]] math::lie::detail::Vec<T, 3> interpolate_sample(double t_a,
                                                              const math::lie::detail::Vec<T, 3>& w_a,
                                                              double t_b,
                                                              const math::lie::detail::Vec<T, 3>& w_b,
                                                              double t) {
    if (!(t_b > t_a))
        return w_b;
    const double u = std::clamp((t - t_a) / (t_b - t_a), 0.0, 1.0);
    return w_a + (w_b - w_a) * static_cast<T>(u);
}

/// One interval's increments in the frame at its start, and their Jacobians in
/// the biases. The RK4 mean (rk4_strapdown) is, from (R, v, p),
///   R′ = R·ΔR,  v′ = v + gΔt + R·Δv,  p′ = p + vΔt + ½gΔt² + R·Δp,
/// with ΔR, Δv, Δp depending only on the bias-corrected input. This is the
/// preintegration form: a rotation of the world about gravity, or a
/// translation, commutes with it, which is what makes its exact Jacobian
/// preserve the unobservable directions (Propagator::transition).
template <math::Scalar T>
struct ImuIncrement {
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Mat3 = math::lie::detail::Mat<T, 3, 3>;
    math::lie::SO3<T> dR{};
    Vec3 dv{}, dp{};
    Mat3 dR_dbg{};            ///< ΔR(b + δb) ≈ ΔR·Exp(dR_dbg·δbg)
    Mat3 dv_dbg{}, dv_dba{};  ///< ∂Δv/∂b
    Mat3 dp_dbg{}, dp_dba{};  ///< ∂Δp/∂b
};

/// The increments of one interval by RK4 on the input interpolated between its
/// two samples. With ω(τ) = ω₀ + (ω₁ − ω₀)·τ/Δt (bias removed), the attitude is
///   ΔR(τ) = Exp(θ(τ)),  θ(τ) = ω₀τ + (ω₁ − ω₀)τ²/(2Δt) + τ³/(12Δt)·ω₀ × (ω₁ − ω₀),
/// the rate's integral plus the coning term (the second Magnus term, exact to
/// fourth order for a rate linear in time). Velocity and position are then
/// quadratures of ΔR(τ)·a(τ), which RK4 evaluates at τ = 0, Δt/2, Δt:
///   Δv = Δt/6·(a₀ + 4ΔR½·ā + ΔR₁·a₁),  Δp = Δt²/6·(a₀ + 2ΔR½·ā),  ā = (a₀ + a₁)/2.
/// A held interval (both ends the same sample) is RK4 on a constant input.
template <math::Scalar T>
[[nodiscard]] ImuIncrement<T> rk4_increment(const math::lie::detail::Vec<T, 3>& bg,
                                            const math::lie::detail::Vec<T, 3>& ba,
                                            const ImuInterval<T>& in) {
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Mat3 = math::lie::detail::Mat<T, 3, 3>;
    using SO3 = math::lie::SO3<T>;
    using math::lie::detail::hat;
    const T h = in.dt;
    const T half = T{1} / T{2};
    const Vec3 w0 = in.gyro0 - bg, w1 = in.gyro1 - bg;
    const Vec3 a0 = in.accel0 - ba, a1 = in.accel1 - ba;
    const Vec3 am = (a0 + a1) * half;
    const Vec3 dw = w1 - w0;
    const Vec3 coning = math::lie::detail::cross(w0, dw);
    // θ at the fraction u of the interval, and ∂θ/∂bg there: ω₀ carries −δbg,
    // ω₁ − ω₀ does not, and ∂(ω₀ × Δω)/∂bg = [Δω]×.
    const auto theta = [&](T u) {
        return w0 * (h * u) + dw * (h * u * u * half) + coning * (h * h * u * u * u / T{12});
    };
    const auto dtheta_dbg = [&](T u) { return Mat3::identity() * (-h * u) + hat(dw) * (h * h * u * u * u / T{12}); };
    const Vec3 th_m = theta(half), th_1 = theta(T{1});
    const SO3 E_m = SO3::exp(th_m), E_1 = SO3::exp(th_1);
    const Mat3 Em = E_m.matrix(), E1 = E_1.matrix();
    // ∂(ΔR(u)·a)/∂bg = −ΔR(u)·[a]×·J_r(θ(u))·∂θ(u)/∂bg.
    const Mat3 Gm = Em * hat(am) * SO3::right_jacobian(th_m) * dtheta_dbg(half);
    const Mat3 G1 = E1 * hat(a1) * SO3::right_jacobian(th_1) * dtheta_dbg(T{1});
    const T h6 = h / T{6}, hh6 = h * h / T{6};

    ImuIncrement<T> out;
    out.dR = E_1;
    out.dv = (a0 + Em * am * T{4} + E1 * a1) * h6;
    out.dp = (a0 + Em * am * T{2}) * hh6;
    out.dR_dbg = SO3::right_jacobian(th_1) * dtheta_dbg(T{1});
    out.dv_dbg = (Gm * T{4} + G1) * (-h6);
    out.dv_dba = (Mat3::identity() + Em * T{4} + E1) * (-h6);
    out.dp_dbg = Gm * (T{-2} * hh6);
    out.dp_dba = (Mat3::identity() + Em * T{2}) * (-hh6);
    return out;
}

/// The navigation mean at the end of an interval.
template <math::Scalar T>
struct StrapdownMean {
    math::lie::SO3<T> R{};
    math::lie::detail::Vec<T, 3> v{}, p{};
};

/// RK4 strapdown over one interval, from (R, v, p) with the biases `bg`, `ba`:
/// R′ = R·ΔR, v′ = v + gΔt + R·Δv, p′ = p + vΔt + ½gΔt² + R·Δp (rk4_increment).
template <math::Scalar T>
[[nodiscard]] StrapdownMean<T> rk4_strapdown(const math::lie::SO3<T>& R,
                                             const math::lie::detail::Vec<T, 3>& v,
                                             const math::lie::detail::Vec<T, 3>& p,
                                             const math::lie::detail::Vec<T, 3>& bg,
                                             const math::lie::detail::Vec<T, 3>& ba,
                                             const ImuInterval<T>& in,
                                             const math::lie::detail::Vec<T, 3>& gravity) {
    const ImuIncrement<T> inc = rk4_increment(bg, ba, in);
    const T h = in.dt;
    StrapdownMean<T> out;
    out.R = R * inc.dR;
    out.v = v + gravity * h + R * inc.dv;
    out.p = p + v * h + gravity * (h * h / T{2}) + R * inc.dp;
    return out;
}

template <math::Scalar T>
class Propagator {
public:
    using Vec3 = typename State<T>::Vec3;
    using SO3 = typename State<T>::SO3;
    using Mat3 = math::lie::detail::Mat<T, 3, 3>;

    explicit Propagator(const ImuNoise<T>& noise = {}, const Vec3& gravity = {{T{0}, T{0}, T{-9.81}}})
        : noise_(noise), gravity_(gravity) {}

    /// The error-state transition of one interval and the diagonal process-noise
    /// injection Q_d on the IMU block. Exposed so a stage bench can check
    /// P' = Φ P Φᵀ + Q_d and the unobservable subspace against exactly the Φ the
    /// filter uses. Precondition: dt > 0.
    struct Transition {
        DynMat<T> F;
        std::array<NoiseTerm<T>, 12> q;
    };

    /// Φ over an interval: the exact Jacobian of the RK4 mean (rk4_strapdown)
    /// in the body-frame right-perturbation error (R = R̂·Exp(δθ)), clones static:
    ///   δθ′ = ΔRᵀ δθ + (∂ΔR/∂bg) δbg
    ///   δv′ = δv − R[Δv]× δθ + R (∂Δv/∂b) δb
    ///   δp′ = δp + Δt δv − R[Δp]× δθ + R (∂Δp/∂b) δb
    /// (#616). It is consistent with the mean it propagates: the θ, v, p blocks
    /// are exact, and the unobservable basis has no bias components, so Φ maps
    /// the global translation and the yaw about gravity onto themselves to
    /// roundoff. The pre-#616 I + A·dt (first_order_transition) was consistent
    /// only with the one-sample Euler mean it was linearized against (#471).
    template <class Cov>
    [[nodiscard]] Transition transition(const State<T, Cov>& s, const ImuInterval<T>& in) const {
        using math::lie::detail::hat;
        const ImuIncrement<T> inc = rk4_increment(s.bg, s.ba, in);
        const Mat3 R = s.R.matrix();
        const std::size_t d = s.dim();
        Transition tr{DynMat<T>::identity(d), {}};
        DynMat<T>& F = tr.F;
        set3(F, State<T>::kTheta, State<T>::kTheta, math::lie::detail::transpose(inc.dR.matrix()));
        set3(F, State<T>::kTheta, State<T>::kBg, inc.dR_dbg);
        set3(F, State<T>::kVel, State<T>::kTheta, (R * hat(inc.dv)) * T{-1});
        set3(F, State<T>::kVel, State<T>::kBg, R * inc.dv_dbg);
        set3(F, State<T>::kVel, State<T>::kBa, R * inc.dv_dba);
        set3(F, State<T>::kPos, State<T>::kTheta, (R * hat(inc.dp)) * T{-1});
        for (std::size_t i = 0; i < 3; ++i)
            F(State<T>::kPos + i, State<T>::kVel + i) = in.dt;
        set3(F, State<T>::kPos, State<T>::kBg, R * inc.dp_dbg);
        set3(F, State<T>::kPos, State<T>::kBa, R * inc.dp_dba);
        push_noise(tr, in.dt);
        return tr;
    }

    /// Φ over `dt` on one sample held constant (ImuInterval::held).
    template <class Cov>
    [[nodiscard]] Transition transition(const State<T, Cov>& s, const Vec3& gyro, const Vec3& accel, T dt) const {
        return transition(s, ImuInterval<T>::held(gyro, accel, dt));
    }

    /// The pre-#616 transition, Φ = I + A·dt from the continuous error dynamics
    /// at one held sample. Not used by the filter; kept so the S2 bench can
    /// compare its yaw leak against the shipped Φ (#471, #616).
    template <class Cov>
    [[nodiscard]] Transition
    first_order_transition(const State<T, Cov>& s, const Vec3& gyro, const Vec3& accel, T dt) const {
        const Vec3 w = gyro - s.bg;
        const Vec3 a = accel - s.ba;
        const Mat3 R = s.R.matrix();
        const std::size_t d = s.dim();
        Transition tr{DynMat<T>::identity(d), {}};
        DynMat<T>& F = tr.F;
        const Mat3 negR_dt = R * (-dt);
        const Mat3 negRahat_dt = (R * math::lie::detail::hat(a)) * (-dt);
        const Mat3 negwhat_dt = math::lie::detail::hat(w) * (-dt);
        // Body-frame right-perturbation error model (R ← R·Exp(δθ)):
        //   δθ̇ = −[ω]ₓ δθ − δbg  ⇒  F[θ,θ] += −[ω]ₓ dt, F[θ,bg] = −I dt
        place3(F, State<T>::kTheta, State<T>::kTheta, negwhat_dt);
        for (std::size_t i = 0; i < 3; ++i)
            F(State<T>::kTheta + i, State<T>::kBg + i) += -dt;
        // δṗ = δv       ⇒  F[p, v] = I·dt
        for (std::size_t i = 0; i < 3; ++i)
            F(State<T>::kPos + i, State<T>::kVel + i) += dt;
        // δv̇ = −R[a]ₓ δθ − R δba
        place3(F, State<T>::kVel, State<T>::kTheta, negRahat_dt);
        place3(F, State<T>::kVel, State<T>::kBa, negR_dt);
        push_noise(tr, dt);
        return tr;
    }

    /// Propagate over `dt` on one sample held constant (ImuInterval::held):
    /// a constant input, as in a bench or a hold up to an image time. Generic
    /// over the covariance representation: builds the error-state transition F
    /// and the diagonal process-noise Q (see `transition`), then hands both to
    /// the covariance policy (which applies F P Fᵀ + Q in whichever
    /// representation it carries).
    template <class Cov>
    void propagate(State<T, Cov>& s, const Vec3& gyro, const Vec3& accel, T dt) const {
        propagate(s, ImuInterval<T>::held(gyro, accel, dt));
    }

    /// Propagate over one IMU interval (#616): the mean by RK4 on the input
    /// interpolated between the interval's two samples (rk4_strapdown), the
    /// covariance by that mean's exact Jacobian Φ and the diagonal Q_d
    /// (transition). Ignores a non-positive dt.
    template <class Cov>
    void propagate(State<T, Cov>& s, const ImuInterval<T>& in) const {
        if (!(in.dt > T{0}))
            return;
        const Transition tr = transition(s, in);
        s.cov.predict(tr.F, std::span<const NoiseTerm<T>>{tr.q});
        const StrapdownMean<T> m = rk4_strapdown(s.R, s.v, s.p, s.bg, s.ba, in, gravity_);
        s.R = m.R;
        s.v = m.v;
        s.p = m.p;
        s.timestamp += static_cast<double>(in.dt);  // timestamps stay double; T may be a posit
    }

    [[nodiscard]] const ImuNoise<T>& noise() const noexcept {
        return noise_;
    }
    [[nodiscard]] const Vec3& gravity() const noexcept {
        return gravity_;
    }

private:
    static void place3(DynMat<T>& M, std::size_t r0, std::size_t c0, const Mat3& B) {
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                M(r0 + i, c0 + j) += B(i, j);
    }
    static void set3(DynMat<T>& M, std::size_t r0, std::size_t c0, const Mat3& B) {
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                M(r0 + i, c0 + j) = B(i, j);
    }
    // Discrete process noise on the IMU block (diagonal injection, σ²·dt).
    void push_noise(Transition& tr, T dt) const {
        push_diag(tr.q, 0, State<T>::kTheta, noise_.gyro * noise_.gyro * dt);
        push_diag(tr.q, 3, State<T>::kVel, noise_.accel * noise_.accel * dt);
        push_diag(tr.q, 6, State<T>::kBg, noise_.gyro_bias * noise_.gyro_bias * dt);
        push_diag(tr.q, 9, State<T>::kBa, noise_.accel_bias * noise_.accel_bias * dt);
    }
    static void push_diag(std::array<NoiseTerm<T>, 12>& q, std::size_t at, std::size_t off, T val) {
        for (std::size_t i = 0; i < 3; ++i)
            q[at + i] = NoiseTerm<T>{off + i, val};
    }

    ImuNoise<T> noise_;
    Vec3 gravity_;
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_PROPAGATOR_HPP
