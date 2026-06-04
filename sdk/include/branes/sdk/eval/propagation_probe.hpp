// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/propagation_probe.hpp — the S2 (IMU propagation) probe of the
// VIO contract program (docs/arch/vio-pipeline-canonical.md).
//
// S2 advances the mean (strapdown integration) and the covariance
// (P ← Φ P Φᵀ + Q_d) between measurement updates. Its contract:
//   • the mean stays on the manifold (R ∈ SO(3));
//   • P stays symmetric PSD;
//   • Q_d carries the right structure — and this is the #212 candidate: the
//     cortex propagator injects a DIAGONAL Q (σ²·Δt on θ, v, b_g, b_a) and NO
//     position-block term, whereas the canonical discrete Q_d = B·Σ_η·Bᵀ adds a
//     position-block accel-noise term (¼σ_a²Δt³) and a v–p cross term (½σ_a²Δt²).
//     Whether that omission materially under-states the propagated covariance is
//     a MEASUREMENT, in native units (position σ in mm), not a guess.
//   • propagation does not manufacture information along the unobservable
//     global-position directions.
//
// Native units: m/s² and mm (drift), mm & mm/s & deg (covariance σ), NEES
// (dimensionless, target = dof). The probe drives the REAL Propagator for the
// cortex path (so it tests the shipped code) and a reference propagation with the
// same Φ but the canonical Q_d, isolating the Q contribution exactly.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_PROPAGATION_PROBE_HPP
#define BRANES_SDK_EVAL_PROPAGATION_PROBE_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

namespace branes::sdk::eval {

namespace pp_detail {

using msckf::DynMat;
template <math::Scalar T>
using Vec3 = math::lie::detail::Vec<T, 3>;
template <math::Scalar T>
using Mat3 = math::lie::detail::Mat<T, 3, 3>;
template <math::Scalar T>
using SO3 = math::lie::SO3<T>;

constexpr std::size_t kTheta = 0, kPos = 3, kVel = 6, kBg = 9, kBa = 12, kImu = 15;

/// Place a 3×3 block (additively) into a DynMat at (r0,c0).
template <math::Scalar T>
void place3(DynMat<T>& M, std::size_t r0, std::size_t c0, const Mat3<T>& B) {
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            M(r0 + i, c0 + j) += B(i, j);
}

/// The 15×15 error-state transition Φ = I + A·Δt, reconstructed to match
/// sdk/msckf/propagator.hpp exactly (FEJ off ⇒ linearize at the passed state).
template <math::Scalar T>
[[nodiscard]] DynMat<T> imu_F(const SO3<T>& R, const Vec3<T>& w_lin, const Vec3<T>& a_lin, T dt) {
    DynMat<T> F = DynMat<T>::identity(kImu);
    const Mat3<T> R_m = R.matrix();
    place3(F, kTheta, kTheta, math::lie::detail::hat(w_lin) * (-dt));
    for (std::size_t i = 0; i < 3; ++i)
        F(kTheta + i, kBg + i) += -dt;
    for (std::size_t i = 0; i < 3; ++i)
        F(kPos + i, kVel + i) += dt;
    place3(F, kVel, kTheta, (R_m * math::lie::detail::hat(a_lin)) * (-dt));
    place3(F, kVel, kBa, R_m * (-dt));
    return F;
}

/// The cortex DIAGONAL Q_d: σ²·Δt on θ, v, b_g, b_a — and NOTHING on position.
template <math::Scalar T>
[[nodiscard]] DynMat<T> qd_cortex(const msckf::ImuNoise<T>& n, T dt) {
    DynMat<T> Q(kImu, kImu);
    for (std::size_t i = 0; i < 3; ++i) {
        Q(kTheta + i, kTheta + i) = n.gyro * n.gyro * dt;
        Q(kVel + i, kVel + i) = n.accel * n.accel * dt;
        Q(kBg + i, kBg + i) = n.gyro_bias * n.gyro_bias * dt;
        Q(kBa + i, kBa + i) = n.accel_bias * n.accel_bias * dt;
    }
    return Q;
}

/// The canonical discrete Q_d = B·Σ_η·Bᵀ (Forster/OpenVINS), which additionally
/// carries the position-block accel-noise term and the v–p cross term that a
/// diagonal injection drops. (R drops out of the block magnitudes for isotropic
/// noise, since R·(σ²I)·Rᵀ = σ²I.)
template <math::Scalar T>
[[nodiscard]] DynMat<T> qd_canonical(const msckf::ImuNoise<T>& n, T dt) {
    DynMat<T> Q = qd_cortex(n, dt);
    const T a2 = n.accel * n.accel;
    const T pp = a2 * dt * dt * dt / T{4};  // ¼ σ_a² Δt³
    const T vp = a2 * dt * dt / T{2};       // ½ σ_a² Δt²
    for (std::size_t i = 0; i < 3; ++i) {
        Q(kPos + i, kPos + i) += pp;
        Q(kVel + i, kPos + i) += vp;
        Q(kPos + i, kVel + i) += vp;
    }
    return Q;
}

/// One step of reference covariance propagation: P ← F P Fᵀ + Q.
template <math::Scalar T>
[[nodiscard]] DynMat<T> propagate_cov(const DynMat<T>& P, const DynMat<T>& F, const DynMat<T>& Q) {
    return msckf::add(msckf::mul(msckf::mul(F, P), msckf::transpose(F)), Q);
}

/// √(trace) of a 3×3 diagonal sub-block at offset `o` — the total σ of that block.
template <math::Scalar T>
[[nodiscard]] T block_sigma(const DynMat<T>& P, std::size_t o) {
    T tr{0};
    for (std::size_t i = 0; i < 3; ++i)
        tr += P(o + i, o + i);
    using std::sqrt;
    return sqrt(tr);
}

/// Smallest eigenvalue of a symmetric matrix via cyclic Jacobi rotations — for
/// the PSD health readout. Bounded and robust at the 15×15 sizes here.
template <math::Scalar T>
[[nodiscard]] T min_eigenvalue(DynMat<T> A) {
    using std::abs;
    using std::sqrt;
    const std::size_t n = A.rows;
    for (int sweep = 0; sweep < 60; ++sweep) {
        T off = T{0};
        for (std::size_t p = 0; p < n; ++p)
            for (std::size_t q = p + 1; q < n; ++q)
                off += A(p, q) * A(p, q);
        if (off < T(1e-22))
            break;
        for (std::size_t p = 0; p < n; ++p) {
            for (std::size_t q = p + 1; q < n; ++q) {
                if (abs(A(p, q)) < T(1e-20))
                    continue;
                const T tau = (A(q, q) - A(p, p)) / (T{2} * A(p, q));
                const T t = (tau >= T{0} ? T{1} : T{-1}) / (abs(tau) + sqrt(T{1} + tau * tau));
                const T c = T{1} / sqrt(T{1} + t * t);
                const T s = t * c;
                for (std::size_t k = 0; k < n; ++k) {
                    const T akp = A(k, p), akq = A(k, q);
                    A(k, p) = c * akp - s * akq;
                    A(k, q) = s * akp + c * akq;
                }
                for (std::size_t k = 0; k < n; ++k) {
                    const T apk = A(p, k), aqk = A(q, k);
                    A(p, k) = c * apk - s * aqk;
                    A(q, k) = s * apk + c * aqk;
                }
            }
        }
    }
    T m = A(0, 0);
    for (std::size_t i = 1; i < n; ++i)
        if (A(i, i) < m)
            m = A(i, i);
    return m;
}

}  // namespace pp_detail

// ── S2.1  Q_d structure: what the diagonal injection drops ─────────────────

template <math::Scalar T>
struct QStructureResult {
    // per-block one-step σ (native): cortex vs canonical
    T theta_sigma_cortex{0}, theta_sigma_canon{0};  // rad
    T vel_sigma_cortex{0}, vel_sigma_canon{0};      // m/s
    T pos_sigma_cortex{0}, pos_sigma_canon{0};      // m  (cortex == 0)
    T vp_cross_canon{0};                            // m·(m/s) v–p cross term cortex drops
    T rel_frobenius_gap{0};                         // ‖Q_canon − Q_cortex‖_F / ‖Q_canon‖_F
};

/// Compare the cortex diagonal Q_d to the canonical Q_d at one IMU step.
template <math::Scalar T>
[[nodiscard]] QStructureResult<T> q_structure(const msckf::ImuNoise<T>& noise, T dt) {
    using namespace pp_detail;
    const auto Qc = qd_cortex(noise, dt);
    const auto Qk = qd_canonical(noise, dt);
    QStructureResult<T> r;
    r.theta_sigma_cortex = block_sigma(Qc, kTheta);
    r.theta_sigma_canon = block_sigma(Qk, kTheta);
    r.vel_sigma_cortex = block_sigma(Qc, kVel);
    r.vel_sigma_canon = block_sigma(Qk, kVel);
    r.pos_sigma_cortex = block_sigma(Qc, kPos);
    r.pos_sigma_canon = block_sigma(Qk, kPos);
    r.vp_cross_canon = Qk(kVel, kPos);
    using std::sqrt;
    T num{0}, den{0};
    for (std::size_t i = 0; i < kImu; ++i)
        for (std::size_t j = 0; j < kImu; ++j) {
            const T d = Qk(i, j) - Qc(i, j);
            num += d * d;
            den += Qk(i, j) * Qk(i, j);
        }
    r.rel_frobenius_gap = den > T{0} ? sqrt(num) / sqrt(den) : T{0};
    return r;
}

// ── S2.2  Covariance growth: cortex-Q vs canonical-Q in native σ ───────────

template <math::Scalar T>
struct CovGrowthPoint {
    T t_s{0};
    T pos_sigma_mm_cortex{0}, pos_sigma_mm_canon{0};
    T vel_sigma_mm_s_cortex{0}, vel_sigma_mm_s_canon{0};
    T att_sigma_deg_cortex{0}, att_sigma_deg_canon{0};
    T R_ortho_residual{0};  ///< ‖RᵀR − I‖ of the integrated mean (stays ~0)
    T min_eig_cortex{0};    ///< PSD health of the cortex P
};

template <math::Scalar T>
struct CovGrowthResult {
    std::vector<CovGrowthPoint<T>> curve;
    T F_validation_residual{0};  ///< ‖P_realPropagator − P_reference(cortex-Q)‖ (validates Φ reconstruction)
    T pos_underreport_pct_interframe{0};  ///< at ~0.05 s: 100·(canon−cortex)/canon
    T pos_underreport_pct_final{0};
};

/// Propagate the covariance from zero under a fixed IMU sample, comparing the
/// REAL Propagator (cortex diagonal Q) against a reference propagation with the
/// same Φ but the canonical Q_d. Reports native position/velocity/attitude σ
/// growth and the cortex under-report.
template <math::Scalar T>
[[nodiscard]] CovGrowthResult<T> cov_growth(const msckf::ImuNoise<T>& noise,
                                            const pp_detail::Vec3<T>& gyro,
                                            const pp_detail::Vec3<T>& accel,
                                            T dt = T{1} / T{200},
                                            T duration_s = T{0.5},
                                            std::size_t samples = 60) {
    using namespace pp_detail;
    using msckf::State;
    CovGrowthResult<T> out;

    State<T> s(T{0});  // P0 = 0: isolate the process-noise growth
    // The probe linearizes Φ at the current estimate (imu_F above); the
    // Propagator's default does the same on main. (If first-estimates Jacobians
    // are ever enabled by default, pass first_estimates=false here to keep Φ and
    // the reconstruction in step — the F_validation_residual check will flag it.)
    msckf::Propagator<T> prop(noise, Vec3<T>{{T{0}, T{0}, T{-9.81}}});

    DynMat<T> Pcanon(kImu, kImu);  // reference covariance with canonical Q_d
    DynMat<T> Pcref(kImu, kImu);   // reference covariance with cortex Q_d (validates Φ)

    const auto nsteps = static_cast<std::size_t>(duration_s / dt);
    const std::size_t stride = samples ? (nsteps / samples == 0 ? 1 : nsteps / samples) : 1;
    const T interframe_s = T{0.05};
    using std::abs;
    using std::sqrt;

    for (std::size_t k = 0; k < nsteps; ++k) {
        // Φ at the pre-step linearization (FEJ off ⇒ current mean).
        const DynMat<T> F = imu_F(s.R, gyro - s.bg, accel - s.ba, dt);
        Pcanon = propagate_cov(Pcanon, F, qd_canonical(noise, dt));
        Pcref = propagate_cov(Pcref, F, qd_cortex(noise, dt));

        prop.propagate(s, gyro, accel, dt);  // the real code: updates mean + cortex-Q covariance

        if (k % stride == 0 || k + 1 == nsteps) {
            const DynMat<T> Preal = s.covariance();
            // R orthonormality of the integrated mean.
            const Mat3<T> R = s.R.matrix();
            const Mat3<T> RtR = math::lie::detail::transpose(R) * R;
            T ortho{0};
            for (std::size_t i = 0; i < 3; ++i)
                for (std::size_t j = 0; j < 3; ++j) {
                    const T e = RtR(i, j) - (i == j ? T{1} : T{0});
                    ortho += e * e;
                }
            CovGrowthPoint<T> p;
            p.t_s = static_cast<T>(k + 1) * dt;
            p.pos_sigma_mm_cortex = block_sigma(Preal, kPos) * T{1000};
            p.pos_sigma_mm_canon = block_sigma(Pcanon, kPos) * T{1000};
            p.vel_sigma_mm_s_cortex = block_sigma(Preal, kVel) * T{1000};
            p.vel_sigma_mm_s_canon = block_sigma(Pcanon, kVel) * T{1000};
            p.att_sigma_deg_cortex = block_sigma(Preal, kTheta) * (T{180} / T{3.14159265358979323846});
            p.att_sigma_deg_canon = block_sigma(Pcanon, kTheta) * (T{180} / T{3.14159265358979323846});
            p.R_ortho_residual = sqrt(ortho);
            p.min_eig_cortex = min_eigenvalue(Preal);
            out.curve.push_back(p);
        }
    }
    // Φ-reconstruction validation: the reference cortex-Q path must match the
    // real Propagator's covariance to working precision.
    {
        const DynMat<T> Preal = s.covariance();
        T num{0}, den{0};
        for (std::size_t i = 0; i < kImu; ++i)
            for (std::size_t j = 0; j < kImu; ++j) {
                const T d = Preal(i, j) - Pcref(i, j);
                num += d * d;
                den += Preal(i, j) * Preal(i, j);
            }
        out.F_validation_residual = den > T{0} ? sqrt(num) / sqrt(den) : sqrt(num);
    }
    if (!out.curve.empty()) {
        auto pct = [](T cortex, T canon) { return canon > T{0} ? T{100} * (canon - cortex) / canon : T{0}; };
        // interframe point
        const CovGrowthPoint<T>* nearest = &out.curve.front();
        for (const auto& p : out.curve)
            if (abs(p.t_s - interframe_s) < abs(nearest->t_s - interframe_s))
                nearest = &p;
        out.pos_underreport_pct_interframe = pct(nearest->pos_sigma_mm_cortex, nearest->pos_sigma_mm_canon);
        out.pos_underreport_pct_final = pct(out.curve.back().pos_sigma_mm_cortex, out.curve.back().pos_sigma_mm_canon);
    }
    return out;
}

// ── S2.3  Mean GT-injection: integration accuracy vs a closed-form trajectory ─

template <math::Scalar T>
struct GtInjectionPoint {
    T dt_s{0};
    T pos_error_mm{0};
    T att_error_deg{0};
    T vel_error_mm_s{0};
};

/// Integrate a genuinely coupled trajectory — constant BODY-frame angular rate
/// ω_b and constant BODY-frame specific force f_b, so the world acceleration
/// R(t)·f_b + g rotates within every step — through the REAL Propagator at the
/// test `dt` and at a 32× finer reference dt. The coarse-vs-fine difference is
/// the integrator's discretization error (the scheme holds the measurement
/// constant over each step, so a rotating trajectory is NOT integrated exactly);
/// sweeping dt reveals the convergence order.
template <math::Scalar T>
[[nodiscard]] GtInjectionPoint<T> gt_injection(T dt, T duration_s = T{2}) {
    using namespace pp_detail;
    using msckf::State;
    const Vec3<T> g_world{{T{0}, T{0}, T{-9.81}}};
    const Vec3<T> w_b{{T{0.3}, T{-0.2}, T{0.5}}};  // tumbling body rate
    const Vec3<T> f_b{{T{0.6}, T{0.4}, T{9.2}}};   // body-frame specific force

    auto integrate = [&](T step) {
        State<T> s(T{0});
        msckf::Propagator<T> prop(msckf::ImuNoise<T>{T{0}, T{0}, T{0}, T{0}}, g_world);
        s.R = SO3<T>{};
        const auto n = static_cast<std::size_t>(duration_s / step);
        for (std::size_t k = 0; k < n; ++k)
            prop.propagate(s, w_b, f_b, step);
        return s;
    };
    const State<T> coarse = integrate(dt);
    const State<T> fine = integrate(dt / T{32});  // high-accuracy reference

    using std::sqrt;
    auto nrm = [](const Vec3<T>& a) { return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); };
    GtInjectionPoint<T> r;
    r.dt_s = dt;
    r.pos_error_mm = nrm(coarse.p - fine.p) * T{1000};
    r.vel_error_mm_s = nrm(coarse.v - fine.v) * T{1000};
    r.att_error_deg = nrm((fine.R.inverse() * coarse.R).log()) * (T{180} / T{3.14159265358979323846});
    return r;
}

/// dt sweep of gt_injection — the integration-order sensitivity.
template <math::Scalar T>
[[nodiscard]] std::vector<GtInjectionPoint<T>> gt_injection_dt_sweep() {
    std::vector<GtInjectionPoint<T>> out;
    for (const T rate : {T{50}, T{100}, T{200}, T{400}, T{800}})
        out.push_back(gt_injection<T>(T{1} / rate));
    return out;
}

// ── S2.4  Propagation-only NEES vs Q scale (the #212 consistency lever) ────

template <math::Scalar T>
struct NeesPoint {
    T q_scale{0};
    T nees_pose{0};  ///< averaged 6-DoF (attitude+position) NEES; target = 6
};

template <math::Scalar T>
struct NeesResult {
    std::vector<NeesPoint<T>> curve;
    std::size_t dof = 6;
    std::size_t trials = 0;
};

/// Monte-Carlo: simulate a true trajectory, drive the filter with NOISE-CORRUPTED
/// IMU, and measure the 6-DoF (θ,p) NEES of the propagated error against P, as a
/// function of a Q-scale multiplier. NEES≈dof ⇒ consistent; NEES≫dof ⇒
/// over-confident (Q too small). This is the propagation-side reading of the
/// "NEES ∝ 1/Q" lever from #212.
template <math::Scalar T>
[[nodiscard]] NeesResult<T> nees_vs_qscale(const msckf::ImuNoise<T>& noise,
                                           T dt = T{1} / T{200},
                                           T duration_s = T{0.5},
                                           std::size_t trials = 400,
                                           std::uint64_t seed = 0x5723) {
    using namespace pp_detail;
    using msckf::State;
    NeesResult<T> out;
    out.trials = trials;
    const Vec3<T> g_world{{T{0}, T{0}, T{-9.81}}};
    const Vec3<T> w_b{{T{0.2}, T{-0.1}, T{0.3}}};  // exciting body rate
    const Vec3<T> a_w{{T{0.8}, T{-0.5}, T{0.2}}};  // world accel
    const auto nsteps = static_cast<std::size_t>(duration_s / dt);
    using std::sqrt;
    // Discrete per-sample measurement-noise std (continuous density / √dt).
    const T sg = noise.gyro / sqrt(dt);
    const T sa = noise.accel / sqrt(dt);

    std::mt19937_64 rng(seed);
    std::normal_distribution<T> N01(T{0}, T{1});

    for (const T qscale : {T{0.25}, T{0.5}, T{1}, T{2}, T{4}}) {
        msckf::ImuNoise<T> sn = noise;
        sn.gyro *= sqrt(qscale);
        sn.accel *= sqrt(qscale);  // Q ∝ σ² ⇒ scale σ by √qscale
        T nees_sum{0};
        for (std::size_t trial = 0; trial < trials; ++trial) {
            State<T> sf(T{0});  // filter, P0 = 0
            sf.R = SO3<T>{};
            msckf::Propagator<T> prop(sn, g_world);
            SO3<T> R_true{};
            Vec3<T> v_true{}, p_true{};
            for (std::size_t k = 0; k < nsteps; ++k) {
                // true specific force at the true state
                const Vec3<T> a_meas_true = R_true.inverse() * (a_w - g_world);
                // corrupt the measurement the filter sees
                const Vec3<T> w_m{{w_b[0] + sg * N01(rng), w_b[1] + sg * N01(rng), w_b[2] + sg * N01(rng)}};
                const Vec3<T> a_m{
                    {a_meas_true[0] + sa * N01(rng), a_meas_true[1] + sa * N01(rng), a_meas_true[2] + sa * N01(rng)}};
                prop.propagate(sf, w_m, a_m, dt);
                // advance the truth (clean)
                p_true = p_true + v_true * dt + a_w * (T{0.5} * dt * dt);
                v_true = v_true + a_w * dt;
                R_true = R_true * SO3<T>::exp(w_b * dt);
            }
            // 6-DoF error e = [θ; p], θ = Log(R_trueᵀ R_filter), p = p_f − p_true.
            const Vec3<T> th = (R_true.inverse() * sf.R).log();
            const Vec3<T> dp{{sf.p[0] - p_true[0], sf.p[1] - p_true[1], sf.p[2] - p_true[2]}};
            DynMat<T> e(6, 1);
            for (std::size_t i = 0; i < 3; ++i) {
                e(i, 0) = th[i];
                e(3 + i, 0) = dp[i];
            }
            // 6×6 covariance over [θ(0..3), p(3..6)] from the full P.
            const DynMat<T> P = sf.covariance();
            DynMat<T> P6(6, 6);
            const std::size_t idx[6] = {kTheta, kTheta + 1, kTheta + 2, kPos, kPos + 1, kPos + 2};
            for (std::size_t i = 0; i < 6; ++i)
                for (std::size_t j = 0; j < 6; ++j)
                    P6(i, j) = P(idx[i], idx[j]);
            for (std::size_t i = 0; i < 6; ++i)
                P6(i, i) += T(1e-12);  // tiny ridge for invertibility
            const DynMat<T> Pinv_e = msckf::spd_solve(P6, e);
            T nees{0};
            for (std::size_t i = 0; i < 6; ++i)
                nees += e(i, 0) * Pinv_e(i, 0);
            nees_sum += nees;
        }
        out.curve.push_back(NeesPoint<T>{qscale, nees_sum / static_cast<T>(trials)});
    }
    return out;
}

// ── S2.5  Observability: global-position nullspace preservation by Φ ───────
//
// The propagation must not manufacture information along the unobservable
// global-position directions. In the error state those are δp = e_i (a constant
// world-position shift), and Φ must map them back into the same subspace
// (Φ·N ⊂ span N). The leak ‖Φ·N − N‖ measures any spurious coupling. (Yaw is the
// harder unobservable direction and is a JOINT property requiring the S6
// measurement Jacobian — flagged, not tested here.)

template <math::Scalar T>
struct NullspaceResult {
    T position_leak{0};  ///< ‖Φ·N_pos − N_pos‖_F over a representative step (≈0)
};

template <math::Scalar T>
[[nodiscard]] NullspaceResult<T>
nullspace_position(const pp_detail::Vec3<T>& gyro, const pp_detail::Vec3<T>& accel, T dt = T{1} / T{200}) {
    using namespace pp_detail;
    const SO3<T> R{};  // representative orientation
    const DynMat<T> F = imu_F(R, gyro, accel, dt);
    using std::sqrt;
    T leak{0};
    for (std::size_t col = 0; col < 3; ++col) {
        // N column = e_{kPos+col}; (Φ·N)_i = F(i, kPos+col). Leak = Φ·N minus N.
        for (std::size_t i = 0; i < kImu; ++i) {
            const T target = (i == kPos + col) ? T{1} : T{0};
            const T d = F(i, kPos + col) - target;
            leak += d * d;
        }
    }
    return NullspaceResult<T>{sqrt(leak)};
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_PROPAGATION_PROBE_HPP
