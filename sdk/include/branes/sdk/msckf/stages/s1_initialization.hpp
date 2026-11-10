// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s1_initialization.hpp — stage S1_initialization as an
// explicit transformation (issue #452, docs/pipeline/canonical-pipeline.md §S1).
//
//   seed_from_imu(state, init result, t, accel window, prior)     → (state', diagnostics)
//   seed_from_alignment(state, VI alignment, SfM window, t, ...)  → (state', diagnostics)
//   tilt_bias_prior(accel window, method, σ_ba, ...)              → what a static window determines (#609)
//
// Seeds a fresh filter state from a resolved initialization: the static /
// gravity-only / identity bootstrap of ImuInitializer, or the dynamic
// visual-inertial alignment over an SfM keyframe window. Deciding WHEN an
// initializer has enough data (windows, timeouts, the dynamic-vs-static
// preference) is the backend's policy; this stage is the state seed and the
// record of how it went.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S1_INITIALIZATION_HPP
#define BRANES_SDK_MSCKF_STAGES_S1_INITIALIZATION_HPP

#include <branes/sdk/imu_init.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/sfm/init_window.hpp>

#include <cstddef>
#include <span>

namespace branes::sdk {

/// How the backend resolved its initial attitude/bias. Surfacing this is the
/// single most useful divergence-triage signal: a tilted real-world start that
/// silently lands on `Identity` injects a phantom ~g acceleration and the
/// filter diverges within seconds.
enum class InitMethod {
    None,          ///< not initialized yet
    Static,        ///< stationary window: full roll/pitch + gyro bias
    Dynamic,       ///< moving start: full visual-inertial alignment (yaw + scale + velocity)
    GravityAlign,  ///< moving start: roll/pitch from mean specific force only
    Identity,      ///< last resort: mean force wasn't even gravity-like
};

[[nodiscard]] inline const char* to_string(InitMethod m) noexcept {
    switch (m) {
    case InitMethod::Static:
        return "static";
    case InitMethod::Dynamic:
        return "dynamic";
    case InitMethod::GravityAlign:
        return "gravity_align";
    case InitMethod::Identity:
        return "identity";
    case InitMethod::None:
        break;
    }
    return "none";
}

/// Read-only record of how initialization went (telemetry, like `initialized`).
template <math::Scalar T>
struct InitDiagnostics {
    using DVec3 = math::lie::detail::Vec<T, 3>;
    InitMethod method = InitMethod::None;
    double t_s = 0.0;               ///< filter time at which init completed
    DVec3 up_body{};                ///< measured "up" (specific-force) direction in the body frame
    double gravity_residual = 0.0;  ///< ||mean accel| − g| / g over the init window
    DVec3 gyro_bias{};
    DVec3 accel_bias{};
    // Dynamic-path diagnostics (only meaningful when method == Dynamic): the
    // recovered metric scale of the vision poses, the speed the filter is
    // seeded with (|v_world| of the last keyframe), and the number of SfM
    // keyframes the alignment used. Surfaced to localize a bad dynamic seed.
    double dyn_scale = 0.0;
    double dyn_seed_speed = 0.0;
    int dyn_keyframes = 0;
    // The window the alignment used: its first and last keyframe times, and the
    // distance between those keyframes in vision units. Scaled by dyn_scale it
    // is the metric displacement, so a caller with ground truth can check the
    // scale (the vision units are arbitrary, so dyn_scale alone means nothing).
    double dyn_t_first = 0.0;
    double dyn_t_last = 0.0;
    double dyn_vision_span = 0.0;
    // Dynamic-init attempt accounting, accumulated across every try while
    // uninitialized — populated even when dynamic never fires, to localize why
    // (#247). dyn_attempts: tries past kMinDynFrames; dyn_window_builds: how
    // many produced a valid SfM window (else the two-view/PnP bootstrap is
    // failing on real tracks); dyn_best_keyframes / dyn_best_metric_motion: the
    // best window size and resolved metric motion (scale · displacement) seen —
    // if builds succeed but the motion stays below min_dynamic_motion, the
    // vision trajectory is inconsistent with the IMU (scale collapses).
    int dyn_attempts = 0;
    int dyn_window_builds = 0;
    int dyn_best_keyframes = 0;
    double dyn_best_metric_motion = 0.0;
    // Roll/pitch sanity for a dynamic seed (#247): angle (deg) between the
    // seed's world-up expressed in the body frame and the accelerometer's
    // mean specific-force direction over the init window. Large ⇒ the
    // VI-solved gravity DIRECTION is wrong (under-observable on a low-excitation
    // window), injecting phantom horizontal gravity → divergence.
    double dyn_tilt_vs_accel_deg = 0.0;
};

namespace msckf::stages::s1_initialization {

/// What an IMU seed window determines about tilt and accelerometer bias (#609).
///
/// At rest the accelerometer reads f = Rᵀ·g·e_z + b_a + n. A seed that takes
/// its "up" û from the window's mean f̄ with b_a = 0 makes the tilt error
/// δθ (R = R̂·Exp(δθ)) and the bias b_a one quantity seen once:
///
///   δθ⊥ = [û]ₓ (b_a + n̄) / g,    the yaw component δθ·û untouched.
///
/// So P₀ is not isotropic. With b_a ~ N(0, σ_ba² I) and the mean force's own
/// error n̄ ~ N(0, σ_f² I):
///
///   P_θθ = (σ_ba² + σ_f²)/g² · (I − ûûᵀ) + σ_ψ² ûûᵀ
///   P_θb = σ_ba²/g · [û]ₓ                       (P_bθ = P_θbᵀ)
///   P_bb = σ_ba² (I − ûûᵀ) + σ_∥² ûûᵀ
///
/// The tilt and the horizontal bias are nearly fully correlated: the filter
/// starts knowing their combination, not either alone, and separates them as
/// the platform rotates. Along û the window sees |f̄| − g = b_a·û + n̄·û: a
/// static seed takes the bias along up from it, with σ_∥ the error of that
/// difference; otherwise σ_∥ = σ_ba.
template <math::Scalar T>
struct TiltBiasPrior {
    T accel_bias_sigma = T{0};  ///< σ_ba per axis; 0 leaves the fresh state's prior as it is
    T mean_force_sigma = T{0};  ///< σ_f: per-axis error of the window's mean specific force
    T along_up_sigma = T{0};    ///< σ_∥ when the bias along up is seeded from |f̄| − g; 0 = not seeded
};

/// Half the spread of surface gravity (9.780 at the equator to 9.832 m/s² at the
/// poles): the error of a configured |g| that was not measured on site. It
/// bounds how well |f̄| − g measures the bias along up.
template <math::Scalar T>
inline constexpr T kGravityMagnitudeSigma = T{26} / T{1000};

/// The prior a seed window implies, for `method`:
///  - Static: the window is at rest, so σ_f is its mean's noise, the larger of
///    the sample scatter over √N and the white-noise model σ_a/√(window); the
///    bias along up is seeded, σ_∥² = σ_f² + kGravityMagnitudeSigma².
///  - GravityAlign, Dynamic and Identity: none; the fresh state's isotropic
///    prior stays. A gravity-aligned window moves: its mean force includes a
///    mean acceleration (|f̄| − g is 2.6 m/s² on the EuRoC gate's MH_03 seeds),
///    so the honest tilt σ is about 15°, and the seed's zero velocity (σ 0.1
///    m/s) is then what is wrong. Given that tilt σ the first updates explain
///    the velocity by tilt (2.4° → 12–19°) and three of the gate's five MH_03
///    draws diverge, against one with the isotropic 5.7° (#609). A dynamic
///    seed's tilt comes from the visual-inertial alignment, not the
///    accelerometer mean.
/// `accel_bias_sigma` ≤ 0 disables the prior (the isotropic P₀ before #609).
template <math::Scalar T>
[[nodiscard]] TiltBiasPrior<T> tilt_bias_prior(std::span<const math::lie::detail::Vec<T, 3>> accel_window,
                                               InitMethod method,
                                               T accel_bias_sigma,
                                               T accel_noise_density,
                                               T window_seconds) {
    namespace ld = math::lie::detail;
    using DVec3 = ld::Vec<T, 3>;
    TiltBiasPrior<T> p;
    const std::size_t n = accel_window.size();
    if (!(accel_bias_sigma > T{0}) || n < 2 || method != InitMethod::Static)
        return p;
    DVec3 am{};
    for (const DVec3& a : accel_window)
        am = am + a;
    am = am * (T{1} / static_cast<T>(n));
    T ss{0};
    for (const DVec3& a : accel_window) {
        const DVec3 d = a - am;
        ss = ss + ld::dot(d, d);
    }
    const T scatter2 = ss / (T{3} * static_cast<T>(n - 1));  // per-axis sample variance
    const T model2 = window_seconds > T{0} ? accel_noise_density * accel_noise_density / window_seconds : T{0};
    const T mean2 = scatter2 / static_cast<T>(n) > model2 ? scatter2 / static_cast<T>(n) : model2;
    p.accel_bias_sigma = accel_bias_sigma;
    p.mean_force_sigma = ld::sqrt_(mean2);
    p.along_up_sigma = ld::sqrt_(mean2 + kGravityMagnitudeSigma<T> * kGravityMagnitudeSigma<T>);
    return p;
}

/// Rewrite the attitude and accelerometer-bias blocks of a fresh state's prior
/// with `prior`, about the seed's body "up" û = R̂ᵀe_z (see TiltBiasPrior). The
/// yaw variance and every other block are kept. `s` must be fresh: no clones,
/// and no IMU–calibration cross-covariance. Returns false, leaving `s` as it
/// was, when the prior is disabled or not positive-definite.
template <math::Scalar T, class Cov>
bool apply_tilt_bias_prior(State<T, Cov>& s, T gravity_magnitude, const TiltBiasPrior<T>& prior) {
    namespace ld = math::lie::detail;
    using St = State<T, Cov>;
    constexpr std::size_t n = St::kImuDim;
    if (!(prior.accel_bias_sigma > T{0}) || !(gravity_magnitude > T{0}))
        return false;
    const auto ez = ld::Vec<T, 3>{{T{0}, T{0}, T{1}}};
    const auto u = s.R.inverse() * ez;
    const auto ux = ld::hat(u);
    const DynMat<T> P = s.covariance();
    DynMat<T> cur(n, n), M(n, n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            cur(i, j) = M(i, j) = P(i, j);
    T yaw2{0};  // the yaw variance ûᵀ P_θθ û, kept
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            yaw2 = yaw2 + u[i] * P(St::kTheta + i, St::kTheta + j) * u[j];
    const T sb2 = prior.accel_bias_sigma * prior.accel_bias_sigma;
    const T sf2 = prior.mean_force_sigma * prior.mean_force_sigma;
    const T g = gravity_magnitude;
    const T tilt2 = (sb2 + sf2) / (g * g);
    const T par2 = prior.along_up_sigma > T{0} ? prior.along_up_sigma * prior.along_up_sigma : sb2;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            const T uu = u[i] * u[j];
            const T perp = (i == j ? T{1} : T{0}) - uu;
            M(St::kTheta + i, St::kTheta + j) = tilt2 * perp + yaw2 * uu;
            M(St::kBa + i, St::kBa + j) = sb2 * perp + par2 * uu;
            M(St::kTheta + i, St::kBa + j) = sb2 / g * ux(i, j);
            M(St::kBa + j, St::kTheta + i) = sb2 / g * ux(i, j);
        }
        for (std::size_t k = 0; k < n; ++k) {  // nothing else is correlated with the new blocks
            if (k < St::kTheta || k >= St::kTheta + 3) {
                if (k < St::kBa || k >= St::kBa + 3) {
                    M(St::kTheta + i, k) = M(k, St::kTheta + i) = T{0};
                    M(St::kBa + i, k) = M(k, St::kBa + i) = T{0};
                }
            }
        }
    }
    // P' = G P Gᵀ with G = L_M · L_cur⁻¹ on the IMU block, identity elsewhere.
    DynMat<T> Lm, Lc;
    if (!cholesky(M, Lm) || !cholesky(cur, Lc))
        return false;
    DynMat<T> Lci(n, n);  // L_cur⁻¹ by forward substitution, column by column
    for (std::size_t c = 0; c < n; ++c) {
        for (std::size_t i = c; i < n; ++i) {
            T acc = i == c ? T{1} : T{0};
            for (std::size_t k = c; k < i; ++k)
                acc = acc - Lc(i, k) * Lci(k, c);
            Lci(i, c) = acc / Lc(i, i);
        }
    }
    const DynMat<T> Gi = mul(Lm, Lci);
    DynMat<T> G = DynMat<T>::identity(s.cov.dim());
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            G(i, j) = Gi(i, j);
    s.cov.transform(G);
    return true;
}

/// Seed `s` (a fresh state) from an IMU-only initialization — static, gravity
/// alignment, or the identity fallback (a default-constructed `res`) — and
/// record the result in `diag`: method, time, the measured up-direction and
/// gravity residual over `accel_window`, and the biases. The dynamic-attempt
/// counters in `diag` are left as they are.
template <math::Scalar T, class Cov>
void seed_from_imu(State<T, Cov>& s,
                   const ImuInitResult<T>& res,
                   InitMethod method,
                   double t,
                   std::span<const math::lie::detail::Vec<T, 3>> accel_window,
                   T gravity_magnitude,
                   InitDiagnostics<T>& diag,
                   const TiltBiasPrior<T>& prior = {}) {
    using DVec3 = math::lie::detail::Vec<T, 3>;
    s.R = res.R_world_imu;  // identity for a default-constructed result
    s.bg = res.gyro_bias;
    s.ba = res.accel_bias;
    s.timestamp = t;

    DVec3 am{};
    for (const DVec3& a : accel_window)
        am = am + a;
    if (!accel_window.empty())
        am = am * (T{1} / static_cast<T>(accel_window.size()));
    const T g_meas = math::lie::detail::norm(am);
    const T g_cfg = gravity_magnitude;
    diag.method = method;
    diag.t_s = t;
    diag.up_body = g_meas > T{0} ? am * (T{1} / g_meas) : DVec3{};
    // Explicit narrowing: the diagnostic is double telemetry, T may be a posit.
    diag.gravity_residual =
        static_cast<double>((g_meas > T{0} && g_cfg > T{0}) ? math::lie::detail::abs_(g_meas - g_cfg) / g_cfg : T{1});
    // The bias along up, from |f̄| − g, when the prior says the window resolves it.
    if (prior.along_up_sigma > T{0} && g_meas > T{0}) {
        const DVec3 up = s.R.inverse() * DVec3{{T{0}, T{0}, T{1}}};
        s.ba = s.ba + up * (g_meas - g_cfg);
    }
    (void)apply_tilt_bias_prior(s, g_cfg, prior);
    diag.gyro_bias = res.gyro_bias;
    diag.accel_bias = s.ba;
}

/// Seed `s` (a fresh state) from a successful dynamic visual-inertial
/// alignment `r` over the SfM keyframe window `win`. try_dynamic resolves
/// gravity, metric per-keyframe velocity, scale, and the gravity-aligned
/// attitude of the *first* keyframe; the vision relative rotation carries it to
/// the last keyframe (the current pose), whose velocity seeds the filter. The
/// alignment's accelerometer-bias estimate is recorded, not seeded: the bias
/// starts at zero. Records
/// the seed in `diag`, including the roll/pitch sanity check against the mean
/// specific force over `accel_window` (#247).
template <math::Scalar T, class Cov>
void seed_from_alignment(State<T, Cov>& s,
                         const ImuInitResult<T>& r,
                         const sfm::InitWindowResult<T>& win,
                         double t,
                         std::span<const math::lie::detail::Vec<T, 3>> accel_window,
                         T gravity_magnitude,
                         InitDiagnostics<T>& diag) {
    using DVec3 = math::lie::detail::Vec<T, 3>;
    const auto rel = win.keyframes.front().R_world_imu.inverse() * win.keyframes.back().R_world_imu;
    s.R = r.R_world_imu * rel;
    s.v = r.velocities_world.empty() ? DVec3{} : r.velocities_world.back();
    s.bg = r.gyro_bias;
    // The alignment estimates an accelerometer bias so that it does not leak
    // into the scale (#501), but over a 1 s window it is weakly observable and
    // held toward zero by a prior. The filter starts from zero and its own
    // bias covariance: seeding the estimate moved EuRoC MH_01 / MH_02 by
    // +6 % / −0.5 % in ATE. The estimate is kept in the diagnostics.
    s.ba = DVec3{};
    s.timestamp = t;

    const T g_norm = math::lie::detail::norm(r.gravity_world);
    const T g_cfg = gravity_magnitude;
    const DVec3 up_world = g_norm > T{0} ? r.gravity_world * (-T{1} / g_norm) : DVec3{};
    diag.method = InitMethod::Dynamic;
    diag.t_s = t;
    diag.up_body = s.R.inverse() * up_world;  // "up" in the current body frame
    // Explicit narrowing: the diagnostic is double telemetry, T may be a posit.
    diag.gravity_residual =
        static_cast<double>((g_norm > T{0} && g_cfg > T{0}) ? math::lie::detail::abs_(g_norm - g_cfg) / g_cfg : T{1});
    diag.gyro_bias = r.gyro_bias;
    diag.accel_bias = r.accel_bias;  // the alignment's estimate, not seeded (above)
    // Roll/pitch sanity: how far the seed's up disagrees with the mean
    // accelerometer up over the init window (gravity-dominated). Large ⇒
    // the VI-solved gravity direction is wrong → divergence (#247).
    if (!accel_window.empty()) {
        namespace ld = math::lie::detail;
        DVec3 am{};
        for (const auto& a : accel_window)
            am = am + a;
        const T amn = ld::norm(am);
        const T ubn = ld::norm(diag.up_body);
        if (amn > T{0} && ubn > T{0}) {
            const DVec3 au = am * (T{1} / amn);            // accelerometer up (body)
            const DVec3 su = diag.up_body * (T{1} / ubn);  // seed up (body)
            // angle(u,v) = atan2(|u×v|, u·v) — robust near 0 and π.
            const T ang = ld::atan2_(ld::norm(ld::cross(au, su)), ld::dot(au, su));
            diag.dyn_tilt_vs_accel_deg = static_cast<double>(ang) * 180.0 / 3.14159265358979323846;
        }
    }
    diag.dyn_scale = static_cast<double>(r.scale);
    diag.dyn_seed_speed = static_cast<double>(math::lie::detail::norm(s.v));
    diag.dyn_keyframes = static_cast<int>(win.keyframes.size());
}

}  // namespace msckf::stages::s1_initialization

}  // namespace branes::sdk

#endif  // BRANES_SDK_MSCKF_STAGES_S1_INITIALIZATION_HPP
