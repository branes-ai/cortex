// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s1_initialization.hpp — stage S1_initialization as an
// explicit transformation (issue #452, docs/arch/vio-pipeline-canonical.md §S1).
//
//   seed_from_imu(state, init result, t, accel window)            → (state', diagnostics)
//   seed_from_alignment(state, VI alignment, SfM window, t, ...)  → (state', diagnostics)
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
                   InitDiagnostics<T>& diag) {
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
    diag.gyro_bias = res.gyro_bias;
    diag.accel_bias = res.accel_bias;
}

/// Seed `s` (a fresh state) from a successful dynamic visual-inertial
/// alignment `r` over the SfM keyframe window `win`. try_dynamic resolves
/// gravity, metric per-keyframe velocity, scale, and the gravity-aligned
/// attitude of the *first* keyframe; the vision relative rotation carries it to
/// the last keyframe (the current pose), whose velocity seeds the filter. The
/// accel bias is not observable from the alignment and starts at zero. Records
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
    s.ba = DVec3{};  // accel bias is not observable from the dynamic alignment
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
    diag.accel_bias = DVec3{};
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
