// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/sensor_model_probe.hpp — the S0 (sensor & calibration
// model) probe of the VIO contract program (docs/arch/vio-pipeline-canonical.md).
//
// S0 is the substrate every later stage trusts: the camera projection model,
// the IMU measurement/integration model, and the camera↔IMU calibration
// (extrinsics + time offset). If these are wrong — or if their *uncertainty*
// is unmodeled — every downstream Jacobian, residual, and covariance inherits
// the error, and the filter is silently biased or over-confident with no local
// symptom. This header is the measurement device for S0's contracts.
//
// Everything is reported in units NATIVE to the modeling problem, because that
// is what builds intuition and what a threshold can be set against:
//   • pixels                 — projection round-trip, Jacobian linearization error
//   • pixels per millisecond — camera↔IMU time-offset sensitivity
//   • pixels per degree, per millimeter — extrinsic-calibration sensitivity
//   • m/s², mm/s, mm         — IMU static-identity residual and dead-reckoning drift
//   • degrees                — field-of-view / incidence-angle coverage (input domain)
//
// The pattern mirrors the other eval/ probes (allan_variance, consistency):
// pure, header-only, type-generic computation that returns plain structs. The
// caller (a test) asserts the contract and writes CSV artifacts that
// docs-site/scripts/gen-sensor-model-figures.mjs renders to SVG for qualitative
// inspection.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_SENSOR_MODEL_PROBE_HPP
#define BRANES_SDK_EVAL_SENSOR_MODEL_PROBE_HPP

#include <branes/math/cameras.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace branes::sdk::eval {

namespace smp_detail {
// π as a type-generic constant. (std::numbers::pi_v<T> is unusable here: this
// layer is generic over the Universal scalar types, for which pi_v is ill-formed.)
template <math::Scalar T>
inline constexpr T kPi = T{3.14159265358979323846L};
}  // namespace smp_detail

// ── Shared native-unit result types ──────────────────────────────────────

/// Summary of a scalar metric over a sampled domain. `unit` is carried as a
/// string so artifacts and assertions are self-describing (e.g. "px").
template <math::Scalar T>
struct FieldStats {
    T max{0};
    T mean{0};
    T rms{0};
    std::size_t count{0};

    void accumulate(T v) {
        const T a = v < T{0} ? -v : v;
        if (a > max)
            max = a;
        mean += a;
        rms += a * a;
        ++count;
    }
    /// Convert the running sums to mean/RMS. Call exactly once, after the last
    /// accumulate() — it is not idempotent (a second call would re-divide).
    void finalize() {
        if (count > 0) {
            mean /= static_cast<T>(count);
            using std::sqrt;
            rms = sqrt(rms / static_cast<T>(count));
        }
    }
};

/// One sample on the image plane: pixel location (x, y) and the native-unit
/// metric `value` measured there. Used to render image-plane heatmaps.
template <math::Scalar T>
struct ImageSample {
    T x{0};
    T y{0};
    T value{0};
    T incidence_deg{0};  ///< angle of the bearing from the optical axis (input coverage)
};

// ── S0.1  Camera projection round-trip (pixels) ──────────────────────────
//
// Contract: project(unproject(px)) ≈ px for every pixel in the valid image.
// A distortion model that does not round-trip poisons the front end before any
// geometry runs. The residual grows toward the FOV edge where distortion is
// strongest — the heatmap makes that visible.

template <math::Scalar T>
struct RoundTripResult {
    FieldStats<T> residual_px;          ///< |project(unproject(px)) − px|
    std::vector<ImageSample<T>> field;  ///< per-grid-cell residual, for the heatmap
    T min_incidence_deg{0};             ///< input-domain coverage (centre)
    T max_incidence_deg{0};             ///< input-domain coverage (corner)
};

/// Sweep an `nx × ny` grid over the image (inset by `margin_px`), round-trip
/// each pixel through unproject→project, and report the residual in pixels.
/// `Cam` is any model exposing `project(Vec3)`/`unproject(Vec2)` (pinhole-radtan,
/// equidistant, unified).
template <class Cam, math::Scalar T>
[[nodiscard]] RoundTripResult<T> camera_round_trip(const Cam& cam,
                                                   T width,
                                                   T height,
                                                   std::size_t nx,
                                                   std::size_t ny,
                                                   T margin_px = T{2},
                                                   T incidence_cap_deg = T{89}) {
    using math::cameras::Vec2;
    using math::cameras::Vec3;
    RoundTripResult<T> out;
    out.field.reserve(nx * ny);
    bool first = true;
    for (std::size_t iy = 0; iy < ny; ++iy) {
        for (std::size_t ix = 0; ix < nx; ++ix) {
            const T fx = nx > 1 ? static_cast<T>(ix) / static_cast<T>(nx - 1) : T{0.5};
            const T fy = ny > 1 ? static_cast<T>(iy) / static_cast<T>(ny - 1) : T{0.5};
            const Vec2<T> px{margin_px + fx * (width - T{2} * margin_px), margin_px + fy * (height - T{2} * margin_px)};
            const Vec3<T> bearing = cam.unproject(px);
            // Skip points that fold behind the camera (extreme fisheye corners).
            if (!(bearing[2] > T{0}))
                continue;
            const Vec2<T> back = cam.project(bearing);
            using std::sqrt;
            const T dx = back[0] - px[0], dy = back[1] - px[1];
            const T res = sqrt(dx * dx + dy * dy);
            using std::atan2;
            const T rxy = sqrt(bearing[0] * bearing[0] + bearing[1] * bearing[1]);
            const T inc = atan2(rxy, bearing[2]) * (T{180} / smp_detail::kPi<T>);
            // The distortion inverse is only defined within the lens FOV; past
            // the incidence cap (near 90° for a fisheye) tan(θ) explodes and the
            // round-trip is meaningless. Excluded from the contract; the cap is
            // the field-of-view boundary the heatmap then renders as a disk.
            if (inc > incidence_cap_deg)
                continue;
            out.residual_px.accumulate(res);
            out.field.push_back(ImageSample<T>{px[0], px[1], res, inc});
            if (first || inc < out.min_incidence_deg)
                out.min_incidence_deg = inc;
            if (first || inc > out.max_incidence_deg)
                out.max_incidence_deg = inc;
            first = false;
        }
    }
    out.residual_px.finalize();
    return out;
}

// ── S0.2  Projection-Jacobian consistency (pixels & %) ───────────────────
//
// Contract: the analytic d(pixel)/d(point) matches the true map to first order.
// Two readings, one scale-free (for the assertion) and one native (for intuition):
//   • rel_frob  — ‖J_analytic − J_numeric‖_F / ‖J_numeric‖_F   (dimensionless)
//   • lin_px    — ‖(project(p+δ) − project(p)) − J·δ‖ for a fixed METRIC scene
//                 perturbation δ of size `probe_step_m`. "How many pixels does
//                 the linearization mispredict for a `probe_step_m` scene move."
//                 ≈0 ⇒ correct Jacobian; a hotspot ⇒ a Jacobian bug at that field
//                 position. This is the picture that localizes a derivative error.

template <math::Scalar T>
struct JacobianResult {
    FieldStats<T> rel_frob;             ///< dimensionless relative Frobenius error
    FieldStats<T> lin_px;               ///< native pixel linearization error
    std::vector<ImageSample<T>> field;  ///< per-cell lin_px, for the heatmap
};

/// Numeric d(pixel)/d(point) by central difference; analytic via the model's
/// `project_jacobian`. Evaluated at points placed at depth `depth_m` along the
/// bearing through each grid pixel.
template <class Cam, math::Scalar T>
[[nodiscard]] JacobianResult<T> camera_jacobian_consistency(const Cam& cam,
                                                            T width,
                                                            T height,
                                                            std::size_t nx,
                                                            std::size_t ny,
                                                            T depth_m = T{4},
                                                            T probe_step_m = T{0.01},
                                                            T margin_px = T{8}) {
    using math::cameras::Mat23;
    using math::cameras::Vec2;
    using math::cameras::Vec3;
    using std::sqrt;
    JacobianResult<T> out;
    out.field.reserve(nx * ny);
    const T fd = probe_step_m * static_cast<T>(1e-4);  // finite-diff step (m), ≪ probe_step_m
    for (std::size_t iy = 0; iy < ny; ++iy) {
        for (std::size_t ix = 0; ix < nx; ++ix) {
            const T fx = nx > 1 ? static_cast<T>(ix) / static_cast<T>(nx - 1) : T{0.5};
            const T fy = ny > 1 ? static_cast<T>(iy) / static_cast<T>(ny - 1) : T{0.5};
            const Vec2<T> px{margin_px + fx * (width - T{2} * margin_px), margin_px + fy * (height - T{2} * margin_px)};
            const Vec3<T> bearing = cam.unproject(px);
            if (!(bearing[2] > T{0}))
                continue;
            // Place the point at metric depth along the bearing.
            const T s = depth_m / bearing[2];
            const Vec3<T> p{bearing[0] * s, bearing[1] * s, bearing[2] * s};

            const Mat23<T> Ja = cam.project_jacobian(p);

            // Central-difference numeric Jacobian, column by column.
            Mat23<T> Jn{};
            for (std::size_t k = 0; k < 3; ++k) {
                Vec3<T> pp = p, pm = p;
                pp[k] += fd;
                pm[k] -= fd;
                const Vec2<T> a = cam.project(pp);
                const Vec2<T> b = cam.project(pm);
                Jn[0 * 3 + k] = (a[0] - b[0]) / (T{2} * fd);
                Jn[1 * 3 + k] = (a[1] - b[1]) / (T{2} * fd);
            }
            // Relative Frobenius error (scale-free).
            T num = T{0}, den = T{0};
            for (std::size_t e = 0; e < 6; ++e) {
                const T d = Ja[e] - Jn[e];
                num += d * d;
                den += Jn[e] * Jn[e];
            }
            const T rel = den > T{0} ? sqrt(num) / sqrt(den) : T{0};

            // Native pixel linearization error for a `probe_step_m` scene move.
            // Use the worst of the three axis-aligned perturbations.
            T worst = T{0};
            for (std::size_t k = 0; k < 3; ++k) {
                Vec3<T> dp{};
                dp[k] = probe_step_m;
                const Vec3<T> pq{p[0] + dp[0], p[1] + dp[1], p[2] + dp[2]};
                const Vec2<T> truth = cam.project(pq);
                const Vec2<T> base = cam.project(p);
                const T lin0 = Ja[0] * dp[0] + Ja[1] * dp[1] + Ja[2] * dp[2];
                const T lin1 = Ja[3] * dp[0] + Ja[4] * dp[1] + Ja[5] * dp[2];
                const T ex = (truth[0] - base[0]) - lin0;
                const T ey = (truth[1] - base[1]) - lin1;
                const T e = sqrt(ex * ex + ey * ey);
                if (e > worst)
                    worst = e;
            }
            out.rel_frob.accumulate(rel);
            out.lin_px.accumulate(worst);
            out.field.push_back(ImageSample<T>{px[0], px[1], worst, T{0}});
        }
    }
    out.rel_frob.finalize();
    out.lin_px.finalize();
    return out;
}

// ── S0.3  Camera↔IMU time-offset sensitivity (pixels per millisecond) ────
//
// The camera and IMU clocks are rarely synchronized; a constant offset t_d maps
// directly into pose error during motion. A static world point seen by a camera
// moving with linear velocity v and angular velocity ω (camera frame) sweeps
// across the image at ṗ_px = J · (−ω×p_c − v). The reprojection error for an
// unmodeled offset Δt is ‖ṗ_px‖·Δt. This is the cost of NOT having t_d in the
// state — quantified in pixels per millisecond.

template <math::Scalar T>
struct MotionRegime {
    const char* name = "";
    math::lie::detail::Vec<T, 3> lin_vel{};  ///< camera-frame linear velocity (m/s)
    math::lie::detail::Vec<T, 3> ang_vel{};  ///< camera-frame angular velocity (rad/s)
};

template <math::Scalar T>
struct TimeOffsetPoint {
    T dt_ms{0};
    T error_px{0};
};

template <math::Scalar T>
struct TimeOffsetResult {
    T px_per_ms{0};                         ///< slope = pixel rate of the moving feature
    T dt_budget_ms_at_1px{0};               ///< offset that costs one pixel of error
    std::vector<TimeOffsetPoint<T>> curve;  ///< error vs offset, for the line plot
};

/// Time-offset sensitivity for one motion regime, observing a static point at
/// camera-frame position `p_c`. Sweeps `dt_ms` over a symmetric range.
template <class Cam, math::Scalar T>
[[nodiscard]] TimeOffsetResult<T> time_offset_sensitivity(const Cam& cam,
                                                          const math::cameras::Vec3<T>& p_c,
                                                          const MotionRegime<T>& m,
                                                          T dt_max_ms = T{10},
                                                          std::size_t steps = 21) {
    using math::cameras::Mat23;
    using math::cameras::Vec2;
    using math::cameras::Vec3;
    using math::lie::detail::cross;
    using std::sqrt;
    // Camera-frame velocity of the static point: ṗ_c = −ω×p_c − v.
    const math::lie::detail::Vec<T, 3> pcv{{p_c[0], p_c[1], p_c[2]}};
    const auto wx = cross(m.ang_vel, pcv);
    const Vec3<T> pdot{-wx[0] - m.lin_vel[0], -wx[1] - m.lin_vel[1], -wx[2] - m.lin_vel[2]};
    const Mat23<T> J = cam.project_jacobian(p_c);
    const T u = J[0] * pdot[0] + J[1] * pdot[1] + J[2] * pdot[2];  // px/s
    const T v = J[3] * pdot[0] + J[4] * pdot[1] + J[5] * pdot[2];  // px/s
    const T px_per_s = sqrt(u * u + v * v);
    TimeOffsetResult<T> out;
    out.px_per_ms = px_per_s / T{1000};
    out.dt_budget_ms_at_1px = out.px_per_ms > T{0} ? T{1} / out.px_per_ms : T{0};
    out.curve.reserve(steps);
    for (std::size_t i = 0; i < steps; ++i) {
        const T f = steps > 1 ? static_cast<T>(i) / static_cast<T>(steps - 1) : T{0.5};
        const T dt = -dt_max_ms + f * (T{2} * dt_max_ms);
        out.curve.push_back(TimeOffsetPoint<T>{dt, out.px_per_ms * (dt < T{0} ? -dt : dt)});
    }
    return out;
}

// ── S0.4  Camera↔IMU extrinsic sensitivity (px per degree, px per mm) ────
//
// The extrinsic T_CI (rotation R_imu_cam, translation p_imu_cam) is taken from
// calibration and, in this filter, assumed perfect. Perturb it and measure the
// reprojection bias the filter would absorb as if it were noise. Rotation error
// is ~depth-independent; translation error scales with 1/depth — both made
// explicit so the depth dependence is visible.

template <math::Scalar T>
struct ExtrinsicPoint {
    T perturb{0};  ///< degrees (rotation sweep) or millimetres (translation sweep)
    T error_px{0};
};

template <math::Scalar T>
struct ExtrinsicResult {
    T px_per_deg{0};
    T px_per_mm{0};
    std::vector<ExtrinsicPoint<T>> rot_curve;    ///< error vs rotation perturbation (deg)
    std::vector<ExtrinsicPoint<T>> trans_curve;  ///< error vs translation perturbation (mm)
};

/// Extrinsic sensitivity for a feature at IMU-frame position `p_imu_feat`, with
/// nominal extrinsic (`R_imu_cam`, `p_imu_cam`). Sweeps a rotation about the
/// camera y-axis (worst-case yaw-like) and a translation along the camera x-axis.
template <class Cam, math::Scalar T>
[[nodiscard]] ExtrinsicResult<T> extrinsic_sensitivity(const Cam& cam,
                                                       const math::lie::detail::Vec<T, 3>& p_imu_feat,
                                                       const math::lie::SO3<T>& R_imu_cam,
                                                       const math::lie::detail::Vec<T, 3>& p_imu_cam,
                                                       T rot_max_deg = T{2},
                                                       T trans_max_mm = T{20},
                                                       std::size_t steps = 21) {
    using math::cameras::Vec2;
    using math::cameras::Vec3;
    using SO3 = math::lie::SO3<T>;
    using Vec3d = math::lie::detail::Vec<T, 3>;
    using std::sqrt;
    const T deg2rad = smp_detail::kPi<T> / T{180};

    auto project_with = [&](const SO3& Ric, const Vec3d& pic) -> Vec2<T> {
        // Camera-frame point: p_c = R_cam_imu (p_imu_feat − p_imu_cam),
        // and R_cam_imu = R_imu_cam⁻¹.
        const Vec3d d{p_imu_feat[0] - pic[0], p_imu_feat[1] - pic[1], p_imu_feat[2] - pic[2]};
        const Vec3d pc = Ric.inverse() * d;
        return cam.project(Vec3<T>{pc[0], pc[1], pc[2]});
    };
    const Vec2<T> nominal = project_with(R_imu_cam, p_imu_cam);

    ExtrinsicResult<T> out;
    out.rot_curve.reserve(steps);
    out.trans_curve.reserve(steps);
    auto err_from = [&](const Vec2<T>& p) {
        const T dx = p[0] - nominal[0], dy = p[1] - nominal[1];
        return sqrt(dx * dx + dy * dy);
    };
    for (std::size_t i = 0; i < steps; ++i) {
        const T f = steps > 1 ? static_cast<T>(i) / static_cast<T>(steps - 1) : T{0.5};
        const T ang = (-rot_max_deg + f * (T{2} * rot_max_deg));
        const SO3 Rp = R_imu_cam * SO3::exp(Vec3d{{T{0}, ang * deg2rad, T{0}}});
        out.rot_curve.push_back(ExtrinsicPoint<T>{ang, err_from(project_with(Rp, p_imu_cam))});

        const T tr = (-trans_max_mm + f * (T{2} * trans_max_mm));
        const Vec3d pp{{p_imu_cam[0] + tr / T{1000}, p_imu_cam[1], p_imu_cam[2]}};
        out.trans_curve.push_back(ExtrinsicPoint<T>{tr, err_from(project_with(R_imu_cam, pp))});
    }
    // Slopes from the largest perturbation (curves are linear to first order).
    out.px_per_deg =
        rot_max_deg > T{0}
            ? err_from(project_with(R_imu_cam * SO3::exp(Vec3d{{T{0}, rot_max_deg * deg2rad, T{0}}}), p_imu_cam)) /
                  rot_max_deg
            : T{0};
    out.px_per_mm = trans_max_mm > T{0}
                        ? err_from(project_with(
                              R_imu_cam, Vec3d{{p_imu_cam[0] + trans_max_mm / T{1000}, p_imu_cam[1], p_imu_cam[2]}})) /
                              trans_max_mm
                        : T{0};
    return out;
}

// ── S0.5  IMU static identity & dead-reckoning drift (m/s², mm) ───────────
//
// Contract: integrating the REAL propagator on a stationary IMU through the
// correct gravity-aligned orientation must produce ~zero world acceleration and
// hence ~zero position drift. This exercises the actual mean-integration code,
// and the failure cases build the single most important VIO intuition: small
// orientation / bias errors leak through the gravity term into large position
// drift (the doc's "orientation error leaks directly into position").
//
//   case "ideal"      : correct R, no bias            → drift ≈ 0
//   case "gravity_sign": gravity sign flipped         → drift ≈ g·t² (catastrophic)
//   case "tilt_0.5deg" : 0.5° attitude error          → drift ≈ ½·g·sin(δ)·t²
//   case "accel_bias"  : unmodeled b_a = 0.05 m/s²    → drift ≈ ½·b_a·t²

template <math::Scalar T>
struct DriftPoint {
    T t_s{0};
    T pos_drift_mm{0};
    T vel_drift_mm_s{0};
};

template <math::Scalar T>
struct ImuStaticCase {
    const char* name = "";
    T residual_accel_ms2{0};  ///< |a_world| at t=0 (should be ~0 for ideal)
    T final_pos_drift_mm{0};
    std::vector<DriftPoint<T>> curve;
};

/// Run the real `Propagator` on a stationary IMU for `duration_s` at `rate_hz`,
/// for each diagnostic case. `gravity` is world-frame (default (0,0,−g)).
template <math::Scalar T>
[[nodiscard]] std::vector<ImuStaticCase<T>>
imu_static_identity(T duration_s = T{10}, T rate_hz = T{200}, T g = T{9.81}, std::size_t samples = 50) {
    using msckf::ImuNoise;
    using msckf::Propagator;
    using msckf::State;
    using SO3 = math::lie::SO3<T>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using std::sqrt;

    const Vec3 g_world{{T{0}, T{0}, -g}};
    const T dt = T{1} / rate_hz;
    const auto nsteps = static_cast<std::size_t>(duration_s * rate_hz);
    const std::size_t stride = samples > 0 ? (nsteps / samples == 0 ? 1 : nsteps / samples) : 1;

    struct Setup {
        const char* name;
        SO3 R_true;        // truth used to FORM the specific-force measurement
        SO3 R_filter;      // what the filter BELIEVES (the error is R_true vs R_filter)
        Vec3 ba_meas;      // accel bias present in the measurement but NOT estimated
        Vec3 grav_filter;  // gravity the propagator integrates with
    };
    const T deg = smp_detail::kPi<T> / T{180};
    const SO3 level{};
    std::vector<Setup> setups = {
        {"ideal", level, level, Vec3{}, g_world},
        {"gravity_sign", level, level, Vec3{}, Vec3{{T{0}, T{0}, g}}},  // wrong sign
        // Truth is level; the filter believes a 0.5° tilt → gravity leaks horizontal.
        {"tilt_0.5deg", level, SO3::exp(Vec3{{T{0.5} * deg, T{0}, T{0}}}), Vec3{}, g_world},
        // Measurement carries a 0.05 m/s² accel bias the filter does not subtract.
        {"accel_bias", level, level, Vec3{{T{0.05}, T{0}, T{0}}}, g_world},
    };

    std::vector<ImuStaticCase<T>> out;
    for (const auto& su : setups) {
        // The IMU measures specific force ã = R_trueᵀ(0 − g_world) + ba.
        const Vec3 a_meas = su.R_true.inverse() * (-g_world) + su.ba_meas;
        const Vec3 w_meas{};  // stationary: zero angular rate

        Propagator<T> prop(ImuNoise<T>{}, su.grav_filter);
        State<T> s(T{1});
        s.R = su.R_filter;  // filter believes the (possibly wrong) orientation

        ImuStaticCase<T> rec;
        rec.name = su.name;
        // Residual world acceleration the filter sees at t=0.
        const Vec3 a0 = s.R.matrix() * a_meas + su.grav_filter;
        rec.residual_accel_ms2 = sqrt(a0[0] * a0[0] + a0[1] * a0[1] + a0[2] * a0[2]);

        for (std::size_t k = 0; k < nsteps; ++k) {
            prop.propagate(s, w_meas, a_meas, dt);
            if (k % stride == 0 || k + 1 == nsteps) {
                const T pos = sqrt(s.p[0] * s.p[0] + s.p[1] * s.p[1] + s.p[2] * s.p[2]);
                const T vel = sqrt(s.v[0] * s.v[0] + s.v[1] * s.v[1] + s.v[2] * s.v[2]);
                rec.curve.push_back(DriftPoint<T>{static_cast<T>(k + 1) * dt, pos * T{1000}, vel * T{1000}});
            }
        }
        const T finalpos = sqrt(s.p[0] * s.p[0] + s.p[1] * s.p[1] + s.p[2] * s.p[2]);
        rec.final_pos_drift_mm = finalpos * T{1000};
        out.push_back(std::move(rec));
    }
    return out;
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_SENSOR_MODEL_PROBE_HPP
