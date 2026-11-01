// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/calibration_probe.hpp — the S10 (online calibration) probe of
// the VIO contract program (docs/pipeline/canonical-pipeline.md), and the
// leading candidate for the #212 over-confidence.
//
// The cortex MSCKF treats the camera↔IMU calibration (extrinsics T_CI, time
// offset t_d, intrinsics) as PERFECTLY KNOWN: there are no calibration states
// and no calibration term in the measurement-noise budget. The #212
// troubleshooting found that inflating the camera noise R by ~4× restores
// filter consistency (NEES→1) on well-tracked sequences — and R×4 is precisely
// what unmodeled calibration uncertainty would require. This probe MEASURES that
// hypothesis two ways:
//
//   1. Analytic noise budget — given a realistic calibration uncertainty
//      (extrinsic rotation σ in deg, translation σ in mm, time-offset σ in ms),
//      compute the reprojection error it induces (reusing the S0 sensitivities)
//      and the equivalent measurement-noise inflation factor. Does it reproduce
//      the empirical R×4?
//
//   2. End-to-end R-inflation sweep — on the synthetic world with PERFECT
//      calibration and clean sensors, sweep the measurement noise R and find
//      the factor (if any) that brings pose NEES to dof, then compare it to the
//      1° calibration budget. The sensors are EXACT and the assumed R (4.6 px)
//      is pessimistic, so its NEES 0.04 at R×1 (#537) says only that the world
//      is self-consistent; it says nothing about consistency under noise (#551).
//      The NEES ≈ 43 once measured here came from the world's warm-up velocity
//      step (#475), not from the filter.
//
//   3. Matched-noise control (#551) — the same backend with camera and IMU noise
//      injected at exactly the σ's it assumes, the NEES split into roll/pitch,
//      yaw and position (world frame). Finding: the body-frame MSCKF is
//      over-confident here. Yaw is unobservable, yet its σ collapses from the
//      prior within ~2 s while the yaw error grows: spurious information in a
//      gauge direction, the #212 leak, which spills into roll/pitch through
//      their correlation. The R-IEKF on the same kind of noise is consistent.
//
// Native units: pixels (induced reprojection error), × (R-inflation factor),
// NEES (dimensionless, target = dof). Drives the REAL backend on the REAL
// synthetic world, so it tests the shipped code.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_CALIBRATION_PROBE_HPP
#define BRANES_SDK_EVAL_CALIBRATION_PROBE_HPP

#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf_backend.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <span>
#include <vector>

namespace branes::sdk::eval {

namespace calib_detail {
template <math::Scalar T>
using Vec3 = math::lie::detail::Vec<T, 3>;
template <math::Scalar T>
using SO3 = math::lie::SO3<T>;
template <math::Scalar T>
inline constexpr T kPi = T{3.14159265358979323846L};

// EuRoC cam0 focal length (px) — the synthetic world uses these intrinsics.
template <math::Scalar T>
inline constexpr T kFocal = T{458654} / T{1000};
}  // namespace calib_detail

// ── S10.1  Analytic calibration noise budget (px, ×) ───────────────────────

template <math::Scalar T>
struct NoiseBudget {
    T ext_rot_px = T{0};          ///< reprojection σ induced by extrinsic-rotation uncertainty
    T ext_trans_px = T{0};        ///< … by extrinsic-translation uncertainty (∝1/depth)
    T time_offset_px = T{0};      ///< … by camera↔IMU time-offset uncertainty (∝ motion)
    T total_calib_px = T{0};      ///< RSS of the calibration sources
    T assumed_px = T{0};          ///< the filter's assumed measurement σ (normalized·focal)
    T effective_px = T{0};        ///< √(assumed² + total_calib²) — the σ it SHOULD use
    T r_inflation_factor = T{0};  ///< (effective/assumed)² — the variance inflation R needs
};

/// Compute the measurement-noise inflation that a calibration uncertainty
/// induces, for a feature at `depth_m` under a camera moving at `pixel_rate_px_per_ms`
/// (the time-offset lever — from S0; ~0.12 slow … ~0.87 aggressive). `assumed_norm_sigma`
/// is the filter's `camera_noise_normalized` (default 0.01).
template <math::Scalar T>
[[nodiscard]] NoiseBudget<T> noise_budget(T ext_rot_deg,
                                          T ext_trans_mm,
                                          T time_offset_ms,
                                          T pixel_rate_px_per_ms,
                                          T depth_m = T{5},
                                          T assumed_norm_sigma = T{1} / T{100}) {
    using calib_detail::kFocal;
    using calib_detail::kPi;
    using std::sqrt;
    const T f = kFocal<T>;
    NoiseBudget<T> b;
    // A small extrinsic rotation δ rotates the bearing by δ ⇒ pixel shift ≈ f·δ(rad).
    b.ext_rot_px = f * ext_rot_deg * (kPi<T> / T{180});
    // A translation δp at depth d shifts the bearing by δp/d ⇒ pixel shift ≈ f·δp/d.
    b.ext_trans_px = f * (ext_trans_mm / T{1000}) / depth_m;
    // A time offset Δt while the feature sweeps the image at pixel_rate.
    b.time_offset_px = pixel_rate_px_per_ms * time_offset_ms;
    b.total_calib_px =
        sqrt(b.ext_rot_px * b.ext_rot_px + b.ext_trans_px * b.ext_trans_px + b.time_offset_px * b.time_offset_px);
    b.assumed_px = assumed_norm_sigma * f;
    b.effective_px = sqrt(b.assumed_px * b.assumed_px + b.total_calib_px * b.total_calib_px);
    b.r_inflation_factor =
        b.assumed_px > T{0} ? (b.effective_px / b.assumed_px) * (b.effective_px / b.assumed_px) : T{0};
    return b;
}

// ── S10.2  End-to-end R-inflation sweep (NEES vs R) ───────────────────────

namespace calib_detail {
/// Run the synthetic world through the backend with a given camera extrinsic and
/// measurement-noise σ; return the mean 6-DoF (θ,p) pose NEES and the ATE vs GT.
template <math::Scalar T>
struct CalibRun {
    T nees = T{0};
    T ate = T{0};
    std::size_t frames = 0;
    // The same pose NEES split by direction, with attitude in the WORLD frame so
    // that yaw (about gravity, unobservable) separates from roll/pitch.
    T nees_rp = T{0};   ///< roll/pitch, target 2
    T nees_yaw = T{0};  ///< yaw, target 1
    T nees_pos = T{0};  ///< position, target 3 (gauge: σ stays near its prior)
    T yaw_sigma_first = T{0};  ///< world-yaw σ at the first initialized frame (rad)
    T yaw_sigma_last = T{0};   ///< … at the last frame; a consistent filter never lowers it
};

template <math::Scalar T>
[[nodiscard]] CalibRun<T>
run_with_calib(const SyntheticData<T>& w, const SO3<T>& R_ic, const Vec3<T>& p_ic, T cam_noise_norm) {
    using Backend = branes::sdk::MsckfBackend<T>;
    using Cal = typename Backend::CameraCalibration;
    using msckf::DynMat;
    Cal cal;
    cal.intrinsics = w.camera;
    cal.extrinsics.R_imu_cam = R_ic;
    cal.extrinsics.p_imu_cam = p_ic;
    Backend backend(std::vector<Cal>{cal});
    VioConfig cfg;
    cfg.camera_noise_normalized = static_cast<double>(cam_noise_norm);
    backend.initialize(cfg);

    constexpr std::size_t kTheta = msckf::State<T>::kTheta, kPos = msckf::State<T>::kPos;
    const std::size_t idx[6] = {kTheta, kTheta + 1, kTheta + 2, kPos, kPos + 1, kPos + 2};
    using std::sqrt;

    CalibRun<T> out;
    T nees_sum = T{0}, sq_sum = T{0}, rp_sum = T{0}, yaw_sum = T{0}, pos_sum = T{0};
    // eᵀ P⁻¹ e over the listed components of a 6-vector.
    const auto nees_of = [](const DynMat<T>& P6, const DynMat<T>& e6, std::span<const std::size_t> c) {
        const std::size_t n = c.size();
        DynMat<T> Ps(n, n), es(n, 1);
        for (std::size_t i = 0; i < n; ++i) {
            es(i, 0) = e6(c[i], 0);
            for (std::size_t j = 0; j < n; ++j)
                Ps(i, j) = P6(c[i], c[j]);
            Ps(i, i) += T(1e-12);
        }
        const DynMat<T> x = msckf::spd_solve(Ps, es);
        T v = T{0};
        for (std::size_t i = 0; i < n; ++i)
            v += es(i, 0) * x(i, 0);
        return v;
    };
    constexpr std::size_t kAll[6] = {0, 1, 2, 3, 4, 5}, kRp[2] = {0, 1}, kYaw[1] = {2}, kP[3] = {3, 4, 5};
    std::size_t imu_idx = 0;
    for (std::size_t fi = 0; fi < w.frames.size(); ++fi) {
        const double t = w.frames[fi].t;
        for (; imu_idx < w.imu.size() && w.imu[imu_idx].timestamp_s <= t; ++imu_idx)
            backend.process_imu(w.imu[imu_idx]);
        backend.process_camera(t, std::span<const FrontendObservation<T>>{w.frames[fi].obs});
        if (!backend.initialized())
            continue;
        const auto& st = backend.state();
        const auto ns = backend.current_state();
        const Vec3<T> ep = ns.T_world_imu.translation();
        const Vec3<T>& gp = w.gt[fi].p;
        const Vec3<T> dp{{ep[0] - gp[0], ep[1] - gp[1], ep[2] - gp[2]}};
        sq_sum += dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2];

        // 6-DoF pose error: θ = Log(R_estᵀ R_gt) (right perturbation), p = p_gt − p_est.
        // Rotated to the world frame (θ_w = R_est·θ, P_w = J P Jᵀ) for the split.
        const auto Rm = ns.T_world_imu.rotation().matrix();
        const Vec3<T> th = (ns.T_world_imu.rotation().inverse() * w.gt[fi].R).log();
        const DynMat<T> P = st.covariance();
        DynMat<T> e(6, 1), P6(6, 6), J(6, 6);
        for (std::size_t i = 0; i < 3; ++i) {
            e(i, 0) = th[i];
            e(3 + i, 0) = gp[i] - ep[i];
            J(3 + i, 3 + i) = T{1};
            for (std::size_t j = 0; j < 3; ++j)
                J(i, j) = Rm(i, j);
        }
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                P6(i, j) = P(idx[i], idx[j]);
        const DynMat<T> ew = msckf::mul(J, e);
        const DynMat<T> Pw = msckf::mul(msckf::mul(J, P6), msckf::transpose(J));
        nees_sum += nees_of(Pw, ew, kAll);
        rp_sum += nees_of(Pw, ew, kRp);
        yaw_sum += nees_of(Pw, ew, kYaw);
        pos_sum += nees_of(Pw, ew, kP);
        out.yaw_sigma_last = sqrt(Pw(2, 2));
        if (out.frames == 0)
            out.yaw_sigma_first = out.yaw_sigma_last;
        ++out.frames;
    }
    if (out.frames) {
        const T n = static_cast<T>(out.frames);
        out.nees_rp = rp_sum / n;
        out.nees_yaw = yaw_sum / n;
        out.nees_pos = pos_sum / n;
    }
    out.nees = out.frames ? nees_sum / static_cast<T>(out.frames) : T{0};
    out.ate = out.frames ? sqrt(sq_sum / static_cast<T>(out.frames)) : T{0};
    return out;
}
}  // namespace calib_detail

template <math::Scalar T>
struct RInflationPoint {
    T r_scale = T{0};  ///< measurement-noise VARIANCE multiplier (σ scales by √)
    T nees = T{0};     ///< mean 6-DoF pose NEES (target = dof)
    T ate = T{0};
};

template <math::Scalar T>
struct RInflationSweep {
    std::vector<RInflationPoint<T>> curve;
    T r_scale_for_consistency = T{0};  ///< the R-scale that brings NEES down to dof (empirical)
    T budget_r_inflation_1deg = T{0};  ///< the analytic R-inflation for 1° extrinsic uncertainty
    std::size_t dof = 6;
};

/// The #212 control: on the synthetic world with PERFECT calibration and clean
/// sensors, sweep the measurement-noise variance R by a factor and find the
/// factor that brings pose NEES down to dof — `r_scale_for_consistency`, 0 when
/// NEES is already at or below dof at R×1 (as it is since #537). If the filter
/// were over-confident here, comparing that factor to the analytic calibration
/// budget would say whether ~1° of unmodeled extrinsic uncertainty explains it.
template <math::Scalar T>
[[nodiscard]] RInflationSweep<T> r_inflation_sweep() {
    using namespace calib_detail;
    SyntheticConfig<T> scfg;  // gentle motion, short + sparse for speed
    scfg.trans_amp = T{0.5};
    scfg.motion_rate = T{0.6};
    scfg.yaw_amp = T{0.3};
    scfg.duration_s = T{8};
    scfg.num_landmarks = 140;
    const auto w = generate_world<T>(scfg);
    const T base_norm = T{1} / T{100};
    using std::sqrt;

    RInflationSweep<T> out;
    out.budget_r_inflation_1deg = noise_budget<T>(T{1}, T{10}, T{0}, T{0.12}, T{5}).r_inflation_factor;
    for (const T s : {T{1}, T{2}, T{4}, T{8}, T{16}, T{32}, T{64}}) {
        const auto run = run_with_calib(w, w.R_imu_cam, w.p_imu_cam, base_norm * sqrt(s));
        out.curve.push_back(RInflationPoint<T>{s, run.nees, run.ate});
    }
    // Interpolate (in log-R) the scale where NEES first crosses dof.
    const T dof = static_cast<T>(out.dof);
    for (std::size_t i = 1; i < out.curve.size(); ++i) {
        const auto& a = out.curve[i - 1];
        const auto& b = out.curve[i];
        if (a.nees > dof && b.nees <= dof) {
            const T f = (a.nees - dof) / (a.nees - b.nees);  // linear in R-scale
            out.r_scale_for_consistency = a.r_scale + f * (b.r_scale - a.r_scale);
            break;
        }
    }
    return out;
}

// ── S10.3  Matched-noise control (#551) ───────────────────────────────────

/// The control S10.2 is not: the default synthetic world with camera noise of
/// `pixel_sigma` px on every observation and IMU white noise at `VioConfig`'s
/// densities, and the backend assuming exactly those σ's, with perfect
/// calibration. A consistent filter gives pose NEES ≈ 6 (roll/pitch ≈ 2, yaw ≈ 1,
/// position ≈ 3) and never lowers its yaw σ, since yaw is unobservable.
/// Deterministic in `seed`.
template <math::Scalar T>
[[nodiscard]] calib_detail::CalibRun<T>
matched_noise_control(std::uint32_t seed, T pixel_sigma = T{1}, T onset_s = SyntheticConfig<T>{}.onset_s) {
    using std::sqrt;
    SyntheticConfig<T> scfg;
    scfg.onset_s = onset_s;
    auto w = generate_world<T>(scfg);
    const VioConfig vc;
    const T dt = T{1} / scfg.imu_rate_hz;
    const T gyro_sigma = static_cast<T>(vc.gyro_noise_density) / sqrt(dt);
    const T accel_sigma = static_cast<T>(vc.accel_noise_density) / sqrt(dt);
    std::mt19937 rng(seed);
    std::normal_distribution<T> nd(T{0}, T{1});
    // Each draw is its own statement: argument evaluation order is unspecified (#437).
    for (auto& m : w.imu)
        for (std::size_t k = 0; k < 3; ++k) {
            const T a = nd(rng);
            const T b = nd(rng);
            m.angular_velocity[k] += gyro_sigma * a;
            m.linear_acceleration[k] += accel_sigma * b;
        }
    for (auto& f : w.frames)
        for (auto& o : f.obs) {
            const T a = nd(rng);
            const T b = nd(rng);
            o.u += pixel_sigma * a;
            o.v += pixel_sigma * b;
        }
    return calib_detail::run_with_calib(w, w.R_imu_cam, w.p_imu_cam, pixel_sigma / w.camera.fx());
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_CALIBRATION_PROBE_HPP
