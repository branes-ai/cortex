// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/calibration_probe.hpp — the S10 (online calibration) probe of
// the VIO contract program (docs/arch/vio-pipeline-canonical.md), and the
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
//      calibration and clean sensors, the backend is already over-confident
//      (pose NEES ≫ dof — the structural #212 fault); sweep the measurement
//      noise R and find the factor that restores NEES → dof, then compare it to
//      the 1° calibration budget. (Honest finding: calibration uncertainty
//      cleanly accounts for the R×4 *component*; full restoration needs more
//      because the empirical fix also raised Q — calibration is one contributor.)
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
    T nees_sum = T{0}, sq_sum = T{0};
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
        const Vec3<T> th = (ns.T_world_imu.rotation().inverse() * w.gt[fi].R).log();
        DynMat<T> e(6, 1);
        for (std::size_t i = 0; i < 3; ++i) {
            e(i, 0) = th[i];
            e(3 + i, 0) = gp[i] - ep[i];
        }
        const DynMat<T> P = st.covariance();
        DynMat<T> P6(6, 6);
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                P6(i, j) = P(idx[i], idx[j]);
        for (std::size_t i = 0; i < 6; ++i)
            P6(i, i) += T(1e-12);
        const DynMat<T> Pe = msckf::spd_solve(P6, e);
        T nees = T{0};
        for (std::size_t i = 0; i < 6; ++i)
            nees += e(i, 0) * Pe(i, 0);
        nees_sum += nees;
        ++out.frames;
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

/// The decisive #212 measurement, done cleanly. On the synthetic world with
/// PERFECT calibration and clean sensors, the backend is already over-confident
/// (pose NEES ≫ dof) — the structural #212 fault. Sweep the measurement-noise
/// variance R by a factor and find the factor that restores consistency
/// (NEES → dof). Compare that empirical factor to the analytic calibration
/// budget: if the R-inflation the filter needs ≈ what ~1° of unmodeled extrinsic
/// uncertainty induces, then the empirical "R×4 fixes NEES" is explained by
/// unmodeled calibration uncertainty — an S10 fault, not observability/FEJ.
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

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_CALIBRATION_PROBE_HPP
