// SPDX-License-Identifier: MIT
//
// branes/tools/vio_stage_contracts.hpp — the machine-readable registry of the
// VIO pipeline stage contracts S0–S10, the executable counterpart of
// docs/arch/vio-pipeline-canonical.md.
//
// Each `StageInfo` carries the stage's signature, pre/post-conditions, the
// native-unit assessments that test the post-conditions, the CSV artifacts the
// probe emits, and the cortex file the stage exercises. The stage-probe
// utilities (tools/src/sN_*.cpp) drive these; keeping the contracts in one
// header keeps the utilities, the figures, and the doc from drifting apart.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_VIO_STAGE_CONTRACTS_HPP
#define BRANES_TOOLS_VIO_STAGE_CONTRACTS_HPP

#include <branes/tools/stage_probe.hpp>

#include <vector>

namespace branes::tools {

// ── S0  Sensor & calibration models ────────────────────────────────────────
inline const StageInfo kS0{
    "S0",
    "Sensor & calibration models",
    "camera: project(p_c,ζ)↔unproject;  imu: ã=Rᵀ(a−g)+ba;  T_CI∈SE(3);  t_d∈ℝ",
    {"camera intrinsics valid (fx,fy>0); distortion in convergence basin",
     "gravity sign & magnitude pinned and frame-consistent (static IMU reads +g)",
     "T_CI and t_d known with a STATED uncertainty that feeds the filter"},
    {"projection round-trips: unproject∘project ≈ identity within the FOV",
     "analytic ∂(pixel)/∂(point) matches the true derivative (first order)",
     "cheirality enforced: points with z≤0 are rejected, never projected",
     "stationary IMU through correct gravity-aligned R integrates to ~zero drift"},
    {{"projection round-trip residual", "px", "distortion model is its own inverse; grows toward FOV edge"},
     {"Jacobian linearization error", "px", "analytic projection Jacobian is the true derivative (≈0)"},
     {"time-offset sensitivity", "px/ms", "pixel cost of an unmodeled camera↔IMU time offset t_d"},
     {"extrinsic rotation sensitivity", "px/deg", "reprojection bias per degree of T_CI rotation error"},
     {"extrinsic translation sensitivity", "px/mm", "reprojection bias per mm of T_CI translation error (∝1/depth)"},
     {"IMU dead-reckoning drift", "mm", "gravity leakage: small tilt/bias → large position drift"}},
    {"roundtrip_radtan.csv",
     "roundtrip_fisheye.csv",
     "jacobian_radtan.csv",
     "jacobian_fisheye.csv",
     "timeoffset.csv",
     "extrinsic.csv",
     "imu_drift.csv"},
    "math/cameras/*.hpp, sdk/msckf/propagator.hpp",
    "implemented"};

// ── S1  Initialization ─────────────────────────────────────────────────────
inline const StageInfo kS1{
    "S1",
    "Initialization (static & dynamic bootstrap)",
    "init(imu_window,[vision]) → (R_GI,v,b_g,b_a,g,[scale]) + initial P",
    {"static: accel & gyro std below stationarity thresholds",
     "dynamic: sufficient acceleration excitation (scale-observability gate)"},
    {"R_GI aligns measured accel mean to gravity; post-alignment gravity residual ≈ 0",
     "|g| within tolerance of local gravity (9.78–9.83)",
     "dynamic: scale s>0, finite; alignment linear-system condition number bounded",
     "initial P honestly large on the unobservable directions (yaw, accel-bias, scale)"},
    {{"static gravity-leveling residual", "m/s²", "the recovered R_GI actually levels gravity (≈0)"},
     {"recovered gravity magnitude", "m/s²", "≈9.81; a wrong value flags a sign/frame bug"},
     {"roll/pitch leveling error vs noise", "deg", "accel-noise sensitivity; the stationarity gate"},
     {"dynamic scale-recovery error", "%", "metric scale from the VI alignment under excitation"},
     {"scale-observability cliff", "% / m", "error blows up & the gate declines as excitation drops"},
     {"initial-P seed per block", "deg / m / m/s²", "isotropic σ·I — NOT enlarged on yaw/scale/accel-bias"}},
    {"init_static_sweep.csv", "init_excitation_sweep.csv", "init_p_sizing.csv"},
    "sdk/imu_init.hpp, sdk/imu_preintegration.hpp",
    "implemented"};

// ── S2  IMU propagation ─────────────────────────────────────────────────────
inline const StageInfo kS2{
    "S2",
    "IMU propagation (mean + covariance)",
    "propagate(x⁻,P⁻,imu[t_k,t_{k+1}]) → (x⁻_{k+1}, P⁻_{k+1})",
    {"Δt>0, monotonic timestamps; measurements debiased; P⁺_k symmetric PSD"},
    {"R_{k+1}∈SO(3) (orthonormal, det=1)",
     "P⁻ symmetric PSD; Φ matches the canonical block structure",
     "Q_d PSD and carries the θ–v–p cross-correlations (NOT diagonal)",
     "the 4-D unobservable subspace stays in the linearized null space (FEJ)"},
    {{"Q_d position-block & v–p drop", "m, m²/s", "cortex diagonal Q omits ¼σ_a²Δt³ (pos) and ½σ_a²Δt² (v–p)"},
     {"pos-σ under-report (cortex vs canon)", "% / mm", "does the diagonal-Q omission actually cost covariance?"},
     {"GT-injection propagation error", "mm / deg", "zero-noise propagation reproduces the GT trajectory; dt-order"},
     {"propagation-only NEES vs Q scale", "—", "NEES vs target dof=6 — the #212 consistency lever (NEES∝1/Q)"},
     {"R orthonormality residual", "—", "‖RᵀR−I‖ stays at machine zero through integration"},
     {"P min eigenvalue", "—", "covariance stays PSD every step"},
     {"global-position nullspace leak", "—", "Φ preserves the unobservable position subspace (≈0)"}},
    {"prop_q_structure.csv", "prop_growth.csv", "prop_gt_injection.csv", "prop_nees.csv"},
    "sdk/msckf/propagator.hpp",
    "implemented"};

// ── S3  State augmentation / cloning ────────────────────────────────────────
inline const StageInfo kS3{
    "S3",
    "State augmentation / stochastic cloning",
    "augment(x,P) → (x',P') appending a clone of the current camera pose",
    {"clone count < window max; current pose valid; P PSD"},
    {"P' = G P Gᵀ exactly; P' symmetric PSD",
     "clone marginal == cloned-pose marginal (the clone is a deterministic copy)",
     "cross-cov(clone, every other state) == that of the cloned pose (NOT zero/independent)",
     "G carries the right convention (T_CI / t_d if the clone is the camera pose)"},
    {{"clone-vs-pose marginal error", "—", "‖P'[clone]−P[pose]‖ ≈ 0 at augmentation"},
     {"clone cross-covariance error", "—", "‖P'[clone,*]−P[pose,*]‖ ≈ 0 — zero cross-cov is the classic bug"},
     {"augment∘marginalize round-trip", "—", "removing a just-added clone restores P"},
     {"P min eigenvalue after augment", "—", "covariance stays PSD"}},
    {"augment_block_equality.csv"},
    "sdk/msckf/state_helper.hpp (augment_clone)",
    "scaffold"};

// ── S4  Visual frontend ─────────────────────────────────────────────────────
inline const StageInfo kS4{
    "S4",
    "Visual frontend (track generation)",
    "track(image_t, prev_tracks) → observations[(id,cam,u,v)]",
    {"image in expected format; previous tracks available; optional gyro rotation prior"},
    {"every surviving track passes forward-backward (round-trip pixel error < τ)",
     "RANSAC inlier ratio above a floor; outliers removed before the backend",
     "spatial coverage across the image grid (clusters ⇒ poor rotational DoF)",
     "track-length distribution healthy (not all length-2 — that starves parallax)"},
    {{"forward-backward residual", "px", "tracking is self-consistent both directions"},
     {"RANSAC inlier ratio", "%", "geometric outliers are rejected pre-backend"},
     {"grid coverage occupancy", "%", "features spread across the frame, not clustered"},
     {"track-length histogram", "frames", "tracks live long enough to build parallax"},
     {"pixel-noise → track survival", "px", "the true measurement noise the backend should use"}},
    {"frontend_fb_residual.csv", "frontend_coverage.csv", "frontend_tracklen.csv"},
    "sdk/vio_estimator.hpp (frontend), cv KLT",
    "scaffold"};

// ── S5  Feature triangulation ──────────────────────────────────────────────
inline const StageInfo kS5{"S5",
                           "Feature triangulation",
                           "triangulate(observations, clone_poses, T_CI) → (p_f∈ℝ³, status)",
                           {"≥2 observations from distinct, sufficiently-parallax clones; cheirality holds"},
                           {"reprojection residual at the solution below threshold",
                            "depth positive and finite; cheirality holds in every observing view",
                            "triangulation normal-matrix condition number bounded (low parallax ⇒ defer/down-weight)"},
                           {{"reprojection RMS at solution", "px", "the triangulated point explains its observations"},
                            {"parallax angle (max over views)", "deg", "the geometric conditioner of depth"},
                            {"triangulation condition number", "—", "low parallax ⇒ ill-conditioned ⇒ huge depth σ"},
                            {"depth error vs parallax", "m", "where the parallax gate must sit"},
                            {"depth uncertainty vs parallax", "m", "the σ a low-parallax feature truly has"}},
                           {"triang_parallax_sweep.csv", "triang_reproj.csv"},
                           "sdk/msckf/camera_updater.hpp (triangulate)",
                           "scaffold"};

// ── S6  MSCKF update ────────────────────────────────────────────────────────
inline const StageInfo kS6{"S6",
                           "MSCKF update (null-space → compress → gate → EKF)",
                           "update(x⁻,P⁻,{tracks}) → (x⁺,P⁺)",
                           {"triangulated p_f valid (S5); ≥2 observations; H evaluated at FEJ, residual at current"},
                           {"H_f full column-rank 3; analytic H_x,H_f == finite-diff",
                            "null-space N orthonormal (NᵀN=I, NᵀH_f≈0) ⇒ projected noise stays σ²I; rows=2m−3",
                            "QR compression exact (Q₁ᵀ orthogonal ⇒ noise still σ²I)",
                            "post-update P⁺ symmetric PSD (Joseph); x⁺ on-manifold; FEJ points untouched",
                            "MASTER: post-update NEES≈dim, NIS≈dof, innovations white"},
                           {{"H_f column rank", "—", "the feature direction is observable (rank 3)"},
                            {"Jacobian analytic-vs-numeric", "—", "measurement Jacobians are the true derivatives"},
                            {"null-space orthonormality", "—", "‖NᵀN−I‖, ‖NᵀH_f‖ ≈ 0 — else noise model is wrong"},
                            {"NIS vs χ²(dof)", "—", "innovations sized as predicted; NIS≫dof ⇒ over-confidence"},
                            {"innovation whiteness (lag-1)", "—", "no information left uncaptured frame-to-frame"},
                            {"FEJ clone divergence", "deg / cm", "how far the frozen linearization point has drifted"}},
                           {"update_nis.csv", "update_jacobian.csv", "update_nullspace.csv", "update_fej_div.csv"},
                           "sdk/msckf/camera_updater.hpp, features/msckf_nullspace.hpp, msckf/covariance.hpp",
                           "scaffold"};

// ── S7  SLAM-feature update (optional) ─────────────────────────────────────
inline const StageInfo kS7{"S7",
                           "SLAM-feature update (in-state landmarks)",
                           "features kept in-state (3 or 1 params), updated by reprojection without marginalization",
                           {"feature stochastically initialized into the state with correct cross-covariance"},
                           {"in-state feature P blocks stay PSD and correlated with the poses that see them",
                            "pruning a SLAM feature = info-preserving Schur complement"},
                           {{"feature-init cross-covariance error", "—", "stochastic init, not independent injection"},
                            {"in-state landmark NEES", "—", "the landmark estimate is consistent with its covariance"},
                            {"long-track drift reduction", "m", "persistent landmarks bound drift vs pure-MSCKF"}},
                           {"slam_feat_nees.csv"},
                           "sdk/features/representations.hpp",
                           "scaffold"};

// ── S8  Zero-velocity update (optional) ────────────────────────────────────
inline const StageInfo kS8{"S8",
                           "Zero-velocity update (robustness)",
                           "detect stationarity (IMU + visual disparity) → apply v=0 pseudo-measurement",
                           {"stationarity detector tuned; not firing during slow motion"},
                           {"prevents spurious motion / feature-starved drift while static",
                            "detector does not false-positive during slow motion (would inject wrong v=0)"},
                           {{"detector ROC (true/false positive)", "%", "stationary correctly detected, motion not"},
                            {"static-segment drift with/without ZUPT", "mm", "ZUPT actually arrests stationary drift"},
                            {"false-ZUPT-induced bias", "mm/s", "cost of a wrongly-fired v=0 constraint"}},
                           {"zupt_roc.csv", "zupt_static_drift.csv"},
                           "(not yet present in cortex)",
                           "scaffold"};

// ── S9  Marginalization / clone management ─────────────────────────────────
inline const StageInfo kS9{"S9",
                           "Marginalization / clone management",
                           "marginalize(x,P,clone_idx) → (x',P')  (slide the window)",
                           {"clone selected by policy (oldest, or two-way keyframe/non-keyframe)"},
                           {"P' symmetric PSD",
                            "kept-state marginal UNCHANGED by removing a clone (principal-submatrix extraction)",
                            "marginalization does not constrain the 4 gauge directions"},
                           {{"kept-marginal invariance", "—", "‖P'[keep]−P[keep,keep]‖ ≈ 0 for pure extraction"},
                            {"P min eigenvalue after marginalize", "—", "covariance stays PSD"},
                            {"window length bound", "clones", "the window stays bounded as designed"}},
                           {"marg_invariance.csv"},
                           "sdk/msckf/state_helper.hpp (marginalize_clone)",
                           "scaffold"};

// ── S10  Online calibration ─────────────────────────────────────────────────
inline const StageInfo kS10{
    "S10",
    "Online calibration (extrinsics / intrinsics / time offset)",
    "calibration parameters as state: T_CI(6), intrinsics+distortion(8), t_d(1) per camera",
    {"if estimated: observable only under sufficient excitation",
     "if fixed: the assumed-perfect calibration's true uncertainty must appear in the noise budget"},
    {"estimated calibration states converge & stay consistent under excitation",
     "CANDIDATE (#212): unmodeled calibration uncertainty ⇒ structural over-confidence"},
    {{"time-offset sensitivity (from S0)", "px/ms", "the σ the filter omits by fixing t_d"},
     {"extrinsic sensitivity (from S0)", "px/deg, px/mm", "the σ the filter omits by fixing T_CI"},
     {"R-inflation equivalent of calib σ", "×", "does adding calib uncertainty match the empirical R×4?"},
     {"calibration-state NEES (if estimated)", "—", "online calibration is itself consistent"}},
    {"calib_sensitivity.csv", "calib_r_equivalent.csv"},
    "(no calibration states in cortex — the S10 gap)",
    "scaffold"};

/// The whole pipeline, in dataflow order — for `--list`.
[[nodiscard]] inline std::vector<StageInfo> pipeline() {
    return {kS0, kS1, kS2, kS3, kS4, kS5, kS6, kS7, kS8, kS9, kS10};
}

}  // namespace branes::tools

#endif  // BRANES_TOOLS_VIO_STAGE_CONTRACTS_HPP
