// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/invariants.hpp — the VIO invariant-check library (issue #445,
// epic #444 §B): the oracle of the per-stage test benches (#453–#458), the
// composition benches and live-assertion mode (#447), and the S0–S10 inspectors.
//
// Every check returns an inv::InvariantResult — the measured residual in native
// units, its threshold, the bound direction, and the verdict — and collects into
// an inv::InvariantReport. Arithmetic thresholds scale with the type under test
// (safety · n · ε_T · scale, see invariants/core.hpp), never a hardcoded 1e-9, so a
// check means the same thing in float, posit16, posit<32,2> and double.
//
// ── §B invariant → check ─────────────────────────────────────────────────────
//
//  Inputs   timestamps strictly increasing ........ check_timestamps_increasing
//           Δt within sensor-rate bounds ........... check_dt_bounds
//           samples finite / within range .......... check_finite, check_bounded
//           static-window ‖a‖ ≈ g .................. check_static_gravity
//           R_imu_cam ∈ SO(3) ...................... check_so3
//           intrinsics positive, image-bounded ..... check_intrinsics
//  S0       project/unproject, distort round trip .. check_round_trip
//           analytic Jacobians vs FD ............... check_jacobian_fd
//  S1       gravity sign and magnitude (#262) ...... check_gravity
//           biases bounded ......................... check_bounded
//           P₀ symmetric positive-definite ......... check_symmetric, check_spd
//           initial attitude ∈ SO(3) ............... check_so3
//  S2       R ∈ SO(3), quaternion unit norm ........ check_so3, check_unit_quaternion
//           Q_d ⪰ 0 ................................ check_psd
//           P = ΦPΦᵀ + Q_d, symmetric, ⪰ 0 ......... check_covariance_propagation,
//                                                    check_symmetric, check_psd
//           Φ preserves the 4-D unobservable space . check_subspace_preserved
//           propagation-only NEES ≈ dim ............ check_chi2_window
//  S3       P grows by 6 per clone; blocks J P Jᵀ .. check_augmentation
//           P ⪰ 0 preserved ........................ check_psd
//  S4       tracks inside the image ................ check_points_in_image
//           KLT forward–backward error bound ....... check_bounded
//           track length vs clone window ........... check_track_lengths
//  S5       positive depth in every camera ......... check_at_least
//           reprojection-error bound ............... check_bounded
//           parallax, normal-matrix κ (#265) ....... check_scalar(Report),
//                                                    report_condition_number
//  S6a      H_x, H_f vs FD of the measurement model  check_jacobian_fd
//  S6b      ‖Nᵀ H_f‖ ≈ 0 ........................... check_annihilates
//           NᵀN = I ................................ check_orthonormal_rows
//           rank 2m − 3 ............................ check_rank, check_dimension
//  S6c      QR compression preserves HᵀH, Hᵀr ...... check_compression
//  S6d      S symmetric positive-definite .......... check_symmetric, check_spd
//           NIS ~ χ²(dof) over windows ............. check_chi2_window
//  S6e      K = P Hᵀ S⁻¹ solve residual ............ check_solve_residual
//           Joseph P⁺ symmetric ⪰ 0 ................ check_symmetric, check_psd
//           P⁺ ⪯ P⁻ (Loewner) ...................... check_loewner_le
//           δx finite .............................. check_finite
//           δx ⟂ unobservable directions ........... check_no_update_along
//  S9       P after = principal submatrix .......... check_principal_submatrix
//           dimension / clone bookkeeping .......... check_dimension
//           P ⪰ 0 .................................. check_psd
//  S10      extrinsic ∈ SO(3) ...................... check_so3
//           calibration covariance ⪰ 0 ............. check_psd
//  End      NEES / NIS over windows (#264) ......... check_chi2_window
//           ATE / RPE vs ground truth .............. check_scalar
//           per-stage residuals per run ............ InvariantReport::for_stage,
//                                                    first_failure, tightest
//
// Header-only, C++20.

#ifndef BRANES_SDK_EVAL_INVARIANTS_HPP
#define BRANES_SDK_EVAL_INVARIANTS_HPP

#include <branes/sdk/eval/invariants/core.hpp>
#include <branes/sdk/eval/invariants/geometry_checks.hpp>
#include <branes/sdk/eval/invariants/linalg.hpp>
#include <branes/sdk/eval/invariants/matrix_checks.hpp>
#include <branes/sdk/eval/invariants/stream_checks.hpp>

#endif  // BRANES_SDK_EVAL_INVARIANTS_HPP
