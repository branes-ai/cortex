---
title: Trajectory Accuracy (ATE / RPE)
description: How trajectory accuracy is measured — the EuRoC replay harness and the clean-room ATE/RPE metrics.
---

Accuracy is measured with the two standard VIO/SLAM trajectory metrics, computed
clean-room from their definitions (no `ov_eval`/`evo` source contact), and driven by
an EuRoC replay harness (#46).

## The metrics (`branes::sdk::eval`)

### Absolute Trajectory Error (ATE)

ATE rigidly aligns the estimated trajectory to ground truth — VIO has a gauge
freedom, so the global pose is arbitrary — and reports the **translational RMSE** of
the aligned positions. The alignment is **Horn's closed-form absolute-orientation
solution** (Horn, 1987): the optimal rotation is the eigenvector of the largest
eigenvalue of a 4×4 profile matrix.

> **Implementation note worth keeping:** the profile matrix's top two eigenvalues are
> routinely near-degenerate, which makes power iteration converge far too slowly (and
> silently return a wrong rotation). The implementation uses a **cyclic Jacobi
> eigensolver** on the 4×4 instead — robust regardless of the eigenvalue gap.

### Relative Pose Error (RPE)

RPE compares relative motions over a fixed step `Δ`: for each `i`, it compares
`Pᵢ⁻¹·Pᵢ₊Δ` between estimate and ground truth and reports the translational RMSE of
the difference. RPE is invariant to a global rigid transform, so it measures local
drift rather than global alignment.

A `associate()` helper matches estimated poses to the nearest-in-time ground-truth
pose within a tolerance.

## The EuRoC replay harness (`branes::sdk::euroc`)

Parsers read the ASL CSV layout — `imu0/data.csv` (gyro+accel), `cam0/data.csv`
(image timestamps + filenames), `state_groundtruth_estimate0/data.csv` (`T_world_body`)
— and a `replay` driver feeds a `VioEstimator` the time-ordered IMU+image stream
(loading PNGs on demand), returning the estimated trajectory. Parsers reject
non-finite values and zero-norm quaternions; a single unreadable frame is skipped
rather than aborting the run.

## How it runs in CI vs. locally

The metrics and CSV parsers are **unit-tested in CI** on synthetic fixtures (ATE = 0
for identical / rigidly-transformed trajectories, ≈ 0.1 m for a known offset; RPE = 0
under a global transform; association windowing; ASL parsing). The **full V1_01_easy
replay + ATE-threshold assertion is dataset-gated** — set
`CORTEX_EUROC_V101=/path/to/V1_01_easy/mav0` to run it; otherwise it skips so
`ctest -R vio_euroc` stays green without the ~1.5 GB sequence.

## The accuracy gate

| Sequence | Gate (ATE) | Reference (published, keyframe) |
|---|---|---|
| EuRoC V1_01_easy | **< 0.50 m** | OpenVINS ~0.05–0.06 m · VINS-Fusion ~0.08 m |

The gate is deliberately generous relative to tuned SOTA: this MVP MSCKF has no online
intrinsics/extrinsics refinement and no loop closure, so the bar is set to **catch a
broken pipeline without over-fitting to a tuned number**, and is documented in the
test file to be tightened as the estimator improves.
