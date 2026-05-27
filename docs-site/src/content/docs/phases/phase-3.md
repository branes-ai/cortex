---
title: Phase 3 — Visual-Inertial Odometry
description: The complete MSCKF VIO pipeline (epic E3) — what was built, validated, and deferred.
---

> Status: **complete** (2026-05-26). Epic E3 (#33) closed; all 16 sub-issues merged.

Phase 3 delivered a complete **MSCKF visual-inertial odometry** pipeline. See the
[VIO Overview](/vio/overview/) for the architecture; this page is the phase record.

## What landed

Each row is one merged PR, through the full gate (ready PR → CodeRabbit review →
gcc+clang+CI green → merge):

| Issue | Component | Page |
|---|---|---|
| #34 | `VioBackend` interface | [MSCKF Backend](/vio/msckf-backend/) |
| #48 | Image container + PGM/PNG I/O | [Front End](/vio/front-end/) |
| #49 | Gaussian image pyramid | [Front End](/vio/front-end/) |
| #39 | Camera models (pinhole/fisheye/unified) | [Cameras](/math/cameras/) |
| #37 | FAST corner detector | [Front End](/vio/front-end/) |
| #38 | Pyramidal KLT tracker | [Front End](/vio/front-end/) |
| #40 | IMU preintegration | [IMU](/vio/imu/) |
| #41 | Feature representations + null-space types | [Camera Updaters](/vio/camera-updaters/) |
| #43 | State + Propagator + StateHelper | [MSCKF State](/vio/msckf-state/) |
| #42 | IMU initialization (static + dynamic) | [IMU](/vio/imu/) |
| #44 | Camera updaters (null-space MSCKF) | [Camera Updaters](/vio/camera-updaters/) |
| #35 | MSCKF backend | [MSCKF Backend](/vio/msckf-backend/) |
| #45 | Top-level `VioEstimator` | [VioEstimator](/vio/vio-estimator/) |
| #36 | Sliding-window backend skeleton | [MSCKF Backend](/vio/msckf-backend/) |
| #46 | EuRoC replay harness + ATE/RPE | [Accuracy](/benchmarks/accuracy/) |
| #47 | Per-frame latency budget | [Latency](/benchmarks/latency/) |

## What was validated

- **Filter stability** — covariance stays PD/PSD and the innovation well-conditioned
  over **1000 consecutive camera updates**; trace monotonically non-increasing.
- **Initialization** — gravity recovered within **0.5°** (static); gravity, velocities,
  and gyro bias recovered exactly on synthetic trajectories (dynamic).
- **Accuracy** — ATE/RPE via the [EuRoC harness](/benchmarks/accuracy/); V1_01_easy
  gate < 0.50 m (dataset-gated).
- **Latency** — per-frame `feed_image` budget, ~38 ms median at `-O2` on 752×480
  ([details](/benchmarks/latency/)).

## Notable engineering calls

- **Jacobi over power iteration** for ATE's Horn alignment — the 4×4 profile matrix's
  top two eigenvalues are near-degenerate, so power iteration silently returned wrong
  rotations.
- **Dataset / optimized-build gating** so CI stays green without the EuRoC sequence
  and without an optimized build.

## Deferred

- **Square-root covariance backend** — [#187](https://github.com/branes-ai/cortex/issues/187),
  split out rather than shipping a `static_assert`-only template knob.

Next: [Phase 4 — SLAM](/phases/roadmap/).
