---
title: Visual-Inertial Odometry Overview
description: The MSCKF visual-inertial odometry pipeline that epic E3 delivered, end to end.
---

Epic **E3 (Phase 3)** delivered a complete **Multi-State Constraint Kalman Filter
(MSCKF)** visual-inertial odometry pipeline — clean-room from the papers (Mourikis &
Roumeliotis 2007, Forster et al., VINS-Mono), no GPL source contact.

## The pipeline

```text
  images ─▶ Front End ──┐
                        │ FrontendObservation (feature id, pixel u,v)
  IMU ────────────────┐ │
                      ▼ ▼
              ┌──────────────────────┐
              │     VioEstimator     │  lifecycle-gated façade
              └───────────┬──────────┘
                          │ pixels → bearings (camera model)
                          ▼
              ┌──────────────────────┐
              │   MsckfBackend       │
              │  • IMU init          │  static / dynamic
              │  • propagate         │  per-sample mean + covariance
              │  • augment / margin. │  sliding window of cloned poses
              │  • camera update     │  null-space + Mahalanobis gate
              └───────────┬──────────┘
                          ▼
                   NavState (T_world_imu, velocity, stamp)
```

## The components

| Component | Page | Issue |
|---|---|---|
| Visual front end (FAST + KLT + pyramid) | [Front End](/vio/front-end/) | #37, #38, #49, #48 |
| IMU preintegration + initialization | [IMU](/vio/imu/) | #40, #42 |
| MSCKF state / propagator / clones | [MSCKF State](/vio/msckf-state/) | #43 |
| Camera updaters (null-space) | [Camera Updaters](/vio/camera-updaters/) | #44 |
| MSCKF backend (the wiring) | [MSCKF Backend](/vio/msckf-backend/) | #35 |
| Sliding-window backend skeleton | [MSCKF Backend](/vio/msckf-backend/#a-second-backend-the-sliding-window-skeleton) | #36 |
| Top-level estimator API | [VioEstimator](/vio/vio-estimator/) | #45 |

## Why MSCKF

MSCKF keeps a **sliding window of past camera poses** in the filter state and uses
each tracked feature as a multi-view geometric constraint that is **marginalized**
(removed from the state) the moment it is consumed — via a left-null-space projection
of its measurement Jacobian. The feature is never added to the state vector, so the
filter cost stays bounded and roughly linear in the window size, which is exactly the
property a real-time embedded estimator needs.

## How it was validated

- **Stability:** the camera-update path holds the covariance positive-semidefinite
  and the innovation well-conditioned over **1000 consecutive updates**.
- **Accuracy:** ATE/RPE against ground truth via an [EuRoC replay harness](/benchmarks/accuracy/).
- **Latency:** a [per-frame budget](/benchmarks/latency/) on `feed_image`.

See [Benchmarks & Validation](/benchmarks/accuracy/) for the numbers and methodology.

## What's deferred

A **square-root covariance** variant of the backend (numerically superior
conditioning via a QR-form update) was split out as a post-MVP follow-up
([#187](https://github.com/branes-ai/cortex/issues/187)) rather than shipping a dead
`static_assert`-guarded template knob. The current backend is the full-covariance
form.
