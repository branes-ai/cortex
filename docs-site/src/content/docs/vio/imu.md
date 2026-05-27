---
title: IMU Preintegration & Initialization
description: On-manifold IMU preintegration (Forster) and static/dynamic visual-inertial initialization.
---

## IMU preintegration (#40)

`ImuPreintegrator<T>` accumulates gyro + accel samples between two keyframes into the
preintegrated rotation **ΔR**, velocity **Δv**, and position **Δp**, together with
their first-order **Jacobians w.r.t. the gyro/accel biases** — so a later bias update
can be applied without re-integrating the whole window.

It is clean-room from the papers only — Forster et al., *"On-Manifold Preintegration
for Real-Time Visual-Inertial Odometry"* (2016/2017) and Solà's error-state kinematics
(2017). The convention is that the accelerometer measures specific force
`a_m = Rᵀ(a_world − g)`, so preintegration is gravity- and initial-state-independent.
A non-positive or non-finite `dt` (out-of-order or duplicate timestamps) is ignored
rather than corrupting the accumulators.

Preintegration is the natural input for a sliding-window-optimization backend; the
MSCKF backend instead propagates the filter per IMU sample (see
[MSCKF State](/vio/msckf-state/)), and uses the same on-manifold math.

## Initialization (#42)

`ImuInitializer<T>` bootstraps the estimator's initial state two ways.

### Static initialization

From a window of **stationary** IMU samples:

- gate the window on gyro/accel standard deviation and on the measured gravity
  magnitude (reject a moving or accelerating window);
- the gyro mean is the **gyro bias** (true angular rate is zero at rest);
- the accelerometer mean points along **−gravity**, which fixes roll and pitch — the
  initial attitude aligns the measured "up" with world +z. Yaw is unobservable from
  gravity and left at zero.

**Validated:** recovers gravity direction to within **0.5°** on a synthetic tilted +
biased stationary window (the EuRoC acceptance bar), and rejects a non-stationary
window.

### Dynamic initialization

When no static window is available (the platform is already moving), from ≥2 metric
keyframes:

- recover the **gyro bias** with one Gauss-Newton step on the rotation mismatch
  between vision and IMU preintegration;
- recover the **gravity vector and per-keyframe velocities** by linear least squares
  on the preintegration position/velocity constraints, then **rotate the whole result
  into a gravity-aligned world** (gravity along −z) about a horizontal axis — so the
  recovered attitude and gravity are mutually consistent, exactly like the static
  path.

Clean-room from VINS-Mono (Qin et al., 2018) and Martinelli (2014). **Validated:**
recovers gravity, all velocities, and the gyro bias exactly on a synthetic trajectory
(including a non-identity, per-keyframe bias Jacobian).
