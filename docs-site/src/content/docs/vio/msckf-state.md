---
title: MSCKF State Machinery
description: The error-state vector, per-sample propagation, clone augmentation/marginalization, and the EKF update.
---

The MSCKF state machinery (`sdk/include/branes/sdk/msckf/`, #43) is the filter's
core: the state vector, IMU propagation, the sliding window of cloned poses, and the
Kalman update. All of it is header-only and type-generic.

## The state vector

`State<T>` holds the inertial navigation state plus a sliding window of cloned IMU
poses and the joint error-state covariance. The error state (world-centric, right
perturbation `R ← R·Exp(δθ)`) is laid out as:

```text
[ δθ(3) δp(3) δv(3) δbg(3) δba(3) | per clone: δθ_c(3) δp_c(3) … ]
```

so the covariance `P` is `(15 + 6·#clones)` square. Constants name the block offsets
(`kTheta=0, kPos=3, kVel=6, kBg=9, kBa=12`, `kImuDim=15`, `kCloneDim=6`). Timestamps
are wall-clock seconds (`double`) — intentionally not the state scalar `T`.

## Dense linear algebra

`dense.hpp` provides the runtime-sized `DynMat<T>` (the covariance grows/shrinks with
the window) and just the operations the filter needs: `mul`, `transpose`, `add`,
`symmetrize` (`A ← (A+Aᵀ)/2`, to counter drift), `cholesky`, `cholesky_solve`,
`spd_solve`, and PD/PSD checks. Every operation asserts its shape preconditions.

## Propagation

`Propagator<T>::propagate(state, gyro, accel, dt)` advances the state one IMU sample:

- **mean** — discrete strapdown integration of orientation, velocity, position;
- **covariance** — `P ← F·P·Fᵀ + Q`, with `F = I + A·dt` the first-order error-state
  transition on the 15×15 IMU block (clones are static during propagation) and `Q`
  the discrete process-noise injection. The transition follows the right-perturbation
  model `δθ̇ = −[ω]× δθ − δbg`. A non-positive `dt` is ignored.

## Clone augmentation & marginalization (`StateHelper`)

- **`augment_clone`** appends a clone of the current IMU pose with the exact
  cross-covariance `P' = [[P, PJᵀ],[JP, JPJᵀ]]`, `J` mapping the clone's δθ/δp from the
  IMU block. A fresh clone is an exact copy of the IMU pose, so the joint covariance
  is **singular at that instant** (PSD, not strictly PD) — correct MSCKF behavior;
  subsequent propagation injects noise and restores full rank.
- **`marginalize_clone(i)`** drops a clone's 6 rows/cols (a principal submatrix of a
  PD matrix stays PD).

## The EKF update

`StateHelper::ekf_update(state, H, r, R_diag)` applies a measurement:

- innovation covariance `S = H·P·Hᵀ + diag(R)`, Kalman gain `K = P·Hᵀ·S⁻¹` (via the
  `spd_solve` Cholesky path);
- **Joseph-form** covariance update `P = (I−KH)P(I−KH)ᵀ + K·diag(R)·Kᵀ`, which keeps
  `P` symmetric positive-definite;
- the error-state correction `δx = K·r` is applied through the manifold **box-plus**
  (orientation via `SO3::exp`, the rest additively), for the nav state and every
  clone.

Preconditions (matching `H`/`r`/`R` dimensions, strictly positive measurement noise)
are asserted.

## Validated

The state machinery's tests check that the covariance stays positive-(semi)definite
through propagation, augmentation, marginalization, and update, and that an update
never increases the covariance trace (an update only ever removes information). These
invariants are the foundation the [camera updater](/vio/camera-updaters/) relies on.
