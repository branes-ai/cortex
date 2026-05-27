---
title: Camera Updaters
description: Feature triangulation, null-space marginalization, Mahalanobis gating, and the EKF measurement update.
---

`CameraUpdater<T>` (`sdk/include/branes/sdk/msckf/camera_updater.hpp`, #44) is the
MSCKF camera measurement update: it turns a feature tracked across a window of cloned
poses into a state correction. Clean-room from the MSCKF measurement model (Mourikis
& Roumeliotis, 2007).

It works entirely in **normalized image coordinates** — camera intrinsics and
distortion are removed by the front end — which keeps it intrinsics-agnostic. Mono
and stereo are handled uniformly: an observation names the clone it was taken in and
the camera (extrinsic) it came through, so a stereo feature simply contributes
observations through two extrinsics that share the clone poses.

## What `update` does, for one feature track

1. **Triangulate** the landmark — a linear ray-perpendicular (DLT-style) solve across
   all observing clones, followed by a few Gauss-Newton reprojection refinements.
   Tracks that triangulate behind a camera are dropped.
2. **Form the stacked residual and Jacobians** in normalized coordinates: the
   reprojection residual `r`, the feature Jacobian `H_f` (∂h/∂p_f), and the state
   Jacobian `H_x` — per clone, `∂h/∂δθ` and a `δp` block equal to `−H_f`, in the
   right-perturbation convention of the [state](/vio/msckf-state/).
3. **Marginalize the feature** by left-null-space projection: left-multiply by `Nᵀ`
   whose columns span the left null space of `H_f` (so `Nᵀ H_f = 0`), computed by a
   Householder QR of `H_f`. The compressed system `Nᵀr = Nᵀ H_x δx + Nᵀn` no longer
   depends on the feature. Because the reflectors are orthonormal, the projected
   measurement noise stays `σ²·I`.
4. **Gate** the innovation with a **Mahalanobis** test, `γ = rᵀ(H_x P H_xᵀ + R)⁻¹ r`,
   rejecting tracks that exceed the threshold (a `χ²` quantile, configurable).
5. **Apply** the surviving constraint via `StateHelper::ekf_update`.

## Configuration & safety

`CameraUpdaterOptions` carries `normalized_sigma` (the measurement σ in normalized
coordinates — *not* pixels, deliberately named so daemon config can't pass pixel-space
values) and the gating threshold. The constructor rejects an empty calibration list,
and an out-of-range `camera_id` is rejected rather than silently aliased to camera 0.

## Validated

The end-to-end test builds a window of clones at known poses, places synthetic
features, and checks that:

- a camera update **reduces the covariance and keeps it PSD**;
- a single-observation track and a behind-the-camera track are **rejected**;
- the filter **stays stable with a well-conditioned innovation over 1000 consecutive
  updates** — the covariance trace is monotonically non-increasing per step (an
  update only removes information), and every innovation factorizes (stays PD).

This 1000-step stability result is the acceptance bar from issue #44.
