---
title: Lie Groups (SO3 / SE3 / Sim3)
description: The rotation, rigid-motion, and similarity groups used to represent and perturb poses on-manifold.
---

`lie/{so3,se3,sim3}.hpp` implement the matrix Lie groups that pose estimation lives
on. Representing rotations and rigid motions as group elements — and perturbing them
through the Lie algebra — is what keeps the estimator's increments well-defined and
its covariance meaningful.

## SO(3) — rotations

`SO3<T>` stores a **unit quaternion** `(w, x, y, z)` and provides:

- `exp(φ)` / `log()` — the exponential and logarithm between a rotation vector
  `φ ∈ ℝ³` (the algebra `so(3)`) and the group;
- composition (`*`), `inverse()`, `matrix()` (the 3×3 rotation), and `adjoint()`
  (which for SO(3) is the rotation matrix itself);
- the **left and right Jacobians** `Jl(φ)`, `Jr(φ)` and their inverses, used to
  propagate covariance and to relate perturbations on either side of a rotation.

The convention used across the estimator is the **right perturbation**
`R ← R · Exp(δθ)`, which the [MSCKF state](/vio/msckf-state/) and the
[camera updaters](/vio/camera-updaters/) follow consistently.

## SE(3) — rigid motions

`SE3<T>` is a `SO3<T>` rotation plus a translation 3-vector. Its algebra is a
6-vector `ξ = [ρ; φ]` with the **translation part first** (the Solà / Barfoot
convention). It provides `exp`/`log`, composition, `inverse()`, `rotation()`,
`translation()`, and the 4×4 / 6×6 matrix and adjoint forms. Poses
(`T_world_imu`, camera extrinsics, relative motions) are all SE(3) elements.

## Sim(3) — similarities

`Sim3<T>` adds a scale to SE(3). It exists for the SLAM layer (loop closure and map
alignment recover a similarity, not just a rigid transform) and for the
scale-ambiguous monocular cases. The VIO layer is metric and uses SE(3).

## `detail.hpp` — the shared primitives

All three groups sit on `lie/detail.hpp`: fixed-size `Vec<T,N>` and `Mat<T,R,C>`
(row-major, `constexpr`), `hat`/`vee` (the skew-symmetric ↔ vector maps), `dot`,
`cross`, `norm`, `transpose`, matrix/vector products, and the per-scalar math shims
(`sqrt_`, `sin_`, `cos_`, `atan2_`, `exp_`, `log_`, `abs_`). These are the same
fixed-size types the rest of the SDK builds with.
