---
title: Non-linear Least Squares
description: The Gauss-Newton and Levenberg-Marquardt solvers that underpin estimation and calibration.
---

`nls.hpp` + `nls/{dense_linalg,solver}.hpp` provide the iterative non-linear
least-squares machinery used across calibration, triangulation refinement, and the
estimator's internal solves. Like the rest of the layer, it is header-only and
templated over `branes::math::Scalar`.

## What it solves

Given a residual function **r(x)** and its Jacobian **J = ∂r/∂x**, minimize
‖r(x)‖². Two methods are provided:

- **Gauss-Newton** — solves the normal equations `JᵀJ δ = −Jᵀ r` each iteration and
  steps `x ← x ⊞ δ`.
- **Levenberg-Marquardt** — damps the normal equations, `(JᵀJ + λ·diag) δ = −Jᵀ r`,
  adapting λ between gradient-descent-like and Gauss-Newton-like behavior for
  robustness far from the optimum.

The `⊞` (box-plus) retraction matters because the state can live on a manifold (a
pose in SE(3)), not a flat vector space — increments are applied through the Lie
exponential, not naive addition.

## Options

The solver takes a small typed `Options` POD (max iterations, function/step
tolerances, damping schedule). Tolerances are written in a scalar-relative form (e.g.
`T(1e-12)` rather than a braced `T{1e-12}`) so they instantiate correctly for `float`
and Universal posits as well as `double`.

## Linear inner solve

`nls/dense_linalg.hpp` carries the dense building blocks (Cholesky factorization and
SPD solve) for the normal equations. For larger sparse systems the layer also offers
Krylov inner solvers (`krylov.hpp`, via MTL5 ITL) and sparse direct solvers
(`sparse_direct.hpp`).

## Correctness

The NLS solvers ship with **golden-value tests** — known problems with known optima —
and the analytical Jacobians used throughout are validated against finite differences.
This is what lets higher layers trust a hand-derived Jacobian (for example, the camera
[reprojection Jacobians](/vio/camera-updaters/)) instead of paying for numerical
differentiation at runtime.
