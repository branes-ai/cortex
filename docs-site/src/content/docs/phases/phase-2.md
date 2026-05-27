---
title: Phase 2 — Math Layer
description: The header-only, type-generic C++20 numerics (epic E2).
---

> Status: **complete**. Epic E2 (#3, sub-issues #24–#32) merged.

Phase 2 delivered the **header-only, type-generic** math layer
(`math/include/branes/math/`, namespace `branes::math`) that every estimator is built
from. See the [Math Layer Overview](/math/overview/) for the full surface; this page
is the phase summary.

## What landed

- **Lie groups** — `SO3`, `SE3`, `Sim3` with exp/log, composition, adjoints, and
  left/right Jacobians (with inverses). See [Lie Groups](/math/lie-groups/).
- **Fixed-size primitives** (`lie/detail.hpp`) — `Vec<T,N>`, `Mat<T,R,C>`, `hat`/`vee`,
  `dot`/`cross`/`norm`, and per-scalar math shims (`sqrt_`, `sin_`, `atan2_`, …).
- **Non-linear least squares** — Gauss-Newton and Levenberg-Marquardt with analytical,
  finite-difference-validated Jacobians and golden-value tests. See [NLS](/math/nls/).
- **Camera models** — pinhole+radtan, equidistant fisheye, unified omnidirectional,
  each with project/unproject/distort + analytical Jacobian. See [Cameras](/math/cameras/).
- **Linear algebra** — tensor views, sparse matrices, Krylov inner solvers (MTL5 ITL),
  sparse direct solvers, dense Cholesky/SPD solve.

## Design rules ratified

- **Scalar-generic, never `double`-hardcoded** — everything templates over
  `branes::math::Scalar`; the same code runs in `double`, `float`, and Universal
  posits.
- **Call the shims, not `std::`** — algorithms call `detail::sqrt_` etc. so they
  dispatch correctly per scalar type.
- **Analytical + FD-validated Jacobians** — hand-derived Jacobians are checked against
  finite differences, so higher layers can trust them at runtime.

## Gotchas recorded

- **Universal include order** — include `<number>/<type>.hpp` *before*
  `<number>/mathlib.hpp`, guarded with `// clang-format off` so the include-regroup
  formatter doesn't reorder them and break posit compilation.
- **Braced float-narrowing** — write parenthesized `T(1e-12)` rather than braced
  `T{1e-12}` for sub-epsilon constants, so they instantiate for `float`/posits.

This layer is the foundation for [Phase 3 — VIO](/phases/phase-3/).
