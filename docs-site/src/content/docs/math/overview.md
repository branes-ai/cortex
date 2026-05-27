---
title: Math Layer Overview
description: The header-only, type-generic C++20 numerics that the estimators are built from (epic E2).
---

The math layer (`math/include/branes/math/`, namespace `branes::math`) is
**header-only, C++20, and type-generic**. Every primitive is templated over a scalar
that satisfies the `branes::math::Scalar` concept, so the same code compiles in
`double`, `float`, or a [Universal](https://github.com/stillwater-sc/universal) posit
for mixed-precision study. **Nothing hardcodes `double`.**

It depends only on Universal (custom arithmetic) and [MTL5](https://github.com/stillwater-sc/mtl)
(matrix template library) — and on nothing heavier. No I/O, no middleware, no
hardware awareness.

## What it provides

| Area | Header(s) | Summary |
|---|---|---|
| Lie groups | `lie/{so3,se3,sim3}.hpp` | SO(3), SE(3), Sim(3) with exp/log, composition, adjoints, left/right Jacobians. See [Lie Groups](/math/lie-groups/). |
| Lie detail | `lie/detail.hpp` | Fixed-size `Vec<T,N>` / `Mat<T,R,C>`, `hat`/`vee`, scalar shims (`sqrt_`, `sin_`, `atan2_`, …) that dispatch to the right overloads per scalar type. |
| Non-linear least squares | `nls.hpp`, `nls/{dense_linalg,solver}.hpp` | Gauss-Newton and Levenberg-Marquardt with analytical Jacobians. See [NLS](/math/nls/). |
| Camera models | `cameras.hpp`, `cameras/*` | Pinhole+radtan, equidistant fisheye, unified omnidirectional — project/unproject/distort + analytical Jacobians. See [Cameras](/math/cameras/). |
| Linear algebra | `tensor.hpp`, `sparse.hpp`, `krylov.hpp`, `sparse_direct.hpp`, `arithmetic.hpp` | Tensor views, sparse matrices, Krylov inner solvers (via MTL5 ITL), sparse direct solvers. |

## Design choices that recur

- **Scalar-generic, with shims.** Free functions like `branes::math::lie::detail::sqrt_`
  exist so a single algorithm body works whether `T` is `double` (calls `std::sqrt`)
  or a Universal type (calls the type's `sqrt`). Algorithms call the shim, never
  `std::sqrt` directly.
- **Analytical Jacobians, finite-difference-validated.** Where a Jacobian is derived
  by hand, a test checks it against a numerical finite-difference of the function.
- **Fixed-size where the dimension is known.** Poses and small blocks use compile-time
  `Vec`/`Mat`; only runtime-sized state (e.g. the growing MSCKF covariance) uses a
  dynamic matrix.
- **Clean-room.** Everything is implemented from the mathematical definitions and
  published papers — never copied from GPL or other restrictively-licensed source.
  See [Algorithms & Provenance](/algorithms/citations/).

## Status

Epic E2 (Phase 2) is complete: the surface above is in place and unit-tested,
including golden-value tests for the NLS solvers. It is the foundation the
[VIO layer](/vio/overview/) is built on.
