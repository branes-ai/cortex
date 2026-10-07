# ADR 0015 — Arithmetic type is a first-class template parameter

**Status:** Accepted
**Date:** 2026-05-20
**Source:** `docs/arch/cortex-repo.md`, *Data structure abstractions* (MTL5) and *Non-linear Least Squares*; `CLAUDE.md`
**Related:** [ADR-0003](0003-hybrid-sdk-header-only-operators-compiled-glue.md), [ADR-0013](0013-analytical-jacobians.md)

## Context

The KPU targets mixed-precision arithmetic: number formats chosen per
operator for energy and accuracy, not locked to IEEE-754 `double`. Studying
and deploying that requires the same algorithm code to run in many number
systems. MTL5 (linear algebra) and Universal (posits, cfloat, fixed-point,
LNS, …) are designed for exactly this.

## Decision

- Math (`branes::math`) and SDK operators are **templated on the scalar
  type** and constrained by concepts (`LinearAlgebraScalar`,
  `OrderedField`, …), never hardcoded to `double` or `float`.
- **MTL5** is the linear-algebra layer (dense/sparse types, ITL Krylov
  solvers, sparse direct solvers); **Universal** supplies the number
  systems. Non-linear least squares is a header-only addition on top of MTL5.
- C++20 is required (concepts, `std::span`) and must not be lowered.

## Consequences

- New math must instantiate for the validated arithmetic set (IEEE
  `double`/`float`, Universal posit/cfloat/fixed-point). The type-smoke and
  Kalman-filter-over-posits tests catch regressions when MTL5 or Universal is
  bumped.
- Generic code may only use operations the concepts guarantee. Gaps in
  upstream genericity are fixed upstream (e.g. stillwater-sc/mtl5#538).
- Compile times are higher (ADR-0003, mitigated by sccache).
