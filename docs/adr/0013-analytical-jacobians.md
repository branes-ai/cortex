# ADR 0013 — Analytical Jacobians only (no automatic differentiation, for now)

**Status:** Accepted
**Date:** 2026-05-21
**Source:** `docs/plan/bootstrap-plan.md`, judgment call #8

## Context

The non-linear least-squares and filter updates need Jacobians. Automatic
differentiation (AD) is convenient and hard to get wrong, but adds
significant template machinery and scope, and interacts with the
arithmetic-type genericity of the math layer
([ADR-0015](0015-type-generic-arithmetic.md)). Analytical Jacobians are
faster and standard in production VIO, but easier to get wrong.

## Decision

**Analytical Jacobians only, for now.** No AD framework in `math/` or
`sdk/`.

## Consequences

- Each Jacobian is derived by hand, documented with its derivation or
  citation, and **checked numerically** against finite differences in the
  tests. That check is what makes hand-derived Jacobians safe.
- Jacobian conventions (left/right perturbation, ordering of the error
  state) must be stated and kept consistent across the Lie-group layer and
  the backends.
- Revisit if the operator count grows to where hand derivation becomes the
  bottleneck. AD would then be a scoped addition, not a rewrite.
