# ADR 0003 — Hybrid SDK: header-only templated operators, compiled glue

**Status:** Accepted
**Date:** 2026-05-21
**Source:** `docs/plan/bootstrap-plan.md`, judgment call #1
**Related:** [ADR-0015](0015-type-generic-arithmetic.md)

## Context

The SDK could be fully header-only (every operator a template, no
library) or a conventional compiled library. Header-only keeps every
operator generic in its arithmetic type, which mixed-precision work needs.
It costs compile time and leaves no ABI boundary. A compiled library is the
reverse.

## Decision

**Hybrid.**

- **Core operators are header-only templates parameterized on the
  arithmetic type** (`VioEstimator<T, Backend>`, the MSCKF backend, the math
  layer), so the same code instantiates in `double`, `float`, or Universal
  posits/cfloat/fixed-point for mixed-precision optimization.
- **Non-template glue is compiled and consumed as a library:** file loading
  and unloading, calibration I/O, resource-management plumbing, and anything
  else that is not naturally parameterized on a scalar type.

## Consequences

- Arithmetic type is a first-class template parameter throughout `math/`
  and `sdk/`; code there must not hardcode `double`.
- Compile times are higher for template-heavy translation units; sccache
  absorbs the CI cost ([ADR-0011](0011-dependencies-via-fetchcontent.md)).
- **Current state:** everything so far is template code, so `branes::sdk`
  is still an `INTERFACE` target (`sdk/CMakeLists.txt`). It gains a compiled
  `STATIC` part when the first non-template glue lands.
