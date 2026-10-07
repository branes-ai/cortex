# ADR 0002 — No GPL contact; clean-room implementation from papers

**Status:** Accepted
**Date:** 2026-05-21
**Source:** `docs/plan/bootstrap-plan.md`, judgment call #3
**Related:** [ADR-0001](0001-third-party-license-compatibility.md) (dependency licenses), [ADR-0006](0006-clean-room-cv-stack.md)

## Context

The reference implementations for VIO and SLAM are mostly GPL-licensed:
OpenVINS and MINS (GPL-3.0), ORB-SLAM3, DBoW3, and the GPL parts of the
Sophus/g2o ecosystem. Vendoring OpenVINS as a test-only oracle was proposed,
since a separate test target would not link GPL code into the SDK.

cortex is meant to be redistributed by OEM partners (AV makers, robotics
vendors) inside their own certified products. Any GPL obligation, even one
that seems confined to tests, is something those partners' legal reviews
would have to clear.

## Decision

**No GPL-licensed code anywhere, for any purpose.** Not as an SDK
dependency, not in a test target, not as a build-time tool, and not as a
ground-truth oracle.

- Operators are implemented **clean-room from papers and documentation**,
  plus permissively licensed (MIT / BSD / Apache-2.0) references.
- **Reading GPL source and then "writing from scratch" is not clean-room.**
  Check a reference's license *before* opening its code. GPL projects may be
  consulted for *structure* only (directory layout, sensor-modular
  decomposition), never for code.
- **Test oracles are in-house:** synthetic trajectories, landmarks, and IMU
  noise models (`sdk/include/branes/sdk/eval/synthetic_world.hpp`), plus
  published-paper numerical thresholds. Never compare against GPL binaries.
- LGPL / MPL / SSPL candidates are flagged for explicit review and default
  to rejected (see ADR-0001 for the dependency matrix).

## Consequences

- Every operator costs more to build: the math has to come from the
  literature, and we have to build our own simulators to validate it.
- Each algorithm records its citations so provenance can be audited (docs
  site: *Algorithms & Provenance → Clean-room Citations*).
- OEMs can redistribute cortex under MIT terms with no copyleft review.

## Enforcement

- PR review and the CodeRabbit path instructions flag forbidden references
  (OpenVINS, MINS, ORB-SLAM3, DBoW3, Sophus/g2o) and non-permissive
  FetchContent dependencies.
- Issues touching provenance carry the `clean-room` / `no-gpl` labels.
