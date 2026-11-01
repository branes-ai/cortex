# ADR 0002 — No GPL code; clean-room implementation from papers

**Status:** Accepted (amended 2026-10-31: scope, the structure allowance, weak copyleft rejected)
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
dependency, not in a test target, not as a build-time tool in what cortex
builds, and not as a ground-truth oracle. The scope is what cortex ships,
the code that builds it and the tests; the docs site's toolchain is not
part of it (see the amendment below).

- Operators are implemented **clean-room from papers and documentation**,
  plus permissively licensed (MIT / BSD / Apache-2.0) references.
- **Reading GPL source and then "writing from scratch" is not clean-room.**
  Check a reference's license *before* opening its code. A GPL project's
  structure may be consulted, meaning its directory layout and how it splits
  the problem into modules (e.g. sensor-modular decomposition), and nothing
  else: never its code, algorithm details or data (tuning tables, gains,
  coefficients, vocabularies, weights).
- **Test oracles are in-house:** synthetic trajectories, landmarks, and IMU
  noise models (`sdk/include/branes/sdk/eval/synthetic_world.hpp`), plus
  published-paper numerical thresholds. Never compare against GPL binaries.
- **LGPL, MPL, AGPL and SSPL sources are rejected,** as dependencies and as
  references to port from, as ADR-0001 rejects them. An exception would need
  an ADR of its own that supersedes this one.
- **Amendment (2026-10-31):** the three rules above replace looser wording
  ("no GPL contact", "structure only", LGPL / MPL / SSPL "flagged for review
  and default to rejected"), to match the same policy as recorded in
  synapse, reflex, anatomy, racing-drone and scout-drone. The scope rule
  matters for the docs site: Astro brings in sharp (Apache-2.0), and sharp
  brings prebuilt libvips binaries, the `@img/sharp-libvips-*` packages
  (LGPL-3.0-or-later). They only build the documentation, in CI and on
  developers' machines, and are not part of anything cortex ships.

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
