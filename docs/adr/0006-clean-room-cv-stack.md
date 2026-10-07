# ADR 0006 — Clean-room CV stack with an OpenCV-shaped API; no OpenCV

**Status:** Accepted
**Date:** 2026-05-21
**Source:** `docs/plan/bootstrap-plan.md`, judgment call #7 (and its post-audit scope clarification)
**Related:** [ADR-0002](0002-no-gpl-clean-room-policy.md)

## Context

The VIO front end (feature detection and tracking) is straightforward to
write against OpenCV. Accepting OpenCV as a host-only (SITL) dependency, gated
out of the KPU build, was proposed. But OpenCV is very large, and much of the
robotics market already depends on it, so customers will expect familiar APIs.

## Decision

**Do not depend on OpenCV.** Build a small in-house CV stack from our own
requirements, and **shape its API after OpenCV** (`cv::`-like function names
and signatures) so moving from OpenCV to cortex is close to a namespace
replace.

Scope is only what VIO and SLAM need: an image container and I/O, image
pyramids, the FAST detector, a sub-pixel pyramidal KLT tracker, one binary
descriptor (ORB or BRIEF), and a brute-force matcher. That is roughly 3000 lines
of clean-room code. Camera distortion models live in `math/` next to the Lie
groups; calibration loading lives with configuration.

## Consequences

- `cv/` (`branes::cv`) is pixel-type templated where it pays off and has no
  third-party image dependency except `stb_image` for PNG I/O.
- Algorithms come from public papers (Rosten & Drummond for FAST, Bouguet
  for pyramidal KLT, Rublee et al. for ORB), per ADR-0002.
- **Current state:** image, image I/O, pyramid, FAST, and KLT are
  implemented (`cv/include/branes/cv/`). Descriptors and matching land with
  SLAM (epic E4).
