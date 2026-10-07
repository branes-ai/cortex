# ADR 0012 — MSCKF first, behind a pluggable VIO backend interface

**Status:** Accepted
**Date:** 2026-05-21
**Source:** `docs/plan/bootstrap-plan.md`, judgment call #2

## Context

The two VIO families trade differently. The MSCKF filter (OpenVINS/MINS
lineage) is fast with bounded state. Sliding-window optimization
(VINS-Fusion/ORB-SLAM3 lineage) is more accurate but costs more compute.
Committing to one risks building assumptions into the whole stack.

## Decision

- **Start with MSCKF** as the primary backend.
- **Define the backend as an interface** (`VioBackend`) shared by all
  backends behind one front end, and ship a **sliding-window
  optimization skeleton** alongside. It implements the interface but does
  no estimation yet, proving the abstraction is real and substitutable before
  MSCKF-specific assumptions creep in.

## Consequences

- `VioEstimator<T, Backend>` is templated on the backend; new algorithms
  plug in without touching the front end or the consumers.
- The MSCKF backend is the validated path (EuRoC accuracy gates in
  `tests/sdk/vio_euroc.cpp`; consistency instrumentation in
  `branes/sdk/eval/`). An invariant (R-IEKF) MSCKF variant is in development
  (`msckf/invariant_vio_backend.hpp`); wiring it to the full `VioBackend`
  interface is a follow-up.
- **Current state:** `msckf_backend.hpp` is implemented;
  `sliding_window_backend.hpp` is the skeleton. Its real math lands
  post-MVP.
