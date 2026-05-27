---
title: Introduction
description: What Cortex is, the problem it solves, and the design philosophy behind its strict layering.
---

**Cortex** is the perception core of the Branes Embodied AI framework — a real-time
**visual-inertial odometry (VIO)** and SLAM stack targeting the **KPU**, a spatial
dataflow accelerator, with a host CPU acting as a thin resource broker.

## The problem

Embodied AI has a property datacenter AI does not: **latency and energy are
functional requirements**, not optimization targets. A perception pipeline that
cannot estimate pose at frame rate, within a power envelope, simply fails its
mission. Cortex is built so those constraints are first-class:

- The numerics are **type-generic** — the same estimator compiles in `double`,
  `float`, or Universal posits, so mixed-precision trade-offs are a compile-time
  choice, not a rewrite.
- The hardware boundary is a **Hardware Abstraction Layer** in Rust, so the same
  algorithms run host-emulated (SITL) or on real KPU silicon.
- Efficiency is **validated in CI** — trajectory accuracy (ATE/RPE) and a per-frame
  latency budget are enforced, not assumed.

## The design philosophy: strict layering

The single most load-bearing idea in Cortex is that **algorithms never see
middleware, and middleware never sees hardware.** Each layer has a strict,
one-directional dependency budget:

| Layer | Path | May depend on | Must **not** depend on |
|---|---|---|---|
| Resource Manager | `core/` (Rust) | OS, kernel drivers, `cxx` | C++ SDK, Zenoh, ROS 2 |
| Math | `math/` (header-only C++20) | Universal, MTL5 | anything heavier |
| Operator SDK | `sdk/` (C++) | `math/`, `core/` via cxx, `cv/` | Zenoh, ROS 2, YAML |
| Daemons | `daemons/` (C++) | `sdk/`, Zenoh, yaml-cpp | (top of stack) |

This is why, for example, the MSCKF estimator works in normalized image
coordinates and never touches a camera-intrinsics struct from a config file: the
intrinsics belong to a higher layer. The discipline is what lets the hardware
target swap underneath without disturbing the math.

## What's built so far

Cortex is being built phase by phase against a 12-phase roadmap. Phases 0–3 are
complete:

- **[Phase 0 — Foundation](/phases/phase-0/):** build system, CI, release flow.
- **[Phase 1 — Rust Core](/phases/phase-1/):** the `MemoryProvider` HAL + cxx bridge + lifecycle state machine.
- **[Phase 2 — Math](/phases/phase-2/):** header-only Lie groups, NLS solvers, camera models.
- **[Phase 3 — VIO](/phases/phase-3/):** the full MSCKF visual-inertial odometry pipeline.

This documentation site is the consolidated record of those phases: **what was
built, which algorithms, and the performance measured.**

## Next steps

- [Build & Run](/getting-started/build-and-run/) — get a SITL build going.
- [Architecture Overview](/architecture/overview/) — the Host-Broker / Thin-Client topology.
- [VIO Overview](/vio/overview/) — the estimator pipeline, end to end.
