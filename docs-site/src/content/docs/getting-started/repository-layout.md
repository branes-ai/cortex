---
title: Repository Layout
description: Where everything lives in the Cortex repository, and the rule for what belongs in each directory.
---

```text
branes-ai/cortex/
├── core/          # Rust Resource Manager (HAL, memory, KPU sync)
│   └── src/
│       ├── lib.rs            # #[cxx::bridge] FFI surface
│       └── memory_provider.rs # SITL-vs-KPU HAL trait
├── math/          # header-only C++20 numerics (branes::math)
│   └── include/branes/math/
├── cv/            # CV front-end primitives (branes::cv)
│   └── include/branes/cv/
├── sdk/           # VIO / SLAM / scene-graph operators (branes::sdk)
│   └── include/branes/sdk/
├── daemons/       # one thin executable per operator (Zenoh + YAML + SDK)
│   └── src/
├── config/        # unified YAML schemas (read by both Rust serde and C++ yaml-cpp)
├── tests/         # Catch2 tests + integration tests
├── docs/          # architecture, ADRs, session logs, plans
├── docs-site/     # this Starlight + Doxygen documentation site
└── CMakeLists.txt # the master build
```

## What lives where

- **`core/`** — the Rust Resource Manager. `memory_provider.rs` is the SITL-vs-KPU
  HAL trait; the cxx bridge is defined in `core/src/lib.rs` under
  `#[cxx::bridge(namespace = "branes::core")]`. Expand the HAL here, not in C++.

- **`math/include/branes/math/`** — header-only numerics: Lie groups (SO3/SE3/Sim3),
  non-linear least squares (Gauss-Newton, Levenberg-Marquardt), Krylov inner solvers,
  camera models. First-class support for custom arithmetic — **do not hardcode
  `double`**.

- **`cv/include/branes/cv/`** — the visual front end: image container + PGM/PNG I/O,
  Gaussian pyramid, FAST corner detector, pyramidal KLT tracker.

- **`sdk/include/branes/sdk/`** — the operators: IMU preintegration, feature
  representations, the MSCKF state machinery, camera updaters, the backends, and the
  top-level `VioEstimator`. The 3D-scene-graph backing store (future) must be
  contiguous arrays indexed by integer node IDs — data-oriented, not pointer-chasing
  heap nodes.

- **`daemons/src/`** — one thin executable per operator (`vio_daemon.cpp`, …). They
  subscribe to Zenoh, deserialize config, call the SDK, publish results. **No
  algorithm code here.**

- **`config/`** — unified YAML schemas consumed by *both* Rust (`serde`, for
  memory-pool sizing) and C++ (`yaml-cpp`, parsed once at the daemon entry point into
  typed POD structs).

## Namespaces

Code uses `branes::` — `branes::math`, `branes::sdk`, `branes::cv`, `branes::core`.
(Some older prose in the design docs says `cortex::`; the code is authoritative —
it's `branes::`.)

## The authoritative design record

`docs/arch/cortex-repo.md` is the canonical architecture document. When the code is
ambiguous, it wins. This site distills it; that file is the source of truth.
