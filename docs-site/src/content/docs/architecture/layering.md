---
title: Layering Invariants
description: The strict, one-directional dependency budget between layers — the load-bearing rule of the codebase.
---

The whole point of the layering is that **algorithms never see middleware, and
middleware never sees hardware.** These invariants are load-bearing: collapsing them
is how a clean perception stack turns into an unportable monolith.

## The dependency budget

| Layer | Path | May depend on | Must **not** depend on |
|---|---|---|---|
| Resource Manager | `core/` (Rust) | OS, kernel drivers, `cxx` | C++ SDK, Zenoh, ROS 2 |
| Math | `math/` (header-only C++20) | Universal, MTL5 | anything heavier |
| Operator SDK | `sdk/` (C++) | `math/`, `core/` via cxx bridge, `cv/`, `std::span` | Zenoh, ROS 2, YAML parsing |
| Daemons | `daemons/` (C++ executables) | `sdk/`, Zenoh, yaml-cpp | (top of stack) |

## The cxx bridge — how tensors cross into Rust

The Rust/C++ boundary is defined in `core/src/lib.rs` under
`#[cxx::bridge(namespace = "branes::core")]`. Tensor metadata crosses as a `repr(C)`
struct — `TensorMetadata { data_ptr, rows, cols, ... }`: a **raw pointer plus shape**,
no Arrow-style metadata, no ownership semantics. The C++ side immediately wraps the
pointer in `std::span<T>` and hands it to math-layer views; **it never copies.**

## Configuration discipline

YAML/JSON are parsed **only at the daemon entry point** into typed C++ POD structs
(`VioConfig`, …). The SDK operators take those structs by value — they never see
`yaml-cpp` types. The Rust layer uses `serde` to parse the *same* config file for
memory-pool sizing. So the config format is shared, but neither the algorithms nor
the math ever link a parser.

## Worked example: the camera updater

A good illustration of the discipline is the MSCKF [camera updater](/vio/camera-updaters/).
It works entirely in **normalized image coordinates** — it never sees camera
intrinsics or a distortion model. Pixel→bearing unprojection happens one layer up (in
`VioEstimator`, which holds the calibration); the estimator core stays
intrinsics-agnostic. Mono and stereo are handled uniformly because an observation
just names its clone and its extrinsic, not a camera struct.

## Why `cv/` is an SDK dependency

The layering table predates the `cv/` directory. `cv/` (FAST, KLT, pyramid, image
I/O) is part of the operator surface: the `VioEstimator` owns the visual front end,
so `branes::sdk` links `branes::cv`. This is consistent with the rule — `cv/` is
algorithm code, not middleware or hardware.

## Enforcement

These boundaries are enforced socially (code review, this doc, `CLAUDE.md`) and
structurally (the CMake target dependencies: `branes::math` is `INTERFACE` linking
only Universal/MTL5; `branes::sdk` links `math` + `cv`; only `daemons/` links Zenoh).
A PR that makes `math/` include a middleware header, or makes an `sdk/` operator
branch on the build target, is a layering violation and gets bounced.
