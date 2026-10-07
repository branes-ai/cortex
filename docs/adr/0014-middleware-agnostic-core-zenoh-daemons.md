# ADR 0014 — Middleware-agnostic core; Zenoh daemons as the black-box surface

**Status:** Accepted
**Date:** 2026-05-20
**Source:** `docs/arch/cortex-repo.md`, *Messaging middleware*, *Surface 1/Surface 2*, *The Configuration Hierarchy*
**Related:** [ADR-0005](0005-managed-lifecycle-no-dynamic-reconfiguration.md)

## Context

Customers come in two kinds. Algorithm developers want a C++ SDK they can
integrate and extend. Application composers want perception as a utility
they subscribe to. Robotics middleware also varies: native Zenoh for minimum
latency, or ROS 2 (where Zenoh is now a Tier-1 RMW). Hardware-accelerated
stacks such as NVIDIA Isaac ROS keep the core operators independent of any
middleware.

## Decision

**Two surfaces from one core, with strict layering:**

- **Surface 1, the C++ SDK ("white box"):** pure compute. It takes
  `std::span` views of memory and typed config structs and returns results. It
  has **zero knowledge of Zenoh, ROS 2, or YAML**.
- **Surface 2, the Zenoh daemons ("black box"):** thin executables (one
  per operator, e.g. `cortex-vio-daemon`). Each subscribes to sensor topics,
  calls the SDK, and publishes results (e.g. `cortex/state/pose`). The daemons
  are the SDK's first customers: an SDK fix fixes the daemons on the next build.
- **Configuration:** YAML/JSON is parsed **only at the daemon (or
  application) entry point** into typed POD structs (`VioConfig`, …), which
  the SDK takes by value. The Rust RM parses the same file with `serde` for
  memory-pool sizing.
- **ROS 2** support is an adapter on top of the same SDK, not a dependency
  of it.

## Consequences

- Dependency rules are enforced by layer (see `CONTRIBUTING.md`): `sdk/`
  must not depend on Zenoh, ROS 2, or yaml-cpp; `daemons/` holds no algorithm
  code.
- Zero-copy egress for large payloads (images, scene graphs) belongs in the
  daemon layer (issue #82).
- **Current state:** the SDK surface is implemented, and `examples/hello_vio` is
  its reference consumer. The daemons are epic #75, and `examples/hello_zenoh`
  (#104) depends on them.
