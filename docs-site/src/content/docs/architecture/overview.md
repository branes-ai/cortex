---
title: Architecture Overview & Topology
description: The Host-Broker / Thin-Client topology and how the perception stack is organized.
---

Cortex runs perception on a **KPU** — a spatial dataflow accelerator — with a host
CPU acting as a resource broker. The runtime topology is **Host Broker + Thin
Client**.

## Host Broker + Thin Client

There is exactly **one Resource Manager process** with access to `/dev/kpu`. It owns
the live map of which compiled dataflow programs occupy which physical tiles and
memory regions, performs conflict detection, and releases resources on completion.

Operator containers (the SDK consumers and daemons) are **unprivileged thin
clients**. They never touch hardware directly. They request execution over a Unix
Domain Socket + POSIX shared-memory IPC channel, and the broker places the work.

```text
   ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
   │ vio_daemon  │   │ slam_daemon │   │   …         │   unprivileged
   │ (thin client)│  │ (thin client)│  │ thin clients│
   └──────┬──────┘   └──────┬──────┘   └──────┬──────┘
          │  UDS + POSIX shm (request / replay)
          ▼                 ▼                 ▼
   ┌──────────────────────────────────────────────────┐
   │  Resource Manager (Rust)   ── the ONE broker      │
   │  /dev/kpu · tile+memory allocation map · conflict  │
   │  detection · release-on-completion                 │
   └──────────────────────────────────────────────────┘
                         │
                         ▼  domain flow programs
                   ┌───────────┐
                   │    KPU    │   spatial dataflow fabric
                   └───────────┘
```

## The four surfaces

The architecture deliberately exposes four "surfaces", from hardware up:

1. **Resource Manager (`core/`, Rust).** Owns the HAL and the allocation map. The
   only privileged component. See [Resource Manager](/architecture/resource-manager/).
2. **Math (`math/`, header-only C++20).** Pure numerics — no I/O, no middleware, no
   hardware. See the [Math Layer](/math/overview/).
3. **Operator SDK (`sdk/` + `cv/`, C++20).** Builds algorithms (VIO, SLAM) on the
   math layer; talks to the broker via the cxx bridge. See [VIO](/vio/overview/).
4. **Daemons (`daemons/`, C++ executables).** The application surface: subscribe to
   Zenoh, parse YAML config, call the SDK, publish results. Thin by construction.

The strict dependency budget between these surfaces is the subject of
[Layering Invariants](/architecture/layering/).

## Two decisions worth knowing up front

- **The fabric is treated as stateless.** Recovery from a broker crash is by **client
  replay**, not checkpointing: the SDK's IPC layer enters a brief "holding pattern"
  on socket close, then replays pending requests when the broker respawns. There are
  no disk-based checkpoint paths.

- **No dynamic reconfiguration.** Parameter changes go through a managed lifecycle
  (`Unconfigured → Inactive → Active → teardown`). The hot path has no mutex-guarded
  mutable parameters — an explicit decision to preserve real-time determinism. The
  `VioEstimator` lifecycle (see [VioEstimator](/vio/vio-estimator/)) mirrors this.

## Delivery

Cortex is delivered two ways: as **OCI containers** (the thin clients) and via a
**Yocto** layer (the broker + system integration). The dual strategy keeps operator
development container-fast while the production image is a reproducible embedded
build.
