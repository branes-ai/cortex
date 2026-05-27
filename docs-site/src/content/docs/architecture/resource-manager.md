---
title: Resource Manager (Rust)
description: The MemoryProvider HAL, the SITL/KPU split, the cxx bridge, and the lifecycle state machine — what Phase 1 delivered.
---

The Resource Manager (`core/`, Rust) is the only privileged component and the one
place hardware is visible. Phase 1 delivered its SITL-only form: a strictly typed
memory-allocation surface exposed to C++ via the cxx bridge, plus a lifecycle state
machine.

## The `MemoryProvider` HAL

A single object-safe trait is the allocate / release / lock surface every backend
implements:

- **`SitlMemoryProvider`** — host emulation using POSIX shared memory + `mmap`, with
  a heap fallback. This is what the default `BUILD_TARGET_KPU=OFF` build uses.
- **`KpuMemoryProvider`** — the real kernel-driver backend. In the current phase it
  is a **stub** that returns `Err(NotYetImplemented)` for every call; the KPU
  kernel-driver work is deferred.

Both implementations compile into the binary; the active one is chosen **at
construction time** from `RUST_HAL_TARGET` (`SITL` or `KPU`), which the CMake preset
sets. Nothing above the HAL branches on the target.

Buffers are referenced by an opaque `BufferHandle` (a `#[repr(transparent)]` `u64`
whose inner field is private, so external callers cannot fabricate handles). A C++
caller requests a buffer of a given size/alignment, locks it to get a raw pointer +
`TensorMetadata`, writes, and on release the bytes are visible from Rust.

## The cxx bridge

The FFI is defined in `core/src/lib.rs` under
`#[cxx::bridge(namespace = "branes::core")]`, with headers generated at build time by
`build.rs`. Tensor metadata crosses as a `repr(C)` `TensorMetadata` — a raw pointer
plus shape, no ownership. The C++ side wraps it in `std::span<T>` without copying.
(See [Layering Invariants](/architecture/layering/).)

## The lifecycle state machine

Operators are walked through a managed lifecycle, and invalid transitions return
**typed errors — they never panic**:

```text
Unconfigured ──configure──▶ Inactive ──activate──▶ Active
                              ▲                       │
                              └──────teardown─────────┘
```

This is the determinism guarantee: parameters are fixed at `configure`, and the hot
path (`Active`) has no reconfiguration. The C++ `VioEstimator`
([VioEstimator API](/vio/vio-estimator/)) mirrors these exact states, so the managed
lifecycle is consistent from the Rust broker up to the estimator façade.

## What's deferred

The KPU-facing pieces are parked behind the `phase-soc-deferred` label: the IPC broker
over UDS + POSIX shm, the crash-only recovery / client-replay protocol, and the KPU
resource-allocation arbiter (live tile/memory map + conflict detection). The Phase 1
RM is SITL-complete and unblocks the math and VIO layers above it.

## Quality gates

The Rust core is held to **≥85% line coverage** (cargo-llvm-cov, enforced in CI) and
passes a C++/Rust integration test that exercises the bridge end-to-end. See
[Coverage & CI Gates](/benchmarks/ci/).
