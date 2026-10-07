# ADR 0010 — `repr(C)` tensor metadata across the Rust/C++ boundary (not Arrow)

**Status:** Accepted
**Date:** 2026-05-20
**Source:** `docs/arch/cortex-repo.md`, *Data structure abstractions* (Arrow vs. C-structs; the zero-copy-to-expression-template pipeline)

## Context

Sensor data and tensors cross from the Rust RM (which owns the DMA
buffers) into the C++ SDK. Apache Arrow was considered: standard zero-copy
interop and self-describing metadata. But Arrow is columnar, while sensor
data is dense tensors and row-based IMU records. Its libraries are large for an
edge target, and its layout and metadata don't map directly onto hardware
register boundaries.

## Decision

Pass tensors as a **plain `repr(C)` metadata struct** through the `cxx`
bridge (`#[cxx::bridge(namespace = "branes::core")]` in
`core/src/bridge.rs`). It carries a raw pointer, size, shape, stride, and dtype,
and nothing else: no Arrow-style schema and no ownership semantics. It is a lightweight
tensor descriptor in the spirit of DLPack.

The C++ side wraps the pointer in a `std::span<T>` immediately and hands
it to MTL5 views. **It never copies.**

## Consequences

- Zero translation layer between DMA buffers and the math; identical
  binary layout on both sides of the FFI.
- The binary layout is the contract. Changing the struct means recompiling
  both sides together; that's fine in a monorepo built by one CMake tree.
- No Arrow dependency. If a data-lake or visualization export ever needs
  Arrow, it belongs at the edge (a daemon or tool), not on the hot path.
- **Current state:** `TensorMetadata { handle, data_ptr, byte_size, rows,
  cols, stride, dtype }` is defined in `core/src/bridge.rs` and exercised by
  `tests/integration/cxx_bridge.cpp`.
