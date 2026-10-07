# ADR 0016 — SITL vs. KPU is one build flag, confined to the Rust HAL

**Status:** Accepted
**Date:** 2026-05-21
**Source:** `docs/arch/cortex-repo.md`, *Abstracting the Memory Source in Rust* and the master CMake build; `CLAUDE.md`
**Related:** [ADR-0008](0008-host-broker-thin-client-topology.md)

## Context

Development and CI run on host x86 (software-in-the-loop, SITL) long before
silicon exists. The real target uses kernel-driver DMA on the KPU. If the
hardware distinction leaks into algorithms, every operator grows `#ifdef`
branches and the SITL build stops testing the code that ships.

## Decision

- **One CMake option, `BUILD_TARGET_KPU`** (default `OFF`), selects the
  target. It defines `TARGET_KPU` / `TARGET_SITL` for C++ and exports
  `RUST_HAL_TARGET=KPU|SITL` to the Rust build.
- **The distinction is confined to the Rust `MemoryProvider` HAL**
  (`core/src/memory_provider.rs`): a SITL provider emulates DMA memory with
  POSIX shared memory, and a KPU provider talks to the kernel driver.
- **Operators (`sdk/`), math (`math/`), and CV (`cv/`) never branch on the
  build target.**

## Consequences

- The SITL build exercises exactly the operator code that ships on the KPU.
- Hardware-specific work goes into `core/`; extend the HAL trait there, not
  in C++.
- The cross-compile preset (`kpu-cross`) builds the same tree for aarch64.
