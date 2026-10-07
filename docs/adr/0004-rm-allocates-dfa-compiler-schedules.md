# ADR 0004 — The Resource Manager allocates; the DFA compiler schedules

**Status:** Accepted
**Date:** 2026-05-21
**Source:** `docs/plan/bootstrap-plan.md`, judgment call #4 (correction recorded there); `docs/arch/cortex-repo.md`, *The All-Knowing Resource Manager*
**Related:** [ADR-0008](0008-host-broker-thin-client-topology.md), [ADR-0009](0009-crash-only-rm-recovery.md)

## Context

The KPU is a spatial dataflow fabric. Tensors are physically distributed
across the compute tiles, and their shape and location are part of how the
KPU computes ("computational spacetime"). Time-slicing the fabric between
operators would destroy the latency and energy efficiency that are its
reason to exist.

Early drafts put "computational-spacetime scheduling" inside the Rust
Resource Manager (RM). That was wrong.

## Decision

Split the responsibilities cleanly:

- **The DFA graph compiler (offline, separate repo)** schedules. It fuses
  subgraphs and emits **domain flow programs**: scheduled, placed entities
  whose spacetime footprint is fixed at compile time.
- **The Rust RM (`core/`)** manages **allocations**: the live map of which
  domain flow programs occupy which physical tiles and memory regions,
  conflict detection, and release on completion.
- **The KPU** runs a loaded domain flow program's dataflow itself. The host
  does no runtime scheduling.

## Consequences

- No scheduler, fusion, or tile-placement logic belongs in `core/`. A
  feature that sounds like scheduling belongs in the compiler.
- Use the term "domain flow programs" (never "streamer programs") for the
  compiler's output.
- The RM's state is small (an allocation map), which is what makes
  crash-only recovery feasible (ADR-0009).
- **Current state:** the RM lifecycle and memory-provider HAL exist; the
  allocation arbiter is issue #23.
