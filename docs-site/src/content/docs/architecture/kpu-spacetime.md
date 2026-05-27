---
title: The KPU Spacetime Constraint
description: Why the KPU is a spatial dataflow fabric, not a time-sliced accelerator, and what that means for this repo.
---

The KPU is a **spatial dataflow fabric**, not a time-sliced accelerator. Tensors are
physically distributed across the compute tiles; **shape and location *are* part of
the schedule**, baked in offline by a DFA graph compiler (a separate repo, not
`cortex`). That compiler emits **domain flow programs** — scheduled, fused subgraphs
— which the Rust Resource Manager places on allocated hardware. Once a domain flow
program is loaded, the KPU runs its own dataflow internally; **no runtime scheduling
happens on the host.**

This has three concrete consequences for any code in this repository.

## 1. The Resource Manager manages *allocations*, not *schedules*

The RM's job is the live map of which domain flow programs occupy which physical
tiles and memory regions, conflict detection, and release-on-completion. **Scheduler
logic does not belong here** — it lives in the DFA compiler. The RM never decides
*when* a tile computes; it decides *whether* a program may be placed.

## 2. Exactly one process owns the hardware

There is one Resource Manager with access to `/dev/kpu`. Operator containers are
unprivileged and request execution via UDS + POSIX shared memory. An SDK consumer or
daemon is **never** given direct hardware access. (See
[Overview & Topology](/architecture/overview/).)

## 3. The fabric is treated as stateless

Recovery from an RM crash is by **client replay**, not checkpointing. The SDK's IPC
layer enters a brief "holding pattern" on socket close and replays pending requests
when the RM respawns. There are no disk-based checkpoint paths — adding one would be
fighting the model.

## Why this shapes the math too

Because placement is offline and static, the host-side code is free of the dynamic
tiling/scheduling machinery a GPU runtime carries. The math layer is **pure,
type-generic, header-only** numerics with no awareness of the fabric at all — the
DFA compiler and the RM absorb the spatial concerns. That separation is what keeps
the math reusable in SITL (pure host) exactly as written.
