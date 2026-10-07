# ADR 0009 — Crash-only Resource Manager recovery by client replay

**Status:** Accepted
**Date:** 2026-05-20
**Source:** `docs/arch/cortex-repo.md`, *The "Crash-Only" Recovery Architecture*
**Related:** [ADR-0008](0008-host-broker-thin-client-topology.md); issue #22

## Context

A single privileged Resource Manager (ADR-0008) is a single point of
failure. The requirement is to recover in about a millisecond, without
restarting the dependent operator processes. The KPU fabric can be treated
as **stateless**: reprogramming it is cheap, and its on-chip memories (the L3s)
are a transient near-memory store for data flowing through. The real inputs
stay in host-RAM DMA buffers.

## Decision

Adopt **crash-only software** for the RM. Recovery is driven by the
surviving clients, not by checkpoints.

1. **Holding pattern.** When the RM's socket closes, the SDK's IPC layer
   does not fail. It keeps its pointers to the input buffers in host RAM and
   retries the connection for a short window (hundreds of microseconds).
2. **Micro-reboot.** The OS respawns the RM immediately (`systemd`
   `Restart=always`, `RestartSec=0`); it starts with an empty allocation map.
3. **Idempotent replay.** Reconnecting clients replay their pending
   execution requests. Since every domain flow program's footprint is known
   offline, the RM rebuilds its allocation map from the replayed requests.
4. **Fabric reinitialization.** The RM reloads the domain flow programs; the
   KPU pulls inputs back from the DMA buffers and regenerates its transient
   state as data flows.

**No disk- or NVRAM-based checkpointing.**

## Consequences

- Client requests must be idempotent and replayable; the SDK IPC layer owns
  the holding-pattern logic.
- An RM crash costs roughly one dropped or delayed frame, which a VIO/SLAM
  estimator absorbs. How downstream control reacts (IMU dead-reckoning for
  a UAV, a brief slowdown for a passenger AV) is an application decision.
- Do not add checkpoint paths to `core/`.
- **Current state:** design only. The protocol is issue #22.
