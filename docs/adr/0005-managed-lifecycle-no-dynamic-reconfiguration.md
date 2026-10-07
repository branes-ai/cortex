# ADR 0005 — Managed lifecycle; no dynamic reconfiguration

**Status:** Accepted
**Date:** 2026-05-20
**Source:** `docs/arch/cortex-repo.md`, *Why Dynamic Reconfiguration Kills Real-Time Systems* / *The Solution: Managed Lifecycles*

## Context

Robotics stacks often allow parameters (exposure, solver tolerances,
feature counts) to change while an operator runs. Supporting that safely
requires:

- mutex-guarded parameters on the hot path, which blow real-time deadlines;
- re-allocating DMA memory pools on the fly, which fragments memory and
  causes latency spikes;
- mixing parameters mid-history in a filter or factor graph, which corrupts
  its mathematical continuity.

## Decision

**No dynamic reconfiguration.** A configuration change is a
**teardown/restart** through a managed lifecycle:

| State | Meaning |
|---|---|
| `Unconfigured` | Alive, no memory allocated |
| `Inactive` | Configuration applied, memory pools pre-allocated, operators constructed but idle |
| `Active` | Processing: lock-free and deterministic |
| `Teardown` | State flushed, operators destroyed, pools released |

A new configuration takes the component out of `Active`, through teardown,
and back to `Inactive` with the new parameters.

## Consequences

- Parameters are immutable while `Active`. Do not add setters or
  mutex-guarded mutable parameters to the hot path.
- Configuration crosses into the SDK as typed POD structs
  ([ADR-0014](0014-middleware-agnostic-core-zenoh-daemons.md)), passed at
  `configure` time.
- **Current state:** implemented by the Rust RM (`core/src/lifecycle.rs`,
  typed errors on invalid transitions; `Teardown` returns to `Unconfigured`
  via `reset`) and mirrored by `branes::sdk::VioEstimator`. In the
  estimator, a new configuration is applied by `deactivate()` → `configure()`
  (back in `Inactive`, runtime state cleared, never while `Active`);
  `teardown()` is terminal, so a restart after it is a new estimator
  instance. The daemons adopt it when they land (epic #75). Sister repos
  follow the same rule (e.g. `branes::reflex::Pid` fixes its gains at
  construction).
