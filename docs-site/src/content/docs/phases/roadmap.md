---
title: Roadmap
description: The 12-phase plan for Cortex and where the project stands today.
---

Cortex is built phase by phase against a 12-phase roadmap (`docs/plan/bootstrap-plan.md`).
Each phase is an epic of single-issue PRs, landed on green CI.

## Phases

| Phase | Epic | Status |
|---|---|---|
| 0 | Foundation — build, CI, release flow | ✅ complete ([details](/phases/phase-0/)) |
| 1 | Rust Resource Manager + cxx bridge | ✅ complete ([details](/phases/phase-1/)) |
| 2 | Math layer (Lie, NLS, cameras) | ✅ complete ([details](/phases/phase-2/)) |
| 3 | Visual-Inertial Odometry (MSCKF) | ✅ complete ([details](/phases/phase-3/)) |
| 4 | SLAM (loop closure, mapping) | ⏳ next |
| 5 | 3D Scene Graph | planned |
| 6 | Unified configuration | planned |
| 7 | Daemons (Zenoh integration) | planned |
| 8 | SITL system integration | planned |
| 9 | Profiling (Tracy + Valgrind) | planned |
| 10 | Deployment (OCI + Yocto) | planned |
| 11 | Documentation | in progress (this site) |

Plus a parked hardware track (the `phase-soc-deferred` label, issues #21–#23): the IPC
broker, crash-only recovery, and the KPU allocation arbiter — deferred until the
silicon path is active.

## How a phase runs

The workflow is consistent across phases and recorded in `CLAUDE.md` + the project
memory:

1. **One PR per issue.** Branch `<cc-type>/<slug>`, PR title is the Conventional
   Commits message.
2. **Ready PR → CodeRabbit review → address findings → CI green → merge.** No
   auto-merge; no direct commits to `main`.
3. **Both compilers.** Every C++ change builds under gcc and clang locally before push.
4. **Clean-room.** Implement from papers, cite the source, write independent test
   oracles ([provenance](/algorithms/citations/)).
5. **Phase close.** A retrospective page (these pages) + a session log + a memory entry.

## Standing decisions

- **No GPL**, including test oracles.
- **Type-generic** math — never hardcode `double`.
- **`BUILD_TARGET_KPU`** is the single FFI-spanning build flag.
- **Rust pinned exactly** in `rust-toolchain.toml`; bumps are their own PRs.
