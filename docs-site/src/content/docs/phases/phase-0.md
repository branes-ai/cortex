---
title: Phase 0 — Foundation
description: Build system, CI, release flow, and dev-environment bootstrap — making the skeleton build cleanly.
---

> Status: **complete** (2026-05-22). 10 PRs landed; Phase 0 issues #4–#12 closed.

Phase 0 had one mission: **make the repository's skeleton build cleanly on green CI**,
so feature work in later phases never drags build-system fights into every PR.

## What landed

In dependency order:

1. **Pinned the Rust toolchain** (`1.83.0`) so Corrosion finds it deterministically,
   and unblocked the broken seed skeleton (empty `Cargo.toml`, phantom `.cpp` refs,
   `BUILD_TARGET_SOC` → `BUILD_TARGET_KPU` rename; Corrosion bumped to fix a
   `rustc --version` parse bug).
2. **Style enforcement** — `.clang-format` (LLVM base, 4-space, 120 columns),
   `.editorconfig`, and ADR-0001 (third-party license audit).
3. **CMake presets** — `sitl-debug`, `sitl-release`, `kpu-cross`, `wsl2-sitl` (Ninja
   single-config; per-preset binary dir).
4. **Cross-toolchain** — `cross-toolchain.cmake` for aarch64 KPU cross-compile, and
   the CI job re-enabled.
5. **FetchContent dependencies** — MTL5, Universal, yaml-cpp, Catch2, Tracy, stb (all
   pinned).
6. **sccache** — compiler caching for Rust + C++ (warm-cache: ~73% SITL, ~50%
   KPU-cross hit rates).
7. **Lint job** — clippy + `cargo fmt` + `clang-format`.
8. **Release flow** — commitlint + release-please for Conventional-Commits-driven
   SemVer.
9. **Dev bootstrap** — idempotent `bootstrap.sh` / `bootstrap.ps1` for new-dev setup.

## Conventions ratified

These are the project-wide rules still in force:

- **PR-only, even solo** — no direct commits to `main`; every change goes through a PR
  with green CI.
- **Branch = `<cc-type>/<slug>`**, **PR title = Conventional Commits message**,
  squash-merge, commitlint-enforced.
- **Claude opens a ready PR, waits for green CI + CodeRabbit, and stops; a human
  merges.** No auto-merge.

## Explicitly deferred

No algorithm code, no real cxx bridge (the stub was deleted — the real one is Phase
1), no SoC broker/IPC, no live release tag, no docs site (that's Phase 11 — this
site). Phase 0 only set conventions and stub interfaces.

Companion: `docs/arch/phase0-foundation/README.md`.
