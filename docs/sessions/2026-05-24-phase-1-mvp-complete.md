# Session — 2026-05-24 — Phase 1 MVP Complete

> Session arc: from `/init` of the autonomous-loop infrastructure (PR #119
> CodeRabbit config) through the closure of all eight Phase 1 MVP issues
> (#13–#20). Total PRs landed: 11 (#119–#137). All Phase 1 MVP issues
> closed; the parent Epic #2 remains open behind the deferred sub-issues
> (#21–#23 — `phase-soc-deferred`).
> Cortex released **v0.1.0 → v0.6.0** through release-please.
> Pair: Theo Omtzigt (Ravenwater) + Claude Opus 4.7 (1M context).

---

## What was accomplished

### Autonomous-loop enablement (PRs #119 → #125)

- **`.coderabbit.yaml`** (#119) — assertive profile, request_changes_workflow, path_instructions for every layer, tools (clippy / rustfmt / cppcheck / actionlint / shellcheck / yamllint / markdownlint), no-GPL / KPU terminology / hybrid-SDK rules encoded.
- **Regression-test infrastructure** (#123) — `tests/include/branes/test/{timing,budgets}.hpp` for latency-budget assertions, `cargo-llvm-cov` coverage job in CI (measure-only at first).
- **release-please semantics fix** (#121) — `initial-version: "0.1.0"` and dropped the over-conservative `bump-patch-for-minor-pre-major` flag. Cut v0.1.0 the same day.
- **Phase 1 design doc** (#125) — 567-line spec at `docs/arch/phase1-design/README.md` pre-deciding every judgment call for #13–#20. Drove the hands-free flow.

### Phase 1 implementation (PRs #127 → #137)

Implemented in strict dependency order out of the design doc:

| Issue | PR | Release | Tests |
|---|---|---|---|
| #13 — `MemoryProvider` trait + `BufferHandle` + `MemErr` | #127 | v0.2.0 | 12 |
| #14 — SITL provider: Linux shm + heap backends | #129 | v0.3.0 | 13 |
| #15 — KPU stub (`Err(NotYetImplemented { target: "kpu" })`) | #131 | v0.4.0 | 5 |
| #16 + #17 + #18 — cxx bridge + build.rs + lifecycle state machine | #133 | v0.5.0 | 14 (lifecycle) + 6 (bridge) |
| #19 — coverage gate flipped to `--fail-under-lines 85` | #135 | v0.5.1 | (gate only) |
| #20 — C++/Rust integration test through cxx bridge | #137 | v0.6.0 | 4 (C++) |

End-of-day state:
- **46 Rust unit tests + 5 C++ Catch2 tests = 51 tests passing.**
- Coverage gated at ≥ 85% line coverage on `core/`.
- `cxx::bridge` generates `<cortex_core_cxx/bridge.h>` consumed by C++ tests.
- CMake `kpu-cross` preset cross-compiles `libcortex_core.a` to aarch64.
- All three workflows (CI, commitlint, release-please) green on main.

### Architectural decisions ratified along the way

1. **Pinned `cxx = "=1.0.130"`** — latest cxx pulls hashbrown 0.17 which requires `edition2024`, not stable in Rust 1.83.0. Pinning keeps the toolchain bump as a separate, deliberate decision.
2. **Switched cxx codegen from cargo's `build.rs` to Corrosion's `corrosion_add_cxxbridge`** — gave CMake clean access to the generated header without depending on Corrosion's per-package hash-suffixed cache dir.
3. **`BufferHandle` is opaque** — `pub struct BufferHandle(u64)` (private field) with `pub(crate) const fn from_raw / as_raw`. External callers can't forge or pattern-match the inner integer.
4. **Lifecycle is enum-backed, not type-state** — the cxx bridge surfaces a single C++-visible type; type-state parameters can't cross the FFI. Enum + verified transitions is the right fit.
5. **`TensorMetadata` is a DLPack-style descriptor, not a raw memory view.** CR initially asked to strip the `handle`/`byte_size`/`stride`/`dtype` fields; I pushed back with the design-doc rationale, CR agreed and added a "Learning" entry so the question won't be re-asked.

---

## Mid-flight discoveries that became commits or memory entries

Things we didn't know at the start of the day:

- **Conventional Commits `subject-case` rule** rejects first-word upper-case/start-case/pascal-case. `feat(core): SITL ...` was rejected on PR #129; amended to `feat(core): add SITL ...`. Memory entry under `cortex-workflow-defaults`.
- **`cargo fmt --check` exit was masked by `tail -3`** in my pre-push verification (PR #127). Fix: `set -o pipefail` or `${PIPESTATUS[0]}`. Same memory entry.
- **CodeRabbit's GitHub-App slug is `coderabbitai` (no hyphen).** My monitor's check-suite filter excluded `coderabbit-ai` (with hyphen) and spun forever waiting for CR's queued check to "complete" on draft PRs. Fixed and saved as memory.
- **Corrosion's `corrosion_add_cxxbridge`:**
  - `CRATE` wants the cmake target name (underscore `cortex_core`), not the cargo package name (hyphen `cortex-core`).
  - `FILES` is auto-prefixed with `src/` — so the value is `bridge.rs`, not `src/bridge.rs`.
  Both gotchas documented inline in `CMakeLists.txt`.
- **release-please's default first-release behavior** is to propose `1.0.0`. To start at `0.1.0` for alpha software, set `initial-version: "0.1.0"` per-package. Captured in [`docs/arch/phase0-foundation/README.md` §6.4](../arch/phase0-foundation/README.md).
- **`cargo-llvm-cov` post-1.0.7 needs `edition2024`** — pinned to v0.6.21 indirectly via `taiki-e/install-action`, which still works fine on Rust 1.83.0.

---

## What landed (chronological)

```text
PR #119 chore(coderabbit): .coderabbit.yaml with layering + license + terminology
PR #121 chore(release): pin initial-version to 0.1.0
PR #123 chore(testing): regression infra — coverage + latency budget framework
PR #125 docs:          Phase 1 design — Rust RM + cxx bridge

  ── release-please cut v0.1.0, v0.1.1, v0.1.2, v0.2.0 ──

PR #127 feat(core):    define MemoryProvider trait + BufferHandle + MemErr   (closes #13)
PR #129 feat(core):    add SITL MemoryProvider with shm + heap backends      (closes #14)
PR #131 feat(core):    add KPU MemoryProvider stub                           (closes #15)
PR #133 feat(core):    add cxx bridge, build.rs, and lifecycle state machine (closes #16, #17, #18)
PR #135 ci:            gate Rust coverage at >= 85% line coverage            (closes #19)
PR #137 test(bridge):  add C++/Rust cxx::bridge integration test             (closes #20)
```

All MVP issues closed in dependency order. No PRs reverted; one (#137) had its build-system restructured mid-flight (cxx codegen moved from cargo `build.rs` to Corrosion's `corrosion_add_cxxbridge`).

---

## CodeRabbit interactions

CR's assertive profile picked up consistently after #119's parsing-error fix landed. Notable interactions:

- **#127 (MemoryProvider trait):** CR flagged `pub struct BufferHandle(pub u64)` as violating the documented opacity invariant. Right call — fixed by making the field private and adding `pub(crate) const fn from_raw / as_raw`. Also flagged duplicate public paths (`pub mod` + `pub use`) — fixed.
- **#129 (SITL provider):** CR flagged the SHA-pinning of GitHub Actions (`@v4` style is mutable). Did a comprehensive workflow-wide SHA pin in the same PR.
- **#133 (cxx bridge):** CR flagged TensorMetadata's "extra" fields as an FFI-contract violation. **Pushed back** with the DLPack-descriptor rationale — CR agreed, withdrew the concern, and added a "Learning" entry so the question won't be re-asked. Also flagged a legitimate leak (lock-failure path didn't release the handle); fixed in the same iteration.
- **#137 (integration test):** CR approved on first review. No actionable comments.

Workflow shape: every PR went through `draft → CI green → mark ready → CR review → address → merge`. CR rate-limited twice (Pro plan has 1 review/hour); upgrade kicked in mid-day and removed the constraint.

---

## Process drift caught

- "Standing by" responses to monitor phase events read as me waiting for the user to prod. Memory rule updated: react to actionable events, stay silent on informational ones.
- Monitor exited on CR's "review in progress" interim ping. Refined the script to skip those and only emit on terminal review outcomes.
- One monitor was keyed to a pre-amend SHA after a `--force-with-lease` push; re-armed against the new SHA. Patterns documented inline.

---

## Known follow-ups (deferred)

| Item | Where | Action |
|---|---|---|
| Bump Rust toolchain past 1.83.0 to unblock newer crate deps that require `edition2024` | `rust-toolchain.toml` | Deliberate workflow decision; do when newer deps actually bite. |
| Universal upstream `${CMAKE_SOURCE_DIR}` fix | bootstrap-plan §6.1 | Outstanding from Phase 0. |
| Windows backend for `SitlMemoryProvider` (`CreateFileMappingA` + `MapViewOfFile`) | `core/src/sitl_provider.rs` | Currently TODO comment; no Windows CI runner yet. |
| QEMU step for KPU cross-compile tests | bootstrap-plan §6.4 | Outstanding from Phase 0. |
| Phase 1.5 polish: `configure(config: YAML)` accepting a typed struct + auto-teardown on new-config arrival | `core/src/lifecycle.rs` | Deferred to Phase 6 (#62, config schema). |
| Deferred SoC sub-epic | #21, #22, #23 | `phase-soc-deferred` label; ADR-only until silicon ships. |

---

## What's next

**Phase 2 (Math, #24–#32)** and **Phase 3 (VIO, #33–#49)** are now unblocked:

- Phase 2 is parallelizable from here — pure header-only math layer (tensor views over `std::span`, sparse storage, Krylov + direct linear solvers via MTL5, Lie groups, NLS solvers). The trait surface from Phase 1 isn't a dependency.
- Phase 3 begins the actual VIO work and uses `TensorMetadata` directly to wrap zero-copy camera frames. Depends on Phase 2's math layer and #14's SITL `MemoryProvider`.

Critical path: Phase 2 → Phase 3 → Phase 7 daemons (Zenoh middleware adapters consuming `cortex::VioEstimator`).

Phase retrospective for Phase 1 will land at `docs/arch/phase1-foundation/README.md` (or similar) when the parent Epic #2 closes — i.e. when the SoC-deferred sub-epic also lands. That's likely a 2026-Q3-or-later milestone tied to silicon availability.

---

## Memory state at session end

Five project-scoped memory entries in force:

- `cortex-rm-vs-dfa-compiler` — RM does allocations only; scheduling is the DFA graph compiler's job.
- `cortex-no-gpl` — strict no-GPL rule, including test oracles; clean-room from papers.
- `cortex-issue-creation-defaults` — every new issue gets assignee=Ravenwater, project=Branes CORTEX, Estimate=5, milestone-by-phase, type-by-CC-prefix.
- `cortex-workflow-defaults` — always PR (even solo); Rust pinned exactly; `BUILD_TARGET_KPU` naming; auto-merge after CI + CR clean (subject to branch protection); CR-slug filter (`coderabbitai`); CC subject-case rule; pipefail-safe verification.
- `cortex-workflow-defaults` (continued) — auto-merge handoff: react to monitor events autonomously, stay silent on informational ones.

These rules are what made today's hands-free Phase 1 implementation possible. Future Claude sessions inherit them automatically.
