# Session — 2026-05-22 — Phase 0 Foundation

> Session arc: from `/init` → bootstrap plan → 106 GitHub issues filed → 9 Phase 0
> issues implemented → Phase 0 retrospective.
> Total PRs landed: 11 (#107–#117). All Phase 0 issues #4–#12 closed.
> Pair: Theo Omtzigt (Ravenwater) + Claude Opus 4.7 (1M context).

---

## Arc of the conversation

This session covered a wide arc — from cold-start orientation to first-line-of-code on
the actual project. Four phases inside the session itself:

1. **Repo orientation and CLAUDE.md** (early). Read the architecture conversation
   in `docs/arch/cortex-repo.md`, generated `CLAUDE.md`.
2. **Bootstrap plan and structured triage**. Studied peer repos (MINS, isaac_ros_visual_slam)
   for implementation patterns. Built a 12-phase, ~89-issue plan, with 8 open
   judgment calls. Theo edited the plan inline — the resulting decisions are
   captured in `docs/plan/bootstrap-plan.md`.
3. **Mass issue filing**. Filed all 106 issues across 12 epics with full automation:
   labels, milestones, project board placement, GitHub Issue Types, assignee
   defaults. Memorized the automation rules so future issues inherit them.
4. **Phase 0 implementation**. Worked the dependency chain #4 → #12 → #5 → #6 → #7
   → #8 → #9 → #10 → #11, plus an initial docs PR. PR-only workflow, draft-→-CI-→
   ready-→-Theo-merges per PR. Closed with a comprehensive retrospective.

---

## Decisions ratified (with timestamps for traceability)

| Decision | Set on | Where it lives |
|---|---|---|
| Hybrid SDK (templated operators + glue library) | 2026-05-21 (plan answer #1) | `docs/plan/bootstrap-plan.md` |
| MSCKF as primary VIO backend, SW-opt skeleton as second | 2026-05-21 (#2) | bootstrap plan |
| No GPL anywhere, including tests | 2026-05-21 (#3) | `cortex-no-gpl` memory + ADR-0001 |
| SoC/KPU phases: scaffold trait, defer implementation | 2026-05-21 (#4) | bootstrap plan |
| GitHub Issues + Conventional Commits + SemVer for tracking | 2026-05-21 (#5) | `commitlint.config.cjs`, `release-please-config.json` |
| 3DSG: scaffold structure, defer math | 2026-05-21 (#6) | bootstrap plan |
| No OpenCV — in-house CV stack with OpenCV-compatible API | 2026-05-21 (#7) | bootstrap plan + E3 epic |
| Analytical Jacobians only (no AD for now) | 2026-05-21 (#8) | bootstrap plan |
| Always-PR workflow, even solo | 2026-05-22 (work day) | `cortex-workflow-defaults` memory |
| Rust pinned exactly at `1.83.0` (not floating stable) | 2026-05-22 | `rust-toolchain.toml` |
| `BUILD_TARGET_KPU` (not `BUILD_TARGET_SOC`) | 2026-05-22 | root `CMakeLists.txt` |
| Theo reviews and merges; Claude doesn't auto-merge | 2026-05-22 | `cortex-workflow-defaults` memory |
| "domain flow programs" (not "streamer programs") | 2026-05-22 (PR #107 review) | CLAUDE.md, plan, issue #23, memory |
| Per-phase milestones (one per phase) | 2026-05-22 (issue-setup) | GitHub milestones |
| Phase retrospective in `docs/arch/phaseN-<name>/` | 2026-05-22 (today) | this session's PR #117 |

---

## Mid-flight discoveries that turned into commits

Things we didn't know at the start of the day:

- **Corrosion v0.4.2 can't parse `rustc --version` output on Rust 1.80+.** Bumped to
  v0.6.1 in PR #108.
- **Seed `core/Cargo.toml` was empty and `sdk/CMakeLists.txt` referenced
  nonexistent `vio.cpp`/`scene_graph.cpp`**. The "pin Rust" PR had to grow to
  actually unblock the build.
- **stb has no GitHub releases**; pinned by commit. First attempt used a wrong SHA
  (12-char prefix instead of full 40); fixed in PR #112.
- **Universal's `CMakeLists.txt` uses `${CMAKE_SOURCE_DIR}` which under FetchContent
  resolves to *cortex* root, not Universal's**. Worked around with manual
  `FetchContent_Populate` + a manual INTERFACE target, suppressing the CMP0169
  deprecation warning. Track upstream fix.
- **Debian/Ubuntu's multiarch `libm.so` is a GNU ld linker script with absolute
  paths**. With `--sysroot` set, the linker double-prefixes and fails. Fix: sysroot
  is opt-in only (PR #111).
- **Workflow-level `RUSTC_WRAPPER=sccache` breaks any job without sccache installed**.
  Discovered when adding the lint job (PR #114); fix is to add the sccache-action
  step to every job that touches cargo.
- **GHA cache backend silently drops 25–50% of writes** on cold runs (no error
  message, just sccache stat counters). Capped first-warm-rerun hit rate at ~50–73%.
  Watching as the cache fills in over future PRs.
- **release-please needs a repo-level "Allow Actions to create PRs" setting**
  toggled on. Action created the branch + commit successfully but the final
  PR-create API call returned an explicit permission denial. Action item filed for
  Theo to toggle.

---

## What landed (chronological)

```
Day-1 (2026-05-21) — orientation + planning + issue filing
  - CLAUDE.md generated from docs/arch/cortex-repo.md
  - bootstrap-plan.md drafted, judgment calls answered, scope locked
  - 12 epics + 94 sub-issues filed (#1-#106) with full default automation
  - All 32 first-batch issues retroactively assigned + milestoned + estimated

Day-2 (2026-05-22) — Phase 0 implementation
  morning:
   #107 docs: add CLAUDE.md + bootstrap-plan + .gitignore .claude/
       └─ review cycle: rename "streamer programs" → "domain flow programs"
   #108 chore(toolchain): pin Rust 1.83.0 + unblock skeleton build  (closes #4)
   #109 chore(format): .clang-format + .editorconfig + ADR-0001     (closes #12)
   #110 build(cmake): CMakePresets.json with 4 presets              (closes #5)
   #111 build(cmake): cross-toolchain.cmake (aarch64)               (closes #6)
   #112 build(cmake): FetchContent for 6 foundational deps          (closes #7)

  afternoon/evening:
   #113 ci: sccache integration                                     (closes #8)
   #114 ci: lint job (clippy + cargo fmt + clang-format)            (closes #9)
   #115 chore(release): commitlint + release-please                 (closes #10)
   #116 chore(bootstrap): bootstrap.sh + bootstrap.ps1              (closes #11)
   #117 docs: Phase 0 Foundation retrospective                      (closes nothing)
```

---

## Velocity observations

- **11 PRs in ~26 hours of wall time.** Each PR was small and focused;
  draft-→-CI-→-ready took ~2-5 minutes on average; Theo merged each one quickly.
- **CI times dropped meaningfully across the day** as sccache filled the cache:
  SITL went from 1m48s (first FetchContent run) → 55s (warm cache by end of day).
- **One process refinement mid-day**: after PR #113 (sccache), validated the
  acceptance criterion ">50% hit rate on rerun" by triggering a manual `gh run rerun`
  — a pattern worth keeping for acceptance criteria that depend on second-run state.

---

## Known follow-ups (deferred, with owners)

| Item | Where it's tracked | Action |
|---|---|---|
| Toggle "Allow GitHub Actions to create and approve pull requests" | repo Settings → Actions | Theo |
| Upstream Universal fix (`CMAKE_SOURCE_DIR` → `PROJECT_SOURCE_DIR`) | bootstrap plan §6.1 | file upstream PR |
| Investigate GHA cache write errors if hit rate plateaus < 80% | bootstrap plan §6.2 | observe over Phase 1/2 PRs |
| Add QEMU step so KPU cross tests actually run | bootstrap plan §6.4 | file as follow-up when SDK has runnable tests |
| Switch release-please to multi-package config when `cortex-core` ships to crates.io | bootstrap plan §6.6 | re-visit when relevant |

---

## What's next

Phase 1 (Rust core, #13–#20 MVP) and Phase 2 (math, #24–#32) are parallelizable. Both
feed into Phase 3 (VIO MVP). The plan picks Phase 1 first by convention; #13
(MemoryProvider trait) is the natural starting issue.

Phase 1 will introduce the first cross-language surface (`cxx::bridge`), and the
first real code that exercises the new dev-environment scaffolding end-to-end. Good
opportunity to validate the workflow with actual implementation work, not just
infrastructure.

---

## Memory state at session end

Five project-scoped memory entries in force:

- `cortex-rm-vs-dfa-compiler` — RM allocates; DFA compiler schedules.
- `cortex-no-gpl` — strict no-GPL rule, including tests; clean-room from papers.
- `cortex-issue-creation-defaults` — assignee=Ravenwater, project=Branes CORTEX,
  Estimate=5, milestone-by-phase-label, GitHub Issue Type by CC prefix.
- `cortex-workflow-defaults` — always PR (even solo); Rust pinned exactly;
  `BUILD_TARGET_KPU` naming; Theo merges, Claude doesn't.

Together these encode the rules ratified during Phases 0-prep + Phase 0 itself.
Future Claude sessions inherit them automatically.
