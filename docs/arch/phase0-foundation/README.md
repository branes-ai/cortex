# Phase 0 — Foundation

> Status: **closed** (2026-05-22). 10 PRs landed; all Phase 0 issues #4–#12 closed.
> Companion documents: [`docs/arch/cortex-repo.md`](../cortex-repo.md) (overall stack design),
> [`docs/plan/bootstrap-plan.md`](../../plan/bootstrap-plan.md) (full 12-phase roadmap),
> [`docs/adr/0001-third-party-license-compatibility.md`](../../adr/0001-third-party-license-compatibility.md) (license audit).

---

## 1. Goals and non-goals

### Goals

Phase 0 had a single high-level mission: **make the repository's skeleton actually build cleanly so Phase 1+ work can land on a green CI without dragging build-system fights into every feature PR.**

Concrete deliverables, in dependency order:

1. Pin the Rust toolchain so Corrosion can find it deterministically.
2. Unblock the C++ build (the seed skeleton had a `cortex_sdk SHARED` rule referencing nonexistent `.cpp` files and an empty `core/Cargo.toml`).
3. Establish style enforcement (`.clang-format`, `.editorconfig`) so subsequent PRs aren't bikeshedding format on review.
4. Define the four build presets (SITL debug/release, KPU cross, WSL2 SITL) so developers don't memorize flag chains.
5. Wire a cross-compile toolchain for the KPU silicon target.
6. Pull in the foundational third-party dependencies via `FetchContent` (no system packages, no vcpkg).
7. Cut CI build time with a compiler cache (sccache).
8. Add lint gates (clippy, cargo fmt, clang-format) to CI.
9. Wire Conventional Commits + dynamic-SemVer release flow.
10. Provide a one-shot dev-environment bootstrap script for Linux and Windows.

### Non-goals

These were *explicitly* deferred to later phases. Phase 0 only set conventions and stub interfaces for them:

- **No algorithm code.** No VIO, no SLAM, no math kernels — those live in Phases 2–5.
- **No real `cxx::bridge`.** The seed skeleton had a stub bridge; Phase 0 deleted it. The real bridge lands in #16 (Phase 1).
- **No SoC broker / IPC.** Issues #21–#23 are parked behind the `phase-soc-deferred` label.
- **No live release.** release-please was wired up but no `v0.1.0` tag has been cut yet.
- **No examples, no docs site, no ADR mass-population.** Those land in Phase 11.

---

## 2. What landed

The bootstrap pre-PR (`#107`) didn't close an issue — it just landed `CLAUDE.md` + this `docs/plan/bootstrap-plan.md` + a `.gitignore` for `.claude/`. The other nine PRs each closed exactly one Phase 0 sub-issue.

| PR | Issue closed | Title | Notes |
|---|---|---|---|
| [#107](https://github.com/branes-ai/cortex/pull/107) | (none) | `docs: add CLAUDE.md and bootstrap plan; gitignore .claude/` | Pre-PR; landed CLAUDE.md and the 106-issue roadmap. Rebased after #108 unblocked CI. |
| [#108](https://github.com/branes-ai/cortex/pull/108) | [#4](https://github.com/branes-ai/cortex/issues/4) | `chore(toolchain): pin Rust 1.83.0 and unblock skeleton build` | Bundled with fixes for the broken seed skeleton (empty `Cargo.toml`, phantom `vio.cpp` refs, `BUILD_TARGET_SOC` → `BUILD_TARGET_KPU` rename). A follow-up commit bumped Corrosion v0.4.2 → v0.6.1 to fix a `rustc --version` parse bug. |
| [#109](https://github.com/branes-ai/cortex/pull/109) | [#12](https://github.com/branes-ai/cortex/issues/12) | `chore(format): add .clang-format, .editorconfig, and license-compatibility ADR` | LLVM-base + 4-space + ColumnLimit 120; markdown opts out of whitespace trimming. ADR-0001 audited every planned dep. |
| [#110](https://github.com/branes-ai/cortex/pull/110) | [#5](https://github.com/branes-ai/cortex/issues/5) | `build(cmake): add CMakePresets.json with sitl/kpu-cross/wsl2 presets` | Ninja single-config; per-preset binary dir under `build/<preset>/`. CI switched to `cmake --preset sitl-debug`. |
| [#111](https://github.com/branes-ai/cortex/pull/111) | [#6](https://github.com/branes-ai/cortex/issues/6) | `build(cmake): add cross-toolchain.cmake for KPU aarch64 cross-compile` | Re-enabled the previously commented-out CI job. Sysroot is opt-in — Debian's multiarch libm conflicts with `--sysroot`. |
| [#112](https://github.com/branes-ai/cortex/pull/112) | [#7](https://github.com/branes-ai/cortex/issues/7) | `build(cmake): wire FetchContent for foundational dependencies` | MTL5 v5.2.1, Universal v4.7.0, yaml-cpp 0.9.0, Catch2 v3.15.0, Tracy v0.13.1, stb (pinned commit). Plus a working Universal workaround. |
| [#113](https://github.com/branes-ai/cortex/pull/113) | [#8](https://github.com/branes-ai/cortex/issues/8) | `ci: integrate sccache for Rust + C++ object caching` | Warm-cache rerun: SITL **73.4%** hit, KPU cross **50.0%** hit. Above the >50% bar. |
| [#114](https://github.com/branes-ai/cortex/pull/114) | [#9](https://github.com/branes-ai/cortex/issues/9) | `ci: add lint job (clippy + cargo fmt + clang-format)` | Required a follow-up commit to install sccache on the lint job (workflow-level `RUSTC_WRAPPER=sccache` failed without it). |
| [#115](https://github.com/branes-ai/cortex/pull/115) | [#10](https://github.com/branes-ai/cortex/issues/10) | `chore(release): add commitlint + release-please for SemVer` | commitlint via `wagoid/commitlint-github-action@v6.2.1`; release-please via `googleapis/release-please-action@v5.0.0`. **First release-please run blocked** — see Known Gaps. |
| [#116](https://github.com/branes-ai/cortex/pull/116) | [#11](https://github.com/branes-ai/cortex/issues/11) | `chore(bootstrap): add bootstrap.sh + bootstrap.ps1 for new-dev setup` | Idempotent dev-env setup. The Bash variant also offers `--install-deps` to apt-install the missing system packages. |

**Aggregate stats:**
- 9 active PRs + 1 pre-PR (=10) merged in ~26 hours of wall time.
- Mid-flight discoveries that became their own commits/PRs (and worth knowing): Corrosion v0.4.2 parse bug, empty `core/Cargo.toml`, broken `sdk/CMakeLists.txt`, Universal's `${CMAKE_SOURCE_DIR}` issue, Debian multiarch sysroot conflict, lint job needing sccache, release-please needing the repo-level "Allow Actions to create PRs" toggle.

---

## 3. Conventions ratified

These are the project-wide rules in force at the end of Phase 0. They live across `CLAUDE.md`, the Claude memory files, this doc, and the PR templates.

### 3.1. Source-control workflow

- **PR-only, even solo.** No direct commits to `main`. Even when only Theo is committing, every change goes through a PR with green CI before merge.
- **Branch names** follow `<cc-type>/<slug>` — `chore/pin-rust-toolchain`, `build/cmake-presets`, `ci/lint-checks`. The branch type matches the Conventional Commits prefix.
- **PR title = Conventional Commits message.** Squash-merge (or merge commit) consumes that title, which release-please then parses on `main`. commitlint enforces this on every PR.
- **Claude opens draft PRs, waits for CI to be green, marks the PR ready, and stops.** Theo reviews and merges. Auto-merging by Claude is not enabled.

### 3.2. Conventional Commits

Allowed types: `feat`, `fix`, `build`, `ci`, `chore`, `docs`, `test`, `refactor`, `perf`, `style`, `revert`. Scopes are lowercase only. Headers max 100 chars.

Each CC type maps to a GitHub Issue Type:
- `feat(...)` → **Feature** (org type id `IT_kwDOC7WhuM4BgJ2B`)
- `fix(...)` → **Bug** (id `IT_kwDOC7WhuM4BgJ2A`)
- `chore/build/ci/test/docs` → **Task** (id `IT_kwDOC7WhuM4BgJ1_`)
- Epic issues (titled `Epic: E... · ...`) have no CC prefix and are left untyped.

### 3.3. Versioning and release

- **release-please** generates the version bumps and `CHANGELOG.md` from CC commit messages on `main`.
- `release-type: simple` — a single SemVer tag for the whole repo. We can split into per-package later when subsystems start releasing independently.
- **Initial version `0.0.0`** in `.release-please-manifest.json`.
- `bump-minor-pre-major: true` — `feat:` commits bump the *minor* version until a `1.0.0` is cut. Breaking changes stay in `0.x` minor bumps for now.

### 3.4. Toolchain pins

| Tool | Pin | Set in |
|---|---|---|
| Rust | `1.83.0` | `rust-toolchain.toml` |
| Rust targets | `x86_64-pc-windows-msvc`, `aarch64-unknown-linux-gnu`, `riscv64gc-unknown-linux-gnu` | `rust-toolchain.toml` |
| CMake | ≥ 3.25 (for CMakePresets v6) | enforced by `CMakePresets.json` + `bootstrap.{sh,ps1}` |
| C++ standard | C++20 | root `CMakeLists.txt` |
| Corrosion | v0.6.1 | root `CMakeLists.txt` |
| MTL5 | v5.2.1 | `cmake/deps.cmake` |
| Universal | v4.7.0 | `cmake/deps.cmake` |
| yaml-cpp | 0.9.0 | `cmake/deps.cmake` |
| Catch2 | v3.15.0 | `cmake/deps.cmake` |
| Tracy | v0.13.1 | `cmake/deps.cmake` |
| stb | commit `31c1ad3` | `cmake/deps.cmake` |

Bumps are deliberate PRs of their own — they show up in the release-please changelog separately from feature work.

### 3.5. CMake conventions

- **Build flag:** `BUILD_TARGET_KPU` (replaces the seed skeleton's `BUILD_TARGET_SOC`). When `ON`, sets `TARGET_KPU` for C++ and `RUST_HAL_TARGET=KPU` for the Rust core; when `OFF`, sets `TARGET_SITL` and `RUST_HAL_TARGET=SITL`. Operators (`sdk/`, `math/`, `cv/` planned) must *never* branch on this — that distinction stays inside the Rust `core/` HAL.
- **Generator:** Ninja single-config.
- **Per-preset binary dirs:** `build/<preset>/` so `sitl-debug` and `sitl-release` coexist.
- **`CMAKE_EXPORT_COMPILE_COMMANDS=ON`** in every preset so clangd / VS IntelliSense work.
- **Presets** (from [`CMakePresets.json`](../../../CMakePresets.json)):
  - `sitl-debug` — host x86 Debug. Day-to-day default.
  - `sitl-release` — host x86 Release.
  - `kpu-cross` — aarch64 cross via [`cross-toolchain.cmake`](../../../cross-toolchain.cmake). Tests off (no aarch64 emulator).
  - `wsl2-sitl` — same as `sitl-debug` but with Visual Studio vendor metadata so VS 2022 on Windows routes through the WSL2 toolchain.

### 3.6. Dependency policy

- **All deps via CMake `FetchContent`.** No vcpkg, no Conan, no system packages baked into the build.
- **No GPL/LGPL/MPL/SSPL/AGPL anywhere** — not in distribution, not in tests, not as ground-truth oracles. See [ADR-0001](../../adr/0001-third-party-license-compatibility.md) and the `cortex-no-gpl` memory entry. This rejects OpenVINS, MINS, ORB-SLAM3, DBoW3 as code references for later phases.
- **Compiler cache via sccache** rather than a package manager, when build times start to bite.

### 3.7. CI gates

Four jobs run on every push and PR:
1. **Conventional Commits lint** (14s) — `wagoid/commitlint-github-action`.
2. **Lint** (~33s) — `cargo fmt --check`, `cargo clippy --all-targets --all-features -- -D warnings`, `clang-format --dry-run --Werror` on every tracked C/C++ file.
3. **Build and Test (SITL — x86)** (~1m on a warm cache) — `cmake --preset sitl-debug && cmake --build --preset sitl-debug && ctest --preset sitl-debug`.
4. **Cross-Compile (KPU Target — aarch64)** (~1m on a warm cache) — `cmake --preset kpu-cross && cmake --build --preset kpu-cross`, with an `objdump -f` step to confirm the output is aarch64.

The `lint` job runs on `ubuntu-24.04` (for clang-format 18); the build jobs stay on `ubuntu-22.04`.

---

## 4. Architectural decisions ratified

These weren't *new* in Phase 0 — most were settled in the bootstrap plan — but Phase 0 is when they crystallized into code. Listed here in load-bearing order.

### 4.1. RM allocates; DFA compiler schedules

The Rust Resource Manager's job is **allocation arbitration**, not scheduling. Computational-spacetime scheduling lives in the DFA graph compiler (separate repo, likely `branes-ai/domain_flow`), which emits **domain flow programs** — pre-scheduled subgraphs that the RM places onto allocated KPU resources. Once a domain flow program is loaded, the KPU runs its own dataflow internally; no runtime scheduling on the host.

Why this matters now: it shapes the Phase 1 trait surface (`MemoryProvider`, lifecycle, IPC broker design) and rules out scheduler code in `core/`. See `CLAUDE.md § "The KPU spacetime constraint"`.

Terminology note: we standardized on **"domain flow programs"** (not "streamer programs", which was the wording in the bootstrap-plan drafts and one earlier issue body). Renamed across CLAUDE.md, bootstrap-plan.md, issue #23, and the project memory in PR #107's review cycle.

### 4.2. Hybrid SDK: templated operators + library glue

Originally the conversation in `docs/arch/cortex-repo.md` framed cortex as "header-only". Phase 0 ratified a **hybrid** model:

- Core operators (the math, the kernels) are **header-only templates** parameterized on arithmetic type, so mixed-precision (Universal posits, custom floats, fixed-point) is a compile-time choice with no runtime overhead.
- Glue logic (I/O, calibration loaders, lifecycle management, daemon entry points) is compiled into a library — there's no point templating those on arithmetic type, and ABI stability there matters for shipping daemons.

This is why the original `cortex_sdk SHARED` placeholder was deleted (PR #108) — `sdk/CMakeLists.txt` will land real targets only when actual code arrives in Phase 3 (#34, #45).

### 4.3. Clean-room policy

Every Phase 2+ implementation is derived **from papers and permissively-licensed references only**. No source-code contact with OpenVINS, MINS, Sophus, g2o, ORB-SLAM3, DBoW3, or any other GPL/LGPL reference. Even for testing — no GPL ground-truth oracle. The clean-room VIO simulator (issue #86) replaces that.

### 4.4. Universal manual-populate hack

Upstream `stillwater-sc/universal` v4.7.0's `CMakeLists.txt` uses `${CMAKE_SOURCE_DIR}` (which under FetchContent resolves to the *parent project's* root — i.e. cortex) for its `configure_file` call, breaking the build. Workaround: skip `add_subdirectory` and expose Universal as a manual INTERFACE target rooted at `<src>/include/sw`, behind `CMP0169 OLD` to silence the `FetchContent_Populate` deprecation warning.

This is a deliberate *temporary* hack — track upstream Universal fix (`${CMAKE_SOURCE_DIR}` → `${PROJECT_SOURCE_DIR}`) to retire it. Code comment in `cmake/deps.cmake` documents the rationale.

### 4.5. Sysroot opt-in by default

The KPU cross-compile (`cross-toolchain.cmake`) does **not** default `CMAKE_SYSROOT`. Debian/Ubuntu's multiarch cross-libraries (`gcc-aarch64-linux-gnu`) ship `libm.so` as a GNU ld linker script with **absolute** paths; with `--sysroot` set, those paths get double-prefixed and the linker fails. Letting the cross-compiler use its built-in lib paths is the right answer for the standard Ubuntu cross-toolchain. A real KPU board rootfs will set `CROSS_SYSROOT` explicitly.

### 4.6. KPU naming over SoC

The seed skeleton used `BUILD_TARGET_SOC` and `TARGET_SOC`. Renamed to `BUILD_TARGET_KPU` / `TARGET_KPU` in PR #108. The silicon's actual name is the **Knowledge Processing Unit (KPU)**, and a generic "SoC" naming is ambiguous on a host that might have multiple accelerators (e.g. KPU plus an NPU plus a GPU).

---

## 5. State-of-the-skeleton snapshot

At end-of-Phase-0, the repo looks like this:

```
cortex/
├── .clang-format                              # LLVM base, 4-space, ColumnLimit 120
├── .editorconfig                              # UTF-8, LF, trim ws (md excluded)
├── .gitignore                                 # standard CMake / Rust / IDE + .claude/
├── .github/
│   └── workflows/
│       ├── ci.yml                             # 3 jobs: lint, SITL, KPU cross
│       ├── commitlint.yml                     # Conventional Commits gate
│       └── release-please.yml                 # SemVer + CHANGELOG.md automation
├── CLAUDE.md                                  # project conventions for Claude sessions
├── CMakeLists.txt                             # root build: Corrosion + deps + subdirs
├── CMakePresets.json                          # sitl-debug/release, kpu-cross, wsl2-sitl
├── Cargo.lock                                 # (in core/) Rust workspace lockfile
├── LICENSE                                    # MIT
├── README.md                                  # short repo intro (carry-over from seed)
├── cmake/
│   └── deps.cmake                             # FetchContent for the 6 third-party deps
├── commitlint.config.cjs                      # CC type-enum + lowercase-scope + 100-char header
├── config/
│   └── default_bot.yaml                       # placeholder (#74 fills in)
├── core/                                      # Rust Resource Manager — placeholder
│   ├── Cargo.lock
│   ├── Cargo.toml                             # minimal staticlib + rlib
│   └── src/
│       ├── lib.rs                             # placeholder; real bridge in #16
│       └── memory_provider.rs                 # empty; trait lands in #13
├── cross-toolchain.cmake                      # aarch64-linux-gnu for kpu-cross
├── daemons/
│   ├── CMakeLists.txt                         # placeholder
│   └── src/
│       └── vio_daemon.cpp                     # empty; lands in #78
├── docs/
│   ├── adr/
│   │   └── 0001-third-party-license-compatibility.md
│   ├── arch/
│   │   ├── cortex-repo.md                     # full architecture conversation (seed)
│   │   └── phase0-foundation/
│   │       └── README.md                      # this file
│   └── plan/
│       └── bootstrap-plan.md                  # 106-issue roadmap, edited inline
├── math/
│   ├── CMakeLists.txt                         # placeholder
│   └── include/
│       └── branes/
│           └── math/                          # empty; populated in Phase 2 (#24-#32)
├── release-please-config.json                 # SemVer + changelog grouping
├── .release-please-manifest.json              # { ".": "0.0.0" }
├── rust-toolchain.toml                        # channel 1.83.0 + targets
├── scripts/
│   ├── bootstrap.ps1                          # Windows dev setup
│   └── bootstrap.sh                           # Linux/WSL2 dev setup
├── sdk/
│   ├── CMakeLists.txt                         # placeholder (broken SHARED removed)
│   ├── include/
│   │   └── branes/
│   │       └── sdk/                           # empty; populated in Phase 3 (#34, #45)
│   └── src/                                   # empty; library glue lands here
└── tests/
    ├── CMakeLists.txt                         # adds the single smoke test
    └── deps_smoke_test.cpp                    # exercises Catch2 + MTL5 + Universal +
                                               # yaml-cpp + stb_image
```

**Build commands (the official entrypoint):**

```bash
# Host SITL build, day-to-day default
cmake --preset sitl-debug
cmake --build --preset sitl-debug
ctest --preset sitl-debug

# Cross-compile for the KPU silicon (requires gcc-aarch64-linux-gnu)
cmake --preset kpu-cross
cmake --build --preset kpu-cross
```

**What actually compiles today:**
- `libcortex_core.a` — placeholder Rust staticlib (an empty `lib.rs` with a doc comment pointing at #13–#20).
- `deps_smoke_test` — the single Catch2 test that pulls in MTL5, Universal, yaml-cpp, and stb_image.

That's it. No SDK code, no daemons, no math kernels yet. The point of Phase 0 was *infrastructure*; algorithm code arrives starting in Phase 1.

---

## 6. Known gaps deferred

These are real loose threads that did *not* block Phase 0 from closing, but a future contributor should know about them.

### 6.1. Universal upstream fix

The `CMP0169 OLD` workaround in `cmake/deps.cmake` is a temporary hack. The right fix is upstream: have `stillwater-sc/universal` change `${CMAKE_SOURCE_DIR}` to `${PROJECT_SOURCE_DIR}` in its `configure_file` call. Until then, our deps.cmake will keep emitting a (suppressed) deprecation reminder on every cmake configure.

**Action:** file a PR upstream when convenient. Single-line fix.

### 6.2. sccache cache write errors on GHA

The first cold-cache run on PR #113 logged ~25–50% cache write errors (`Cache write errors: 71` on the KPU job, `38` on SITL). No accompanying error message — sccache just counts them. Possible causes: GHA cache backend rate limiting, concurrent-write contention between parallel jobs, or transient GHA backend issues. Effect: the cap on long-run hit rate is bounded by however many of those write-failed entries get re-tried successfully on later runs.

**Action:** none yet. Watch hit rates as more PRs land. If they plateau below ~80%, investigate the GHA cache backend more deeply (likely solution: switch to an S3-backed cache or accept the ceiling).

### 6.3. First release-please run blocked on repo setting

After PR #115 merged, the `release-please` workflow ran but failed at the final PR-create call: *"GitHub Actions is not permitted to create or approve pull requests"*. The branch and commit were created successfully — only the PR-creation API call was blocked.

**Action:** in repo settings (`https://github.com/branes-ai/cortex/settings/actions`), under **Workflow permissions**, enable *"Allow GitHub Actions to create and approve pull requests"*. The next push to `main` will then open the initial release PR (proposed version `0.1.0` per `bump-minor-pre-major`).

### 6.4. KPU cross-compile tests skipped

The `kpu-cross` preset sets `BUILD_TESTING=OFF`. We can cross-compile the test binary just fine, but we have no aarch64 emulator step in CI to *run* it. The cross-compile job currently only verifies that the artifact is valid aarch64 ELF (via `objdump -f`).

**Action:** when the SDK has real C++ code to validate, add a QEMU step to CI (`qemu-user` runs aarch64 binaries on x86 hosts). Issue-able as a follow-up but not blocking Phases 1–3.

### 6.5. clang-format pinned to Ubuntu 24.04 only

The lint job runs on `ubuntu-24.04` because Ubuntu 22.04's clang-format 14 doesn't support all of our `.clang-format` options. The build jobs stay on 22.04 to track our deployment-target Linux versions. If we ever need 22.04 for the lint job too, we'd have to install clang-format-18 from the LLVM apt repo.

**Action:** none. Document only.

### 6.6. release-please version mismatch with `Cargo.toml`

release-please's `release-type: simple` manages a top-level version (currently captured in `.release-please-manifest.json`). It does **not** update `core/Cargo.toml`'s version. When `cortex-core` matters as a published crate (it doesn't yet — `publish = false`), we'll need to switch to a multi-package release-please config that tracks the Rust crate version separately.

**Action:** revisit when `cortex-core` is first published to a registry. Not soon.

---

## 7. For new contributors

If you've just cloned the repo and have never touched it:

```bash
# Linux / WSL2:
scripts/bootstrap.sh --install-deps    # installs cmake, ninja, clang-format, rustup
cmake --preset sitl-debug
cmake --build --preset sitl-debug
ctest --preset sitl-debug

# Windows (PowerShell):
scripts\bootstrap.ps1
cmake --preset sitl-debug
cmake --build --preset sitl-debug
ctest --preset sitl-debug
```

That's the entire dev loop today. Five minutes from a clean machine to passing tests.

**Where to read what:**
- **`CLAUDE.md`** — the load-bearing conventions, terse. Read this first.
- **`docs/arch/cortex-repo.md`** — the original architecture dialogue. Long but rich; gives "why" behind the layout.
- **`docs/plan/bootstrap-plan.md`** — the 12-phase, 106-issue roadmap with decisions inline.
- **`docs/adr/`** — short architectural decisions, numbered. Currently just 0001 (licenses).
- **`docs/arch/phase0-foundation/`** — this directory. Records what Phase 0 actually delivered.

**Where to look for what:**
- Build config: `CMakeLists.txt`, `CMakePresets.json`, `cmake/deps.cmake`, `cross-toolchain.cmake`.
- Style enforcement: `.clang-format`, `.editorconfig`, `commitlint.config.cjs`.
- CI: `.github/workflows/{ci,commitlint,release-please}.yml`.
- Release flow: `release-please-config.json`, `.release-please-manifest.json`.
- Tests: `tests/`.
- Rust core: `core/`. C++ math: `math/`. Operators: `sdk/`. Daemons: `daemons/`. CV primitives: `cv/` (Phase 3).

---

## 8. Forward links

Phase 0 was *prerequisite* work. The MVP critical path runs through:

```
Phase 0 (done) ───┬─── Phase 1: Rust Core (#13-#20 MVP, #21-#23 deferred)
                  │
                  └─── Phase 2: Math (#24-#32)            ──┐
                                                            │
                       Phase 3: VIO (#33-#49)  ◀────────────┘  [MVP target]
                          │
                          ├── Phase 4: Visual SLAM (#50-#60)
                          ├── Phase 5: 3D Scene Graph (#61-#67)
                          └── Phase 6: Config (#68-#74)
                                  │
                                  ▼
                              Phase 7: Daemons (#75-#83)
                                  │
                                  ▼
                              Phase 8: SITL (#84-#89)
                              Phase 9: Profiling (#90-#94)
                              Phase 10: Deployment (#95-#100, deferred)
                              Phase 11: Docs (#101-#106)
```

**Next up: Phase 1 and Phase 2 (parallelizable).** Phase 1 lands the `MemoryProvider` trait + SITL implementation + cxx bridge; Phase 2 lands the header-only math layer (NLS solvers, sparse linear algebra, Lie groups). Both feed into Phase 3 VIO.

Each phase will get its own retrospective doc in `docs/arch/phaseN-<name>/` following this pattern as it closes.
