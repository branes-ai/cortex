# Changelog

All notable changes to this project will be documented in this file.

This project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html) and
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) conventions. Starting with
the first release, [release-please](https://github.com/googleapis/release-please)
generates entries automatically from Conventional Commits on `main`.

<!-- Manual milestone markers. release-please appends release sections
     below this block, so these entries survive across releases. Keep
     them brief — one line per phase closure; detailed narrative lives
     in docs/sessions/. -->

## Milestones

- **2026-05-24 — Phase 1 MVP complete (v0.1.0 → v0.6.0).** Rust Resource
  Manager + cxx bridge in place. All eight MVP sub-issues #13–#20 closed:
  `MemoryProvider` trait, SITL provider (Linux shm + heap), KPU stub,
  cxx::bridge + build.rs + lifecycle state machine, ≥85% coverage gate,
  C++/Rust integration test. Critical path now unblocks Phase 2 (math)
  and Phase 3 (VIO). See [`docs/sessions/2026-05-24-phase-1-mvp-complete.md`](docs/sessions/2026-05-24-phase-1-mvp-complete.md).
- **2026-05-22 — Phase 0 (foundation) complete.** Build system, CI, release
  flow, dev-environment scripts. See [`docs/arch/phase0-foundation/README.md`](docs/arch/phase0-foundation/README.md).
## [0.15.0](https://github.com/branes-ai/cortex/compare/v0.14.0...v0.15.0) (2026-05-24)


### Features

* **math:** camera models — pinhole+radtan, fisheye, unified omni ([#172](https://github.com/branes-ai/cortex/issues/172)) ([8ce771b](https://github.com/branes-ai/cortex/commit/8ce771bd1eb377ed07c972409e8a035eb8e31d26))

## [0.14.0](https://github.com/branes-ai/cortex/compare/v0.13.0...v0.14.0) (2026-05-24)


### Features

* **cv:** gaussian image pyramid with configurable scale ([#170](https://github.com/branes-ai/cortex/issues/170)) ([afefa22](https://github.com/branes-ai/cortex/commit/afefa225c70371bca72af8cb75575b50f17cf587))

## [0.13.0](https://github.com/branes-ai/cortex/compare/v0.12.3...v0.13.0) (2026-05-24)


### Features

* **cv:** image container + grayscale PGM/PNG I/O ([#169](https://github.com/branes-ai/cortex/issues/169)) ([087a432](https://github.com/branes-ai/cortex/commit/087a432d03455d53cdf5e87b170f320e43d4ef86))
* **sdk:** vio backend interface separating front-end from estimator ([377dfaa](https://github.com/branes-ai/cortex/commit/377dfaace0d0a9709b1836e763c9d41c58048db7)), closes [#34](https://github.com/branes-ai/cortex/issues/34)
* **sdk:** VioBackend interface separating front-end from estimator ([#167](https://github.com/branes-ai/cortex/issues/167)) ([377dfaa](https://github.com/branes-ai/cortex/commit/377dfaace0d0a9709b1836e763c9d41c58048db7))

## [0.12.3](https://github.com/branes-ai/cortex/compare/v0.12.2...v0.12.3) (2026-05-24)


### Bug Fixes

* **math:** correctness fixes from post-merge review of Phase 2 ([#159](https://github.com/branes-ai/cortex/issues/159)) ([fbb35e0](https://github.com/branes-ai/cortex/commit/fbb35e00c336ff9834e6138fcbd9eab1a98285f1))

## [0.12.2](https://github.com/branes-ai/cortex/compare/v0.12.1...v0.12.2) (2026-05-24)


### Documentation

* **math:** Doxygen API docs + concepts reference page ([#157](https://github.com/branes-ai/cortex/issues/157)) ([f54a245](https://github.com/branes-ai/cortex/commit/f54a245ec34a81b7dd589b8a9edc6c225849bea3))
* **math:** doxygen API docs and a concepts reference page ([f54a245](https://github.com/branes-ai/cortex/commit/f54a245ec34a81b7dd589b8a9edc6c225849bea3)), closes [#32](https://github.com/branes-ai/cortex/issues/32)

## [0.12.1](https://github.com/branes-ai/cortex/compare/v0.12.0...v0.12.1) (2026-05-24)


### Tests

* **math:** golden-data NLS suite + Jacobian FD checks + type matrix ([#155](https://github.com/branes-ai/cortex/issues/155)) ([2348e87](https://github.com/branes-ai/cortex/commit/2348e87e5615d43bcc8b7f5c11d7c6919e629653)), closes [#31](https://github.com/branes-ai/cortex/issues/31)

## [0.12.0](https://github.com/branes-ai/cortex/compare/v0.11.0...v0.12.0) (2026-05-24)


### Features

* **math:** non-linear least squares solvers (GN, LM, Dogleg) ([d85444d](https://github.com/branes-ai/cortex/commit/d85444d169255098258622cc1f621dcf59a503dd)), closes [#30](https://github.com/branes-ai/cortex/issues/30)
* **math:** non-linear least squares solvers (gn, lm, dogleg) ([#153](https://github.com/branes-ai/cortex/issues/153)) ([d85444d](https://github.com/branes-ai/cortex/commit/d85444d169255098258622cc1f621dcf59a503dd))

## [0.11.0](https://github.com/branes-ai/cortex/compare/v0.10.0...v0.11.0) (2026-05-24)


### Features

* **math:** direct sparse Cholesky/LDLT for SPD normal equations ([#151](https://github.com/branes-ai/cortex/issues/151)) ([818ef32](https://github.com/branes-ai/cortex/commit/818ef32433ffee70b896ae3a01d34b702713a950)), closes [#28](https://github.com/branes-ai/cortex/issues/28)

## [0.10.0](https://github.com/branes-ai/cortex/compare/v0.9.0...v0.10.0) (2026-05-24)


### Features

* **math:** Krylov solvers (CG, GMRES, BiCGSTAB) over MTL5 ITL ([#149](https://github.com/branes-ai/cortex/issues/149)) ([f27b47c](https://github.com/branes-ai/cortex/commit/f27b47c959f0e126c879a2431873fc09cc5d95d1))
* **math:** thin Krylov solver wrappers (cg, gmres, bicgstab) over MTL5 ITL ([f27b47c](https://github.com/branes-ai/cortex/commit/f27b47c959f0e126c879a2431873fc09cc5d95d1)), closes [#27](https://github.com/branes-ai/cortex/issues/27)

## [0.9.0](https://github.com/branes-ai/cortex/compare/v0.8.0...v0.9.0) (2026-05-24)


### Features

* **math:** header-only Lie groups SO3, SE3, Sim3 ([#147](https://github.com/branes-ai/cortex/issues/147)) ([3dd410f](https://github.com/branes-ai/cortex/commit/3dd410fa8170f4668452d6206d7a281073b49d38)), closes [#29](https://github.com/branes-ai/cortex/issues/29)

## [0.8.0](https://github.com/branes-ai/cortex/compare/v0.7.0...v0.8.0) (2026-05-24)


### Features

* **math:** sparse storage views (CSR, CSC, COO) over caller buffers ([#145](https://github.com/branes-ai/cortex/issues/145)) ([d556f14](https://github.com/branes-ai/cortex/commit/d556f144f66dc697e831dc443a7f00719b901216)), closes [#26](https://github.com/branes-ai/cortex/issues/26)

## [0.7.0](https://github.com/branes-ai/cortex/compare/v0.6.0...v0.7.0) (2026-05-24)


### Features

* **math:** arithmetic-type plumbing for MTL5 over Universal types ([#143](https://github.com/branes-ai/cortex/issues/143)) ([4163b22](https://github.com/branes-ai/cortex/commit/4163b2261da08fc1058c6678df067077d0e370b8)), closes [#25](https://github.com/branes-ai/cortex/issues/25)

## [0.6.0](https://github.com/branes-ai/cortex/compare/v0.5.3...v0.6.0) (2026-05-24)


### Features

* **math:** tensor views over std::span with stride/shape metadata ([bb868d5](https://github.com/branes-ai/cortex/commit/bb868d5bda1ec3fc683fa5a1957701a41cdda041)), closes [#24](https://github.com/branes-ai/cortex/issues/24)
* **math:** TensorView&lt;T,Rank&gt; over std::span with stride/shape metadata ([#141](https://github.com/branes-ai/cortex/issues/141)) ([bb868d5](https://github.com/branes-ai/cortex/commit/bb868d5bda1ec3fc683fa5a1957701a41cdda041))

## [0.5.3](https://github.com/branes-ai/cortex/compare/v0.5.2...v0.5.3) (2026-05-24)


### Maintenance

* wrapup 2026-05-24 — Phase 1 MVP session log + CHANGELOG marker ([#139](https://github.com/branes-ai/cortex/issues/139)) ([215cae4](https://github.com/branes-ai/cortex/commit/215cae44114b888c7e44834c665b309d3525aeca))

## [0.5.2](https://github.com/branes-ai/cortex/compare/v0.5.1...v0.5.2) (2026-05-24)


### Tests

* **bridge:** add C++/Rust cxx::bridge integration test ([#137](https://github.com/branes-ai/cortex/issues/137)) ([50e3e3c](https://github.com/branes-ai/cortex/commit/50e3e3c15e89c47c710f20f67fd7b655f8657d79))

## [0.5.1](https://github.com/branes-ai/cortex/compare/v0.5.0...v0.5.1) (2026-05-24)


### Continuous Integration

* gate Rust coverage at &gt;= 85% line coverage ([#135](https://github.com/branes-ai/cortex/issues/135)) ([739c12c](https://github.com/branes-ai/cortex/commit/739c12cc3ab85c05df9622c7d7130131eac084b8))

## [0.5.0](https://github.com/branes-ai/cortex/compare/v0.4.0...v0.5.0) (2026-05-24)


### Features

* **core:** add cxx bridge, build.rs, and lifecycle state machine ([#133](https://github.com/branes-ai/cortex/issues/133)) ([3ee12fb](https://github.com/branes-ai/cortex/commit/3ee12fb13fcde29844c70b35d226a7d9e2a59755))

## [0.4.0](https://github.com/branes-ai/cortex/compare/v0.3.0...v0.4.0) (2026-05-23)


### Features

* **core:** add KPU MemoryProvider stub ([#131](https://github.com/branes-ai/cortex/issues/131)) ([a2445c8](https://github.com/branes-ai/cortex/commit/a2445c859adad8c5d753e6249001df6e4c0d183a))

## [0.3.0](https://github.com/branes-ai/cortex/compare/v0.2.0...v0.3.0) (2026-05-23)


### Features

* **core:** add SITL MemoryProvider with shm + heap backends ([3d99738](https://github.com/branes-ai/cortex/commit/3d99738832746b5b32f6ccf0c79cce118b882f39))
* **core:** SITL MemoryProvider — Linux shm + heap backends ([#129](https://github.com/branes-ai/cortex/issues/129)) ([3d99738](https://github.com/branes-ai/cortex/commit/3d99738832746b5b32f6ccf0c79cce118b882f39))

## [0.2.0](https://github.com/branes-ai/cortex/compare/v0.1.2...v0.2.0) (2026-05-23)


### Features

* **core:** define MemoryProvider trait + BufferHandle + MemErr ([#127](https://github.com/branes-ai/cortex/issues/127)) ([5c1d67a](https://github.com/branes-ai/cortex/commit/5c1d67a7658dd9d9ef2814bcc1e7d277bdd31eb5))

## [0.1.2](https://github.com/branes-ai/cortex/compare/v0.1.1...v0.1.2) (2026-05-23)


### Documentation

* add Phase 1 design — Rust RM + cxx bridge ([#125](https://github.com/branes-ai/cortex/issues/125)) ([bec0037](https://github.com/branes-ai/cortex/commit/bec0037ffeeda49bfaa2a7a632343c8991ae81ea))

## [0.1.1](https://github.com/branes-ai/cortex/compare/v0.1.0...v0.1.1) (2026-05-23)


### Maintenance

* **testing:** add regression infra — coverage + latency budget framework ([#123](https://github.com/branes-ai/cortex/issues/123)) ([5e62988](https://github.com/branes-ai/cortex/commit/5e62988d70cb5e2df143b929d9696423aa4e6b21))

## 0.1.0 (2026-05-23)


### Build System

* **cmake:** add CMakePresets.json with sitl/kpu-cross/wsl2 presets ([89440ff](https://github.com/branes-ai/cortex/commit/89440ffadd94e479e6debb71e985a628f18d230c))
* **cmake:** add CMakePresets.json with sitl/kpu-cross/wsl2 presets ([8d37aa8](https://github.com/branes-ai/cortex/commit/8d37aa850b191298e000dbaac5216b1cded612b6))
* **cmake:** add cross-toolchain.cmake for KPU aarch64 cross-compile ([d6b1471](https://github.com/branes-ai/cortex/commit/d6b147147d34ce2bbfc620478901a5ed3e635a3c))
* **cmake:** add cross-toolchain.cmake for KPU aarch64 cross-compile ([89e0e1c](https://github.com/branes-ai/cortex/commit/89e0e1c35fd1f53ee9c4c8ce77acc767c6d80fd1))
* **cmake:** wire FetchContent for foundational dependencies ([8eb02f8](https://github.com/branes-ai/cortex/commit/8eb02f885effd3a1474016d9fd001701d1792839))
* **cmake:** wire FetchContent for foundational dependencies ([0e6dc7c](https://github.com/branes-ai/cortex/commit/0e6dc7c6db10e6837f5e63455163a84250fbbf63))


### Continuous Integration

* add lint job (clippy + cargo fmt + clang-format) ([2622eb7](https://github.com/branes-ai/cortex/commit/2622eb79b6e0df2c29ebb8622517d59b4405b07f))
* add lint job (clippy + cargo fmt + clang-format) ([75d3c91](https://github.com/branes-ai/cortex/commit/75d3c91f96968fb4c74b80d0256f304b851f1717))
* install sccache in lint job too (RUSTC_WRAPPER consistency) ([dd341a0](https://github.com/branes-ai/cortex/commit/dd341a086faecd364fc915410ecf99b1a15020e0))
* integrate sccache for Rust + C++ object caching ([69851c6](https://github.com/branes-ai/cortex/commit/69851c61008f20dda729f6d9833e0aabfbe6f67c))
* integrate sccache for Rust + C++ object caching ([0865db2](https://github.com/branes-ai/cortex/commit/0865db201b35be3403a788fd33b176afecf783ef))


### Documentation

* add CLAUDE.md and bootstrap plan ([c2dc4f1](https://github.com/branes-ai/cortex/commit/c2dc4f1c10aa2b4992b9d0b3369a99e2630cb631))
* add CLAUDE.md and bootstrap plan; gitignore .claude/ ([4509cca](https://github.com/branes-ai/cortex/commit/4509ccabfb9f7b6d8623f274e10088202961877d))
* add Phase 0 Foundation retrospective ([589a74e](https://github.com/branes-ai/cortex/commit/589a74e52350749ede14238367f069d7da54d3e8))
* add Phase 0 Foundation retrospective ([6e2bd0e](https://github.com/branes-ai/cortex/commit/6e2bd0e4c06797153ab15a9aaf65e9314666b0a5))


### Maintenance

* **bootstrap:** add bootstrap.sh + bootstrap.ps1 for new-dev setup ([602b172](https://github.com/branes-ai/cortex/commit/602b17209a13e3606d59137f7dbb233a1b7d6302))
* **bootstrap:** add bootstrap.sh + bootstrap.ps1 for new-dev setup ([a5d2397](https://github.com/branes-ai/cortex/commit/a5d2397276e35b0dbf97892309f38c76632fe21b))
* **coderabbit:** add .coderabbit.yaml with layering + license + terminology rules ([#119](https://github.com/branes-ai/cortex/issues/119)) ([5abf053](https://github.com/branes-ai/cortex/commit/5abf0539ff83bed548f098d923b289320f7b8809))
* **format:** add .clang-format, .editorconfig, and license ADR ([c95aadb](https://github.com/branes-ai/cortex/commit/c95aadb6a5083d9d8140ac08ca72754f0d421e34))
* **format:** add .clang-format, .editorconfig, and license-compatibility ADR ([b1f52fd](https://github.com/branes-ai/cortex/commit/b1f52fdb103165f6c028679269c8d57784187ae6))
* **release:** add commitlint + release-please for SemVer ([8cd388b](https://github.com/branes-ai/cortex/commit/8cd388b2772c57a14380752677adcf809e9e7f69))
* **release:** add commitlint + release-please for SemVer ([24e2eab](https://github.com/branes-ai/cortex/commit/24e2eab68bc7e138067756cf7481b2e9877a3f6b))
* **release:** pin initial-version to 0.1.0 (alpha pre-1.0 semantics) ([#121](https://github.com/branes-ai/cortex/issues/121)) ([527a620](https://github.com/branes-ai/cortex/commit/527a620daac9d1d4c437eb568e53c915995bbaf2))
* **toolchain:** bump Corrosion v0.4.2 -&gt; v0.6.1 ([bf96317](https://github.com/branes-ai/cortex/commit/bf9631756731dcb0879c8db80738026cf02f7a4d))
* **toolchain:** pin Rust 1.83.0 and unblock skeleton build ([417bd10](https://github.com/branes-ai/cortex/commit/417bd1049548cca2d231cddfaca15e483ea5b6fd))
* **toolchain:** pin Rust 1.83.0 and unblock skeleton build ([68b4c62](https://github.com/branes-ai/cortex/commit/68b4c62e8a0faf2e953cdba8ae6e769479dc77a4))
* wrapup Phase 0 — add CHANGELOG.md, session log, gitignore cargo target ([fcb855e](https://github.com/branes-ai/cortex/commit/fcb855e4a2c70d0908be27ae3702e00f4cd75ee9))
* wrapup Phase 0 — CHANGELOG.md, session log, gitignore target/ ([37143d4](https://github.com/branes-ai/cortex/commit/37143d43c8950b609c125f9eca29062f19f05b88))

## [Unreleased]

### Phase 0 — Foundation

Bootstrap of the cortex repository: build system, dependency management, CI gates,
and developer onboarding. No user-facing functionality yet — the Phase 0 work is
infrastructure that unblocks Phase 1+ implementation. See
[docs/arch/phase0-foundation/README.md](docs/arch/phase0-foundation/README.md) for
the full retrospective.

#### Build System

- Pin the Rust toolchain at `1.83.0` via `rust-toolchain.toml`, with targets
  `x86_64-pc-windows-msvc`, `aarch64-unknown-linux-gnu`, `riscv64gc-unknown-linux-gnu`
  (#108, closes #4).
- Add `CMakePresets.json` with four presets: `sitl-debug`, `sitl-release`,
  `kpu-cross`, `wsl2-sitl`. Generator: Ninja single-config (#110, closes #5).
- Add `cross-toolchain.cmake` for aarch64 KPU cross-compile via the Debian/Ubuntu
  multiarch toolchain. Sysroot is opt-in via `CROSS_SYSROOT` (#111, closes #6).
- Wire `FetchContent` for foundational dependencies: MTL5 v5.2.1, Universal v4.7.0,
  yaml-cpp 0.9.0, Catch2 v3.15.0, Tracy v0.13.1, stb (commit `31c1ad3`). Centralized
  in `cmake/deps.cmake` (#112, closes #7).
- Bump Corrosion v0.4.2 → v0.6.1 to fix a `rustc --version` parse bug on Rust 1.80+.
- Rename `BUILD_TARGET_SOC` → `BUILD_TARGET_KPU` to match the silicon's actual name.

#### Continuous Integration

- Integrate sccache via `mozilla-actions/sccache-action@v0.0.10` with GitHub Actions
  cache backend. Warm-cache hit rates: SITL 73.4%, KPU cross 50.0% (#113, closes #8).
- Add lint job: `cargo fmt --check`, `cargo clippy --all-targets --all-features
  -- -D warnings`, `clang-format --dry-run --Werror` on every tracked C/C++ file
  (#114, closes #9).
- Add Conventional Commits enforcement via `wagoid/commitlint-github-action@v6.2.1`
  on every PR and push to `main` (#115, closes #10).
- Re-enable the KPU cross-compile CI job (previously stubbed out pending the
  toolchain file).

#### Release Automation

- Wire `googleapis/release-please-action@v5.0.0` for SemVer + CHANGELOG automation.
  `release-type: simple`; initial version `0.0.0` in `.release-please-manifest.json`
  (#115, closes #10).
- Conventional Commit type-to-section mapping pinned in `release-please-config.json`.

#### Style and Conventions

- Add `.clang-format` (LLVM base, 4-space indent, ColumnLimit 120) and
  `.editorconfig` (UTF-8 + LF + trim trailing whitespace, markdown exempted)
  (#109, closes #12).
- Ratify the layering rules in `CLAUDE.md`: `core/` (Rust) → `math/` (header-only)
  → `cv/` (Phase 3) → `sdk/` (operators) → `daemons/` (Zenoh wrappers). Strict
  one-direction dependency.
- Ratify the **no-GPL** policy across the entire dependency graph, including for
  testing. See [ADR-0001](docs/adr/0001-third-party-license-compatibility.md).
- Ratify the **hybrid SDK** model: header-only templated operators for arithmetic
  generality, library glue for I/O / lifecycle / non-template surface.
- Standardize on **"domain flow programs"** as the DFA-graph-compiler output
  terminology (replacing earlier draft term "streamer programs").
- Establish the **PR-only workflow** for the project, even with a single contributor.

#### Developer Experience

- Add `scripts/bootstrap.sh` (Linux/WSL2, idempotent, `--install-deps` flag) and
  `scripts/bootstrap.ps1` (Windows, PowerShell 5.1+, idempotent) for one-shot
  dev environment setup (#116, closes #11).

#### Project Tracking

- File 106 issues across 12 epics, milestones, labels, and the Branes CORTEX
  project board. Issue-creation defaults (assignee, project, estimate, milestone,
  GitHub Issue Type) automated.
- Add [`docs/plan/bootstrap-plan.md`](docs/plan/bootstrap-plan.md) — the 12-phase
  roadmap with judgment-call decisions inline.
- Add [`docs/arch/phase0-foundation/README.md`](docs/arch/phase0-foundation/README.md)
  — Phase 0 retrospective. Establishes the `docs/arch/phaseN-<name>/` pattern for
  future phases.

[Unreleased]: https://github.com/branes-ai/cortex/compare/...HEAD
