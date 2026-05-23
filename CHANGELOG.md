# Changelog

All notable changes to this project will be documented in this file.

This project follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html) and
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) conventions. Starting with
the first release, [release-please](https://github.com/googleapis/release-please)
generates entries automatically from Conventional Commits on `main`.

## [0.1.1](https://github.com/branes-ai/cortex/compare/v0.1.0...v0.1.1) (2026-05-23)


### Maintenance

* **testing:** add regression infra — coverage + latency budget framework ([#123](https://github.com/branes-ai/cortex/issues/123)) ([d0c9805](https://github.com/branes-ai/cortex/commit/d0c98051e60966f247b6268384b0e7a7bd428f5f))

## 0.1.0 (2026-05-23)


### Build System

* **cmake:** add CMakePresets.json with sitl/kpu-cross/wsl2 presets ([83ec0fe](https://github.com/branes-ai/cortex/commit/83ec0fe4a05dd46f923faf8729eacdaf35d65100))
* **cmake:** add CMakePresets.json with sitl/kpu-cross/wsl2 presets ([3eb0943](https://github.com/branes-ai/cortex/commit/3eb09438a069192bec9bea483e2d428113f4ed66))
* **cmake:** add cross-toolchain.cmake for KPU aarch64 cross-compile ([3e93b9c](https://github.com/branes-ai/cortex/commit/3e93b9cbc1fb801b254458f5445f31ac17e3440f))
* **cmake:** add cross-toolchain.cmake for KPU aarch64 cross-compile ([ad40c9f](https://github.com/branes-ai/cortex/commit/ad40c9f06ee18b58d773d8de25c40155d376067e))
* **cmake:** wire FetchContent for foundational dependencies ([59345f6](https://github.com/branes-ai/cortex/commit/59345f6c41ff93562b5f20ff37a978e44706ed64))
* **cmake:** wire FetchContent for foundational dependencies ([8316fce](https://github.com/branes-ai/cortex/commit/8316fce336eab515deec8f683a133defc5d33eca))


### Continuous Integration

* add lint job (clippy + cargo fmt + clang-format) ([dd288c8](https://github.com/branes-ai/cortex/commit/dd288c82eba34ec78daf2a67c55ac0ee95940b8a))
* add lint job (clippy + cargo fmt + clang-format) ([2075637](https://github.com/branes-ai/cortex/commit/20756370e458ae8d35ed6ca5eca8012951dfb019))
* install sccache in lint job too (RUSTC_WRAPPER consistency) ([daf0e7d](https://github.com/branes-ai/cortex/commit/daf0e7d5f0cdb7e4cb49fa2b7265b2fa5374aad1))
* integrate sccache for Rust + C++ object caching ([818b018](https://github.com/branes-ai/cortex/commit/818b018302a72fad30471297e1c0f9b64969c4b1))
* integrate sccache for Rust + C++ object caching ([8478572](https://github.com/branes-ai/cortex/commit/84785724f7f1d5648e819c534493578232c8456a))


### Documentation

* add CLAUDE.md and bootstrap plan ([ed34b81](https://github.com/branes-ai/cortex/commit/ed34b81191272be569f7e188dbec9cf42a5b8ad1))
* add CLAUDE.md and bootstrap plan; gitignore .claude/ ([fa7cefa](https://github.com/branes-ai/cortex/commit/fa7cefa38256a35a8d177ca3737915a4475773f9))
* add Phase 0 Foundation retrospective ([5afc918](https://github.com/branes-ai/cortex/commit/5afc9182a200dd5d50d000a16eed61c9eea9b645))
* add Phase 0 Foundation retrospective ([ea4b5f4](https://github.com/branes-ai/cortex/commit/ea4b5f4e405e69a7693b8627ac8baea41e64bef2))


### Maintenance

* **bootstrap:** add bootstrap.sh + bootstrap.ps1 for new-dev setup ([61c0889](https://github.com/branes-ai/cortex/commit/61c08895bde2bf17d37956eefc104b6766277915))
* **bootstrap:** add bootstrap.sh + bootstrap.ps1 for new-dev setup ([2f3167e](https://github.com/branes-ai/cortex/commit/2f3167e0726bd4726883b04205837b9b6a88483a))
* **coderabbit:** add .coderabbit.yaml with layering + license + terminology rules ([#119](https://github.com/branes-ai/cortex/issues/119)) ([7618067](https://github.com/branes-ai/cortex/commit/761806787be80670878047aecd0ba93c1c1cda79))
* **format:** add .clang-format, .editorconfig, and license ADR ([0c3e10c](https://github.com/branes-ai/cortex/commit/0c3e10cc2755145b967ddad59f279f7dd43a0428))
* **format:** add .clang-format, .editorconfig, and license-compatibility ADR ([b45958e](https://github.com/branes-ai/cortex/commit/b45958e5a25ff04fa2173c44c93774c6c7b76002))
* **release:** add commitlint + release-please for SemVer ([d5cf0e1](https://github.com/branes-ai/cortex/commit/d5cf0e15eeaa0f1081dc34133011d4c12e034163))
* **release:** add commitlint + release-please for SemVer ([8e69d68](https://github.com/branes-ai/cortex/commit/8e69d68d05baf1d9dd24ad3fde85e9086bd109f1))
* **release:** pin initial-version to 0.1.0 (alpha pre-1.0 semantics) ([#121](https://github.com/branes-ai/cortex/issues/121)) ([39d8309](https://github.com/branes-ai/cortex/commit/39d8309f58e506278152e96a3af1146fff7a9544))
* **toolchain:** bump Corrosion v0.4.2 -&gt; v0.6.1 ([164d8a5](https://github.com/branes-ai/cortex/commit/164d8a568f43cce5a5349fe28bafc76a49847fca))
* **toolchain:** pin Rust 1.83.0 and unblock skeleton build ([2f83879](https://github.com/branes-ai/cortex/commit/2f83879aa3737ff2fc1649e011bd50cdfc53467d))
* **toolchain:** pin Rust 1.83.0 and unblock skeleton build ([c84504e](https://github.com/branes-ai/cortex/commit/c84504ec82741a87a54fc73ed917543054930b92))
* wrapup Phase 0 — add CHANGELOG.md, session log, gitignore cargo target ([8abbaff](https://github.com/branes-ai/cortex/commit/8abbaff6a14c39a1d4291bfc20b229fa0ca70b2a))
* wrapup Phase 0 — CHANGELOG.md, session log, gitignore target/ ([dbc353f](https://github.com/branes-ai/cortex/commit/dbc353f40d29dd1706daea3f46ce69a4864a27dd))

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
