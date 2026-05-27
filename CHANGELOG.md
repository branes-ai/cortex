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
## [0.20.1](https://github.com/branes-ai/cortex/compare/v0.20.0...v0.20.1) (2026-05-27)


### Documentation

* add Starlight + Doxygen documentation site for Phases 0-3 ([#192](https://github.com/branes-ai/cortex/issues/192)) ([2fac010](https://github.com/branes-ai/cortex/commit/2fac0105d8bc24ecf04eb798c697a9d77b1fa631))

## [0.20.0](https://github.com/branes-ai/cortex/compare/v0.19.0...v0.20.0) (2026-05-26)


### Features

* **sdk:** camera updaters with MSCKF null-space projection ([#185](https://github.com/branes-ai/cortex/issues/185)) ([f16cde0](https://github.com/branes-ai/cortex/commit/f16cde06e542674cb3e626d05ff34cf52c6957d2))
* **sdk:** IMU initialization (static + dynamic) ([#184](https://github.com/branes-ai/cortex/issues/184)) ([6db837d](https://github.com/branes-ai/cortex/commit/6db837d403c07bf34f917ac39e5fe95356c86c0b))
* **sdk:** MSCKF backend implementation ([#186](https://github.com/branes-ai/cortex/issues/186)) ([f02bc78](https://github.com/branes-ai/cortex/commit/f02bc78358572e92345b378295d843c5a6211cba))
* **sdk:** sliding-window optimization backend skeleton ([#189](https://github.com/branes-ai/cortex/issues/189)) ([45b357f](https://github.com/branes-ai/cortex/commit/45b357fa518baa6bbaa58818e1192329ee5c183d))
* **sdk:** state vector + propagator + state-helper (MSCKF core) ([#182](https://github.com/branes-ai/cortex/issues/182)) ([2e6003a](https://github.com/branes-ai/cortex/commit/2e6003ad0bd77de9122eb09057528c8d995928cf))
* **sdk:** top-level vio estimator API ([#188](https://github.com/branes-ai/cortex/issues/188)) ([3546f32](https://github.com/branes-ai/cortex/commit/3546f32ffc89450a3b651222e291fbc6e21a229c))


### Tests

* **sdk:** euroc replay harness + ATE/RPE benchmark ([#190](https://github.com/branes-ai/cortex/issues/190)) ([6fc5c12](https://github.com/branes-ai/cortex/commit/6fc5c12e6f516d4024fcef3c2d95c6462af1bd96))
* **sdk:** latency budget enforcement (per-frame upper bound) ([#191](https://github.com/branes-ai/cortex/issues/191)) ([90a98fa](https://github.com/branes-ai/cortex/commit/90a98fae3f5220678eb0d58f7317649f3d47990b))

## [0.19.0](https://github.com/branes-ai/cortex/compare/v0.18.0...v0.19.0) (2026-05-25)


### Features

* **sdk:** feature representations + MSCKF null-space projection ([#180](https://github.com/branes-ai/cortex/issues/180)) ([deaa092](https://github.com/branes-ai/cortex/commit/deaa092724248fd7e9d3886498c213de564b7648))

## [0.18.0](https://github.com/branes-ai/cortex/compare/v0.17.0...v0.18.0) (2026-05-25)


### Features

* **sdk:** imu preintegration (closed-form, clean-room) ([#178](https://github.com/branes-ai/cortex/issues/178)) ([6c5e1b1](https://github.com/branes-ai/cortex/commit/6c5e1b1531a9b89db7b74d5e698b11ad0c720a59))

## [0.17.0](https://github.com/branes-ai/cortex/compare/v0.16.0...v0.17.0) (2026-05-25)


### Features

* **cv:** sub-pixel pyramidal KLT feature tracker ([#176](https://github.com/branes-ai/cortex/issues/176)) ([67ec732](https://github.com/branes-ai/cortex/commit/67ec732950e680540702593e0f0f040807d67a27))

## [0.16.0](https://github.com/branes-ai/cortex/compare/v0.15.0...v0.16.0) (2026-05-24)


### Features

* **cv:** FAST-9 corner detector with non-maximum suppression ([#174](https://github.com/branes-ai/cortex/issues/174)) ([905b81f](https://github.com/branes-ai/cortex/commit/905b81f0f94810cab2f745fc3fb5f7b5f791cdb1))

## [0.15.0](https://github.com/branes-ai/cortex/compare/v0.14.0...v0.15.0) (2026-05-24)


### Features

* **math:** camera models — pinhole+radtan, fisheye, unified omni ([#172](https://github.com/branes-ai/cortex/issues/172)) ([79a4b52](https://github.com/branes-ai/cortex/commit/79a4b522f35f41f7e38630c2afb362743a0762f8))

## [0.14.0](https://github.com/branes-ai/cortex/compare/v0.13.0...v0.14.0) (2026-05-24)


### Features

* **cv:** gaussian image pyramid with configurable scale ([#170](https://github.com/branes-ai/cortex/issues/170)) ([3f0704a](https://github.com/branes-ai/cortex/commit/3f0704a65d7d8ba3f5fbb0e6f42d99a8f6105e87))

## [0.13.0](https://github.com/branes-ai/cortex/compare/v0.12.3...v0.13.0) (2026-05-24)


### Features

* **cv:** image container + grayscale PGM/PNG I/O ([#169](https://github.com/branes-ai/cortex/issues/169)) ([6e5966b](https://github.com/branes-ai/cortex/commit/6e5966b85ebfa74904840920dbeea47e84b054c0))
* **sdk:** vio backend interface separating front-end from estimator ([bb56e46](https://github.com/branes-ai/cortex/commit/bb56e4628339bb4c5fff1285f6af50bad6d0753c)), closes [#34](https://github.com/branes-ai/cortex/issues/34)
* **sdk:** VioBackend interface separating front-end from estimator ([#167](https://github.com/branes-ai/cortex/issues/167)) ([bb56e46](https://github.com/branes-ai/cortex/commit/bb56e4628339bb4c5fff1285f6af50bad6d0753c))

## [0.12.3](https://github.com/branes-ai/cortex/compare/v0.12.2...v0.12.3) (2026-05-24)


### Bug Fixes

* **math:** correctness fixes from post-merge review of Phase 2 ([#159](https://github.com/branes-ai/cortex/issues/159)) ([87a3a82](https://github.com/branes-ai/cortex/commit/87a3a824e967a799a3754ce1b93487a29f3d90b1))

## [0.12.2](https://github.com/branes-ai/cortex/compare/v0.12.1...v0.12.2) (2026-05-24)


### Documentation

* **math:** Doxygen API docs + concepts reference page ([#157](https://github.com/branes-ai/cortex/issues/157)) ([fdc9aed](https://github.com/branes-ai/cortex/commit/fdc9aed23fd3b6ec3c4a0f1cd33c5f9c9e87d629))
* **math:** doxygen API docs and a concepts reference page ([fdc9aed](https://github.com/branes-ai/cortex/commit/fdc9aed23fd3b6ec3c4a0f1cd33c5f9c9e87d629)), closes [#32](https://github.com/branes-ai/cortex/issues/32)

## [0.12.1](https://github.com/branes-ai/cortex/compare/v0.12.0...v0.12.1) (2026-05-24)


### Tests

* **math:** golden-data NLS suite + Jacobian FD checks + type matrix ([#155](https://github.com/branes-ai/cortex/issues/155)) ([c0ba096](https://github.com/branes-ai/cortex/commit/c0ba096d29174bf8400d823b0d6ec4c50cea4612)), closes [#31](https://github.com/branes-ai/cortex/issues/31)

## [0.12.0](https://github.com/branes-ai/cortex/compare/v0.11.0...v0.12.0) (2026-05-24)


### Features

* **math:** non-linear least squares solvers (GN, LM, Dogleg) ([aac636c](https://github.com/branes-ai/cortex/commit/aac636cca3a4cefce5b2bf6b48174b3941d4cd84)), closes [#30](https://github.com/branes-ai/cortex/issues/30)
* **math:** non-linear least squares solvers (gn, lm, dogleg) ([#153](https://github.com/branes-ai/cortex/issues/153)) ([aac636c](https://github.com/branes-ai/cortex/commit/aac636cca3a4cefce5b2bf6b48174b3941d4cd84))

## [0.11.0](https://github.com/branes-ai/cortex/compare/v0.10.0...v0.11.0) (2026-05-24)


### Features

* **math:** direct sparse Cholesky/LDLT for SPD normal equations ([#151](https://github.com/branes-ai/cortex/issues/151)) ([46687ff](https://github.com/branes-ai/cortex/commit/46687ffae48de72c231a940b413e15e2328c6be7)), closes [#28](https://github.com/branes-ai/cortex/issues/28)

## [0.10.0](https://github.com/branes-ai/cortex/compare/v0.9.0...v0.10.0) (2026-05-24)


### Features

* **math:** Krylov solvers (CG, GMRES, BiCGSTAB) over MTL5 ITL ([#149](https://github.com/branes-ai/cortex/issues/149)) ([d0c3a93](https://github.com/branes-ai/cortex/commit/d0c3a9330f88ba19d13a1f77a1c3b00a5e53cb2e))
* **math:** thin Krylov solver wrappers (cg, gmres, bicgstab) over MTL5 ITL ([d0c3a93](https://github.com/branes-ai/cortex/commit/d0c3a9330f88ba19d13a1f77a1c3b00a5e53cb2e)), closes [#27](https://github.com/branes-ai/cortex/issues/27)

## [0.9.0](https://github.com/branes-ai/cortex/compare/v0.8.0...v0.9.0) (2026-05-24)


### Features

* **math:** header-only Lie groups SO3, SE3, Sim3 ([#147](https://github.com/branes-ai/cortex/issues/147)) ([6efe30f](https://github.com/branes-ai/cortex/commit/6efe30fd35529d0b4a0b92ba8c85c2fd37d97532)), closes [#29](https://github.com/branes-ai/cortex/issues/29)

## [0.8.0](https://github.com/branes-ai/cortex/compare/v0.7.0...v0.8.0) (2026-05-24)


### Features

* **math:** sparse storage views (CSR, CSC, COO) over caller buffers ([#145](https://github.com/branes-ai/cortex/issues/145)) ([957e8ca](https://github.com/branes-ai/cortex/commit/957e8ca20c62c198d81a1ef4ec32d727f7a243b6)), closes [#26](https://github.com/branes-ai/cortex/issues/26)

## [0.7.0](https://github.com/branes-ai/cortex/compare/v0.6.0...v0.7.0) (2026-05-24)


### Features

* **math:** arithmetic-type plumbing for MTL5 over Universal types ([#143](https://github.com/branes-ai/cortex/issues/143)) ([c98df33](https://github.com/branes-ai/cortex/commit/c98df330cc823e3ebd78119c35203bc75a6ac4fb)), closes [#25](https://github.com/branes-ai/cortex/issues/25)

## [0.6.0](https://github.com/branes-ai/cortex/compare/v0.5.3...v0.6.0) (2026-05-24)


### Features

* **math:** tensor views over std::span with stride/shape metadata ([081e66f](https://github.com/branes-ai/cortex/commit/081e66f334b767e355122079805629250140e62e)), closes [#24](https://github.com/branes-ai/cortex/issues/24)
* **math:** TensorView&lt;T,Rank&gt; over std::span with stride/shape metadata ([#141](https://github.com/branes-ai/cortex/issues/141)) ([081e66f](https://github.com/branes-ai/cortex/commit/081e66f334b767e355122079805629250140e62e))

## [0.5.3](https://github.com/branes-ai/cortex/compare/v0.5.2...v0.5.3) (2026-05-24)


### Maintenance

* wrapup 2026-05-24 — Phase 1 MVP session log + CHANGELOG marker ([#139](https://github.com/branes-ai/cortex/issues/139)) ([f0cab34](https://github.com/branes-ai/cortex/commit/f0cab347a6434489e350da97367ac70b1304cc81))

## [0.5.2](https://github.com/branes-ai/cortex/compare/v0.5.1...v0.5.2) (2026-05-24)


### Tests

* **bridge:** add C++/Rust cxx::bridge integration test ([#137](https://github.com/branes-ai/cortex/issues/137)) ([53ab21c](https://github.com/branes-ai/cortex/commit/53ab21c940359c0cf3971a4819f512e755ac5ca3))

## [0.5.1](https://github.com/branes-ai/cortex/compare/v0.5.0...v0.5.1) (2026-05-24)


### Continuous Integration

* gate Rust coverage at &gt;= 85% line coverage ([#135](https://github.com/branes-ai/cortex/issues/135)) ([486466c](https://github.com/branes-ai/cortex/commit/486466c9c9c6b85d1f4c02203e37fe1125e97548))

## [0.5.0](https://github.com/branes-ai/cortex/compare/v0.4.0...v0.5.0) (2026-05-24)


### Features

* **core:** add cxx bridge, build.rs, and lifecycle state machine ([#133](https://github.com/branes-ai/cortex/issues/133)) ([ff45bf9](https://github.com/branes-ai/cortex/commit/ff45bf9a0d2a00ee03c8905161a06817ab45534f))

## [0.4.0](https://github.com/branes-ai/cortex/compare/v0.3.0...v0.4.0) (2026-05-23)


### Features

* **core:** add KPU MemoryProvider stub ([#131](https://github.com/branes-ai/cortex/issues/131)) ([cf519ab](https://github.com/branes-ai/cortex/commit/cf519ab3ba1c5e8ef5eade206280ffb428a57451))

## [0.3.0](https://github.com/branes-ai/cortex/compare/v0.2.0...v0.3.0) (2026-05-23)


### Features

* **core:** add SITL MemoryProvider with shm + heap backends ([7075ac7](https://github.com/branes-ai/cortex/commit/7075ac7d50cc1827f918724b0cdc29d6553db123))
* **core:** SITL MemoryProvider — Linux shm + heap backends ([#129](https://github.com/branes-ai/cortex/issues/129)) ([7075ac7](https://github.com/branes-ai/cortex/commit/7075ac7d50cc1827f918724b0cdc29d6553db123))

## [0.2.0](https://github.com/branes-ai/cortex/compare/v0.1.2...v0.2.0) (2026-05-23)


### Features

* **core:** define MemoryProvider trait + BufferHandle + MemErr ([#127](https://github.com/branes-ai/cortex/issues/127)) ([fd6a755](https://github.com/branes-ai/cortex/commit/fd6a7558e27798502bc3dfca532e034b58e1823e))

## [0.1.2](https://github.com/branes-ai/cortex/compare/v0.1.1...v0.1.2) (2026-05-23)


### Documentation

* add Phase 1 design — Rust RM + cxx bridge ([#125](https://github.com/branes-ai/cortex/issues/125)) ([d23eeec](https://github.com/branes-ai/cortex/commit/d23eeec017a57f6aefef9012940699ddfc639c23))

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
