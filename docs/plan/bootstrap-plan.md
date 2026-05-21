# `branes-ai/cortex` Bootstrap Plan

**Status:** DRAFT — awaiting review. Edit this file directly. When done, tell Claude "plan is updated" and it will diff against this version before generating GitHub epics + sub-issues.

**Source for design decisions referenced below:** [`docs/arch/cortex-repo.md`](../arch/cortex-repo.md).

**Reference repos studied (peer directories on this machine):**
- `MINS` — modular multi-sensor INS (built on OpenVINS). License: GPL-3.0. Pattern reference only, no source vendoring.
- `isaac_ros_visual_slam` — thin ROS 2 wrapper around NVIDIA's closed cuVSLAM. Lessons are middleware-adapter shape, not algorithm internals.

---

## MVP definition (please confirm or edit)

**v0.1 = a header-only C++20 SDK that performs VIO on EuRoC MAV data in SITL, callable from a thin Zenoh daemon, with the Rust Resource Manager stubbed for SITL only.**

Out of MVP: SLAM loop closure, 3D Scene Graph beyond skeleton, KPU/SoC build path, OTA, OCI/Yocto delivery, MLIR/DFA hookup.

> **Editor:** confirm, or rewrite the MVP boundary here.

---

## Open judgment calls (answer inline)

### 1. Header-only for the entire SDK, not just math?
The conversation in `docs/arch/cortex-repo.md` and your message both say "header-only component library for VIO/SLAM/3DSG." But `sdk/CMakeLists.txt` currently builds `cortex_sdk SHARED src/vio.cpp src/scene_graph.cpp`.
- **If header-only:** delete `sdk/src/` and the `SHARED` library; operators are templated headers parameterized on arithmetic type. Consistent with "purity of precision" but harder for compile times and ABI stability.
- **If hybrid:** keep `.cpp` for non-template glue (I/O, calibration loaders).
- **Claude's lean:** header-only with thin `.cpp` allowed only for non-template glue.
- **Your decision:** hybrid and Claude's lean. the core operators need to be parameterized in arithmetic type so that we can apply mixed-precision optimization, but there will be file loading and unloading, resource management, etc. that may or may not be parameterized, and that glue logic clearly should be compile and consumed as a library.

### 2. VIO algorithm family
- **MSCKF** (MINS/OpenVINS — fast, bounded state) vs.
- **Sliding-window optimization** (VINS-Fusion/ORB-SLAM3 — more accurate, more compute).
- **Claude's lean:** start with MSCKF (closest reference); add SW-optimization later as a second backend sharing the same front-end.
- **Your decision:** start with MSCKF, add Sliding-window-optimization skeleton as a second backend sharing the same front-end so that we have the pluggable backend functionality present and ready to receive other algorithms.

### 3. Vendor open_vins as a third-party submodule for testing only?
GPL-3.0 contaminates anything it links against, so it cannot be an SDK dependency. Useful only as a SITL ground-truth oracle in a separate test target.
- **Your decision (yes / no / zero-GPL-contact):** no GPL allowed whatsoever, not even for testing. We would need an clean-room implementation of operators and test methodology.

### 4. SoC/KPU phases — scaffolding now or fully deferred?
Until silicon, the Rust RM only needs the SITL `MemoryProvider`. The IPC broker, computational-spacetime scheduling, and crash-only recovery don't gate the VIO MVP.
- **Claude's lean:** scaffold the trait/API now, defer implementations.
- **Your decision:** scaffold the trait/API now, note implementation plan and outline. By the way, computational spacetime scheduling is NOT per se a component of the Rust RM. computational spacetime scheduling is the task of the DFA graph compiler, which will fuse parts of the computational graphand create a scheduled entity. This will constitute a domain flow program which will be placed on allocated hardware resources. At that point, the KPU takes care of the scheduling, as it is a data flow machine. The Rust RM needs to manage these allocations, not the schedules.

### 5. Issue tracker target
- GitHub Issues on `branes-ai/cortex`? Other system? Label conventions?
- **Your decision:** yes, `branes-ai/cortex` holds tracking issues, use conventional commit workflow and dynamic updating SEMVER.

### 6. 3DSG depth
Real Bayesian-update math, or just a data-oriented container with TODO hooks?
- **Claude's lean:** scaffold the data structure, defer the math.
- **Your decision:** scaffold the data structure, defer the math.

### 7. OpenCV as a SITL-time dependency? (raised inside E3)
Front-end tracker (KLT + FAST) is straightforward against OpenCV. Replacing it in-tree is multi-month work.
- **Claude's lean:** accept OpenCV as a SITL-time dependency; it's gated out of the SoC build.
- **Your decision:** do not include OpenCV, it is too big, we should construct a CV stack from our own requirements. Look very careful at OpenCV and try to stay compatible so that moving potential customers to our stack from OpenCV would be trivial for them.
- **Scope clarification (post-audit):** the operators VIO + SLAM actually need are small — image container + I/O, pyramid, FAST detector, sub-pixel KLT tracker, one descriptor (ORB or BRIEF), brute-force matcher. ~3000 LOC of clean-room code total. Camera distortion math lives in `math/` next to Lie groups; calibration loader lives in `config/`. No standalone CV epic; sub-issues distributed across E3 (front-end) and E4 (descriptors). API signatures shaped to mirror `cv::` so customer migration is a namespace replace.

### 8. Automatic differentiation or analytical Jacobians? (raised inside E2)
- AD adds significant scope and template machinery.
- Analytical Jacobians are faster, more brittle, and the standard in production VIO.
- **Claude's lean:** analytical only for now.
- **Your decision:** analytical only for now.

---

## Epic structure

Each epic gets a tracking issue; each bullet is a candidate sub-issue. Dependencies in `[deps:…]`.

### Phase 0 — Toolchain & repo plumbing  *(blocks everything else)*

**E0 · Build & toolchain foundation**
- Pin Rust via `rust-toolchain.toml` (channel, components, targets including `aarch64-unknown-linux-gnu`, `riscv64gc-unknown-linux-gnu`, and `x86_64-pc-windows-msvc`).
- `CMakePresets.json`: presets for `sitl-debug`, `sitl-release`, `kpu-cross`, `wsl2-sitl`.
- Create the missing `cross-toolchain.cmake` referenced by CI (currently broken).
- Wire `FetchContent` for MTL5, Universal, yaml-cpp, Catch2, Tracy, `stb_image` (loader-only). Header-only deps first; defer Zenoh until Phase 7.
- Add `sccache` to CI.
- Expand `.github/workflows/ci.yml`: SITL build+test, KPU cross-compile, `cargo clippy`, `cargo fmt --check`, `clang-format` check.
- Conventional Commits workflow: commitlint check in CI + `release-please` (or `semantic-release`) for dynamic-updating SemVer tags and `CHANGELOG.md`.
- `bootstrap.ps1` (Windows) and `bootstrap.sh` (Linux) for new-developer setup.
- `.clang-format`, `.editorconfig`, top-level `LICENSE` confirmation (currently MIT — confirm compatible with vendored deps).

### Phase 1 — Rust Resource Manager + cxx bridge  *(blocks SDK linking)* [deps: E0]

**E1 · core/ Rust resource manager (SITL-only for MVP)**
- Define `MemoryProvider` trait (request/release/lock zero-copy buffers).
- SITL implementation: POSIX shm via `shm_open` + `mmap`; on Windows, `CreateFileMapping`. Heap fallback for unit tests.
- SoC implementation: **stub only** — trait shape locked, `unimplemented!()` body.
- `cxx::bridge` module in `lib.rs`: `TensorMetadata` (`#[repr(C)]`), `request_buffer`, `release_buffer`, lifecycle ops.
- `build.rs` for cxx header emission to `branes/core/`.
- Lifecycle state machine: `Unconfigured → Inactive → Active → Teardown`, exposed across the bridge.
- Rust unit tests (provider behavior, lifecycle transitions).
- Round-trip integration test: C++ calls bridge, gets pointer, writes, Rust reads back.

**E1-DEFERRED · SoC broker (parked behind `phase-2-soc` label)**
- IPC broker (Unix domain socket + shm) — host broker, unprivileged client containers.
- Crash-only recovery / client replay protocol.
- **KPU resource allocation arbiter**: live map of which domain flow programs occupy which physical tiles/memory regions, conflict detection, release-on-completion. (Scheduling itself is offline DFA-graph-compiler responsibility, emitting domain flow programs that this arbiter places. Out of scope for `cortex`.)

### Phase 2 — Math layer  *(parallel with Phase 1 after E0)*

**E2 · math/ header-only NLS + linear algebra**
- Tensor view types backed by `std::span<T>`, with stride/shape metadata (DLPack-style lightweight struct).
- Arithmetic-type plumbing: ensure MTL5 expression templates instantiate cleanly against Universal posit/cfloat/fixed-point types. Compile-only smoke tests with several types.
- Sparse storage: CSR, CSC, COO views over caller-owned buffers.
- Krylov solvers (CG, GMRES, BiCGSTAB) — thin wrappers on MTL5 ITL fixed for our type concepts.
- Direct sparse Cholesky / LDLT for SPD normal equations.
- Lie groups (header-only): `SO3<T>`, `SE3<T>`, `Sim3<T>`, exp/log, adjoint, Jacobians. Sophus-style API but type-generic.
- Analytical Jacobian convention (see judgment call #8).
- Non-linear solvers: Gauss-Newton, Levenberg-Marquardt, Dogleg. Trust-region machinery shared.
- Test suite: golden-data NLS problems (Rosenbrock, Powell, Beale), Jacobian finite-difference checks, type-genericity tests.
- Doxygen-friendly headers, concept-constrained templates.

### Phase 3 — VIO operator  *(MVP target)* [deps: E1 bridge, E2 math]

**E3 · sdk/ Visual-Inertial Odometry (MSCKF-style; hybrid: templated operators + library glue)**

*Backend abstraction (must land first so SW-opt skeleton plugs in later):*
- `cortex::VioBackend` interface separating front-end (feature tracking) from estimator backend (MSCKF, future SW-opt). Front-end output is a backend-agnostic observation stream.
- MSCKF backend implementation.
- SW-optimization backend skeleton (interface conformance only, no math).

*In-house CV operators (clean-room, OpenCV-shaped API) — live under `cv/` directory:*
- `branes::cv::Image` container + simple I/O (PGM/PNG via `stb_image`; EuRoC is PNG).
- Image pyramid (Gaussian downsample, 3–5 levels).
- FAST corner detector with non-maximum suppression.
- Sub-pixel KLT pyramidal tracker.

*Camera & geometry (templated, lives in `math/`):*
- Camera models: pinhole + radial-tangential, equidistant (fisheye), unified omnidirectional. Header-only, type-generic. Undistortion / projection / unprojection.

*VIO core (MSCKF):*
- IMU preintegration (closed-form, clean-room implementation; no OpenVINS code contact).
- Feature representation (anchored inverse-depth, XYZ, MSCKF nullspace types).
- IMU initialization: static (gravity from rest) + dynamic (visual-inertial alignment).
- State + Propagator + StateHelper: MSCKF state vector, square-root covariance optional, propagation step.
- Updaters: monocular/stereo cam update (MSCKF null-space projection), interface for future GPS/wheel/lidar.
- Top-level `cortex::VioEstimator<Scalar>` API consuming `std::span` views.

*Validation:*
- EuRoC MAV replay harness + ATE/RPE accuracy benchmark against published-paper numerical thresholds (no GPL-code oracle — see judgment call #3).
- Latency budget enforcement: per-frame upper bound asserted in tests.

### Phase 4 — Visual SLAM  *(post-MVP)* [deps: E3]

**E4 · sdk/ Visual SLAM**
- Keyframe + observation data structures (data-oriented: SoA, contiguous index-addressed arrays).
- Local map representation (no `shared_ptr` graphs — integer-indexed adjacency).
- One descriptor (ORB or BRIEF) — clean-room, under `cv/`.
- Brute-force descriptor matcher with Hamming/L2 distance.
- Sliding-window bundle adjustment (reuse E2 LM solver).
- Loop closure detection: BoW interface (DBoW-style); pluggable for future learned descriptors from KPU.
- Pose graph optimization (NLS over SE(3), sparse).
- Global BA.
- Relocalization API.
- KITTI / TUM-VI replay benchmarks (published-number thresholds).

### Phase 5 — 3D Scene Graph  *(post-MVP scaffold)* [deps: E4 map types]

**E5 · sdk/ 3D Scene Graph**
- Data-oriented node store: `NodeId` = `uint32_t`, parallel arrays per attribute, edge list as `(src, dst, type)` triples.
- Hierarchical schema (building → floor → room → object → part) — types via enum + per-type attribute table.
- Probabilistic attributes: class distribution, pose covariance (SE(3) tangent-space).
- Bayesian update operator (scaffold; defer measurement-model math — see judgment call #6).
- Geometry+semantics fusion API: takes pose stream from VIO/SLAM + semantic tensor from inference, returns updated graph.
- Lightweight on-wire serialization (for Zenoh egress) — flat-buffer-style or hand-rolled `repr(C)` payload.

### Phase 6 — Configuration  *(parallel with E3 onward)* [deps: E0]

**E6 · Unified config schema**
- YAML schema versions for: camera intrinsics/extrinsics, IMU noise model, VIO solver tolerances, lifecycle params, memory-pool sizes.
- C++ deserializer → typed POD structs (`VioConfig`, `ImuNoiseModel`, …) via yaml-cpp, parsed *only* in daemons.
- Calibration loader: Kalibr-format + ROS-style camera-info YAML → typed POD camera structs.
- Rust deserializer (serde) for memory-pool sizing in the RM.
- Schema validator + golden good/bad fixtures.
- `config/default_bot.yaml` populated with EuRoC-MAV values as the reference.

### Phase 7 — Daemons (Zenoh middleware)  *(post-VIO-MVP)* [deps: E3, E6]

**E7 · daemons/ Zenoh wrappers**
- FetchContent for zenoh-c / zenoh-cpp.
- Lifecycle daemon base class implementing the managed state machine.
- `vio_daemon`: subscribes sensor topics → drives `cortex::VioEstimator` → publishes pose + (later) scene graph.
- File-based config (`--config path.yaml`) + network-based (`cortex/config/vio` subscriber).
- `slam_daemon`, `scene_graph_daemon` (skeletons).
- Zero-copy egress strategy: publish shm pointers / Zenoh shared memory, not serialized payloads.
- Integration test: replay EuRoC into the daemon, observe pose topic, compare ATE to in-process E3 benchmark — they must match.

### Phase 8 — SITL test harness  *(parallel after E3)*

**E8 · SITL infrastructure**
- Mock ingress node: reads EuRoC/KITTI/TUM-VI, writes frames into shm, publishes Zenoh notifications.
- **Clean-room VIO simulator** (replaces forbidden GPL oracle): B-spline trajectory generator, pinhole projection of synthetic landmarks, IMU integration with configurable bias/noise model. Outputs ground-truth poses + synthetic image+IMU stream.
- End-to-end SITL test: mock ingress → daemon → pose sink → accuracy assertion against published-paper thresholds and against in-house simulator ground truth.
- Latency regression harness with budgets per operator.
- Dataset bootstrap script (download + checksum verify).

### Phase 9 — Instrumentation  *(parallel after E1, E3)*

**E9 · Profiling**
- Tracy zones in C++ SDK hot paths.
- `tracing-tracy` in Rust RM.
- Custom hooks for tracking buffer-pool occupancy.
- Valgrind step in WSL2 + CI for the SITL test target.

### Phase 10 — Deployment  *(deferred, design-only for now)*

**E10 · Deployment artifacts**
- Dockerfile per daemon + multi-stage builds.
- CDI JSON generator for KPU device mapping.
- Yocto recipe placeholder (`meta-branes`).
- A/B partition + pull-based OTA ADR.
- systemd unit files.

### Phase 11 — Docs & examples  *(parallel after E3)*

**E11 · Documentation**
- Doxygen + GitHub Pages site for SDK API.
- Examples: `hello_vio` (white-box SDK consumer), `hello_zenoh` (black-box daemon consumer).
- ADR directory with one-pager per major decision (mirror what's in `docs/arch/cortex-repo.md` but smaller and updateable).
- CONTRIBUTING.md with the layering rules from `CLAUDE.md`.

---

## Suggested labels for tracking
**Phase:** `phase-0-foundation`, `phase-1-rust-core`, `phase-2-math`, `phase-3-vio`, `phase-4-slam`, `phase-5-3dsg`, `phase-6-config`, `phase-7-daemons`, `phase-8-sitl`, `phase-9-profiling`, `phase-10-deployment`, `phase-11-docs`, `phase-soc-deferred` (parked sub-epic for SoC silicon work — name disambiguates from `phase-2-math`).
**Cross-cutting:** `mvp-blocker`, `clean-room`, `no-gpl`, `cv-stack`, `decision-needed`, `epic`.

## Critical path to MVP

`E0 → (E1 ∥ E2) → E3 → E6 → E7 → E8 + E9`

CV operators are folded into E3 (front-end) and E4 (descriptors); no standalone CV epic. Everything else is parallelizable or post-MVP.

## Estimated sub-issue count

~89 sub-issues across 13 epics (12 active + 1 parked SoC).

## Filed issues (as of 2026-05-21)

All 12 active epics + their sub-issues filed in `branes-ai/cortex`:

| Epic | Issue | Sub-issues | Milestone |
|---|---|---|---|
| E0 · Build & toolchain foundation | [#1](https://github.com/branes-ai/cortex/issues/1) | #4–#12 | Phase 0: Foundation |
| E1 · Rust Resource Manager + cxx bridge | [#2](https://github.com/branes-ai/cortex/issues/2) | #13–#20 (MVP), #21–#23 (deferred) | Phase 1: Rust Core, SoC (deferred) |
| E2 · Header-only math layer | [#3](https://github.com/branes-ai/cortex/issues/3) | #24–#32 | Phase 2: Math |
| E3 · Visual-Inertial Odometry | [#33](https://github.com/branes-ai/cortex/issues/33) | #34–#49 | Phase 3: VIO |
| E4 · Visual SLAM | [#50](https://github.com/branes-ai/cortex/issues/50) | #51–#60 | Phase 4: SLAM |
| E5 · 3D Scene Graph | [#61](https://github.com/branes-ai/cortex/issues/61) | #62–#67 | Phase 5: 3DSG |
| E6 · Unified configuration | [#68](https://github.com/branes-ai/cortex/issues/68) | #69–#74 | Phase 6: Config |
| E7 · Zenoh daemons | [#75](https://github.com/branes-ai/cortex/issues/75) | #76–#83 | Phase 7: Daemons |
| E8 · SITL test harness | [#84](https://github.com/branes-ai/cortex/issues/84) | #85–#89 | Phase 8: SITL |
| E9 · Instrumentation | [#90](https://github.com/branes-ai/cortex/issues/90) | #91–#94 | Phase 9: Profiling |
| E10 · Deployment | [#95](https://github.com/branes-ai/cortex/issues/95) | #96–#100 | Phase 10: Deployment |
| E11 · Documentation | [#101](https://github.com/branes-ai/cortex/issues/101) | #102–#106 | Phase 11: Docs |

**Total filed: 12 epics + 94 sub-issues = 106 issues.**

All issues have: assignee=Ravenwater, project=Branes CORTEX, Estimate=5, milestone-by-phase, and (for CC-prefixed titles) GitHub issue type (Feature/Bug/Task).

---

## Editor notes

Add notes here as you review:

- notes have been interspersed with the answers.
