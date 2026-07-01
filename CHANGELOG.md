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

- **2026-06-30 — End-to-end R-IEKF EuRoC harness; measured, and it underperforms
  (main, post-v0.65.0).** Built the harness that drives the R-IEKF backend through the
  validated `VioEstimator` front end on real EuRoC (adapter + `--invariant` path +
  shared `euroc_cam0()` calibration + non-finite-δx update guard + gate test), so it
  is directly comparable with the standard MSCKF. R-IEKF now runs **stably** (Joseph
  `FullCovariance`; the square-root form copied from the synthetic path diverged to
  6.24e+40 m) but **underperforms** the body-frame filter — ATE 1.3 m vs 0.27 m,
  position ATE/σ 17–19 vs 2.1. The load-bearing finding: **position-norm ATE/σ does
  not test the yaw-gauge thesis** (standard MSCKF is already ~calibrated at 2.1 on
  position because the leak is rotational) — a gauge-anchored attitude-NEES is the
  metric that can see the fix (the #365 follow-up). 1 PR (#429). See
  [`docs/sessions/2026-06-30-riekf-euroc-harness.md`](docs/sessions/2026-06-30-riekf-euroc-harness.md).
- **2026-06-13→15 — R-IEKF backend built; FEJ refuted; system verdict localized a
  bug (→ v0.50.3).** Built the Right-Invariant EKF backend end to end — `SE2(3)`
  state, invariant propagation, invariant camera update, `MsckfInvariantBackend` —
  each **unit-proven** to annihilate/preserve the 4-DoF unobservable gauge by
  construction (the #212 yaw-leak fix, no FEJ). The surgical FEJ alternative was
  **implemented, measured, and refuted** (passed the unit metric < 1e-15 but nearly
  doubled end-to-end NEES, 42.6 → 77.7) — only its production-driver gauge check was
  kept. Published the camera-calibration / projection-Jacobian docs (KaTeX +
  interactive 3D triangulation scene). Triaged the Phase-3 milestone (15 → 8 open
  issues). Drove the invariant backend through the validated front end for the
  system-level verdict, which **localized a real continuous-update bug** (#366 — the
  velocity-coupling chain that makes roll/pitch observable is inconsistent). 9 PRs
  (#353, #354, #356, #357, #358, #360, #362, #364, #367). See
  [`docs/sessions/2026-06-13-riekf-backend-fej-refuted-system-verdict.md`](docs/sessions/2026-06-13-riekf-backend-fej-refuted-system-verdict.md).
- **2026-06-05 — VIO Pipeline docs section + 3D visualization (→ v0.40.0).**
  Published the #212 over-confidence analysis as a Starlight section with six
  data-backed stage pages (S0 holds, S1 seed-suspect, S2 cleared, S5 gate-missing,
  S6 algebra-correct, S10 leading), a methodology primer, and the consistency
  analysis — each stage measured by its own probe (S5's wired from scratch). Plus a
  two-tier 3D visualization environment: an interactive Three.js path/pose viewer
  with the position-covariance ellipsoid and an offline mp4 renderer. 10 PRs
  (#300–#316). See [`docs/sessions/2026-06-05-vio-pipeline-docs-and-3d-viz.md`](docs/sessions/2026-06-05-vio-pipeline-docs-and-3d-viz.md).
- **2026-05-24 — Phase 1 MVP complete (v0.1.0 → v0.6.0).** Rust Resource
  Manager + cxx bridge in place. All eight MVP sub-issues #13–#20 closed:
  `MemoryProvider` trait, SITL provider (Linux shm + heap), KPU stub,
  cxx::bridge + build.rs + lifecycle state machine, ≥85% coverage gate,
  C++/Rust integration test. Critical path now unblocks Phase 2 (math)
  and Phase 3 (VIO). See [`docs/sessions/2026-05-24-phase-1-mvp-complete.md`](docs/sessions/2026-05-24-phase-1-mvp-complete.md).
- **2026-05-22 — Phase 0 (foundation) complete.** Build system, CI, release
  flow, dev-environment scripts. See [`docs/arch/phase0-foundation/README.md`](docs/arch/phase0-foundation/README.md).
## [0.66.0](https://github.com/branes-ai/cortex/compare/v0.65.0...v0.66.0) (2026-07-01)


### Features

* **tools:** drive R-IEKF through the real EuRoC pipeline ([#429](https://github.com/branes-ai/cortex/issues/429)) ([3c6f4f5](https://github.com/branes-ai/cortex/commit/3c6f4f554f95e9004047555eaf44ef0b2b8e6da9))

## [0.65.0](https://github.com/branes-ai/cortex/compare/v0.64.0...v0.65.0) (2026-06-22)


### Features

* **tools:** obs_inspect — R-IEKF vs standard yaw-leak comparison ([f05a5e7](https://github.com/branes-ai/cortex/commit/f05a5e762e457ec8d4ad317dc87608ecea155e95))
* **tools:** obs_inspect R-IEKF vs standard yaw-leak comparison ([#426](https://github.com/branes-ai/cortex/issues/426)) ([f05a5e7](https://github.com/branes-ai/cortex/commit/f05a5e762e457ec8d4ad317dc87608ecea155e95))


### Documentation

* **pipeline:** record the yaw-leak localization and R-IEKF validation ([#427](https://github.com/branes-ai/cortex/issues/427)) ([e059079](https://github.com/branes-ai/cortex/commit/e0590794602321a358c065fcdf5d0b80b1f6e880))

## [0.64.0](https://github.com/branes-ai/cortex/compare/v0.63.1...v0.64.0) (2026-06-21)


### Features

* **tools:** real-data observability / null-space-leak diagnostic ([#424](https://github.com/branes-ai/cortex/issues/424)) ([8c54ed5](https://github.com/branes-ai/cortex/commit/8c54ed5f0c656232d2d66538fbe7ae511f2af360))

## [0.63.1](https://github.com/branes-ai/cortex/compare/v0.63.0...v0.63.1) (2026-06-21)


### Bug Fixes

* **tools:** correct s6_inspect PSD check and NIS headline ([#423](https://github.com/branes-ai/cortex/issues/423)) ([b384355](https://github.com/branes-ai/cortex/commit/b3843556b284ce87dee509850ec0f415a5d3efd0))


### Documentation

* **pipeline:** record the completed real-data inspector epic ([#371](https://github.com/branes-ai/cortex/issues/371)) ([#421](https://github.com/branes-ai/cortex/issues/421)) ([40f6788](https://github.com/branes-ai/cortex/commit/40f67883c971995e26aacbe1f4f7ba8c3fc7db4b))

## [0.63.0](https://github.com/branes-ai/cortex/compare/v0.62.0...v0.63.0) (2026-06-19)


### Features

* **tools:** s10 online-calibration inspector — real EuRoC → extrinsic convergence ([#419](https://github.com/branes-ai/cortex/issues/419)) ([23388b9](https://github.com/branes-ai/cortex/commit/23388b9b21d5737812bad041a4b3be23f16c6377)), closes [#384](https://github.com/branes-ai/cortex/issues/384)

## [0.62.0](https://github.com/branes-ai/cortex/compare/v0.61.0...v0.62.0) (2026-06-18)


### Features

* **tools:** s3 augmentation inspector — real EuRoC keyframe → cloned covariance ([#416](https://github.com/branes-ai/cortex/issues/416)) ([e07dc9b](https://github.com/branes-ai/cortex/commit/e07dc9b98cd2514a4d349747afddbd8f3517943f))
* **tools:** s9 marginalization inspector — real EuRoC clone drop → reduced covariance ([#418](https://github.com/branes-ai/cortex/issues/418)) ([9fa7f05](https://github.com/branes-ai/cortex/commit/9fa7f057caf67030476c4ab875bf5282f29ab821))

## [0.61.0](https://github.com/branes-ai/cortex/compare/v0.60.2...v0.61.0) (2026-06-17)


### Features

* **tools:** s2 propagation inspector — real EuRoC IMU window → covariance growth & drift ([#414](https://github.com/branes-ai/cortex/issues/414)) ([f8b1292](https://github.com/branes-ai/cortex/commit/f8b1292430379ef61cc3913154f8cc788ad10b23))


### Documentation

* wip workload analysis ([7d1f437](https://github.com/branes-ai/cortex/commit/7d1f437ff1e4174dac5330dc46957337855ecc6e))

## [0.60.2](https://github.com/branes-ai/cortex/compare/v0.60.1...v0.60.2) (2026-06-17)


### Documentation

* **site:** fix un-gated inline math delimiters and escape sequences ([#412](https://github.com/branes-ai/cortex/issues/412)) ([d0df72a](https://github.com/branes-ai/cortex/commit/d0df72ae86b1e1bf2aa35f9d1f09e168ac76cd18))
* **site:** fix un-gated inline math delimiters and escape sequences on overview, S0, and S1 ([d0df72a](https://github.com/branes-ai/cortex/commit/d0df72ae86b1e1bf2aa35f9d1f09e168ac76cd18))
* **site:** restore authoritative VIO stage-flow architectural SVG in overview ([c095c80](https://github.com/branes-ai/cortex/commit/c095c8043cb46d3d27f58792260c9843cb92f442))

## [0.60.1](https://github.com/branes-ai/cortex/compare/v0.60.0...v0.60.1) (2026-06-17)


### Maintenance

* **main:** release 0.60.0 ([#410](https://github.com/branes-ai/cortex/issues/410)) ([61e62a7](https://github.com/branes-ai/cortex/commit/61e62a7e0ea6cc6f2304d64b262aa6cbc841b293))

## [0.60.0](https://github.com/branes-ai/cortex/compare/v0.59.1...v0.60.0) (2026-06-17)


### Features

* 3D drone path/pose viewer (Phase 0 + Phase 1 prototype) ([#302](https://github.com/branes-ai/cortex/issues/302)) ([b4da4a4](https://github.com/branes-ai/cortex/commit/b4da4a40c4e13b32967ee21c98cd69e2ef6a2f23))
* add the S4 forward-backward and S5 parallax outlier gates ([#323](https://github.com/branes-ai/cortex/issues/323)) ([73bcd81](https://github.com/branes-ai/cortex/commit/73bcd81c404d0f9e396680e190291da4c0e2e2f9))
* **bench:** graphs energy-model integration (cpu/gpu/kpu-t64) ([#204](https://github.com/branes-ai/cortex/issues/204)) ([0fc9249](https://github.com/branes-ai/cortex/commit/0fc92494b231e7a95ba61fe3a6997c9dfbf129c9))
* **bench:** on-device energy backends — tegrastats + external meter ([#207](https://github.com/branes-ai/cortex/issues/207)) ([e257a25](https://github.com/branes-ai/cortex/commit/e257a25078e5463d72c7d0f68e2e4ae64d420355))
* **bench:** vio_bench — euroc accuracy + latency + rapl energy report ([#201](https://github.com/branes-ai/cortex/issues/201)) ([ba13dec](https://github.com/branes-ai/cortex/commit/ba13decf994ac8997a479c2fc24feb028504d117))
* **bench:** vio_bench progress output + usage docs ([#208](https://github.com/branes-ai/cortex/issues/208)) ([077751a](https://github.com/branes-ai/cortex/commit/077751ad35185a7d38b303fa40e54fcb8b0c5c2b))
* **ci:** add a Windows MSVC build + test target ([#216](https://github.com/branes-ai/cortex/issues/216)) ([5f439b4](https://github.com/branes-ai/cortex/commit/5f439b4c3adc700a0dcdef12051d94a84db09cd7))
* complete the structural stage pages (S3, S4, S7-S9) ([#319](https://github.com/branes-ai/cortex/issues/319)) ([d3ec53b](https://github.com/branes-ai/cortex/commit/d3ec53b437f7bf70864e03b3618ceeda2c0c1c16))
* **core:** add cxx bridge, build.rs, and lifecycle state machine ([#133](https://github.com/branes-ai/cortex/issues/133)) ([3ee12fb](https://github.com/branes-ai/cortex/commit/3ee12fb13fcde29844c70b35d226a7d9e2a59755))
* **core:** add KPU MemoryProvider stub ([#131](https://github.com/branes-ai/cortex/issues/131)) ([a2445c8](https://github.com/branes-ai/cortex/commit/a2445c859adad8c5d753e6249001df6e4c0d183a))
* **core:** add SITL MemoryProvider with shm + heap backends ([3d99738](https://github.com/branes-ai/cortex/commit/3d99738832746b5b32f6ccf0c79cce118b882f39))
* **core:** define MemoryProvider trait + BufferHandle + MemErr ([#127](https://github.com/branes-ai/cortex/issues/127)) ([5c1d67a](https://github.com/branes-ai/cortex/commit/5c1d67a7658dd9d9ef2814bcc1e7d277bdd31eb5))
* **core:** SITL MemoryProvider — Linux shm + heap backends ([#129](https://github.com/branes-ai/cortex/issues/129)) ([3d99738](https://github.com/branes-ai/cortex/commit/3d99738832746b5b32f6ccf0c79cce118b882f39))
* **cv:** FAST-9 corner detector with non-maximum suppression ([#174](https://github.com/branes-ai/cortex/issues/174)) ([53c8850](https://github.com/branes-ai/cortex/commit/53c885060e355f35c39fd665e4e7009629996f21))
* **cv:** gaussian image pyramid with configurable scale ([#170](https://github.com/branes-ai/cortex/issues/170)) ([afefa22](https://github.com/branes-ai/cortex/commit/afefa225c70371bca72af8cb75575b50f17cf587))
* **cv:** image container + grayscale PGM/PNG I/O ([#169](https://github.com/branes-ai/cortex/issues/169)) ([087a432](https://github.com/branes-ai/cortex/commit/087a432d03455d53cdf5e87b170f320e43d4ef86))
* **cv:** sub-pixel pyramidal KLT feature tracker ([#176](https://github.com/branes-ai/cortex/issues/176)) ([9a54aec](https://github.com/branes-ai/cortex/commit/9a54aec24bb0d0043a1d744a14d5135fb76d1105))
* **eval:** add IMU Allan-variance noise characterization ([#268](https://github.com/branes-ai/cortex/issues/268)) ([#276](https://github.com/branes-ai/cortex/issues/276)) ([4eafd31](https://github.com/branes-ai/cortex/commit/4eafd316ba2d69fde7bf392af872b4cd443d0a06))
* **eval:** add NEES-against-ground-truth consistency (closes [#264](https://github.com/branes-ai/cortex/issues/264)) ([#273](https://github.com/branes-ai/cortex/issues/273)) ([a93cc17](https://github.com/branes-ai/cortex/commit/a93cc178edf41630179abad8c2c8d7169dfa2359))
* **eval:** add NEES/NIS filter-consistency statistics ([9dc9a1b](https://github.com/branes-ai/cortex/commit/9dc9a1bd5ce278bd2a998951129fe3507466ec1d))
* **eval:** NEES/NIS filter-consistency statistics ([#264](https://github.com/branes-ai/cortex/issues/264)) ([#271](https://github.com/branes-ai/cortex/issues/271)) ([9dc9a1b](https://github.com/branes-ai/cortex/commit/9dc9a1bd5ce278bd2a998951129fe3507466ec1d))
* landmark cloud + camera frustum in the 3D viewer ([#303](https://github.com/branes-ai/cortex/issues/303)) ([7f4f532](https://github.com/branes-ai/cortex/commit/7f4f5320f76ee891162d5ca0b53b0ad9835a0410))
* **math:** add SE2(3) extended-pose Lie group — the R-IEKF state manifold ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#353](https://github.com/branes-ai/cortex/issues/353)) ([1a7da91](https://github.com/branes-ai/cortex/commit/1a7da91b593e7065d0fdb09aae90562eccbf2870))
* **math:** arithmetic-type plumbing for MTL5 over Universal types ([#143](https://github.com/branes-ai/cortex/issues/143)) ([4163b22](https://github.com/branes-ai/cortex/commit/4163b2261da08fc1058c6678df067077d0e370b8)), closes [#25](https://github.com/branes-ai/cortex/issues/25)
* **math:** camera models — pinhole+radtan, fisheye, unified omni ([#172](https://github.com/branes-ai/cortex/issues/172)) ([8ce771b](https://github.com/branes-ai/cortex/commit/8ce771bd1eb377ed07c972409e8a035eb8e31d26))
* **math:** direct sparse Cholesky/LDLT for SPD normal equations ([#151](https://github.com/branes-ai/cortex/issues/151)) ([818ef32](https://github.com/branes-ai/cortex/commit/818ef32433ffee70b896ae3a01d34b702713a950)), closes [#28](https://github.com/branes-ai/cortex/issues/28)
* **math:** header-only Lie groups SO3, SE3, Sim3 ([#147](https://github.com/branes-ai/cortex/issues/147)) ([3dd410f](https://github.com/branes-ai/cortex/commit/3dd410fa8170f4668452d6206d7a281073b49d38)), closes [#29](https://github.com/branes-ai/cortex/issues/29)
* **math:** Krylov solvers (CG, GMRES, BiCGSTAB) over MTL5 ITL ([#149](https://github.com/branes-ai/cortex/issues/149)) ([f27b47c](https://github.com/branes-ai/cortex/commit/f27b47c959f0e126c879a2431873fc09cc5d95d1))
* **math:** non-linear least squares solvers (GN, LM, Dogleg) ([d85444d](https://github.com/branes-ai/cortex/commit/d85444d169255098258622cc1f621dcf59a503dd)), closes [#30](https://github.com/branes-ai/cortex/issues/30)
* **math:** non-linear least squares solvers (gn, lm, dogleg) ([#153](https://github.com/branes-ai/cortex/issues/153)) ([d85444d](https://github.com/branes-ai/cortex/commit/d85444d169255098258622cc1f621dcf59a503dd))
* **math:** sparse storage views (CSR, CSC, COO) over caller buffers ([#145](https://github.com/branes-ai/cortex/issues/145)) ([d556f14](https://github.com/branes-ai/cortex/commit/d556f144f66dc697e831dc443a7f00719b901216)), closes [#26](https://github.com/branes-ai/cortex/issues/26)
* **math:** tensor views over std::span with stride/shape metadata ([bb868d5](https://github.com/branes-ai/cortex/commit/bb868d5bda1ec3fc683fa5a1957701a41cdda041)), closes [#24](https://github.com/branes-ai/cortex/issues/24)
* **math:** TensorView&lt;T,Rank&gt; over std::span with stride/shape metadata ([#141](https://github.com/branes-ai/cortex/issues/141)) ([bb868d5](https://github.com/branes-ai/cortex/commit/bb868d5bda1ec3fc683fa5a1957701a41cdda041))
* **math:** thin Krylov solver wrappers (cg, gmres, bicgstab) over MTL5 ITL ([f27b47c](https://github.com/branes-ai/cortex/commit/f27b47c959f0e126c879a2431873fc09cc5d95d1)), closes [#27](https://github.com/branes-ai/cortex/issues/27)
* S5 feature-triangulation probe + stage page (parallax gate, quantified) ([#313](https://github.com/branes-ai/cortex/issues/313)) ([8fbf1cd](https://github.com/branes-ai/cortex/commit/8fbf1cddfe0863b4cbe1ca806abd513c73c5f591))
* **sdk:** add 6-DoF rigid camera extrinsics to MsckfInvariantBackend ([105852f](https://github.com/branes-ai/cortex/commit/105852f7f4d1801b4d45f6df20258a0b619ad1a2))
* **sdk:** add 6-DoF rigid camera extrinsics to MsckfInvariantBackend ([#396](https://github.com/branes-ai/cortex/issues/396)) ([8ed59cc](https://github.com/branes-ai/cortex/commit/8ed59cc4e50313bf1a2414392ed4f78f0e14fac5))
* **sdk:** add S10 calibration-uncertainty R-term; measure it end-to-end on V2_03 ([#328](https://github.com/branes-ai/cortex/issues/328)) ([bce345f](https://github.com/branes-ai/cortex/commit/bce345ff897d35134b8fb02479551ea682d1ea7a))
* **sdk:** add the R-IEKF invariant camera measurement update ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#356](https://github.com/branes-ai/cortex/issues/356)) ([c136991](https://github.com/branes-ai/cortex/commit/c136991766f7d97e655987a0988bbffac78d5a95))
* **sdk:** add the R-IEKF invariant IMU propagation on SE2(3) ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#354](https://github.com/branes-ai/cortex/issues/354)) ([98683ab](https://github.com/branes-ai/cortex/commit/98683ab7a87248d1ff3c9d3460e878cfaef474e5))
* **sdk:** camera updaters with MSCKF null-space projection ([#185](https://github.com/branes-ai/cortex/issues/185)) ([191c4e7](https://github.com/branes-ai/cortex/commit/191c4e71448a5374716ede832374c0eeec33e934))
* **sdk:** feature representations + MSCKF null-space projection ([#180](https://github.com/branes-ai/cortex/issues/180)) ([58bc198](https://github.com/branes-ai/cortex/commit/58bc198fd5be6a69569735a060a20422a2fc1528))
* **sdk:** force-dynamic VI-init validation on MH_05 (real-data [#211](https://github.com/branes-ai/cortex/issues/211)) ([#246](https://github.com/branes-ai/cortex/issues/246)) ([f80fa63](https://github.com/branes-ai/cortex/commit/f80fa638b3846cf0210c716b5f289bb3b7dd1adb))
* **sdk:** implement Phase C 6-DoF online extrinsic calibration for MsckfInvariantBackend ([f9007da](https://github.com/branes-ai/cortex/commit/f9007dad737fc18815c3f3208d754df45f319625))
* **sdk:** IMU initialization (static + dynamic) ([#184](https://github.com/branes-ai/cortex/issues/184)) ([b638566](https://github.com/branes-ai/cortex/commit/b638566edd6d023a0c508d4f1bd5a1713bbc2588))
* **sdk:** imu preintegration (closed-form, clean-room) ([#178](https://github.com/branes-ai/cortex/issues/178)) ([355661b](https://github.com/branes-ai/cortex/commit/355661b64d4d6d2ad3080b60fde8cb11bfaad3c9))
* **sdk:** MSCKF backend implementation ([#186](https://github.com/branes-ai/cortex/issues/186)) ([02f3948](https://github.com/branes-ai/cortex/commit/02f3948300ad3365497ea329d3e8ef1d9fc4ab97))
* **sdk:** observability null-space probe — localizes the [#212](https://github.com/branes-ai/cortex/issues/212) NEES leak to yaw ([#337](https://github.com/branes-ai/cortex/issues/337)) ([#338](https://github.com/branes-ai/cortex/issues/338)) ([2720ec7](https://github.com/branes-ai/cortex/commit/2720ec7fae90a6a417488eee8bb9a5b524b1b5c1))
* **sdk:** online extrinsic calibration (S10-as-state), mechanism + synthetic proof ([#332](https://github.com/branes-ai/cortex/issues/332)) ([#333](https://github.com/branes-ai/cortex/issues/333)) ([e5fc23f](https://github.com/branes-ai/cortex/commit/e5fc23f235bd31cda7f82e5df8822b47fd5bf335))
* **sdk:** perspective-n-point resectioning via DLT + RANSAC ([#236](https://github.com/branes-ai/cortex/issues/236)) ([fb3370a](https://github.com/branes-ai/cortex/commit/fb3370aee8434c061c2d66268d83e615b94d5d36))
* **sdk:** Phase C - 6-DoF online extrinsic calibration for MsckfInvariantBackend ([#400](https://github.com/branes-ai/cortex/issues/400)) ([f9007da](https://github.com/branes-ai/cortex/commit/f9007dad737fc18815c3f3208d754df45f319625))
* **sdk:** prove the right-invariant propagation Φ is state-independent (R-IEKF Phase A, [#348](https://github.com/branes-ai/cortex/issues/348)) ([#351](https://github.com/branes-ai/cortex/issues/351)) ([a0c35aa](https://github.com/branes-ai/cortex/commit/a0c35aafb60be143978b39c88675fa009dc1d116))
* **sdk:** Q/R consistency-sweep knobs + [#212](https://github.com/branes-ai/cortex/issues/212) root-cause diagnosis ([#279](https://github.com/branes-ai/cortex/issues/279)) ([b718e5e](https://github.com/branes-ai/cortex/commit/b718e5e92cc177f06ae1de391824777388d12788))
* **sdk:** R-IEKF Phase-A gate — invariant parameterization flattens the yaw leak ([#348](https://github.com/branes-ai/cortex/issues/348)) ([#349](https://github.com/branes-ai/cortex/issues/349)) ([427134e](https://github.com/branes-ai/cortex/commit/427134ea0516f31292b101428759cadcc7d2fdd3))
* **sdk:** R-IEKF VIO backend adapter + continuous-update divergence localization ([#365](https://github.com/branes-ai/cortex/issues/365)) ([#367](https://github.com/branes-ai/cortex/issues/367)) ([f7c2671](https://github.com/branes-ai/cortex/commit/f7c26718b36bce00066048ed658db6db781cb67b))
* **sdk:** resolve metric scale in ImuInitializer::try_dynamic ([#233](https://github.com/branes-ai/cortex/issues/233)) ([0df58a0](https://github.com/branes-ai/cortex/commit/0df58a0680fb0db2c92a81b048dfd1b437ee1860))
* **sdk:** S6 update probe; lock S3/S5/S6/S9 contracts by measurement ([#325](https://github.com/branes-ai/cortex/issues/325)) ([df6cafe](https://github.com/branes-ai/cortex/commit/df6cafe53035a9c7861852163a7afeafa7315aa5))
* **sdk:** sliding-window optimization backend skeleton ([#189](https://github.com/branes-ai/cortex/issues/189)) ([456a373](https://github.com/branes-ai/cortex/commit/456a373d1e8d803f641a3fbfbecae96e1eb4ac98))
* **sdk:** square-root covariance MSCKF backend ([#218](https://github.com/branes-ai/cortex/issues/218)) ([fe8014b](https://github.com/branes-ai/cortex/commit/fe8014b70754680de9b912e74138cfed8f11af42))
* **sdk:** state vector + propagator + state-helper (MSCKF core) ([#182](https://github.com/branes-ai/cortex/issues/182)) ([0e78154](https://github.com/branes-ai/cortex/commit/0e78154e8d284d1649a35eedf69fe34af5b92798))
* **sdk:** surface per-update NIS for filter-consistency monitoring ([149c127](https://github.com/branes-ai/cortex/commit/149c127c1d4254ea17d64abf684847466ddffde1))
* **sdk:** surface per-update NIS for filter-consistency monitoring ([#264](https://github.com/branes-ai/cortex/issues/264)) ([#272](https://github.com/branes-ai/cortex/issues/272)) ([149c127](https://github.com/branes-ai/cortex/commit/149c127c1d4254ea17d64abf684847466ddffde1))
* **sdk:** top-level vio estimator API ([#188](https://github.com/branes-ai/cortex/issues/188)) ([c068bd9](https://github.com/branes-ai/cortex/commit/c068bd9b76be2ae356d79413266dc907286038ed))
* **sdk:** two-view SfM bootstrap (essential matrix + RANSAC) ([#231](https://github.com/branes-ai/cortex/issues/231)) ([1900a16](https://github.com/branes-ai/cortex/commit/1900a168f2d0c08ff1054bee6fb14f43a314f1c7))
* **sdk:** vio backend interface separating front-end from estimator ([377dfaa](https://github.com/branes-ai/cortex/commit/377dfaace0d0a9709b1836e763c9d41c58048db7)), closes [#34](https://github.com/branes-ai/cortex/issues/34)
* **sdk:** VioBackend interface separating front-end from estimator ([#167](https://github.com/branes-ai/cortex/issues/167)) ([377dfaa](https://github.com/branes-ai/cortex/commit/377dfaace0d0a9709b1836e763c9d41c58048db7))
* **sdk:** wire dynamic VI init into MsckfBackend (InitMethod::Dynamic) ([#238](https://github.com/branes-ai/cortex/issues/238)) ([108cf02](https://github.com/branes-ai/cortex/commit/108cf02bf8f1148e51505a96e9c52f45c9c64416))
* **sdk:** wire the MsckfInvariantBackend (R-IEKF filter) ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#357](https://github.com/branes-ai/cortex/issues/357)) ([a52f6f8](https://github.com/branes-ai/cortex/commit/a52f6f838c15aa76c8095d53d836d383303e651c))
* **test:** EuRoC verdict — online extrinsic calibration cleared as [#212](https://github.com/branes-ai/cortex/issues/212) driver ([#332](https://github.com/branes-ai/cortex/issues/332)) ([#343](https://github.com/branes-ai/cortex/issues/343)) ([98488f6](https://github.com/branes-ai/cortex/commit/98488f6a0e5b8aa2a204accf52c0e380320277eb))
* **tools:** --trace tap for EuRoC asl_replay — real per-stage trace dumps ([#387](https://github.com/branes-ai/cortex/issues/387)) ([b9cc112](https://github.com/branes-ai/cortex/commit/b9cc112605d9029b15cbf55fce07cd3a75d768a5))
* **tools:** add EuRoC source + scene-video metric overlay to vio_pipeline ([#292](https://github.com/branes-ai/cortex/issues/292)) ([f4b8e9e](https://github.com/branes-ai/cortex/commit/f4b8e9e95e3f9a42786e3177b76c295bf7be8061))
* **tools:** add VIO stage-probe utilities — S0 sensor-model + S2 propagation ([#283](https://github.com/branes-ai/cortex/issues/283)) ([785e875](https://github.com/branes-ai/cortex/commit/785e87575f2a34121db17ad60ce2fd5b97cd77fa))
* **tools:** depth-colored scene overlay + vio_pipeline how-to guide ([#300](https://github.com/branes-ai/cortex/issues/300)) ([96df842](https://github.com/branes-ai/cortex/commit/96df8421dde43a19806bd8bdbba5d89140afcba6))
* **tools:** end-to-end VIO noise-&gt;robustness demo (synthetic source) ([#290](https://github.com/branes-ai/cortex/issues/290)) ([047defa](https://github.com/branes-ai/cortex/commit/047defaca0fe8cf60eb791053647c1803009ccf2))
* **tools:** inter-stage VIO trace bus — typed, human-readable per-stage I/O records ([#385](https://github.com/branes-ai/cortex/issues/385)) ([e5b8caf](https://github.com/branes-ai/cortex/commit/e5b8caf8be515ae572491822ff187dd777bf65d6))
* **tools:** s0 sensor-model inspector — real EuRoC frame/IMU → camera & noise contract ([#394](https://github.com/branes-ai/cortex/issues/394)) ([f1599e6](https://github.com/branes-ai/cortex/commit/f1599e6ab25ee2ef3ac4d62430bdcd5ddaee3046))
* **tools:** s1 initialization inspector — real EuRoC bootstrap → initial state & covariance ([#407](https://github.com/branes-ai/cortex/issues/407)) ([60872dd](https://github.com/branes-ai/cortex/commit/60872ddaf20fdf8e2daf713251fb92bb45818b0c))
* **tools:** S4 frontend inspector — real EuRoC FAST+KLT tracks, visualized ([#390](https://github.com/branes-ai/cortex/issues/390)) ([4ecb210](https://github.com/branes-ai/cortex/commit/4ecb210a8eef88c531fec472b7dd995704a70ce2))
* **tools:** s5 triangulation inspector — real EuRoC tracks+poses → 3D landmarks ([#398](https://github.com/branes-ai/cortex/issues/398)) ([7a5ffb0](https://github.com/branes-ai/cortex/commit/7a5ffb0cb65a2771db4d1a19019852e9b7738b7a))
* **tools:** s6 msckf-update inspector — real EuRoC updates → state/covariance ([#403](https://github.com/branes-ai/cortex/issues/403)) ([a707fbd](https://github.com/branes-ai/cortex/commit/a707fbdfbbd06ce2164672eaae9241ecffda8e90))
* **tools:** wire S1 (initialization) stage-probe ([#286](https://github.com/branes-ai/cortex/issues/286)) ([22c8b71](https://github.com/branes-ai/cortex/commit/22c8b713f655180a2f2871a98e83599a635b5cb7))
* **tools:** wire S10 (calibration) stage-probe to quantify the [#212](https://github.com/branes-ai/cortex/issues/212) Rx4 ([#295](https://github.com/branes-ai/cortex/issues/295)) ([9a01499](https://github.com/branes-ai/cortex/commit/9a01499186229199eeb6ccd7a95df6ed14e1a7a1))
* wire the S4 visual-frontend probe + data-backed page ([#321](https://github.com/branes-ai/cortex/issues/321)) ([67ac45c](https://github.com/branes-ai/cortex/commit/67ac45c2282bdf23202339750c127d477e69e801))


### Bug Fixes

* **build:** link the Rust core in MSVC Debug by pinning the release CRT ([#226](https://github.com/branes-ai/cortex/issues/226)) ([59075af](https://github.com/branes-ai/cortex/commit/59075af8ba67f7d8c5bef2d18acd6148e03c4a28))
* **docs:** make topology + VIO stage-flow SVGs readable in dark theme ([#334](https://github.com/branes-ai/cortex/issues/334)) ([c6d494c](https://github.com/branes-ai/cortex/commit/c6d494cecdd8e3a2f9b883b4dc50b8b84e6b5b85))
* **docs:** point the benchmark generator at the moved euroc test ([#223](https://github.com/branes-ai/cortex/issues/223)) ([41bd61b](https://github.com/branes-ai/cortex/commit/41bd61beb5c23dc0f1867bf63e1792ad9fb3ade0))
* **docs:** re-anchor EuRoC gate extraction (unbreak Deploy Documentation) ([#256](https://github.com/branes-ai/cortex/issues/256)) ([7021cd6](https://github.com/branes-ai/cortex/commit/7021cd64ac5899e541356fb34f9d56521d345ff0))
* **docs:** re-anchor the EuRoC gate extraction to the run_euroc_replay call ([7021cd6](https://github.com/branes-ai/cortex/commit/7021cd64ac5899e541356fb34f9d56521d345ff0))
* **math:** correctness fixes from post-merge review of Phase 2 ([#159](https://github.com/branes-ai/cortex/issues/159)) ([fbb35e0](https://github.com/branes-ai/cortex/commit/fbb35e00c336ff9834e6138fcbd9eab1a98285f1))
* **math:** guard degenerate inputs in the Lie-group helpers ([#214](https://github.com/branes-ai/cortex/issues/214)) ([d8034bf](https://github.com/branes-ai/cortex/commit/d8034bf0226bab71e9c2ccd2ae7e2fb314b4969e)), closes [#161](https://github.com/branes-ai/cortex/issues/161)
* **sdk:** bootstrap dynamic-init SfM on the widest baseline ([445f1bb](https://github.com/branes-ai/cortex/commit/445f1bbe79a945d3c71d85bc4c4e654528f52da3))
* **sdk:** bootstrap dynamic-init SfM on the widest baseline ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#251](https://github.com/branes-ai/cortex/issues/251)) ([445f1bb](https://github.com/branes-ai/cortex/commit/445f1bbe79a945d3c71d85bc4c4e654528f52da3))
* **sdk:** correct dynamic-init gravity sign — real-data divergence → bounded ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#252](https://github.com/branes-ai/cortex/issues/252)) ([84cc8ba](https://github.com/branes-ai/cortex/commit/84cc8ba458d3dcccd5e96a367559e1676375e53d))
* **sdk:** correct dynamic-init gravity sign + widest-parallax bootstrap ([84cc8ba](https://github.com/branes-ai/cortex/commit/84cc8ba458d3dcccd5e96a367559e1676375e53d))
* **sdk:** resolve R-IEKF continuous divergence ([#366](https://github.com/branes-ai/cortex/issues/366)) ([4f4ee1d](https://github.com/branes-ai/cortex/commit/4f4ee1d4707647152be84dfbc511dff1fc77afaf))
* **sdk:** resolve R-IEKF continuous divergence ([#366](https://github.com/branes-ai/cortex/issues/366)) ([#388](https://github.com/branes-ai/cortex/issues/388)) ([4f4ee1d](https://github.com/branes-ai/cortex/commit/4f4ee1d4707647152be84dfbc511dff1fc77afaf))
* **sdk:** set EuRoC extrinsics + gravity-align init; surface VIO divergence ([#209](https://github.com/branes-ai/cortex/issues/209)) ([f14a450](https://github.com/branes-ai/cortex/commit/f14a45020934eecffb194dc8cef95871815ab2c7))
* **test:** make the vio_latency budget informational on CI ([#240](https://github.com/branes-ai/cortex/issues/240)) ([#259](https://github.com/branes-ai/cortex/issues/259)) ([9edb5ed](https://github.com/branes-ai/cortex/commit/9edb5edf179afb8c294a1c981ffe9484a54c7a79))
* **test:** split euroc-moving-start local decls (set -u unbound var) ([#243](https://github.com/branes-ai/cortex/issues/243)) ([4737ebf](https://github.com/branes-ai/cortex/commit/4737ebff64a471bbcc6613002d4cb94f2435735e))
* **tools:** create the --out directory so artifacts are not silently dropped ([#298](https://github.com/branes-ai/cortex/issues/298)) ([029727d](https://github.com/branes-ai/cortex/commit/029727d13b99d4a29c2695aef579378e04913144))


### Reverts

* **docs:** restore original working vio_run.jsonl ([397a2d2](https://github.com/branes-ai/cortex/commit/397a2d2c3df4645987b2181728e6da96321a9f15))


### Code Refactoring

* **test:** drop redundant layer/_test segments from test filenames ([#225](https://github.com/branes-ai/cortex/issues/225)) ([2ff7cd3](https://github.com/branes-ai/cortex/commit/2ff7cd356d9e0752e1e0dd71979e52a6bcccfcec))


### Build System

* **cmake:** add CMakePresets.json with sitl/kpu-cross/wsl2 presets ([89440ff](https://github.com/branes-ai/cortex/commit/89440ffadd94e479e6debb71e985a628f18d230c))
* **cmake:** add CMakePresets.json with sitl/kpu-cross/wsl2 presets ([8d37aa8](https://github.com/branes-ai/cortex/commit/8d37aa850b191298e000dbaac5216b1cded612b6))
* **cmake:** add cross-toolchain.cmake for KPU aarch64 cross-compile ([d6b1471](https://github.com/branes-ai/cortex/commit/d6b147147d34ce2bbfc620478901a5ed3e635a3c))
* **cmake:** add cross-toolchain.cmake for KPU aarch64 cross-compile ([89e0e1c](https://github.com/branes-ai/cortex/commit/89e0e1c35fd1f53ee9c4c8ce77acc767c6d80fd1))
* **cmake:** wire FetchContent for foundational dependencies ([8eb02f8](https://github.com/branes-ai/cortex/commit/8eb02f885effd3a1474016d9fd001701d1792839))
* **cmake:** wire FetchContent for foundational dependencies ([0e6dc7c](https://github.com/branes-ai/cortex/commit/0e6dc7c6db10e6837f5e63455163a84250fbbf63))
* **test:** generate test targets from a compile_all glob macro ([#220](https://github.com/branes-ai/cortex/issues/220)) ([33c6126](https://github.com/branes-ai/cortex/commit/33c6126921d96e24f41b6dd35c775d0f9915b4b3))


### Continuous Integration

* add lint job (clippy + cargo fmt + clang-format) ([2622eb7](https://github.com/branes-ai/cortex/commit/2622eb79b6e0df2c29ebb8622517d59b4405b07f))
* add lint job (clippy + cargo fmt + clang-format) ([75d3c91](https://github.com/branes-ai/cortex/commit/75d3c91f96968fb4c74b80d0256f304b851f1717))
* gate Rust coverage at &gt;= 85% line coverage ([#135](https://github.com/branes-ai/cortex/issues/135)) ([739c12c](https://github.com/branes-ai/cortex/commit/739c12cc3ab85c05df9622c7d7130131eac084b8))
* install sccache in lint job too (RUSTC_WRAPPER consistency) ([dd341a0](https://github.com/branes-ai/cortex/commit/dd341a086faecd364fc915410ecf99b1a15020e0))
* integrate sccache for Rust + C++ object caching ([69851c6](https://github.com/branes-ai/cortex/commit/69851c61008f20dda729f6d9833e0aabfbe6f67c))
* integrate sccache for Rust + C++ object caching ([0865db2](https://github.com/branes-ai/cortex/commit/0865db201b35be3403a788fd33b176afecf783ef))
* **windows:** build the MSVC job via Ninja + sccache ([#222](https://github.com/branes-ai/cortex/issues/222)) ([2d59380](https://github.com/branes-ai/cortex/commit/2d593809cf22a2e3d41ead85d598afb6a03172dd))


### Documentation

* add CLAUDE.md and bootstrap plan ([c2dc4f1](https://github.com/branes-ai/cortex/commit/c2dc4f1c10aa2b4992b9d0b3369a99e2630cb631))
* add CLAUDE.md and bootstrap plan; gitignore .claude/ ([4509cca](https://github.com/branes-ai/cortex/commit/4509ccabfb9f7b6d8623f274e10088202961877d))
* add Phase 0 Foundation retrospective ([589a74e](https://github.com/branes-ai/cortex/commit/589a74e52350749ede14238367f069d7da54d3e8))
* add Phase 0 Foundation retrospective ([6e2bd0e](https://github.com/branes-ai/cortex/commit/6e2bd0e4c06797153ab15a9aaf65e9314666b0a5))
* add Phase 1 design — Rust RM + cxx bridge ([#125](https://github.com/branes-ai/cortex/issues/125)) ([bec0037](https://github.com/branes-ai/cortex/commit/bec0037ffeeda49bfaa2a7a632343c8991ae81ea))
* add PR template with the per-PR docs-update rule ([#196](https://github.com/branes-ai/cortex/issues/196)) ([d2a669c](https://github.com/branes-ai/cortex/commit/d2a669c5be03787654f16a8fb027e9b5047e924d))
* add Starlight + Doxygen documentation site for Phases 0-3 ([#192](https://github.com/branes-ai/cortex/issues/192)) ([9ccfac2](https://github.com/branes-ai/cortex/commit/9ccfac24dceb1385d1fa9ce8829db7d9f0224d57))
* add VIO S0–S10 stage-flow SVG and replace the pipeline ASCII dataflow ([2b93b51](https://github.com/branes-ai/cortex/commit/2b93b512fd3405d3fcb69a6c8b4322f60869ee78))
* add VIO S0–S10 stage-flow SVG; replace pipeline ASCII dataflow ([#330](https://github.com/branes-ai/cortex/issues/330)) ([2b93b51](https://github.com/branes-ai/cortex/commit/2b93b512fd3405d3fcb69a6c8b4322f60869ee78))
* **assessment:** a plain-language tutorial on NEES/NIS filter consistency ([98c1f58](https://github.com/branes-ai/cortex/commit/98c1f586b71d23bda384bf7ed30a4f4f6b216921))
* **assessment:** plain-language NEES/NIS filter-consistency tutorial ([#274](https://github.com/branes-ai/cortex/issues/274)) ([98c1f58](https://github.com/branes-ai/cortex/commit/98c1f586b71d23bda384bf7ed30a4f4f6b216921))
* **assessment:** VIO diagnostic methodology + instrumentation build list ([#269](https://github.com/branes-ai/cortex/issues/269)) ([b2bf23e](https://github.com/branes-ai/cortex/commit/b2bf23ea0cf4923750f61bda98fc39fbae97f1b3))
* fix doubled base path on the API reference sidebar link ([#194](https://github.com/branes-ai/cortex/issues/194)) ([857354a](https://github.com/branes-ai/cortex/commit/857354af54b5a98cecf10298c7b8a56dc174e692))
* improve overview phrasing and trigger documentation rebuild ([56f6811](https://github.com/branes-ai/cortex/commit/56f6811b01eb34a97b2d29639351f19a6613df6c))
* **math:** Doxygen API docs + concepts reference page ([#157](https://github.com/branes-ai/cortex/issues/157)) ([f54a245](https://github.com/branes-ai/cortex/commit/f54a245ec34a81b7dd589b8a9edc6c225849bea3))
* **math:** doxygen API docs and a concepts reference page ([f54a245](https://github.com/branes-ai/cortex/commit/f54a245ec34a81b7dd589b8a9edc6c225849bea3)), closes [#32](https://github.com/branes-ai/cortex/issues/32)
* migrate diagnostic-methodology as the section's method primer ([#316](https://github.com/branes-ai/cortex/issues/316)) ([fd4b7bb](https://github.com/branes-ai/cortex/commit/fd4b7bb288a32eed1278f27f0f186033ad668e63))
* pin benchmark gate numbers to their source ([#197](https://github.com/branes-ai/cortex/issues/197)) ([01d43da](https://github.com/branes-ai/cortex/commit/01d43da4e851ca4f9bf2a33b474ddae9d4666ac5))
* process documentation ([441810a](https://github.com/branes-ai/cortex/commit/441810a715bb1e2eb4e7a80d106affeece4919d2))
* record the [#212](https://github.com/branes-ai/cortex/issues/212) observability investigation — yaw leak localized, FEJ refuted, R-IEKF next ([#345](https://github.com/branes-ai/cortex/issues/345)) ([714bc9f](https://github.com/branes-ai/cortex/commit/714bc9f5d1485741e1bfc403b51a72ce165ca971))
* render architecture topology as SVG; wire in orphaned S0 fisheye figures ([#327](https://github.com/branes-ai/cortex/issues/327)) ([af801c6](https://github.com/branes-ai/cortex/commit/af801c6dfd69fae88e5299f807a416ea80a14f41))
* render LaTeX math on the site (KaTeX) + camera-calibration walkthrough ([#362](https://github.com/branes-ai/cortex/issues/362)) ([5b0e426](https://github.com/branes-ai/cortex/commit/5b0e426d28f131cc4d1353a1d3e389416fca6ea0))
* S1 initialization stage page (algorithm sound, seed suspect) ([#315](https://github.com/branes-ai/cortex/issues/315)) ([54d8f33](https://github.com/branes-ai/cortex/commit/54d8f33fb227251ab7f5386f82c7822d56ca6b2a))
* S10 online-calibration stage page (the [#212](https://github.com/branes-ai/cortex/issues/212) centerpiece) ([#307](https://github.com/branes-ai/cortex/issues/307)) ([91ead13](https://github.com/branes-ai/cortex/commit/91ead132c3a0e76770ab26d991bb45a3fbe45784))
* S2 IMU-propagation stage page (a cleared [#212](https://github.com/branes-ai/cortex/issues/212) candidate) ([#311](https://github.com/branes-ai/cortex/issues/311)) ([8623a9a](https://github.com/branes-ai/cortex/commit/8623a9a633cbd5b75f6035333b4a302df4986158))
* S6 MSCKF-update stage + consistency-analysis page ([#309](https://github.com/branes-ai/cortex/issues/309)) ([08a9864](https://github.com/branes-ai/cortex/commit/08a986465460b76670256075ef816a6ebfbc6478))
* session log + changelog for the R-IEKF/FEJ/verdict arc ([#369](https://github.com/branes-ai/cortex/issues/369)) ([24c21ff](https://github.com/branes-ai/cortex/commit/24c21ffd05cbc2d05f8479d0b8334dd6c7cece34))
* **session:** wrap up 2026-06-05 — VIO pipeline docs + 3D viz ([#318](https://github.com/branes-ai/cortex/issues/318)) ([7ec2091](https://github.com/branes-ai/cortex/commit/7ec2091398fe6ed7de6d1d0f8c65f11afd0927f6))
* **session:** wrap up 2026-06-05 — VIO pipeline docs section + 3D viz ([7ec2091](https://github.com/branes-ai/cortex/commit/7ec2091398fe6ed7de6d1d0f8c65f11afd0927f6))
* **site:** add S2 IMU-propagation stage page (a cleared [#212](https://github.com/branes-ai/cortex/issues/212) candidate) ([8623a9a](https://github.com/branes-ai/cortex/commit/8623a9a633cbd5b75f6035333b4a302df4986158))
* **site:** add the S1 initialization stage page ([54d8f33](https://github.com/branes-ai/cortex/commit/54d8f33fb227251ab7f5386f82c7822d56ca6b2a))
* **site:** add the S10 online-calibration stage page ([#212](https://github.com/branes-ai/cortex/issues/212) centerpiece) ([91ead13](https://github.com/branes-ai/cortex/commit/91ead132c3a0e76770ab26d991bb45a3fbe45784))
* **site:** improve VIO pipeline overview, S0, and S1 stages ([d701c99](https://github.com/branes-ai/cortex/commit/d701c99b6431d534fec64bdfa75f9bd13b1832d2))
* **site:** improve VIO pipeline overview, S0, and S1 stages ([d701c99](https://github.com/branes-ai/cortex/commit/d701c99b6431d534fec64bdfa75f9bd13b1832d2))
* **site:** migrate the diagnostic-methodology doc as a section page ([fd4b7bb](https://github.com/branes-ai/cortex/commit/fd4b7bb288a32eed1278f27f0f186033ad668e63))
* **site:** widen the triangulation baseline so the 3D view reads ([#364](https://github.com/branes-ai/cortex/issues/364)) ([31d584f](https://github.com/branes-ai/cortex/commit/31d584f467fe0052182441c3a2623450b4680ff0))
* **test:** explain pyramidal KLT + add the numerical failure-mode taxonomy ([49f77d2](https://github.com/branes-ai/cortex/commit/49f77d230dede5b19c35886e31af9cc39e51ffc5))
* **test:** pyramidal KLT explainer + EuRoC numerical failure-mode taxonomy ([#255](https://github.com/branes-ai/cortex/issues/255)) ([49f77d2](https://github.com/branes-ai/cortex/commit/49f77d230dede5b19c35886e31af9cc39e51ffc5))
* **test:** vio_euroc goal & architecture ([#249](https://github.com/branes-ai/cortex/issues/249)) ([b0b75d2](https://github.com/branes-ai/cortex/commit/b0b75d2a584fa4cf99002c4fc7d44b78f010017a))
* VIO pipeline section (overview, metrics, S0, status) + issue-ref policy ([#305](https://github.com/branes-ai/cortex/issues/305)) ([af69f04](https://github.com/branes-ai/cortex/commit/af69f04211876371ce8d132612ea4bd46c7ee9a2))


### Tests

* **bridge:** add C++/Rust cxx::bridge integration test ([#137](https://github.com/branes-ai/cortex/issues/137)) ([50e3e3c](https://github.com/branes-ai/cortex/commit/50e3e3c15e89c47c710f20f67fd7b655f8657d79))
* **math:** golden-data NLS suite + Jacobian FD checks + type matrix ([#155](https://github.com/branes-ai/cortex/issues/155)) ([2348e87](https://github.com/branes-ai/cortex/commit/2348e87e5615d43bcc8b7f5c11d7c6919e629653)), closes [#31](https://github.com/branes-ai/cortex/issues/31)
* **sdk:** dataset-gated MH_05/V2_03 moving-start dynamic-init validation ([#241](https://github.com/branes-ai/cortex/issues/241)) ([b5a0df3](https://github.com/branes-ai/cortex/commit/b5a0df3d4831e3d28cfc4718132c0f778faba09b))
* **sdk:** drive the observability gauge check through the production CameraUpdater ([#358](https://github.com/branes-ai/cortex/issues/358)) ([637990e](https://github.com/branes-ai/cortex/commit/637990e67cb49e2d99dd47406c207435e4ade284))
* **sdk:** dynamic-init diagnostics to localize the real-data divergence ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#248](https://github.com/branes-ai/cortex/issues/248)) ([48be50e](https://github.com/branes-ai/cortex/commit/48be50e27d0a5c38d48402382446fa0f9d1570a2))
* **sdk:** euroc replay harness + ATE/RPE benchmark ([#190](https://github.com/branes-ai/cortex/issues/190)) ([577e201](https://github.com/branes-ai/cortex/commit/577e201af31dcdad55ba00577880aa25940c9da7))
* **sdk:** instrument dynamic-init attempts ([#247](https://github.com/branes-ai/cortex/issues/247) localization) ([#250](https://github.com/branes-ai/cortex/issues/250)) ([972015d](https://github.com/branes-ai/cortex/commit/972015d0544e8f5b86c99ad0c1f609918bb20ac4))
* **sdk:** instrument dynamic-init attempts to localize why it won't fire ([972015d](https://github.com/branes-ai/cortex/commit/972015d0544e8f5b86c99ad0c1f609918bb20ac4))
* **sdk:** latency budget enforcement (per-frame upper bound) ([#191](https://github.com/branes-ai/cortex/issues/191)) ([8dadc96](https://github.com/branes-ai/cortex/commit/8dadc9646bf6eccba940d02a2eb2c002edc2ad18))
* **sdk:** real EuRoC extrinsics — MH_05 moving-start ATE validated (~0.79 m) ([#245](https://github.com/branes-ai/cortex/issues/245)) ([7f35ebb](https://github.com/branes-ai/cortex/commit/7f35ebbe219b9972ae16475e535e9c8028c83fd6))
* **sdk:** score EuRoC ATE from initialization onward ([#253](https://github.com/branes-ai/cortex/issues/253)) ([8304d67](https://github.com/branes-ai/cortex/commit/8304d67105f605a4a137f602a94e3f80ad64986b))
* **sdk:** score EuRoC ATE only from initialization onward ([8304d67](https://github.com/branes-ai/cortex/commit/8304d67105f605a4a137f602a94e3f80ad64986b))


### Maintenance

* **bootstrap:** add bootstrap.sh + bootstrap.ps1 for new-dev setup ([602b172](https://github.com/branes-ai/cortex/commit/602b17209a13e3606d59137f7dbb233a1b7d6302))
* **bootstrap:** add bootstrap.sh + bootstrap.ps1 for new-dev setup ([a5d2397](https://github.com/branes-ai/cortex/commit/a5d2397276e35b0dbf97892309f38c76632fe21b))
* **coderabbit:** add .coderabbit.yaml with layering + license + terminology rules ([#119](https://github.com/branes-ai/cortex/issues/119)) ([5abf053](https://github.com/branes-ai/cortex/commit/5abf0539ff83bed548f098d923b289320f7b8809))
* **format:** add .clang-format, .editorconfig, and license ADR ([c95aadb](https://github.com/branes-ai/cortex/commit/c95aadb6a5083d9d8140ac08ca72754f0d421e34))
* **format:** add .clang-format, .editorconfig, and license-compatibility ADR ([b1f52fd](https://github.com/branes-ai/cortex/commit/b1f52fdb103165f6c028679269c8d57784187ae6))
* **main:** release 0.1.0 ([#122](https://github.com/branes-ai/cortex/issues/122)) ([906c44b](https://github.com/branes-ai/cortex/commit/906c44b3f99fb66df82ec6be8c34738f520fa73b))
* **main:** release 0.1.1 ([#124](https://github.com/branes-ai/cortex/issues/124)) ([726e213](https://github.com/branes-ai/cortex/commit/726e213f0d7ddca126b1d7e9dec33234807959b5))
* **main:** release 0.1.2 ([#126](https://github.com/branes-ai/cortex/issues/126)) ([876d47a](https://github.com/branes-ai/cortex/commit/876d47aa822223ebc317108957b34c138062148c))
* **main:** release 0.10.0 ([#150](https://github.com/branes-ai/cortex/issues/150)) ([c63a79d](https://github.com/branes-ai/cortex/commit/c63a79d67a30f5624c6a83f9cf823b1dbbf3591f))
* **main:** release 0.11.0 ([#152](https://github.com/branes-ai/cortex/issues/152)) ([f79bf4b](https://github.com/branes-ai/cortex/commit/f79bf4b686f7221f2725663b617d9ae3a92e1de8))
* **main:** release 0.12.0 ([#154](https://github.com/branes-ai/cortex/issues/154)) ([dae918f](https://github.com/branes-ai/cortex/commit/dae918ffb26384b7dbbb5ba8c44efae2f3638777))
* **main:** release 0.12.1 ([#156](https://github.com/branes-ai/cortex/issues/156)) ([ef50fb5](https://github.com/branes-ai/cortex/commit/ef50fb5e4d5e2eb696a929e736823fa4c043dc27))
* **main:** release 0.12.2 ([#158](https://github.com/branes-ai/cortex/issues/158)) ([d8c2ae2](https://github.com/branes-ai/cortex/commit/d8c2ae2330e9fe70fb622435d435434a268a118e))
* **main:** release 0.12.3 ([#160](https://github.com/branes-ai/cortex/issues/160)) ([451b9fb](https://github.com/branes-ai/cortex/commit/451b9fbe00be5c010dcbe7a911c89eb45833167b))
* **main:** release 0.13.0 ([#168](https://github.com/branes-ai/cortex/issues/168)) ([5d3f047](https://github.com/branes-ai/cortex/commit/5d3f047bed99fc63b4701f80078c3eba17e24a36))
* **main:** release 0.14.0 ([#171](https://github.com/branes-ai/cortex/issues/171)) ([77c3d7a](https://github.com/branes-ai/cortex/commit/77c3d7a0f40a0e7bbe53be6ea676c006e02c6ad3))
* **main:** release 0.15.0 ([#173](https://github.com/branes-ai/cortex/issues/173)) ([b072ab2](https://github.com/branes-ai/cortex/commit/b072ab2e744e2954a63852146e244d30668aa0b7))
* **main:** release 0.16.0 ([#175](https://github.com/branes-ai/cortex/issues/175)) ([f797289](https://github.com/branes-ai/cortex/commit/f7972898d03aa2abafab51084b19271c39dc4ea9))
* **main:** release 0.17.0 ([#177](https://github.com/branes-ai/cortex/issues/177)) ([2cebf98](https://github.com/branes-ai/cortex/commit/2cebf98d0f5130eb0d7b3e660709f86806c56928))
* **main:** release 0.18.0 ([#179](https://github.com/branes-ai/cortex/issues/179)) ([d738870](https://github.com/branes-ai/cortex/commit/d738870fe2ed6c3db494dfd9bae0c619be6f99fb))
* **main:** release 0.19.0 ([#181](https://github.com/branes-ai/cortex/issues/181)) ([1cd419f](https://github.com/branes-ai/cortex/commit/1cd419fd2b35a514b7cac830b30bd23c8779f364))
* **main:** release 0.2.0 ([#128](https://github.com/branes-ai/cortex/issues/128)) ([fee48a7](https://github.com/branes-ai/cortex/commit/fee48a73d9cf02614df3c415664bc7b8bdad1f64))
* **main:** release 0.20.0 ([#183](https://github.com/branes-ai/cortex/issues/183)) ([e21b268](https://github.com/branes-ai/cortex/commit/e21b268d987d1620d7012c8089d5cb1f7b1191be))
* **main:** release 0.20.1 ([#193](https://github.com/branes-ai/cortex/issues/193)) ([8ca6608](https://github.com/branes-ai/cortex/commit/8ca6608ea7832e4eb7e2b6249ae1c775ff4d9abb))
* **main:** release 0.20.2 ([#195](https://github.com/branes-ai/cortex/issues/195)) ([cfa1399](https://github.com/branes-ai/cortex/commit/cfa139909f221245bdc1b6eb6283a4244c7d6f95))
* **main:** release 0.20.3 ([#198](https://github.com/branes-ai/cortex/issues/198)) ([028a604](https://github.com/branes-ai/cortex/commit/028a604ebaa9277c5d4dbd785e8d737873e8eabc))
* **main:** release 0.21.0 ([#202](https://github.com/branes-ai/cortex/issues/202)) ([b1ca448](https://github.com/branes-ai/cortex/commit/b1ca448f69c164cf79a6169e491c8642bfd2f62c))
* **main:** release 0.22.0 ([#205](https://github.com/branes-ai/cortex/issues/205)) ([626cb2e](https://github.com/branes-ai/cortex/commit/626cb2e4ceb447acd3c70b547798ab675c7b0df2))
* **main:** release 0.23.0 ([#210](https://github.com/branes-ai/cortex/issues/210)) ([aae0e27](https://github.com/branes-ai/cortex/commit/aae0e277da7f735807b0282eed7d18acc50391c3))
* **main:** release 0.23.1 ([#215](https://github.com/branes-ai/cortex/issues/215)) ([f32d309](https://github.com/branes-ai/cortex/commit/f32d3090deec33b6fe12f2c29b1b66550ac4e735))
* **main:** release 0.24.0 ([#217](https://github.com/branes-ai/cortex/issues/217)) ([fa472a2](https://github.com/branes-ai/cortex/commit/fa472a27f0d482251b4548ccc2cc700a34932382))
* **main:** release 0.25.0 ([#219](https://github.com/branes-ai/cortex/issues/219)) ([8d01da3](https://github.com/branes-ai/cortex/commit/8d01da33855c6a934a9512e6190191b22973639f))
* **main:** release 0.25.1 ([#221](https://github.com/branes-ai/cortex/issues/221)) ([e1280a0](https://github.com/branes-ai/cortex/commit/e1280a0d734da304d0442788601ca52b97f71a14))
* **main:** release 0.25.2 ([#224](https://github.com/branes-ai/cortex/issues/224)) ([d79c8e9](https://github.com/branes-ai/cortex/commit/d79c8e9a966e4334831ce921c6bd42f64170252c))
* **main:** release 0.25.3 ([#227](https://github.com/branes-ai/cortex/issues/227)) ([28d9ba7](https://github.com/branes-ai/cortex/commit/28d9ba76838bd2d6a12be9735836da7ed2d7058d))
* **main:** release 0.26.0 ([#232](https://github.com/branes-ai/cortex/issues/232)) ([b37e5c5](https://github.com/branes-ai/cortex/commit/b37e5c51caccde8b6a5a530778406b694dbc769f))
* **main:** release 0.27.0 ([#234](https://github.com/branes-ai/cortex/issues/234)) ([31fd668](https://github.com/branes-ai/cortex/commit/31fd6682f46facf5165d1504b3626443bb58b9b9))
* **main:** release 0.28.0 ([#237](https://github.com/branes-ai/cortex/issues/237)) ([16ee1f6](https://github.com/branes-ai/cortex/commit/16ee1f6ac762cdf89368d411a78cf41f644c00f4))
* **main:** release 0.29.0 ([#239](https://github.com/branes-ai/cortex/issues/239)) ([690c48b](https://github.com/branes-ai/cortex/commit/690c48b1650f50507fbaa671cc1bda647fa39e4b))
* **main:** release 0.3.0 ([#130](https://github.com/branes-ai/cortex/issues/130)) ([b1ff4dd](https://github.com/branes-ai/cortex/commit/b1ff4dde2381975013e3badc3b8f41d5d4605de1))
* **main:** release 0.30.0 ([#242](https://github.com/branes-ai/cortex/issues/242)) ([ada1b27](https://github.com/branes-ai/cortex/commit/ada1b27d43244465cef31e5439f1c87b7f937533))
* **main:** release 0.30.0 ([#257](https://github.com/branes-ai/cortex/issues/257)) ([45344de](https://github.com/branes-ai/cortex/commit/45344de27bef140ec135201102dfa4562534c44b))
* **main:** release 0.30.1 ([#258](https://github.com/branes-ai/cortex/issues/258)) ([98b5f64](https://github.com/branes-ai/cortex/commit/98b5f646e5332b75f4ad661fc60e291e9833db6b))
* **main:** release 0.30.2 ([#260](https://github.com/branes-ai/cortex/issues/260)) ([66785e6](https://github.com/branes-ai/cortex/commit/66785e6b3692b8d70bb3a22e3a7be4444226d8e6))
* **main:** release 0.31.0 ([#270](https://github.com/branes-ai/cortex/issues/270)) ([28c41d9](https://github.com/branes-ai/cortex/commit/28c41d94867ba33ae9aaf44fbe145d463e35b21c))
* **main:** release 0.32.0 ([#275](https://github.com/branes-ai/cortex/issues/275)) ([b676fa2](https://github.com/branes-ai/cortex/commit/b676fa2c9a72eaf1a535affd919f1587862a7b3d))
* **main:** release 0.33.0 ([#277](https://github.com/branes-ai/cortex/issues/277)) ([e3c425d](https://github.com/branes-ai/cortex/commit/e3c425d885741a8e4f1a9ca60039fdd832d0f1a8))
* **main:** release 0.34.0 ([#281](https://github.com/branes-ai/cortex/issues/281)) ([f33b37d](https://github.com/branes-ai/cortex/commit/f33b37d8a78bebe1af4b3901cbd9c1eed7a579fd))
* **main:** release 0.35.0 ([#284](https://github.com/branes-ai/cortex/issues/284)) ([a12560b](https://github.com/branes-ai/cortex/commit/a12560b36f7a2166bbca05f9d94f75cf83c35d6d))
* **main:** release 0.36.0 ([#289](https://github.com/branes-ai/cortex/issues/289)) ([816f55e](https://github.com/branes-ai/cortex/commit/816f55e8e1a8f72cd15821a243c64b482896efc2))
* **main:** release 0.37.0 ([#293](https://github.com/branes-ai/cortex/issues/293)) ([488e708](https://github.com/branes-ai/cortex/commit/488e70841eef3d142e0fe71188378140b5ed59d2))
* **main:** release 0.38.0 ([#301](https://github.com/branes-ai/cortex/issues/301)) ([6a82437](https://github.com/branes-ai/cortex/commit/6a82437fd8b50c894fc13ac39c14948447fbe7bf))
* **main:** release 0.39.0 ([#304](https://github.com/branes-ai/cortex/issues/304)) ([5aeb901](https://github.com/branes-ai/cortex/commit/5aeb9018322e5fd5ba549eb93a4bfa60899734ef))
* **main:** release 0.39.1 ([#306](https://github.com/branes-ai/cortex/issues/306)) ([6df6738](https://github.com/branes-ai/cortex/commit/6df6738182d1897d1fb574ded20105bdfd588099))
* **main:** release 0.39.2 ([#308](https://github.com/branes-ai/cortex/issues/308)) ([7286b27](https://github.com/branes-ai/cortex/commit/7286b279c81a24896f2d8298aa7d86c74976b714))
* **main:** release 0.39.3 ([#310](https://github.com/branes-ai/cortex/issues/310)) ([a641bfa](https://github.com/branes-ai/cortex/commit/a641bfacdfae21da9336746f3cbc974bc3870a9b))
* **main:** release 0.39.4 ([#312](https://github.com/branes-ai/cortex/issues/312)) ([810384f](https://github.com/branes-ai/cortex/commit/810384f4ad98e50b0858f5662b54b8c10b5b577b))
* **main:** release 0.4.0 ([#132](https://github.com/branes-ai/cortex/issues/132)) ([3a7a2da](https://github.com/branes-ai/cortex/commit/3a7a2dad68022bd9530063d732151930436288b6))
* **main:** release 0.40.0 ([#314](https://github.com/branes-ai/cortex/issues/314)) ([2354e89](https://github.com/branes-ai/cortex/commit/2354e898e924393a02f20ee850103956e704deca))
* **main:** release 0.40.1 ([#317](https://github.com/branes-ai/cortex/issues/317)) ([514d2f3](https://github.com/branes-ai/cortex/commit/514d2f3815e21c62459f9b6b495a9bbf490ac698))
* **main:** release 0.41.0 ([#320](https://github.com/branes-ai/cortex/issues/320)) ([ca0f3e4](https://github.com/branes-ai/cortex/commit/ca0f3e4da516d4f26dfdd2a0cff9bf860b2e16f0))
* **main:** release 0.42.0 ([#322](https://github.com/branes-ai/cortex/issues/322)) ([fec3ed0](https://github.com/branes-ai/cortex/commit/fec3ed048ca3645bf3eac9d7487b9a959ff6069d))
* **main:** release 0.43.0 ([#324](https://github.com/branes-ai/cortex/issues/324)) ([10a0bd8](https://github.com/branes-ai/cortex/commit/10a0bd89d8e69cde7861714f9885133e29a50245))
* **main:** release 0.44.0 ([#326](https://github.com/branes-ai/cortex/issues/326)) ([5ec26dd](https://github.com/branes-ai/cortex/commit/5ec26dd3b6a468fd7be4b7705d39d9de7d722533))
* **main:** release 0.45.0 ([#329](https://github.com/branes-ai/cortex/issues/329)) ([36c92b1](https://github.com/branes-ai/cortex/commit/36c92b1ada450393d715992b65aa479ccf3de02b))
* **main:** release 0.45.1 ([#331](https://github.com/branes-ai/cortex/issues/331)) ([b463890](https://github.com/branes-ai/cortex/commit/b4638908fe38c3ce35d9706eb27ae4bee46569c9))
* **main:** release 0.45.2 ([#336](https://github.com/branes-ai/cortex/issues/336)) ([63bf737](https://github.com/branes-ai/cortex/commit/63bf737fc271fe53c2ad41c5cb9a0fff863a5aa5))
* **main:** release 0.46.0 ([#342](https://github.com/branes-ai/cortex/issues/342)) ([e3c1f12](https://github.com/branes-ai/cortex/commit/e3c1f123b36969682c4db3d35e583ba08755ec9f))
* **main:** release 0.47.0 ([#344](https://github.com/branes-ai/cortex/issues/344)) ([1304022](https://github.com/branes-ai/cortex/commit/1304022a90008d6d44863c7e261a51e29578015f))
* **main:** release 0.47.1 ([#346](https://github.com/branes-ai/cortex/issues/346)) ([23c08fb](https://github.com/branes-ai/cortex/commit/23c08fb071895c543caf90560c56d78c91498500))
* **main:** release 0.48.0 ([#350](https://github.com/branes-ai/cortex/issues/350)) ([8c41659](https://github.com/branes-ai/cortex/commit/8c41659242c857a4383ea000c6abb75ba985ac10))
* **main:** release 0.49.0 ([#352](https://github.com/branes-ai/cortex/issues/352)) ([ec1ce65](https://github.com/branes-ai/cortex/commit/ec1ce65bfb46382612d367d8aaea5aa29297edfb))
* **main:** release 0.5.0 ([#134](https://github.com/branes-ai/cortex/issues/134)) ([2831099](https://github.com/branes-ai/cortex/commit/2831099c8082268cdaf01d39726c913735d78f64))
* **main:** release 0.5.1 ([#136](https://github.com/branes-ai/cortex/issues/136)) ([9d1e52e](https://github.com/branes-ai/cortex/commit/9d1e52ed8f16b77b6a1236b88708a0bebeef0194))
* **main:** release 0.5.2 ([#138](https://github.com/branes-ai/cortex/issues/138)) ([f042a4c](https://github.com/branes-ai/cortex/commit/f042a4c0529d06ca521a10ee0c5c7a43d8ebc1e7))
* **main:** release 0.5.3 ([#140](https://github.com/branes-ai/cortex/issues/140)) ([176b3a6](https://github.com/branes-ai/cortex/commit/176b3a6cc39dc430dea8fc560d4b96509e6317b6))
* **main:** release 0.50.0 ([#355](https://github.com/branes-ai/cortex/issues/355)) ([7aab4d7](https://github.com/branes-ai/cortex/commit/7aab4d72a90957d82ed8fb512285749c45c4f7ce))
* **main:** release 0.50.1 ([#359](https://github.com/branes-ai/cortex/issues/359)) ([5a4ef99](https://github.com/branes-ai/cortex/commit/5a4ef99b09b5fed9788f9247fa7af85cbe7358d0))
* **main:** release 0.50.2 ([#361](https://github.com/branes-ai/cortex/issues/361)) ([87569ee](https://github.com/branes-ai/cortex/commit/87569ee6f93a76c82e9d190debcd87b65224c0a2))
* **main:** release 0.50.3 ([#363](https://github.com/branes-ai/cortex/issues/363)) ([2bf3a84](https://github.com/branes-ai/cortex/commit/2bf3a8496cc274216289c04c0bf0cd8d929f4cfe))
* **main:** release 0.51.0 ([#368](https://github.com/branes-ai/cortex/issues/368)) ([f1c071f](https://github.com/branes-ai/cortex/commit/f1c071f8053218d55b83fa442cb0ae563969efee))
* **main:** release 0.51.1 ([#370](https://github.com/branes-ai/cortex/issues/370)) ([0820a0b](https://github.com/branes-ai/cortex/commit/0820a0bec3aaddaf9c757e64d4dbe58e7f5c1a13))
* **main:** release 0.52.0 ([#386](https://github.com/branes-ai/cortex/issues/386)) ([2b2a66e](https://github.com/branes-ai/cortex/commit/2b2a66ec2f6260e8d85b0b78061cf7c1781516b4))
* **main:** release 0.53.0 ([#389](https://github.com/branes-ai/cortex/issues/389)) ([930cbf1](https://github.com/branes-ai/cortex/commit/930cbf16e89587e437d9e495acfae8219ce397cc))
* **main:** release 0.53.1 ([#391](https://github.com/branes-ai/cortex/issues/391)) ([cac68b6](https://github.com/branes-ai/cortex/commit/cac68b6e1e71bd5a7fd65217b6e40247fd2cdf25))
* **main:** release 0.54.0 ([#392](https://github.com/branes-ai/cortex/issues/392)) ([54b2110](https://github.com/branes-ai/cortex/commit/54b21102d8b552c652d2a12a54109f5b79cfce22))
* **main:** release 0.55.0 ([#395](https://github.com/branes-ai/cortex/issues/395)) ([cdfda2b](https://github.com/branes-ai/cortex/commit/cdfda2b031324107a850eaa2a398d18d761c66e6))
* **main:** release 0.56.0 ([#397](https://github.com/branes-ai/cortex/issues/397)) ([1090c5f](https://github.com/branes-ai/cortex/commit/1090c5fa9116b2f6ef0dc22b2a9fc2c6372168c6))
* **main:** release 0.57.0 ([#401](https://github.com/branes-ai/cortex/issues/401)) ([cf3ebc9](https://github.com/branes-ai/cortex/commit/cf3ebc959baef1d709ada85549dd1e1f124e7975))
* **main:** release 0.58.0 ([#402](https://github.com/branes-ai/cortex/issues/402)) ([792cc1c](https://github.com/branes-ai/cortex/commit/792cc1cd7f4106f59db24cac8e559cac524ee17c))
* **main:** release 0.59.0 ([#404](https://github.com/branes-ai/cortex/issues/404)) ([b06cc6c](https://github.com/branes-ai/cortex/commit/b06cc6cc895b389170942819bd21760113807f53))
* **main:** release 0.59.1 ([#408](https://github.com/branes-ai/cortex/issues/408)) ([e1c4f40](https://github.com/branes-ai/cortex/commit/e1c4f40df4a6034d5393ad863c8341aaa4ea1132))
* **main:** release 0.6.0 ([#142](https://github.com/branes-ai/cortex/issues/142)) ([e3fceea](https://github.com/branes-ai/cortex/commit/e3fceea4f45f83214431950966fb27453ee30f94))
* **main:** release 0.7.0 ([#144](https://github.com/branes-ai/cortex/issues/144)) ([de2565a](https://github.com/branes-ai/cortex/commit/de2565a80d3f3767731380e4b7a71bb4e86a946a))
* **main:** release 0.8.0 ([#146](https://github.com/branes-ai/cortex/issues/146)) ([b89bc55](https://github.com/branes-ai/cortex/commit/b89bc557065dce6f76d3ab0caa5ead86707064e1))
* **main:** release 0.9.0 ([#148](https://github.com/branes-ai/cortex/issues/148)) ([f9b0b38](https://github.com/branes-ai/cortex/commit/f9b0b3866f0b75b26b71c0a81ca570e876aec01b))
* **release:** add commitlint + release-please for SemVer ([8cd388b](https://github.com/branes-ai/cortex/commit/8cd388b2772c57a14380752677adcf809e9e7f69))
* **release:** add commitlint + release-please for SemVer ([24e2eab](https://github.com/branes-ai/cortex/commit/24e2eab68bc7e138067756cf7481b2e9877a3f6b))
* **release:** pin initial-version to 0.1.0 (alpha pre-1.0 semantics) ([#121](https://github.com/branes-ai/cortex/issues/121)) ([527a620](https://github.com/branes-ai/cortex/commit/527a620daac9d1d4c437eb568e53c915995bbaf2))
* **repo:** track team Claude config — docs-site build allowlist + auto-build ([#393](https://github.com/branes-ai/cortex/issues/393)) ([ac570c5](https://github.com/branes-ai/cortex/commit/ac570c5cd0984917f5393b8213d55224063e1d28))
* **testing:** add regression infra — coverage + latency budget framework ([#123](https://github.com/branes-ai/cortex/issues/123)) ([5e62988](https://github.com/branes-ai/cortex/commit/5e62988d70cb5e2df143b929d9696423aa4e6b21))
* **toolchain:** bump Corrosion v0.4.2 -&gt; v0.6.1 ([bf96317](https://github.com/branes-ai/cortex/commit/bf9631756731dcb0879c8db80738026cf02f7a4d))
* **toolchain:** pin Rust 1.83.0 and unblock skeleton build ([417bd10](https://github.com/branes-ai/cortex/commit/417bd1049548cca2d231cddfaca15e483ea5b6fd))
* **toolchain:** pin Rust 1.83.0 and unblock skeleton build ([68b4c62](https://github.com/branes-ai/cortex/commit/68b4c62e8a0faf2e953cdba8ae6e769479dc77a4))
* **tools:** add the R-IEKF system-level NEES diagnostic ([#212](https://github.com/branes-ai/cortex/issues/212)) ([#360](https://github.com/branes-ai/cortex/issues/360)) ([6640263](https://github.com/branes-ai/cortex/commit/664026305b4fcb85022d5e198b9dbdeb2b61d851))
* wrapup 2026-05-24 — Phase 1 MVP session log + CHANGELOG marker ([#139](https://github.com/branes-ai/cortex/issues/139)) ([215cae4](https://github.com/branes-ai/cortex/commit/215cae44114b888c7e44834c665b309d3525aeca))
* wrapup Phase 0 — add CHANGELOG.md, session log, gitignore cargo target ([fcb855e](https://github.com/branes-ai/cortex/commit/fcb855e4a2c70d0908be27ae3702e00f4cd75ee9))
* wrapup Phase 0 — CHANGELOG.md, session log, gitignore target/ ([37143d4](https://github.com/branes-ai/cortex/commit/37143d43c8950b609c125f9eca29062f19f05b88))

## [0.59.1](https://github.com/branes-ai/cortex/compare/v0.59.0...v0.59.1) (2026-06-17)


### Documentation

* improve overview phrasing and trigger documentation rebuild ([56f6811](https://github.com/branes-ai/cortex/commit/56f6811b01eb34a97b2d29639351f19a6613df6c))

## [0.59.0](https://github.com/branes-ai/cortex/compare/v0.58.0...v0.59.0) (2026-06-17)


### Features

* **tools:** s6 msckf-update inspector — real EuRoC updates → state/covariance ([#403](https://github.com/branes-ai/cortex/issues/403)) ([a707fbd](https://github.com/branes-ai/cortex/commit/a707fbdfbbd06ce2164672eaae9241ecffda8e90))

## [0.58.0](https://github.com/branes-ai/cortex/compare/v0.57.0...v0.58.0) (2026-06-17)


### Features

* **sdk:** implement Phase C 6-DoF online extrinsic calibration for MsckfInvariantBackend ([f9007da](https://github.com/branes-ai/cortex/commit/f9007dad737fc18815c3f3208d754df45f319625))
* **sdk:** Phase C - 6-DoF online extrinsic calibration for MsckfInvariantBackend ([#400](https://github.com/branes-ai/cortex/issues/400)) ([f9007da](https://github.com/branes-ai/cortex/commit/f9007dad737fc18815c3f3208d754df45f319625))

## [0.57.0](https://github.com/branes-ai/cortex/compare/v0.56.0...v0.57.0) (2026-06-17)


### Features

* **tools:** s5 triangulation inspector — real EuRoC tracks+poses → 3D landmarks ([#398](https://github.com/branes-ai/cortex/issues/398)) ([7a5ffb0](https://github.com/branes-ai/cortex/commit/7a5ffb0cb65a2771db4d1a19019852e9b7738b7a))

## [0.56.0](https://github.com/branes-ai/cortex/compare/v0.55.0...v0.56.0) (2026-06-16)


### Features

* **sdk:** add 6-DoF rigid camera extrinsics to MsckfInvariantBackend ([#396](https://github.com/branes-ai/cortex/issues/396)) ([8ed59cc](https://github.com/branes-ai/cortex/commit/8ed59cc4e50313bf1a2414392ed4f78f0e14fac5))


### Reverts

* **docs:** restore original working vio_run.jsonl ([397a2d2](https://github.com/branes-ai/cortex/commit/397a2d2c3df4645987b2181728e6da96321a9f15))

## [0.55.0](https://github.com/branes-ai/cortex/compare/v0.54.0...v0.55.0) (2026-06-16)


### Features

* **sdk:** add 6-DoF rigid camera extrinsics to MsckfInvariantBackend ([105852f](https://github.com/branes-ai/cortex/commit/105852f7f4d1801b4d45f6df20258a0b619ad1a2))
* **tools:** s0 sensor-model inspector — real EuRoC frame/IMU → camera & noise contract ([#394](https://github.com/branes-ai/cortex/issues/394)) ([f1599e6](https://github.com/branes-ai/cortex/commit/f1599e6ab25ee2ef3ac4d62430bdcd5ddaee3046))

## [0.54.0](https://github.com/branes-ai/cortex/compare/v0.53.1...v0.54.0) (2026-06-16)


### Features

* **tools:** S4 frontend inspector — real EuRoC FAST+KLT tracks, visualized ([#390](https://github.com/branes-ai/cortex/issues/390)) ([4ecb210](https://github.com/branes-ai/cortex/commit/4ecb210a8eef88c531fec472b7dd995704a70ce2))


### Maintenance

* **repo:** track team Claude config — docs-site build allowlist + auto-build ([#393](https://github.com/branes-ai/cortex/issues/393)) ([ac570c5](https://github.com/branes-ai/cortex/commit/ac570c5cd0984917f5393b8213d55224063e1d28))

## [0.53.1](https://github.com/branes-ai/cortex/compare/v0.53.0...v0.53.1) (2026-06-16)


### Bug Fixes

* **sdk:** resolve R-IEKF continuous divergence ([#366](https://github.com/branes-ai/cortex/issues/366)) ([4f4ee1d](https://github.com/branes-ai/cortex/commit/4f4ee1d4707647152be84dfbc511dff1fc77afaf))
* **sdk:** resolve R-IEKF continuous divergence ([#366](https://github.com/branes-ai/cortex/issues/366)) ([#388](https://github.com/branes-ai/cortex/issues/388)) ([4f4ee1d](https://github.com/branes-ai/cortex/commit/4f4ee1d4707647152be84dfbc511dff1fc77afaf))

## [0.53.0](https://github.com/branes-ai/cortex/compare/v0.52.0...v0.53.0) (2026-06-16)


### Features

* **tools:** --trace tap for EuRoC asl_replay — real per-stage trace dumps ([#387](https://github.com/branes-ai/cortex/issues/387)) ([b9cc112](https://github.com/branes-ai/cortex/commit/b9cc112605d9029b15cbf55fce07cd3a75d768a5))

## [0.52.0](https://github.com/branes-ai/cortex/compare/v0.51.1...v0.52.0) (2026-06-15)


### Features

* **tools:** inter-stage VIO trace bus — typed, human-readable per-stage I/O records ([#385](https://github.com/branes-ai/cortex/issues/385)) ([e5b8caf](https://github.com/branes-ai/cortex/commit/e5b8caf8be515ae572491822ff187dd777bf65d6))

## [0.51.1](https://github.com/branes-ai/cortex/compare/v0.51.0...v0.51.1) (2026-06-15)


### Documentation

* session log + changelog for the R-IEKF/FEJ/verdict arc ([#369](https://github.com/branes-ai/cortex/issues/369)) ([24c21ff](https://github.com/branes-ai/cortex/commit/24c21ffd05cbc2d05f8479d0b8334dd6c7cece34))

## [0.51.0](https://github.com/branes-ai/cortex/compare/v0.50.3...v0.51.0) (2026-06-15)


### Features

* **sdk:** R-IEKF VIO backend adapter + continuous-update divergence localization ([#365](https://github.com/branes-ai/cortex/issues/365)) ([#367](https://github.com/branes-ai/cortex/issues/367)) ([f7c2671](https://github.com/branes-ai/cortex/commit/f7c26718b36bce00066048ed658db6db781cb67b))

## [0.50.3](https://github.com/branes-ai/cortex/compare/v0.50.2...v0.50.3) (2026-06-14)


### Documentation

* render LaTeX math on the site (KaTeX) + camera-calibration walkthrough ([#362](https://github.com/branes-ai/cortex/issues/362)) ([5b0e426](https://github.com/branes-ai/cortex/commit/5b0e426d28f131cc4d1353a1d3e389416fca6ea0))
* **site:** widen the triangulation baseline so the 3D view reads ([#364](https://github.com/branes-ai/cortex/issues/364)) ([31d584f](https://github.com/branes-ai/cortex/commit/31d584f467fe0052182441c3a2623450b4680ff0))

## [0.50.2](https://github.com/branes-ai/cortex/compare/v0.50.1...v0.50.2) (2026-06-14)


### Maintenance

* **tools:** add the R-IEKF system-level NEES diagnostic ([#212](https://github.com/branes-ai/cortex/issues/212)) ([#360](https://github.com/branes-ai/cortex/issues/360)) ([6640263](https://github.com/branes-ai/cortex/commit/664026305b4fcb85022d5e198b9dbdeb2b61d851))

## [0.50.1](https://github.com/branes-ai/cortex/compare/v0.50.0...v0.50.1) (2026-06-13)


### Tests

* **sdk:** drive the observability gauge check through the production CameraUpdater ([#358](https://github.com/branes-ai/cortex/issues/358)) ([637990e](https://github.com/branes-ai/cortex/commit/637990e67cb49e2d99dd47406c207435e4ade284))

## [0.50.0](https://github.com/branes-ai/cortex/compare/v0.49.0...v0.50.0) (2026-06-13)


### Features

* **sdk:** add the R-IEKF invariant camera measurement update ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#356](https://github.com/branes-ai/cortex/issues/356)) ([c136991](https://github.com/branes-ai/cortex/commit/c136991766f7d97e655987a0988bbffac78d5a95))
* **sdk:** add the R-IEKF invariant IMU propagation on SE2(3) ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#354](https://github.com/branes-ai/cortex/issues/354)) ([98683ab](https://github.com/branes-ai/cortex/commit/98683ab7a87248d1ff3c9d3460e878cfaef474e5))
* **sdk:** wire the MsckfInvariantBackend (R-IEKF filter) ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#357](https://github.com/branes-ai/cortex/issues/357)) ([a52f6f8](https://github.com/branes-ai/cortex/commit/a52f6f838c15aa76c8095d53d836d383303e651c))

## [0.49.0](https://github.com/branes-ai/cortex/compare/v0.48.0...v0.49.0) (2026-06-13)


### Features

* **math:** add SE2(3) extended-pose Lie group — the R-IEKF state manifold ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#353](https://github.com/branes-ai/cortex/issues/353)) ([1a7da91](https://github.com/branes-ai/cortex/commit/1a7da91b593e7065d0fdb09aae90562eccbf2870))
* **sdk:** prove the right-invariant propagation Φ is state-independent (R-IEKF Phase A, [#348](https://github.com/branes-ai/cortex/issues/348)) ([#351](https://github.com/branes-ai/cortex/issues/351)) ([a0c35aa](https://github.com/branes-ai/cortex/commit/a0c35aafb60be143978b39c88675fa009dc1d116))

## [0.48.0](https://github.com/branes-ai/cortex/compare/v0.47.1...v0.48.0) (2026-06-13)


### Features

* **sdk:** R-IEKF Phase-A gate — invariant parameterization flattens the yaw leak ([#348](https://github.com/branes-ai/cortex/issues/348)) ([#349](https://github.com/branes-ai/cortex/issues/349)) ([427134e](https://github.com/branes-ai/cortex/commit/427134ea0516f31292b101428759cadcc7d2fdd3))

## [0.47.1](https://github.com/branes-ai/cortex/compare/v0.47.0...v0.47.1) (2026-06-13)


### Documentation

* record the [#212](https://github.com/branes-ai/cortex/issues/212) observability investigation — yaw leak localized, FEJ refuted, R-IEKF next ([#345](https://github.com/branes-ai/cortex/issues/345)) ([714bc9f](https://github.com/branes-ai/cortex/commit/714bc9f5d1485741e1bfc403b51a72ce165ca971))

## [0.47.0](https://github.com/branes-ai/cortex/compare/v0.46.0...v0.47.0) (2026-06-13)


### Features

* **sdk:** observability null-space probe — localizes the [#212](https://github.com/branes-ai/cortex/issues/212) NEES leak to yaw ([#337](https://github.com/branes-ai/cortex/issues/337)) ([#338](https://github.com/branes-ai/cortex/issues/338)) ([2720ec7](https://github.com/branes-ai/cortex/commit/2720ec7fae90a6a417488eee8bb9a5b524b1b5c1))

## [0.46.0](https://github.com/branes-ai/cortex/compare/v0.45.2...v0.46.0) (2026-06-12)


### Features

* **sdk:** online extrinsic calibration (S10-as-state), mechanism + synthetic proof ([#332](https://github.com/branes-ai/cortex/issues/332)) ([#333](https://github.com/branes-ai/cortex/issues/333)) ([e5fc23f](https://github.com/branes-ai/cortex/commit/e5fc23f235bd31cda7f82e5df8822b47fd5bf335))
* **test:** EuRoC verdict — online extrinsic calibration cleared as [#212](https://github.com/branes-ai/cortex/issues/212) driver ([#332](https://github.com/branes-ai/cortex/issues/332)) ([#343](https://github.com/branes-ai/cortex/issues/343)) ([98488f6](https://github.com/branes-ai/cortex/commit/98488f6a0e5b8aa2a204accf52c0e380320277eb))

## [0.45.2](https://github.com/branes-ai/cortex/compare/v0.45.1...v0.45.2) (2026-06-12)


### Bug Fixes

* **docs:** make topology + VIO stage-flow SVGs readable in dark theme ([#334](https://github.com/branes-ai/cortex/issues/334)) ([c6d494c](https://github.com/branes-ai/cortex/commit/c6d494cecdd8e3a2f9b883b4dc50b8b84e6b5b85))

## [0.45.1](https://github.com/branes-ai/cortex/compare/v0.45.0...v0.45.1) (2026-06-12)


### Documentation

* add VIO S0–S10 stage-flow SVG and replace the pipeline ASCII dataflow ([2b93b51](https://github.com/branes-ai/cortex/commit/2b93b512fd3405d3fcb69a6c8b4322f60869ee78))
* add VIO S0–S10 stage-flow SVG; replace pipeline ASCII dataflow ([#330](https://github.com/branes-ai/cortex/issues/330)) ([2b93b51](https://github.com/branes-ai/cortex/commit/2b93b512fd3405d3fcb69a6c8b4322f60869ee78))

## [0.45.0](https://github.com/branes-ai/cortex/compare/v0.44.0...v0.45.0) (2026-06-12)


### Features

* **sdk:** add S10 calibration-uncertainty R-term; measure it end-to-end on V2_03 ([#328](https://github.com/branes-ai/cortex/issues/328)) ([bce345f](https://github.com/branes-ai/cortex/commit/bce345ff897d35134b8fb02479551ea682d1ea7a))


### Documentation

* render architecture topology as SVG; wire in orphaned S0 fisheye figures ([#327](https://github.com/branes-ai/cortex/issues/327)) ([af801c6](https://github.com/branes-ai/cortex/commit/af801c6dfd69fae88e5299f807a416ea80a14f41))

## [0.44.0](https://github.com/branes-ai/cortex/compare/v0.43.0...v0.44.0) (2026-06-11)


### Features

* **sdk:** S6 update probe; lock S3/S5/S6/S9 contracts by measurement ([#325](https://github.com/branes-ai/cortex/issues/325)) ([df6cafe](https://github.com/branes-ai/cortex/commit/df6cafe53035a9c7861852163a7afeafa7315aa5))

## [0.43.0](https://github.com/branes-ai/cortex/compare/v0.42.0...v0.43.0) (2026-06-06)


### Features

* add the S4 forward-backward and S5 parallax outlier gates ([#323](https://github.com/branes-ai/cortex/issues/323)) ([73bcd81](https://github.com/branes-ai/cortex/commit/73bcd81c404d0f9e396680e190291da4c0e2e2f9))

## [0.42.0](https://github.com/branes-ai/cortex/compare/v0.41.0...v0.42.0) (2026-06-06)


### Features

* wire the S4 visual-frontend probe + data-backed page ([#321](https://github.com/branes-ai/cortex/issues/321)) ([67ac45c](https://github.com/branes-ai/cortex/commit/67ac45c2282bdf23202339750c127d477e69e801))

## [0.41.0](https://github.com/branes-ai/cortex/compare/v0.40.1...v0.41.0) (2026-06-06)


### Features

* complete the structural stage pages (S3, S4, S7-S9) ([#319](https://github.com/branes-ai/cortex/issues/319)) ([d3ec53b](https://github.com/branes-ai/cortex/commit/d3ec53b437f7bf70864e03b3618ceeda2c0c1c16))

## [0.40.1](https://github.com/branes-ai/cortex/compare/v0.40.0...v0.40.1) (2026-06-06)


### Documentation

* migrate diagnostic-methodology as the section's method primer ([#316](https://github.com/branes-ai/cortex/issues/316)) ([fd4b7bb](https://github.com/branes-ai/cortex/commit/fd4b7bb288a32eed1278f27f0f186033ad668e63))
* **session:** wrap up 2026-06-05 — VIO pipeline docs + 3D viz ([#318](https://github.com/branes-ai/cortex/issues/318)) ([7ec2091](https://github.com/branes-ai/cortex/commit/7ec2091398fe6ed7de6d1d0f8c65f11afd0927f6))
* **session:** wrap up 2026-06-05 — VIO pipeline docs section + 3D viz ([7ec2091](https://github.com/branes-ai/cortex/commit/7ec2091398fe6ed7de6d1d0f8c65f11afd0927f6))
* **site:** migrate the diagnostic-methodology doc as a section page ([fd4b7bb](https://github.com/branes-ai/cortex/commit/fd4b7bb288a32eed1278f27f0f186033ad668e63))

## [0.40.0](https://github.com/branes-ai/cortex/compare/v0.39.4...v0.40.0) (2026-06-06)


### Features

* S5 feature-triangulation probe + stage page (parallax gate, quantified) ([#313](https://github.com/branes-ai/cortex/issues/313)) ([8fbf1cd](https://github.com/branes-ai/cortex/commit/8fbf1cddfe0863b4cbe1ca806abd513c73c5f591))


### Documentation

* S1 initialization stage page (algorithm sound, seed suspect) ([#315](https://github.com/branes-ai/cortex/issues/315)) ([54d8f33](https://github.com/branes-ai/cortex/commit/54d8f33fb227251ab7f5386f82c7822d56ca6b2a))
* **site:** add the S1 initialization stage page ([54d8f33](https://github.com/branes-ai/cortex/commit/54d8f33fb227251ab7f5386f82c7822d56ca6b2a))

## [0.39.4](https://github.com/branes-ai/cortex/compare/v0.39.3...v0.39.4) (2026-06-05)


### Documentation

* S2 IMU-propagation stage page (a cleared [#212](https://github.com/branes-ai/cortex/issues/212) candidate) ([#311](https://github.com/branes-ai/cortex/issues/311)) ([8623a9a](https://github.com/branes-ai/cortex/commit/8623a9a633cbd5b75f6035333b4a302df4986158))
* **site:** add S2 IMU-propagation stage page (a cleared [#212](https://github.com/branes-ai/cortex/issues/212) candidate) ([8623a9a](https://github.com/branes-ai/cortex/commit/8623a9a633cbd5b75f6035333b4a302df4986158))

## [0.39.3](https://github.com/branes-ai/cortex/compare/v0.39.2...v0.39.3) (2026-06-05)


### Documentation

* S6 MSCKF-update stage + consistency-analysis page ([#309](https://github.com/branes-ai/cortex/issues/309)) ([08a9864](https://github.com/branes-ai/cortex/commit/08a986465460b76670256075ef816a6ebfbc6478))

## [0.39.2](https://github.com/branes-ai/cortex/compare/v0.39.1...v0.39.2) (2026-06-05)


### Documentation

* S10 online-calibration stage page (the [#212](https://github.com/branes-ai/cortex/issues/212) centerpiece) ([#307](https://github.com/branes-ai/cortex/issues/307)) ([91ead13](https://github.com/branes-ai/cortex/commit/91ead132c3a0e76770ab26d991bb45a3fbe45784))
* **site:** add the S10 online-calibration stage page ([#212](https://github.com/branes-ai/cortex/issues/212) centerpiece) ([91ead13](https://github.com/branes-ai/cortex/commit/91ead132c3a0e76770ab26d991bb45a3fbe45784))

## [0.39.1](https://github.com/branes-ai/cortex/compare/v0.39.0...v0.39.1) (2026-06-05)


### Documentation

* VIO pipeline section (overview, metrics, S0, status) + issue-ref policy ([#305](https://github.com/branes-ai/cortex/issues/305)) ([af69f04](https://github.com/branes-ai/cortex/commit/af69f04211876371ce8d132612ea4bd46c7ee9a2))

## [0.39.0](https://github.com/branes-ai/cortex/compare/v0.38.0...v0.39.0) (2026-06-05)


### Features

* landmark cloud + camera frustum in the 3D viewer ([#303](https://github.com/branes-ai/cortex/issues/303)) ([7f4f532](https://github.com/branes-ai/cortex/commit/7f4f5320f76ee891162d5ca0b53b0ad9835a0410))

## [0.38.0](https://github.com/branes-ai/cortex/compare/v0.37.0...v0.38.0) (2026-06-05)


### Features

* 3D drone path/pose viewer (Phase 0 + Phase 1 prototype) ([#302](https://github.com/branes-ai/cortex/issues/302)) ([b4da4a4](https://github.com/branes-ai/cortex/commit/b4da4a40c4e13b32967ee21c98cd69e2ef6a2f23))
* **tools:** depth-colored scene overlay + vio_pipeline how-to guide ([#300](https://github.com/branes-ai/cortex/issues/300)) ([96df842](https://github.com/branes-ai/cortex/commit/96df8421dde43a19806bd8bdbba5d89140afcba6))

## [0.37.0](https://github.com/branes-ai/cortex/compare/v0.36.0...v0.37.0) (2026-06-05)


### Features

* **tools:** add EuRoC source + scene-video metric overlay to vio_pipeline ([#292](https://github.com/branes-ai/cortex/issues/292)) ([f4b8e9e](https://github.com/branes-ai/cortex/commit/f4b8e9e95e3f9a42786e3177b76c295bf7be8061))
* **tools:** wire S10 (calibration) stage-probe to quantify the [#212](https://github.com/branes-ai/cortex/issues/212) Rx4 ([#295](https://github.com/branes-ai/cortex/issues/295)) ([9a01499](https://github.com/branes-ai/cortex/commit/9a01499186229199eeb6ccd7a95df6ed14e1a7a1))


### Bug Fixes

* **tools:** create the --out directory so artifacts are not silently dropped ([#298](https://github.com/branes-ai/cortex/issues/298)) ([029727d](https://github.com/branes-ai/cortex/commit/029727d13b99d4a29c2695aef579378e04913144))

## [0.36.0](https://github.com/branes-ai/cortex/compare/v0.35.0...v0.36.0) (2026-06-05)


### Features

* **tools:** end-to-end VIO noise-&gt;robustness demo (synthetic source) ([#290](https://github.com/branes-ai/cortex/issues/290)) ([047defa](https://github.com/branes-ai/cortex/commit/047defaca0fe8cf60eb791053647c1803009ccf2))
* **tools:** wire S1 (initialization) stage-probe ([#286](https://github.com/branes-ai/cortex/issues/286)) ([22c8b71](https://github.com/branes-ai/cortex/commit/22c8b713f655180a2f2871a98e83599a635b5cb7))

## [0.35.0](https://github.com/branes-ai/cortex/compare/v0.34.0...v0.35.0) (2026-06-04)


### Features

* **tools:** add VIO stage-probe utilities — S0 sensor-model + S2 propagation ([#283](https://github.com/branes-ai/cortex/issues/283)) ([785e875](https://github.com/branes-ai/cortex/commit/785e87575f2a34121db17ad60ce2fd5b97cd77fa))

## [0.34.0](https://github.com/branes-ai/cortex/compare/v0.33.0...v0.34.0) (2026-06-03)


### Features

* **sdk:** Q/R consistency-sweep knobs + [#212](https://github.com/branes-ai/cortex/issues/212) root-cause diagnosis ([#279](https://github.com/branes-ai/cortex/issues/279)) ([b718e5e](https://github.com/branes-ai/cortex/commit/b718e5e92cc177f06ae1de391824777388d12788))

## [0.33.0](https://github.com/branes-ai/cortex/compare/v0.32.0...v0.33.0) (2026-06-03)


### Features

* **eval:** add IMU Allan-variance noise characterization ([#268](https://github.com/branes-ai/cortex/issues/268)) ([#276](https://github.com/branes-ai/cortex/issues/276)) ([4eafd31](https://github.com/branes-ai/cortex/commit/4eafd316ba2d69fde7bf392af872b4cd443d0a06))


### Documentation

* process documentation ([441810a](https://github.com/branes-ai/cortex/commit/441810a715bb1e2eb4e7a80d106affeece4919d2))

## [0.32.0](https://github.com/branes-ai/cortex/compare/v0.31.0...v0.32.0) (2026-06-02)


### Features

* **eval:** add NEES-against-ground-truth consistency (closes [#264](https://github.com/branes-ai/cortex/issues/264)) ([#273](https://github.com/branes-ai/cortex/issues/273)) ([a93cc17](https://github.com/branes-ai/cortex/commit/a93cc178edf41630179abad8c2c8d7169dfa2359))


### Documentation

* **assessment:** a plain-language tutorial on NEES/NIS filter consistency ([98c1f58](https://github.com/branes-ai/cortex/commit/98c1f586b71d23bda384bf7ed30a4f4f6b216921))
* **assessment:** plain-language NEES/NIS filter-consistency tutorial ([#274](https://github.com/branes-ai/cortex/issues/274)) ([98c1f58](https://github.com/branes-ai/cortex/commit/98c1f586b71d23bda384bf7ed30a4f4f6b216921))

## [0.31.0](https://github.com/branes-ai/cortex/compare/v0.30.2...v0.31.0) (2026-06-02)


### Features

* **eval:** add NEES/NIS filter-consistency statistics ([9dc9a1b](https://github.com/branes-ai/cortex/commit/9dc9a1bd5ce278bd2a998951129fe3507466ec1d))
* **eval:** NEES/NIS filter-consistency statistics ([#264](https://github.com/branes-ai/cortex/issues/264)) ([#271](https://github.com/branes-ai/cortex/issues/271)) ([9dc9a1b](https://github.com/branes-ai/cortex/commit/9dc9a1bd5ce278bd2a998951129fe3507466ec1d))
* **sdk:** surface per-update NIS for filter-consistency monitoring ([149c127](https://github.com/branes-ai/cortex/commit/149c127c1d4254ea17d64abf684847466ddffde1))
* **sdk:** surface per-update NIS for filter-consistency monitoring ([#264](https://github.com/branes-ai/cortex/issues/264)) ([#272](https://github.com/branes-ai/cortex/issues/272)) ([149c127](https://github.com/branes-ai/cortex/commit/149c127c1d4254ea17d64abf684847466ddffde1))


### Documentation

* **assessment:** VIO diagnostic methodology + instrumentation build list ([#269](https://github.com/branes-ai/cortex/issues/269)) ([b2bf23e](https://github.com/branes-ai/cortex/commit/b2bf23ea0cf4923750f61bda98fc39fbae97f1b3))

## [0.30.2](https://github.com/branes-ai/cortex/compare/v0.30.1...v0.30.2) (2026-06-02)


### Bug Fixes

* **test:** make the vio_latency budget informational on CI ([#240](https://github.com/branes-ai/cortex/issues/240)) ([#259](https://github.com/branes-ai/cortex/issues/259)) ([9edb5ed](https://github.com/branes-ai/cortex/commit/9edb5edf179afb8c294a1c981ffe9484a54c7a79))

## [0.30.1](https://github.com/branes-ai/cortex/compare/v0.30.0...v0.30.1) (2026-06-02)


### Maintenance

* **main:** release 0.30.0 ([#257](https://github.com/branes-ai/cortex/issues/257)) ([45344de](https://github.com/branes-ai/cortex/commit/45344de27bef140ec135201102dfa4562534c44b))

## [0.30.0](https://github.com/branes-ai/cortex/compare/v0.29.0...v0.30.0) (2026-06-02)


### Features

* **sdk:** force-dynamic VI-init validation on MH_05 (real-data [#211](https://github.com/branes-ai/cortex/issues/211)) ([#246](https://github.com/branes-ai/cortex/issues/246)) ([f80fa63](https://github.com/branes-ai/cortex/commit/f80fa638b3846cf0210c716b5f289bb3b7dd1adb))


### Bug Fixes

* **docs:** re-anchor EuRoC gate extraction (unbreak Deploy Documentation) ([#256](https://github.com/branes-ai/cortex/issues/256)) ([7021cd6](https://github.com/branes-ai/cortex/commit/7021cd64ac5899e541356fb34f9d56521d345ff0))
* **docs:** re-anchor the EuRoC gate extraction to the run_euroc_replay call ([7021cd6](https://github.com/branes-ai/cortex/commit/7021cd64ac5899e541356fb34f9d56521d345ff0))
* **sdk:** bootstrap dynamic-init SfM on the widest baseline ([445f1bb](https://github.com/branes-ai/cortex/commit/445f1bbe79a945d3c71d85bc4c4e654528f52da3))
* **sdk:** bootstrap dynamic-init SfM on the widest baseline ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#251](https://github.com/branes-ai/cortex/issues/251)) ([445f1bb](https://github.com/branes-ai/cortex/commit/445f1bbe79a945d3c71d85bc4c4e654528f52da3))
* **sdk:** correct dynamic-init gravity sign — real-data divergence → bounded ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#252](https://github.com/branes-ai/cortex/issues/252)) ([84cc8ba](https://github.com/branes-ai/cortex/commit/84cc8ba458d3dcccd5e96a367559e1676375e53d))
* **sdk:** correct dynamic-init gravity sign + widest-parallax bootstrap ([84cc8ba](https://github.com/branes-ai/cortex/commit/84cc8ba458d3dcccd5e96a367559e1676375e53d))
* **test:** split euroc-moving-start local decls (set -u unbound var) ([#243](https://github.com/branes-ai/cortex/issues/243)) ([4737ebf](https://github.com/branes-ai/cortex/commit/4737ebff64a471bbcc6613002d4cb94f2435735e))


### Documentation

* **test:** explain pyramidal KLT + add the numerical failure-mode taxonomy ([49f77d2](https://github.com/branes-ai/cortex/commit/49f77d230dede5b19c35886e31af9cc39e51ffc5))
* **test:** pyramidal KLT explainer + EuRoC numerical failure-mode taxonomy ([#255](https://github.com/branes-ai/cortex/issues/255)) ([49f77d2](https://github.com/branes-ai/cortex/commit/49f77d230dede5b19c35886e31af9cc39e51ffc5))
* **test:** vio_euroc goal & architecture ([#249](https://github.com/branes-ai/cortex/issues/249)) ([b0b75d2](https://github.com/branes-ai/cortex/commit/b0b75d2a584fa4cf99002c4fc7d44b78f010017a))


### Tests

* **sdk:** dataset-gated MH_05/V2_03 moving-start dynamic-init validation ([#241](https://github.com/branes-ai/cortex/issues/241)) ([b5a0df3](https://github.com/branes-ai/cortex/commit/b5a0df3d4831e3d28cfc4718132c0f778faba09b))
* **sdk:** dynamic-init diagnostics to localize the real-data divergence ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#248](https://github.com/branes-ai/cortex/issues/248)) ([48be50e](https://github.com/branes-ai/cortex/commit/48be50e27d0a5c38d48402382446fa0f9d1570a2))
* **sdk:** instrument dynamic-init attempts ([#247](https://github.com/branes-ai/cortex/issues/247) localization) ([#250](https://github.com/branes-ai/cortex/issues/250)) ([972015d](https://github.com/branes-ai/cortex/commit/972015d0544e8f5b86c99ad0c1f609918bb20ac4))
* **sdk:** instrument dynamic-init attempts to localize why it won't fire ([972015d](https://github.com/branes-ai/cortex/commit/972015d0544e8f5b86c99ad0c1f609918bb20ac4))
* **sdk:** real EuRoC extrinsics — MH_05 moving-start ATE validated (~0.79 m) ([#245](https://github.com/branes-ai/cortex/issues/245)) ([7f35ebb](https://github.com/branes-ai/cortex/commit/7f35ebbe219b9972ae16475e535e9c8028c83fd6))
* **sdk:** score EuRoC ATE from initialization onward ([#253](https://github.com/branes-ai/cortex/issues/253)) ([8304d67](https://github.com/branes-ai/cortex/commit/8304d67105f605a4a137f602a94e3f80ad64986b))
* **sdk:** score EuRoC ATE only from initialization onward ([8304d67](https://github.com/branes-ai/cortex/commit/8304d67105f605a4a137f602a94e3f80ad64986b))

## [0.29.0](https://github.com/branes-ai/cortex/compare/v0.28.0...v0.29.0) (2026-06-01)


### Features

* **sdk:** wire dynamic VI init into MsckfBackend (InitMethod::Dynamic) ([#238](https://github.com/branes-ai/cortex/issues/238)) ([108cf02](https://github.com/branes-ai/cortex/commit/108cf02bf8f1148e51505a96e9c52f45c9c64416))

## [0.28.0](https://github.com/branes-ai/cortex/compare/v0.27.0...v0.28.0) (2026-06-01)


### Features

* **sdk:** perspective-n-point resectioning via DLT + RANSAC ([#236](https://github.com/branes-ai/cortex/issues/236)) ([fb3370a](https://github.com/branes-ai/cortex/commit/fb3370aee8434c061c2d66268d83e615b94d5d36))

## [0.27.0](https://github.com/branes-ai/cortex/compare/v0.26.0...v0.27.0) (2026-05-31)


### Features

* **sdk:** resolve metric scale in ImuInitializer::try_dynamic ([#233](https://github.com/branes-ai/cortex/issues/233)) ([0df58a0](https://github.com/branes-ai/cortex/commit/0df58a0680fb0db2c92a81b048dfd1b437ee1860))

## [0.26.0](https://github.com/branes-ai/cortex/compare/v0.25.3...v0.26.0) (2026-05-31)


### Features

* **sdk:** two-view SfM bootstrap (essential matrix + RANSAC) ([#231](https://github.com/branes-ai/cortex/issues/231)) ([1900a16](https://github.com/branes-ai/cortex/commit/1900a168f2d0c08ff1054bee6fb14f43a314f1c7))

## [0.25.3](https://github.com/branes-ai/cortex/compare/v0.25.2...v0.25.3) (2026-05-31)


### Bug Fixes

* **build:** link the Rust core in MSVC Debug by pinning the release CRT ([#226](https://github.com/branes-ai/cortex/issues/226)) ([59075af](https://github.com/branes-ai/cortex/commit/59075af8ba67f7d8c5bef2d18acd6148e03c4a28))

## [0.25.2](https://github.com/branes-ai/cortex/compare/v0.25.1...v0.25.2) (2026-05-30)


### Bug Fixes

* **docs:** point the benchmark generator at the moved euroc test ([#223](https://github.com/branes-ai/cortex/issues/223)) ([41bd61b](https://github.com/branes-ai/cortex/commit/41bd61beb5c23dc0f1867bf63e1792ad9fb3ade0))


### Code Refactoring

* **test:** drop redundant layer/_test segments from test filenames ([#225](https://github.com/branes-ai/cortex/issues/225)) ([2ff7cd3](https://github.com/branes-ai/cortex/commit/2ff7cd356d9e0752e1e0dd71979e52a6bcccfcec))

## [0.25.1](https://github.com/branes-ai/cortex/compare/v0.25.0...v0.25.1) (2026-05-30)


### Build System

* **test:** generate test targets from a compile_all glob macro ([#220](https://github.com/branes-ai/cortex/issues/220)) ([33c6126](https://github.com/branes-ai/cortex/commit/33c6126921d96e24f41b6dd35c775d0f9915b4b3))


### Continuous Integration

* **windows:** build the MSVC job via Ninja + sccache ([#222](https://github.com/branes-ai/cortex/issues/222)) ([2d59380](https://github.com/branes-ai/cortex/commit/2d593809cf22a2e3d41ead85d598afb6a03172dd))

## [0.25.0](https://github.com/branes-ai/cortex/compare/v0.24.0...v0.25.0) (2026-05-30)


### Features

* **sdk:** square-root covariance MSCKF backend ([#218](https://github.com/branes-ai/cortex/issues/218)) ([fe8014b](https://github.com/branes-ai/cortex/commit/fe8014b70754680de9b912e74138cfed8f11af42))

## [0.24.0](https://github.com/branes-ai/cortex/compare/v0.23.1...v0.24.0) (2026-05-30)


### Features

* **ci:** add a Windows MSVC build + test target ([#216](https://github.com/branes-ai/cortex/issues/216)) ([5f439b4](https://github.com/branes-ai/cortex/commit/5f439b4c3adc700a0dcdef12051d94a84db09cd7))

## [0.23.1](https://github.com/branes-ai/cortex/compare/v0.23.0...v0.23.1) (2026-05-30)


### Bug Fixes

* **math:** guard degenerate inputs in the Lie-group helpers ([#214](https://github.com/branes-ai/cortex/issues/214)) ([d8034bf](https://github.com/branes-ai/cortex/commit/d8034bf0226bab71e9c2ccd2ae7e2fb314b4969e)), closes [#161](https://github.com/branes-ai/cortex/issues/161)

## [0.23.0](https://github.com/branes-ai/cortex/compare/v0.22.0...v0.23.0) (2026-05-29)


### Features

* **bench:** vio_bench progress output + usage docs ([#208](https://github.com/branes-ai/cortex/issues/208)) ([077751a](https://github.com/branes-ai/cortex/commit/077751ad35185a7d38b303fa40e54fcb8b0c5c2b))


### Bug Fixes

* **sdk:** set EuRoC extrinsics + gravity-align init; surface VIO divergence ([#209](https://github.com/branes-ai/cortex/issues/209)) ([f14a450](https://github.com/branes-ai/cortex/commit/f14a45020934eecffb194dc8cef95871815ab2c7))

## [0.22.0](https://github.com/branes-ai/cortex/compare/v0.21.0...v0.22.0) (2026-05-28)


### Features

* **bench:** graphs energy-model integration (cpu/gpu/kpu-t64) ([#204](https://github.com/branes-ai/cortex/issues/204)) ([0fc9249](https://github.com/branes-ai/cortex/commit/0fc92494b231e7a95ba61fe3a6997c9dfbf129c9))
* **bench:** on-device energy backends — tegrastats + external meter ([#207](https://github.com/branes-ai/cortex/issues/207)) ([e257a25](https://github.com/branes-ai/cortex/commit/e257a25078e5463d72c7d0f68e2e4ae64d420355))

## [0.21.0](https://github.com/branes-ai/cortex/compare/v0.20.3...v0.21.0) (2026-05-28)


### Features

* **bench:** vio_bench — euroc accuracy + latency + rapl energy report ([#201](https://github.com/branes-ai/cortex/issues/201)) ([ba13dec](https://github.com/branes-ai/cortex/commit/ba13decf994ac8997a479c2fc24feb028504d117))

## [0.20.3](https://github.com/branes-ai/cortex/compare/v0.20.2...v0.20.3) (2026-05-27)


### Documentation

* add PR template with the per-PR docs-update rule ([#196](https://github.com/branes-ai/cortex/issues/196)) ([d2a669c](https://github.com/branes-ai/cortex/commit/d2a669c5be03787654f16a8fb027e9b5047e924d))
* pin benchmark gate numbers to their source ([#197](https://github.com/branes-ai/cortex/issues/197)) ([01d43da](https://github.com/branes-ai/cortex/commit/01d43da4e851ca4f9bf2a33b474ddae9d4666ac5))

## [0.20.2](https://github.com/branes-ai/cortex/compare/v0.20.1...v0.20.2) (2026-05-27)


### Documentation

* fix doubled base path on the API reference sidebar link ([#194](https://github.com/branes-ai/cortex/issues/194)) ([857354a](https://github.com/branes-ai/cortex/commit/857354af54b5a98cecf10298c7b8a56dc174e692))

## [0.20.1](https://github.com/branes-ai/cortex/compare/v0.20.0...v0.20.1) (2026-05-27)


### Documentation

* add Starlight + Doxygen documentation site for Phases 0-3 ([#192](https://github.com/branes-ai/cortex/issues/192)) ([9ccfac2](https://github.com/branes-ai/cortex/commit/9ccfac24dceb1385d1fa9ce8829db7d9f0224d57))

## [0.20.0](https://github.com/branes-ai/cortex/compare/v0.19.0...v0.20.0) (2026-05-26)


### Features

* **sdk:** camera updaters with MSCKF null-space projection ([#185](https://github.com/branes-ai/cortex/issues/185)) ([191c4e7](https://github.com/branes-ai/cortex/commit/191c4e71448a5374716ede832374c0eeec33e934))
* **sdk:** IMU initialization (static + dynamic) ([#184](https://github.com/branes-ai/cortex/issues/184)) ([b638566](https://github.com/branes-ai/cortex/commit/b638566edd6d023a0c508d4f1bd5a1713bbc2588))
* **sdk:** MSCKF backend implementation ([#186](https://github.com/branes-ai/cortex/issues/186)) ([02f3948](https://github.com/branes-ai/cortex/commit/02f3948300ad3365497ea329d3e8ef1d9fc4ab97))
* **sdk:** sliding-window optimization backend skeleton ([#189](https://github.com/branes-ai/cortex/issues/189)) ([456a373](https://github.com/branes-ai/cortex/commit/456a373d1e8d803f641a3fbfbecae96e1eb4ac98))
* **sdk:** state vector + propagator + state-helper (MSCKF core) ([#182](https://github.com/branes-ai/cortex/issues/182)) ([0e78154](https://github.com/branes-ai/cortex/commit/0e78154e8d284d1649a35eedf69fe34af5b92798))
* **sdk:** top-level vio estimator API ([#188](https://github.com/branes-ai/cortex/issues/188)) ([c068bd9](https://github.com/branes-ai/cortex/commit/c068bd9b76be2ae356d79413266dc907286038ed))


### Tests

* **sdk:** euroc replay harness + ATE/RPE benchmark ([#190](https://github.com/branes-ai/cortex/issues/190)) ([577e201](https://github.com/branes-ai/cortex/commit/577e201af31dcdad55ba00577880aa25940c9da7))
* **sdk:** latency budget enforcement (per-frame upper bound) ([#191](https://github.com/branes-ai/cortex/issues/191)) ([8dadc96](https://github.com/branes-ai/cortex/commit/8dadc9646bf6eccba940d02a2eb2c002edc2ad18))

## [0.19.0](https://github.com/branes-ai/cortex/compare/v0.18.0...v0.19.0) (2026-05-25)


### Features

* **sdk:** feature representations + MSCKF null-space projection ([#180](https://github.com/branes-ai/cortex/issues/180)) ([58bc198](https://github.com/branes-ai/cortex/commit/58bc198fd5be6a69569735a060a20422a2fc1528))

## [0.18.0](https://github.com/branes-ai/cortex/compare/v0.17.0...v0.18.0) (2026-05-25)


### Features

* **sdk:** imu preintegration (closed-form, clean-room) ([#178](https://github.com/branes-ai/cortex/issues/178)) ([355661b](https://github.com/branes-ai/cortex/commit/355661b64d4d6d2ad3080b60fde8cb11bfaad3c9))

## [0.17.0](https://github.com/branes-ai/cortex/compare/v0.16.0...v0.17.0) (2026-05-25)


### Features

* **cv:** sub-pixel pyramidal KLT feature tracker ([#176](https://github.com/branes-ai/cortex/issues/176)) ([9a54aec](https://github.com/branes-ai/cortex/commit/9a54aec24bb0d0043a1d744a14d5135fb76d1105))

## [0.16.0](https://github.com/branes-ai/cortex/compare/v0.15.0...v0.16.0) (2026-05-24)


### Features

* **cv:** FAST-9 corner detector with non-maximum suppression ([#174](https://github.com/branes-ai/cortex/issues/174)) ([53c8850](https://github.com/branes-ai/cortex/commit/53c885060e355f35c39fd665e4e7009629996f21))

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
