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
