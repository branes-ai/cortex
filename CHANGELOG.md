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
## [0.83.7](https://github.com/branes-ai/cortex/compare/v0.83.6...v0.83.7) (2026-11-02)


### Tests

* **sim:** assert the synthetic world's imu integrates to its ground truth ([#567](https://github.com/branes-ai/cortex/issues/567)) ([fd7d28a](https://github.com/branes-ai/cortex/commit/fd7d28a4c5d84eb907da60e0579e5f6526110012)), closes [#560](https://github.com/branes-ai/cortex/issues/560)

## [0.83.6](https://github.com/branes-ai/cortex/compare/v0.83.5...v0.83.6) (2026-11-02)


### Documentation

* **vio:** draw the coordinate-frames and s1 alignment diagrams ([#564](https://github.com/branes-ai/cortex/issues/564)) ([e743b19](https://github.com/branes-ai/cortex/commit/e743b190293127c42f17c40574598083a2c55f6b))

## [0.83.5](https://github.com/branes-ai/cortex/compare/v0.83.4...v0.83.5) (2026-11-01)


### Build System

* docs-kit v0.6.0 for the docs site ([#558](https://github.com/branes-ai/cortex/issues/558)) ([9d6f237](https://github.com/branes-ai/cortex/commit/9d6f2375bbb4be3e566a50f1fcb287090d4d2b12))


### Documentation

* **adr:** add the third-party data rule; quote EuRoC's calibration as facts ([#559](https://github.com/branes-ai/cortex/issues/559)) ([d68ce87](https://github.com/branes-ai/cortex/commit/d68ce874474f1a93597cf7bc4fe1c7e9d0649b70))
* **vio:** add the vio from first principles section from the [#552](https://github.com/branes-ai/cortex/issues/552) storyline ([#562](https://github.com/branes-ai/cortex/issues/562)) ([23b88a1](https://github.com/branes-ai/cortex/commit/23b88a11bb263ab1730d7ca2cbec3e505fea5c6a))

## [0.83.4](https://github.com/branes-ai/cortex/compare/v0.83.3...v0.83.4) (2026-11-01)


### Bug Fixes

* **sdk:** add a matched-noise s10 control; it exposes the body-frame yaw leak ([#555](https://github.com/branes-ai/cortex/issues/555)) ([18d6074](https://github.com/branes-ai/cortex/commit/18d607461e1a6c9bb93077dccd70d3f06e2f1b57))


### Documentation

* **adr:** align the no-GPL policy with the other repos' ADRs ([#556](https://github.com/branes-ai/cortex/issues/556)) ([7a84999](https://github.com/branes-ai/cortex/commit/7a84999b5c580d59687997fd667f99fad0a03c94))

## [0.83.3](https://github.com/branes-ai/cortex/compare/v0.83.2...v0.83.3) (2026-10-31)


### Bug Fixes

* **sdk:** refine r-iekf triangulation with gauss-newton; slow onset was over-confident ([#550](https://github.com/branes-ai/cortex/issues/550)) ([ac39f0e](https://github.com/branes-ai/cortex/commit/ac39f0e3c2bd169e8f72cc08bcd69cbdf07e312d))


### Build System

* docs-kit v0.4.1 for the docs site ([#553](https://github.com/branes-ai/cortex/issues/553)) ([a29bb1c](https://github.com/branes-ai/cortex/commit/a29bb1c1f36a91ba00a83c1c7014640f03894f70))

## [0.83.2](https://github.com/branes-ai/cortex/compare/v0.83.1...v0.83.2) (2026-10-31)


### Bug Fixes

* **sdk:** r-iekf test world stepped its velocity; right-invariant process noise ([#547](https://github.com/branes-ai/cortex/issues/547)) ([3b7c8dc](https://github.com/branes-ai/cortex/commit/3b7c8dc25b1b284b5d97afbfdcd86d6def454c43))

## [0.83.1](https://github.com/branes-ai/cortex/compare/v0.83.0...v0.83.1) (2026-10-31)


### Continuous Integration

* lint the PR title as well as its commits ([#542](https://github.com/branes-ai/cortex/issues/542)) ([df28ead](https://github.com/branes-ai/cortex/commit/df28eade1a10ead4ae2513d457c3df49dafbb6ff))


### Documentation

* merge the engineering notes into the site, one decision each ([#545](https://github.com/branes-ai/cortex/issues/545)) ([5758bcd](https://github.com/branes-ai/cortex/commit/5758bcdef262be1369597278bebfc5919947a805))

## [0.83.0](https://github.com/branes-ai/cortex/compare/v0.82.2...v0.83.0) (2026-10-31)


### Features

* **docs-site:** generate the site from docs/, with docs-kit ([#541](https://github.com/branes-ai/cortex/issues/541)) ([8cc9011](https://github.com/branes-ai/cortex/commit/8cc901107fff2e01d8c1f7ad25986daeb7c3eab9))


### Continuous Integration

* give each run on main a concurrency group of its own ([#539](https://github.com/branes-ai/cortex/issues/539)) ([e76a1d1](https://github.com/branes-ai/cortex/commit/e76a1d1cc11673f0e29d8846ded03ca66a2844fc))

## [0.82.2](https://github.com/branes-ai/cortex/compare/v0.82.1...v0.82.2) (2026-10-31)


### Tests

* sequence the invariant test's imu noise draws; run it under clang ([#437](https://github.com/branes-ai/cortex/issues/437)) ([#538](https://github.com/branes-ai/cortex/issues/538)) ([b15498b](https://github.com/branes-ai/cortex/commit/b15498bb0c3ecd2bb8b885b1bd906a579e3e1fca))

## [0.82.1](https://github.com/branes-ai/cortex/compare/v0.82.0...v0.82.1) (2026-10-31)


### Bug Fixes

* **sdk:** square-root update reads past a tall factor; vio_pipeline --covariance ([#533](https://github.com/branes-ai/cortex/issues/533)) ([#535](https://github.com/branes-ai/cortex/issues/535)) ([fd38b2b](https://github.com/branes-ai/cortex/commit/fd38b2bf49249f3041341eb9cad9cdbfdb51fa44))

## [0.82.0](https://github.com/branes-ai/cortex/compare/v0.81.0...v0.82.0) (2026-10-30)


### Features

* per-stage type presets, a separate ekf update type, euroc nees runs ([#450](https://github.com/branes-ai/cortex/issues/450)) ([#532](https://github.com/branes-ai/cortex/issues/532)) ([f99aff7](https://github.com/branes-ai/cortex/commit/f99aff7d4407c2c9ca0f426e1fd993d11a89c3e3))

## [0.81.0](https://github.com/branes-ai/cortex/compare/v0.80.0...v0.81.0) (2026-10-30)


### Features

* **sdk:** per-stage arithmetic types on the msckf backend ([#450](https://github.com/branes-ai/cortex/issues/450)) ([#530](https://github.com/branes-ai/cortex/issues/530)) ([eca8d19](https://github.com/branes-ai/cortex/commit/eca8d19026b8b09ee34a018dc3e7c062451f40a2))

## [0.80.0](https://github.com/branes-ai/cortex/compare/v0.79.5...v0.80.0) (2026-10-29)


### Features

* **tools:** per-stage precision characterization in every stage bench ([#450](https://github.com/branes-ai/cortex/issues/450)) ([#527](https://github.com/branes-ai/cortex/issues/527)) ([989f185](https://github.com/branes-ai/cortex/commit/989f185e5b7d1457f6e6ac9206e0e3111ba1b493))

## [0.79.5](https://github.com/branes-ai/cortex/compare/v0.79.4...v0.79.5) (2026-10-29)


### Bug Fixes

* **sdk:** estimator runs end to end in float, posit32 and long double ([#449](https://github.com/branes-ai/cortex/issues/449)) ([#524](https://github.com/branes-ai/cortex/issues/524)) ([0c9e771](https://github.com/branes-ai/cortex/commit/0c9e771c4f90710daaa505cb4c683f0a6b5d9a24))


### Documentation

* publish the plans and adrs on the architecture site ([#525](https://github.com/branes-ai/cortex/issues/525)) ([9353187](https://github.com/branes-ai/cortex/commit/9353187143794f73a46e5e2126ecc02025ff23b4))

## [0.79.4](https://github.com/branes-ai/cortex/compare/v0.79.3...v0.79.4) (2026-10-28)


### Documentation

* the five-tier stack diagram, with anatomy and scout-drone ([#522](https://github.com/branes-ai/cortex/issues/522)) ([8b98740](https://github.com/branes-ai/cortex/commit/8b98740be742a90f49937cbd724e21f17778faf3))

## [0.79.3](https://github.com/branes-ai/cortex/compare/v0.79.2...v0.79.3) (2026-10-28)


### Code Refactoring

* **sdk:** sfm eigen family on mtl5 5.20 eigen_symmetric_jacobi ([#448](https://github.com/branes-ai/cortex/issues/448)) ([#520](https://github.com/branes-ai/cortex/issues/520)) ([ad44ab2](https://github.com/branes-ai/cortex/commit/ad44ab2b8c0f023971b6f5879e34fe8389e42d67))

## [0.79.2](https://github.com/branes-ai/cortex/compare/v0.79.1...v0.79.2) (2026-10-25)


### Build System

* **deps:** bump MTL5 to v5.20.0 and Universal to v5.11.0 ([#518](https://github.com/branes-ai/cortex/issues/518)) ([e891e9d](https://github.com/branes-ai/cortex/commit/e891e9d1b3f682e5f426b3cad65e7b8d94f63e33)), closes [#517](https://github.com/branes-ai/cortex/issues/517)

## [0.79.1](https://github.com/branes-ai/cortex/compare/v0.79.0...v0.79.1) (2026-10-24)


### Documentation

* **readme:** four-tier stack diagram ([#514](https://github.com/branes-ai/cortex/issues/514)) ([700ed82](https://github.com/branes-ai/cortex/commit/700ed82548a03b427609c51ae0f68cc598db6215))
* **readme:** qualify hardware targets and explain the stack diagram arrows ([#516](https://github.com/branes-ai/cortex/issues/516)) ([4870896](https://github.com/branes-ai/cortex/commit/4870896cc7c07e2da5dd818e86a0c7eb465363bb))

## [0.79.0](https://github.com/branes-ai/cortex/compare/v0.78.0...v0.79.0) (2026-10-12)


### Features

* **math:** small-angle threshold and tolerances scale with the arithmetic type ([#512](https://github.com/branes-ai/cortex/issues/512)) ([3d556b0](https://github.com/branes-ai/cortex/commit/3d556b0e3084e505b24baf0cc92d8a9cccf132ea))

## [0.78.0](https://github.com/branes-ai/cortex/compare/v0.77.0...v0.78.0) (2026-10-12)


### Features

* **tools:** strict_double bench type audits genericity bit for bit ([#510](https://github.com/branes-ai/cortex/issues/510)) ([ced6345](https://github.com/branes-ai/cortex/commit/ced6345615c628fced9a27f69163ae60c677439a))

## [0.77.0](https://github.com/branes-ai/cortex/compare/v0.76.1...v0.77.0) (2026-10-12)


### Features

* **tools:** s2 bench invariant-propagator and higher-order phi variants ([#509](https://github.com/branes-ai/cortex/issues/509)) ([ef73696](https://github.com/branes-ai/cortex/commit/ef73696393d357fcac3f9adbdc29154fce5bb8cc)), closes [#471](https://github.com/branes-ai/cortex/issues/471)


### Code Refactoring

* **sdk:** square-root form solves and gain product on mtl5 trsm/mult ([#507](https://github.com/branes-ai/cortex/issues/507)) ([dfb38b2](https://github.com/branes-ai/cortex/commit/dfb38b2b8b36ac87eaf9730c84d063f12d06801b))

## [0.76.1](https://github.com/branes-ai/cortex/compare/v0.76.0...v0.76.1) (2026-10-12)


### Code Refactoring

* **sdk:** qr and msckf null space on mtl5 5.15 textbook reflector ([#505](https://github.com/branes-ai/cortex/issues/505)) ([530a918](https://github.com/branes-ai/cortex/commit/530a9181cab0217d98c00ea26a59f3e4f5179282))

## [0.76.0](https://github.com/branes-ai/cortex/compare/v0.75.3...v0.76.0) (2026-10-12)


### Features

* **tools:** s1 bench sfm front-end variants on observation fixtures ([#504](https://github.com/branes-ai/cortex/issues/504)) ([f776280](https://github.com/branes-ai/cortex/commit/f776280f4d23611030790bdfa509d3aaee40f07f))


### Bug Fixes

* **sdk:** sfm jacobi_eigh stops at convergence instead of after 100 sweeps ([#498](https://github.com/branes-ai/cortex/issues/498)) ([c20d4c8](https://github.com/branes-ai/cortex/commit/c20d4c8aad17d69d02a7948dfc1745460920916b))

## [0.75.3](https://github.com/branes-ai/cortex/compare/v0.75.2...v0.75.3) (2026-10-11)


### Code Refactoring

* **sdk:** dynmat products and cholesky factor on mtl5 5.14 ([#448](https://github.com/branes-ai/cortex/issues/448)) ([#495](https://github.com/branes-ai/cortex/issues/495)) ([69a0443](https://github.com/branes-ai/cortex/commit/69a044336fb15d199c2ad93225f55d3e41df414d))

## [0.75.2](https://github.com/branes-ai/cortex/compare/v0.75.1...v0.75.2) (2026-10-11)


### Code Refactoring

* **sdk:** spd solves on mtl5 trsm; cholesky factor stays local ([#448](https://github.com/branes-ai/cortex/issues/448)) ([#493](https://github.com/branes-ai/cortex/issues/493)) ([d0a0dd5](https://github.com/branes-ai/cortex/commit/d0a0dd53ebb6ed582dea7b73e5e6ad20cb3b9d12))

## [0.75.1](https://github.com/branes-ai/cortex/compare/v0.75.0...v0.75.1) (2026-10-11)


### Tests

* **tools:** equivalence gate for the mtl5 port ([#491](https://github.com/branes-ai/cortex/issues/491)) ([86ba080](https://github.com/branes-ai/cortex/commit/86ba080ad70b127e8a3ab9a151953a287975f951))

## [0.75.0](https://github.com/branes-ai/cortex/compare/v0.74.0...v0.75.0) (2026-10-10)


### Features

* **tools:** composition bench, live assertion and trajectory report for the vio loop ([#489](https://github.com/branes-ai/cortex/issues/489)) ([622c209](https://github.com/branes-ai/cortex/commit/622c209782407d79fb8c833d64455019ab64444d))

## [0.74.0](https://github.com/branes-ai/cortex/compare/v0.73.0...v0.74.0) (2026-10-10)


### Features

* **tools:** capture msckf stage boundaries as replayable bench fixtures ([#486](https://github.com/branes-ai/cortex/issues/486)) ([ebc8571](https://github.com/branes-ai/cortex/commit/ebc8571d36ee31630477d6e689904836a2e0f7f4))

## [0.73.0](https://github.com/branes-ai/cortex/compare/v0.72.0...v0.73.0) (2026-10-10)


### Features

* **test:** s10 online-calibration bench and s9 window-policy variants ([#484](https://github.com/branes-ai/cortex/issues/484)) ([5d31c69](https://github.com/branes-ai/cortex/commit/5d31c692248ef8c122d046a4d53be367933bd4fc))

## [0.72.0](https://github.com/branes-ai/cortex/compare/v0.71.0...v0.72.0) (2026-10-09)


### Features

* **test:** s6 sub-step benches for the msckf update (s6a-s6e) ([#479](https://github.com/branes-ai/cortex/issues/479)) ([bce0d4d](https://github.com/branes-ai/cortex/commit/bce0d4d47856c3eec12ba65bb5da6afa4127de1a))

## [0.71.0](https://github.com/branes-ai/cortex/compare/v0.70.0...v0.71.0) (2026-10-09)


### Features

* **test:** s4 inspector calls the stage; alternative s5 triangulation methods ([#477](https://github.com/branes-ai/cortex/issues/477)) ([c8c855c](https://github.com/branes-ai/cortex/commit/c8c855c038c290eec63311512b139f62227b8217))

## [0.70.0](https://github.com/branes-ai/cortex/compare/v0.69.0...v0.70.0) (2026-10-09)


### Features

* **test:** s4-s5 stage benches for the visual frontend and triangulation ([#473](https://github.com/branes-ai/cortex/issues/473)) ([4fc6d6d](https://github.com/branes-ai/cortex/commit/4fc6d6df29d04dc1d4fcae83d1e05fd6728d7527))

## [0.69.0](https://github.com/branes-ai/cortex/compare/v0.68.0...v0.69.0) (2026-10-09)


### Features

* **test:** s0-s1 stage benches for sensor models and initialization ([#466](https://github.com/branes-ai/cortex/issues/466)) ([d1cfcc2](https://github.com/branes-ai/cortex/commit/d1cfcc2c755adb20d7ae4045702ea125db9855f1))
* **test:** s2-s3 stage benches for imu propagation and state augmentation ([#470](https://github.com/branes-ai/cortex/issues/470)) ([c07fab2](https://github.com/branes-ai/cortex/commit/c07fab2a45e06044ce91c2c77dbff14821d3e338))

## [0.68.0](https://github.com/branes-ai/cortex/compare/v0.67.0...v0.68.0) (2026-10-09)


### Features

* **tools:** vio stage test-bench framework with synthetic, gt-injected, captured fixtures ([#463](https://github.com/branes-ai/cortex/issues/463)) ([eeb4716](https://github.com/branes-ai/cortex/commit/eeb47166750a84b5c626772fb6b8ca1667657f4e))


### Documentation

* **site:** add the epic [#444](https://github.com/branes-ai/cortex/issues/444) divide-and-conquer debugging section ([#464](https://github.com/branes-ai/cortex/issues/464)) ([ca34312](https://github.com/branes-ai/cortex/commit/ca3431224fd1dca5db85fe47602883fb79aa5f8b))

## [0.67.0](https://github.com/branes-ai/cortex/compare/v0.66.4...v0.67.0) (2026-10-08)


### Features

* **sdk,eval:** invariant-check library with arithmetic-scaled tolerances ([#459](https://github.com/branes-ai/cortex/issues/459)) ([f32e58c](https://github.com/branes-ai/cortex/commit/f32e58c7162ac56787124134a056e54f9ae65572))


### Code Refactoring

* **sdk:** expose vio stages as transformations; backend becomes the sequence ([#460](https://github.com/branes-ai/cortex/issues/460)) ([c976b81](https://github.com/branes-ai/cortex/commit/c976b81a542a831c85a29831d0435aa7f72d4360))

## [0.66.4](https://github.com/branes-ai/cortex/compare/v0.66.3...v0.66.4) (2026-10-07)


### Continuous Integration

* harden runners against a stalled ubuntu mirror ([#443](https://github.com/branes-ai/cortex/issues/443)) ([ef8948f](https://github.com/branes-ai/cortex/commit/ef8948fc870143d6e3f65c5eb3dc59cf5cf4cd41))


### Documentation

* complete epic e11 docs, adrs, contributing, and hello_vio ([#441](https://github.com/branes-ai/cortex/issues/441)) ([9c9e324](https://github.com/branes-ai/cortex/commit/9c9e324dd015d513034ecae04c475e7b4fe06df7))

## [0.66.3](https://github.com/branes-ai/cortex/compare/v0.66.2...v0.66.3) (2026-10-07)


### Build System

* **deps:** adopt the stillwater header-only fetch pattern for mtl5 and universal ([#436](https://github.com/branes-ai/cortex/issues/436)) ([4f19828](https://github.com/branes-ai/cortex/commit/4f1982898b2dc31fd3d39b33bef9f6325d78e09d)), closes [#435](https://github.com/branes-ai/cortex/issues/435)


### Continuous Integration

* add a clang sitl build-and-test job ([#438](https://github.com/branes-ai/cortex/issues/438)) ([e7c2e6f](https://github.com/branes-ai/cortex/commit/e7c2e6ff57415a271d1750fcc05dbbe3e898e5c1))

## [0.66.2](https://github.com/branes-ai/cortex/compare/v0.66.1...v0.66.2) (2026-10-07)


### Build System

* require cmake 4.0 as the minimum version ([#434](https://github.com/branes-ai/cortex/issues/434)) ([eefcabf](https://github.com/branes-ai/cortex/commit/eefcabf3460093e58ec37e97b3cab8401d655be3))


### Documentation

* flight controller analysis ([40d3453](https://github.com/branes-ai/cortex/commit/40d34539e86bb13ed10650747a27ba6e40816bec))

## [0.66.1](https://github.com/branes-ai/cortex/compare/v0.66.0...v0.66.1) (2026-07-01)


### Documentation

* **sessions:** record the end-to-end R-IEKF EuRoC harness day ([#431](https://github.com/branes-ai/cortex/issues/431)) ([ca0217c](https://github.com/branes-ai/cortex/commit/ca0217c957a729c9f1febafa9ba380259367fb5c))

## [0.66.0](https://github.com/branes-ai/cortex/compare/v0.65.0...v0.66.0) (2026-07-01)


### Features

* **tools:** drive R-IEKF through the real EuRoC pipeline ([#429](https://github.com/branes-ai/cortex/issues/429)) ([567ee30](https://github.com/branes-ai/cortex/commit/567ee300138204b477b2b77c93533ce06565834c))

## [0.65.0](https://github.com/branes-ai/cortex/compare/v0.64.0...v0.65.0) (2026-06-22)


### Features

* **tools:** obs_inspect — R-IEKF vs standard yaw-leak comparison ([89b77a4](https://github.com/branes-ai/cortex/commit/89b77a44f9db1828775f42990ce7ca65516dca77))
* **tools:** obs_inspect R-IEKF vs standard yaw-leak comparison ([#426](https://github.com/branes-ai/cortex/issues/426)) ([89b77a4](https://github.com/branes-ai/cortex/commit/89b77a44f9db1828775f42990ce7ca65516dca77))


### Documentation

* **pipeline:** record the yaw-leak localization and R-IEKF validation ([#427](https://github.com/branes-ai/cortex/issues/427)) ([e2f4c0c](https://github.com/branes-ai/cortex/commit/e2f4c0cff5172fc304ede3cdf219ef1b42a96f04))

## [0.64.0](https://github.com/branes-ai/cortex/compare/v0.63.1...v0.64.0) (2026-06-21)


### Features

* **tools:** real-data observability / null-space-leak diagnostic ([#424](https://github.com/branes-ai/cortex/issues/424)) ([0316dbb](https://github.com/branes-ai/cortex/commit/0316dbb746499dd1a4b649d4db9450588aa6bd97))

## [0.63.1](https://github.com/branes-ai/cortex/compare/v0.63.0...v0.63.1) (2026-06-21)


### Bug Fixes

* **tools:** correct s6_inspect PSD check and NIS headline ([#423](https://github.com/branes-ai/cortex/issues/423)) ([599143a](https://github.com/branes-ai/cortex/commit/599143a7293ab05da506f2b1d16f9d24e523ae77))


### Documentation

* **pipeline:** record the completed real-data inspector epic ([#371](https://github.com/branes-ai/cortex/issues/371)) ([#421](https://github.com/branes-ai/cortex/issues/421)) ([a01b3a2](https://github.com/branes-ai/cortex/commit/a01b3a2df2ab96fe96140581fda377ab7a2c85bd))

## [0.63.0](https://github.com/branes-ai/cortex/compare/v0.62.0...v0.63.0) (2026-06-19)


### Features

* **tools:** s10 online-calibration inspector — real EuRoC → extrinsic convergence ([#419](https://github.com/branes-ai/cortex/issues/419)) ([e30e94b](https://github.com/branes-ai/cortex/commit/e30e94b1eeb78373165502b84f69083bf8e0de2d)), closes [#384](https://github.com/branes-ai/cortex/issues/384)

## [0.62.0](https://github.com/branes-ai/cortex/compare/v0.61.0...v0.62.0) (2026-06-18)


### Features

* **tools:** s3 augmentation inspector — real EuRoC keyframe → cloned covariance ([#416](https://github.com/branes-ai/cortex/issues/416)) ([d2fcd5d](https://github.com/branes-ai/cortex/commit/d2fcd5d0bbd84c5661f16d6addb2b893ae70fba2))
* **tools:** s9 marginalization inspector — real EuRoC clone drop → reduced covariance ([#418](https://github.com/branes-ai/cortex/issues/418)) ([02098de](https://github.com/branes-ai/cortex/commit/02098deb1f0661bd20f4c8148a807477e2bf3012))

## [0.61.0](https://github.com/branes-ai/cortex/compare/v0.60.2...v0.61.0) (2026-06-17)


### Features

* **tools:** s2 propagation inspector — real EuRoC IMU window → covariance growth & drift ([#414](https://github.com/branes-ai/cortex/issues/414)) ([addef52](https://github.com/branes-ai/cortex/commit/addef5239eee59fbe012be3e0f3000d8dedfcdae))


### Documentation

* wip workload analysis ([f2f9102](https://github.com/branes-ai/cortex/commit/f2f9102ae2ad4fb579482b08c407c44a11616aa1))

## [0.60.2](https://github.com/branes-ai/cortex/compare/v0.60.1...v0.60.2) (2026-06-17)


### Documentation

* **site:** fix un-gated inline math delimiters and escape sequences ([#412](https://github.com/branes-ai/cortex/issues/412)) ([fcf64bc](https://github.com/branes-ai/cortex/commit/fcf64bc62b097cf6e65c2537062c8b5ba15b982c))
* **site:** fix un-gated inline math delimiters and escape sequences on overview, S0, and S1 ([fcf64bc](https://github.com/branes-ai/cortex/commit/fcf64bc62b097cf6e65c2537062c8b5ba15b982c))
* **site:** restore authoritative VIO stage-flow architectural SVG in overview ([a1a2729](https://github.com/branes-ai/cortex/commit/a1a272937ca254975f170b3fa6bac9b6ec921cb6))

## [0.60.1](https://github.com/branes-ai/cortex/compare/v0.60.0...v0.60.1) (2026-06-17)


### Maintenance

* **main:** release 0.60.0 ([#410](https://github.com/branes-ai/cortex/issues/410)) ([bb044a4](https://github.com/branes-ai/cortex/commit/bb044a4ec2329dc8fcd5bcd9145d4749113b6b2e))

## [0.60.0](https://github.com/branes-ai/cortex/compare/v0.59.1...v0.60.0) (2026-06-17)


### Features

* 3D drone path/pose viewer (Phase 0 + Phase 1 prototype) ([#302](https://github.com/branes-ai/cortex/issues/302)) ([7e47ce3](https://github.com/branes-ai/cortex/commit/7e47ce3eeac87bd9159d39f6e2aa04de88f786c5))
* add the S4 forward-backward and S5 parallax outlier gates ([#323](https://github.com/branes-ai/cortex/issues/323)) ([ed125d5](https://github.com/branes-ai/cortex/commit/ed125d5e6cee81fc1a33cb534f0446a271ad6b7c))
* **bench:** graphs energy-model integration (cpu/gpu/kpu-t64) ([#204](https://github.com/branes-ai/cortex/issues/204)) ([9ec7b91](https://github.com/branes-ai/cortex/commit/9ec7b91931e3bed5b499fae299facb00c2030c79))
* **bench:** on-device energy backends — tegrastats + external meter ([#207](https://github.com/branes-ai/cortex/issues/207)) ([dda2ac7](https://github.com/branes-ai/cortex/commit/dda2ac7a6d67976f68c6feecb30462b743bc8db2))
* **bench:** vio_bench — euroc accuracy + latency + rapl energy report ([#201](https://github.com/branes-ai/cortex/issues/201)) ([a5abbcb](https://github.com/branes-ai/cortex/commit/a5abbcb448cfa93a629396e74f7709a5a53fc9d3))
* **bench:** vio_bench progress output + usage docs ([#208](https://github.com/branes-ai/cortex/issues/208)) ([e569a04](https://github.com/branes-ai/cortex/commit/e569a0413f8e41fca6e54d6e2bdc7af5564e2370))
* **ci:** add a Windows MSVC build + test target ([#216](https://github.com/branes-ai/cortex/issues/216)) ([b79fa07](https://github.com/branes-ai/cortex/commit/b79fa07bcc13924a5c7f26f61ec049e370a2206d))
* complete the structural stage pages (S3, S4, S7-S9) ([#319](https://github.com/branes-ai/cortex/issues/319)) ([13be4c6](https://github.com/branes-ai/cortex/commit/13be4c60f20cdd65d4c2abb053f8f02b28531ead))
* **core:** add cxx bridge, build.rs, and lifecycle state machine ([#133](https://github.com/branes-ai/cortex/issues/133)) ([ff45bf9](https://github.com/branes-ai/cortex/commit/ff45bf9a0d2a00ee03c8905161a06817ab45534f))
* **core:** add KPU MemoryProvider stub ([#131](https://github.com/branes-ai/cortex/issues/131)) ([cf519ab](https://github.com/branes-ai/cortex/commit/cf519ab3ba1c5e8ef5eade206280ffb428a57451))
* **core:** add SITL MemoryProvider with shm + heap backends ([7075ac7](https://github.com/branes-ai/cortex/commit/7075ac7d50cc1827f918724b0cdc29d6553db123))
* **core:** define MemoryProvider trait + BufferHandle + MemErr ([#127](https://github.com/branes-ai/cortex/issues/127)) ([fd6a755](https://github.com/branes-ai/cortex/commit/fd6a7558e27798502bc3dfca532e034b58e1823e))
* **core:** SITL MemoryProvider — Linux shm + heap backends ([#129](https://github.com/branes-ai/cortex/issues/129)) ([7075ac7](https://github.com/branes-ai/cortex/commit/7075ac7d50cc1827f918724b0cdc29d6553db123))
* **cv:** FAST-9 corner detector with non-maximum suppression ([#174](https://github.com/branes-ai/cortex/issues/174)) ([905b81f](https://github.com/branes-ai/cortex/commit/905b81f0f94810cab2f745fc3fb5f7b5f791cdb1))
* **cv:** gaussian image pyramid with configurable scale ([#170](https://github.com/branes-ai/cortex/issues/170)) ([3f0704a](https://github.com/branes-ai/cortex/commit/3f0704a65d7d8ba3f5fbb0e6f42d99a8f6105e87))
* **cv:** image container + grayscale PGM/PNG I/O ([#169](https://github.com/branes-ai/cortex/issues/169)) ([6e5966b](https://github.com/branes-ai/cortex/commit/6e5966b85ebfa74904840920dbeea47e84b054c0))
* **cv:** sub-pixel pyramidal KLT feature tracker ([#176](https://github.com/branes-ai/cortex/issues/176)) ([67ec732](https://github.com/branes-ai/cortex/commit/67ec732950e680540702593e0f0f040807d67a27))
* **eval:** add IMU Allan-variance noise characterization ([#268](https://github.com/branes-ai/cortex/issues/268)) ([#276](https://github.com/branes-ai/cortex/issues/276)) ([32b0c4b](https://github.com/branes-ai/cortex/commit/32b0c4b258416c5e9b1613806186144ea21e5478))
* **eval:** add NEES-against-ground-truth consistency (closes [#264](https://github.com/branes-ai/cortex/issues/264)) ([#273](https://github.com/branes-ai/cortex/issues/273)) ([66cc815](https://github.com/branes-ai/cortex/commit/66cc8158b84dccefce1f4dc1a520338a25cee3af))
* **eval:** add NEES/NIS filter-consistency statistics ([4a1e361](https://github.com/branes-ai/cortex/commit/4a1e3615eef26993b48d44a271278170250872f3))
* **eval:** NEES/NIS filter-consistency statistics ([#264](https://github.com/branes-ai/cortex/issues/264)) ([#271](https://github.com/branes-ai/cortex/issues/271)) ([4a1e361](https://github.com/branes-ai/cortex/commit/4a1e3615eef26993b48d44a271278170250872f3))
* landmark cloud + camera frustum in the 3D viewer ([#303](https://github.com/branes-ai/cortex/issues/303)) ([86fc1c7](https://github.com/branes-ai/cortex/commit/86fc1c7685209bfa9fb4d441eb2f09ab20143aed))
* **math:** add SE2(3) extended-pose Lie group — the R-IEKF state manifold ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#353](https://github.com/branes-ai/cortex/issues/353)) ([866fa8d](https://github.com/branes-ai/cortex/commit/866fa8d8d04f1ef071b08358f41bb84eaf1a624c))
* **math:** arithmetic-type plumbing for MTL5 over Universal types ([#143](https://github.com/branes-ai/cortex/issues/143)) ([c98df33](https://github.com/branes-ai/cortex/commit/c98df330cc823e3ebd78119c35203bc75a6ac4fb)), closes [#25](https://github.com/branes-ai/cortex/issues/25)
* **math:** camera models — pinhole+radtan, fisheye, unified omni ([#172](https://github.com/branes-ai/cortex/issues/172)) ([79a4b52](https://github.com/branes-ai/cortex/commit/79a4b522f35f41f7e38630c2afb362743a0762f8))
* **math:** direct sparse Cholesky/LDLT for SPD normal equations ([#151](https://github.com/branes-ai/cortex/issues/151)) ([46687ff](https://github.com/branes-ai/cortex/commit/46687ffae48de72c231a940b413e15e2328c6be7)), closes [#28](https://github.com/branes-ai/cortex/issues/28)
* **math:** header-only Lie groups SO3, SE3, Sim3 ([#147](https://github.com/branes-ai/cortex/issues/147)) ([6efe30f](https://github.com/branes-ai/cortex/commit/6efe30fd35529d0b4a0b92ba8c85c2fd37d97532)), closes [#29](https://github.com/branes-ai/cortex/issues/29)
* **math:** Krylov solvers (CG, GMRES, BiCGSTAB) over MTL5 ITL ([#149](https://github.com/branes-ai/cortex/issues/149)) ([d0c3a93](https://github.com/branes-ai/cortex/commit/d0c3a9330f88ba19d13a1f77a1c3b00a5e53cb2e))
* **math:** non-linear least squares solvers (GN, LM, Dogleg) ([aac636c](https://github.com/branes-ai/cortex/commit/aac636cca3a4cefce5b2bf6b48174b3941d4cd84)), closes [#30](https://github.com/branes-ai/cortex/issues/30)
* **math:** non-linear least squares solvers (gn, lm, dogleg) ([#153](https://github.com/branes-ai/cortex/issues/153)) ([aac636c](https://github.com/branes-ai/cortex/commit/aac636cca3a4cefce5b2bf6b48174b3941d4cd84))
* **math:** sparse storage views (CSR, CSC, COO) over caller buffers ([#145](https://github.com/branes-ai/cortex/issues/145)) ([957e8ca](https://github.com/branes-ai/cortex/commit/957e8ca20c62c198d81a1ef4ec32d727f7a243b6)), closes [#26](https://github.com/branes-ai/cortex/issues/26)
* **math:** tensor views over std::span with stride/shape metadata ([081e66f](https://github.com/branes-ai/cortex/commit/081e66f334b767e355122079805629250140e62e)), closes [#24](https://github.com/branes-ai/cortex/issues/24)
* **math:** TensorView&lt;T,Rank&gt; over std::span with stride/shape metadata ([#141](https://github.com/branes-ai/cortex/issues/141)) ([081e66f](https://github.com/branes-ai/cortex/commit/081e66f334b767e355122079805629250140e62e))
* **math:** thin Krylov solver wrappers (cg, gmres, bicgstab) over MTL5 ITL ([d0c3a93](https://github.com/branes-ai/cortex/commit/d0c3a9330f88ba19d13a1f77a1c3b00a5e53cb2e)), closes [#27](https://github.com/branes-ai/cortex/issues/27)
* S5 feature-triangulation probe + stage page (parallax gate, quantified) ([#313](https://github.com/branes-ai/cortex/issues/313)) ([cb2c938](https://github.com/branes-ai/cortex/commit/cb2c938fe4e7d2608abe719c6abbedb7076328da))
* **sdk:** add 6-DoF rigid camera extrinsics to MsckfInvariantBackend ([efb48c5](https://github.com/branes-ai/cortex/commit/efb48c5d123d24a4946ddf0a83f72e85468e7067))
* **sdk:** add 6-DoF rigid camera extrinsics to MsckfInvariantBackend ([#396](https://github.com/branes-ai/cortex/issues/396)) ([586081d](https://github.com/branes-ai/cortex/commit/586081de6f1c03bbfee207936a3e9baad246d32e))
* **sdk:** add S10 calibration-uncertainty R-term; measure it end-to-end on V2_03 ([#328](https://github.com/branes-ai/cortex/issues/328)) ([670defd](https://github.com/branes-ai/cortex/commit/670defda056f5d8dcc0d6dda69a26f3753f3df6b))
* **sdk:** add the R-IEKF invariant camera measurement update ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#356](https://github.com/branes-ai/cortex/issues/356)) ([03d34ae](https://github.com/branes-ai/cortex/commit/03d34ae0a7fa1012bb91ca879de1c5da4bfa2134))
* **sdk:** add the R-IEKF invariant IMU propagation on SE2(3) ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#354](https://github.com/branes-ai/cortex/issues/354)) ([4b41726](https://github.com/branes-ai/cortex/commit/4b417268efba64020fda3e3165af257ceb8d6933))
* **sdk:** camera updaters with MSCKF null-space projection ([#185](https://github.com/branes-ai/cortex/issues/185)) ([f16cde0](https://github.com/branes-ai/cortex/commit/f16cde06e542674cb3e626d05ff34cf52c6957d2))
* **sdk:** feature representations + MSCKF null-space projection ([#180](https://github.com/branes-ai/cortex/issues/180)) ([deaa092](https://github.com/branes-ai/cortex/commit/deaa092724248fd7e9d3886498c213de564b7648))
* **sdk:** force-dynamic VI-init validation on MH_05 (real-data [#211](https://github.com/branes-ai/cortex/issues/211)) ([#246](https://github.com/branes-ai/cortex/issues/246)) ([d736d4f](https://github.com/branes-ai/cortex/commit/d736d4f6680814a800eff92120adeeb421173953))
* **sdk:** implement Phase C 6-DoF online extrinsic calibration for MsckfInvariantBackend ([9094c64](https://github.com/branes-ai/cortex/commit/9094c64a28fb3934acfc366f720f096c0953ffcf))
* **sdk:** IMU initialization (static + dynamic) ([#184](https://github.com/branes-ai/cortex/issues/184)) ([6db837d](https://github.com/branes-ai/cortex/commit/6db837d403c07bf34f917ac39e5fe95356c86c0b))
* **sdk:** imu preintegration (closed-form, clean-room) ([#178](https://github.com/branes-ai/cortex/issues/178)) ([6c5e1b1](https://github.com/branes-ai/cortex/commit/6c5e1b1531a9b89db7b74d5e698b11ad0c720a59))
* **sdk:** MSCKF backend implementation ([#186](https://github.com/branes-ai/cortex/issues/186)) ([f02bc78](https://github.com/branes-ai/cortex/commit/f02bc78358572e92345b378295d843c5a6211cba))
* **sdk:** observability null-space probe — localizes the [#212](https://github.com/branes-ai/cortex/issues/212) NEES leak to yaw ([#337](https://github.com/branes-ai/cortex/issues/337)) ([#338](https://github.com/branes-ai/cortex/issues/338)) ([5e5c3a1](https://github.com/branes-ai/cortex/commit/5e5c3a1e1ad610612dc06c2974de009a43720dfa))
* **sdk:** online extrinsic calibration (S10-as-state), mechanism + synthetic proof ([#332](https://github.com/branes-ai/cortex/issues/332)) ([#333](https://github.com/branes-ai/cortex/issues/333)) ([24cb8b3](https://github.com/branes-ai/cortex/commit/24cb8b3fc086458d47ba896fbebabcfc5356414b))
* **sdk:** perspective-n-point resectioning via DLT + RANSAC ([#236](https://github.com/branes-ai/cortex/issues/236)) ([542456c](https://github.com/branes-ai/cortex/commit/542456c186f514fcc7751862381d8ae5736efc88))
* **sdk:** Phase C - 6-DoF online extrinsic calibration for MsckfInvariantBackend ([#400](https://github.com/branes-ai/cortex/issues/400)) ([9094c64](https://github.com/branes-ai/cortex/commit/9094c64a28fb3934acfc366f720f096c0953ffcf))
* **sdk:** prove the right-invariant propagation Φ is state-independent (R-IEKF Phase A, [#348](https://github.com/branes-ai/cortex/issues/348)) ([#351](https://github.com/branes-ai/cortex/issues/351)) ([8c48e06](https://github.com/branes-ai/cortex/commit/8c48e060d367bfcbb5d497c7150453a959e41366))
* **sdk:** Q/R consistency-sweep knobs + [#212](https://github.com/branes-ai/cortex/issues/212) root-cause diagnosis ([#279](https://github.com/branes-ai/cortex/issues/279)) ([910f1c6](https://github.com/branes-ai/cortex/commit/910f1c6323a75bbc361f24a45693aecbb8bac997))
* **sdk:** R-IEKF Phase-A gate — invariant parameterization flattens the yaw leak ([#348](https://github.com/branes-ai/cortex/issues/348)) ([#349](https://github.com/branes-ai/cortex/issues/349)) ([5350f9e](https://github.com/branes-ai/cortex/commit/5350f9ec77daa70fe202012ccf38940b9d98f68c))
* **sdk:** R-IEKF VIO backend adapter + continuous-update divergence localization ([#365](https://github.com/branes-ai/cortex/issues/365)) ([#367](https://github.com/branes-ai/cortex/issues/367)) ([51ca579](https://github.com/branes-ai/cortex/commit/51ca579834aa30e6126660eb3207acb7c43d4600))
* **sdk:** resolve metric scale in ImuInitializer::try_dynamic ([#233](https://github.com/branes-ai/cortex/issues/233)) ([89b4c1d](https://github.com/branes-ai/cortex/commit/89b4c1d718b14099e7d93d74bc5ab8595f842c1a))
* **sdk:** S6 update probe; lock S3/S5/S6/S9 contracts by measurement ([#325](https://github.com/branes-ai/cortex/issues/325)) ([bb8b591](https://github.com/branes-ai/cortex/commit/bb8b591438aaff34291ce3fe6292f5369e54c23e))
* **sdk:** sliding-window optimization backend skeleton ([#189](https://github.com/branes-ai/cortex/issues/189)) ([45b357f](https://github.com/branes-ai/cortex/commit/45b357fa518baa6bbaa58818e1192329ee5c183d))
* **sdk:** square-root covariance MSCKF backend ([#218](https://github.com/branes-ai/cortex/issues/218)) ([b9a4478](https://github.com/branes-ai/cortex/commit/b9a4478692456dd0b60634314ef9feb5ecc945bd))
* **sdk:** state vector + propagator + state-helper (MSCKF core) ([#182](https://github.com/branes-ai/cortex/issues/182)) ([2e6003a](https://github.com/branes-ai/cortex/commit/2e6003ad0bd77de9122eb09057528c8d995928cf))
* **sdk:** surface per-update NIS for filter-consistency monitoring ([3da99b0](https://github.com/branes-ai/cortex/commit/3da99b02f81b90ab8dd1a1c34faa9bab0ecae0cc))
* **sdk:** surface per-update NIS for filter-consistency monitoring ([#264](https://github.com/branes-ai/cortex/issues/264)) ([#272](https://github.com/branes-ai/cortex/issues/272)) ([3da99b0](https://github.com/branes-ai/cortex/commit/3da99b02f81b90ab8dd1a1c34faa9bab0ecae0cc))
* **sdk:** top-level vio estimator API ([#188](https://github.com/branes-ai/cortex/issues/188)) ([3546f32](https://github.com/branes-ai/cortex/commit/3546f32ffc89450a3b651222e291fbc6e21a229c))
* **sdk:** two-view SfM bootstrap (essential matrix + RANSAC) ([#231](https://github.com/branes-ai/cortex/issues/231)) ([a5b3031](https://github.com/branes-ai/cortex/commit/a5b303177db45935f0f84053135b8aee480c478c))
* **sdk:** vio backend interface separating front-end from estimator ([bb56e46](https://github.com/branes-ai/cortex/commit/bb56e4628339bb4c5fff1285f6af50bad6d0753c)), closes [#34](https://github.com/branes-ai/cortex/issues/34)
* **sdk:** VioBackend interface separating front-end from estimator ([#167](https://github.com/branes-ai/cortex/issues/167)) ([bb56e46](https://github.com/branes-ai/cortex/commit/bb56e4628339bb4c5fff1285f6af50bad6d0753c))
* **sdk:** wire dynamic VI init into MsckfBackend (InitMethod::Dynamic) ([#238](https://github.com/branes-ai/cortex/issues/238)) ([8f50204](https://github.com/branes-ai/cortex/commit/8f5020405391567aca8b2f2552b3492fc88fd48b))
* **sdk:** wire the MsckfInvariantBackend (R-IEKF filter) ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#357](https://github.com/branes-ai/cortex/issues/357)) ([30cd651](https://github.com/branes-ai/cortex/commit/30cd65130705558817c1ea2df03f4593f747786b))
* **test:** EuRoC verdict — online extrinsic calibration cleared as [#212](https://github.com/branes-ai/cortex/issues/212) driver ([#332](https://github.com/branes-ai/cortex/issues/332)) ([#343](https://github.com/branes-ai/cortex/issues/343)) ([17f7570](https://github.com/branes-ai/cortex/commit/17f757010ae8bea0aeedc641254d50399a0b3fd2))
* **tools:** --trace tap for EuRoC asl_replay — real per-stage trace dumps ([#387](https://github.com/branes-ai/cortex/issues/387)) ([eb8f9c0](https://github.com/branes-ai/cortex/commit/eb8f9c05134ba5046a2696199552ee383863002e))
* **tools:** add EuRoC source + scene-video metric overlay to vio_pipeline ([#292](https://github.com/branes-ai/cortex/issues/292)) ([94fa599](https://github.com/branes-ai/cortex/commit/94fa5998d5514dd9f22c5296f7b73a04e15b6da4))
* **tools:** add VIO stage-probe utilities — S0 sensor-model + S2 propagation ([#283](https://github.com/branes-ai/cortex/issues/283)) ([4a2c1ef](https://github.com/branes-ai/cortex/commit/4a2c1efc029e799623d7ebf9b523c7041b86f7f6))
* **tools:** depth-colored scene overlay + vio_pipeline how-to guide ([#300](https://github.com/branes-ai/cortex/issues/300)) ([648ecc3](https://github.com/branes-ai/cortex/commit/648ecc3c2dc9eeb25d8df3d2fa53fc01f4d62cf0))
* **tools:** end-to-end VIO noise-&gt;robustness demo (synthetic source) ([#290](https://github.com/branes-ai/cortex/issues/290)) ([2492262](https://github.com/branes-ai/cortex/commit/2492262ef21a518011f895d62f8d6ba3115fa185))
* **tools:** inter-stage VIO trace bus — typed, human-readable per-stage I/O records ([#385](https://github.com/branes-ai/cortex/issues/385)) ([ce63b06](https://github.com/branes-ai/cortex/commit/ce63b0660fafefa42850539f087939fea8d4e484))
* **tools:** s0 sensor-model inspector — real EuRoC frame/IMU → camera & noise contract ([#394](https://github.com/branes-ai/cortex/issues/394)) ([6a88cad](https://github.com/branes-ai/cortex/commit/6a88cad9c55f5487938b83ebcebd2f91e0c08ec9))
* **tools:** s1 initialization inspector — real EuRoC bootstrap → initial state & covariance ([#407](https://github.com/branes-ai/cortex/issues/407)) ([e7930c9](https://github.com/branes-ai/cortex/commit/e7930c9064e32f66e86503ab96bdfdc9ed2e428e))
* **tools:** S4 frontend inspector — real EuRoC FAST+KLT tracks, visualized ([#390](https://github.com/branes-ai/cortex/issues/390)) ([e2add78](https://github.com/branes-ai/cortex/commit/e2add78cbc7bfb6e7a15444121714e0b8f8f7419))
* **tools:** s5 triangulation inspector — real EuRoC tracks+poses → 3D landmarks ([#398](https://github.com/branes-ai/cortex/issues/398)) ([a3999ab](https://github.com/branes-ai/cortex/commit/a3999ab112a0af40c60ef68bc7bcd3595861438e))
* **tools:** s6 msckf-update inspector — real EuRoC updates → state/covariance ([#403](https://github.com/branes-ai/cortex/issues/403)) ([bc60f6e](https://github.com/branes-ai/cortex/commit/bc60f6ee72a8b76fb0334cfeeccdeda0c809fce4))
* **tools:** wire S1 (initialization) stage-probe ([#286](https://github.com/branes-ai/cortex/issues/286)) ([fa3d92b](https://github.com/branes-ai/cortex/commit/fa3d92ba708b3402ffd7d3d0a8f66af74c369717))
* **tools:** wire S10 (calibration) stage-probe to quantify the [#212](https://github.com/branes-ai/cortex/issues/212) Rx4 ([#295](https://github.com/branes-ai/cortex/issues/295)) ([7ccb85c](https://github.com/branes-ai/cortex/commit/7ccb85ced059f85c3d70920b096339a5675324ae))
* wire the S4 visual-frontend probe + data-backed page ([#321](https://github.com/branes-ai/cortex/issues/321)) ([2e30b15](https://github.com/branes-ai/cortex/commit/2e30b15a25f589207af75c1eb59ad33f1c3fb3ee))


### Bug Fixes

* **build:** link the Rust core in MSVC Debug by pinning the release CRT ([#226](https://github.com/branes-ai/cortex/issues/226)) ([f963950](https://github.com/branes-ai/cortex/commit/f9639500bad40564ea5c9bd264ce60ec1c84ac90))
* **docs:** make topology + VIO stage-flow SVGs readable in dark theme ([#334](https://github.com/branes-ai/cortex/issues/334)) ([623b34e](https://github.com/branes-ai/cortex/commit/623b34ec4924da28cda583b0a2fe240a792b4717))
* **docs:** point the benchmark generator at the moved euroc test ([#223](https://github.com/branes-ai/cortex/issues/223)) ([023fb74](https://github.com/branes-ai/cortex/commit/023fb74e7df7b3adeeac12d97fefb1faaf706f27))
* **docs:** re-anchor EuRoC gate extraction (unbreak Deploy Documentation) ([#256](https://github.com/branes-ai/cortex/issues/256)) ([f99c4e1](https://github.com/branes-ai/cortex/commit/f99c4e12d0d604e4ef5f93f1f0c5e65a56f7d198))
* **docs:** re-anchor the EuRoC gate extraction to the run_euroc_replay call ([f99c4e1](https://github.com/branes-ai/cortex/commit/f99c4e12d0d604e4ef5f93f1f0c5e65a56f7d198))
* **math:** correctness fixes from post-merge review of Phase 2 ([#159](https://github.com/branes-ai/cortex/issues/159)) ([87a3a82](https://github.com/branes-ai/cortex/commit/87a3a824e967a799a3754ce1b93487a29f3d90b1))
* **math:** guard degenerate inputs in the Lie-group helpers ([#214](https://github.com/branes-ai/cortex/issues/214)) ([f51c3a9](https://github.com/branes-ai/cortex/commit/f51c3a91ec457b62d691d87ff834c48cef2dd33e)), closes [#161](https://github.com/branes-ai/cortex/issues/161)
* **sdk:** bootstrap dynamic-init SfM on the widest baseline ([df517c4](https://github.com/branes-ai/cortex/commit/df517c4a9bec74659f72ec880aeb72105be329aa))
* **sdk:** bootstrap dynamic-init SfM on the widest baseline ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#251](https://github.com/branes-ai/cortex/issues/251)) ([df517c4](https://github.com/branes-ai/cortex/commit/df517c4a9bec74659f72ec880aeb72105be329aa))
* **sdk:** correct dynamic-init gravity sign — real-data divergence → bounded ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#252](https://github.com/branes-ai/cortex/issues/252)) ([1876ba0](https://github.com/branes-ai/cortex/commit/1876ba064efce0f0cfef26f0e7cd6ef384053b54))
* **sdk:** correct dynamic-init gravity sign + widest-parallax bootstrap ([1876ba0](https://github.com/branes-ai/cortex/commit/1876ba064efce0f0cfef26f0e7cd6ef384053b54))
* **sdk:** resolve R-IEKF continuous divergence ([#366](https://github.com/branes-ai/cortex/issues/366)) ([7f024dc](https://github.com/branes-ai/cortex/commit/7f024dc61533a25def5f88b5c59a021b6925f38c))
* **sdk:** resolve R-IEKF continuous divergence ([#366](https://github.com/branes-ai/cortex/issues/366)) ([#388](https://github.com/branes-ai/cortex/issues/388)) ([7f024dc](https://github.com/branes-ai/cortex/commit/7f024dc61533a25def5f88b5c59a021b6925f38c))
* **sdk:** set EuRoC extrinsics + gravity-align init; surface VIO divergence ([#209](https://github.com/branes-ai/cortex/issues/209)) ([91a92e8](https://github.com/branes-ai/cortex/commit/91a92e8547bbf2d12ccddd5a67f9a0be299b06f3))
* **test:** make the vio_latency budget informational on CI ([#240](https://github.com/branes-ai/cortex/issues/240)) ([#259](https://github.com/branes-ai/cortex/issues/259)) ([22e01c7](https://github.com/branes-ai/cortex/commit/22e01c720699d92a7dde434d54ad17aa7b189f9d))
* **test:** split euroc-moving-start local decls (set -u unbound var) ([#243](https://github.com/branes-ai/cortex/issues/243)) ([71de21c](https://github.com/branes-ai/cortex/commit/71de21c46917ace4660720d05edabad21bb6ba2d))
* **tools:** create the --out directory so artifacts are not silently dropped ([#298](https://github.com/branes-ai/cortex/issues/298)) ([a1e7fb1](https://github.com/branes-ai/cortex/commit/a1e7fb15f5f6f75ed681d0c7b80a91d6b1f9544c))


### Reverts

* **docs:** restore original working vio_run.jsonl ([8762ee7](https://github.com/branes-ai/cortex/commit/8762ee7c1efc7e12326f38da3e066b15125ae8ea))


### Code Refactoring

* **test:** drop redundant layer/_test segments from test filenames ([#225](https://github.com/branes-ai/cortex/issues/225)) ([1cafd9a](https://github.com/branes-ai/cortex/commit/1cafd9a4f467cc38369fed72a53ddbc640b730a6))


### Build System

* **cmake:** add CMakePresets.json with sitl/kpu-cross/wsl2 presets ([83ec0fe](https://github.com/branes-ai/cortex/commit/83ec0fe4a05dd46f923faf8729eacdaf35d65100))
* **cmake:** add CMakePresets.json with sitl/kpu-cross/wsl2 presets ([3eb0943](https://github.com/branes-ai/cortex/commit/3eb09438a069192bec9bea483e2d428113f4ed66))
* **cmake:** add cross-toolchain.cmake for KPU aarch64 cross-compile ([3e93b9c](https://github.com/branes-ai/cortex/commit/3e93b9cbc1fb801b254458f5445f31ac17e3440f))
* **cmake:** add cross-toolchain.cmake for KPU aarch64 cross-compile ([ad40c9f](https://github.com/branes-ai/cortex/commit/ad40c9f06ee18b58d773d8de25c40155d376067e))
* **cmake:** wire FetchContent for foundational dependencies ([59345f6](https://github.com/branes-ai/cortex/commit/59345f6c41ff93562b5f20ff37a978e44706ed64))
* **cmake:** wire FetchContent for foundational dependencies ([8316fce](https://github.com/branes-ai/cortex/commit/8316fce336eab515deec8f683a133defc5d33eca))
* **test:** generate test targets from a compile_all glob macro ([#220](https://github.com/branes-ai/cortex/issues/220)) ([9a4194d](https://github.com/branes-ai/cortex/commit/9a4194dc872d2cab30a87b61b0ba4a618a46bdef))


### Continuous Integration

* add lint job (clippy + cargo fmt + clang-format) ([dd288c8](https://github.com/branes-ai/cortex/commit/dd288c82eba34ec78daf2a67c55ac0ee95940b8a))
* add lint job (clippy + cargo fmt + clang-format) ([2075637](https://github.com/branes-ai/cortex/commit/20756370e458ae8d35ed6ca5eca8012951dfb019))
* gate Rust coverage at &gt;= 85% line coverage ([#135](https://github.com/branes-ai/cortex/issues/135)) ([486466c](https://github.com/branes-ai/cortex/commit/486466c9c9c6b85d1f4c02203e37fe1125e97548))
* install sccache in lint job too (RUSTC_WRAPPER consistency) ([daf0e7d](https://github.com/branes-ai/cortex/commit/daf0e7d5f0cdb7e4cb49fa2b7265b2fa5374aad1))
* integrate sccache for Rust + C++ object caching ([818b018](https://github.com/branes-ai/cortex/commit/818b018302a72fad30471297e1c0f9b64969c4b1))
* integrate sccache for Rust + C++ object caching ([8478572](https://github.com/branes-ai/cortex/commit/84785724f7f1d5648e819c534493578232c8456a))
* **windows:** build the MSVC job via Ninja + sccache ([#222](https://github.com/branes-ai/cortex/issues/222)) ([d39aed9](https://github.com/branes-ai/cortex/commit/d39aed9be8de648b86c2963d0cba07679601e3c3))


### Documentation

* add CLAUDE.md and bootstrap plan ([ed34b81](https://github.com/branes-ai/cortex/commit/ed34b81191272be569f7e188dbec9cf42a5b8ad1))
* add CLAUDE.md and bootstrap plan; gitignore .claude/ ([fa7cefa](https://github.com/branes-ai/cortex/commit/fa7cefa38256a35a8d177ca3737915a4475773f9))
* add Phase 0 Foundation retrospective ([5afc918](https://github.com/branes-ai/cortex/commit/5afc9182a200dd5d50d000a16eed61c9eea9b645))
* add Phase 0 Foundation retrospective ([ea4b5f4](https://github.com/branes-ai/cortex/commit/ea4b5f4e405e69a7693b8627ac8baea41e64bef2))
* add Phase 1 design — Rust RM + cxx bridge ([#125](https://github.com/branes-ai/cortex/issues/125)) ([d23eeec](https://github.com/branes-ai/cortex/commit/d23eeec017a57f6aefef9012940699ddfc639c23))
* add PR template with the per-PR docs-update rule ([#196](https://github.com/branes-ai/cortex/issues/196)) ([183f3e1](https://github.com/branes-ai/cortex/commit/183f3e18d16cbd358cea4e45c7e2d78ae579e765))
* add Starlight + Doxygen documentation site for Phases 0-3 ([#192](https://github.com/branes-ai/cortex/issues/192)) ([2fac010](https://github.com/branes-ai/cortex/commit/2fac0105d8bc24ecf04eb798c697a9d77b1fa631))
* add VIO S0–S10 stage-flow SVG and replace the pipeline ASCII dataflow ([7ffb706](https://github.com/branes-ai/cortex/commit/7ffb706a8c242331d13863a76f454f86f1848edb))
* add VIO S0–S10 stage-flow SVG; replace pipeline ASCII dataflow ([#330](https://github.com/branes-ai/cortex/issues/330)) ([7ffb706](https://github.com/branes-ai/cortex/commit/7ffb706a8c242331d13863a76f454f86f1848edb))
* **assessment:** a plain-language tutorial on NEES/NIS filter consistency ([acc8f8d](https://github.com/branes-ai/cortex/commit/acc8f8d1acb5d09d99995af9416ae90f6e4c29bb))
* **assessment:** plain-language NEES/NIS filter-consistency tutorial ([#274](https://github.com/branes-ai/cortex/issues/274)) ([acc8f8d](https://github.com/branes-ai/cortex/commit/acc8f8d1acb5d09d99995af9416ae90f6e4c29bb))
* **assessment:** VIO diagnostic methodology + instrumentation build list ([#269](https://github.com/branes-ai/cortex/issues/269)) ([755e694](https://github.com/branes-ai/cortex/commit/755e69404f79c567c31c2f6c4369d4828466ada4))
* fix doubled base path on the API reference sidebar link ([#194](https://github.com/branes-ai/cortex/issues/194)) ([d878e47](https://github.com/branes-ai/cortex/commit/d878e47d7da41b97dc174c43700166927209519f))
* improve overview phrasing and trigger documentation rebuild ([2978a7d](https://github.com/branes-ai/cortex/commit/2978a7d7cc06af5850f191eda8196d048af5495c))
* **math:** Doxygen API docs + concepts reference page ([#157](https://github.com/branes-ai/cortex/issues/157)) ([fdc9aed](https://github.com/branes-ai/cortex/commit/fdc9aed23fd3b6ec3c4a0f1cd33c5f9c9e87d629))
* **math:** doxygen API docs and a concepts reference page ([fdc9aed](https://github.com/branes-ai/cortex/commit/fdc9aed23fd3b6ec3c4a0f1cd33c5f9c9e87d629)), closes [#32](https://github.com/branes-ai/cortex/issues/32)
* migrate diagnostic-methodology as the section's method primer ([#316](https://github.com/branes-ai/cortex/issues/316)) ([0097585](https://github.com/branes-ai/cortex/commit/009758509072ac129b5dccd409ac45a90dbd1ccb))
* pin benchmark gate numbers to their source ([#197](https://github.com/branes-ai/cortex/issues/197)) ([707d5d6](https://github.com/branes-ai/cortex/commit/707d5d62cb581368fa8073459c16e943444fec8b))
* process documentation ([3aacba2](https://github.com/branes-ai/cortex/commit/3aacba2cb25e05006f21d0a16311dc9befa30c1b))
* record the [#212](https://github.com/branes-ai/cortex/issues/212) observability investigation — yaw leak localized, FEJ refuted, R-IEKF next ([#345](https://github.com/branes-ai/cortex/issues/345)) ([7dff9f5](https://github.com/branes-ai/cortex/commit/7dff9f503cc856366a48747e6a6f7f767d6c6d09))
* render architecture topology as SVG; wire in orphaned S0 fisheye figures ([#327](https://github.com/branes-ai/cortex/issues/327)) ([4aa72c1](https://github.com/branes-ai/cortex/commit/4aa72c185c44b0f83b224b0ea15d4a9d491fe07f))
* render LaTeX math on the site (KaTeX) + camera-calibration walkthrough ([#362](https://github.com/branes-ai/cortex/issues/362)) ([675a414](https://github.com/branes-ai/cortex/commit/675a414208f3104ba268b81a25601e601462fdba))
* S1 initialization stage page (algorithm sound, seed suspect) ([#315](https://github.com/branes-ai/cortex/issues/315)) ([936cdb1](https://github.com/branes-ai/cortex/commit/936cdb1925e23c752cba8971c13e45dbf63cfd5f))
* S10 online-calibration stage page (the [#212](https://github.com/branes-ai/cortex/issues/212) centerpiece) ([#307](https://github.com/branes-ai/cortex/issues/307)) ([40b483b](https://github.com/branes-ai/cortex/commit/40b483beae187f0d21a3e6e9e47be68cbbb2981f))
* S2 IMU-propagation stage page (a cleared [#212](https://github.com/branes-ai/cortex/issues/212) candidate) ([#311](https://github.com/branes-ai/cortex/issues/311)) ([b930f07](https://github.com/branes-ai/cortex/commit/b930f078d1a88953ba6e36cb4bcc731328d0066e))
* S6 MSCKF-update stage + consistency-analysis page ([#309](https://github.com/branes-ai/cortex/issues/309)) ([5607dd1](https://github.com/branes-ai/cortex/commit/5607dd10cb7172e021beb71a0845c978c07a4e7e))
* session log + changelog for the R-IEKF/FEJ/verdict arc ([#369](https://github.com/branes-ai/cortex/issues/369)) ([8383b41](https://github.com/branes-ai/cortex/commit/8383b416f377dd6ec715b623295c12d155cf815b))
* **session:** wrap up 2026-06-05 — VIO pipeline docs + 3D viz ([#318](https://github.com/branes-ai/cortex/issues/318)) ([d5d8942](https://github.com/branes-ai/cortex/commit/d5d894257c4b5cc732b7ea95fee81157aa43934a))
* **session:** wrap up 2026-06-05 — VIO pipeline docs section + 3D viz ([d5d8942](https://github.com/branes-ai/cortex/commit/d5d894257c4b5cc732b7ea95fee81157aa43934a))
* **site:** add S2 IMU-propagation stage page (a cleared [#212](https://github.com/branes-ai/cortex/issues/212) candidate) ([b930f07](https://github.com/branes-ai/cortex/commit/b930f078d1a88953ba6e36cb4bcc731328d0066e))
* **site:** add the S1 initialization stage page ([936cdb1](https://github.com/branes-ai/cortex/commit/936cdb1925e23c752cba8971c13e45dbf63cfd5f))
* **site:** add the S10 online-calibration stage page ([#212](https://github.com/branes-ai/cortex/issues/212) centerpiece) ([40b483b](https://github.com/branes-ai/cortex/commit/40b483beae187f0d21a3e6e9e47be68cbbb2981f))
* **site:** improve VIO pipeline overview, S0, and S1 stages ([65a7680](https://github.com/branes-ai/cortex/commit/65a76806f91720c0a6f011205eb82ac8c70e0758))
* **site:** improve VIO pipeline overview, S0, and S1 stages ([65a7680](https://github.com/branes-ai/cortex/commit/65a76806f91720c0a6f011205eb82ac8c70e0758))
* **site:** migrate the diagnostic-methodology doc as a section page ([0097585](https://github.com/branes-ai/cortex/commit/009758509072ac129b5dccd409ac45a90dbd1ccb))
* **site:** widen the triangulation baseline so the 3D view reads ([#364](https://github.com/branes-ai/cortex/issues/364)) ([7536a5e](https://github.com/branes-ai/cortex/commit/7536a5eec984cde232c320174b7ab82c0540b2ce))
* **test:** explain pyramidal KLT + add the numerical failure-mode taxonomy ([c8f8ce9](https://github.com/branes-ai/cortex/commit/c8f8ce97e025bddd3074f345ce0b4bb9c417a376))
* **test:** pyramidal KLT explainer + EuRoC numerical failure-mode taxonomy ([#255](https://github.com/branes-ai/cortex/issues/255)) ([c8f8ce9](https://github.com/branes-ai/cortex/commit/c8f8ce97e025bddd3074f345ce0b4bb9c417a376))
* **test:** vio_euroc goal & architecture ([#249](https://github.com/branes-ai/cortex/issues/249)) ([0f88066](https://github.com/branes-ai/cortex/commit/0f8806675941d355ce7fee804bf1d909168f4282))
* VIO pipeline section (overview, metrics, S0, status) + issue-ref policy ([#305](https://github.com/branes-ai/cortex/issues/305)) ([1212cde](https://github.com/branes-ai/cortex/commit/1212cde93c900cdbdd0c17b96ae7f9342161cf32))


### Tests

* **bridge:** add C++/Rust cxx::bridge integration test ([#137](https://github.com/branes-ai/cortex/issues/137)) ([53ab21c](https://github.com/branes-ai/cortex/commit/53ab21c940359c0cf3971a4819f512e755ac5ca3))
* **math:** golden-data NLS suite + Jacobian FD checks + type matrix ([#155](https://github.com/branes-ai/cortex/issues/155)) ([c0ba096](https://github.com/branes-ai/cortex/commit/c0ba096d29174bf8400d823b0d6ec4c50cea4612)), closes [#31](https://github.com/branes-ai/cortex/issues/31)
* **sdk:** dataset-gated MH_05/V2_03 moving-start dynamic-init validation ([#241](https://github.com/branes-ai/cortex/issues/241)) ([20fe9db](https://github.com/branes-ai/cortex/commit/20fe9dbaf5c3c49a9c73a46b76d585a09bc452c3))
* **sdk:** drive the observability gauge check through the production CameraUpdater ([#358](https://github.com/branes-ai/cortex/issues/358)) ([7607e32](https://github.com/branes-ai/cortex/commit/7607e32cbc2b4e656e52b59024abc8d251a59e82))
* **sdk:** dynamic-init diagnostics to localize the real-data divergence ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#248](https://github.com/branes-ai/cortex/issues/248)) ([dbd90b4](https://github.com/branes-ai/cortex/commit/dbd90b488aba6604177f2daad2e45ef326c4d16c))
* **sdk:** euroc replay harness + ATE/RPE benchmark ([#190](https://github.com/branes-ai/cortex/issues/190)) ([6fc5c12](https://github.com/branes-ai/cortex/commit/6fc5c12e6f516d4024fcef3c2d95c6462af1bd96))
* **sdk:** instrument dynamic-init attempts ([#247](https://github.com/branes-ai/cortex/issues/247) localization) ([#250](https://github.com/branes-ai/cortex/issues/250)) ([7b1493b](https://github.com/branes-ai/cortex/commit/7b1493bc9c8130a5edf7aa9eb15e57fdfc79c624))
* **sdk:** instrument dynamic-init attempts to localize why it won't fire ([7b1493b](https://github.com/branes-ai/cortex/commit/7b1493bc9c8130a5edf7aa9eb15e57fdfc79c624))
* **sdk:** latency budget enforcement (per-frame upper bound) ([#191](https://github.com/branes-ai/cortex/issues/191)) ([90a98fa](https://github.com/branes-ai/cortex/commit/90a98fae3f5220678eb0d58f7317649f3d47990b))
* **sdk:** real EuRoC extrinsics — MH_05 moving-start ATE validated (~0.79 m) ([#245](https://github.com/branes-ai/cortex/issues/245)) ([df70f84](https://github.com/branes-ai/cortex/commit/df70f84e103ae0a99e023ebb12fe349ac747c6ae))
* **sdk:** score EuRoC ATE from initialization onward ([#253](https://github.com/branes-ai/cortex/issues/253)) ([36a8334](https://github.com/branes-ai/cortex/commit/36a8334751ee11429e4d6f6fb47db8abd52908d2))
* **sdk:** score EuRoC ATE only from initialization onward ([36a8334](https://github.com/branes-ai/cortex/commit/36a8334751ee11429e4d6f6fb47db8abd52908d2))


### Maintenance

* **bootstrap:** add bootstrap.sh + bootstrap.ps1 for new-dev setup ([61c0889](https://github.com/branes-ai/cortex/commit/61c08895bde2bf17d37956eefc104b6766277915))
* **bootstrap:** add bootstrap.sh + bootstrap.ps1 for new-dev setup ([2f3167e](https://github.com/branes-ai/cortex/commit/2f3167e0726bd4726883b04205837b9b6a88483a))
* **coderabbit:** add .coderabbit.yaml with layering + license + terminology rules ([#119](https://github.com/branes-ai/cortex/issues/119)) ([7618067](https://github.com/branes-ai/cortex/commit/761806787be80670878047aecd0ba93c1c1cda79))
* **format:** add .clang-format, .editorconfig, and license ADR ([0c3e10c](https://github.com/branes-ai/cortex/commit/0c3e10cc2755145b967ddad59f279f7dd43a0428))
* **format:** add .clang-format, .editorconfig, and license-compatibility ADR ([b45958e](https://github.com/branes-ai/cortex/commit/b45958e5a25ff04fa2173c44c93774c6c7b76002))
* **main:** release 0.1.0 ([#122](https://github.com/branes-ai/cortex/issues/122)) ([98173c1](https://github.com/branes-ai/cortex/commit/98173c10c186437b1def8b182e1c22ebb17551dc))
* **main:** release 0.1.1 ([#124](https://github.com/branes-ai/cortex/issues/124)) ([c71ba68](https://github.com/branes-ai/cortex/commit/c71ba68b44600bbd1158821f7e67dc9e37587990))
* **main:** release 0.1.2 ([#126](https://github.com/branes-ai/cortex/issues/126)) ([c3fdb5c](https://github.com/branes-ai/cortex/commit/c3fdb5ce46bc56f1726038163e7d1018b5aa1455))
* **main:** release 0.10.0 ([#150](https://github.com/branes-ai/cortex/issues/150)) ([72c9120](https://github.com/branes-ai/cortex/commit/72c912068f55d520cd2ec35008aa4feac9a47ae8))
* **main:** release 0.11.0 ([#152](https://github.com/branes-ai/cortex/issues/152)) ([d24d591](https://github.com/branes-ai/cortex/commit/d24d59166f3d567bcd9eaf3a7f09cab2971615f1))
* **main:** release 0.12.0 ([#154](https://github.com/branes-ai/cortex/issues/154)) ([e748d0d](https://github.com/branes-ai/cortex/commit/e748d0da54e45f0e1e5a1c0960f7060adec93e1f))
* **main:** release 0.12.1 ([#156](https://github.com/branes-ai/cortex/issues/156)) ([b37725a](https://github.com/branes-ai/cortex/commit/b37725a31a0c0a1698e4ba213adb046968f0e509))
* **main:** release 0.12.2 ([#158](https://github.com/branes-ai/cortex/issues/158)) ([a730534](https://github.com/branes-ai/cortex/commit/a730534c0f549ae7439bd4560fad75d429029571))
* **main:** release 0.12.3 ([#160](https://github.com/branes-ai/cortex/issues/160)) ([a78e2ae](https://github.com/branes-ai/cortex/commit/a78e2ae4cb0fb08eee237f48cf1cdb6d22366db6))
* **main:** release 0.13.0 ([#168](https://github.com/branes-ai/cortex/issues/168)) ([d337aa2](https://github.com/branes-ai/cortex/commit/d337aa2be6953c75e79608518b8e6e70ca6777d9))
* **main:** release 0.14.0 ([#171](https://github.com/branes-ai/cortex/issues/171)) ([9612853](https://github.com/branes-ai/cortex/commit/9612853cc9f67b3debd51c13b052e462d50b54af))
* **main:** release 0.15.0 ([#173](https://github.com/branes-ai/cortex/issues/173)) ([92a99a1](https://github.com/branes-ai/cortex/commit/92a99a1c35b4a9eaac16dc812de0b8b66bae0308))
* **main:** release 0.16.0 ([#175](https://github.com/branes-ai/cortex/issues/175)) ([07275c4](https://github.com/branes-ai/cortex/commit/07275c4f4429fcff6941a7f3d3893f1a450075ce))
* **main:** release 0.17.0 ([#177](https://github.com/branes-ai/cortex/issues/177)) ([09f2997](https://github.com/branes-ai/cortex/commit/09f299794f41163ed1538d461ce8e02dddbf2d8c))
* **main:** release 0.18.0 ([#179](https://github.com/branes-ai/cortex/issues/179)) ([d9ae937](https://github.com/branes-ai/cortex/commit/d9ae9373762d3f4092d0037b1b133b23e059d005))
* **main:** release 0.19.0 ([#181](https://github.com/branes-ai/cortex/issues/181)) ([c5c216e](https://github.com/branes-ai/cortex/commit/c5c216e98c0ef4374fd56638744846e0532e35ee))
* **main:** release 0.2.0 ([#128](https://github.com/branes-ai/cortex/issues/128)) ([447a2db](https://github.com/branes-ai/cortex/commit/447a2db297ea1d73e2c03174fc7541059e5464d7))
* **main:** release 0.20.0 ([#183](https://github.com/branes-ai/cortex/issues/183)) ([ea09dbd](https://github.com/branes-ai/cortex/commit/ea09dbdfe314c3732a9eeada15ddc4b5f9f5a8dc))
* **main:** release 0.20.1 ([#193](https://github.com/branes-ai/cortex/issues/193)) ([85a5460](https://github.com/branes-ai/cortex/commit/85a5460624925cf3321cc7febf6c165661e80b18))
* **main:** release 0.20.2 ([#195](https://github.com/branes-ai/cortex/issues/195)) ([42c644a](https://github.com/branes-ai/cortex/commit/42c644a05f8362773e9aad541162b44f17e5bd75))
* **main:** release 0.20.3 ([#198](https://github.com/branes-ai/cortex/issues/198)) ([641f8d3](https://github.com/branes-ai/cortex/commit/641f8d31845d9f3a62dd1bb153c561e80b1dbf6d))
* **main:** release 0.21.0 ([#202](https://github.com/branes-ai/cortex/issues/202)) ([9580b19](https://github.com/branes-ai/cortex/commit/9580b1918659af0f01952079558e8fa5ef701daa))
* **main:** release 0.22.0 ([#205](https://github.com/branes-ai/cortex/issues/205)) ([5baa9f9](https://github.com/branes-ai/cortex/commit/5baa9f91919ea702aa8ee24a46f4c3758f6f1541))
* **main:** release 0.23.0 ([#210](https://github.com/branes-ai/cortex/issues/210)) ([1d54a48](https://github.com/branes-ai/cortex/commit/1d54a48e8a5358d4f435c49362b39099b7dd4404))
* **main:** release 0.23.1 ([#215](https://github.com/branes-ai/cortex/issues/215)) ([29527e4](https://github.com/branes-ai/cortex/commit/29527e4e0b6ec8eee817cdf056de3c918db85a4a))
* **main:** release 0.24.0 ([#217](https://github.com/branes-ai/cortex/issues/217)) ([c1d58bf](https://github.com/branes-ai/cortex/commit/c1d58bf92f248976d9fdea4ff3efc36c81a32b87))
* **main:** release 0.25.0 ([#219](https://github.com/branes-ai/cortex/issues/219)) ([3dfd037](https://github.com/branes-ai/cortex/commit/3dfd0379b6a4cd67c13b72ee339c339b3664d2fb))
* **main:** release 0.25.1 ([#221](https://github.com/branes-ai/cortex/issues/221)) ([0d757ea](https://github.com/branes-ai/cortex/commit/0d757ea1c9bbb37075bc742914afe1aca3d43e37))
* **main:** release 0.25.2 ([#224](https://github.com/branes-ai/cortex/issues/224)) ([2ff6364](https://github.com/branes-ai/cortex/commit/2ff63645c6334aa3f0c246d6d9601c9a1e5df324))
* **main:** release 0.25.3 ([#227](https://github.com/branes-ai/cortex/issues/227)) ([b195284](https://github.com/branes-ai/cortex/commit/b19528491e3a4e7a21ec955c49b289b6b8af0ad3))
* **main:** release 0.26.0 ([#232](https://github.com/branes-ai/cortex/issues/232)) ([0cb93ee](https://github.com/branes-ai/cortex/commit/0cb93eea6cf80488958638cfeaf4d013b1fd402a))
* **main:** release 0.27.0 ([#234](https://github.com/branes-ai/cortex/issues/234)) ([94573ce](https://github.com/branes-ai/cortex/commit/94573ced57ddd44707090e1aa1bf09b4c7b417fe))
* **main:** release 0.28.0 ([#237](https://github.com/branes-ai/cortex/issues/237)) ([b791154](https://github.com/branes-ai/cortex/commit/b7911547461bea5f3ec1bab39f0263e2ec073413))
* **main:** release 0.29.0 ([#239](https://github.com/branes-ai/cortex/issues/239)) ([92a1ee4](https://github.com/branes-ai/cortex/commit/92a1ee41328c72c477c0fec10b9ba3ece3003f9c))
* **main:** release 0.3.0 ([#130](https://github.com/branes-ai/cortex/issues/130)) ([3066e9f](https://github.com/branes-ai/cortex/commit/3066e9f5746fcbb40e36ab6189f260a751ea08f6))
* **main:** release 0.30.0 ([#242](https://github.com/branes-ai/cortex/issues/242)) ([a030a06](https://github.com/branes-ai/cortex/commit/a030a06c93038627128be36b297219b4ebe01ce8))
* **main:** release 0.30.0 ([#257](https://github.com/branes-ai/cortex/issues/257)) ([1eceabc](https://github.com/branes-ai/cortex/commit/1eceabc36793955f64bc193012cf0183086b9eaa))
* **main:** release 0.30.1 ([#258](https://github.com/branes-ai/cortex/issues/258)) ([9eac95c](https://github.com/branes-ai/cortex/commit/9eac95cec63c3da77c4436a8f83c28cad2707fdd))
* **main:** release 0.30.2 ([#260](https://github.com/branes-ai/cortex/issues/260)) ([f974ba6](https://github.com/branes-ai/cortex/commit/f974ba605eb8120c45fed031cd9ed9342dd5054a))
* **main:** release 0.31.0 ([#270](https://github.com/branes-ai/cortex/issues/270)) ([a661012](https://github.com/branes-ai/cortex/commit/a661012369f83586c4cfa2febf0b2b2caccffd2d))
* **main:** release 0.32.0 ([#275](https://github.com/branes-ai/cortex/issues/275)) ([e51079e](https://github.com/branes-ai/cortex/commit/e51079e6d75efd9dd12ed8e29aa4cd392bbc0f22))
* **main:** release 0.33.0 ([#277](https://github.com/branes-ai/cortex/issues/277)) ([f38144f](https://github.com/branes-ai/cortex/commit/f38144fcefcd87a2ca1c79de63a76ead2fcb5332))
* **main:** release 0.34.0 ([#281](https://github.com/branes-ai/cortex/issues/281)) ([22e9da8](https://github.com/branes-ai/cortex/commit/22e9da8d29546bf5709ea12a594ec5e55423ee49))
* **main:** release 0.35.0 ([#284](https://github.com/branes-ai/cortex/issues/284)) ([6be0aa3](https://github.com/branes-ai/cortex/commit/6be0aa323ea11aeba95e1b5464a25f8391e6ab3e))
* **main:** release 0.36.0 ([#289](https://github.com/branes-ai/cortex/issues/289)) ([2a37795](https://github.com/branes-ai/cortex/commit/2a37795d2d6da3db1728b4aab7c8176ac73ad374))
* **main:** release 0.37.0 ([#293](https://github.com/branes-ai/cortex/issues/293)) ([4df3a67](https://github.com/branes-ai/cortex/commit/4df3a67f08f92c5fbad5e21549f81c3ddd3ff9a6))
* **main:** release 0.38.0 ([#301](https://github.com/branes-ai/cortex/issues/301)) ([cbf319d](https://github.com/branes-ai/cortex/commit/cbf319dd9169488f51afbdb853e1ea1fa63e239a))
* **main:** release 0.39.0 ([#304](https://github.com/branes-ai/cortex/issues/304)) ([286164c](https://github.com/branes-ai/cortex/commit/286164cca361a3904f84dc784584e2e3c0dd33c4))
* **main:** release 0.39.1 ([#306](https://github.com/branes-ai/cortex/issues/306)) ([3193338](https://github.com/branes-ai/cortex/commit/31933381568f444d460b745c0405ed876597176d))
* **main:** release 0.39.2 ([#308](https://github.com/branes-ai/cortex/issues/308)) ([c23ce1c](https://github.com/branes-ai/cortex/commit/c23ce1ceab847fcc4006dad34e27bf080d4ba189))
* **main:** release 0.39.3 ([#310](https://github.com/branes-ai/cortex/issues/310)) ([6480f58](https://github.com/branes-ai/cortex/commit/6480f58f306206b65ea264709fc1c925272f5014))
* **main:** release 0.39.4 ([#312](https://github.com/branes-ai/cortex/issues/312)) ([71d988d](https://github.com/branes-ai/cortex/commit/71d988d54815de09f86b806974ebe5f731cad523))
* **main:** release 0.4.0 ([#132](https://github.com/branes-ai/cortex/issues/132)) ([8e5ac64](https://github.com/branes-ai/cortex/commit/8e5ac64c22222784c128189df9fb6f568efd7de8))
* **main:** release 0.40.0 ([#314](https://github.com/branes-ai/cortex/issues/314)) ([189b776](https://github.com/branes-ai/cortex/commit/189b7765f1120417110fdeb758ffe9262496210c))
* **main:** release 0.40.1 ([#317](https://github.com/branes-ai/cortex/issues/317)) ([f353c10](https://github.com/branes-ai/cortex/commit/f353c104cc28db68c90384f06e4f81daf7c44d27))
* **main:** release 0.41.0 ([#320](https://github.com/branes-ai/cortex/issues/320)) ([96cc8c2](https://github.com/branes-ai/cortex/commit/96cc8c2b5cb74bebf27d4b81b6b599c188762ed6))
* **main:** release 0.42.0 ([#322](https://github.com/branes-ai/cortex/issues/322)) ([1b07a7f](https://github.com/branes-ai/cortex/commit/1b07a7fbb086c3b1ffbc7e7f19ccf76cec23eefc))
* **main:** release 0.43.0 ([#324](https://github.com/branes-ai/cortex/issues/324)) ([52718ca](https://github.com/branes-ai/cortex/commit/52718ca8f1b10d276a942e67b770c49fbf5decef))
* **main:** release 0.44.0 ([#326](https://github.com/branes-ai/cortex/issues/326)) ([f5b82f0](https://github.com/branes-ai/cortex/commit/f5b82f0434b05d1b34d26c90feabe1d8a9bb93e5))
* **main:** release 0.45.0 ([#329](https://github.com/branes-ai/cortex/issues/329)) ([b5bad63](https://github.com/branes-ai/cortex/commit/b5bad633dfb1a5731cbb0e931210b727b38ad54b))
* **main:** release 0.45.1 ([#331](https://github.com/branes-ai/cortex/issues/331)) ([60f449e](https://github.com/branes-ai/cortex/commit/60f449e925f4a81cb003c2ca15586c2bf03f87e6))
* **main:** release 0.45.2 ([#336](https://github.com/branes-ai/cortex/issues/336)) ([b263de9](https://github.com/branes-ai/cortex/commit/b263de980237f63a29b16539d7407fdbe55dc28e))
* **main:** release 0.46.0 ([#342](https://github.com/branes-ai/cortex/issues/342)) ([0003c09](https://github.com/branes-ai/cortex/commit/0003c091fedae4e2fde4333fda7480a5f2ae06b7))
* **main:** release 0.47.0 ([#344](https://github.com/branes-ai/cortex/issues/344)) ([5a1d3a0](https://github.com/branes-ai/cortex/commit/5a1d3a0930937a99c63a32fa00a31b9cd1daa4e3))
* **main:** release 0.47.1 ([#346](https://github.com/branes-ai/cortex/issues/346)) ([6167ae5](https://github.com/branes-ai/cortex/commit/6167ae545ff3f7f5847dd8833f59ed9fe006fbbb))
* **main:** release 0.48.0 ([#350](https://github.com/branes-ai/cortex/issues/350)) ([e4ac4ed](https://github.com/branes-ai/cortex/commit/e4ac4ed5efa8aabf65256401b58ba716dbc5e2db))
* **main:** release 0.49.0 ([#352](https://github.com/branes-ai/cortex/issues/352)) ([33e81dc](https://github.com/branes-ai/cortex/commit/33e81dce6083978ebb791489bef8bc842c7875c2))
* **main:** release 0.5.0 ([#134](https://github.com/branes-ai/cortex/issues/134)) ([5317661](https://github.com/branes-ai/cortex/commit/53176613fbde27db939d3d005878a1889b89275a))
* **main:** release 0.5.1 ([#136](https://github.com/branes-ai/cortex/issues/136)) ([7845ea5](https://github.com/branes-ai/cortex/commit/7845ea51da66e3dbab16d054925ada1151312f39))
* **main:** release 0.5.2 ([#138](https://github.com/branes-ai/cortex/issues/138)) ([15cf147](https://github.com/branes-ai/cortex/commit/15cf147713d52bc7efdd08b49bc3bb2d4b8e4a25))
* **main:** release 0.5.3 ([#140](https://github.com/branes-ai/cortex/issues/140)) ([a49e6a6](https://github.com/branes-ai/cortex/commit/a49e6a6a3694b486509787bb50e6a772b2ae5772))
* **main:** release 0.50.0 ([#355](https://github.com/branes-ai/cortex/issues/355)) ([cadd91b](https://github.com/branes-ai/cortex/commit/cadd91b1793b44cca8198448a97013476b85d60b))
* **main:** release 0.50.1 ([#359](https://github.com/branes-ai/cortex/issues/359)) ([f849892](https://github.com/branes-ai/cortex/commit/f849892df711ca422b44193b721e81d9f4cff4b9))
* **main:** release 0.50.2 ([#361](https://github.com/branes-ai/cortex/issues/361)) ([b0e10e7](https://github.com/branes-ai/cortex/commit/b0e10e70732bd5983dc7230304a8cd7f357385da))
* **main:** release 0.50.3 ([#363](https://github.com/branes-ai/cortex/issues/363)) ([8c9eaa7](https://github.com/branes-ai/cortex/commit/8c9eaa7d3b95a09050ad2437d4966276655ecb78))
* **main:** release 0.51.0 ([#368](https://github.com/branes-ai/cortex/issues/368)) ([391ae00](https://github.com/branes-ai/cortex/commit/391ae003f19a4fa427f75508b8c8bc5919a3c08d))
* **main:** release 0.51.1 ([#370](https://github.com/branes-ai/cortex/issues/370)) ([b5cc249](https://github.com/branes-ai/cortex/commit/b5cc249f3ab3a1c00291e7b7868830e178f2464d))
* **main:** release 0.52.0 ([#386](https://github.com/branes-ai/cortex/issues/386)) ([de764ef](https://github.com/branes-ai/cortex/commit/de764effde3ec0a0f656b635a94ca78c7ad88175))
* **main:** release 0.53.0 ([#389](https://github.com/branes-ai/cortex/issues/389)) ([c69f164](https://github.com/branes-ai/cortex/commit/c69f16438c53741d05a3b718d62759d2bafa8b8a))
* **main:** release 0.53.1 ([#391](https://github.com/branes-ai/cortex/issues/391)) ([35ae394](https://github.com/branes-ai/cortex/commit/35ae39415a1aea909c065dce7dc7f65b7b01dd0b))
* **main:** release 0.54.0 ([#392](https://github.com/branes-ai/cortex/issues/392)) ([1144f38](https://github.com/branes-ai/cortex/commit/1144f3851cbe3a5777a0fafbd2b6abacb3c5783c))
* **main:** release 0.55.0 ([#395](https://github.com/branes-ai/cortex/issues/395)) ([18067b3](https://github.com/branes-ai/cortex/commit/18067b3d0803cf1ba5e03e059d62e7a060ebec73))
* **main:** release 0.56.0 ([#397](https://github.com/branes-ai/cortex/issues/397)) ([8747fb7](https://github.com/branes-ai/cortex/commit/8747fb7a47937863b57b69f47c83be405bbf206e))
* **main:** release 0.57.0 ([#401](https://github.com/branes-ai/cortex/issues/401)) ([c39daf0](https://github.com/branes-ai/cortex/commit/c39daf01231ab88d3f1bfb679d009c98d2351f3c))
* **main:** release 0.58.0 ([#402](https://github.com/branes-ai/cortex/issues/402)) ([1dd9b7c](https://github.com/branes-ai/cortex/commit/1dd9b7cd848075b8702b80bc9ffc519411d6449b))
* **main:** release 0.59.0 ([#404](https://github.com/branes-ai/cortex/issues/404)) ([4a6eb9c](https://github.com/branes-ai/cortex/commit/4a6eb9c9ca5951eb1399b8d19155279426d9e046))
* **main:** release 0.59.1 ([#408](https://github.com/branes-ai/cortex/issues/408)) ([4d3ee16](https://github.com/branes-ai/cortex/commit/4d3ee162cafed05b9570e805f320994f12e7a366))
* **main:** release 0.6.0 ([#142](https://github.com/branes-ai/cortex/issues/142)) ([b6ece0c](https://github.com/branes-ai/cortex/commit/b6ece0ca1ba9693547b96a8a6452a2502254d26e))
* **main:** release 0.7.0 ([#144](https://github.com/branes-ai/cortex/issues/144)) ([7079108](https://github.com/branes-ai/cortex/commit/70791082fffe8711c6e9a3e48bea536de2310f7e))
* **main:** release 0.8.0 ([#146](https://github.com/branes-ai/cortex/issues/146)) ([711ed65](https://github.com/branes-ai/cortex/commit/711ed657eaaa4f79210d6b2239f55daa3c634270))
* **main:** release 0.9.0 ([#148](https://github.com/branes-ai/cortex/issues/148)) ([16a79e6](https://github.com/branes-ai/cortex/commit/16a79e6dfb95df430d731cfa774f468b32efea22))
* **release:** add commitlint + release-please for SemVer ([d5cf0e1](https://github.com/branes-ai/cortex/commit/d5cf0e15eeaa0f1081dc34133011d4c12e034163))
* **release:** add commitlint + release-please for SemVer ([8e69d68](https://github.com/branes-ai/cortex/commit/8e69d68d05baf1d9dd24ad3fde85e9086bd109f1))
* **release:** pin initial-version to 0.1.0 (alpha pre-1.0 semantics) ([#121](https://github.com/branes-ai/cortex/issues/121)) ([39d8309](https://github.com/branes-ai/cortex/commit/39d8309f58e506278152e96a3af1146fff7a9544))
* **repo:** track team Claude config — docs-site build allowlist + auto-build ([#393](https://github.com/branes-ai/cortex/issues/393)) ([c867a1f](https://github.com/branes-ai/cortex/commit/c867a1f42961b98896c76eca1394c90b7ea85c3a))
* **testing:** add regression infra — coverage + latency budget framework ([#123](https://github.com/branes-ai/cortex/issues/123)) ([d0c9805](https://github.com/branes-ai/cortex/commit/d0c98051e60966f247b6268384b0e7a7bd428f5f))
* **toolchain:** bump Corrosion v0.4.2 -&gt; v0.6.1 ([164d8a5](https://github.com/branes-ai/cortex/commit/164d8a568f43cce5a5349fe28bafc76a49847fca))
* **toolchain:** pin Rust 1.83.0 and unblock skeleton build ([2f83879](https://github.com/branes-ai/cortex/commit/2f83879aa3737ff2fc1649e011bd50cdfc53467d))
* **toolchain:** pin Rust 1.83.0 and unblock skeleton build ([c84504e](https://github.com/branes-ai/cortex/commit/c84504ec82741a87a54fc73ed917543054930b92))
* **tools:** add the R-IEKF system-level NEES diagnostic ([#212](https://github.com/branes-ai/cortex/issues/212)) ([#360](https://github.com/branes-ai/cortex/issues/360)) ([76b677f](https://github.com/branes-ai/cortex/commit/76b677fac2ac9906e004a59796ac8e3c9af83426))
* wrapup 2026-05-24 — Phase 1 MVP session log + CHANGELOG marker ([#139](https://github.com/branes-ai/cortex/issues/139)) ([f0cab34](https://github.com/branes-ai/cortex/commit/f0cab347a6434489e350da97367ac70b1304cc81))
* wrapup Phase 0 — add CHANGELOG.md, session log, gitignore cargo target ([8abbaff](https://github.com/branes-ai/cortex/commit/8abbaff6a14c39a1d4291bfc20b229fa0ca70b2a))
* wrapup Phase 0 — CHANGELOG.md, session log, gitignore target/ ([dbc353f](https://github.com/branes-ai/cortex/commit/dbc353f40d29dd1706daea3f46ce69a4864a27dd))

## [0.59.1](https://github.com/branes-ai/cortex/compare/v0.59.0...v0.59.1) (2026-06-17)


### Documentation

* improve overview phrasing and trigger documentation rebuild ([2978a7d](https://github.com/branes-ai/cortex/commit/2978a7d7cc06af5850f191eda8196d048af5495c))

## [0.59.0](https://github.com/branes-ai/cortex/compare/v0.58.0...v0.59.0) (2026-06-17)


### Features

* **tools:** s6 msckf-update inspector — real EuRoC updates → state/covariance ([#403](https://github.com/branes-ai/cortex/issues/403)) ([bc60f6e](https://github.com/branes-ai/cortex/commit/bc60f6ee72a8b76fb0334cfeeccdeda0c809fce4))

## [0.58.0](https://github.com/branes-ai/cortex/compare/v0.57.0...v0.58.0) (2026-06-17)


### Features

* **sdk:** implement Phase C 6-DoF online extrinsic calibration for MsckfInvariantBackend ([9094c64](https://github.com/branes-ai/cortex/commit/9094c64a28fb3934acfc366f720f096c0953ffcf))
* **sdk:** Phase C - 6-DoF online extrinsic calibration for MsckfInvariantBackend ([#400](https://github.com/branes-ai/cortex/issues/400)) ([9094c64](https://github.com/branes-ai/cortex/commit/9094c64a28fb3934acfc366f720f096c0953ffcf))

## [0.57.0](https://github.com/branes-ai/cortex/compare/v0.56.0...v0.57.0) (2026-06-17)


### Features

* **tools:** s5 triangulation inspector — real EuRoC tracks+poses → 3D landmarks ([#398](https://github.com/branes-ai/cortex/issues/398)) ([a3999ab](https://github.com/branes-ai/cortex/commit/a3999ab112a0af40c60ef68bc7bcd3595861438e))

## [0.56.0](https://github.com/branes-ai/cortex/compare/v0.55.0...v0.56.0) (2026-06-16)


### Features

* **sdk:** add 6-DoF rigid camera extrinsics to MsckfInvariantBackend ([#396](https://github.com/branes-ai/cortex/issues/396)) ([586081d](https://github.com/branes-ai/cortex/commit/586081de6f1c03bbfee207936a3e9baad246d32e))


### Reverts

* **docs:** restore original working vio_run.jsonl ([8762ee7](https://github.com/branes-ai/cortex/commit/8762ee7c1efc7e12326f38da3e066b15125ae8ea))

## [0.55.0](https://github.com/branes-ai/cortex/compare/v0.54.0...v0.55.0) (2026-06-16)


### Features

* **sdk:** add 6-DoF rigid camera extrinsics to MsckfInvariantBackend ([efb48c5](https://github.com/branes-ai/cortex/commit/efb48c5d123d24a4946ddf0a83f72e85468e7067))
* **tools:** s0 sensor-model inspector — real EuRoC frame/IMU → camera & noise contract ([#394](https://github.com/branes-ai/cortex/issues/394)) ([6a88cad](https://github.com/branes-ai/cortex/commit/6a88cad9c55f5487938b83ebcebd2f91e0c08ec9))

## [0.54.0](https://github.com/branes-ai/cortex/compare/v0.53.1...v0.54.0) (2026-06-16)


### Features

* **tools:** S4 frontend inspector — real EuRoC FAST+KLT tracks, visualized ([#390](https://github.com/branes-ai/cortex/issues/390)) ([e2add78](https://github.com/branes-ai/cortex/commit/e2add78cbc7bfb6e7a15444121714e0b8f8f7419))


### Maintenance

* **repo:** track team Claude config — docs-site build allowlist + auto-build ([#393](https://github.com/branes-ai/cortex/issues/393)) ([c867a1f](https://github.com/branes-ai/cortex/commit/c867a1f42961b98896c76eca1394c90b7ea85c3a))

## [0.53.1](https://github.com/branes-ai/cortex/compare/v0.53.0...v0.53.1) (2026-06-16)


### Bug Fixes

* **sdk:** resolve R-IEKF continuous divergence ([#366](https://github.com/branes-ai/cortex/issues/366)) ([7f024dc](https://github.com/branes-ai/cortex/commit/7f024dc61533a25def5f88b5c59a021b6925f38c))
* **sdk:** resolve R-IEKF continuous divergence ([#366](https://github.com/branes-ai/cortex/issues/366)) ([#388](https://github.com/branes-ai/cortex/issues/388)) ([7f024dc](https://github.com/branes-ai/cortex/commit/7f024dc61533a25def5f88b5c59a021b6925f38c))

## [0.53.0](https://github.com/branes-ai/cortex/compare/v0.52.0...v0.53.0) (2026-06-16)


### Features

* **tools:** --trace tap for EuRoC asl_replay — real per-stage trace dumps ([#387](https://github.com/branes-ai/cortex/issues/387)) ([eb8f9c0](https://github.com/branes-ai/cortex/commit/eb8f9c05134ba5046a2696199552ee383863002e))

## [0.52.0](https://github.com/branes-ai/cortex/compare/v0.51.1...v0.52.0) (2026-06-15)


### Features

* **tools:** inter-stage VIO trace bus — typed, human-readable per-stage I/O records ([#385](https://github.com/branes-ai/cortex/issues/385)) ([ce63b06](https://github.com/branes-ai/cortex/commit/ce63b0660fafefa42850539f087939fea8d4e484))

## [0.51.1](https://github.com/branes-ai/cortex/compare/v0.51.0...v0.51.1) (2026-06-15)


### Documentation

* session log + changelog for the R-IEKF/FEJ/verdict arc ([#369](https://github.com/branes-ai/cortex/issues/369)) ([8383b41](https://github.com/branes-ai/cortex/commit/8383b416f377dd6ec715b623295c12d155cf815b))

## [0.51.0](https://github.com/branes-ai/cortex/compare/v0.50.3...v0.51.0) (2026-06-15)


### Features

* **sdk:** R-IEKF VIO backend adapter + continuous-update divergence localization ([#365](https://github.com/branes-ai/cortex/issues/365)) ([#367](https://github.com/branes-ai/cortex/issues/367)) ([51ca579](https://github.com/branes-ai/cortex/commit/51ca579834aa30e6126660eb3207acb7c43d4600))

## [0.50.3](https://github.com/branes-ai/cortex/compare/v0.50.2...v0.50.3) (2026-06-14)


### Documentation

* render LaTeX math on the site (KaTeX) + camera-calibration walkthrough ([#362](https://github.com/branes-ai/cortex/issues/362)) ([675a414](https://github.com/branes-ai/cortex/commit/675a414208f3104ba268b81a25601e601462fdba))
* **site:** widen the triangulation baseline so the 3D view reads ([#364](https://github.com/branes-ai/cortex/issues/364)) ([7536a5e](https://github.com/branes-ai/cortex/commit/7536a5eec984cde232c320174b7ab82c0540b2ce))

## [0.50.2](https://github.com/branes-ai/cortex/compare/v0.50.1...v0.50.2) (2026-06-14)


### Maintenance

* **tools:** add the R-IEKF system-level NEES diagnostic ([#212](https://github.com/branes-ai/cortex/issues/212)) ([#360](https://github.com/branes-ai/cortex/issues/360)) ([76b677f](https://github.com/branes-ai/cortex/commit/76b677fac2ac9906e004a59796ac8e3c9af83426))

## [0.50.1](https://github.com/branes-ai/cortex/compare/v0.50.0...v0.50.1) (2026-06-13)


### Tests

* **sdk:** drive the observability gauge check through the production CameraUpdater ([#358](https://github.com/branes-ai/cortex/issues/358)) ([7607e32](https://github.com/branes-ai/cortex/commit/7607e32cbc2b4e656e52b59024abc8d251a59e82))

## [0.50.0](https://github.com/branes-ai/cortex/compare/v0.49.0...v0.50.0) (2026-06-13)


### Features

* **sdk:** add the R-IEKF invariant camera measurement update ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#356](https://github.com/branes-ai/cortex/issues/356)) ([03d34ae](https://github.com/branes-ai/cortex/commit/03d34ae0a7fa1012bb91ca879de1c5da4bfa2134))
* **sdk:** add the R-IEKF invariant IMU propagation on SE2(3) ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#354](https://github.com/branes-ai/cortex/issues/354)) ([4b41726](https://github.com/branes-ai/cortex/commit/4b417268efba64020fda3e3165af257ceb8d6933))
* **sdk:** wire the MsckfInvariantBackend (R-IEKF filter) ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#357](https://github.com/branes-ai/cortex/issues/357)) ([30cd651](https://github.com/branes-ai/cortex/commit/30cd65130705558817c1ea2df03f4593f747786b))

## [0.49.0](https://github.com/branes-ai/cortex/compare/v0.48.0...v0.49.0) (2026-06-13)


### Features

* **math:** add SE2(3) extended-pose Lie group — the R-IEKF state manifold ([#347](https://github.com/branes-ai/cortex/issues/347)) ([#353](https://github.com/branes-ai/cortex/issues/353)) ([866fa8d](https://github.com/branes-ai/cortex/commit/866fa8d8d04f1ef071b08358f41bb84eaf1a624c))
* **sdk:** prove the right-invariant propagation Φ is state-independent (R-IEKF Phase A, [#348](https://github.com/branes-ai/cortex/issues/348)) ([#351](https://github.com/branes-ai/cortex/issues/351)) ([8c48e06](https://github.com/branes-ai/cortex/commit/8c48e060d367bfcbb5d497c7150453a959e41366))

## [0.48.0](https://github.com/branes-ai/cortex/compare/v0.47.1...v0.48.0) (2026-06-13)


### Features

* **sdk:** R-IEKF Phase-A gate — invariant parameterization flattens the yaw leak ([#348](https://github.com/branes-ai/cortex/issues/348)) ([#349](https://github.com/branes-ai/cortex/issues/349)) ([5350f9e](https://github.com/branes-ai/cortex/commit/5350f9ec77daa70fe202012ccf38940b9d98f68c))

## [0.47.1](https://github.com/branes-ai/cortex/compare/v0.47.0...v0.47.1) (2026-06-13)


### Documentation

* record the [#212](https://github.com/branes-ai/cortex/issues/212) observability investigation — yaw leak localized, FEJ refuted, R-IEKF next ([#345](https://github.com/branes-ai/cortex/issues/345)) ([7dff9f5](https://github.com/branes-ai/cortex/commit/7dff9f503cc856366a48747e6a6f7f767d6c6d09))

## [0.47.0](https://github.com/branes-ai/cortex/compare/v0.46.0...v0.47.0) (2026-06-13)


### Features

* **sdk:** observability null-space probe — localizes the [#212](https://github.com/branes-ai/cortex/issues/212) NEES leak to yaw ([#337](https://github.com/branes-ai/cortex/issues/337)) ([#338](https://github.com/branes-ai/cortex/issues/338)) ([5e5c3a1](https://github.com/branes-ai/cortex/commit/5e5c3a1e1ad610612dc06c2974de009a43720dfa))

## [0.46.0](https://github.com/branes-ai/cortex/compare/v0.45.2...v0.46.0) (2026-06-12)


### Features

* **sdk:** online extrinsic calibration (S10-as-state), mechanism + synthetic proof ([#332](https://github.com/branes-ai/cortex/issues/332)) ([#333](https://github.com/branes-ai/cortex/issues/333)) ([24cb8b3](https://github.com/branes-ai/cortex/commit/24cb8b3fc086458d47ba896fbebabcfc5356414b))
* **test:** EuRoC verdict — online extrinsic calibration cleared as [#212](https://github.com/branes-ai/cortex/issues/212) driver ([#332](https://github.com/branes-ai/cortex/issues/332)) ([#343](https://github.com/branes-ai/cortex/issues/343)) ([17f7570](https://github.com/branes-ai/cortex/commit/17f757010ae8bea0aeedc641254d50399a0b3fd2))

## [0.45.2](https://github.com/branes-ai/cortex/compare/v0.45.1...v0.45.2) (2026-06-12)


### Bug Fixes

* **docs:** make topology + VIO stage-flow SVGs readable in dark theme ([#334](https://github.com/branes-ai/cortex/issues/334)) ([623b34e](https://github.com/branes-ai/cortex/commit/623b34ec4924da28cda583b0a2fe240a792b4717))

## [0.45.1](https://github.com/branes-ai/cortex/compare/v0.45.0...v0.45.1) (2026-06-12)


### Documentation

* add VIO S0–S10 stage-flow SVG and replace the pipeline ASCII dataflow ([7ffb706](https://github.com/branes-ai/cortex/commit/7ffb706a8c242331d13863a76f454f86f1848edb))
* add VIO S0–S10 stage-flow SVG; replace pipeline ASCII dataflow ([#330](https://github.com/branes-ai/cortex/issues/330)) ([7ffb706](https://github.com/branes-ai/cortex/commit/7ffb706a8c242331d13863a76f454f86f1848edb))

## [0.45.0](https://github.com/branes-ai/cortex/compare/v0.44.0...v0.45.0) (2026-06-12)


### Features

* **sdk:** add S10 calibration-uncertainty R-term; measure it end-to-end on V2_03 ([#328](https://github.com/branes-ai/cortex/issues/328)) ([670defd](https://github.com/branes-ai/cortex/commit/670defda056f5d8dcc0d6dda69a26f3753f3df6b))


### Documentation

* render architecture topology as SVG; wire in orphaned S0 fisheye figures ([#327](https://github.com/branes-ai/cortex/issues/327)) ([4aa72c1](https://github.com/branes-ai/cortex/commit/4aa72c185c44b0f83b224b0ea15d4a9d491fe07f))

## [0.44.0](https://github.com/branes-ai/cortex/compare/v0.43.0...v0.44.0) (2026-06-11)


### Features

* **sdk:** S6 update probe; lock S3/S5/S6/S9 contracts by measurement ([#325](https://github.com/branes-ai/cortex/issues/325)) ([bb8b591](https://github.com/branes-ai/cortex/commit/bb8b591438aaff34291ce3fe6292f5369e54c23e))

## [0.43.0](https://github.com/branes-ai/cortex/compare/v0.42.0...v0.43.0) (2026-06-06)


### Features

* add the S4 forward-backward and S5 parallax outlier gates ([#323](https://github.com/branes-ai/cortex/issues/323)) ([ed125d5](https://github.com/branes-ai/cortex/commit/ed125d5e6cee81fc1a33cb534f0446a271ad6b7c))

## [0.42.0](https://github.com/branes-ai/cortex/compare/v0.41.0...v0.42.0) (2026-06-06)


### Features

* wire the S4 visual-frontend probe + data-backed page ([#321](https://github.com/branes-ai/cortex/issues/321)) ([2e30b15](https://github.com/branes-ai/cortex/commit/2e30b15a25f589207af75c1eb59ad33f1c3fb3ee))

## [0.41.0](https://github.com/branes-ai/cortex/compare/v0.40.1...v0.41.0) (2026-06-06)


### Features

* complete the structural stage pages (S3, S4, S7-S9) ([#319](https://github.com/branes-ai/cortex/issues/319)) ([13be4c6](https://github.com/branes-ai/cortex/commit/13be4c60f20cdd65d4c2abb053f8f02b28531ead))

## [0.40.1](https://github.com/branes-ai/cortex/compare/v0.40.0...v0.40.1) (2026-06-06)


### Documentation

* migrate diagnostic-methodology as the section's method primer ([#316](https://github.com/branes-ai/cortex/issues/316)) ([0097585](https://github.com/branes-ai/cortex/commit/009758509072ac129b5dccd409ac45a90dbd1ccb))
* **session:** wrap up 2026-06-05 — VIO pipeline docs + 3D viz ([#318](https://github.com/branes-ai/cortex/issues/318)) ([d5d8942](https://github.com/branes-ai/cortex/commit/d5d894257c4b5cc732b7ea95fee81157aa43934a))
* **session:** wrap up 2026-06-05 — VIO pipeline docs section + 3D viz ([d5d8942](https://github.com/branes-ai/cortex/commit/d5d894257c4b5cc732b7ea95fee81157aa43934a))
* **site:** migrate the diagnostic-methodology doc as a section page ([0097585](https://github.com/branes-ai/cortex/commit/009758509072ac129b5dccd409ac45a90dbd1ccb))

## [0.40.0](https://github.com/branes-ai/cortex/compare/v0.39.4...v0.40.0) (2026-06-06)


### Features

* S5 feature-triangulation probe + stage page (parallax gate, quantified) ([#313](https://github.com/branes-ai/cortex/issues/313)) ([cb2c938](https://github.com/branes-ai/cortex/commit/cb2c938fe4e7d2608abe719c6abbedb7076328da))


### Documentation

* S1 initialization stage page (algorithm sound, seed suspect) ([#315](https://github.com/branes-ai/cortex/issues/315)) ([936cdb1](https://github.com/branes-ai/cortex/commit/936cdb1925e23c752cba8971c13e45dbf63cfd5f))
* **site:** add the S1 initialization stage page ([936cdb1](https://github.com/branes-ai/cortex/commit/936cdb1925e23c752cba8971c13e45dbf63cfd5f))

## [0.39.4](https://github.com/branes-ai/cortex/compare/v0.39.3...v0.39.4) (2026-06-05)


### Documentation

* S2 IMU-propagation stage page (a cleared [#212](https://github.com/branes-ai/cortex/issues/212) candidate) ([#311](https://github.com/branes-ai/cortex/issues/311)) ([b930f07](https://github.com/branes-ai/cortex/commit/b930f078d1a88953ba6e36cb4bcc731328d0066e))
* **site:** add S2 IMU-propagation stage page (a cleared [#212](https://github.com/branes-ai/cortex/issues/212) candidate) ([b930f07](https://github.com/branes-ai/cortex/commit/b930f078d1a88953ba6e36cb4bcc731328d0066e))

## [0.39.3](https://github.com/branes-ai/cortex/compare/v0.39.2...v0.39.3) (2026-06-05)


### Documentation

* S6 MSCKF-update stage + consistency-analysis page ([#309](https://github.com/branes-ai/cortex/issues/309)) ([5607dd1](https://github.com/branes-ai/cortex/commit/5607dd10cb7172e021beb71a0845c978c07a4e7e))

## [0.39.2](https://github.com/branes-ai/cortex/compare/v0.39.1...v0.39.2) (2026-06-05)


### Documentation

* S10 online-calibration stage page (the [#212](https://github.com/branes-ai/cortex/issues/212) centerpiece) ([#307](https://github.com/branes-ai/cortex/issues/307)) ([40b483b](https://github.com/branes-ai/cortex/commit/40b483beae187f0d21a3e6e9e47be68cbbb2981f))
* **site:** add the S10 online-calibration stage page ([#212](https://github.com/branes-ai/cortex/issues/212) centerpiece) ([40b483b](https://github.com/branes-ai/cortex/commit/40b483beae187f0d21a3e6e9e47be68cbbb2981f))

## [0.39.1](https://github.com/branes-ai/cortex/compare/v0.39.0...v0.39.1) (2026-06-05)


### Documentation

* VIO pipeline section (overview, metrics, S0, status) + issue-ref policy ([#305](https://github.com/branes-ai/cortex/issues/305)) ([1212cde](https://github.com/branes-ai/cortex/commit/1212cde93c900cdbdd0c17b96ae7f9342161cf32))

## [0.39.0](https://github.com/branes-ai/cortex/compare/v0.38.0...v0.39.0) (2026-06-05)


### Features

* landmark cloud + camera frustum in the 3D viewer ([#303](https://github.com/branes-ai/cortex/issues/303)) ([86fc1c7](https://github.com/branes-ai/cortex/commit/86fc1c7685209bfa9fb4d441eb2f09ab20143aed))

## [0.38.0](https://github.com/branes-ai/cortex/compare/v0.37.0...v0.38.0) (2026-06-05)


### Features

* 3D drone path/pose viewer (Phase 0 + Phase 1 prototype) ([#302](https://github.com/branes-ai/cortex/issues/302)) ([7e47ce3](https://github.com/branes-ai/cortex/commit/7e47ce3eeac87bd9159d39f6e2aa04de88f786c5))
* **tools:** depth-colored scene overlay + vio_pipeline how-to guide ([#300](https://github.com/branes-ai/cortex/issues/300)) ([648ecc3](https://github.com/branes-ai/cortex/commit/648ecc3c2dc9eeb25d8df3d2fa53fc01f4d62cf0))

## [0.37.0](https://github.com/branes-ai/cortex/compare/v0.36.0...v0.37.0) (2026-06-05)


### Features

* **tools:** add EuRoC source + scene-video metric overlay to vio_pipeline ([#292](https://github.com/branes-ai/cortex/issues/292)) ([94fa599](https://github.com/branes-ai/cortex/commit/94fa5998d5514dd9f22c5296f7b73a04e15b6da4))
* **tools:** wire S10 (calibration) stage-probe to quantify the [#212](https://github.com/branes-ai/cortex/issues/212) Rx4 ([#295](https://github.com/branes-ai/cortex/issues/295)) ([7ccb85c](https://github.com/branes-ai/cortex/commit/7ccb85ced059f85c3d70920b096339a5675324ae))


### Bug Fixes

* **tools:** create the --out directory so artifacts are not silently dropped ([#298](https://github.com/branes-ai/cortex/issues/298)) ([a1e7fb1](https://github.com/branes-ai/cortex/commit/a1e7fb15f5f6f75ed681d0c7b80a91d6b1f9544c))

## [0.36.0](https://github.com/branes-ai/cortex/compare/v0.35.0...v0.36.0) (2026-06-05)


### Features

* **tools:** end-to-end VIO noise-&gt;robustness demo (synthetic source) ([#290](https://github.com/branes-ai/cortex/issues/290)) ([2492262](https://github.com/branes-ai/cortex/commit/2492262ef21a518011f895d62f8d6ba3115fa185))
* **tools:** wire S1 (initialization) stage-probe ([#286](https://github.com/branes-ai/cortex/issues/286)) ([fa3d92b](https://github.com/branes-ai/cortex/commit/fa3d92ba708b3402ffd7d3d0a8f66af74c369717))

## [0.35.0](https://github.com/branes-ai/cortex/compare/v0.34.0...v0.35.0) (2026-06-04)


### Features

* **tools:** add VIO stage-probe utilities — S0 sensor-model + S2 propagation ([#283](https://github.com/branes-ai/cortex/issues/283)) ([4a2c1ef](https://github.com/branes-ai/cortex/commit/4a2c1efc029e799623d7ebf9b523c7041b86f7f6))

## [0.34.0](https://github.com/branes-ai/cortex/compare/v0.33.0...v0.34.0) (2026-06-03)


### Features

* **sdk:** Q/R consistency-sweep knobs + [#212](https://github.com/branes-ai/cortex/issues/212) root-cause diagnosis ([#279](https://github.com/branes-ai/cortex/issues/279)) ([910f1c6](https://github.com/branes-ai/cortex/commit/910f1c6323a75bbc361f24a45693aecbb8bac997))

## [0.33.0](https://github.com/branes-ai/cortex/compare/v0.32.0...v0.33.0) (2026-06-03)


### Features

* **eval:** add IMU Allan-variance noise characterization ([#268](https://github.com/branes-ai/cortex/issues/268)) ([#276](https://github.com/branes-ai/cortex/issues/276)) ([32b0c4b](https://github.com/branes-ai/cortex/commit/32b0c4b258416c5e9b1613806186144ea21e5478))


### Documentation

* process documentation ([3aacba2](https://github.com/branes-ai/cortex/commit/3aacba2cb25e05006f21d0a16311dc9befa30c1b))

## [0.32.0](https://github.com/branes-ai/cortex/compare/v0.31.0...v0.32.0) (2026-06-02)


### Features

* **eval:** add NEES-against-ground-truth consistency (closes [#264](https://github.com/branes-ai/cortex/issues/264)) ([#273](https://github.com/branes-ai/cortex/issues/273)) ([66cc815](https://github.com/branes-ai/cortex/commit/66cc8158b84dccefce1f4dc1a520338a25cee3af))


### Documentation

* **assessment:** a plain-language tutorial on NEES/NIS filter consistency ([acc8f8d](https://github.com/branes-ai/cortex/commit/acc8f8d1acb5d09d99995af9416ae90f6e4c29bb))
* **assessment:** plain-language NEES/NIS filter-consistency tutorial ([#274](https://github.com/branes-ai/cortex/issues/274)) ([acc8f8d](https://github.com/branes-ai/cortex/commit/acc8f8d1acb5d09d99995af9416ae90f6e4c29bb))

## [0.31.0](https://github.com/branes-ai/cortex/compare/v0.30.2...v0.31.0) (2026-06-02)


### Features

* **eval:** add NEES/NIS filter-consistency statistics ([4a1e361](https://github.com/branes-ai/cortex/commit/4a1e3615eef26993b48d44a271278170250872f3))
* **eval:** NEES/NIS filter-consistency statistics ([#264](https://github.com/branes-ai/cortex/issues/264)) ([#271](https://github.com/branes-ai/cortex/issues/271)) ([4a1e361](https://github.com/branes-ai/cortex/commit/4a1e3615eef26993b48d44a271278170250872f3))
* **sdk:** surface per-update NIS for filter-consistency monitoring ([3da99b0](https://github.com/branes-ai/cortex/commit/3da99b02f81b90ab8dd1a1c34faa9bab0ecae0cc))
* **sdk:** surface per-update NIS for filter-consistency monitoring ([#264](https://github.com/branes-ai/cortex/issues/264)) ([#272](https://github.com/branes-ai/cortex/issues/272)) ([3da99b0](https://github.com/branes-ai/cortex/commit/3da99b02f81b90ab8dd1a1c34faa9bab0ecae0cc))


### Documentation

* **assessment:** VIO diagnostic methodology + instrumentation build list ([#269](https://github.com/branes-ai/cortex/issues/269)) ([755e694](https://github.com/branes-ai/cortex/commit/755e69404f79c567c31c2f6c4369d4828466ada4))

## [0.30.2](https://github.com/branes-ai/cortex/compare/v0.30.1...v0.30.2) (2026-06-02)


### Bug Fixes

* **test:** make the vio_latency budget informational on CI ([#240](https://github.com/branes-ai/cortex/issues/240)) ([#259](https://github.com/branes-ai/cortex/issues/259)) ([22e01c7](https://github.com/branes-ai/cortex/commit/22e01c720699d92a7dde434d54ad17aa7b189f9d))

## [0.30.1](https://github.com/branes-ai/cortex/compare/v0.30.0...v0.30.1) (2026-06-02)


### Maintenance

* **main:** release 0.30.0 ([#257](https://github.com/branes-ai/cortex/issues/257)) ([1eceabc](https://github.com/branes-ai/cortex/commit/1eceabc36793955f64bc193012cf0183086b9eaa))

## [0.30.0](https://github.com/branes-ai/cortex/compare/v0.29.0...v0.30.0) (2026-06-02)


### Features

* **sdk:** force-dynamic VI-init validation on MH_05 (real-data [#211](https://github.com/branes-ai/cortex/issues/211)) ([#246](https://github.com/branes-ai/cortex/issues/246)) ([d736d4f](https://github.com/branes-ai/cortex/commit/d736d4f6680814a800eff92120adeeb421173953))


### Bug Fixes

* **docs:** re-anchor EuRoC gate extraction (unbreak Deploy Documentation) ([#256](https://github.com/branes-ai/cortex/issues/256)) ([f99c4e1](https://github.com/branes-ai/cortex/commit/f99c4e12d0d604e4ef5f93f1f0c5e65a56f7d198))
* **docs:** re-anchor the EuRoC gate extraction to the run_euroc_replay call ([f99c4e1](https://github.com/branes-ai/cortex/commit/f99c4e12d0d604e4ef5f93f1f0c5e65a56f7d198))
* **sdk:** bootstrap dynamic-init SfM on the widest baseline ([df517c4](https://github.com/branes-ai/cortex/commit/df517c4a9bec74659f72ec880aeb72105be329aa))
* **sdk:** bootstrap dynamic-init SfM on the widest baseline ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#251](https://github.com/branes-ai/cortex/issues/251)) ([df517c4](https://github.com/branes-ai/cortex/commit/df517c4a9bec74659f72ec880aeb72105be329aa))
* **sdk:** correct dynamic-init gravity sign — real-data divergence → bounded ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#252](https://github.com/branes-ai/cortex/issues/252)) ([1876ba0](https://github.com/branes-ai/cortex/commit/1876ba064efce0f0cfef26f0e7cd6ef384053b54))
* **sdk:** correct dynamic-init gravity sign + widest-parallax bootstrap ([1876ba0](https://github.com/branes-ai/cortex/commit/1876ba064efce0f0cfef26f0e7cd6ef384053b54))
* **test:** split euroc-moving-start local decls (set -u unbound var) ([#243](https://github.com/branes-ai/cortex/issues/243)) ([71de21c](https://github.com/branes-ai/cortex/commit/71de21c46917ace4660720d05edabad21bb6ba2d))


### Documentation

* **test:** explain pyramidal KLT + add the numerical failure-mode taxonomy ([c8f8ce9](https://github.com/branes-ai/cortex/commit/c8f8ce97e025bddd3074f345ce0b4bb9c417a376))
* **test:** pyramidal KLT explainer + EuRoC numerical failure-mode taxonomy ([#255](https://github.com/branes-ai/cortex/issues/255)) ([c8f8ce9](https://github.com/branes-ai/cortex/commit/c8f8ce97e025bddd3074f345ce0b4bb9c417a376))
* **test:** vio_euroc goal & architecture ([#249](https://github.com/branes-ai/cortex/issues/249)) ([0f88066](https://github.com/branes-ai/cortex/commit/0f8806675941d355ce7fee804bf1d909168f4282))


### Tests

* **sdk:** dataset-gated MH_05/V2_03 moving-start dynamic-init validation ([#241](https://github.com/branes-ai/cortex/issues/241)) ([20fe9db](https://github.com/branes-ai/cortex/commit/20fe9dbaf5c3c49a9c73a46b76d585a09bc452c3))
* **sdk:** dynamic-init diagnostics to localize the real-data divergence ([#247](https://github.com/branes-ai/cortex/issues/247)) ([#248](https://github.com/branes-ai/cortex/issues/248)) ([dbd90b4](https://github.com/branes-ai/cortex/commit/dbd90b488aba6604177f2daad2e45ef326c4d16c))
* **sdk:** instrument dynamic-init attempts ([#247](https://github.com/branes-ai/cortex/issues/247) localization) ([#250](https://github.com/branes-ai/cortex/issues/250)) ([7b1493b](https://github.com/branes-ai/cortex/commit/7b1493bc9c8130a5edf7aa9eb15e57fdfc79c624))
* **sdk:** instrument dynamic-init attempts to localize why it won't fire ([7b1493b](https://github.com/branes-ai/cortex/commit/7b1493bc9c8130a5edf7aa9eb15e57fdfc79c624))
* **sdk:** real EuRoC extrinsics — MH_05 moving-start ATE validated (~0.79 m) ([#245](https://github.com/branes-ai/cortex/issues/245)) ([df70f84](https://github.com/branes-ai/cortex/commit/df70f84e103ae0a99e023ebb12fe349ac747c6ae))
* **sdk:** score EuRoC ATE from initialization onward ([#253](https://github.com/branes-ai/cortex/issues/253)) ([36a8334](https://github.com/branes-ai/cortex/commit/36a8334751ee11429e4d6f6fb47db8abd52908d2))
* **sdk:** score EuRoC ATE only from initialization onward ([36a8334](https://github.com/branes-ai/cortex/commit/36a8334751ee11429e4d6f6fb47db8abd52908d2))

## [0.29.0](https://github.com/branes-ai/cortex/compare/v0.28.0...v0.29.0) (2026-06-01)


### Features

* **sdk:** wire dynamic VI init into MsckfBackend (InitMethod::Dynamic) ([#238](https://github.com/branes-ai/cortex/issues/238)) ([8f50204](https://github.com/branes-ai/cortex/commit/8f5020405391567aca8b2f2552b3492fc88fd48b))

## [0.28.0](https://github.com/branes-ai/cortex/compare/v0.27.0...v0.28.0) (2026-06-01)


### Features

* **sdk:** perspective-n-point resectioning via DLT + RANSAC ([#236](https://github.com/branes-ai/cortex/issues/236)) ([542456c](https://github.com/branes-ai/cortex/commit/542456c186f514fcc7751862381d8ae5736efc88))

## [0.27.0](https://github.com/branes-ai/cortex/compare/v0.26.0...v0.27.0) (2026-05-31)


### Features

* **sdk:** resolve metric scale in ImuInitializer::try_dynamic ([#233](https://github.com/branes-ai/cortex/issues/233)) ([89b4c1d](https://github.com/branes-ai/cortex/commit/89b4c1d718b14099e7d93d74bc5ab8595f842c1a))

## [0.26.0](https://github.com/branes-ai/cortex/compare/v0.25.3...v0.26.0) (2026-05-31)


### Features

* **sdk:** two-view SfM bootstrap (essential matrix + RANSAC) ([#231](https://github.com/branes-ai/cortex/issues/231)) ([a5b3031](https://github.com/branes-ai/cortex/commit/a5b303177db45935f0f84053135b8aee480c478c))

## [0.25.3](https://github.com/branes-ai/cortex/compare/v0.25.2...v0.25.3) (2026-05-31)


### Bug Fixes

* **build:** link the Rust core in MSVC Debug by pinning the release CRT ([#226](https://github.com/branes-ai/cortex/issues/226)) ([f963950](https://github.com/branes-ai/cortex/commit/f9639500bad40564ea5c9bd264ce60ec1c84ac90))

## [0.25.2](https://github.com/branes-ai/cortex/compare/v0.25.1...v0.25.2) (2026-05-30)


### Bug Fixes

* **docs:** point the benchmark generator at the moved euroc test ([#223](https://github.com/branes-ai/cortex/issues/223)) ([023fb74](https://github.com/branes-ai/cortex/commit/023fb74e7df7b3adeeac12d97fefb1faaf706f27))


### Code Refactoring

* **test:** drop redundant layer/_test segments from test filenames ([#225](https://github.com/branes-ai/cortex/issues/225)) ([1cafd9a](https://github.com/branes-ai/cortex/commit/1cafd9a4f467cc38369fed72a53ddbc640b730a6))

## [0.25.1](https://github.com/branes-ai/cortex/compare/v0.25.0...v0.25.1) (2026-05-30)


### Build System

* **test:** generate test targets from a compile_all glob macro ([#220](https://github.com/branes-ai/cortex/issues/220)) ([9a4194d](https://github.com/branes-ai/cortex/commit/9a4194dc872d2cab30a87b61b0ba4a618a46bdef))


### Continuous Integration

* **windows:** build the MSVC job via Ninja + sccache ([#222](https://github.com/branes-ai/cortex/issues/222)) ([d39aed9](https://github.com/branes-ai/cortex/commit/d39aed9be8de648b86c2963d0cba07679601e3c3))

## [0.25.0](https://github.com/branes-ai/cortex/compare/v0.24.0...v0.25.0) (2026-05-30)


### Features

* **sdk:** square-root covariance MSCKF backend ([#218](https://github.com/branes-ai/cortex/issues/218)) ([b9a4478](https://github.com/branes-ai/cortex/commit/b9a4478692456dd0b60634314ef9feb5ecc945bd))

## [0.24.0](https://github.com/branes-ai/cortex/compare/v0.23.1...v0.24.0) (2026-05-30)


### Features

* **ci:** add a Windows MSVC build + test target ([#216](https://github.com/branes-ai/cortex/issues/216)) ([b79fa07](https://github.com/branes-ai/cortex/commit/b79fa07bcc13924a5c7f26f61ec049e370a2206d))

## [0.23.1](https://github.com/branes-ai/cortex/compare/v0.23.0...v0.23.1) (2026-05-30)


### Bug Fixes

* **math:** guard degenerate inputs in the Lie-group helpers ([#214](https://github.com/branes-ai/cortex/issues/214)) ([f51c3a9](https://github.com/branes-ai/cortex/commit/f51c3a91ec457b62d691d87ff834c48cef2dd33e)), closes [#161](https://github.com/branes-ai/cortex/issues/161)

## [0.23.0](https://github.com/branes-ai/cortex/compare/v0.22.0...v0.23.0) (2026-05-29)


### Features

* **bench:** vio_bench progress output + usage docs ([#208](https://github.com/branes-ai/cortex/issues/208)) ([e569a04](https://github.com/branes-ai/cortex/commit/e569a0413f8e41fca6e54d6e2bdc7af5564e2370))


### Bug Fixes

* **sdk:** set EuRoC extrinsics + gravity-align init; surface VIO divergence ([#209](https://github.com/branes-ai/cortex/issues/209)) ([91a92e8](https://github.com/branes-ai/cortex/commit/91a92e8547bbf2d12ccddd5a67f9a0be299b06f3))

## [0.22.0](https://github.com/branes-ai/cortex/compare/v0.21.0...v0.22.0) (2026-05-28)


### Features

* **bench:** graphs energy-model integration (cpu/gpu/kpu-t64) ([#204](https://github.com/branes-ai/cortex/issues/204)) ([9ec7b91](https://github.com/branes-ai/cortex/commit/9ec7b91931e3bed5b499fae299facb00c2030c79))
* **bench:** on-device energy backends — tegrastats + external meter ([#207](https://github.com/branes-ai/cortex/issues/207)) ([dda2ac7](https://github.com/branes-ai/cortex/commit/dda2ac7a6d67976f68c6feecb30462b743bc8db2))

## [0.21.0](https://github.com/branes-ai/cortex/compare/v0.20.3...v0.21.0) (2026-05-28)


### Features

* **bench:** vio_bench — euroc accuracy + latency + rapl energy report ([#201](https://github.com/branes-ai/cortex/issues/201)) ([a5abbcb](https://github.com/branes-ai/cortex/commit/a5abbcb448cfa93a629396e74f7709a5a53fc9d3))

## [0.20.3](https://github.com/branes-ai/cortex/compare/v0.20.2...v0.20.3) (2026-05-27)


### Documentation

* add PR template with the per-PR docs-update rule ([#196](https://github.com/branes-ai/cortex/issues/196)) ([183f3e1](https://github.com/branes-ai/cortex/commit/183f3e18d16cbd358cea4e45c7e2d78ae579e765))
* pin benchmark gate numbers to their source ([#197](https://github.com/branes-ai/cortex/issues/197)) ([707d5d6](https://github.com/branes-ai/cortex/commit/707d5d62cb581368fa8073459c16e943444fec8b))

## [0.20.2](https://github.com/branes-ai/cortex/compare/v0.20.1...v0.20.2) (2026-05-27)


### Documentation

* fix doubled base path on the API reference sidebar link ([#194](https://github.com/branes-ai/cortex/issues/194)) ([d878e47](https://github.com/branes-ai/cortex/commit/d878e47d7da41b97dc174c43700166927209519f))

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
