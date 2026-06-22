# VIO stage-probe utilities

Runnable utilities to study **one pipeline stage in isolation** — its
pre-conditions, post-conditions/invariants, and native-unit assessments. These
are study tools, **not tests** (no Catch2) and **not daemons** (no Zenoh): you
run one, perturb an input, and watch a problem-native metric move, to build
intuition for where a stage's contract bends.

They are the executable companion to the contract program in
[`docs/arch/vio-pipeline-canonical.md`](../docs/arch/vio-pipeline-canonical.md):
the canonical OpenVINS/MINS pipeline deconstructed into stages **S0–S10**, each
written as a contract. One executable per stage.

## End-to-end noise→robustness demo (`vio_pipeline`)

> **New to this tool?** See the guide: [docs/assessments/vio-pipeline-howto.md](../docs/assessments/vio-pipeline-howto.md) — what/why/how, the experiment, results, and how accurate VIO needs to be for different robot form factors.

Where the stage probes study one stage in isolation, `vio_pipeline` runs the
**whole pipeline as a stream**: a synthetic world with *exact* ground truth →
an additive-noise injector on the two sensor streams → the real MSCKF backend →
an estimate stream, measuring how well the filter cleans up the noise. The whole
VIO premise is additive noise; this shows — and graphs — the filter rejecting it.

```bash
# One command → scene.mp4 (+ curves): runs the experiment, overlay, and ffmpeg:
scripts/vio_scene_video.sh --out /tmp/vio_run --robot ground --sweep

# Synthetic source (exact GT — controlled noise studies):
./build/tools/vio_pipeline --out build/pipeline               # one run, matched noise
./build/tools/vio_pipeline --noise 3 --out build/pipeline     # 3× the assumed sensor noise
./build/tools/vio_pipeline --sweep --out build/pipeline       # noise level → robustness curve
./build/tools/vio_pipeline --robot drone                      # aggressive-motion preset
node docs-site/scripts/gen-sensor-model-figures.mjs build/pipeline docs/assessments/figures/pipeline

# EuRoC source (real images through the KLT front end; needs the dataset):
./build/tools/vio_pipeline --source euroc --dataset /path/to/V1_01_easy/mav0 --video --out build/euroc

# Overlay the metrics + live feature tracks on the scene video (either source):
./build/tools/vio_pipeline --video --out build/pipeline       # emits scene/ + frames.jsonl
node docs-site/scripts/gen-overlay.mjs build/pipeline          # → frames/*.svg
```

Artifacts: `run.jsonl` (the saveable typed per-frame stream — `gt`/`est` records),
`trajectory.csv`, `noise_sweep.csv`. Figures: estimated-vs-GT trajectory (top-down),
position error over time, **error vs noise level** (the robustness cliff), and
**NIS vs noise level** (consistency — NIS≈1 where injected noise matches the
filter's model, over-conservative below, over-confident above).

**Sources** (`--source`): `synthetic` (`sdk/eval/synthetic_world.hpp`, exact GT — the
controlled-noise studies) and `euroc` (the real EuRoC MAV dataset through the KLT
front end, with the dataset's mm ground truth; ATE is Horn-aligned).

**Scene-video overlay** (`--video` + `gen-overlay.mjs`): emits, per frame, the camera
image (real EuRoC frame, or a rendered synthetic scene) plus a `frames.jsonl` of the
live feature tracks and metrics; the generator composites each into a self-contained
SVG — the image, the tracked features (red), the true projections (gray, synthetic
only — so the additive camera noise reads as the red↔gray offset), and a HUD
(frame, time, #features, NIS, position error). Rasterize the SVGs (`rsvg-convert`) and
assemble with `ffmpeg` for an `.mp4`.

The reading: the filter holds up to ~2× its assumed noise, NIS crosses 1 right at
the matched level, then loses lock — a measured operating envelope.

## Real-data stage inspectors (epic #371)

The stage probes above are **synthetic** contract gates: known ground truth, run
in CI. Their **real-data** companions run the *actual* operator on a real EuRoC
sequence, dump what happened at each stage boundary, and render it — so you can
study one stage on real motion, blur, and texture. The flow is **trace-tapped**:
run the pipeline once with tracing on → a per-stage JSONL trace → each inspector
replays a stage and visualizes it.

| Piece | Role |
|---|---|
| `tools/include/branes/tools/vio_trace.hpp` | the **trace bus** — the JSONL schema (`{frame, t_s, stage, input, output}`) every inspector reads/writes |
| `tools/src/asl_trace.cpp` | the **`--trace` tap**: runs the real estimator over EuRoC and dumps per-stage records |
| `tools/src/s4_inspect.cpp` | **S4 visual-frontend inspector** (the template) — real frames → FAST+KLT → enriched per-track JSONL |
| `tools/src/s0_inspect.cpp` | **S0 sensor-model inspector** — real `cam0` frame → camera distortion grid + round-trip; real `imu0` stream → per-channel Allan / noise density |
| `tools/src/s5_inspect.cpp` | **S5 triangulation inspector** (3-D tier) — real tracks + GT clone poses → real triangulator → 3-D landmark cloud + covariance + reprojection residuals |
| `docs-site/scripts/gen-overlay.mjs` | renders an inspector's `frames.jsonl` to per-frame SVG overlays (image-domain tier) |
| `docs-site/src/components/scene3d.js` | the **3-D tier** — renders the landmark cloud + per-landmark covariance ellipsoids (`scene.json`) |
| `tools/src/s6_inspect.cpp` | **S6 MSCKF-update inspector** (filter-internal tier) — taps every real update (NIS, χ²-gate, residuals, covariance before/after) via the backend observer |
| `docs-site/scripts/gen-update-figures.mjs` | the **filter-internal tier** — NIS-over-updates + χ² band, residuals, covariance before/after heatmap |
| `tools/src/s1_inspect.cpp` | **S1 initialization inspector** (filter-internal tier) — captures the real bootstrap (state, initial covariance, gravity/scale/velocity vs GT) by polling `initialized()` |
| `docs-site/scripts/gen-init-figures.mjs` | recovered-state-vs-GT panel + initial-covariance heatmap (θ/p/v/bg/ba blocks) |
| `tools/src/s2_inspect.cpp` | **S2 propagation inspector** (filter-internal tier) — real IMU window → real propagator → covariance growth + propagated-vs-GT drift inside the ±σ envelope |
| `docs-site/scripts/gen-propagation-figures.mjs` | per-state σ-growth heatmap + drift-vs-3σ-envelope plot |
| `tools/src/s3_inspect.cpp` | **S3 augmentation inspector** (filter-internal tier) — real keyframe state → real `augment_clone` → before/after covariance with the new clone block |
| `docs-site/scripts/gen-augmentation-figures.mjs` | before/after covariance heatmap with the new clone block outlined |
| `tools/src/s9_inspect.cpp` | **S9 marginalization inspector** (filter-internal tier) — real keyframe state → real `marginalize_clone` → before/after covariance, kept-marginal unchanged + info dropped |
| `docs-site/scripts/gen-marginalization-figures.mjs` | before/after covariance heatmap with the dropped clone block outlined |
| `tools/src/s10_inspect.cpp` | **S10 online-calibration inspector** (filter-internal tier) — perturbed extrinsic + `estimate_extrinsics` → in-state extrinsic converging vs the dataset reference |
| `docs-site/scripts/gen-calibration-figures.mjs` | extrinsic rot/trans error-vs-time convergence inside the ±3σ band |
| `tools/src/obs_inspect.cpp` | **observability / null-space-leak diagnostic** (#212/#337) — taps every real update, drives the shipped `projection_jacobians` over the live clone window → per-update yaw vs translation gauge leak ‖H·N‖ |

EuRoC (~1.5 GB) is not vendored, so these are developer tools, **not CI gates**;
the trace schema and the inspector logic are gated by `tests/tools/vio_trace*.cpp`,
`tests/tools/s4_frontend_inspect.cpp`, `tests/tools/s0_sensor_model_inspect.cpp`,
`tests/tools/s5_triangulation_inspect.cpp`, `tests/tools/s6_update_inspect.cpp`,
`tests/tools/s1_init_inspect.cpp`, `tests/tools/s2_propagation_inspect.cpp`,
`tests/tools/s3_augmentation_inspect.cpp`, `tests/tools/s9_marginalization_inspect.cpp`,
`tests/tools/s10_calibration_inspect.cpp` and `tests/tools/observability_inspect.cpp`.

### Trace tap (`asl_trace`)

Runs the real `VioEstimator` + `MsckfBackend` over an EuRoC sequence and writes
one trace-bus record per stage boundary per frame — the real, ground-truthed
input/output every per-stage inspector replays from.

```bash
./build/tools/asl_trace --dataset /path/to/V1_01_easy/mav0 --out build/trace
#   --max-frames N   cap emitted records (the run still completes)
# → build/trace/s4_frontend.trace.jsonl   (one self-describing line per frame)
```

Each line is a shared header (`frame`, `t_s`, `stage`) plus a per-stage
`input`/`output` payload; an optional leading `meta` banner records the schema
version so a learner can read a trace without reading code. (Today S4 is the
worked boundary; filter-internal stages join as their operators are decoupled.)

### S4 frontend inspector (`s4_inspect`)

Runs the **shipped** frontend operator — `detect_fast` + pyramidal KLT, mirroring
`VioEstimator::track_frame` — over real EuRoC frames with full instrumentation,
exposing what production hides: per-track **forward-backward residual**, status,
and age; the FAST detections added each frame; the pyramid geometry; and a spatial
coverage grid. Unlike the production path it **always** computes the FB residual
(even when the gate is off), so you can *see* which tracks a gate would cull.

```bash
./build/tools/s4_inspect --dataset /path/to/V1_01_easy/mav0 --out build/s4
#   --fast-threshold F   --target-features N   --pyramid-levels N
#   --fb-max PX   (forward-backward GATE; 0 = measure but don't cull)   --min-dist PX
node docs-site/scripts/gen-overlay.mjs build/s4        # → build/s4/frames/*.svg
```

Output `frames.jsonl`, one record per frame:

| field | meaning |
|---|---|
| `frame`, `t`, `image`, `width`, `height` | frame index, timestamp (s), source PNG path, dimensions |
| `pyramid` | `{levels, sizes:[[w,h]…]}` — the KLT pyramid geometry |
| `tracks[]` | per track: `id`, `u`/`v` (this frame), `pu`/`pv` (previous frame), `fb` (forward-backward residual px; `-1` if not computed), `age`, `status` (`"new"` \| `"tracked"`) |
| `detections[]` | `[x, y]` of the FAST corners added this frame |
| `counts` | `{tracked, new, lost, fb_culled}` |
| `grid` | `{cols, rows, occupied}` — spatial coverage |
| `fb_max` | the FB gate threshold in force (0 = measure-only) |

The overlay (`gen-overlay.mjs`, frontend mode — auto-selected when a record has a
`tracks` array) draws, over the actual EuRoC image: KLT **flow vectors coloured by
FB residual**, FAST detections (green `+`), a **coverage grid** shaded by track
occupancy, a **pyramid schematic**, and a count HUD. Rasterize the SVGs
(`rsvg-convert`) and assemble with `ffmpeg` for an `.mp4`, as with the pipeline
overlay.

### S0 sensor-model inspector (`s0_inspect`)

Runs the two real S0 operators on a real EuRoC sequence: the **camera model** over
a pixel grid on an actual `cam0` frame, and the **IMU-noise characterizer** over
the `imu0` stream. The real-data companion to the synthetic S0 contract probe
(`s0_sensor_model`).

```bash
./build/tools/s0_inspect --dataset /path/to/V1_01_easy/mav0 --out build/s0 --frame 0
#   --grid-cols N   --grid-rows N   --allan-small N
node docs-site/scripts/gen-overlay.mjs build/s0   # → build/s0/frames/{distortion,imu_allan}.svg
```

Output `frames.jsonl`, two records:

| record | fields |
|---|---|
| `kind:"s0_camera"` | `image`, `width`/`height`, `model`, `intrinsics`, `grid:{cols,rows}`, `samples[]` (per node `u`/`v` observed pixel, `iu`/`iv` ideal pinhole pixel, `nx`/`ny` undistorted normalized, `dist` lens displacement px, `rt` round-trip px, `inc` incidence °), `max_distortion_px`, `max_roundtrip_px` |
| `kind:"s0_imu"` | `rate_hz`, `dt_s`, `n_samples`, `duration_s`, `channels[]` (per axis `name`, `unit`, `mean`, `std`, `N` white-noise density, `taus[]`, `allan[]`) |

The overlay (`gen-overlay.mjs`, S0 mode — auto-selected on `kind`) draws the
**distortion grid** — the rectilinear ideal grid (grey) vs the lens-warped grid
(green) with displacement vectors coloured by magnitude — over the actual frame
(`distortion.svg`), and the per-channel **Allan-deviation** log-log curve with each
channel's `N` (`imu_allan.svg`).

### S5 triangulation inspector (`s5_inspect`)

**Establishes the 3-D renderer tier.** Runs the shipped triangulator
(`CameraUpdater::triangulate`) on real EuRoC feature tracks (the shipped FAST + KLT
frontend) with **ground-truth clone poses**, turning each multi-view track into a
3-D landmark. Triangulation is decoupled behind the `trace::S5Input` /
`trace::S5Output` I/O struct (in `vio_trace.hpp`).

```bash
./build/tools/s5_inspect --dataset /path/to/V1_01_easy/mav0 --out build/s5 --min-obs 3
#   --px-noise PX   --fast-threshold F   --target-features N   --max-frames N
node docs-site/scripts/gen-overlay.mjs build/s5     # → build/s5/frames/*.svg (reprojection residuals)
```

It exposes what the update path hides — per landmark the **position covariance**
(`σ² · (Σ Hfᵀ Hf)⁻¹`, reusing the operator's own reprojection Jacobian), the
inter-view **parallax**, the system **condition number**, and per-observation
**reprojection residuals** — and writes three artifacts:

| artifact | role |
|---|---|
| `scene.json` | `{landmarks:[[x,y,z]…], landmark_cov:[[9 row-major]…], camera:{R_imu_cam, p_imu_cam, frustum_rays}}` — the 3-D landmark cloud + per-landmark covariance, drawn by `scene3d.js` (low-parallax features stretch along the line of sight) |
| `run.jsonl` | the camera path through the clone window (the viewer's frames) |
| `frames.jsonl` | per-frame reprojection residuals (`kind:"s5_residuals"`) — observed pixel vs the landmark's reprojection, drawn over the image by `gen-overlay.mjs` |

The 3-D landmark cloud + covariance ellipsoids render in the
[Scene3D](../docs-site/src/components/scene3d.js) viewer (load `scene.json` +
`run.jsonl`); the reprojection residuals render image-domain via `gen-overlay.mjs`.

### S6 MSCKF-update inspector (`s6_inspect`)

**Establishes the filter-internal renderer tier.** Runs the **real estimator**
(`VioEstimator` + `MsckfBackend`) over a real EuRoC sequence and taps **every**
MSCKF camera update through the backend's update observer
(`MsckfBackend::set_update_observer`, the #380 decouple — a no-op when unset, so
production is untouched). The local image of the global #212 over-confidence,
measured on real data.

```bash
./build/tools/s6_inspect --dataset /path/to/V1_01_easy/mav0 --out build/s6
#   --camera-noise σ   --calib-rot-deg D   --min-parallax DEG   --dump-update K   --max-frames N
node docs-site/scripts/gen-update-figures.mjs build/s6   # → nis.svg, residuals.svg, covariance.svg
```

Outputs:

| artifact | role |
|---|---|
| `updates.jsonl` | one record/update: `nis`, `dof`, `nis_over_dof`, `accepted`, `gated`, `chi2_threshold`, `innov_sum`, `cov_trace_before`/`after`, `psd_after`, `residual_rms_px`, per-obs `residuals` |
| `covariance.json` | one update's full `P` `before`/`after` (+ `imu_dim`/`clone_dim` block sizes) for the heatmap |

The figures (`gen-update-figures.mjs`, filter-internal tier) draw the
**NIS-over-updates** curve against its χ² consistency band (points coloured by the
gate decision), the **residual-RMS** series, and the covariance **before/after
heatmap** (log|P|, IMU/clone blocks marked). Sweeping `--camera-noise` /
`--calib-rot-deg` moves the NIS curve into the band — the real-data "R×4" analogue.

### S1 initialization inspector (`s1_inspect`)

Runs the **real estimator** over a real EuRoC sequence and watches
`backend().initialized()` flip false→true; at that moment it snapshots the seeded
state + 15×15 covariance + init diagnostics and compares the recovered bootstrap
against EuRoC ground truth. No SDK change — the init operators are already
decoupled and the backend exposes `initialized()`/`state()`/`init_diagnostics()`,
so polling per frame suffices.

```bash
./build/tools/s1_inspect --dataset /path/to/V1_01_easy/mav0 --out build/s1 [--prefer-dynamic]
node docs-site/scripts/gen-init-figures.mjs build/s1   # → init_state.svg, init_covariance.svg
```

Output `init.json`: the init `method`, recovered attitude/velocity/biases/`scale`,
the full initial covariance + per-block σ (with an `isotropic_seed` flag — the
S1.3 contract point that the seed is σ·I, not structured per observability), and
the GT comparison: yaw-invariant **gravity-direction error**, **speed error**,
gyro/accel-bias error, and (dynamic path) **scale error** vs the metric truth. The
figures draw the recovered-state-vs-GT panel and the initial-covariance heatmap
(θ/p/v/bg/ba blocks labelled).

### S2 propagation inspector (`s2_inspect`)

Takes a real EuRoC IMU window, seeds the prior pose+velocity from ground truth at
the window start (plus the initial-P seed), and runs the **real** IMU propagator
(`msckf::Propagator`) step by step. No SDK change — the propagator is already
standalone (`propagate(state,…)` → mean + `state.covariance()`).

```bash
./build/tools/s2_inspect --dataset /path/to/V1_01_easy/mav0 --out build/s2 [--start-t T] [--window-s S] [--sigma0 σ]
node docs-site/scripts/gen-propagation-figures.mjs build/s2   # → cov_growth.svg, pose_drift.svg
```

Output `propagation.json`: per step the per-block covariance σ (θ/p/v/bg/ba) and
the full 15-state diagonal, the propagated mean, and the drift vs GT with a
`pos_within_3sigma` flag. The figures draw the **σ-growth heatmap** (15 states ×
time, row-normalized) and the **drift-vs-±3σ-envelope** plot — where the
dead-reckoning drift escaping the envelope flags the propagated covariance
under-covering the error (the diagonal-Q / no-position-term #212 candidate the
synthetic S2 probe quantifies).

### S3 augmentation inspector (`s3_inspect`)

Runs the real estimator over EuRoC until the window holds a genuinely-correlated
covariance (initialized + a few clones), then takes a **copy** of that keyframe
state and runs the real `StateHelper::augment_clone` on it. No SDK change — the
operator is standalone and `backend().state()` is exposed.

```bash
./build/tools/s3_inspect --dataset /path/to/V1_01_easy/mav0 --out build/s3 --min-clones 5
node docs-site/scripts/gen-augmentation-figures.mjs build/s3   # → augment_covariance.svg
```

Output `augmentation.json`: the before/after covariance, the new clone block's
placement, and the stochastic-cloning residuals — the clone's marginal and its
cross-covariance with every existing state must equal the cloned pose's (a clone
is a deterministic copy), with a PSD check. The figure draws the before/after
heatmap with the **new clone block outlined**, making the clone literally appear.

### S9 marginalization inspector (`s9_inspect`)

The inverse of `s3_inspect`. Runs the real estimator over EuRoC until the window
holds several clones, then takes a **copy** of that keyframe state and runs the
real `StateHelper::marginalize_clone` to drop a clone (default the oldest, as the
backend does). No SDK change.

```bash
./build/tools/s9_inspect --dataset /path/to/V1_01_easy/mav0 --out build/s9 --min-clones 5 --idx 0
node docs-site/scripts/gen-marginalization-figures.mjs build/s9   # → marginalize_covariance.svg
```

Output `marginalization.json`: the before/after covariance, the dropped block's
placement, and the marginalization invariant — dropping a variable is exact
principal-submatrix extraction, so the **kept-state marginal must be unchanged**
(reported as a ~0 residual) — plus the information discarded (the dropped clone's
own σ and its cross-covariance with the kept states), with a PSD check. The figure
draws the before/after heatmap with the **dropped clone block outlined** in the
"before" matrix.

### S10 online-calibration inspector (`s10_inspect`)

Runs the real estimator over EuRoC with online extrinsic estimation on
(`estimate_extrinsics`), seeded from a deliberately **perturbed** camera↔IMU
extrinsic, and samples the in-state estimate converging toward the dataset's true
extrinsic each frame. No SDK change — online calibration is a shipped config and
`backend().state()` exposes the `CalibState` block.

```bash
./build/tools/s10_inspect --dataset /path/to/V1_01_easy/mav0 --out build/s10 --perturb-rot-deg 2 --perturb-trans-mm 30
node docs-site/scripts/gen-calibration-figures.mjs build/s10   # → calib_convergence.svg
```

Output `calibration.json`: the per-frame extrinsic-rotation error (deg) and
translation error (mm) against the reference, plus the estimate's σ. The figure
draws the error decaying toward the reference inside the filter's own (shrinking)
±3σ band. (On a physically-consistent EuRoC run the error converges, as the
`calib_recovery` unit test proves on consistent data.)

### Observability / null-space-leak diagnostic (`obs_inspect`)

The real-data companion to the synthetic `observability_probe` (#337). A monocular
VIO has a 4-DoF unobservable gauge — global translation (3) + rotation about
gravity, *yaw* (1) — and a consistent filter must never gain information along it:
the stacked camera Jacobian must annihilate the gauge, ‖H·N‖ ≈ 0. That holds only
at a single consistent linearization; a standard EKF re-linearizes each clone at
its own drifted estimate, so across a window the gauge leaks and the filter
fabricates information → over-confidence. This tool **measures the leak on real
data**: it taps every MSCKF update, drives the shipped `projection_jacobians` over
the live clone window + triangulated feature, and reports two leaks per update —
the *consistent* leak (H and N at the same estimate; the gauge-annihilation sanity
check, must be ~machine-ε) and the *real* leak (H perturbed by the filter's own
per-clone σ from P, N at the unperturbed gauge).

```bash
./build/tools/obs_inspect --dataset /path/to/V2_03_difficult/mav0 --out build/obs --max-frames 300
```

Output `observability.jsonl`: per-update window size, mean clone σ, and the
consistent/real/invariant yaw & translation leaks. On V2_03 the consistent leak is
~1e-15 (the shipped Jacobian is correct), the translation leak is **structurally 0**
(the ±Hf cancellation protects it), and the **standard yaw leak is nonzero on 100%
of updates and grows with the clone window's attitude drift** — localizing the #212
over-confidence to the yaw gauge on real data.

It also measures the **right-invariant (R-IEKF) leak on the same perturbed clone
windows** (driving the shipped `build_invariant_measurement`): because the
invariant gauge directions are estimate-independent constants (ρ = e_k, φ = ĝ), the
invariant yaw leak stays ~0 where the standard one rises — the real-data evidence
that R-IEKF is observable-by-construction and is the fix the diagnosis points to.

## Layout

| Piece | Role |
|---|---|
| `tools/src/vio_pipeline.cpp` | end-to-end noise→robustness demo (synthetic source → backend → metrics) |
| `sdk/include/branes/sdk/eval/synthetic_world.hpp` | synthetic VIO world with exact ground truth (the demo source) |
| `tools/include/branes/tools/stage_probe.hpp` | harness: contract printer, CLI, native-unit results table, CSV writer |
| `tools/include/branes/tools/vio_stage_contracts.hpp` | the machine-readable registry of all 11 stage contracts (single source of truth) |
| `tools/src/sN_*.cpp` | one thin driver per stage |
| `sdk/include/branes/sdk/eval/*_probe.hpp` | the reusable probe *computation* (header-only, type-generic) — shared with the regression tests |
| `docs-site/scripts/gen-sensor-model-figures.mjs` | renders the CSV artifacts to SVG figures |

The probe math lives in `sdk/eval/` headers so the same computation backs both a
utility (study) and a regression test (gate). The utility is just a `main()`
that prints the contract, runs the probes, and writes artifacts.

## Build & run

```bash
cmake -B build -DBUILD_TARGET_KPU=OFF      # tools are host-only
cmake --build build -j$(nproc)

# Print a stage's contract (pre/post-conditions + native-unit assessments):
./build/tools/s0_sensor_model --help

# Overview of the whole pipeline and which stages are wired:
./build/tools/s0_sensor_model --list

# Run a stage: print the native-unit assessment table + write CSV artifacts:
./build/tools/s0_sensor_model --out build/stage_probes/S0

# Render the artifacts to SVG figures for qualitative inspection:
node docs-site/scripts/gen-sensor-model-figures.mjs \
     build/stage_probes/S0 docs/assessments/figures/s0
```

Flags (all utilities): `--help` (print contract, don't run), `--list` (pipeline
overview), `--out DIR` (artifact directory), `--no-out` (compute & print only).

## Stages

| Util | Stage | Status | Native-unit assessments |
|---|---|---|---|
| `s0_sensor_model` | S0 sensor & calibration models | **wired** | px (round-trip, Jacobian), px/ms (time offset), px/deg, px/mm (extrinsics), mm (IMU drift) |
| `s1_initialization` | S1 init (static/dynamic) | **wired** | m/s² gravity-leveling residual, deg leveling error vs noise, % scale recovery, the scale-observability cliff, isotropic init-P σ |
| `s2_propagation` | S2 IMU propagation | **wired** | Q-structure (cortex diag vs canonical), pos-σ growth mm, GT-injection O(dt) mm, NEES vs Q-scale, R/PSD/nullspace |
| `s3_augmentation` | S3 stochastic cloning | scaffold | clone-vs-pose marginal & cross-cov error, PSD |
| `s4_frontend` | S4 visual frontend | scaffold | px FB residual, inlier %, grid coverage, track-length |
| `s5_triangulation` | S5 triangulation | scaffold | px reproj, parallax deg, cond#, depth σ vs parallax |
| `s6_msckf_update` | S6 MSCKF update | scaffold | H_f rank, Jacobian check, null-space orthonormality, NIS vs χ², whiteness, FEJ divergence |
| `s7_slam_features` | S7 in-state landmarks | scaffold | feature-init cross-cov, landmark NEES |
| `s8_zero_velocity` | S8 ZUPT | scaffold | detector ROC, static drift with/without |
| `s9_marginalization` | S9 marginalization | scaffold | kept-marginal invariance, PSD |
| `s10_online_calibration` | S10 online calibration | **wired** | calibration noise budget → R-inflation (1°≈R×4), pose NEES vs R-inflation, the empirical R-deficit |

**S1 reading:** static init levels gravity to machine zero and the stationarity gate rejects an
over-noisy window; dynamic init recovers the injected metric scale (<1% under excitation) and the
scale-observability **gate declines** (rather than guessing) once `resolved_motion` drops below the
0.05 m floor — the dynamic-init cliff. The filter's initial covariance is an **isotropic σ·I**
(σ=0.1): *not* enlarged on the unobservable yaw/scale or the weakly-observable accel bias.

**S2 reading (#212):** the probe drives the real `Propagator` and finds the diagonal `Q_d` drops
only the canonical position-block & v–p terms (~7% position-σ inter-frame), while **propagation-only
NEES ≈ 6 at the default `Q`** (6-DoF target) — propagation is approximately consistent, so the
over-confidence is downstream (S6 update or S10 calibration), not in S2.

"Scaffold" = the contract is defined and prints, but the probe is not yet wired.
To wire one: add the assessment computation to a `sdk/eval/*_probe.hpp` header
(type-generic, returns plain structs), then drive it from `tools/src/sN_*.cpp`
exactly as `s0_sensor_model.cpp` drives `sdk/eval/sensor_model_probe.hpp`. Update
the stage's `status` to `"implemented"` in `vio_stage_contracts.hpp`.

## Why this shape

A VIO filter is a chain of transformations where one violated invariant three
stages upstream surfaces as "drift" or "over-confidence" with no local symptom.
Studying each stage in isolation — with its contract printed and its assessments
in units you can reason about (pixels, degrees, millimetres) — is how you localize
the fault instead of guessing. S0 already demonstrates the payoff: the EuRoC
camera model round-trips and linearizes to machine precision (contract holds — so
the over-confidence is *not* a projection bug), while the time-offset and
extrinsic sensitivities quantify the uncertainty the filter omits by treating
calibration as perfect — the S10 candidate for issue #212.
