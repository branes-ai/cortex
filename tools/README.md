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
