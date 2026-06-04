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

## Layout

| Piece | Role |
|---|---|
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
| `s10_online_calibration` | S10 online calibration | scaffold | px/ms & px/deg/mm (from S0), R-inflation equivalent, calib NEES |

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
