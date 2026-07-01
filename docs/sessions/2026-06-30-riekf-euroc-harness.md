# Session — 2026-06-30 — End-to-end R-IEKF EuRoC harness (measured, and it underperforms)

> Session arc: took the **unit-proven** R-IEKF observability fix (the #212 yaw-leak
> cure, validated at the Jacobian level by `obs_inspect` in the prior diagnostic
> work — yaw leak 0.39 → 2.3e-16) and built the **end-to-end EuRoC harness** to test
> whether it actually yields a more consistent filter on real data — driven through
> the **same front end** as the standard MSCKF so the two are directly comparable.
> The answer, measured: R-IEKF now runs **stably** but **underperforms** the
> body-frame MSCKF on both ATE and position-σ — and, more usefully, the position-norm
> metric turns out **not to test the thesis** at all.
> PR landed: **1** (#429), on `main` post-v0.65.0 (unreleased).
> Pair: Theo Omtzigt (Ravenwater) + Claude Opus 4.8.

The through-line, as always: **measure, don't assume.** A plausible "R-IEKF fixes the
over-confidence" headline did not survive contact with the numbers — and the reason
why is the real finding.

---

## What was accomplished

### The harness (#429)

Drove `InvariantVioBackend` (R-IEKF) through the validated `VioEstimator` front end /
track manager on real EuRoC, so ATE and reported position-σ are directly comparable
with the standard `MsckfBackend`:

- **`tools/include/branes/tools/invariant_backend_adapter.hpp`** — a `VioBackend`-
  conforming wrapper bridging the invariant backend's parameterization-honest API
  (`set_nav` / `process_imu(gyro,accel,t)` / `process_camera`-normalized / `nav`) to
  the `VioEstimator` contract. Bootstraps from a static, gravity-aligned IMU window
  (the same `ImuInitializer` the standard backend uses), unprojects pixels → bearings,
  and maps SE₂(3) back to `NavState`. Lives in `tools/`, not `sdk/` — a validation
  harness, not a shipped path.
- **`vio_pipeline --invariant`** (`run_euroc_invariant`) — mirrors the standard EuRoC
  run, reporting ATE (Horn) + mean position σ + the over-confidence ratio.
- **Shared `euroc_cam0()` calibration** — both backends read byte-identical
  intrinsics/extrinsics, so the comparison can never drift on calibration.
- **Gate test** (`tests/tools/invariant_backend_adapter.cpp`) — pins the bootstrap
  boundary (uninitialized at `kInitSamples-1`, up as the window fills) and the
  VioEstimator contract bridges.

### Two real-data bugs the harness surfaced

- **Square-root covariance diverges on real data.** The adapter first used
  `SqrtCovariance` (copied from the synthetic `run_synthetic_invariant` path); on
  V1_01 it blew up to **ATE 6.24e+40 m**, σ 3.04e+56. Switching to the Joseph
  **`FullCovariance`** form — the invariant backend's *default*, and the same
  representation the standard MSCKF uses on real data — made it track. The algorithm
  was never the problem; the covariance representation was.
- **Non-finite update on a degenerate measurement.** A barely-conditioned real-data
  update could emit a non-finite δx and abort in `SO3::normalize`. Added a guard in
  the shipped `msckf_invariant_backend`: snapshot the covariance, and on any
  non-finite δx restore it and reject the track (it never happened). δx is the
  sentinel — a poisoned covariance resurfaces as a non-finite δx on the next update.

### The result — R-IEKF vs standard MSCKF (same binary, same front end)

| Seq | Backend | ATE (Horn) | pos σ | ATE/σ | NIS |
|---|---|---|---|---|---|
| V1_01 | MSCKF | **0.29 m** | 0.136 m | 2.13 | 1.51 |
| V1_01 | R-IEKF | 1.31 m | 0.076 m | 17.3 | — |
| V2_03 | MSCKF | **0.27 m** | 0.126 m | 2.13 | 14.7 |
| V2_03 | R-IEKF | 1.36 m | 0.070 m | 19.3 | — |

R-IEKF runs **stably** end to end (a real improvement over the prior ~160 m
continuous-update divergence, #366), but it is **worse on both axes**: ~5× the ATE and
a *tighter* σ — i.e. *more* over-confident, the opposite of the #347/#348 thesis.

## Conventions / discoveries this session

- **The position-norm ATE/σ ratio does not test the yaw-gauge thesis.** The standard
  MSCKF sits at **ATE/σ = 2.13 on both sequences** — already ~calibrated on the
  position norm, because the #212 leak is *rotational*. A position metric is dominated
  by translation, where the body-frame filter is fine. The Jacobian-level result (yaw
  leak → 2e-16) remains the only evidence that actually measures the fix. The
  end-to-end discriminator has to be a **gauge-anchored attitude-NEES (yaw vs
  roll/pitch split)**, not ATE/σ.
- **Don't inherit the synthetic path's `Cov` choice.** `SqrtCovariance` is numerically
  finicky and diverges on real EuRoC; Joseph `FullCovariance` is the robust default.
- **Generic finiteness must be ADL, not `static_cast<double>`.** The δx guard uses
  `using std::isfinite;` + an unqualified call so a custom `math::Scalar` (Universal
  posit) supplies its own — per the no-hardcoded-double rule.
- **CodeRabbit is incremental and pauses on drafts** — a manual `@coderabbitai review`
  won't re-review already-seen commits while draft; marking the PR *ready* resumes the
  authoritative automatic review. Its first automated review leaves a stale
  `reviewDecision: CHANGES_REQUESTED` that does not block merge (`mergeStateStatus:
  CLEAN`).

## State at end of day

- **#429 merged** to `main`; branch cleaned up, `main` clean, no open PRs.
- R-IEKF harness available behind `--invariant`; **default remains the body-frame
  MSCKF**.
- R-IEKF end-to-end: stable but underperforming; yaw-specific consistency still
  **unmeasured**.

## Open / next

- **#365 — the attitude-NEES metric (option B).** Add a gauge-anchored attitude-NEES
  (yaw vs roll/pitch split) to `run_euroc_invariant` and the standard path, plus
  seed/tuning parity, so the comparison measures the rotational consistency R-IEKF
  targets — where it should win. This is the metric that can actually see (or refute)
  the gauge benefit; the position ATE/σ can't. Closes the remaining #365 acceptance
  criterion.
- Only after that does "flip the default backend" become a decision the numbers can
  support.
