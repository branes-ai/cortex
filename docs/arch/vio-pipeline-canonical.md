# The Canonical VIO Pipeline — Distilled for Contract-Driven Reconstruction

> **Purpose.** We have a clean-room OpenVINS-style MSCKF that runs but is structurally
> over-confident (NEES ≫ 1) and we have not been able to find the defect by trial and error
> (FEJ was implemented and *refuted* by measurement — see `docs/assessments/vio-troubleshooting.md`).
> This document goes back to first principles: it distills the **canonical** VIO pipeline as
> designed by OpenVINS (and contrasts MINS / VINS-Mono / ROVIO / ORB-SLAM3), decomposed into
> discrete **stages**. Each stage is written as a *contract*: a type signature, the
> **pre-conditions** that must hold on its inputs, the **post-conditions / invariants** that must
> hold on its outputs, the **probes** that measure whether those conditions hold, and the
> **unit tests + sensitivity analyses** that validate them.
>
> The method this enables: for every stage, build the probe, write the test, confirm the
> contract holds *in isolation* before composing. A VIO filter is a chain of transformations
> where one violated invariant three stages upstream surfaces as "drift" or "over-confidence"
> with no local symptom. Contracts localize the fault.
>
> **Companion docs:** `docs/arch/visual-inertial-odometry.md` (the deep offline math treatment —
> sensor models, preintegration covariance, marginalization+FEJ, the structured solve, the
> frontend). This document is the *operational* counterpart: what the canonical reference
> systems actually do, stage by stage, and how to instrument each stage.

---

## 0. How to read this document

A VIO estimator is a **typed dataflow graph**. Each operator has a signature, and the edges
carry typed quantities (poses on `SE(3)`, rotations on `SO(3)`, covariances that must stay PSD,
residuals that must stay white). The bugs that kill VIO are almost never "the math is wrong in
the abstract" — they are **contract violations at a boundary**: a covariance that lost its
cross-correlations, a Jacobian evaluated at the wrong linearization point, a clone augmented
with zero cross-covariance, a noise term scaled by `Δt` where it should be `1/Δt`, a calibration
treated as perfectly known when it isn't.

The master post-conditions of the *whole* filter are three, and everything below exists to make
them true:

| Master invariant | What it means | Probe |
|---|---|---|
| **NEES ≈ dim(state)** | the filter's reported covariance matches its actual error magnitude (consistency) | `eval/nav_consistency.hpp` against GT |
| **NIS ≈ dim(measurement)** | each innovation is sized as the filter predicted | `eval/consistency.hpp` |
| **Innovations are white** | no information left uncaptured frame-to-frame | `eval/innovation_whiteness.hpp` |

NEES ≫ dim ⇒ **over-confident** (our symptom). NEES ≪ dim ⇒ over-conservative. The job of every
stage contract below is to be a *local* check that, if all pass, forces these three globals true.

The canonical pipeline is laid out in dataflow order. For each stage:

- **Signature** — input types → output types.
- **What it does** — the canonical operation (OpenVINS).
- **Pre** — what must hold on the inputs (else the stage is meaningless or unstable).
- **Post / invariants** — what must hold on the outputs (the contract this stage guarantees).
- **Probes** — the measurement devices to instrument it.
- **Tests + sensitivity** — unit tests and the sensitivity sweep that confirms the contract is
  robust, not coincidental.
- **cortex** — the file that implements it, and **candidate divergences** from canonical worth checking.

---

## 1. The canonical OpenVINS pipeline at a glance

```
            IMU (200–1000 Hz)                    Camera (20–30 Hz)
                  │                                     │
                  ▼                                     ▼
        ┌───────────────────┐                 ┌───────────────────┐
        │ S2 PROPAGATE      │                 │ S4 FRONTEND       │
        │ mean + cov (Φ,Qd) │                 │ KLT track + RANSAC│
        │ FEJ on Φ          │                 └─────────┬─────────┘
        └─────────┬─────────┘                           │ tracks
                  │ predicted state + P                 ▼
                  ▼                           ┌───────────────────┐
        ┌───────────────────┐  (new image)    │ S5 TRIANGULATE    │
        │ S3 AUGMENT/CLONE  │<────────────────│ lost/marg features│
        │ P' = G P Gᵀ       │                 └─────────┬─────────┘
        └─────────┬─────────┘                           │ p_f, observations
                  │                                     ▼
                  │                            ┌───────────────────┐
                  │                            │ S6 MSCKF UPDATE   │
                  │                            │ H_f,H_x → null-sp │
                  │                            │ → QR compress     │
                  │                            │ → χ² gate         │
                  │                            │ → EKF (Joseph)    │
                  │                            │ FEJ on H_x        │
                  │                            └─────────┬─────────┘
                  ▼                                      │ corrected state + P
        ┌───────────────────┐                            │
        │ S9 MARGINALIZE    │<───────────────────────────┘
        │ drop oldest clone │
        └───────────────────┘
   (S1 INIT bootstraps; S7 SLAM-feat, S8 zero-velocity, S10 online-calib are optional layers)
```

The state vector OpenVINS actually carries (this is the first place clean-rooms under-build):

```
x = [ x_IMU(15) | clone_1(6) … clone_n(6) | SLAM_feat_1 … (3 or 1 each) | CALIB ]
                                                                          └─ camera↔IMU extrinsics (6/cam)
                                                                             camera intrinsics+distortion (8/cam)
                                                                             camera↔IMU time offset (1/cam)
                                                                             IMU intrinsics (optional)
```

`x_IMU = [θ_GI (3) | p_GI (3) | v_G (3) | b_g (3) | b_a (3)]`, error-state, **left** quaternion
multiplicative error in OpenVINS (`δq ≈ [½θ; 1]`). The cortex implementation uses a world-centric
right-`SO(3)` perturbation (`R ← R·Exp(δθ)`) — a legitimate alternative convention, but the
choice must be **globally consistent** across propagation, augmentation, update, and the
measurement Jacobians. A mixed convention is a classic silent inconsistency.

> **First load-bearing observation (cortex vs canonical):** the cortex state is
> `[IMU(15) | clones(6n)]` only — **no camera intrinsics, extrinsics, or time-offset states**.
> OpenVINS either estimates these online *or* at minimum its measurement model and noise account
> for their uncertainty. Treating calibration as perfectly known removes real uncertainty from
> the filter — which is structurally indistinguishable from over-confidence, and is *exactly*
> consistent with the troubleshooting finding that inflating `R` (×4) restores NEES on MH_05.
> Inflating `R` is a crude proxy for the unmodeled calibration/extrinsic/time-offset uncertainty.
> This is a **candidate contract to test** (see S0, S10), not a declared fix.

---

## S0 — Sensor & calibration models (the substrate every stage trusts)

**Signature.**
`camera: project(p_c ∈ ℝ³, ζ) → (u,v) ∈ ℝ²` and inverse; `imu: (ω̃, ã) = model(ω, a, R, g, b, n)`;
`extrinsics: T_CI ∈ SE(3)`, `time offset: t_d ∈ ℝ`.

**What it does.** Defines the measurement functions every residual is built from. OpenVINS
composes the camera measurement as `z = h_d(h_p(h_t(h_r(λ, …), R_CG, p_C)), ζ) + n` — four stacked
maps: feature representation → world-to-camera transform (uses extrinsics + clone pose) →
perspective projection → distortion (radtan 8-param or fisheye/KB 4-param). The IMU model is
`ω̃ = ω + b_g + n_g`, `ã = Rᵀ(a − g) + b_a + n_a`.

**Pre.**
- `ζ` valid: `f_x, f_y > 0`; distortion coefficients within the model's convergence basin.
- Gravity **sign and magnitude pinned** and frame-consistent. A static IMU must read `+‖g‖`
  along its up axis (it senses the reaction to gravity), *not* zero.
- `T_CI` and `t_d` known to some accuracy with a **stated uncertainty** (this number must feed
  the filter — see candidate divergence).

**Post / invariants.**
- **Round-trip:** `unproject(project(p_c)) ≈ p_c / ‖p_c‖` within ε for all `z > 0`. (Distortion
  models that don't round-trip are a frontend-poisoning bug.)
- **Jacobian sanity:** `∂h/∂p_c` finite and rank-2 for `z > 0`; matches numerical (finite-diff)
  Jacobian within ε. (This single check catches a large fraction of homegrown projection bugs.)
- **Cheirality:** features behind the camera (`z ≤ 0`) are rejected, never projected.
- **IMU static identity:** integrating a stationary IMU through the init orientation yields
  zero world-frame acceleration.

**Probes.** Projection/Jacobian round-trip residual; analytic-vs-numeric Jacobian max abs error;
static-IMU world-accel magnitude; gravity-magnitude check at init.

**Tests + sensitivity.**
- Unit: random `p_c` round-trip; analytic Jacobian == finite-diff Jacobian (the workhorse test —
  every measurement Jacobian in the system gets one).
- Sensitivity: **time-offset sweep** — inject `t_d ∈ {−10, −5, 0, +5, +10} ms` and measure pose
  error; quantifies how many mm/° of error per ms of unmodeled offset (this is the cost of *not*
  having `t_d` in the state, directly relevant to over-confidence).
- Sensitivity: **extrinsic perturbation** — perturb `T_CI` by `{0.5°, 1°, 2°}` and `{1, 5} mm`,
  measure the induced reprojection bias; this is the bias the filter absorbs into the state as
  if it were noise.

**cortex.** Camera model + `jacobian_at_pose` in `msckf/camera_updater.hpp:236`. IMU model in
`msckf/propagator.hpp`. **Candidate divergence:** no `t_d`, no online `T_CI`, no intrinsics in
state — their uncertainty is unmodeled. The two sensitivity sweeps above quantify the resulting
covariance deficit.

---

## S1 — Initialization (bootstrap the manifold)

**Signature.** `init(imu_window, [vision_window]) → (R_GI, v_G, b_g, b_a, g, [scale])` + initial `P`.

**What it does.** OpenVINS supports **static** and **dynamic** init.
- *Static:* detect a stationary window, average accel → gravity direction (fixes roll/pitch,
  yaw arbitrary), average gyro → `b_g`, `v = 0`. Magnitude checked against `‖g‖`.
- *Dynamic:* a visual-inertial alignment over a short window solving for gravity, velocity, and
  scale by matching SfM motion to preintegrated IMU (Dong-Si/Mourikis-style linear system).

**Pre.**
- Static: accel std and gyro std below stationarity thresholds over the window.
- Dynamic: **sufficient acceleration excitation** — scale and accel-bias are weakly observable
  under gentle or pure-rotation motion (the top real-world init failure). A scale-observability
  gate must guard this.

**Post / invariants.**
- `R_GI` aligns measured-accel-mean to gravity: residual gravity after alignment ≈ 0; roll/pitch
  observable, yaw set to a gauge value.
- `‖g‖` within tolerance of local gravity (≈ 9.78–9.83).
- Dynamic: `scale s > 0` and finite; **condition number of the alignment linear system bounded**
  (an ill-conditioned system ⇒ scale not observable ⇒ reject, don't return garbage).
- **Initial `P` honestly reflects what was *not* observed** — large variance on yaw, on accel
  bias, on scale. Over-tight initial `P` is over-confidence injected at t=0 that the filter never
  recovers from.

**Probes.** Post-alignment gravity residual; recovered `‖g‖`; alignment-system condition number /
smallest singular value; recovered scale vs GT scale; initial-`P` eigenvalues vs the unobservable
directions.

**Tests + sensitivity.**
- Unit: synthetic stationary IMU → recovered `R_GI` tilts measured gravity onto `−z` within ε;
  recovered `b_g` == injected.
- Unit: dynamic init on synthetic excited trajectory recovers injected scale within ε.
- Sensitivity: **excitation sweep** — scale the trajectory's acceleration by `{0.1, 0.3, 1, 3}×`
  and plot recovered-scale error vs excitation; locate the observability cliff. This tells you
  where init *should* refuse to fire (and whether yours refuses or returns a confident wrong answer).

**cortex.** `imu_init.hpp` (`try_static`, `try_gravity_align`, `try_dynamic`); SfM bootstrap in
`sfm/two_view.hpp`, `sfm/pnp.hpp`, `sfm/init_window.hpp`. Per memory, dynamic init won't fire on
real EuRoC (SfM robustness, #247/#211). **Candidate divergence:** verify initial `P` carries large
yaw / accel-bias / scale variance; an over-tight init `P` is a first-frame over-confidence source.

---

## S2 — IMU propagation (mean + covariance)

**Signature.** `propagate(x⁻, P⁻, imu[t_k, t_{k+1}]) → (x⁻_{k+1}, P⁻_{k+1})`.

**What it does — mean.** OpenVINS uses a **zeroth-order hold** (measurement constant over the
sample) or RK4 for the mean; debias, then strapdown-integrate `R, v, p`.

**What it does — covariance.** `P⁻_{k+1} = Φ P⁺_k Φᵀ + Q_d`, with the canonical block structure
(rows/cols `[θ, p, v, b_g, b_a]`):

```
Φ = ⎡  R_{k+1,k}              0     0    −R_{k+1,k} J_r(θ) Δt        0        ⎤
    ⎢ −½ R_kᵀ ⌊â Δt²⌋          I   Δt I          0            −½ R_kᵀ Δt²     ⎥
    ⎢ −R_kᵀ ⌊â Δt⌋             0     I            0             −R_kᵀ Δt       ⎥
    ⎢   0                      0     0            I                 0         ⎥
    ⎣   0                      0     0            0                 I         ⎦
```

The decisive feature is the **off-diagonal skew blocks** coupling orientation error into velocity
and position error — this is gravity-leakage living inside the covariance.

The discrete noise (OpenVINS simplified form): `Q_d` is built by mapping the **measurement-space**
noise `Q_meas = diag(σ_gc²/Δt · I, σ_ac²/Δt · I)` through the noise Jacobian `G` (which carries the
`J_r Δt`, `R Δt`, `½R Δt²` factors — the *same* structure as Φ's last columns), plus a bias-walk
block `Q_bias = diag(σ_wgc² Δt · I, σ_wac² Δt · I)`. The two noise families scale in **opposite
directions** in `Δt`: white noise `÷Δt` (in measurement space) but maps to `×Δt` in angle space;
bias walk `×Δt`. Getting this backwards silently mis-weights every inertial step.

**FEJ.** The consistency fix: evaluate `Φ` (the Jacobian) at the **first-estimate** linearization
point (frozen `R_fej, b_fej`), while the *mean* integrates at the current estimate. This must be
done consistently with the measurement Jacobian (S6) — half-FEJ (one side only) is *worse* than
none, as the troubleshooting confirmed.

**Pre.** `Δt > 0`, monotonic timestamps; measurements debiased with the current bias estimate;
`P⁺_k` symmetric PSD.

**Post / invariants.**
- `R_{k+1} ∈ SO(3)`: orthonormal, `det = 1` (re-normalize / use `Exp`).
- `P⁻_{k+1}` **symmetric PSD** (eigenvalues ≥ 0).
- `Q_d` PSD and carries the **θ–v–p cross-correlations** (NOT diagonal).
- **Observability:** the 4-D unobservable subspace (global position + yaw about gravity) lies in
  the null space of the linearized propagation; with FEJ, `Φ` does not inject information along
  yaw. Probe by propagating the analytic nullspace vectors and checking they stay in the null space.
- **GT-injection identity:** with ground-truth IMU and zero noise, propagation reproduces the GT
  trajectory within integration tolerance over the interval.

**Probes.** `P` min-eigenvalue (PSD monitor); `R` orthonormality residual `‖RᵀR − I‖`; `Q_d`
symmetry/PSD; nullspace-drift metric (`‖projection of P⁻¹ onto yaw direction‖`); per-step
propagation error vs GT (GT-injection harness, #266).

**Tests + sensitivity.**
- Unit: zero-noise GT-injection — propagate against an analytic trajectory, assert mean error
  and that `P` grows monotonically along observable directions.
- Unit: analytic `Φ` == finite-diff Jacobian of the mean-propagation map.
- Unit: **Q_d structure** — assert `Q_d` has nonzero `θ–v` and `θ–p` blocks (catches the
  diagonal-Q bug); assert the `Δt`-scaling direction by halving `Δt` and checking white-noise
  block scales `×½` while bias-walk block scales `×½` in the *opposite* role.
- Sensitivity: **noise-density sweep** — scale `σ_gc, σ_ac` by `{0.5, 1, 2, 5}×` and measure
  NEES of propagation-only segments; confirms NEES ∝ inverse noise (the lever the sweeps already found).

**cortex.** `msckf/propagator.hpp` (Φ assembly `:62`, Q `:81`, FEJ `:54`). **Candidate divergence
(high priority):** the map reports `Q` assembled as a **12-term diagonal `σ²·Δt`** added directly
to the state covariance — i.e. **without** mapping through the noise Jacobian `G`. If so, `Q_d` is
missing the `θ–v–p` cross-correlations *and* the higher-order `Δt²/Δt³` velocity/position scalings
that the canonical `G Q_meas Gᵀ` produces. A diagonal `Q_d` understates position/velocity process
noise and omits the correlations the update relies on — a direct over-confidence mechanism. The
"Q_d structure" unit test above is designed to catch exactly this. **Verify the Φ velocity-row
sign and the `J_r` term too** (the `R_{k+1,k} J_r Δt` block in the `θ`–`b_g` coupling is frequently
dropped in clean-rooms).

---

## S3 — State augmentation / cloning

**Signature.** `augment(x, P) → (x', P')` adding a clone of the current camera pose.

**What it does.** Stochastic cloning: append the current IMU pose (transformed by extrinsics to
the camera pose, or the IMU pose with `T_CI` applied at measurement time) as a new state, with
`P' = G P Gᵀ` where `G = [I; J]` and `J` maps the existing IMU pose error into the new clone's
error. The new clone's pose block is a **copy** of the current pose block.

**Pre.** clone count < window max; current pose valid; `P` PSD.

**Post / invariants — the contract clean-rooms most often break.**
- `P'` symmetric PSD.
- Immediately after cloning, the clone's **marginal covariance equals the cloned pose's marginal**,
  *and* the **cross-covariance between the clone and every other state equals that of the cloned
  pose**. The clone is a deterministic copy — it is **perfectly correlated** with its source, not
  independent. Augmenting with a zero or block-diagonal cross-covariance ("just add a new block")
  is a textbook inconsistency that makes the filter *under*-confident at clone time and corrupts
  every subsequent update that spans clones.
- `G` carries the right convention: if the clone is the camera pose, `J` must include `∂(camera
  pose)/∂(IMU pose)` through `T_CI` (and through `t_d` if modeled).

**Probes.** Block-equality assertion `‖P'[clone,clone] − P[pose,pose]‖` and
`‖P'[clone,*] − P[pose,*]‖` (must be ≈ 0 at augmentation); `P'` min-eigenvalue.

**Tests + sensitivity.**
- Unit: after augment, assert the two block-equality invariants to machine precision.
- Unit: augment then immediately marginalize the clone ⇒ `P` returns to the original (round-trip).
- Sensitivity: if extrinsics enter `G`, perturb `T_CI` and confirm the cross-covariance changes as
  predicted (guards the extrinsic Jacobian in `G`).

**cortex.** `msckf/state_helper.hpp:31` (`augment_clone`, `G = [I; J]`, `cov.transform(G)`). The
map shows `J` mapping onto IMU `δθ, δp` — **verify whether `T_CI` is included** (if the clone is
meant to be the *camera* pose) or whether clones are IMU poses with extrinsics applied later in
the measurement Jacobian. Either is valid; mixing them is not.

---

## S4 — Visual frontend (track generation)

**Signature.** `track(image_t, prev_tracks) → observations[(id, cam, u, v)]`.

**What it does.** FAST/Shi-Tomasi detection with grid-bucketed spatial coverage and min-distance
suppression; pyramidal KLT (or descriptor matching) frame-to-frame; **forward-backward** check;
RANSAC on the epipolar/fundamental constraint. The IMU gyro prior can seed KLT search and reduce
RANSAC to 2-point.

**Pre.** image in expected format; previous tracks available; (optionally) gyro rotation prior.

**Post / invariants.**
- Every surviving track passes forward-backward (round-trip pixel error < threshold).
- RANSAC inlier ratio above a floor; outliers removed *before* the backend sees them (no backend
  trick repairs a confidently-wrong correspondence).
- **Spatial coverage:** features distributed across the image grid (cluster → poorly-conditioned
  rotational DoF).
- Track-length distribution healthy (not all length-2 — that starves parallax).

**Probes.** Per-frame: track count, inlier ratio, grid-occupancy histogram, forward-backward
residual distribution, track-length histogram, parallax distribution.

**Tests + sensitivity.**
- Unit: synthetic translating/rotating pattern — recovered tracks match the warp within ε.
- Unit: inject known outliers — RANSAC + FB rejects them.
- Sensitivity: **pixel-noise injection** — add `σ ∈ {0.1, 0.5, 1, 2} px` to features and measure
  track survival and downstream reprojection residual; calibrates the *true* measurement noise
  the backend should use (vs the assumed pixel σ).

**cortex.** Frontend in `vio_estimator.hpp:162` (`FrontendParams`, KLT in `tests/cv/klt_tracker.cpp`).
**Candidate divergence:** confirm forward-backward + RANSAC are both active and that the pixel σ
fed to S6 matches the *measured* track noise from the sensitivity sweep (not an optimistic constant).

---

## S5 — Feature triangulation

**Signature.** `triangulate(observations, clone_poses, T_CI) → (p_f ∈ ℝ³, status)`.

**What it does.** Linear (ray-perpendicular / DLT) triangulation across the observing clones,
then Gauss-Newton refinement (inverse-depth parameterization is best-conditioned for far points).
OpenVINS triangulates at the **current** clone poses (best geometry).

**Pre.** ≥ 2 observations from **distinct, sufficiently-parallax** clones; cheirality holds.
Under pure rotation parallax → 0 and nothing triangulates — the frontend must flag these, not
feed them in.

**Post / invariants.**
- Reprojection residual at the solution below threshold (else the track is an outlier that
  survived RANSAC, or the geometry is degenerate).
- Depth **positive and finite**; cheirality holds in every observing view.
- **Condition number** of the triangulation normal matrix bounded — low parallax ⇒ ill-conditioned
  ⇒ huge depth uncertainty ⇒ the feature must be deferred or down-weighted, not used as if precise.

**Probes.** Per-feature: parallax angle (max over view pairs), reprojection RMS at solution,
triangulation condition number / depth standard deviation, cheirality pass rate.

**Tests + sensitivity.**
- Unit: synthetic feature seen from known poses → recovered `p_f` within ε; reprojection ≈ 0.
- Unit: degenerate (pure-rotation) input ⇒ triangulation **refuses** (status = degenerate), not
  a confident wrong depth.
- Sensitivity: **parallax sweep** — vary baseline so parallax spans `{0.1°, 0.5°, 2°, 10°}` and
  plot depth error + depth uncertainty vs parallax; locate the threshold below which features
  should be excluded. A feature triangulated at 0.2° parallax and then used at full weight is a
  covariance-corruption source.

**cortex.** `msckf/camera_updater.hpp:272` (linear triangulation `:275`, GN refine `:301`).
**Candidate divergence:** verify there is a **parallax / condition-number gate** before a
triangulated feature is admitted to the update. Admitting low-parallax features at full weight
injects optimistic information — a plausible contributor to the aggressive-motion (V2_03)
divergence where parallax is erratic.

---

## S6 — MSCKF update (the consistency-critical stage)

**Signature.** `update(x⁻, P⁻, {feature tracks}) → (x⁺, P⁺)`.

This is one stage but five sub-contracts. The order is: build `(H_f, H_x, r)` → null-space
project out the feature → QR-compress `H_x` → χ²-gate → EKF update.

### S6a — Feature & state Jacobians, residual

`r = z − ẑ`, `r ≈ H_x δx + H_f δf + n`. `H_x` is nonzero only on the **observing clone** columns
(and on extrinsics/intrinsics if calibrated). **FEJ:** `H_x` and `H_f` are evaluated at the
**first-estimate** clone poses (`R_fej, p_fej`); the residual `r` uses the **current** poses.

- **Pre:** triangulated `p_f` valid (S5); ≥ 2 observations.
- **Post:** `H_f` has full column rank 3 (else the feature direction is unobservable and
  null-space projection is rank-deficient); analytic `H_x, H_f` == finite-diff within ε; `H_x`
  sparsity correct (zero off the observing clones).
- **Probe:** `rank(H_f)`; analytic-vs-numeric Jacobian error; `H_x` nonzero-pattern check;
  **FEJ-divergence** `‖R_fej − R‖, ‖p_fej − p‖` per clone (the troubleshooting already built this).

### S6b — Left null-space projection (marginalize the feature)

Choose `N` spanning the left null space of `H_f` (`Nᵀ H_f = 0`); left-multiply:
`Nᵀ r = Nᵀ H_x δx + Nᵀ n`. Feature eliminated algebraically.

- **Pre:** `m ≥ 2` observations ⇒ `2m ≥ 4 > 3 = dim(feature)` so the null space is non-trivial.
- **Post:** projected residual has dimension `2m − 3`; **`N` is orthonormal** (`NᵀN = I`) so the
  projected noise stays `σ² I` (this is *why* you can keep the scalar noise — break orthonormality
  and the noise model is silently wrong).
- **Probe:** `‖Nᵀ H_f‖` (should be ≈ 0); `‖NᵀN − I‖` (orthonormality); output row count `== 2m−3`.
- **Test:** synthetic `H_f` → assert `Nᵀ H_f ≈ 0` and `NᵀN = I`; assert dimension.

### S6c — Measurement (QR/Givens) compression

Thin-QR the tall stacked `H_x = Q₁ R₁`; keep `R₁` (state-sized) and `Q₁ᵀ r`. Purely for speed
(reduces the EKF matrix multiply). **Noise stays isotropic** because `Q₁ᵀ` is orthogonal.

- **Post:** `Q₁ᵀ` orthogonal ⇒ noise still `σ² I`; the compressed system has the same information
  as the uncompressed (innovation and gain identical up to the rotation).
- **Probe:** assert update result is identical with/without compression on a test case (it must be
  — compression is exact, not an approximation).
- **Note (cortex):** the map did **not** find a second QR compression stage. This is an
  *efficiency* feature, not a correctness one — its absence does **not** cause over-confidence.
  Flagged only for completeness; do not chase it for #212.

### S6d — Chi-square (Mahalanobis) gating

`γ = rᵀ (H P⁻ Hᵀ + R)⁻¹ r`; accept if `γ < χ²_{thresh}(dof)`. Rejects outliers and
linearization-failure measurements.

- **Pre:** innovation covariance `S = H P⁻ Hᵀ + R` well-conditioned (Cholesky succeeds).
- **Post:** accepted innovations satisfy `γ ~ χ²(dof)` — i.e. **NIS is distributed as χ² with the
  right dof**. NIS systematically ≫ dof ⇒ `S` too small ⇒ `R` and/or `P` under-stated (the
  over-confidence signature, measured already: NIS ∝ 1/R²).
- **Probe:** NIS distribution vs χ²(dof) bounds (`eval/consistency.hpp`); gate rejection rate;
  `cond(S)`.
- **Test + sensitivity:** with GT-consistent noise, NIS time-average ≈ dof. **Gate-threshold
  sweep** — vary the χ² multiplier and watch ATE vs rejection rate; a too-tight gate that rejects
  good aggressive-motion measurements is a V2_03-divergence suspect (the filter stops correcting
  exactly when motion is hardest).

### S6e — EKF update (apply correction)

`K = P⁻ Hᵀ S⁻¹`; `δx = K r`; `x⁺ = x⁻ ⊞ δx`; covariance update **Joseph form**
`P⁺ = (I−KH) P⁻ (I−KH)ᵀ + K R Kᵀ`.

- **Post:** `P⁺` symmetric PSD (Joseph guarantees this even with numeric error); `x⁺` on-manifold
  (`R ⊞ δθ` via `Exp`, never additive on rotation); FEJ linearization points (`R_fej`) **not
  touched** by the update.
- **Probe:** `P⁺` min-eigenvalue; `‖RᵀR − I‖` post-update; trace(`P`) trend (should drop on
  update, grow on propagation — a monotone-decreasing trace across *both* is over-confidence).
- **Test:** Joseph form keeps `P` PSD under a stress sequence; `x ⊞ δx` retraction correctness.

**The master post-conditions of S6 as a whole:** post-update **NEES ≈ dim(state)**, **NIS ≈ dof**,
**innovations white**. These are the three globals from §0 — S6 is where they are won or lost.

**cortex.** `msckf/camera_updater.hpp:130` (update), null-space in
`features/msckf_nullspace.hpp:42`, EKF + Joseph in `msckf/covariance.hpp:106`, gating `:143`. FEJ
toggle per-track `:164`. The structure is faithful; the defect (if in S6) is most likely in the
**noise scaling / what `R` and `P⁻` actually contain entering `S`** — which points back to S2's
`Q_d` and S0/S10's unmodeled calibration, not to the null-space/Joseph algebra (which the map
shows are textbook-correct).

---

## S7 — SLAM-feature update (optional, persistent landmarks)

**Signature.** features kept **in-state** (3 or 1 params each, anchored inverse-depth), updated by
reprojection without marginalization; long-lived landmarks bound drift better than pure-MSCKF.

**Post / invariants.** in-state features get `P` blocks that must stay PSD and correlated with the
poses that see them; feature initialization into the state must use **stochastic init** (correct
cross-covariance), the same contract as S3. Pruning/marginalizing a SLAM feature = Schur
complement, info-preserving.

**cortex.** `features/representations.hpp` defines the parameterizations; not the current focus —
the running filter is pure-MSCKF. Listed for pipeline completeness.

---

## S8 — Zero-velocity update (optional, robustness)

**Signature.** detect stationarity (IMU + visual disparity) → apply a `v = 0` pseudo-measurement
(and suppress clone insertion while still).

**Post / invariants.** prevents spurious motion / feature-starved drift while static; the detector
must not false-positive during slow motion (would inject a wrong `v=0` constraint → bias).

**cortex.** not present in the map; a known canonical robustness layer. Relevant to deployment,
not to the EuRoC over-confidence directly.

---

## S9 — Marginalization / clone management

**Signature.** `marginalize(x, P, clone_idx) → (x', P')`.

**What it does.** Slide the window: drop the oldest clone (and features anchored to it that aren't
promoted). In covariance form this is **principal-submatrix extraction** of the kept states; in
information form it's a Schur complement that produces fill-in.

**Pre.** clone selected by policy (oldest, or VINS-Mono two-way: keyframe vs non-keyframe).

**Post / invariants.**
- `P'` symmetric PSD; the kept-state marginal is **unchanged** by removing a clone (marginalizing
  a variable does not alter the marginal of the others — a correctness check on the extraction).
- The marginalization (in a smoother, the prior) must **not constrain the 4 gauge directions**.
- For a filter doing principal-submatrix extraction, no FEJ issue arises here (unlike a smoother's
  marginalization prior); but the *augment* (S3) cross-covariance must have been correct or the
  extracted marginal is already wrong.

**Probes.** `P'` PSD; kept-marginal-invariance `‖P'[keep] − P[keep,keep]‖` (must be 0 for pure
extraction); window length bound.

**cortex.** `msckf/state_helper.hpp:51` (`marginalize_clone`, principal submatrix via
`cov.marginalize(keep)`). The round-trip test in S3 covers augment∘marginalize.

---

## S10 — Online calibration (extrinsics, intrinsics, time offset)

**Signature.** calibration parameters as **state**: `T_CI` (6/cam), intrinsics+distortion (8/cam),
`t_d` (1/cam), optional IMU intrinsics.

**What it does.** OpenVINS estimates these online; their Jacobians enter `H_x` (S6a). Even when
*not* estimated, the filter must **account for their uncertainty** (inflate `R`, or carry a fixed
prior block) — otherwise it treats imperfect calibration as perfect and is over-confident.

**Post / invariants.** if estimated: calibration states observable only under sufficient
excitation (like accel bias); if fixed: the assumed-perfect calibration's true uncertainty must
appear somewhere in the noise budget.

**Probes.** time-offset and extrinsic sensitivity sweeps (S0); if estimated, calibration-state
NEES vs GT calibration.

**cortex — candidate divergence (high priority, ties to #212).** No calibration states and (per
the map) no calibration-uncertainty term in the measurement noise. The EuRoC `T_CI` is taken as
truth. The troubleshooting found honest noise needs `R ×4` to fix NEES on tracked sequences —
**`R ×4` is numerically what unmodeled extrinsic + time-offset + intrinsic uncertainty would
require.** Test directly: add a fixed extrinsic/time-offset uncertainty block (or the equivalent
`R` inflation derived from the S0 sensitivity sweeps) and re-measure NEES/NIS/ATE. If NEES drops
toward dim without the blunt global `R ×4`, the over-confidence is unmodeled-calibration, not a
filter-algebra bug.

---

## 2. Cross-system contrast — what each reference changes per stage

| Stage | **OpenVINS** (filter / MSCKF) | **MINS** (OpenVINS++) | **VINS-Mono/Fusion** (smoother) | **ROVIO** (direct filter) | **ORB-SLAM3** (smoother) |
|---|---|---|---|---|---|
| Backend | EKF, MSCKF null-space + optional in-state SLAM feats | same EKF core, multi-sensor | sliding-window NLS (Ceres) | iterated EKF, **photometric** | local BA + factor graph (g2o/GTSAM-like) |
| Frontend | KLT or descriptor | KLT | KLT | **direct** patches (no features) | ORB descriptors |
| Feature handling | null-space marginalize | same | reprojection factors, inverse-depth | patch intensity in EKF state | map points, covisibility |
| IMU | preintegration not needed (propagate each step) | same | **preintegration** factors | propagate each step | preintegration |
| Consistency | **FEJ** | FEJ + consistent interpolation | marginalization prior + FEJ | iterated EKF | marginalization |
| Online calib | extrinsics, intrinsics, **time offset**, IMU intrinsics | **full spatiotemporal, all sensors** | extrinsics + time offset | extrinsics | extrinsics |
| Loop closure | none (odometry) | optional pose-graph | 4-DoF pose graph + DBoW2 | none | **full** (DBoW2 + global BA) |
| Init | static + dynamic | proprioceptive-only dynamic | SfM + VI alignment | filter converges from rough | IMU + SfM, robust |
| Distinguishing idea | clean, well-documented MSCKF; the reference filter | async multi-sensor via high-order on-manifold interpolation + dynamic cloning | preintegration + marginalization smoother | photometric, no frontend | full SLAM, relocalization |

**Reading for cortex:** the architecture in the repo is squarely the **OpenVINS column** —
correct choice for the compute-constrained KPU target, and the right reference to match
stage-for-stage. The smoother columns (VINS/ORB) differ in backend but share S0/S1/S4/S5 contracts
and the *same* observability/consistency invariants — so the contracts above transfer if the
backend is ever swapped.

---

## 3. MINS — what it adds on top of the OpenVINS core (for later phases)

MINS (Lee, Geneva, Chen, Huang — *JFR 2025*; `github.com/rpng/MINS`, `arXiv:2309.15390`) keeps the
**OpenVINS EKF/MSCKF core** and generalizes it to robust asynchronous multi-sensor fusion. The
ideas worth importing *as contracts*, even for a camera-only system:

1. **High-order on-manifold interpolation for asynchronous measurements.** Instead of requiring a
   clone exactly at each measurement time, MINS interpolates poses on `SE(3)` between clones to the
   measurement timestamp. *Contract relevance:* this is the principled version of handling the
   camera↔IMU **time offset** — the same `t_d` whose unmodeled uncertainty we flagged in S0/S10.
2. **Dynamic cloning** — adaptively choosing interpolation order / clone density from motion-induced
   information. *Contract relevance:* a measurement-driven version of keyframe selection; ties to
   the parallax/conditioning gate in S5.
3. **Full online spatiotemporal calibration for every sensor** (extrinsics + intrinsics + time
   offset, per sensor). *Contract relevance:* directly the S10 gap — MINS treats calibration
   uncertainty as a first-class estimated quantity rather than assuming it away.
4. **Proprioceptive-only initialization** and explicit robustness/consistency (FEJ throughout).
5. **Optional loop closure** via a separate pose-graph layer (turns odometry → SLAM).

For our immediate problem, the MINS lesson is **#3**: a consistent system models calibration
uncertainty rather than trusting datasheet/dataset extrinsics — reinforcing the S10 candidate.

---

## 4. Consolidated contract / probe matrix

The build order: implement the probe, write the test, confirm the contract in isolation, then
compose. Probes that already exist in `sdk/include/branes/sdk/eval/` are marked ✓.

| Stage | Key invariant (post-condition) | Probe | Exists? |
|---|---|---|---|
| S0 model | proj round-trip; analytic==numeric Jacobian | jacobian-diff harness | new |
| S0 calib | pose error per ms `t_d`, per ° extrinsic | sensitivity sweep | new |
| S1 init | gravity residual≈0; ‖g‖ ok; init `P` honest on yaw/scale/b_a | init-P eigen check | partial |
| S2 mean | `R∈SO(3)`; GT-injection error bounded | orthonormality; GT harness (#266) | partial |
| S2 cov | `P` PSD; `Q_d` has θ–v–p cross terms; nullspace preserved | PSD monitor; **Q-structure test** | new |
| S3 augment | clone marginal==pose marginal; cross-cov==pose cross-cov | block-equality assert | ✓ (tested, residual 0) |
| S4 frontend | FB-consistent; coverage; inlier ratio | track stats; pixel-noise sweep | partial |
| S5 triangulate | reproj<thresh; depth>0; parallax-gated | parallax / cond-number probe | ✓ (tested; no soft gate on admit path) |
| S6a Jacobian | `rank(H_f)=3`; analytic==numeric; FEJ divergence | jacobian-diff; FEJ-divergence log | ✓ (FEJ log) |
| S6b nullspace | `NᵀH_f≈0`; `NᵀN=I`; rows=2m−3 | orthonormality assert | ✓ (tested, residual ~2e-16) |
| S6c compress | exact (result == uncompressed) | equivalence test | n/a (absent) |
| S6d gate | **NIS ~ χ²(dof)** | `eval/consistency.hpp` | ✓ |
| S6e update | `P⁺` PSD (Joseph); on-manifold; FEJ frozen | PSD monitor | ✓ |
| **filter** | **NEES≈dim, NIS≈dof, innovations white** | `eval/nav_consistency.hpp`, `innovation_whiteness.hpp` | ✓ |
| S9 marg | `P'` PSD; kept-marginal invariant | extraction-invariance assert | ✓ (tested, residual 0) |
| S10 calib | calibration uncertainty represented | extrinsic/`t_d` sensitivity; calib-NEES | new |

---

## 5. Where the running cortex filter most likely diverges from canonical

The map shows the *algebra* is textbook-correct (null-space projection, Joseph update,
marginalization-by-submatrix, FEJ storage all present and right). FEJ was implemented and refuted
by measurement. So the over-confidence is **not** in the parts that look hard in the papers. The
ranked candidates, each a contract above with a test that decides it:

1. **S10 — unmodeled calibration uncertainty — MEASURED, a quantified contributor.** The
   `s10_online_calibration` probe (run 2026-06-05) confirms the analytic link: a realistic **1°
   extrinsic uncertainty induces ~8 px of reprojection error — larger than the filter's assumed
   ~4.6 px — for an R-inflation of ~4×, quantitatively matching the empirical `R×4`** from #212.
   The end-to-end R-sweep reproduces the EuRoC over-confidence almost exactly (synthetic pose
   **NEES ≈ 43 at R×1 ≈ MH_05's 43**), and R-inflation drives NEES back toward dof. Honest nuance:
   R-*only* needs ~×33 for full consistency (the empirical fix paired `R×4` with `Q×10`), so
   calibration cleanly accounts for the **`R×4` component** but is one contributor — the
   over-confidence is multi-source (calibration + process noise + the S5 parallax gate). *Fix:*
   model calibration uncertainty (estimate `T_CI`/`t_d` online, OpenVINS/MINS-style, or a
   calibration term in `R`) rather than a blunt global `R`-scale.
2. **S2 — diagonal `Q_d` — MEASURED and largely DE-PRIORITIZED.** The `s2_propagation` probe
   (run 2026-06-04) confirms the cortex `Q_d` is diagonal and drops the canonical position-block
   (`¼σ_a²Δt³`) and v–p cross (`½σ_a²Δt²`) terms — but quantifies the cost as **~7% position-σ
   under-report inter-frame, 0.7% at 0.5 s** (Φ-reconstruction validated to 1e-16). Decisively,
   **propagation-only NEES ≈ 6.4 at the cortex default `Q` (6-DoF target 6)** with a clean
   `NEES ∝ 1/Q` — so the IMU process noise is approximately *consistent on its own*. The
   over-confidence is therefore **not** an S2/`Q` fault; it must enter at the update (S6) or via
   unmodeled calibration (S10). Fix the diagonal `Q` for correctness, but it is not the driver.
3. **S5 — no parallax / conditioning gate on triangulated features — gate REFUTED as the V2_03 fix
   (measured 2026-06-11).** The shipped triangulator admits low-parallax features at full weight
   (S5 probe: depth σ up to ~127% of depth, no soft gate). The decisive test was run end-to-end:
   `min_parallax_deg` was plumbed through `VioConfig` + a `CORTEX_MIN_PARALLAX_DEG` knob and swept on
   EuRoC **V2_03**. Enabling the gate **monotonically regressed both accuracy and consistency** —
   ATE 0.27 m → 0.89 m (2°) → 1.99 m (5°); NIS 14.7 → 17.5 → 26.7; NEES 140 → 170 → 322 — because the
   gate starves the fast-motion filter of the short-baseline features it depends on (a 5° gate drops
   >half the updates: 21 585 → 10 180). So the low-parallax admission is a genuine contract gap but
   **not** the V2_03 over-confidence driver; rejecting those features costs more than it saves. Keep
   the gate **default-off**. The over-confidence is dominated by the other input-side source,
   **S10 unmodeled calibration** (candidate #1).
4. **S6d — gate threshold under aggressive motion.** A too-tight χ² gate rejects good measurements
   exactly when motion is hard, starving corrections → divergence. *Decisive test:* gate-threshold
   sweep on V2_03.

These four are independent and individually testable. The discipline the user proposed —
probe + test + sensitivity per stage — is exactly what separates them: instead of one more global
re-tune, each candidate has a *local* measurement that confirms or kills it. Per the standing
guidance (`cortex-212-overconfidence-structural`), do not edit the estimator for #212 until the
over-confidence is traced to one of these named contract violations with a matching instrument reading.

---

## 6. Running the contracts: the stage-probe utilities

The contracts above are executable. Each stage S0–S10 has a runnable study utility under
[`tools/`](../../tools/README.md) — *not* a test and *not* a daemon, but a tool you run to focus
on one stage in isolation, perturb an input, and watch a problem-native metric move:

```bash
cmake -B build -DBUILD_TARGET_KPU=OFF && cmake --build build -j$(nproc)
./build/tools/s0_sensor_model --help      # print this stage's contract
./build/tools/s0_sensor_model --list      # the whole S0–S10 pipeline
./build/tools/s0_sensor_model --out build/stage_probes/S0   # run + write artifacts
node docs-site/scripts/gen-sensor-model-figures.mjs build/stage_probes/S0 docs/assessments/figures/s0
```

The architecture: the probe *computation* lives in header-only, type-generic
`sdk/include/branes/sdk/eval/*_probe.hpp` (shared by the utility and the regression test in
`tests/sdk/sensor_model_probe.cpp`); the contracts are a single machine-readable registry in
`tools/include/branes/tools/vio_stage_contracts.hpp`; each `tools/src/sN_*.cpp` is a thin driver
that prints the contract, runs the probes, prints a native-unit results table, and writes the CSV
artifacts the figure generator renders.

**S0 is wired** and reports in native units — px (round-trip, Jacobian linearization), px/ms (time
offset), px/deg & px/mm (extrinsics), m/s² & mm (IMU drift). Its reading: the EuRoC camera model
and analytic Jacobian round-trip and linearize to **machine precision** (the S0 *camera* contract
holds — the over-confidence is not a projection bug), while the time-offset (0.12→0.87 px/ms) and
extrinsic (≈8 px/deg, 0.21 px/mm @2 m) sensitivities quantify the uncertainty the filter omits by
treating calibration as perfect — the **S10 candidate**. The IMU drift cases make gravity leakage
visceral: a 0.5° attitude error is **4.3 m** of position drift in 10 s. Figures:
`docs/assessments/figures/s0/`.

**S2 is wired** (`s2_propagation`, probe `sdk/eval/propagation_probe.hpp`, test
`tests/sdk/propagation_probe.cpp`). It drives the real `Propagator` and reports: Q-structure
(cortex diagonal vs canonical `B·Σ·Bᵀ`), position-σ growth cortex-vs-canonical (mm), GT-injection
discretization error (clean **O(dt)**, ~15 mm @200 Hz over a 2 s tumble), and **propagation-only
NEES vs Q-scale** (the #212 lever). Headline reading: **NEES ≈ 6 at the default `Q`** ⇒ propagation
is approximately consistent and the diagonal-`Q` deficit is ~7% inter-frame — the over-confidence is
**downstream of S2**. Figures: `docs/assessments/figures/s2/`.

**S1 is wired** (`s1_initialization`, probe `sdk/eval/initialization_probe.hpp`, test
`tests/sdk/initialization_probe.cpp`). It drives the real `ImuInitializer` (+ `ImuPreintegrator` for
the dynamic keyframes) and reports: static gravity-leveling residual (machine zero) + leveling error
vs accel noise (the stationarity gate rejects an over-noisy window); dynamic metric-scale recovery
(<1% under excitation) and the **scale-observability cliff** — the gate *declines* once
`resolved_motion` falls below the 0.05 m floor; and the initial covariance, which is an **isotropic
σ·I** (σ=0.1) — *not* enlarged on the unobservable yaw/scale or the weakly-observable accel bias, a
structural observation worth noting for the S10 calibration work. Figures: `docs/assessments/figures/s1/`.

**S10 is wired** (`s10_online_calibration`, probe `sdk/eval/calibration_probe.hpp`, test
`tests/sdk/calibration_probe.cpp`). It measures the leading #212 candidate: (1) the analytic
calibration noise budget — **1° extrinsic uncertainty ⇒ ~8 px induced error ⇒ R-inflation ~4×**,
matching the empirical `R×4`; (2) an end-to-end R-sweep that reproduces the EuRoC over-confidence
(synthetic **NEES ≈ 43 ≈ MH_05**) and shows R is the consistency lever. Honest finding: calibration
cleanly accounts for the **`R×4` component**, but full restoration needs more (the empirical fix also
raised `Q`), so #212 is multi-source — calibration + process noise + the S5 parallax gate. Figures:
`docs/assessments/figures/s10/`.

**S3, S5, and S9 are also wired** (drivers `s3_augmentation`, `s5_triangulation`,
`s9_marginalization`; probe headers `clone_window_probe.hpp`, `triangulation_probe.hpp`) and now
carry regression tests (`tests/sdk/clone_window_probe.cpp`, `tests/sdk/triangulation_probe.cpp`)
that lock their readings as CI gates. Their measured verdicts:

- **S3 augment — eliminated.** Clone marginal, cross-covariance, and augment∘marginalize round-trip
  residuals are all **identically zero**: `P' = G P Gᵀ` with a copy-selection `G` is exact algebra,
  so the clone carries the full correlation (no zero-cross-covariance inconsistency). Not a #212 source.
- **S9 marginalize — eliminated.** Kept-marginal residual **identically zero**; `cov.marginalize` is
  exact principal-submatrix extraction, `P'` stays PSD. Not a #212 source.
- **S5 triangulate — confirmed optimistic-information injection.** Clean-geometry depth is correct at
  every parallax, but the only guard is the *hard* Cholesky breakdown — there is **no soft parallax /
  conditioning gate on the admit path**. At 0.1–1° parallax the depth σ under 1 px noise is 90–127% of
  the depth itself (cond up to 1.3e6), yet `triangulate()` returns success and the feature reaches the
  EKF update at full weight. The 5%-of-depth uncertainty budget is met only at **~5°**. The
  `min_parallax_deg` gate exists but is **default-off** — the leading S5 candidate for the V2_03 divergence.

**S6 is now wired** (`s6_msckf_update`, probe `sdk/eval/update_probe.hpp`, test
`tests/sdk/update_probe.cpp`) — the consistency-critical stage, and its verdict closes the
localization loop:

- **S6 update — algebra measured-CONSISTENT.** Driven in isolation on a self-consistent `(P, R)`
  scene (clone poses perturbed by a draw from the filter's own `P`, image noise at exactly the
  assumed σ), the shipped `CameraUpdater::update` returns **NIS/dof = 1.002** (χ² band [0.98, 1.02],
  4000 updates), with the left-null-space projection marginalizing the feature (`‖NᵀH_f‖ = 2.8e-16`)
  via orthonormal reflectors (`‖NᵀN − I‖ = 2.2e-16`, so projected noise stays σ²I) at the correct
  dimension `2m−3`, and Joseph keeping `P⁺` PSD on every update. A noise-mismatch sweep confirms the
  lever: NIS/dof = 0.84 → **1.00** → 1.64 → 4.17 as injected noise outgrows the modeled `R`. **The
  #212 over-confidence is therefore not in the update math — it enters via the INPUTS `P⁻`/`R`**
  (the S2 process noise and the S10 unmodeled calibration), exactly as §5 hypothesized.
  *Not yet instrumented:* the analytic-vs-numeric Jacobian check and FEJ-divergence telemetry (the
  updater currently linearizes at the live poses, not a frozen first estimate).

**The remaining scaffolds are S4, S7, S8.** With S0/S2/S3/S5/S6/S9/S10 measured, the over-confidence
is conclusively localized to the **noise budget entering the update** (`P⁻`/`R`), with the EKF
algebra itself measured-consistent (S6) and the S5 parallax-gate fix **refuted end-to-end** on V2_03.
The leading driver is **S10 unmodeled calibration** (the quantified `R×4` component) alongside the
process-noise budget — the fix is to *model those inputs* (online `T_CI`/`t_d` or a calibration term
in `R`), not to gate features or rewrite the update. Next decisive measurement: the `CORTEX_R_SCALE`
/ `CORTEX_Q_SCALE` sweep on V2_03/MH_05 to quantify how much of the residual NEES the honest
calibration+process noise budget recovers.

### End-to-end noise→robustness demo

Beside the per-stage probes, `tools/src/vio_pipeline.cpp` runs the **whole pipeline as a stream**:
a synthetic world with exact ground truth (`sdk/eval/synthetic_world.hpp`) → an additive-noise
injector on the camera + IMU streams → the real MSCKF backend → an estimate stream, measuring how
well the filter rejects the noise. It writes a saveable typed stream (`run.jsonl`) plus CSVs the
figure generator renders to: estimated-vs-GT trajectory, position error over time, **error vs
noise level**, and **NIS vs noise level**. The measured envelope on the synthetic source: the
filter tracks (sub-metre on gentle motion) and is over-conservative when sensors are clean
(NIS ≪ 1), **NIS crosses 1 exactly where the injected noise matches the filter's assumed model**
(scale = 1), becomes over-confident above it, and **loses lock past ~2–4×** — a quantified operating
window. Building a synthetic world with exact GT also surfaced a backend behaviour worth a probe:
features spanning the static→motion boundary triangulate degenerately (no parallax gate, the S5
candidate), and unestimated accel bias dead-reckons into metres — both consistent with the #212 leads.

---

## Sources

- OpenVINS documentation — propagation, discrete propagation, feature update, measurement
  compression, system overview: <https://docs.openvins.com/>
- Mourikis & Roumeliotis, *A Multi-State Constraint Kalman Filter for Vision-aided Inertial
  Navigation*, ICRA 2007 (MSCKF).
- Geneva, Eckenhoff, Lee, Yang, Huang, *OpenVINS: A Research Platform for Visual-Inertial
  Estimation*, ICRA 2020.
- Huang, Mourikis, Roumeliotis — observability-constrained EKF / First-Estimate Jacobians.
- Forster, Carlone, Dellaert, Scaramuzza, *On-Manifold Preintegration*, TRO 2017.
- Lee, Geneva, Chen, Huang, *MINS: Efficient and Robust Multisensor-aided Inertial Navigation
  System*, JFR 2025 — <https://arxiv.org/abs/2309.15390>, <https://github.com/rpng/MINS>.
- Companion: `docs/arch/visual-inertial-odometry.md`; `docs/assessments/vio-troubleshooting.md`.

> **Licensing note.** OpenVINS and MINS are GPL. This distillation is derived from their
> *documentation and the underlying papers* (algorithmic/architectural level), not from their
> source — consistent with the repo's clean-room-from-papers rule.
