# VIO diagnostic methodology — why we steered by ATE, and the instruments we should build

A post-mortem of the dynamic visual-inertial initialization debugging effort
(epic #211 / #247) and the engineering-process correction it motivates. The
short claim: we debugged an estimator by editing across the whole pipeline and
reading the end-to-end ATE, iterating ~10 times. That is a methodological error,
not just an inelegant one, and this document records *why*, the diagnostic
tooling that makes VIO debugging principled, the concrete gaps in this codebase,
and the `characterize → assess → hypothesize → model → test` workflow we should
follow instead. Companion to
[`scripts/vio-euroc-numerical-failure-modes.md`](../../scripts/vio-euroc-numerical-failure-modes.md)
(the F1–F6 failure-mode taxonomy) and
[`scripts/vio-euroc-architecture.md`](../../scripts/vio-euroc-architecture.md)
(the pipeline + frames).

---

## A. Why the process degenerated

The only instrument we had wired up was **at the exhaust pipe** —
`eval/trajectory_metrics.hpp` ATE/RPE. Every component (`ImuPreintegrator`,
`ImuInitializer::try_dynamic`, `init_window`, the `Propagator`, the MSCKF camera
update) was observable *only* through its effect on the final aligned
trajectory. With no rig that feeds a component a known input and checks a known
output, we could never localize a fault — only observe that the whole stack was
unhappy. Three properties of estimators make that fatal:

1. **The failure output is lossy about its cause.** A wrong sign, a wrong scale,
   a residual gravity tilt, a swapped extrinsic, an over/under-confident
   covariance — *all* surface as "innovations blow up, trajectory diverges." The
   map cause→ATE is many-to-one; you cannot invert it from the symptom.
2. **ATE has the gauge freedoms subtracted out first.** It is a scalar *after*
   Horn rigid-SE3 alignment — i.e. after deliberately projecting out the 3
   rotation + 3 translation (and effectively scale) DoF that a VIO/init bug
   actually lives in. We steered by a number with the relevant dimensions
   removed.
3. **The loop is long and the signal is ~1 bit.** A multi-minute replay
   returning "km = bad / sub-meter = good." Ten iterations buys ~10 bits at
   enormous cost — the signature of an undersized, non-localizing instrument.
   Because the metric is many-to-one and error-masking, the search can land on a
   *compensating second bug* and call it green.

### The gravity-sign bug is the precise case study

The gravity-free sign convention shared between `ImuPreintegrator` and
`ImuInitializer::solve_alignment` was an **undefined interface contract** —
documented only in a header comment, with the preintegration deltas crossing the
`InitFrame` boundary as bare, untagged `Vec3` dv/dp (no enum, no sign tag, no
assertion). `solve_alignment` encoded the opposite gravity sign; fed real
gravity-free preintegration it recovered `g = −g_true` — the seed's "up" pointed
~178° down, injecting ~2g of phantom acceleration and diverging MH_05 to ~48 km.

And the synthetic `init_window`/`imu_init` fixtures **built their dv/dp with the
same flipped sign**. The wrong-sign solve and wrong-sign oracle cancelled: a
textbook **compensating error**. Every unit test passed while the physical
convention was inverted, because *the oracle and the code-under-test shared the
bug* — the tests validated self-consistency, not correctness against an
independent definition. That is exactly why a sign/scale/tilt bug survives green
CI and only manifests on real data.

So the degeneration was **structural**, not a discipline lapse: (1) no
per-component ground-truth harness, (2) interface contracts as prose not
assertions, (3) synthetic tests re-encoding the code's own conventions, and (4)
a low-bandwidth, gauge-blinded scalar as the only steering signal.

---

## B. The tools that localize a failure *class*

Each observes the data streams or estimator internals, not the trajectory. Each
of our bugs was catchable with an off-the-shelf tool or a sub-20-line check **at
the component boundary**.

| Failure class | Diagnostic | Would have caught |
|---|---|---|
| **F1** frame / extrinsics | **Kalibr** for true `T_BS`; one-line assert loaded `T_BS` ≠ identity = `sensor.yaml`; Jacobian shows a negated/rotated block | identity-extrinsics 21 km — *before a frame runs* |
| **F2** unobservable / rank | **excitation detector** (windowed accel-−g / gyro std); **observability-Gramian nullspace** — VIO ideal nullity is exactly **4** (3 pos + 1 yaw); threshold the singular-value gap (Hesch/Huang OC-VINS) | excitation-limited 19 m drift; the enabler of scale collapse |
| **F4** ill-conditioning | **cond-number / σ_min monitor** on the init/two-view Hessian (MTL5 exposes it; ~5 lines via SVD); the smallest singular vector names the bad DoF | scale collapse to 1e-8 — at the near-singular solve, not via divergence |
| **F3/F4** degenerate two-view | **parallax gate** (median > ~1° / 20 px) + **E singular-value ratio** + **cheirality** (standard in ORB-SLAM / VINS-Mono) | low-parallax adjacent-frame bootstrap — a ~10-line precondition |
| **F5** sign / convention | **analytic-vs-numerical Jacobian check** (Ceres `GradientChecker` analog); **static-IMU invariant** (accel mean = −g in body); **preintegration known-answer test** | the 178° gravity inversion — **at unit-test level, no dataset** |
| **F6** accumulation / noise | **Allan variance** (`allan_variance_ros` / `kalibr_allan`) for σ_g, σ_a → the filter's `Q`; **per-state NEES** (attitude block climbs first); **innovation whiteness** | the drift's noise term |

Three always-on, cross-cutting instruments:

- **NIS** (`νᵀS⁻¹ν`, gate at the χ² quantile — 5.99 for 2-DoF) at every camera
  update. The pieces already exist at the MSCKF chi² gate — **log the statistic,
  don't only threshold it.** Persistently inflated = overconfident `S` /
  ill-conditioning / biased residual; deflated = inflated `R`, ignoring data.
- **NEES** vs ground truth (Monte-Carlo average-NEES) — the canonical
  inconsistent-EKF detector; per-state blocks say *which* state is overconfident.
- **evo / rpg_trajectory_evaluation** offline, with the **Sim3-vs-SE3 ATE split**
  as the key: small Sim3 + huge SE3 = a *scale* problem (F2), not trajectory
  shape; huge ATE + small RPE = global drift / frame offset, not a local model
  bug.
- **Ground-truth injection (pipeline bisection)** — the highest-leverage tool:
  inject GT at stage *k*; the first stage whose injection makes the failure
  vanish is the culprit. Inject GT extrinsics → 21 km vanishes ⇒ F1 config;
  inject GT seed → scale collapse vanishes ⇒ the fault is init/two-view, not the
  filter.

---

## C. What this codebase is missing — the build list

From the contract/observability audit of the SDK, concrete and bounded. Tracked
as issues under the VIO-instrumentation epic.

1. **Assert the gravity-free sign convention as a shared contract** (typed/tagged
   deltas across `InitFrame`/`DynInitKeyframe`; assert in `solve_alignment` that
   its gravity coefficients match `predict()`). The single highest-value gap —
   the exact seam the 178° bug flowed through, still enforced only by matching
   comments (`imu_init.hpp`, `imu_preintegration.hpp`).
2. **Per-component invariant self-checks + a known-answer test per condition:**
   `ImuPreintegrator` (ΔR ∈ SO3 across accumulation, dropped-sample counter for
   the `dt>0` gate); `solve_alignment` (orthonormal keyframe rotations, finite
   `dR_dbg`); `init_window` / `so3_from_matrix` (orthonormality of SfM rotations,
   currently *assumed*, never checked); `two_view` (parallax / E-conditioning /
   Sampson self-check); `pnp` (DLT conditioning, reprojection RMS).
3. **Always-on consistency logging:** innovation + **NIS per camera update**
   (surface from `CameraUpdater::update`, today only pass/fail), covariance
   condition number / trace trend, per-feature triangulation/reprojection
   residual.
4. **Conditioning / excitation / parallax monitors:** report `cond(H)` / σ_min
   from `solve_alignment` (it silently adds a `1e-9` ridge and only fails on
   outright Cholesky breakdown — an under-excited solve currently yields a
   plausible-but-wrong gravity with no warning); an excitation gate beside the
   `gravity_tol` bands; surface KLT's already-computed min-eigenvalue + final
   photometric residual (discarded in `klt.hpp`) so the front end can weight /
   reject low-confidence tracks.
5. **NEES / NIS in `eval/`** — these do not exist and are the single most
   important missing instrument.
6. **A per-stage ground-truth-injection replay harness** — a synthetic-world
   simulator with injection hooks wrapping each SDK stage (preintegration,
   `init_window`, `two_view`, MSCKF update), with the oracle generated by an
   **independent** convention path so it cannot share the code's assumptions (the
   property our fixtures violated).
7. **Make `InitDiagnostics` gate, not just log** — today a 177° tilt is
   observe-only WARN telemetry and the filter is seeded anyway.
8. **IMU / camera stream characterization tools** — Allan variance + bias /
   saturation / timestamp-monotonicity / rate checks on the IMU; exposure / blur
   / brightness / feature-density / parallax profiling + IMU-camera time-sync
   checks on the camera. The first stage of CHARACTERIZE: validate the *input*
   before blaming the estimator (half our bugs were input/calibration or
   input-conditioned).

---

## D. The workflow we should use

**CHARACTERIZE → ASSESS → HYPOTHESIZE → MODEL → TEST**, where each stage produces
a typed, signed, quantitative artifact, and **TEST never runs the whole
pipeline** — it isolates one component against a known answer.

- **CHARACTERIZE** (measure inputs and internal signals *before* touching code):
  Allan-variance the IMU vs the `propagator.hpp` `ImuNoise` defaults; compute the
  sequence's excitation/parallax profile (this is *why* MH_05's static start
  converges and V2_03 (#244) only converges via static — characterize the
  difference, don't edit); plot MSCKF innovations and preintegration residuals
  over time (zero-mean? white? where do they first leave the band?).
- **ASSESS** (raw signals → consistency/observability verdicts): **NEES**
  (state vs injected truth) and **NIS** (innovation vs predicted `S`) — above the
  χ² band ⇒ overconfident covariance / Jacobian / noise; below ⇒ inflated noise.
  **SVD the linearized system** — confirm exactly the 4 unobservable directions
  are in the null space; report `cond(H)`.
- **HYPOTHESIZE** against a *specific* component contract, **with a sign**: "`F[v,θ]
  = −R[a]ₓ dt` has the wrong sign," not "init is broken." Name the component AND
  the term AND the expected sign/scale.
- **MODEL** — predict the quantitative signature *before* testing: "flipped
  `F[v,θ]` ⇒ NEES grows quadratically from the first propagation; velocity
  innovation sign opposite to gravity-aligned motion." A hypothesis you cannot
  turn into a predicted signature is not yet testable.
- **TEST** — isolate ONE component with a known answer from an **independent**
  oracle (so it does not share the code's assumptions — the test that actually
  catches sign/scale/tilt). Only after every component passes do you compose and
  look at ATE — as *confirmation*, not a search signal.

### Per-component pre/post-conditions (sound by construction)

- **ImuPreintegrator** — PRE: dt finite > 0 (done), gyro/accel finite. POST:
  ΔR ∈ SO3 to tol; Δt = Σdt; bias Jacobians agree with finite-difference
  re-integration; **invariant**: adding a constant gravity in the generator
  leaves Δ unchanged (gravity-independence).
- **ImuInitializer** — PRE (static): stationarity gates are *hard refusals*. POST:
  |g| == magnitude to tol; roll/pitch align specific force to −g; yaw flagged
  unobservable. POST (dynamic): `ŝ>0` finite; gravity tilt ≤ tol; **report
  `cond(H)` and return "under-excited" rather than a number** over threshold.
- **init_window** — PRE: ≥2 frames with sufficient parallax. POST: one shared
  scale gauge (assert reprojection consistency); body poses = camera poses
  composed with the **declared** extrinsic direction (assert `T_BS` vs `T_BS⁻¹`);
  scale left explicitly unresolved; reprojection RMSE below threshold.
- **Propagator** — POST: `F` matches the analytic error-state transition by
  finite-difference; P stays SPD; **gravity contract**: a free-fall (`a_m=0`)
  sample integrates to downward acceleration of magnitude g.
- **Camera update** — POST: left null-space projection removes the feature
  Jacobian; S SPD; P not collapsed in the 4 unobservable directions; NIS in the
  χ² band on known geometry.
- **Cross-cutting:** every boundary carrying a pose declares its frame and
  direction (`T_world_body` vs `T_body_world`, camera↔IMU order) as a typed
  wrapper or asserted invariant — so a wrong composition fails *at* the boundary,
  not 5 layers later as "diverged."

---

## E. Applied to #212 (MSCKF translation drift, RPE ~18%)

RPE — not ATE — already localizes this to **local relative-motion error that
accumulates**: a propagation/update problem, not a global gauge one. Do **not**
edit and re-measure ATE. The first three CHARACTERIZE probes, each with a
predicted reveal:

1. **Build NIS into `eval/` and plot per-update innovations + NIS.** NIS
   persistently > χ² band ⇒ overconfident filter (`Q` too small vs Allan, or a
   wrong Jacobian); the axis/time it first leaves the band points at propagation
   vs update. NIS **in-band while RPE is still 18%** ⇒ the filter *thinks* it is
   consistent while drifting — a systematic scale/bias error the covariance does
   not model. This one plot splits #212 into two different root-cause families.
2. **Run the Propagator alone over short windows with GT injected** + finite-
   difference `F` vs analytic (especially `F[v,θ]=−R[a]ₓ dt`, `F[p,v]=I dt`). If
   open-loop translation already drifts ~18%/window before any camera update, the
   bug is propagation/preintegration or the gravity vector (a small tilt or
   magnitude error integrates twice into exactly this steady drift). If open-loop
   is accurate, propagation is exonerated and drift enters via the update.
3. **Characterize the update geometry:** triangulation conditioning, the
   null-space projection residual, and the camera→IMU extrinsic used by the
   updater vs by triangulation. A constant-fraction translation pull (`T_BS` vs
   `T_BS⁻¹`, or a depth-scale mismatch) is precisely a ~constant 18% RPE.

After these three, #212 is partitioned into **{noise mis-tuning, propagation
Jacobian/gravity, update extrinsic/triangulation scale}**, each with a
quantitative signature. *Then* HYPOTHESIZE the specific term (with a sign), MODEL
its predicted NIS / open-loop-drift signature, and TEST that one component
against an oracle.

**Stop rule before any code edit:** do not change a line until the symptom (18%
RPE) is traced to a **named contract violation whose predicted signature matches
an observed instrument reading** — i.e. you can state which component is wrong,
by how much, and in which sign. If you cannot yet answer "which component, what
magnitude, which sign," you are still in the anti-pattern and have not finished
characterizing.
