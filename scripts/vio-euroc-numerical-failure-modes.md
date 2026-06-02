# EuRoC numerical failure modes in a VIO stack — and how we diagnose them

Companion to [`vio-euroc-architecture.md`](./vio-euroc-architecture.md). That
doc describes *what* the `vio_euroc` benchmark runs; this one is about *why it
breaks* — the numerical pathologies that real EuRoC data trips in a
monocular visual-inertial estimator, how each one manifests, the single
diagnostic that distinguishes it from its neighbours, and the fix. It doubles as
a worked example of the **differential-diagnosis method**: on real data a filter
either works or it doesn't, and "it diverged" is not a diagnosis — you have to
localize *which* numerical assumption failed.

The running example is the dynamic visual-inertial initialization epic (**#211**)
validated on **MH_05_difficult** and **V2_03_difficult**. The numbers below are
the actual measurements from the investigation.

---

## 0. The one mental model: observability + conditioning + accumulation

Almost every VIO numerical failure is one of three things:

1. **Unobservable quantity** — the data simply does not constrain it. The
   estimator must *not* try to "measure" it, or it will get a garbage value
   driven by noise. (Metric scale without translational excitation; gravity
   *direction* without acceleration excitation; absolute yaw and position, ever.)
2. **Ill-conditioned estimate** — the quantity *is* observable but the specific
   geometry makes the linear system nearly singular, so noise is amplified
   enormously. (A two-view essential matrix from a near-zero baseline.)
3. **Accumulated error** — a small constant error integrated over time grows
   without bound. (A gyro-bias error → linear attitude drift → gravity leaks
   into acceleration → *quadratic* position divergence.)

EuRoC is a torture test for all three because (a) the camera and IMU are
rigidly offset and rotated (~90°), (b) the "difficult" sequences have low-motion
startups *and* aggressive segments, and (c) the trajectory is long enough
(~2 min) that accumulation dominates. A method that works in simulation (clean
tracks, wide baselines, identity extrinsics) hits all three on the first real
sequence.

---

## 1. The failure modes, in the order they bite

Each is a distinct numerical fault with a distinct signature. The table is the
index; the sections expand each one.

| # | Failure mode | Class | Signature (ATE / diagnostic) | Fix | Result |
|---|---|---|---|---|---|
| F1 | Wrong camera↔IMU extrinsics | frame error | ATE ~21 km, fires but diverges fast | use real `T_BS` | 21 km → 0.79 m |
| F2 | Unobservable metric scale (rank-deficient `[v,g,s]`) | observability | `scale≈1e-8`, `seed_speed≈0`, ATE ~42 km | observability gate (decline) | no divergence |
| F3 | Wrong init-method race | logic | dynamic wins + diverges where static would converge | gate ⇒ dynamic declines, static wins | V2_03 12 km → 0.29 m |
| F4 | Degenerate two-view (low-parallax bootstrap) | conditioning | builds succeed, `best_metric_motion≈0.14 mm` | widest-parallax bootstrap | scale 1e-8 → 0.056, dynamic fires |
| F5a | **Inverted gravity** (sign-convention mismatch) | sign error | sane scale/vel/bias, ATE ~48 km, `tilt ≈ 178°` | match preintegration gravity sign | 48 km → 19 m |
| F5b | Under-observable gravity *direction* (residual) | observability | `tilt ≈ 2.3°`, ATE ~19 m drift | accel roll/pitch / more excitation | (in progress) |
| F6 | Gyro-bias error → attitude drift | accumulation | (ruled out here: dynamic ≈ static bias) | — | n/a this case |

Note the progression: fixing F1 exposed F2; the F2 gate exposed F3 (a happy
accident) and made F4 visible; fixing F4 exposed F5. **Each fix peels back one
layer.** That is normal — you cannot see the next fault until the current one
stops dominating.

---

### F1 — Wrong camera↔IMU extrinsics (a frame error masquerading as divergence)

**What it is.** The visual measurement model needs the camera pose in the body
(IMU) frame, `T_BS`. EuRoC's cam0 is rotated ~90° from the IMU. Initializing
with **identity** extrinsics means every feature's predicted bearing is ~90° off,
so each EKF update pushes the state the wrong way.

**Signature.** ATE ~**21 km** — pure blow-up, not drift. Init *succeeds* (the
filter thinks it is fine), gravity magnitude is fine; only the *measurement
residuals* are systematically wrong. Easy-sequence (slow) data tolerates it for
a while; fast motion amplifies it immediately.

**Why simulation misses it.** Synthetic tests put the camera at the IMU
(identity extrinsics), so the entire extrinsics-composition path is never
exercised. The first real sequence is the first non-identity test.

**Diagnosis.** "It diverges with a healthy-looking init" + "the only thing real
data adds over the passing synthetic test is the extrinsics" ⇒ check the
extrinsics. **Fix:** load the published cam0 `T_BS` (rotation via
`so3_from_matrix`, translation `p_imu_cam`). **Result:** MH_05 21 km → **0.79 m**.

Lesson: a frame/convention error reads exactly like a divergence bug. Always
ask *what does real data exercise that the green unit test does not.*

---

### F2 — Unobservable metric scale (rank-deficient alignment)

**What it is.** Monocular vision recovers translation only up to scale; the
metric scale comes from fusing it with the IMU. The linear alignment for
`[velocity, gravity, scale]` is **rank-deficient without translational
excitation**: on a near-hover window, a constant velocity offset trades off
against gravity magnitude and scale, and the least-squares solution drives
`scale → 0` — the trivial "ignore the vision, use IMU only" solution.

**Signature.** Dynamic init *fires* (`InitMethod::Dynamic`), but
`dyn_scale ≈ 1.1e-8`, `seed_speed ≈ 0.01 m/s` (because a ~0 scale zeroes the
metric velocities), and ATE ~**42 km**. Crucially `grav_residual ≈ 0` — which is
**not** reassurance: the refinement *forces* `|g| = G`, so a near-zero residual
says nothing about correctness.

**The trap.** The original code rejected only `scale ≤ 0`. But `1e-8 > 0`, so the
degenerate seed passed and diverged the filter.

**Diagnosis.** Surface `scale` and the *resolved metric motion*
(`scale · max-keyframe-displacement`) **even on the decline path**. A scale
orders-of-magnitude below the physical baseline ⇒ unobservable. **Fix:** an
observability gate — require `scale · displacement ≥ min_dynamic_motion` (5 cm),
else **decline** (return failure) so the backend retries on a better-excited
window or falls back to gravity-only. Never seed an unobservable quantity.

---

### F3 — Init-method race (and a free win)

**What it is.** The backend tries static init (stationary window) and dynamic
init (SfM window) concurrently while uninitializing; whichever fires first wins.
Before the F2 gate, dynamic init won the race on V2_03 with a degenerate scale
and diverged.

**Signature / diagnosis.** `InitMethod` in the diagnostics tells you which path
seeded the filter — "the single most useful divergence-triage signal." V2_03 was
`init=dynamic` + ~12 km.

**Fix (free).** The F2 gate makes dynamic init *decline* the degenerate early
window, so the more-accurate static path wins instead. **Result:** V2_03 12 km →
**0.29 m** with no V2_03-specific work. Fixing an observability bug upstream
fixed an apparently-unrelated sequence — because they shared a root cause.

---

### F4 — Degenerate two-view bootstrap (a conditioning failure)

**What it is.** The SfM window fixes its up-to-scale gauge from a two-view
essential matrix between two frames. At 20 Hz, **adjacent** frames are ~0.05 s /
a few cm apart — near-zero parallax. The essential matrix is then **degenerate**
(the epipolar geometry is unconstrained along the motion direction), so the
recovered relative pose and triangulated depths are garbage, and the whole
window's metric scale collapses.

**Signature.** `window_builds = 26` (the SfM *succeeds* — it returns poses) but
`best_metric_motion ≈ 0.14 mm` across **every** window — systematic, not noisy.
A consistent sub-mm "resolved motion" where the drone clearly moves cm/frame is
the fingerprint of a collapsed gauge, not a missing one.

**Why it's distinct from F2.** F2 is "scale is unobservable from this data"
(physics). F4 is "scale *is* observable but our estimator threw it away because
the bootstrap geometry was singular" (numerics). Same symptom (small scale),
opposite cause — which is why you need the `window_builds` counter to tell them
apart: F2 has `window_builds = 0`-ish quality, F4 has builds succeeding with
collapsed scale.

**Diagnosis.** Instrument the *attempt accounting*: how many windows built, and
the best resolved motion across them. Builds succeeding + motion collapsed ⇒
conditioning, not observability. **Fix:** pick the bootstrap pair by **largest
parallax** (a cheap mean-disparity scan, then one RANSAC) instead of the adjacent
pair. **Result:** `scale 1e-8 → 0.056`, `best_metric_motion 0.14 mm → 56 mm`,
dynamic init **fires** with a physical seed.

(See also the performance note in §3: searching *every* candidate baseline with a
full RANSAC is correct but quadratic — select cheaply, solve once.)

---

### F5 — Wrong gravity *direction* (first a sign bug, then under-observability)

This one is in two layers, and they teach different lessons.

**The differential diagnosis (the heart of the method).** The same sequence
converges to 0.79 m via the *static* seed but diverged via *dynamic*. Static and
dynamic differ only in {roll/pitch source, yaw, velocity, bias}. Eliminate them
one at a time on the *same data*:
- **Gyro bias?** Print it for both paths. `static |bg| = 0.0805`,
  `dynamic |bg| = 0.0820` — essentially identical, and static converges with it.
  **Ruled out.**
- **Velocity magnitude?** 0.18 m/s is small and correctable; static seeds 0 and
  converges. **Unlikely.**
- **Yaw?** A gauge freedom — ATE's rigid alignment absorbs a global yaw. **Can't
  cause blow-up.**
- **Roll/pitch (gravity direction)?** The only remaining seed component, and the
  *only* one whose error produces quadratic divergence. **Prime suspect.**

So we instrumented it: the angle between the seed's world-up and the mean
accelerometer up (`roll/pitch err vs accel`).

**F5a — inverted gravity (a sign bug).** The tilt came back at **177.7°** — a
*near-perfect inversion*, not the random spread that under-observability would
produce. That distinction is the whole lesson: **a consistent ~180° is
deterministic, so look for a sign error, not noise.** The cause was a
gravity-sign mismatch — `solve_alignment` encoded `R_iΔv = v_j − v_i + g·Δt`
while the `ImuPreintegrator` (gravity-free Δv/Δp, per `predict()`) gives
`R_iΔv = v_j − v_i − g·Δt`. Fed the real preintegration, the solve returned
`g = −g_true`; "up" pointed down; ~2g of phantom acceleration; ATE ~**48 km**.
And the synthetic fixtures built Δv/Δp with the *same* flipped sign, so they
passed — a **compensating error** that hid the bug from CI until real
(correctly-signed) preintegration met the alignment. **Fix:** flip every gravity
coefficient in `solve_alignment` to match `predict()`, and correct the fixtures
to the physical convention so they actually test it. **Result:** tilt 177.7° →
2.3°, ATE 48 km → **19.3 m** — divergence gone.

**F5b — under-observable gravity direction (the residual).** *Now* the original
F5 hypothesis is what remains: the residual **2.3°** roll/pitch is genuine
under-observability — on MH_05's slow (~0.1 m/s) start there is too little
acceleration excitation to pin the gravity *direction* tightly, leaving ~19 m of
drift (plus pre-init frames from late firing inflating ATE). **Candidate fixes:**
seed roll/pitch from the accelerometer (reliable over a short window) and use the
VI solve only for yaw/velocity/scale; and fire earlier or exclude pre-init
frames. *(Under investigation; tracked in #247, blocking #211.)*

**Lesson.** `grav_residual ≈ 0` certified the magnitude and lulled the triage;
the *direction* needed its own diagnostic. And the shape of the error — a clean
~180° vs a few noisy degrees — told us *sign bug* vs *under-observability*, two
faults with the same symptom and completely different fixes.

---

### F6 — Gyro-bias error → attitude drift (accumulation; ruled out here)

**What it is.** The canonical long-horizon VIO failure: a constant gyro-bias
error `δb` produces attitude error `δb·t`, which mis-rotates gravity by `δb·t`,
which injects `g·sin(δb·t)` acceleration, which integrates to ~`½ g δb t²`
position error. At `δb = 0.01 rad/s`, `t = 120 s` that is tens of km — so even a
*tiny* bias error is fatal over a full sequence.

**Why it's listed even though it's clean here.** It is the *mechanism* behind
F1/F5's blow-ups (a wrong measurement model or a wrong roll/pitch leaks gravity
the same way), and it is the first thing to *rule out* by printing the bias.
Here, static and dynamic agree on `|bg| ≈ 0.08`, so the bias *estimate* is fine;
the leak is coming from roll/pitch (F5), not bias.

---

## 2. The method, abstracted

What the investigation above actually *is*, as a reusable procedure:

1. **Reproduce on real data behind a gate.** Keep it out of CI (dataset is huge,
   nondeterministic), but make it one command locally. Real data is the only
   place these faults appear.
2. **"Diverged" is a symptom, not a diagnosis.** Quantify it: ATE *magnitude*
   (km = blow-up, m = drift, sub-m = converged) and *which init method* fired.
3. **Instrument the internal/decline path, not just pass/fail.** Most faults hide
   in the values the code computes and then discards. Surface scale, seed speed,
   bias, attempt counts, resolved motion, tilt — *even when the step declines*.
4. **Differential diagnosis against a working path.** When one configuration
   converges and another diverges on the *same data*, enumerate exactly how their
   seeds differ and eliminate components one by one. This is what turned "dynamic
   init diverges" into "the gravity *direction* is under-observed."
5. **Reason about observability before reaching for a fix.** Ask *is this
   quantity even determined by the data here?* If not, the fix is to **decline /
   defer**, never to fit harder. (F2, F5.)
6. **Separate observability from conditioning.** Same symptom (collapsed scale),
   different cause (no excitation vs singular geometry). The counter that says
   "the step *succeeded* but produced garbage" is what distinguishes them. (F4 vs
   F2.)
7. **Expect each fix to expose the next fault.** Peeling layers is progress, not
   regression. Record the layer you fixed and the new one it revealed.
8. **Never measure a gauge freedom.** Global position, absolute yaw — these are
   unobservable *by construction*. ATE's rigid alignment absorbs them; the
   estimator must too.

---

## 3. Interpreting the benchmark output

The `[dataset]` cases WARN a fixed vocabulary; here is how to read it.

**`init=` method.** `static` (had a quiet start), `dynamic` (VI alignment fired),
`gravity_align` (timeout fallback — roll/pitch from accel only, no yaw/velocity),
`identity` (last resort — mean force wasn't even gravity-like; expect
divergence). This is the first thing to read.

**ATE magnitude as a class label.**
- **< 1 m** — converged. This MVP MSCKF (no loop closure, no online calibration)
  lands MH_05 static at 0.79 m, V2_03 static at 0.29 m.
- **1–100 m** — drift / partial failure (a marginal seed the filter half-recovers
  from).
- **> 1 km** — divergence. Read it as "which numerical assumption broke," per §1,
  not as "needs tuning." You cannot tune your way out of a blow-up.

**The dynamic-seed fields** (`scale`, `seed_speed`, `grav_residual`,
`sfm_keyframes`, `best_metric_motion`, `roll/pitch err vs accel`,
`window_builds`, `dyn_attempts`) each isolate one failure mode — see the
parenthetical in each F-section. Two cross-cutting cautions:
- `grav_residual ≈ 0` is **forced** (|g| pinned to G); it certifies the magnitude,
  *never* the direction. Do not read it as "gravity is correct."
- A small `scale` is ambiguous until you check `window_builds`: F2 (unobservable)
  vs F4 (degenerate). Same number, opposite fix.

**ATE is rigid SE3, not similarity.** The Horn alignment removes a global
rotation + translation but **not** a scale — so a metrically-wrong estimate
shows up in ATE (deliberate: scale is part of what VIO must get right). A global
*yaw* or *position* offset, by contrast, is absorbed — so never chase those.

**Performance is a correctness concern too.** A per-frame init retry that runs a
full RANSAC on every candidate baseline over a wide window is *correct* but
grinds to an apparent hang. Select candidates cheaply (a disparity scan), run the
expensive solve once, and bound the window. A method that is right but
unschedulable is not done.

---

## 4. Where this leaves the stack

- **Estimator + calibration on real data:** working. Both difficult sequences
  converge sub-meter via static/gravity-align init (F1, F2, F3 fixed).
- **Dynamic VI-init:** fires on real data with a physical scale (F4) and a
  correct gravity *direction* (F5a — the sign bug — fixed: 48 km → 19 m, no
  longer diverges). The last layer is F5b, the residual roll/pitch
  under-observability on low-excitation startups (~2.3° → ~19 m drift), between
  "bounded" and "within gate" — tracked in **#247**, blocking the **#211** epic.
- **The diagnostics built along the way are permanent** — `InitDiagnostics` now
  carries the scale, seed speed, attempt accounting, and tilt fields, so the next
  time a sequence misbehaves the failure mode is one run away, not one
  investigation away.

See [`vio-euroc-architecture.md`](./vio-euroc-architecture.md) for the pipeline
and frames, and the `#211` / `#247` issue threads for the live state.
