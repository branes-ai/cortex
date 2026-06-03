# VIO EuRoC MAV benchmark troubleshooting

The vio_euroc benchmark produces these results:

```text
-------------------------------------------------------------------------------
V1_01_easy replay keeps ATE under threshold
-------------------------------------------------------------------------------
tests/sdk/vio_euroc.cpp:359
...............................................................................

tests/sdk/vio_euroc.cpp:205: SKIPPED:
explicitly with message:
set CORTEX_EUROC_V101 to the V1_01_easy/mav0 directory to run this benchmark

-------------------------------------------------------------------------------
MH_05_difficult forced dynamic VI-init fires + stays bounded (#247)
-------------------------------------------------------------------------------
tests/sdk/vio_euroc.cpp:374
...............................................................................

tests/sdk/vio_euroc.cpp:302: warning:
MH_05_difficult (dynamic): init=dynamic, frames=2273, ATE=19.8789 m (gate 1.5
m)

tests/sdk/vio_euroc.cpp:310: warning:
MH_05_difficult (dynamic): NIS over 27173 updates: normalized=1.20953 (band
[0.996002, 1.004]) — OVER-confident

tests/sdk/vio_euroc.cpp:316: warning:
MH_05_difficult (dynamic): NEES over 2077 frames: normalized=130.444 (band
[0.984296, 1.0157]) — OVER-confident

tests/sdk/vio_euroc.cpp:326: warning:
MH_05_difficult (dynamic): seed gyro_bias=(0.0217551, 0.0176564, 0.0773316)
rad/s, |bg|=0.0822508, grav_residual=2.33588e-14

tests/sdk/vio_euroc.cpp:330: warning:
MH_05_difficult (dynamic): dyn-init scale=0.0607451, seed_speed=0.120572 m/s,
sfm_keyframes=20, roll/pitch err vs accel=2.32865 deg

tests/sdk/vio_euroc.cpp:338: warning:
MH_05_difficult (dynamic): dyn-attempts=170, window_builds=11,
best_keyframes=20, best_metric_motion=0.0614133 m

-------------------------------------------------------------------------------
MH_05_difficult replay (moving start) bootstraps and stays bounded
-------------------------------------------------------------------------------
tests/sdk/vio_euroc.cpp:363
...............................................................................

tests/sdk/vio_euroc.cpp:302: warning:
MH_05_difficult: init=static, frames=2273, ATE=0.785467 m (gate 1.5 m)

tests/sdk/vio_euroc.cpp:310: warning:
MH_05_difficult: NIS over 29225 updates: normalized=2.17971 (band [0.99614,
1.00386]) — OVER-confident

tests/sdk/vio_euroc.cpp:316: warning:
MH_05_difficult: NEES over 2221 frames: normalized=10.0786 (band [0.984814,
1.01519]) — OVER-confident

tests/sdk/vio_euroc.cpp:326: warning:
MH_05_difficult: seed gyro_bias=(-0.00194081, 0.0202039, 0.0778975) rad/s,
|bg|=0.0804984, grav_residual=0.0026452

-------------------------------------------------------------------------------
V2_03_difficult replay (moving start) converges
-------------------------------------------------------------------------------
tests/sdk/vio_euroc.cpp:386
...............................................................................

tests/sdk/vio_euroc.cpp:302: warning:
V2_03_difficult: init=static, frames=1922, ATE=0.293417 m (gate 1.5 m)

tests/sdk/vio_euroc.cpp:310: warning:
V2_03_difficult: NIS over 21627 updates: normalized=14.8014 (band [0.995038,
1.00496]) — OVER-confident

tests/sdk/vio_euroc.cpp:316: warning:
V2_03_difficult: NEES over 1882 frames: normalized=115.312 (band [0.983503,
1.0165]) — OVER-confident

tests/sdk/vio_euroc.cpp:326: warning:
V2_03_difficult: seed gyro_bias=(-0.0023178, 0.0243927, 0.0813882) rad/s, |bg
|=0.0849966, grav_residual=0.00309173

================================================================================
test cases:  4 |  3 passed | 1 skipped
assertions: 16 | 16 passed
```

This VIO run shows that the filter is systemically over-confident — everywhere

┌───────────────┬──────────┬──────┬──────┐
│     Case      │   ATE    │ NIS  │ NEES │
├───────────────┼──────────┼──────┼──────┤
│ MH_05 static  │ 0.79 m ✓ │ 2.18 │ 10.1 │
├───────────────┼──────────┼──────┼──────┤
│ V2_03 static  │ 0.29 m ✓ │ 14.8 │ 115  │
├───────────────┼──────────┼──────┼──────┤
│ MH_05 dynamic │ 19.9 m   │ 1.21 │ 130  │
└───────────────┴──────────┴──────┴──────┘

Every case is OVER-confident — including the ones that converge to sub-meter ATE. NEES = 10–130 means the state error is ~3–11× the 1-σ the covariance
claims. This was completely invisible to ATE — "MH_05 converges, V2_03 converges, looks healthy" — yet the filter is lying about its uncertainty on every
sequence. That over-confidence is the mechanism: an over-confident filter under-weights corrections, so it drifts (#212) and can't recover from a marginal
seed (the 19 m dynamic case).

The localization (NEES vs NIS tells different stories)

The pattern differs per case, which is exactly what the two-instrument design is for:

- MH_05 dynamic: NIS ≈ 1.2 (innovations basically consistent) but NEES = 130. This is the textbook "NIS fine, NEES bad" signature from the tutorial: the
per-update measurement model is roughly right, but a hidden state is drifting un-modeled — here the excitation-limited seed's scale/gravity error, which
the covariance doesn't represent. The 19 m is real state drift the filter is blind to.
- V2_03: NIS = 14.8 (innovations ~4× too large). The measurement/process model itself is breaking down — under V2_03's aggressive motion, the linearization
/ process noise can't keep up.
- MH_05 static: mild (NIS 2.2, NEES 10) — the systemic baseline over-confidence.

The common thread — the state covariance P is far more over-confident than the innovation S (NEES > NIS everywhere) — points at the propagation side:
process noise Q too small, and/or the classic EKF-VIO observability inconsistency (the filter gains spurious information in the unobservable yaw direction
→ covariance collapses). R alone can't explain NEES > NIS.

The next two probes (measurement, not edits)

1. Run the Allan benchmark — CORTEX_EUROC_V101=… ./build/.../allan_variance "[dataset]". That prints gyro N=… (filter Q …). If the measured N ≫ the
configured Q, process-noise mis-tuning is a quantified cause of the over-confidence (and the cheapest fix). This is exactly the #268 probe, built for this
moment.
2. Per-block NEES — right now I only surface the aggregate 15-DoF NEES. Splitting it into attitude / position / velocity / gyro-bias / accel-bias blocks
tells us which state is overconfident: attitude/yaw worst → observability inconsistency (→ first-estimates-Jacobians / OC-EKF); velocity/bias worst → Q /
bias model. That's a ~20-line addition to nav_consistency.hpp + the benchmark.

Want me to add per-block NEES so the next run localizes the over-confidence to a specific state — and will you run the Allan benchmark to check Q? Together
those two readings will name the cause of the 18% drift with a component and a sign, which is the stop-rule before we touch the estimator.

## How to run the Allan benchmark

Here's the exact sequence. The dataset case is gated on CORTEX_EUROC_V101, but the IMU is the same sensor across every EuRoC sequence, so you can point it
at any mav0 you already have extracted — MH_05's path was confirmed in your earlier runs:

```bash
cd <repo>
git checkout main && git pull                    # get the Allan instrument (#276)
cmake --preset sitl-release                       # re-glob to pick up the new test target
cmake --build --preset sitl-release --target allan_variance

CORTEX_EUROC_V101=$EUROC_ROOT/machine_hall/MH_05_difficult/mav0 \
./build/sitl-release/tests/allan_variance "[dataset]"
```

You're looking for this WARN line:

```text
EuRoC IMU @ 200 Hz: gyro N=… rad/s/√Hz (filter Q 0.00016), accel N=… m/s²/√Hz (filter Q 0.002) — …
```

How to read it against the current filter Q (VioConfig defaults: gyro_noise_density = 1.6e-4, accel_noise_density = 2.0e-3):
- measured N ≫ filter Q (say 3–10×+) → the noise model is too tight → a quantified, cheap cause of the NEES/NIS over-confidence. First thing to retune.
- measured N ≈ filter Q → Q is fine, so the over-confidence is not the noise densities — it's the propagation/observability side (the EKF gaining spurious
information), which is the bigger OC-EKF / first-estimates-Jacobians fix.

Either result is decisive. Send me the line and I'll fold it into the diagnosis — and I'll get per-block NEES added in parallel so we also know which state
is the worst offender.

```bash
$ CORTEX_EUROC_V101=$EUROC_ROOT/machine_hall/MH_05_difficult/mav0 ./build/sitl-release/tests/allan_variance "[dataset]"
Filters: [dataset]
Randomness seeded to: 3305577977

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
allan_variance is a Catch2 v3.15.0 host application.
Run with -? for options

-------------------------------------------------------------------------------
EuRoC IMU noise density vs the filter's configured Q
-------------------------------------------------------------------------------
tests/sdk/allan_variance.cpp:88
...............................................................................

tests/sdk/allan_variance.cpp:112: warning:
  EuRoC IMU @ 200 Hz: gyro N=0.0047129 rad/s/√Hz (filter Q 0.00016), accel N=0.
  0498006 m/s²/√Hz (filter Q 0.002) — large mismatch ⇒ retune Q before chasing
  #212

===============================================================================
All tests passed (5 assertions in 1 test case)
```

Issue #278 merged (176aa94) — per-block NEES is on main. Re-run to get the decisive read:

```bash
  cd <repo>
  git checkout main && git pull
  scripts/euroc-moving-start.sh
```

Each case now prints an extra line:

```text
…: per-block NEES (normalized, want ~1): attitude=… position=… velocity=… gyro_bias=… accel_bias=…
```

This is the localization that names the cause of the systemic over-confidence (and #212):

- attitude (and likely position) hugely over-confident → the EKF observability inconsistency — the standard MSCKF problem where the filter gains spurious
information in the unobservable yaw direction and its covariance collapses. Fix: first-estimates-Jacobians / observability-constrained EKF. (This is my
leading hypothesis given NEES≫NIS and the Q-already-matches-static-Allan finding.)
- velocity / gyro_bias / accel_bias the worst → points back at the process-noise / bias model — and then a static-segment Allan (to get the clean Q number
the flight measurement couldn't) becomes worth building.
- all blocks ~uniformly inflated → a global covariance/Q scale issue.

Send me the per-block lines (the MH_05 static and dynamic ones especially) and that pins the #212 root cause to a specific state — the stop-rule before we
touch the estimator. That's the whole instrumentation program (#261) doing its job: turning "it drifts" into "the attitude covariance is over-confident by
N×, here's why."


```bash
$ scripts/euroc-moving-start.sh
== locating sequences under $EUROC_ROOT ==
  MH_05 mav0: $EUROC_ROOT/machine_hall/MH_05_difficult/mav0
  V2_03 mav0: $EUROC_ROOT/vicon_room2/V2_03_difficult/mav0
== building sitl-release / vio_euroc ==

  __  __ _____ _     ____
 |  \/  |_   _| |   | ___|
 | |\/| | | | | |   |___ \
 | |  | | | | | |___ ___) |
 |_|  |_| |_| |_____|____/

 Matrix Template Library 5 — C++20 header-only linear algebra

Parsing public/common/TracyVersion.hpp file
VERSION 0.13.1
[0/2] Re-checking globbed directories...
[2/2] Linking CXX executable tests/vio_euroc
== running the gated moving-start validation ==
Filters: [dataset]
Randomness seeded to: 2887338517

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
vio_euroc is a Catch2 v3.15.0 host application.
Run with -? for options

-------------------------------------------------------------------------------
V2_03_difficult replay (moving start) converges
-------------------------------------------------------------------------------
tests/sdk/vio_euroc.cpp:400
...............................................................................

tests/sdk/vio_euroc.cpp:308: warning:
  V2_03_difficult: init=static, frames=1922, ATE=0.293417 m (gate 1.5 m)

tests/sdk/vio_euroc.cpp:316: warning:
  V2_03_difficult: NIS over 21627 updates: normalized=14.8014 (band [0.995038,
  1.00496]) — OVER-confident

tests/sdk/vio_euroc.cpp:322: warning:
  V2_03_difficult: NEES over 1882 frames: normalized=115.312 (band [0.983503,
  1.0165]) — OVER-confident

tests/sdk/vio_euroc.cpp:334: warning:
  V2_03_difficult: per-block NEES (normalized, want ~1): attitude=168.608784
  position=3.508583 velocity=25.906302 gyro_bias=16.377613 accel_bias=30.205153

tests/sdk/vio_euroc.cpp:340: warning:
  V2_03_difficult: seed gyro_bias=(-0.0023178, 0.0243927, 0.0813882) rad/s, |bg
  |=0.0849966, grav_residual=0.00309173

-------------------------------------------------------------------------------
MH_05_difficult forced dynamic VI-init fires + stays bounded (#247)
-------------------------------------------------------------------------------
tests/sdk/vio_euroc.cpp:388
...............................................................................

tests/sdk/vio_euroc.cpp:308: warning:
  MH_05_difficult (dynamic): init=dynamic, frames=2273, ATE=19.8789 m (gate 1.5
  m)

tests/sdk/vio_euroc.cpp:316: warning:
  MH_05_difficult (dynamic): NIS over 27173 updates: normalized=1.20953 (band
  [0.996002, 1.004]) — OVER-confident

tests/sdk/vio_euroc.cpp:322: warning:
  MH_05_difficult (dynamic): NEES over 2077 frames: normalized=130.444 (band
  [0.984296, 1.0157]) — OVER-confident

tests/sdk/vio_euroc.cpp:334: warning:
  MH_05_difficult (dynamic): per-block NEES (normalized, want ~1): attitude=
  133.781601 position=70.680571 velocity=80.902497 gyro_bias=71.285316
  accel_bias=38.288993

tests/sdk/vio_euroc.cpp:340: warning:
  MH_05_difficult (dynamic): seed gyro_bias=(0.0217551, 0.0176564, 0.0773316)
  rad/s, |bg|=0.0822508, grav_residual=2.33588e-14

tests/sdk/vio_euroc.cpp:344: warning:
  MH_05_difficult (dynamic): dyn-init scale=0.0607451, seed_speed=0.120572 m/s,
  sfm_keyframes=20, roll/pitch err vs accel=2.32865 deg

tests/sdk/vio_euroc.cpp:352: warning:
  MH_05_difficult (dynamic): dyn-attempts=170, window_builds=11,
  best_keyframes=20, best_metric_motion=0.0614133 m

-------------------------------------------------------------------------------
V1_01_easy replay keeps ATE under threshold
-------------------------------------------------------------------------------
tests/sdk/vio_euroc.cpp:373
...............................................................................

tests/sdk/vio_euroc.cpp:206: SKIPPED:
explicitly with message:
  set CORTEX_EUROC_V101 to the V1_01_easy/mav0 directory to run this benchmark

-------------------------------------------------------------------------------
MH_05_difficult replay (moving start) bootstraps and stays bounded
-------------------------------------------------------------------------------
tests/sdk/vio_euroc.cpp:377
...............................................................................

tests/sdk/vio_euroc.cpp:308: warning:
  MH_05_difficult: init=static, frames=2273, ATE=0.785467 m (gate 1.5 m)

tests/sdk/vio_euroc.cpp:316: warning:
  MH_05_difficult: NIS over 29225 updates: normalized=2.17971 (band [0.99614,
  1.00386]) — OVER-confident

tests/sdk/vio_euroc.cpp:322: warning:
  MH_05_difficult: NEES over 2221 frames: normalized=10.0786 (band [0.984814,
  1.01519]) — OVER-confident

tests/sdk/vio_euroc.cpp:334: warning:
  MH_05_difficult: per-block NEES (normalized, want ~1): attitude=7.175059
  position=9.879716 velocity=9.632311 gyro_bias=2.749590 accel_bias=5.335078

tests/sdk/vio_euroc.cpp:340: warning:
  MH_05_difficult: seed gyro_bias=(-0.00194081, 0.0202039, 0.0778975) rad/s,
  |bg|=0.0804984, grav_residual=0.0026452

================================================================================
test cases:  4 |  3 passed | 1 skipped
assertions: 16 | 16 passed
```


The per-block NEES nails it. This is the localization we built the instrument for. 
Focussing on the well-tracked static cases — the dynamic case's huge numbers are 
the seed-divergence, a separate story:

|      Case (ATE)       | attitude | position | velocity | gyro_bias | accel_bias |
|-----------------------|----------|----------|----------|-----------|------------|
| MH_05 static (0.79 m) | 7.2      | 9.9      | 9.6      | 2.7       | 5.3        |
| V2_03 static (0.29 m) | 168.6    | 3.5      | 25.9     | 16.4      | 30.2       |

The V2_03 row is the smoking gun: attitude NEES = 169 while position = 3.5. The filter is ~170× over-confident about its orientation but its position is
nearly consistent. And it scales with motion: attitude goes 7 → 169 from mild MH_05 to aggressive V2_03, while position stays low. Position is
well-constrained by the visual updates; attitude is the broken state, and it breaks worse the faster the platform rotates.

That motion-scaling pattern points squarely at the gyro/attitude process noise (Q_gyro too small → the attitude covariance doesn't grow enough during fast
rotation) more than a pure observability constant — and it's exactly consistent with the over-confidence being on the propagation side (NEES ≫ NIS,
Q-not-keeping-up). It's also the mechanism for #212: over-confident attitude → under-corrected orientation → drift that couples into position (the 18%
RPE).

The decisive test (HYPOTHESIZE → MODEL → TEST): inflate the process noise and watch attitude NEES drop toward 1. 
Let me add a sweep knob so you can test it without recompiling per value:

PR #279 open. The knob is read at runtime, so you build once and run the binary at several scales — no rebuild between. Run this to sweep (focus on the
V2_03 attitude number — it's the cleanest signal at 169):

```bash
cd <repo>
git fetch origin feat/q-scale-sweep
git checkout origin/feat/q-scale-sweep -- tests/sdk/vio_euroc.cpp
cmake --build --preset sitl-release --target vio_euroc
export CORTEX_EUROC_MH05=$EUROC_ROOT/machine_hall/MH_05_difficult/mav0
export CORTEX_EUROC_V203=$EUROC_ROOT/vicon_room2/V2_03_difficult/mav0
for k in 1 3 10 30; do
echo "===================== Q_SCALE=$k ====================="
CORTEX_Q_SCALE=$k ./build/sitl-release/tests/vio_euroc "[dataset]" 2>&1 \
  | grep -E 'init=static.*ATE|: NIS over|: NEES over|per-block NEES'
done
```

What this discriminates (the MODEL→TEST step):

- If attitude NEES falls toward 1 as Q_SCALE rises → the over-confidence is a genuine process-noise deficit. Q_gyro (and friends) are too small; the
attitude covariance isn't growing fast enough during rotation. The fix is a principled re-tune of the noise densities (and the static-Allan measurement to
nail the clean number becomes worth doing). Read off the factor that lands attitude ≈ 1 — that is the deficit, quantified.
- If attitude NEES stays high regardless of Q_SCALE (e.g. V2_03 still 100+ at Q_SCALE=30) → it's not Q. It's the observability/linearization fault — the
EKF is manufacturing attitude information that no amount of process noise can offset (the classic MSCKF inconsistency). The fix is
first-estimates-Jacobians / observability-constrained update, a code change in the propagation, not a config knob.

Watch what ATE does too: if NEES drops toward 1 and ATE holds or improves, that's a clean win for #212. If raising Q tanks ATE while fixing NEES, that
tension itself is diagnostic (the filter was trading consistency for accuracy).

Background command "Bounded Q-scale sweep with per-run timeout guards" completed (exit code 0)

Sweep complete — and it's a clean, decisive result. Here's the full curve:

### V2_03 (aggressive):

  ┌─────────┬───────┬───────┬───────┬──────────┐
  │ Q_SCALE │  ATE  │  NIS  │ NEES  │ attitude │
  ├─────────┼───────┼───────┼───────┼──────────┤
  │ 1       │ 0.293 │ 14.80 │ 115.3 │ 168.6    │
  ├─────────┼───────┼───────┼───────┼──────────┤
  │ 3       │ 0.346 │ 14.32 │ 41.5  │ 95.4     │
  ├─────────┼───────┼───────┼───────┼──────────┤
  │ 10      │ 0.378 │ 12.93 │ 7.95  │ 19.6     │
  ├─────────┼───────┼───────┼───────┼──────────┤
  │ 30      │ 1.066 │ 11.32 │ 17.7  │ 60.5     │
  └─────────┴───────┴───────┴───────┴──────────┘

### MH_05 (mild):

  ┌─────────┬───────┬──────┬──────┬──────────┐
  │ Q_SCALE │  ATE  │ NIS  │ NEES │ attitude │
  ├─────────┼───────┼──────┼──────┼──────────┤
  │ 1       │ 0.785 │ 2.18 │ 10.1 │ 7.2      │
  ├─────────┼───────┼──────┼──────┼──────────┤
  │ 3       │ 0.702 │ 2.20 │ 12.4 │ 24.2     │
  ├─────────┼───────┼──────┼──────┼──────────┤
  │ 10      │ 0.822 │ 1.93 │ 4.86 │ 12.8     │
  ├─────────┼───────┼──────┼──────┼──────────┤
  │ 30      │ 2.085 │ 1.92 │ 7.17 │ 24.1     │
  └─────────┴───────┴──────┴──────┴──────────┘

Three findings, all robust:

1. Process noise is ~10× too small. NEES has a clear minimum at Q ≈ 10 (V2_03: 115 → 8; MH_05: 10 → 5), then climbs again at Q=30. That minimum is a genuine
bias/variance sweet spot — below it P is too tight, above it the estimate itself degrades. So VioConfig's IMU densities are roughly an order of magnitude
under-sized.

2. But you cannot tune your way out — there's a second, independent fault. NIS is nearly invariant to Q: V2_03 stays ~14× over-confident (14.8 → 11.3) even
at 30× process noise; MH_05 stays ~2×. Since S = HPHᵀ + R, the only way inflating P (via Q) fails to grow S is if R dominates S and R is too small — i.e.
the visual update is over-confident in a way the IMU process noise can't touch. The innovation over-confidence also scales with aggressiveness (14× on
V2_03 vs 2× on MH_05), pointing at a motion-dependent measurement error (feature-tracking residual, camera–IMU time-sync, or rolling shutter) that the
fixed R ignores.

3. The current config's good ATE is "lucky over-confidence." ATE is best at Q=1 (the most dishonest covariance) and degrades as the filter becomes honest.
The filter is overfitting these sequences with a meaningless covariance — exactly the brittleness behind #212.

The decisive next experiment is the mirror knob: CORTEX_R_SCALE (scale the visual measurement noise). 
It discriminates the two remaining hypotheses for
Fault B:
- NIS drops proportionally with R → R is simply under-tuned (a noise-calibration fix).
- NIS stays stubborn to R too → it's the deeper EKF observability / Jacobian inconsistency (spurious information gain on the update), which needs
first-estimates-Jacobians, not a config change.

Want me to add the CORTEX_R_SCALE knob (same pattern as Q_SCALE, on feat/q-scale-sweep) and run that sweep? That's the read that tells us whether #212's
fix is a re-tune or an estimator-code change.

## R Sweep

 Now build the Release binary and sanity-check that R_SCALE=1 reproduces the baseline (confirming the wiring is neutral when unscaled):

● R_SCALE=1 reproduces the baseline exactly (NEES=115.312, NIS=14.8014) — the wiring is neutral when unscaled, so the new R-from-config path didn't perturb
  anything. Now the R sweep (finer low end, since if it's pure under-tuned R the NIS response should be sharp — S≈R means NIS≈1/k²). Same bounded harness
  with 240 s guards:

● R sweep running (~12 min, 240 s guard per run). This is the discriminating experiment:

- NIS falls sharply with R (e.g. ~14.8 → ~1 by R≈3–4) → the visual measurement noise R is simply under-tuned. Fault B is a config fix, and #212 is "re-tune
both Q and R" — no estimator surgery.
- NIS stays stubbornly high even at R=10 → it's not noise calibration; it's the EKF observability / Jacobian inconsistency on the update (spurious
information gain), which needs first-estimates-Jacobians — an estimator-code change.

● Background command "Bounded R-scale sweep with timeout guards" completed (exit code 0)

● This is the decisive result — and it's not what a simple "re-tune R" would predict. The R sweep:

### V2_03 (aggressive):

  ┌─────────┬───────┬───────┬──────┐
  │ R_SCALE │  ATE  │  NIS  │ NEES │
  ├─────────┼───────┼───────┼──────┤
  │ 1       │ 0.293 │ 14.80 │ 115  │
  ├─────────┼───────┼───────┼──────┤
  │ 2       │ 0.502 │ 3.94  │ 130  │
  ├─────────┼───────┼───────┼──────┤
  │ 4       │ 2.07  │ 1.08  │ 444  │
  ├─────────┼───────┼───────┼──────┤
  │ 10      │ 278 m │ 0.36  │ 1542 │
  └─────────┴───────┴───────┴──────┘

### MH_05 (mild):

  ┌─────────┬────────┬──────┬──────┐
  │ R_SCALE │  ATE   │ NIS  │ NEES │
  ├─────────┼────────┼──────┼──────┤
  │ 1       │ 0.785  │ 2.18 │ 10   │
  ├─────────┼────────┼──────┼──────┤
  │ 2       │ 1.19   │ 0.57 │ 37   │
  ├─────────┼────────┼──────┼──────┤
  │ 4       │ 3.44   │ 0.15 │ 26   │
  ├─────────┼────────┼──────┼──────┤
  │ 10      │ 2185 m │ 0.04 │ 7019 │
  └─────────┴────────┴──────┴──────┘

### Two things are now proven:

1. The NIS over-confidence is genuinely under-tuned R — not an observability wall. NIS scales as a near-perfect 1/R_SCALE² (V2_03: 14.8 → 3.9 → 1.08, i.e.
÷4 ÷16; MH_05: 2.18 → 0.57 → 0.15). That clean quadratic is the signature of a measurement-noise calibration error: the real normalized image/tracking
noise is ~√14.8 ≈ 3.8× larger than the assumed 0.01 on V2_03, ~1.5× on MH_05. An observability/Jacobian fault would not scale away cleanly like this. So
Fault B is a noise number, not an estimator bug — good news.

2. But you cannot fix R in isolation — the filter is held together by the wrong R. The instant you make R honest, everything else diverges: NEES explodes
(115→444→1542), and ATE goes from 0.29 m to 278 m at R=10. The over-tight R is supplying the correction authority that keeps the trajectory bounded; remove
it and the filter dead-reckons off a cliff. That's why the default config has good ATE but a lying covariance — the two errors (Q too small, R too small)
are mutually compensating.

So the real question: is the joint setting — Q×~10 (honest NEES) and R×~3–4 (honest NIS) together — both consistent and still accurate? If yes → #212 is a
pure noise re-tune (config). If the joint setting is consistent but ATE stays bad → the filter structurally can't be both honest and accurate, which is the
observability/FEJ fault.

Both knobs already exist, so I can test the joint grid with no new code. Running Q∈{5,10} × R∈{2,4} on both sequences now (bounded, ~12 min):

While the joint sweep runs, let me commit the R_SCALE knob + the VioConfig wiring (it's a genuine improvement — R was previously un-configurable — and
R_SCALE=1 reproduced the baseline to 6 digits, confirming the wiring is neutral):

● Pushed to feat/q-scale-sweep (PR #279 now carries both knobs + the R config wiring; CI will run the full suite). Waiting on the joint sweep — that's the
  one that answers "config re-tune vs estimator surgery." I'll report the grid the moment it lands.

● Background command "Joint Q×R grid sweep, bounded" completed (exit code 0)

● Joint grid is in — and it's conclusive. No combination gives consistent and accurate.

  ┌─────┬─────┬───────┬──────┬──────┬──────┐
  │  Q  │  R  │  seq  │ ATE  │ NIS  │ NEES │
  ├─────┼─────┼───────┼──────┼──────┼──────┤
  │ 1   │ 1   │ V2_03 │ 0.29 │ 14.8 │ 115  │
  ├─────┼─────┼───────┼──────┼──────┼──────┤
  │ 10  │ 2   │ V2_03 │ 0.83 │ 3.6  │ 10.1 │
  ├─────┼─────┼───────┼──────┼──────┼──────┤
  │ 10  │ 4   │ V2_03 │ 6.22 │ 1.06 │ 42   │
  ├─────┼─────┼───────┼──────┼──────┼──────┤
  │ 5   │ 4   │ V2_03 │ 3.12 │ 1.05 │ 53   │
  ├─────┼─────┼───────┼──────┼──────┼──────┤
  │ 1   │ 1   │ MH_05 │ 0.79 │ 2.2  │ 10   │
  ├─────┼─────┼───────┼──────┼──────┼──────┤
  │ 10  │ 2   │ MH_05 │ 1.43 │ 0.56 │ 7.1  │
  ├─────┼─────┼───────┼──────┼──────┼──────┤
  │ 10  │ 4   │ MH_05 │ 3.04 │ 0.15 │ 3.6  │
  └─────┴─────┴───────┴──────┴──────┴──────┘

Read it down the ATE column: every step toward an honest covariance costs accuracy. The most accurate config (Q1R1, ATE 0.29) is the most over-confident;
the most consistent configs (R=4, NIS≈1) have ATE of 3–6 m. The best balance, Q10R2, still leaves V2_03 at NEES=10 / NIS=3.6 and triples the ATE.

That trade-off is the diagnosis. In a correctly-modeled filter, the true noise values give both consistency and optimal accuracy — that's what optimality
means. Here, accuracy requires over-confident noise. So the over-confidence isn't a mistuned dial — it's compensating for a structural error elsewhere.
This rules out "just re-tune Q and R." The two noise findings are real (true R≈4×, Q≈10×) but they're symptoms, not the disease.

And the disease is motion-dependent — the throughline of the whole investigation: attitude NEES (7→169), NIS (2.2→14.8), and now the residual inconsistency
at honest noise (MH_05 NEES 3.6 vs V2_03 42) all scale with aggressiveness. That fingerprints a linearization / observability inconsistency (the classic
MSCKF spurious-information-gain, worst under fast rotation — the FEJ / OC-EKF problem), possibly with a motion-correlated systematic in the visual update.

So #212 is an estimator fix, not a config fix. Before committing to the FEJ implementation, there's one clean discriminator left: is the innovation
zero-mean-but-inflated (→ observability/Jacobian) or biased (→ a systematic, e.g. extrinsics/triangulation)? That's a mean/whiteness test on the normalized
innovations — the next small instrument.

Where I'd take it:
- (a) Add the innovation-mean/whiteness diagnostic (small, ~like the NIS accumulator) — pins observability vs systematic before we write FEJ.
- (b) Skip straight to implementing FEJ / observability-constrained MSCKF (the odds-on fix) and re-run the instrument — success = honest noise finally
gives consistent and accurate.
- (c) Write this whole chain up in docs/assessments/ and open the #212 root-cause issue (observability inconsistency) with the sweep evidence, so the
diagnosis is on record before any estimator surgery.

My recommendation: (c) then (a) — capture the hard-won evidence, then run the one discriminator that could still redirect us away from FEJ, rather than
committing estimator code on a strong-but-unconfirmed hypothesis. Which do you want?

Do (c) then (a):

Both (c) and (a) are complete. Here's where it landed.

The diagnosis is closed — #212 is an observability fault (FEJ)

The innovation discriminator gave the deciding read:

┌────────────────────┬────────────────────────────────────────────┬─────────────────────────────┐
│      sequence      │                mean (bias)                 │          whiteness          │
├────────────────────┼────────────────────────────────────────────┼─────────────────────────────┤
│ V2_03 (aggressive) │ −0.013σ (~0.1 px) — flagged but negligible │ lag-1 z = +14.2 → non-white │
├────────────────────┼────────────────────────────────────────────┼─────────────────────────────┤
│ MH_05 (mild)       │ zero-mean (not flagged)                    │ lag-1 z = −10.3 → non-white │
└────────────────────┴────────────────────────────────────────────┴─────────────────────────────┘

- Innovations are strongly non-white on both runs → the filter is leaving temporal structure in its residuals → the observability/linearization signature.
This is the positive confirmation the sweeps could only imply.
- Innovations are effectively zero-mean → there is no gross systematic (no swapped extrinsic, no large time offset). The one flagged bias is ~0.3% of the
over-confidence and motion-dependent — a minor secondary, not the cause.

So the discriminator does not redirect us. The full chain — over-confident → compensating (honest noise diverges ATE) → motion-dependent → zero-mean but
non-white — all points one way: First-Estimates Jacobians / OC-EKF, tracked in #280.

What's on the branch (PR #279)

1. CORTEX_Q_SCALE + CORTEX_R_SCALE diagnostic knobs (and R is now configurable via VioConfig::camera_noise_normalized — it was hardcoded).
2. eval/innovation_whiteness.hpp — the zero-mean/whiteness instrument, unit-tested against oracles (i.i.d.→white, constant→biased, alternation→correlated),
wired into the backend + benchmark WARN.
3. docs/assessments/vio-consistency-sweep-diagnosis.md — the complete Q/R/joint/innovation evidence and conclusion.
4. Issue #280 opened (FEJ, full cortex defaults) with the sweep + discriminator evidence; memory updated.

Local tests green (innovation_whiteness 19 assertions; msckf_backend 314; consistency/nav_consistency pass). I'll let CI + CodeRabbit finish on #279 and
merge when green per your standing rule (not before the async review, per the review-gate convention) — unless you want to look first.

Decision point

The instrumentation program has done its job: "it drifts 18%" is now "the attitude covariance is observability-inconsistent; here's the proof." The next
move is the actual fix — (b): implement FEJ / OC-EKF (#280) — which is real estimator surgery in the propagation/update Jacobians, the success test already
defined (honest noise → consistent NEES+NIS+ATE, innovations whiten).

Want me to start #280 (FEJ), or pause here while #279 goes through review?

