# VIO EuRoC MAV benchmark trouble shooting

The vio_euroc benchmarks produces these results: 
```text
-------------------------------------------------------------------------------
V1_01_easy replay keeps ATE under threshold
-------------------------------------------------------------------------------
/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:359
...............................................................................

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:205: SKIPPED:
explicitly with message:
set CORTEX_EUROC_V101 to the V1_01_easy/mav0 directory to run this benchmark

-------------------------------------------------------------------------------
MH_05_difficult forced dynamic VI-init fires + stays bounded (#247)
-------------------------------------------------------------------------------
/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:374
...............................................................................

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:302: warning:
MH_05_difficult (dynamic): init=dynamic, frames=2273, ATE=19.8789 m (gate 1.5
m)

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:310: warning:
MH_05_difficult (dynamic): NIS over 27173 updates: normalized=1.20953 (band
[0.996002, 1.004]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:316: warning:
MH_05_difficult (dynamic): NEES over 2077 frames: normalized=130.444 (band
[0.984296, 1.0157]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:326: warning:
MH_05_difficult (dynamic): seed gyro_bias=(0.0217551, 0.0176564, 0.0773316)
rad/s, |bg|=0.0822508, grav_residual=2.33588e-14

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:330: warning:
MH_05_difficult (dynamic): dyn-init scale=0.0607451, seed_speed=0.120572 m/s,
sfm_keyframes=20, roll/pitch err vs accel=2.32865 deg

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:338: warning:
MH_05_difficult (dynamic): dyn-attempts=170, window_builds=11,
best_keyframes=20, best_metric_motion=0.0614133 m

-------------------------------------------------------------------------------
MH_05_difficult replay (moving start) bootstraps and stays bounded
-------------------------------------------------------------------------------
/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:363
...............................................................................

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:302: warning:
MH_05_difficult: init=static, frames=2273, ATE=0.785467 m (gate 1.5 m)

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:310: warning:
MH_05_difficult: NIS over 29225 updates: normalized=2.17971 (band [0.99614,
1.00386]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:316: warning:
MH_05_difficult: NEES over 2221 frames: normalized=10.0786 (band [0.984814,
1.01519]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:326: warning:
MH_05_difficult: seed gyro_bias=(-0.00194081, 0.0202039, 0.0778975) rad/s,
|bg|=0.0804984, grav_residual=0.0026452

-------------------------------------------------------------------------------
V2_03_difficult replay (moving start) converges
-------------------------------------------------------------------------------
/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:386
...............................................................................

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:302: warning:
V2_03_difficult: init=static, frames=1922, ATE=0.293417 m (gate 1.5 m)

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:310: warning:
V2_03_difficult: NIS over 21627 updates: normalized=14.8014 (band [0.995038,
1.00496]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:316: warning:
V2_03_difficult: NEES over 1882 frames: normalized=115.312 (band [0.983503,
1.0165]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:326: warning:
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
cd ~/dev/branes/clones/cortex
git checkout main && git pull                    # get the Allan instrument (#276)
cmake --preset sitl-release                       # re-glob to pick up the new test target
cmake --build --preset sitl-release --target allan_variance

CORTEX_EUROC_V101=/srv/samba/sw-21/EuRoC-MAV-dataset/machine_hall/MH_05_difficult/mav0 \
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
(p311) stillwater@sw-21:~/dev/branes/clones/cortex$ CORTEX_EUROC_V101=/srv/samba/sw-21/EuRoC-MAV-dataset/machine_hall/MH_05_difficult/mav0 ./build/sitl-release/tests/allan_variance "[dataset]"
Filters: [dataset]
Randomness seeded to: 3305577977

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
allan_variance is a Catch2 v3.15.0 host application.
Run with -? for options

-------------------------------------------------------------------------------
EuRoC IMU noise density vs the filter's configured Q
-------------------------------------------------------------------------------
/home/stillwater/dev/branes/clones/cortex/tests/sdk/allan_variance.cpp:88
...............................................................................

/home/stillwater/dev/branes/clones/cortex/tests/sdk/allan_variance.cpp:112: warning:
  EuRoC IMU @ 200 Hz: gyro N=0.0047129 rad/s/√Hz (filter Q 0.00016), accel N=0.
  0498006 m/s²/√Hz (filter Q 0.002) — large mismatch ⇒ retune Q before chasing
  #212

===============================================================================
All tests passed (5 assertions in 1 test case)
```

Issue #278 merged (176aa94) — per-block NEES is on main. Re-run to get the decisive read:

```bash
  cd ~/dev/branes/clones/cortex
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
(p311) stillwater@sw-21:~/dev/branes/clones/cortex$ scripts/euroc-moving-start.sh
== locating sequences under /srv/samba/sw-21/EuRoC-MAV-dataset ==
  MH_05 mav0: /srv/samba/sw-21/EuRoC-MAV-dataset/machine_hall/MH_05_difficult/mav0
  V2_03 mav0: /srv/samba/sw-21/EuRoC-MAV-dataset/vicon_room2/V2_03_difficult/mav0
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
/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:400
...............................................................................

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:308: warning:
  V2_03_difficult: init=static, frames=1922, ATE=0.293417 m (gate 1.5 m)

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:316: warning:
  V2_03_difficult: NIS over 21627 updates: normalized=14.8014 (band [0.995038,
  1.00496]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:322: warning:
  V2_03_difficult: NEES over 1882 frames: normalized=115.312 (band [0.983503,
  1.0165]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:334: warning:
  V2_03_difficult: per-block NEES (normalized, want ~1): attitude=168.608784
  position=3.508583 velocity=25.906302 gyro_bias=16.377613 accel_bias=30.205153

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:340: warning:
  V2_03_difficult: seed gyro_bias=(-0.0023178, 0.0243927, 0.0813882) rad/s, |bg
  |=0.0849966, grav_residual=0.00309173

-------------------------------------------------------------------------------
MH_05_difficult forced dynamic VI-init fires + stays bounded (#247)
-------------------------------------------------------------------------------
/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:388
...............................................................................

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:308: warning:
  MH_05_difficult (dynamic): init=dynamic, frames=2273, ATE=19.8789 m (gate 1.5
  m)

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:316: warning:
  MH_05_difficult (dynamic): NIS over 27173 updates: normalized=1.20953 (band
  [0.996002, 1.004]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:322: warning:
  MH_05_difficult (dynamic): NEES over 2077 frames: normalized=130.444 (band
  [0.984296, 1.0157]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:334: warning:
  MH_05_difficult (dynamic): per-block NEES (normalized, want ~1): attitude=
  133.781601 position=70.680571 velocity=80.902497 gyro_bias=71.285316
  accel_bias=38.288993

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:340: warning:
  MH_05_difficult (dynamic): seed gyro_bias=(0.0217551, 0.0176564, 0.0773316)
  rad/s, |bg|=0.0822508, grav_residual=2.33588e-14

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:344: warning:
  MH_05_difficult (dynamic): dyn-init scale=0.0607451, seed_speed=0.120572 m/s,
  sfm_keyframes=20, roll/pitch err vs accel=2.32865 deg

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:352: warning:
  MH_05_difficult (dynamic): dyn-attempts=170, window_builds=11,
  best_keyframes=20, best_metric_motion=0.0614133 m

-------------------------------------------------------------------------------
V1_01_easy replay keeps ATE under threshold
-------------------------------------------------------------------------------
/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:373
...............................................................................

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:206: SKIPPED:
explicitly with message:
  set CORTEX_EUROC_V101 to the V1_01_easy/mav0 directory to run this benchmark

-------------------------------------------------------------------------------
MH_05_difficult replay (moving start) bootstraps and stays bounded
-------------------------------------------------------------------------------
/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:377
...............................................................................

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:308: warning:
  MH_05_difficult: init=static, frames=2273, ATE=0.785467 m (gate 1.5 m)

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:316: warning:
  MH_05_difficult: NIS over 29225 updates: normalized=2.17971 (band [0.99614,
  1.00386]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:322: warning:
  MH_05_difficult: NEES over 2221 frames: normalized=10.0786 (band [0.984814,
  1.01519]) — OVER-confident

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:334: warning:
  MH_05_difficult: per-block NEES (normalized, want ~1): attitude=7.175059
  position=9.879716 velocity=9.632311 gyro_bias=2.749590 accel_bias=5.335078

/home/stillwater/dev/branes/clones/cortex/tests/sdk/vio_euroc.cpp:340: warning:
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
cd ~/dev/branes/clones/cortex
git fetch origin feat/q-scale-sweep
git checkout origin/feat/q-scale-sweep -- tests/sdk/vio_euroc.cpp
cmake --build --preset sitl-release --target vio_euroc
export CORTEX_EUROC_MH05=/srv/samba/sw-21/EuRoC-MAV-dataset/machine_hall/MH_05_difficult/mav0
export CORTEX_EUROC_V203=/srv/samba/sw-21/EuRoC-MAV-dataset/vicon_room2/V2_03_difficult/mav0
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

```bash
(p311) stillwater@sw-21:~/dev/branes/clones/cortex$ for k in 1 3 10 30; do
>    echo "===================== Q_SCALE=$k ====================="
>  CORTEX_Q_SCALE=$k ./build/sitl-release/tests/vio_euroc "[dataset]" 2>&1 \
>    | grep -E 'init=static.*ATE|: NIS over|: NEES over|per-block NEES'
> done
===================== Q_SCALE=1 =====================
  V2_03_difficult: init=static, frames=1922, ATE=0.293417 m (gate 1.5 m)
  V2_03_difficult: NIS over 21627 updates: normalized=14.8014 (band [0.995038,
  V2_03_difficult: NEES over 1882 frames: normalized=115.312 (band [0.983503,
  V2_03_difficult: per-block NEES (normalized, want ~1): attitude=168.608784
  MH_05_difficult (dynamic): NIS over 27173 updates: normalized=1.20953 (band
  MH_05_difficult (dynamic): NEES over 2077 frames: normalized=130.444 (band
  MH_05_difficult (dynamic): per-block NEES (normalized, want ~1): attitude=
  MH_05_difficult: init=static, frames=2273, ATE=0.785467 m (gate 1.5 m)
  MH_05_difficult: NIS over 29225 updates: normalized=2.17971 (band [0.99614,
  MH_05_difficult: NEES over 2221 frames: normalized=10.0786 (band [0.984814,
  MH_05_difficult: per-block NEES (normalized, want ~1): attitude=7.175059
===================== Q_SCALE=3 =====================
  MH_05_difficult: init=static, frames=2273, ATE=0.785467 m (gate 1.5 m)
  MH_05_difficult: NIS over 29225 updates: normalized=2.17971 (band [0.99614,
  MH_05_difficult: NEES over 2221 frames: normalized=10.0786 (band [0.984814,
  MH_05_difficult: per-block NEES (normalized, want ~1): attitude=7.175059
  MH_05_difficult (dynamic): NIS over 27173 updates: normalized=1.20953 (band
  MH_05_difficult (dynamic): NEES over 2077 frames: normalized=130.444 (band
  MH_05_difficult (dynamic): per-block NEES (normalized, want ~1): attitude=
  V2_03_difficult: init=static, frames=1922, ATE=0.293417 m (gate 1.5 m)
  V2_03_difficult: NIS over 21627 updates: normalized=14.8014 (band [0.995038,
  V2_03_difficult: NEES over 1882 frames: normalized=115.312 (band [0.983503,
  V2_03_difficult: per-block NEES (normalized, want ~1): attitude=168.608784
===================== Q_SCALE=10 =====================
  MH_05_difficult: init=static, frames=2273, ATE=0.785467 m (gate 1.5 m)
  MH_05_difficult: NIS over 29225 updates: normalized=2.17971 (band [0.99614,
  MH_05_difficult: NEES over 2221 frames: normalized=10.0786 (band [0.984814,
  MH_05_difficult: per-block NEES (normalized, want ~1): attitude=7.175059
  MH_05_difficult (dynamic): NIS over 27173 updates: normalized=1.20953 (band
  MH_05_difficult (dynamic): NEES over 2077 frames: normalized=130.444 (band
  MH_05_difficult (dynamic): per-block NEES (normalized, want ~1): attitude=
  V2_03_difficult: init=static, frames=1922, ATE=0.293417 m (gate 1.5 m)
  V2_03_difficult: NIS over 21627 updates: normalized=14.8014 (band [0.995038,
  V2_03_difficult: NEES over 1882 frames: normalized=115.312 (band [0.983503,
  V2_03_difficult: per-block NEES (normalized, want ~1): attitude=168.608784
===================== Q_SCALE=30 =====================
  MH_05_difficult (dynamic): NIS over 27173 updates: normalized=1.20953 (band
  MH_05_difficult (dynamic): NEES over 2077 frames: normalized=130.444 (band
  MH_05_difficult (dynamic): per-block NEES (normalized, want ~1): attitude=
  MH_05_difficult: init=static, frames=2273, ATE=0.785467 m (gate 1.5 m)
  MH_05_difficult: NIS over 29225 updates: normalized=2.17971 (band [0.99614,
  MH_05_difficult: NEES over 2221 frames: normalized=10.0786 (band [0.984814,
  MH_05_difficult: per-block NEES (normalized, want ~1): attitude=7.175059
  V2_03_difficult: init=static, frames=1922, ATE=0.293417 m (gate 1.5 m)
  V2_03_difficult: NIS over 21627 updates: normalized=14.8014 (band [0.995038,
  V2_03_difficult: NEES over 1882 frames: normalized=115.312 (band [0.983503,
  V2_03_difficult: per-block NEES (normalized, want ~1): attitude=168.608784
```
