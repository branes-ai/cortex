# The VIO pipeline demo

## What, why, how — and how accurate is "good enough"?

A practical guide to `vio_pipeline` — the end-to-end demo that runs the whole
Visual-Inertial Odometry pipeline as a stream, measures how well the filter
cleans up additive sensor noise, and visualizes it. It also answers the question
that actually matters for a robot program: **what position accuracy does my
platform need, and at what error does the mission start to fail?**

- Tool: `tools/src/vio_pipeline.cpp` · Source data: `sdk/eval/synthetic_world.hpp`
- Visualizers: `docs-site/scripts/gen-sensor-model-figures.mjs` (curves),
  `docs-site/scripts/gen-overlay.mjs` (scene video overlay)
- Background: `docs/arch/vio-pipeline-canonical.md` (the pipeline as stage contracts)

---

## What this is

VIO fuses a **camera** (rich geometry, slow, scale-ambiguous if monocular) and an
**IMU** (fast, senses gravity, but drifts when integrated alone) into a single
metric estimate of where the platform is. The premise of the whole filtering
approach is **additive noise**: the sensors are modeled as truth + zero-mean
noise, and the filter's job is to reject that noise and stay both *accurate* and
*honest about its own uncertainty*.

`vio_pipeline` runs that pipeline end to end from one of two sources:

| Source | What it is | Why use it |
|---|---|---|
| **synthetic** (default) | a controllable world with **exact ground truth**: a known trajectory + a cloud of 3-D landmarks, projected to feature observations, with analytically-consistent IMU | noise → error is clean and *attributable*; reproducible; no dataset needed |
| **euroc** | the real EuRoC MAV dataset (real images through the KLT front end) with the dataset's millimetre ground truth | real sensor characteristics, real corners on real texture |

Both drive the **same MSCKF backend**; the synthetic source trades photorealism
for exact ground truth so we can measure the noise→error relationship precisely.

---

## Why we do the experiment

We want to answer three questions by **measurement**, not opinion:

1. **Does the filter actually reject sensor noise?** Add a controllable amount of
   noise to the camera + IMU streams and watch the trajectory error.
2. **Is the filter honest about its uncertainty?** A controller trusts the
   filter's covariance; an over-confident filter (small reported covariance, large
   true error) makes confident wrong decisions. We measure consistency (NIS/NEES).
3. **Where is the operating envelope?** At what noise level does the filter stop
   tracking — the cliff beyond which the mission is unsafe.

---

## How to run it

### Quickstart — one command to mp4

```bash
cmake -B build -DBUILD_TARGET_KPU=OFF && cmake --build build -j$(nproc)   # once
scripts/vio_scene_video.sh --out /tmp/vio_run --robot ground --sweep
# → /tmp/vio_run/scene.mp4  (the overlaid scene video)
#   /tmp/vio_run/figures/   (the noise→robustness curves, with --sweep)
#   /tmp/vio_run/{run.jsonl,trajectory.csv,frames.jsonl}  (the streams)
```

`scripts/vio_scene_video.sh` runs the experiment, generates the overlay frames,
rasterizes them, and encodes the mp4 in one shot. It needs `node`, `ffmpeg`, and
an SVG rasterizer (`librsvg2-bin`/`rsvg-convert`, or `cairosvg`, or `inkscape`).
Options: `--source synthetic|euroc`, `--dataset <mav0>`, `--noise S`, `--robot`,
`--fps N`, `--sweep`, `--keep`, `--build` (`--help` for all).

### Step by step (what the script does)

```bash
# build (host)
cmake -B build -DBUILD_TARGET_KPU=OFF && cmake --build build -j$(nproc)

# 1) a single run at the filter's assumed noise → trajectory + estimate stream
./build/tools/vio_pipeline --out /tmp/vio_run

# 2) the noise→robustness sweep (the core experiment)
./build/tools/vio_pipeline --sweep --out /tmp/vio_run
node docs-site/scripts/gen-sensor-model-figures.mjs /tmp/vio_run docs/assessments/figures/pipeline

# 3) the scene video with metrics + live features overlaid
./build/tools/vio_pipeline --robot ground --video --out /tmp/vio_run
node docs-site/scripts/gen-overlay.mjs /tmp/vio_run        # → /tmp/vio_run/frames/*.svg

# rasterize the overlay SVGs to a video (needs an SVG rasterizer + ffmpeg)
for f in /tmp/vio_run/frames/*.svg; do rsvg-convert "$f" -o "${f%.svg}.png"; done   # or cairosvg
ffmpeg -framerate 20 -i /tmp/vio_run/frames/frame_%05d.png -pix_fmt yuv420p /tmp/vio_run/scene.mp4
```

Flags: 
- `--source synthetic|euroc`
- `--dataset <mav0>` (EuRoC)
- `--noise S`(noise multiplier, 1 = matches the filter's model)
- `--sweep`
- `--video`
- `--robot ground|drone|default` (motion aggressiveness).


### Reading the scene-video overlay
- **Coloured dots** — the 3-D landmarks projected into this frame, **coloured by
  depth** (near = warm, far = cool). They move because the *camera* moves; the
  static points project to new pixels each frame. **Near dots sweep faster than
  far ones — that apparent motion difference is parallax**, the geometric signal
  that makes depth and metric scale observable.
- **White rings** — the *noisy* observations the filter actually receives; the
  offset between a ring and its coloured dot is the additive camera noise.
- **HUD** — frame, time, feature count, **NIS** (consistency), position error.
- Note: the synthetic scene is a geometry *diagram* (dots, not detected corners,
  because there is no texture image). For real corners on a real image, use the
  **euroc** source — there the dots become FAST/KLT corners on the camera frame.

---

## The experiment and its results

The sweep injects increasing sensor noise (a multiplier on the filter's *assumed*
noise) and records the trajectory error (ATE, RMS position vs ground truth) and
the consistency (NIS, normalized innovation; **1 = the filter's uncertainty
matches reality**). Representative synthetic result (gentle "ground" motion):

| noise × | ATE (m) | NIS | interpretation |
|---:|---:|---:|---|
| 0 | 0.42 | 0.02 | sensors perfect → filter **over-conservative** (NIS ≪ 1) |
| 1 | 0.51 | **1.6** | **injected noise = the filter's model → NIS ≈ 1 (consistent)** |
| 2 | 1.1 | 6.2 | filter now **over-confident** (NIS ≫ 1) |
| 4 | 156 | 24 | **lost lock — divergence** |
| 8 | 76 | 88 | diverged |

**Conclusions:**

1. **The filter rejects noise up to ~2× its assumed level, then loses lock.**
   That cliff (here between 2× and 4×) is the operating envelope. Run your real
   noise characterization (Allan variance for the IMU; pixel-σ for the camera) and
   make sure your actual noise sits well inside it.
2. **NIS crosses 1 exactly where the injected noise matches the model.** Below it
   the filter is over-conservative (wastes information); above it, over-confident
   (dangerous — see below). This is the calibration target for the noise model.
3. **Consistency, not just accuracy, is the safety property.** An over-confident
   filter reports a small covariance while the true error is large — a controller
   that trusts it will act decisively on a wrong position. This is the #212
   over-confidence the stage-probe program is chasing, and it is arguably more
   dangerous than the raw ATE.

---

## Is an ATE of 0.4 m good or bad?

**It depends entirely on the platform and the task — there is no single answer.**
But here is the honest read of *this* number:

- It is on a **small synthetic loop (~1.6 m across, ~12 s)**, so 0.4 m is ~25% of
  the trajectory extent — **mediocre as a benchmark**. Good research VIO lands
  sub-0.3 m on EuRoC; this same backend gets ~0.29–0.79 m on real EuRoC sequences.
- The number reflects the backend's **known consistency problems** (the #212
  over-confidence, the accel-bias dead-reckoning, the missing parallax gate), not
  a fundamental limit. It *works*, but it is not yet field-grade.
- Crucially, **the more telling metrics than a single ATE are (a) the drift rate**
  (% of distance, or RPE) for long missions, and **(b) consistency** (is the
  reported covariance honest). A 0.4 m error the filter *knows about* (large,
  honest covariance) is far safer than a 0.1 m error it is over-confident about.

### What accuracy do different robot form factors need?

The right question is **error relative to the smallest clearance the platform must
pass through**, and whether VIO is **in the control loop** (error → collision) or
**fused/corrected** (GPS, maps, loop closure bound the drift). Rule of thumb: keep
position error below **~⅓ of (gap − vehicle size)** for a safety margin.

| Platform | Speed | VIO's role | "good" position accuracy | Mission at risk above | Dominant failure |
|---|---|---|---|---|---|
| **AR/VR headset** | head/hand | sole, in render loop | smooth + < ~2–5 cm; **jitter/drift-rate matter more than ATE** | visible jitter, or > ~5–10 cm | virtual objects "swim" → nausea, broken immersion |
| **Racing / gap-crossing drone** | 5–15 m/s | sole, in flight-control loop | **< 10 cm** | **> 20–30 cm** | clips a gap/wall → crash |
| **Indoor inspection drone** | 1–3 m/s | sole, in control loop | < 20–30 cm | > ~0.5 m | hits walls in confined spaces |
| **Warehouse AMR** | 1–2 m/s | global nav (+ fiducials for docking) | < 0.2–0.3 m mid-aisle | > ~0.5 m in narrow aisles | aisle collision; docking handled by local vision |
| **Outdoor ground robot / car** | 5–30 m/s | fallback (GPS/lidar/HD-map primary) | < ~1% of distance (≈ 1 m / 100 m) | > ~0.5 m sustained lateral | wrong lane — but GPS/map correct it |
| **Outdoor UAV (GPS-fused)** | 5–20 m/s | bridge GPS dropouts | metres over open field; < 0.3–0.5 m for precision landing | depends on dropout length | drift during a long GPS gap; landing uses a fiducial |
| **Legged / humanoid** | walk/run | global pose (+ proprioception for footholds) | < 0.1–0.3 m with periodic relocalization | > ~0.3 m without relocalization | mis-step / wrong foothold target |

So, **is 0.4 m good?**

- For a **confined-space drone** (VIO in the loop): **bad** — you need < 0.1–0.2 m;
  0.4 m risks collisions.
- For a **warehouse AMR mid-aisle**: **borderline** — fine in wide aisles, risky in
  narrow ones; final docking is rescued by fiducials.
- For an **AR headset**: the absolute 0.4 m might be tolerable *if smooth*, but the
  jitter/consistency is what makes or breaks it.
- For a **GPS-fused outdoor platform**: **fine** as a short-term bridge.

### When is a mission in danger?

Three independent failure conditions, any of which ends a mission:

1. **Absolute error exceeds the clearance margin** — error > ~⅓·(gap − vehicle
   size) for an in-the-loop platform ⇒ collision risk. (A 0.4 m drone in a 0.8 m
   gap is already past this.)
2. **Drift outpaces correction** — for fused/long-range platforms, when the VIO
   drift rate exceeds what loop closure / GPS / maps can pull back between fixes.
3. **The filter loses consistency** — when reported covariance no longer reflects
   true error (the over-confidence on the right of the NIS curve). This is the
   silent killer: accuracy can look acceptable on average while the controller
   acts on a covariance it should not trust, and the failure appears suddenly
   under the conditions the filter was over-confident about.

**Bottom line:** specify your accuracy requirement from the *smallest gap your
platform must traverse* and *whether VIO is corrected by another sensor*, then
size the noise model (run the sweep) so your real sensor noise sits well inside
the operating envelope — and treat **consistency (NIS ≈ 1, NEES ≈ dof) as a
first-class acceptance gate, not just ATE.**

---

## Files & follow-ups

- Curves: `docs/assessments/figures/pipeline/` · Overlay frames: `<out>/frames/`
- Stream artifacts: `run.jsonl` (typed per-frame `gt`/`est`), `trajectory.csv`,
  `noise_sweep.csv`, `frames.jsonl` (overlay data)
- The #212 over-confidence on the right of the NIS curve is decomposed stage by
  stage in `docs/arch/vio-pipeline-canonical.md` (S10 calibration ≈ the `R×4`
  component; plus process noise and the S5 parallax gate).
