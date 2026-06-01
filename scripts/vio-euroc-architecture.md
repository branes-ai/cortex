# `vio_euroc` — EuRoC replay benchmark: goal & architecture

A map of what the `tests/sdk/vio_euroc.cpp` benchmark does, the transformation
pipeline it drives, and how to run it. This is the end-to-end real-data
validation of the Phase-3 VIO stack and the dynamic visual-inertial
initialization epic (**#211**).

---

## 1. Goal

Run the MSCKF visual-inertial estimator over a real [EuRoC MAV][euroc] sequence
(IMU + monocular camera) and measure how close the estimated trajectory lands to
the dataset's ground truth (**ATE** — absolute trajectory error). It exists to:

1. **Catch pipeline breakage** end-to-end on real data — wrong frames, wrong
   calibration, a diverging filter — that synthetic unit tests miss.
2. **Validate the moving-start dynamic VI-init** (#211): can the filter
   bootstrap attitude + yaw + velocity + metric scale from motion alone, with no
   stationary startup?

The cases are **dataset-gated** (the ~1.5 GB sequences are not vendored), so they
`SKIP` in CI and only run locally when the env vars below point at a `mav0`
directory. The synthetic trajectory-metric and CSV-parser tests in the same file
*do* run in CI.

---

## 2. Data flow

```
   EuRoC mav0/                      sdk/euroc/asl_replay.hpp
   ┌───────────────────┐           ┌──────────────────────────────────────┐
   │ imu0/data.csv     │──parse──▶ │ replay(dataset_root, est, cfg):       │
   │ cam0/data.csv     │──parse──▶ │   for each image frame:               │
   │ cam0/data/*.png   │──load───▶ │     feed_imu(samples ≤ frame.t)       │
   │ state_ground-     │           │     feed_image(t, grayscale PNG)      │
   │  truth_estimate0/ │──parse──▶ │     traj.push_back(current_pose())    │
   └───────────────────┘  (T_world_body GT)                                │
                                   └───────────────┬──────────────────────┘
                                                   │ estimated trajectory
                                                   ▼
                          sdk/eval/trajectory_metrics.hpp
                          associate(est, gt, 0.01 s)  →  ATE / RPE
                          (Horn rigid SE3 alignment, no scale)
```

- **`replay()`** (`asl_replay.hpp`) is the driver. For each camera frame it feeds
  every IMU sample up to that timestamp, then the image, then records the
  estimator's current pose. The returned trajectory is a list of
  `StampedPose{ t, T_world_imu }`.
- IMU is ~200 Hz, camera ~20 Hz; the IMU/camera streams are asynchronous and the
  backend propagates to the exact image time (zero-order hold) before cloning.

---

## 3. Coordinate frames & the transformation chain

Three frames, and the calibration that relates them:

| Frame | Symbol | Notes |
|---|---|---|
| **World** | `W` | Gravity along **−z** (down). Initialization aligns the estimate to this. Yaw gauge is free (set by init). |
| **Body / IMU** | `B` (a.k.a. `I`) | The estimator's state is the body pose `T_world_imu`. IMU measurements live here. |
| **Camera** | `C` | cam0. Pixels here; unprojected to bearings via the intrinsics. |

**Camera ↔ IMU (extrinsics, `T_BS`).** `CameraExtrinsics{ R_imu_cam, p_imu_cam }`
— `R_imu_cam` rotates camera axes into the IMU frame, `p_imu_cam` is the camera
origin expressed in the IMU frame. A point transforms camera → world as:

```
p_world = T_world_imu · ( R_imu_cam · p_cam + p_imu_cam )
```

> ⚠️ **EuRoC's cam0 is rotated ~90° from the IMU.** Using identity extrinsics
> makes the visual measurement model grossly wrong and the filter diverges
> (MH_05 ATE was ~21 km with identity, ~0.79 m with the real `T_BS`). The
> benchmark sets the published cam0 intrinsics **and** `T_BS` (#245).

**Pixel ↔ bearing (intrinsics).** `PinholeRadtanCamera::unproject(pixel)` returns
a **+z-forward unit bearing**; the backend normalizes by `z` to an image-plane
point. A non-positive `z` (ray not in front of the camera) is rejected. This
keeps the estimator core intrinsics-agnostic — it only ever sees normalized
bearings, never pixels or distortion coefficients.

**World alignment at init.** Vision (SfM) reconstructs in an arbitrary "vision
world"; the initializer rotates the whole solution so gravity points along `−z`
(`align_to_world_up`), about a horizontal axis only, preserving the vision yaw
gauge.

---

## 4. The estimator pipeline (per `feed_image`)

```
 feed_image(t, gray)                         sdk/vio_estimator.hpp
   │
   ├─ track_frame:  FAST detect ⊕ pyramidal KLT track  ──▶ FrontendObservation{ id, cam, u, v }
   │                (re-detect when tracked < 150; suppress within 15 px)
   │
   └─ backend_.process_camera(t, observations)          sdk/msckf_backend.hpp
        ├─ propagate state to t (ZOH on last IMU), clone the pose
        ├─ if not initialized: buffer SfM InitFrame + IMU preintegration; try init (§5)
        ├─ for each ended/marginalized track: null-space MSCKF EKF update
        └─ marginalize oldest clone to keep window ≤ max_clones (11)
```

- **Front end** (`branes::cv`): FAST corners + pyramidal KLT, producing a stable
  feature id per track and pixel coordinates. It never sees the backend's
  representation choice (triangulate / anchor / null-space).
- **Backend** (`MsckfBackend<T>`): IMU propagation + a sliding window of cloned
  poses + the null-space (left-null-space of the feature Jacobian) camera update.
  `current_pose()` returns `T_world_imu`.

---

## 5. Initialization — the decision the benchmark stresses

Until the filter has a valid attitude/velocity, every camera/IMU sample feeds
init. `InitMethod` (surfaced in `InitDiagnostics`, WARN'd by the benchmark):

```
 process_imu / process_camera while uninitialized
   │
   ├─ STATIC  : a stationary window (≤ kInitSamples) passes the std/gravity gates
   │            → roll/pitch + gyro bias, zero velocity/yaw.   (skipped if prefer_dynamic_init)
   │
   ├─ DYNAMIC : ≥ kMinDynFrames buffered → build_init_window (two-view essential
   │            + PnP SfM) → try_dynamic (gravity + per-keyframe velocity + METRIC
   │            SCALE via VINS-Mono tangent refinement).
   │            └─ SCALE-OBSERVABILITY GATE (#247): require scale · max-keyframe-
   │               displacement ≥ min_dynamic_motion (5 cm), else DECLINE.
   │               On a near-hover window the [v,g,s] solve is rank-deficient and
   │               collapses scale → 0; the gate rejects that degenerate seed so
   │               the filter never diverges. The backend retries on later frames.
   │
   └─ GRAVITY_ALIGN (fallback): past the timeout (kInitMaxSamples = 400 ≈ 2 s;
                     kInitMaxSamplesDynamic = 4000 ≈ 20 s when prefer_dynamic_init),
                     roll/pitch from the mean specific force. IDENTITY only if the
                     mean force isn't even gravity-like (last resort).
```

`VioConfig::prefer_dynamic_init` suppresses the static path (for a known moving
start) and extends the fallback runway so dynamic init has time to fire once the
platform actually moves.

**Why the dynamic seed matters:** `apply_dynamic_init` seeds `state_.v =
velocities_world.back()` (metric). A wrong scale throws a huge initial velocity
into the filter → divergence. The observability gate is the guard against that.

---

## 6. Evaluation

- **`associate(est, gt, 0.01 s)`** — nearest-timestamp matching within a 10 ms
  window; unmatched estimates are dropped.
- **`ate_rmse`** — **rigid SE3** Horn alignment (closed-form absolute
  orientation, clean-room from Horn 1987) then translational RMSE. The alignment
  is **rigid, not similarity** — it does *not* absorb a global scale error, so a
  metrically-wrong estimate shows up in ATE (this is deliberate: scale is part of
  what VIO must get right).
- **`rpe_translation_rmse`** — relative pose error over a fixed stride.

---

## 7. Test cases, gates & current results

| Case | Env var | Init taken | Gate | Measured ATE | Asserts |
|---|---|---|---|---|---|
| `V1_01_easy` | `CORTEX_EUROC_V101` | (not run) | 0.50 m | — | `ate < gate` |
| `MH_05_difficult` (default) | `CORTEX_EUROC_MH05` | static | 1.5 m | **0.785 m** | `ate < gate` |
| `MH_05_difficult` forced-dynamic | `CORTEX_EUROC_MH05` | gravity_align¹ | 1.5 m | **0.794 m** | fires + finite (#247) |
| `V2_03_difficult` | `CORTEX_EUROC_V203` | static² | 1.5 m | **0.293 m** | finite (NaN guard, #244) |

¹ With `prefer_dynamic_init`, dynamic init *declines* the unobservable quiet
start (correct) but doesn't yet *fire* on a later window — the clean-room SfM
isn't producing usable vision poses on real moving frames. Tracked in **#247**;
once it fires + converges, tighten this case back to `expect_converged=true`.

² The #247 observability gate also fixed V2_03: dynamic used to win the init race
and diverge (~12 km); now it declines and static wins → 0.29 m. Likely closes
**#244**.

---

## 8. Running it

```bash
scripts/euroc-moving-start.sh        # locate sequences, build sitl-release, run [dataset] cases
```

The script extracts the EuRoC `.zip`, points `CORTEX_EUROC_MH05` / `_V203` at the
`mav0` dirs, builds the release `vio_euroc`, and runs the `[dataset]`-tagged
cases (which WARN the init method + ATE for each). Overrides:
`CORTEX_EUROC_ROOT` (default `/srv/samba/sw-21/EuRoC-MAV-dataset`),
`CORTEX_PRESET` (default `sitl-release`). Set `CORTEX_EUROC_V101` to also run the
easy static-start case.

---

## 9. Open issues & limitations (this MVP MSCKF)

- **#211** (epic) — dynamic VI-init: machinery implemented + safe on real data
  (no divergence), but real-data *convergence* not yet demonstrated; blocked on
  SfM robustness (#247).
- **#247** — dynamic init declines on real moving windows because the clean-room
  two-view/PnP SfM doesn't yet produce usable poses there (scale stays
  unobservable). The divergence guard is in; the SfM-quality fix is open.
- **#244** — V2_03 divergence: likely resolved by the #247 gate (now 0.29 m via
  static); to be confirmed/closed.
- No online intrinsics/extrinsics refinement, no loop closure — gates are
  generous relative to SOTA (OpenVINS ~0.05 m on V1_01) by design.

---

## 10. Key source files

| Path | Role |
|---|---|
| `tests/sdk/vio_euroc.cpp` | the benchmark + gates (this doc's subject) |
| `sdk/include/branes/sdk/euroc/asl_replay.hpp` | dataset parsing + replay driver |
| `sdk/include/branes/sdk/vio_estimator.hpp` | lifecycle + front end (FAST/KLT) |
| `sdk/include/branes/sdk/msckf_backend.hpp` | MSCKF backend + init decision (§5) |
| `sdk/include/branes/sdk/imu_init.hpp` | static / gravity / dynamic alignment + scale gate |
| `sdk/include/branes/sdk/sfm/{two_view,pnp,init_window}.hpp` | clean-room SfM for dynamic init |
| `sdk/include/branes/sdk/eval/trajectory_metrics.hpp` | associate + ATE/RPE (Horn) |
| `sdk/include/branes/sdk/msckf/camera_updater.hpp` | extrinsics + null-space update |

[euroc]: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
