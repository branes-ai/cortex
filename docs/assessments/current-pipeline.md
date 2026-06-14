# Current Pipeline — Camera Calibration

Camera distortion is an **intrinsic**. Extrinsics are purely the rigid camera–IMU
transform. This note maps the calibration code in the repo and the tools to
interrogate it. The stage-by-stage projection-Jacobian walkthrough (with an
interactive 3D view) has moved to the [Camera Updaters](https://branes-ai.github.io/cortex/vio/camera-updaters/)
page — see §6.

## 1. The mental model: intrinsics vs. extrinsics vs. distortion

- **Intrinsics** — everything internal to one camera, mapping a 3D point in the camera frame to a pixel:
    - Focal length $f_x, f_y$ (pixels) and principal point $c_x, c_y$ (the optical-axis pixel).
    - **Distortion is an intrinsic.** This repo uses radial-tangential (Brown–Conrady): radial $k_1, k_2\,(,k_3)$ and tangential $p_1, p_2$.
    - Skew / sensor-plane non-perpendicularity is *also* an intrinsic (a skew term in the $K$ matrix). This pinhole model assumes zero skew — modern sensors are square-pixel and perpendicular, so it is omitted.
- **Extrinsics** — the rigid transform between two sensors (here camera ↔ IMU): a rotation $\mathbf{R}$ and a translation $\mathbf{t}$. No distortion, no focal length — just where the camera is bolted relative to the IMU. In EuRoC terms this is $T_{BS}$.
- **Time offset $t_d$** — a fourth calibration quantity (camera/IMU clock skew), modeled separately.

Summary: *focal length + principal point + distortion = intrinsics; rotation + translation between sensors = extrinsics.*

## 2. What's currently in this pipeline

**Intrinsics** — `math/include/branes/math/cameras/pinhole_radtan.hpp`, class `PinholeRadtanCamera<T>`:

```cpp
PinholeRadtanCamera(T fx, T fy, T cx, T cy, T k1, T k2, T p1, T p2, T k3 = T{0})
```

with `project()` (3D → pixel), `unproject()` (pixel → bearing), `distort()`/`undistort()`, and `project_jacobian()`. The real EuRoC cam0 values are baked in at `tools/src/s0_sensor_model.cpp` (`euroc_cam0()`) and `tests/sdk/vio_euroc.cpp`:

```text
fx = 458.654   fy = 457.296   cx = 367.215   cy = 248.375
k1 = -0.283    k2 = 0.0740    p1 = 0.000194  p2 = 1.76e-5
```

**Extrinsics** — `sdk/include/branes/sdk/msckf/camera_updater.hpp`, struct `CameraExtrinsics<T>`:

```cpp
SO3  R_imu_cam;   // rotates camera axes into the IMU frame
Vec3 p_imu_cam;   // camera origin expressed in the IMU frame
```

The real EuRoC cam0 ↔ IMU transform (`tests/sdk/vio_euroc.cpp`) is a **~90° rotation** plus $\mathbf{p}_{\text{imu,cam}} = [-0.0216,\ -0.0647,\ 0.0098]\ \text{m}$. This matters: using identity extrinsics on real data diverges — the camera really is mounted sideways.

The full projection chain the filter uses (in `camera_updater.hpp::to_camera`/`project`): world point $\mathbf{p}_f$ → IMU frame via the clone pose $(\mathbf{R}_{\text{clone}}, \mathbf{p}_{\text{clone}})$ → camera frame via the extrinsic $(\mathbf{R}_{\text{imu,cam}}, \mathbf{p}_{\text{imu,cam}})$ → normalized → distort → pixel via intrinsics. The Jacobians of each stage are stacked into the MSCKF measurement matrix.

## 3. Online (in-state) calibration — "S10", issue #332

The pipeline can either **trust** the configured extrinsics, or **estimate them as filter state**. The state (`sdk/include/branes/sdk/msckf/state.hpp`) has an optional `CalibState{ R_imu_cam, p_imu_cam }` and `enable_calibration(init, rot_sigma, trans_sigma)`, which adds a 6-DoF block $(\delta\boldsymbol\theta_{ic}, \delta\mathbf{p}_{ic})$ right after the IMU state with a prior. The measurement Jacobians for it ($\mathbf{H}_{\theta,ic} = [\mathbf{p}_c]_\times$, $\mathbf{H}_{p,ic} = -\mathbf{R}_{ic}^\top$) are in `camera_updater.hpp::project()`. This is toggled via `VioConfig` (`sdk/include/branes/sdk/vio_backend.hpp`):

```cpp
bool   estimate_extrinsics       = false;   // estimate T_CI as state (#332)
double calib_ext_rot_prior_deg   = 1.0;     // prior sigma when estimating
double calib_ext_trans_prior_mm  = 10.0;
double calib_ext_rot_sigma_deg   = 0.0;     // alternative: just inflate R by a rotation sigma
```

## 4. How to interrogate it — the tools

Two purpose-built stage probes live under `tools/` (one executable per `tools/src/*.cpp`). Build and run:

```bash
cmake --build build -j$(nproc) --target s0_sensor_model s10_online_calibration

./build/tools/s0_sensor_model --help       # prints the S0 contract (pre/post-conditions)
./build/tools/s0_sensor_model --no-out      # run all probes, print numbers, write no CSVs
./build/tools/s10_online_calibration --no-out
```

**`s0_sensor_model`** — the sensor & calibration model probe. It exercises `PinholeRadtanCamera` + the IMU model directly and prints native-unit assessments:

```text
radtan round-trip residual (max)      1.28e-13 px   PASS   <- unproject o project ~ identity
radtan Jacobian rel-error (max)       1.18e-09  -    PASS   <- analytic project_jacobian correct
time-offset sensitivity [slow]        0.1185 px/ms         <- how t_d error shows up in pixels
extrinsic rotation sensitivity        ...    px/deg        <- how a wrong R_imu_cam shows up
extrinsic translation sensitivity     ...    px/mm @2m, @10m
```

It writes CSVs (`roundtrip_radtan.csv`, `jacobian_radtan.csv`, `extrinsic.csv`, …) that the docs-site figure generator renders.

**`s10_online_calibration`** — the calibration-uncertainty probe (the #212 lever). It quantifies how much an imperfect extrinsic *should* inflate the measurement noise, and sweeps end-to-end NEES:

```text
R-inflation @ 1deg ext, slow          4.1  x   induced 8.1px vs assumed 4.6px
pose NEES with perfect calibration    ~43      backend over-confident even with exact T_CI
```

## 5. The eval headers and tests behind them

The underlying functions, to call from your own code or a notebook-style harness:

- `sdk/include/branes/sdk/eval/sensor_model_probe.hpp` — `camera_round_trip()`, Jacobian-consistency, time-offset & extrinsic sensitivity.
- `sdk/include/branes/sdk/eval/calibration_probe.hpp` — `noise_budget(...)` (the px → R-inflation calculator) and `r_inflation_sweep()`.
- Tests that validate the calibration math: `tests/sdk/calib_jacobians.cpp` (finite-difference check of the extrinsic Jacobians) and `tests/sdk/calib_recovery.cpp` (online calibration drives a deliberately-wrong extrinsic back to truth).

The whole S0 → S10 contract is written up in `docs/arch/vio-pipeline-canonical.md`.

**Quick start:** run `./build/tools/s0_sensor_model --no-out` to see the camera/extrinsic behavior, and `./build/tools/s10_online_calibration --no-out` for the calibration-uncertainty budget. To change the camera being interrogated, edit `euroc_cam0()` / the extrinsic in `tools/src/s0_sensor_model.cpp`.

One caveat: there is **no YAML/daemon config wired up yet** — calibration values are hardcoded in the tools/tests or environment-gated (`CORTEX_ESTIMATE_EXTRINSICS`, `CORTEX_CALIB_ROT_PRIOR_DEG`). The unified config path is a separate epic (#69).

| Tool | Location | Purpose |
|---|---|---|
| Sensor-model probe (S0) | `tools/src/s0_sensor_model.cpp` | Round-trip, Jacobian, time-offset, extrinsic sensitivity, IMU drift |
| Calibration probe (S10) | `tools/src/s10_online_calibration.cpp` + `eval/calibration_probe.hpp` | Quantify sensitivity to calibration error; R-inflation / NEES sweep |
| Pinhole-Radtan | `math/cameras/pinhole_radtan.hpp` | The actual projection math |
| Jacobian test | `tests/sdk/calib_jacobians.cpp` | Finite-difference check of the online-calib Jacobians |
| Recovery test | `tests/sdk/calib_recovery.cpp` | Verify auto-calibration convergence |

---

## 6. The projection-Jacobian chain — now on the docs-site

The stage-by-stage walkthrough of `camera_updater.hpp` lives on the published **Camera Updaters** page, which adds an **interactive 3D triangulation view** you can orbit and zoom:

- **Live page:** [Camera Updaters](https://branes-ai.github.io/cortex/vio/camera-updaters/)
- **Source:** `docs-site/src/content/docs/vio/camera-updaters.mdx`

It covers the symbol definitions (the live nav state `R, p` vs the frozen per-image clones `R_i, p_i` vs the fixed extrinsic `R_ic, p_ic`), the worked two-view example, the projection derivative `dh` and the per-block Jacobians, the left-null-space feature marginalization, and the `δp = −H_f` translation-gauge cancellation behind the #212 over-confidence.
