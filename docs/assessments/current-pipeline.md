# Current Pipeline — Camera Calibration & the Projection-Jacobian Chain

Camera distortion is an **intrinsic**. Extrinsics are purely the rigid camera–IMU
transform. This note maps the calibration code in the repo, the tools to interrogate
it, and then walks the projection-Jacobian chain in `camera_updater.hpp` end to end.

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

## 6. Walk-through of the projection-Jacobian chain in `camera_updater.hpp`

Following a single feature observation from a 3D point in world space to a row of the Kalman gain.

### Notation: the poses, the extrinsic, and the frames

The key point is that $\mathbf{R}_i, \mathbf{p}_i$ are **not** iterations of $\mathbf{R}, \mathbf{p}$ — they are frozen *snapshots* of it. There is one moving body pose, a window of past snapshots of it, and a fixed camera↔IMU mount.

| symbol | meaning | constant? | code | defined at |
|---|---|---|---|---|
| $\mathbf{R}, \mathbf{p}$ | the **live** nav state — current IMU pose, "now" ($\mathbf{R}$ = world←imu, $\mathbf{p}$ = world position). Moves every IMU sample and every update. | no | `State::R`, `State::p` | `state.hpp` (`SO3 R; Vec3 p;`) |
| $\mathbf{R}_i, \mathbf{p}_i$ | **clone $i$** — the body pose *frozen at the image time of clone $i$*. `augment_clone` copies the live $(\mathbf{R}, \mathbf{p})$ into the sliding window; $i$ indexes which past frame. A feature is seen from several clones $i = 0\ldots m-1$. | yes, after the snapshot | `s.clones[i].R`, `.p` | `state.hpp` `struct Clone`; set in `state_helper.hpp` `augment_clone` (`clones.push_back({s.R, s.p, …})`) |
| $\mathbf{R}_{ic}, \mathbf{p}_{ic}$ | the **extrinsic** — the fixed rigid camera↔IMU mount, *not* a body pose. `ic` reads "imu ← cam": $\mathbf{R}_{ic}$ rotates camera axes into the IMU frame, $\mathbf{p}_{ic}$ is the camera origin in the IMU frame. Same for every clone of that camera (the ~90° EuRoC mount). | yes — unless S10 online calibration estimates it (then it lives in `State::calib[j]`) | `CameraExtrinsics::R_imu_cam`, `p_imu_cam` (via `extrinsic_of`) | `camera_updater.hpp` `struct CameraExtrinsics` |
| $\mathbf{p}_f$ | a **fixed 3D point in the world** — the landmark this observation is *of* (e.g. a corner on a wall), say $\mathbf{p}_f = (1.0,\,0.0,\,4.0)\ \text{m}$. "Triangulated" only means the filter *computes* this point by intersecting the viewing rays from several clones — it is not handed to us. ("Marginalized" — eliminated — is explained in Stage 5.) | yes (a world point) | `p_f` in `update()` | `camera_updater.hpp::triangulate` |
| $\mathbf{y}$ | the same point, re-expressed in the clone's **IMU** frame (what the IMU "sees" relative to itself) | — | `y` in `to_camera` | `camera_updater.hpp::to_camera` |
| $\mathbf{p}_c$ | the same point, re-expressed in the **camera** frame (axis $z$ = optical axis); dividing by $z$ gives the image coordinate | — | `p_c` in `to_camera` | `camera_updater.hpp::to_camera` |

So the two transforms compose to carry a world feature into one camera:

$$
\mathbf{p}_f\ (\text{world})
\;\xrightarrow[\;(\mathbf{R}_i,\,\mathbf{p}_i)\;]{\text{inverse of clone pose}}\;
\mathbf{y}\ (\text{IMU})
\;\xrightarrow[\;(\mathbf{R}_{ic},\,\mathbf{p}_{ic})\;]{\text{inverse of extrinsic}}\;
\mathbf{p}_c\ (\text{camera})
$$

### A concrete example you can picture

Put a feature on a wall **4 m ahead and 1 m to the right** of where the robot started, and watch it through **two** camera frames as the robot slides 20 cm to the right. Use world axes **X = right, Y = down, Z = forward (into the scene)**, and — to keep the arithmetic clean — assume no rotation and a camera bolted 5 cm to the right of the IMU. Because everything sits at the same height ($Y = 0$), the whole picture lives in the X–Z plane, so a top-down view shows all of it:

```text
   Z (forward) ↑
              4 ┤              ☆  p_f = (1.0, 0.0, 4.0)   ← the wall corner
                │             ╱ ╲
                │            ╱   ╲       the two rays point at the SAME world point;
                │           ╱     ╲      where they cross IS the triangulated p_f.
                │          ╱       ╲
                │         ╱         ╲
                │        ╱           ╲
                │       ╱             ╲
                │      ╱               ╲
              0 ┤   ▢ ◉                 ◉ ▢
                └───┬──────────────────┬───→ X (right)
                  clone 0            clone 1
                  p_0 = (0,0,0)      p_1 = (0.20, 0, 0)
                    └── 0.20 m camera baseline ──┘

   top-down view, looking straight down (Y = height is into the page, 0 here).
   ▢ = IMU body        ◉ = camera = IMU shifted by the extrinsic p_ic (+5 cm in X)
```

Plug the example into the two frame hops (all values in metres; rotations are identity so the transposes vanish):

| quantity | clone 0 | clone 1 | what it is |
|---|---|---|---|
| $\mathbf{p}_f$ | $(1.0,\,0,\,4.0)$ | $(1.0,\,0,\,4.0)$ | the **same** world point |
| $\mathbf{R}_i,\ \mathbf{p}_i$ | $\mathbf{I},\ (0,0,0)$ | $\mathbf{I},\ (0.20,0,0)$ | the clone (frozen body pose) |
| $\mathbf{R}_{ic},\ \mathbf{p}_{ic}$ | $\mathbf{I},\ (0.05,0,0)$ | $\mathbf{I},\ (0.05,0,0)$ | the **same** camera mount |
| $\mathbf{y} = \mathbf{R}_i^\top(\mathbf{p}_f-\mathbf{p}_i)$ | $(1.00,\,0,\,4.0)$ | $(0.80,\,0,\,4.0)$ | feature in the IMU frame |
| $\mathbf{p}_c = \mathbf{R}_{ic}^\top(\mathbf{y}-\mathbf{p}_{ic})$ | $(0.95,\,0,\,4.0)$ | $(0.75,\,0,\,4.0)$ | feature in the camera frame |
| image $(x/z,\ y/z)$ | $(0.2375,\,0)$ | $(0.1875,\,0)$ | the normalized observation |

The robot moved +20 cm and the feature slid from $x = 0.2375$ to $0.1875$ in the image — it drifted **left**. That shift is **parallax**, and its size (0.05 over a 0.20 m baseline) is exactly what encodes the **4 m depth**. Running it backwards — *"what single world point projects to both of these?"* — is **triangulation**, and the answer is $\mathbf{p}_f = (1.0,\,0,\,4.0)$. The filter never measured $\mathbf{p}_f$ directly; it inferred it from the two views.

### The measurement model

The updater works in **normalized image coordinates**, not pixels. The front end has already un-distorted and un-projected each detection into a bearing, so `PinholeRadtanCamera`'s focal length and radtan distortion are **not** in this Jacobian chain — they live upstream. What `camera_updater` linearizes is the *geometric* model:

$$
h(\mathbf{x}) = \pi(\mathbf{p}_c),
\qquad
\pi(\mathbf{p}_c) = \left(\frac{p_{c,x}}{p_{c,z}},\ \frac{p_{c,y}}{p_{c,z}}\right)
$$

where $\mathbf{p}_c$ is the feature in the camera frame. The state $\mathbf{x}$ it depends on: the clone pose $(\mathbf{R}_i, \mathbf{p}_i)$, the feature $\mathbf{p}_f$, and (if S10 is on) the extrinsic $(\mathbf{R}_{ic}, \mathbf{p}_{ic})$.

### Stage 1 — the geometric chain (`to_camera`, lines 276–285)

Three frame hops take the world feature into the camera:

```cpp
// world feature p_f  ->  IMU frame
y   = Ri.transpose()  * (p_f - p_i);

// IMU frame  ->  camera frame
p_c = Ric.transpose() * (y - p_ic);

// cheirality: feature must be in front of the camera
return p_c[2] > 0;
```

$\mathbf{y}$ (the feature in the IMU frame) is kept around because the clone-rotation Jacobian needs it.

### Stage 2 — the projection derivative (`project`, lines 297–301)

This is $\partial\pi / \partial\mathbf{p}_c$, the $2\times 3$ derivative of the normalized projection, evaluated at the camera-frame point (call it $\mathrm{dh}$):

$$
\frac{\partial\pi}{\partial\mathbf{p}_c} = \frac{1}{z}
\begin{bmatrix} 1 & 0 & -x/z \\ 0 & 1 & -y/z \end{bmatrix},
\qquad z = p_{c,z},\ x = p_{c,x},\ y = p_{c,y}
$$

Everything below is $\mathrm{dh}$ times one geometric block.

### Stage 3 — chain-rule each state block (`project`, lines 303–321)

With $\mathbf{R}_{ct} = \mathbf{R}_{ic}^\top$, $\mathbf{R}_{it} = \mathbf{R}_i^\top$, and $\mathbf{M} = \mathbf{R}_{ct}\mathbf{R}_{it} = \partial\mathbf{p}_c / \partial\mathbf{p}_f$:

| state | $\partial\mathbf{p}_c / \partial\,(\cdot)$ | Jacobian $\big(\mathrm{dh}\cdot(\cdot)\big)$ | code |
|---|---|---|---|
| feature $\delta\mathbf{p}_f$ | $\mathbf{M} = \mathbf{R}_{ic}^\top \mathbf{R}_i^\top$ | $\mathbf{H}_f = \mathrm{dh}\,\mathbf{M}$ | `J.Hf = dh * M` |
| clone rotation $\delta\boldsymbol\theta$ | $\mathbf{R}_{ic}^\top [\mathbf{y}]_\times$ | $\mathbf{H}_\theta = \mathrm{dh}\,\mathbf{R}_{ic}^\top [\mathbf{y}]_\times$ | `J.Htheta = dh * dpc_dtheta` |
| clone position $\delta\mathbf{p}$ | $-\mathbf{M}$ | $-\mathbf{H}_f$ | assembled as `-J.Hf` |
| extrinsic rotation $\delta\boldsymbol\theta_{ic}$ | $[\mathbf{p}_c]_\times$ | $\mathbf{H}_{\theta,ic} = \mathrm{dh}\,[\mathbf{p}_c]_\times$ | `J.Hext_theta = dh * dpc_dext_theta` |
| extrinsic translation $\delta\mathbf{p}_{ic}$ | $-\mathbf{R}_{ic}^\top$ | $-\mathrm{dh}\,\mathbf{R}_{ic}^\top$ | `J.Hext_p = -dh_Rct` |

The two non-obvious signs both come from the **right-perturbation** convention $\mathbf{R} \leftarrow \mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol\theta)$:

- **Clone rotation:** $\mathbf{R}_i \leftarrow \mathbf{R}_i\,\mathrm{Exp}(\delta\boldsymbol\theta) \Rightarrow \mathbf{R}_i^\top \leftarrow \mathrm{Exp}(-\delta\boldsymbol\theta)\mathbf{R}_i^\top$, so $\mathbf{y} \leftarrow \mathbf{y} + [\mathbf{y}]_\times \delta\boldsymbol\theta \Rightarrow \partial\mathbf{y}/\partial\delta\boldsymbol\theta = [\mathbf{y}]_\times$, then left-multiply by $\mathbf{R}_{ic}^\top$.
- **Clone position** is additive ($\mathbf{p}_i \leftarrow \mathbf{p}_i + \delta\mathbf{p}$), so $\partial\mathbf{p}_c/\partial\delta\mathbf{p} = -\mathbf{R}_{ic}^\top \mathbf{R}_i^\top = -\mathbf{M}$. That is why the $\delta\mathbf{p}$ block is literally $-\mathbf{H}_f$ — the same matrix as the feature block, negated. (Hold that thought — it is load-bearing for observability.)
- **Extrinsic rotation:** $\mathbf{R}_{ic} \leftarrow \mathbf{R}_{ic}\,\mathrm{Exp}(\delta\boldsymbol\theta_{ic}) \Rightarrow \mathbf{p}_c \leftarrow \mathbf{p}_c + [\mathbf{p}_c]_\times \delta\boldsymbol\theta_{ic} \Rightarrow \partial\mathbf{p}_c/\partial\delta\boldsymbol\theta_{ic} = [\mathbf{p}_c]_\times$.

### Stage 4 — stack it into the big matrices (`update`, lines 171–199)

For each of the $m$ observations of this feature, `project` fills one 2-row strip:

```cpp
r[row+a]               = obs.xy[a] - J.h[a];     // residual: measurement - prediction (normalized)
Hf[...]                =  J.Hf;                   // 2x3 feature block
Hx[..., off + b]       =  J.Htheta;               // clone d-theta columns  (off = clone_offset)
Hx[..., off + 3 + b]   = -J.Hf;                   // clone d-p columns = -Hf
```

`Hx` is $2m \times n$ ($n$ = full state dim) and is **zero everywhere except this clone's 6 columns** — that sparsity is the whole point of the sliding window. If S10 calibration is on, the same strip *also* writes `Hext_theta`/`Hext_p` into the camera's shared `calib_offset` block (lines 191–198) — and because *every* observation through that camera writes the *same* extrinsic columns, the coupling across the window is what makes $T_{CI}$ observable.

### Stage 5 — marginalize the feature, then EKF-update (lines 204–236)

The feature $\mathbf{p}_f$ was *computed from* these same clones (we triangulated it from them), so treating it as an independent measurement would be circular — it would feed the clones' own information back into them and make the filter over-confident. **Marginalizing** means algebraically **eliminating** $\mathbf{p}_f$ — like solving a system of equations by substitution so the variable you do not want to carry drops out. In MSCKF that elimination is a projection:

```cpp
proj = msckf_left_nullspace_project(Hf, Hx, r, ...);   // project onto left-null(Hf)
```

`Hf` is the $2m\times 3$ block of how the stacked residual depends on the (3-DoF) feature. Multiplying the whole system $[\,\mathbf{H}_f \mid \mathbf{H}_x \mid \mathbf{r}\,]$ by an orthonormal basis of $\mathbf{H}_f$'s **left null space** zeroes the `Hf` columns — i.e. removes every direction that the feature could have explained — and drops 3 rows. What survives constrains **only the clones/extrinsics**, with the noise still $\sigma^2 \mathbf{I}$ (orthonormal reflectors preserve it). Then:

```cpp
r2     = normalized_sigma^2 + calib_rot_sigma^2;        // R (+ S10 noise term)
mahalanobis(H, proj.r, r2, gamma);                      // NIS gamma = r^T S^-1 r
if (gamma > chi2_per_dof * rows) return false;          // chi-square outlier gate
StateHelper::ekf_update(s, H, proj.r, R_diag);          // K = P H^T S^-1, x (+) Kr, Joseph cov
```

### Why the $\delta\mathbf{p} = -\mathbf{H}_f$ detail matters (the #212 thread)

A global translation shifts **every clone *and* the feature by the same vector** $\mathbf{t}$. Plug that into one observation strip: the feature contributes $+\mathbf{H}_f\,\mathbf{t}$ and the clone-position block contributes $-\mathbf{H}_f\,\mathbf{t}$ — they **cancel exactly**, for any $\mathbf{H}_f$. So translation is unobservable *by construction*, regardless of linearization point. That is the $\pm\mathbf{H}_f$ cancellation that passes as $\text{trans\_leak} < 10^{-10}$ in the observability probe.

The **yaw** gauge has no such clean cancellation: its clone-rotation direction is $\mathbf{R}_i^\top \hat{\mathbf{g}}$, which *depends on each clone's estimate*. Linearize different clones at differently-drifted $\mathbf{R}_i$ and the $[\mathbf{y}]_\times / \mathbf{R}_i^\top$ terms no longer annihilate the yaw null vector — that is the leak we chased, and exactly the $\mathbf{H}_\theta = \mathrm{dh}\,\mathbf{R}_{ic}^\top [\mathbf{y}]_\times$ term that the R-IEKF reparameterization replaces with a state-independent constant.
