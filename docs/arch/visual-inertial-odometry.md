# Visual Inertial Odometry

Visual-Inertial Odometry is one of the harder things in robotics perception, so I'll give you a real mental model rather than a tutorial-shaped wrapper around a GitHub repo. Let me lay out the architecture first, then walk the components, then the parts that actually break, then a concrete build path.

## The core idea: why fuse at all

A camera and an IMU are complementary in almost exactly the right way. A camera gives you rich geometric structure but at ~20–30 Hz, suffers under motion blur and low texture, and — if monocular — is **scale-ambiguous** (you can't tell a small scene close up from a large scene far away). An IMU gives you angular velocity and specific force at 100–1000 Hz, is immune to texture, and crucially senses **gravity**, which fixes metric scale and two rotational DoF (roll/pitch). But integrate an IMU alone and it drifts catastrophically within seconds because you're double-integrating biased, noisy acceleration.

Fusing them: the IMU carries you through fast motion and texture-poor stretches and resolves monocular scale; the camera corrects IMU drift whenever structure is visible. The result is a system that is locally metric and accurate. Note the word *odometry* — VIO drifts globally over long trajectories. Adding loop closure is what turns it into SLAM.

There are two axes of design choice you must understand before anything else:

**Loosely vs. tightly coupled.** Loosely coupled runs a vision pipeline and an IMU pipeline separately and fuses their outputs. Tightly coupled jointly estimates everything from raw measurements (pixel reprojection residuals + IMU residuals in one estimator). Tightly coupled is strictly more accurate and is what every serious system uses; loosely coupled is mostly a historical/embedded compromise.

**Filtering vs. smoothing (optimization).** This is the real fork:

## The pipeline

Here's the data flow of a tightly-coupled, optimization-based VIO system (the dominant modern design — VINS-Mono, OKVIS, Basalt all look like this):

![VIO pipeline](tightly_coupled_vio_pipeline.svg)

The dashed line is the part people underestimate: marginalizing old states out of the window produces a Gaussian prior that feeds back in, and getting that prior wrong is the most common source of subtle drift and inconsistency.

## The sensors and their error models

Get the measurement models right or nothing downstream works.

The **IMU** is the part most software engineers underrespect. The accelerometer measures *specific force* — true acceleration minus gravity, expressed in the sensor frame — not acceleration. The gyroscope measures angular velocity. Both are corrupted by additive white noise *and* a slowly-varying **bias** that follows a random walk. So your measurement model is `measured = true + bias + white_noise`, with the bias itself being a state you must estimate online, not a constant. You characterize the four noise parameters (gyro/accel white noise density and bias random-walk density) from a static Allan-variance analysis of your specific IMU; using a manufacturer datasheet number is a reliable way to get a poorly-tuned estimator.

The **camera** model is the standard pinhole projection plus a distortion model (radial-tangential for normal lenses, equidistant/Kannala-Brandt for fisheye). The detail that destroys naive implementations is **rolling shutter**: most cheap CMOS sensors expose rows sequentially over ~20–30 ms, so under motion every row has a different pose. You either use global-shutter hardware (the correct answer for a first system) or you model the per-row time offset explicitly. Don't pretend it isn't there.

## Calibration and time sync — the unglamorous prerequisite

Two things will silently wreck accuracy before any algorithm runs. First, the **camera-IMU extrinsic** (the rigid transform between the two frames) and the camera intrinsics — you estimate these offline with Kalibr. Second, **temporal alignment**: the camera and IMU clocks are rarely synchronized, and a constant time offset of even a few milliseconds maps directly into pose error during fast motion. Either hardware-trigger the camera off the IMU clock, or estimate the time offset as a state (VINS-Mono does this online). Budget real time for calibration; a beautifully coded estimator on badly-calibrated data is worthless.

## IMU preintegration — the key idea that makes optimization tractable

Naively, between two camera frames you'd integrate the IMU equations forward from the start pose/velocity to get a motion constraint. The problem: in an iterative optimizer the start pose/velocity *changes every iteration*, so you'd have to re-integrate hundreds of raw IMU samples each time. **Preintegration** (Lupton & Sukkarieh, then Forster et al., *On-Manifold Preintegration*, TRO 2017) reformulates the integration into relative quantities — a delta-rotation, delta-velocity, and delta-position — that are independent of the absolute starting state, computed once. Because rotation lives on `SO(3)`, this is done on-manifold. The only remaining dependency is on the bias, handled with a first-order Jacobian correction so a bias update doesn't force re-integration. The output is a single relative-motion factor with a properly propagated covariance connecting two keyframes. This is the standard inertial residual in every modern optimization-based VIO, and GTSAM ships it as `CombinedImuFactor`. Understand this paper before writing any backend code.

## The frontend

This is the part that resembles ordinary computer vision. Two schools: **indirect/feature-based** methods detect points (FAST/Shi-Tomasi corners), track them frame-to-frame with KLT optical flow or match descriptors, reject outliers with RANSAC on the epipolar/fundamental constraint, and feed 2D observations to the backend. **Direct methods** (used by ROVIO, DSO-style systems) skip features and minimize photometric error on image patches directly, which is more robust in low-texture scenes but more sensitive to photometric calibration and large motions. For a first system, build a KLT-based indirect frontend; it's the most forgiving to debug because you can visualize tracked features and immediately see when tracking falls apart.

## The backend: filter vs. smoother

This is the genuine architectural fork, and you should pick deliberately rather than by fashion.

**Filtering — the MSCKF.** The Multi-State Constraint Kalman Filter (Mourikis & Roumeliotis, 2007) is the clever filter design. Instead of putting 3D landmarks in the EKF state (which would make state size grow with the map and blow up the covariance update), it keeps a *sliding window of recent camera poses* in the state. When a feature is finally lost, it's triangulated and its reprojection residuals are projected onto the null space of the feature Jacobian — algebraically marginalizing the landmark while still imposing a multi-view constraint across all the poses that saw it. The result is bounded computation regardless of map size, which is why MSCKF-style systems (OpenVINS, the older MSCKF-VIO) remain attractive on compute-constrained platforms.

**Smoothing — sliding-window optimization.** Here you solve a nonlinear least-squares problem over a window of recent keyframe states, with reprojection residuals and preintegrated IMU residuals as factors (VINS-Mono, OKVIS, Basalt, ORB-SLAM3's inertial mode). This is more accurate than filtering because it *relinearizes* at each iteration rather than committing to a single linearization, at higher computational cost. With modern hardware the accuracy advantage usually wins, which is why most current systems are smoothers. Incremental smoothing (GTSAM's iSAM2) recovers much of the lost efficiency by only re-solving the parts of the factor graph that change.

The honest summary: filters trade accuracy for compute and were historically favored for embedded; smoothers are now the default unless you're severely compute-limited.

## The three things that actually break

**State representation on manifolds.** Orientation is not a vector. You cannot stuff Euler angles into your state and run gradient descent — you'll hit gimbal lock and the optimizer will misbehave near singularities. Rotations live on `SO(3)`, poses on `SE(3)`, and you optimize using the Lie-algebra tangent space (the `exp`/`log` maps), with retraction back onto the manifold after each update. Use Sophus for the group operations and let Ceres (via its `Manifold`/local-parameterization machinery) or GTSAM handle the on-manifold update. This is non-negotiable and a frequent source of bugs in homegrown implementations.

**Initialization.** Monocular VIO must bootstrap scale, gravity direction, initial velocity, and IMU biases before the main estimator can run, and it must do so from a short, possibly poorly-excited motion segment. The standard recipe (VINS-Mono) is a vision-only structure-from-motion solution up to scale, then a visual-inertial alignment that recovers the metric scale, gravity, velocity, and gyro bias by matching the SfM motion to the preintegrated IMU. If the motion lacks sufficient acceleration excitation, scale is weakly observable and initialization fails or converges to garbage. This is one of the top real-world failure modes.

**Observability and consistency.** VIO has exactly **four unobservable directions**: global 3D position and rotation about the gravity vector (yaw). Roll and pitch *are* observable because the accelerometer senses gravity. The danger is that a standard EKF, by linearizing different residuals at inconsistent state estimates, can spuriously gain "information" along the unobservable yaw direction — making the filter overconfident and inconsistent, which manifests as drift the covariance claims shouldn't exist. The fix is **First-Estimate Jacobians** (Huang/Mourikis/Roumeliotis): evaluate the Jacobians at fixed first estimates of each state so the estimator's observability properties match the true system's. Optimization-based systems face the analogous issue through their marginalization prior. If you skip this, your system will work on EuRoC and then drift mysteriously in deployment.

## Odometry vs. SLAM

Everything above is *odometry* — locally accurate, globally drifting. To bound global drift you add a place-recognition module (DBoW2 bag-of-words is the standard) to detect loop closures, then run a pose-graph optimization to redistribute the accumulated error. VINS-Mono does this as a separate 4-DoF pose graph (the 4 corresponding to the observable/unobservable split). Decide early whether you're building VIO or full VI-SLAM; the loop-closure subsystem is largely independent of the odometry core and can be added later.

## A realistic build path

A critical word first: do not write your own nonlinear least-squares solver or Lie-group library for this. The educational return is real but it's a *different* project, and Ceres/GTSAM/Sophus are battle-tested in ways your weekend code won't be. Spend your effort on the VIO-specific logic.

The C++ toolchain: Eigen for linear algebra, Sophus for `SO(3)`/`SE(3)`, OpenCV for image processing and KLT, OpenGV for geometric relative-pose solvers, and then either **GTSAM** (factor graphs, iSAM2, built-in `CombinedImuFactor` — the fastest path to a working smoother) or **Ceres** (general NLS, what VINS-Mono uses). For calibration: Kalibr offline plus an Allan-variance tool for IMU noise. For data and evaluation: the **EuRoC MAV** dataset is the canonical VIO benchmark (stereo + synced IMU + millimeter ground truth), TUM-VI for fisheye, and the `evo` package for computing ATE/RPE — remember to align monocular trajectories with a 7-DoF Sim(3) transform and metric trajectories with 6-DoF SE(3).

A sane sequence: (1) read Forster's preintegration paper, the MSCKF paper, the VINS-Mono paper, and Barfoot's *State Estimation for Robotics* (free PDF — the manifold and EKF foundations); (2) clone and *run* one reference system on EuRoC to establish what "working" looks like — OpenVINS is the best-documented filter-based one, VINS-Fusion the canonical smoother; (3) implement components in isolation against ground truth: first a KLT feature tracker, then IMU preintegration as a GTSAM factor (validate the propagated covariance against integrated raw IMU), then a two-frame visual-inertial alignment, then the sliding window; (4) only then assemble and benchmark with `evo`.

Set expectations honestly: a from-scratch VIO that's competitive on EuRoC is a multi-month effort even for an experienced developer, and most of the difficulty is in calibration, initialization, and consistency — not the parts that look hard in the papers.

One caveat on currency: learning-based and hybrid VIO (deep feature frontends, learned IMU denoising, end-to-end approaches) has been an active research direction, and if you care about the current state of the art beyond the classical pipeline above, that's worth a literature search rather than relying on what I've described, which is the established classical foundation.

Where do you want to go deeper — the math of one component, the GTSAM vs. Ceres tradeoff for your target platform, or the calibration workflow?

## IMU measurement model and noise characteristics in VIO

The IMU model is where careful engineers separate from the ones whose system "works on EuRoC and drifts in the field." Let me give you the model, then the noise structure, then the parts that actually bite.

## The measurement model

A strapdown 6-axis IMU gives you two vector measurements at high rate. The gyroscope measures angular velocity of the body frame in the body frame; the accelerometer measures **specific force** — proper acceleration, *not* coordinate acceleration. This distinction is the single most important thing in the whole model. A stationary IMU reads roughly $+9.81\,\text{m/s}^2$ along its up axis (it senses the normal force resisting gravity), not zero. If your code expects a static IMU to read zero acceleration, your gravity alignment is already broken.

The standard VIO measurement model (Forster et al. notation) is:

$$\tilde{\boldsymbol{\omega}}_b = \boldsymbol{\omega}_b + \mathbf{b}_g + \mathbf{n}_g$$

$$\tilde{\mathbf{a}}_b = \mathbf{R}_{wb}^\top\,(\mathbf{a}_w - \mathbf{g}_w) + \mathbf{b}_a + \mathbf{n}_a$$

The tilde is the measurement, $\mathbf{b}$ is the bias, $\mathbf{n}$ is white noise, $\mathbf{R}_{wb}$ rotates body to world. Read the accelerometer equation carefully: the sensor reports $\mathbf{R}_{wb}^\top(\mathbf{a}_w - \mathbf{g}_w)$, so to recover world acceleration you must invert orientation *and add gravity back*:

$$\mathbf{a}_w = \mathbf{R}_{wb}(\tilde{\mathbf{a}}_b - \mathbf{b}_a - \mathbf{n}_a) + \mathbf{g}_w$$

Two consequences fall out immediately. First, **orientation error leaks directly into position** through that gravity subtraction — a tiny attitude tilt rotates a ~$9.81\,\text{m/s}^2$ gravity vector into your horizontal axes, and double-integrating that error is brutal. Second, gravity must be known: its direction is estimated at initialization, and its magnitude is location-dependent (≈9.78–9.83 depending on latitude/altitude), so a hardcoded 9.81 with the wrong sign convention is a classic init failure. Pin down your gravity sign and frame convention before anything else.

## The bias is a state, not a calibration constant

The biases drift — slowly, with temperature, between power cycles — so they are modeled as random walks driven by white noise and **estimated online** as part of the VIO state:

$$\dot{\mathbf{b}}_g = \mathbf{n}_{bg}, \qquad \dot{\mathbf{b}}_a = \mathbf{n}_{ba}$$

This is the random-walk approximation of what is physically a flicker-noise (1/f) process. It's not exact, but it's tractable in a Kalman/factor-graph framework, which is why everyone uses it. The camera observations are what make the biases observable at all. A critical asymmetry: **gyro bias is well-observed, accelerometer bias is weakly observed**, because a constant accel bias is nearly indistinguishable from a small attitude tilt (both push gravity into the horizontal axes). Accel bias typically converges slowly and noisily, and badly-excited motion can leave it essentially unidentified. Don't be surprised when it's the flakiest part of your state vector.

## The two noise types and their units

There are exactly two stochastic terms per sensor, and they're physically different:

The **white measurement noise** ($\mathbf{n}_g$, $\mathbf{n}_a$) is broadband, zero-mean, modeled as continuous-time white Gaussian with a power spectral density. Integrated once, gyro white noise produces a random walk in *angle* (Angle Random Walk, units $\text{rad/s}/\sqrt{\text{Hz}}$) and accel white noise produces a random walk in *velocity* (Velocity Random Walk, $\text{m/s}^2/\sqrt{\text{Hz}}$).

The **bias random walk** ($\mathbf{n}_{bg}$, $\mathbf{n}_{ba}$) is the white noise driving the bias evolution above, with densities in $\text{rad/s}^2/\sqrt{\text{Hz}}$ (gyro) and $\text{m/s}^3/\sqrt{\text{Hz}}$ (accel).

Those four numbers — gyro noise density, gyro bias random walk, accel noise density, accel bias random walk — are exactly the four fields in a Kalibr-style IMU yaml, and they are the only IMU stochastic parameters a standard VIO consumes.

## Reading them off: Allan deviation

You characterize those four parameters with an Allan-variance analysis: log several hours of a stationary IMU, compute the Allan deviation $\sigma(\tau)$ over averaging window $\tau$, and read the regimes off the log-log plot.

![Allan Deviation Plot](allan_deviation_imu_noise.svg)

The descending $-\tfrac{1}{2}$ branch gives the white-noise density (the value of the fitted line at $\tau=1\,\text{s}$ is the ARW/VRW coefficient); the rising $+\tfrac{1}{2}$ branch gives the bias random walk. A frequent error here: people read the flat minimum (the *bias instability* in $\text{rad/s}$ or $\text{m/s}^2$) and plug it in as the bias random-walk parameter. That's dimensionally wrong — the floor and the $+\tfrac{1}{2}$ slope are different quantities. Use the slope, not the floor.

## The discrete-time conversion — the silent bug

Everything above is continuous-time spectral density. Your IMU is sampled at interval $\Delta t$, and you must convert before feeding the estimator. The scalings go in *opposite* directions for the two noise types:

$$\sigma_{\text{white},d} = \frac{\sigma_{\text{white}}}{\sqrt{\Delta t}}, \qquad \sigma_{\text{biasRW},d} = \sigma_{\text{biasRW}}\sqrt{\Delta t}$$

So the per-sample white-noise covariance is $\sigma^2/\Delta t$ and the per-sample bias-walk increment covariance is $\sigma_b^2\,\Delta t$. GTSAM's `PreintegrationParams` expects the *continuous* densities and applies these internally; if you instead hand it discrete values, or hardcode covariances at one rate and then change the IMU frequency, you silently mis-weight every inertial factor. This is one of the most common bugs in homegrown VIO and it's invisible — the system runs, it's just quietly inconsistent.

## What the model omits, and when that matters

The model above is the consumer-MEMS VIO standard. Be aware of what it drops:

It assumes the sensor triad is perfectly orthogonal and unit-scaled, i.e. $\tilde{\mathbf{a}} = \mathbf{a} + \mathbf{b} + \mathbf{n}$ rather than the fuller $\tilde{\mathbf{a}} = \mathbf{M}\mathbf{a} + \mathbf{b} + \mathbf{n}$ where $\mathbf{M}$ carries scale-factor and axis-misalignment errors. For decent MEMS units folded into a noise budget this is fine; for cheap parts or high accuracy you calibrate $\mathbf{M}$ separately. It also ignores Earth's rotation rate ($\sim 7.29\times10^{-5}\,\text{rad/s}$) and Coriolis terms, which is correct for MEMS gyros whose bias alone is often $10^{-2}$–$10^{-1}\,\text{rad/s}$ — but wrong for tactical/navigation-grade IMUs over long baselines, where you'd integrate them in the true-rate term. Gyro g-sensitivity and temperature dependence are similarly dropped.

The practical warning that matters most: **the Allan-variance numbers from a quiet desk recording are usually too optimistic for the field.** Real platforms have vibration, thermal transients, and mounting resonances that inflate the effective in-flight noise. Inflating your fitted densities by roughly 2–10× is standard practice to get a *consistent* estimator (one whose covariance honestly reflects its error), and tuning that inflation is most of practical VIO tuning. An estimator that overtrusts the IMU diverges; one that underweights it throws away the high-rate benefit that justified adding the IMU at all.

Want to go from here into how these noise terms propagate through preintegration into the covariance of the relative-motion factor, or into the bias observability problem and why accel bias is the troublesome one?

## Noise Propagation

Both. Take them in order — the covariance first, because it sets up why the bias problem in part two is so stubborn.

## How IMU noise becomes the factor covariance

Between two keyframes $i$ and $j$ you accumulate three preintegrated increments, defined so they depend only on the IMU measurements in the interval, not on the absolute state at $i$:

$$\Delta\mathbf{R}_{ij} = \mathbf{R}_i^\top\mathbf{R}_j, \quad \Delta\mathbf{v}_{ij} = \mathbf{R}_i^\top(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}\,\Delta t_{ij}), \quad \Delta\mathbf{p}_{ij} = \mathbf{R}_i^\top\!\left(\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i\Delta t_{ij} - \tfrac{1}{2}\mathbf{g}\,\Delta t_{ij}^2\right)$$

These are built incrementally from the bias-corrected measurements, e.g. $\Delta\mathbf{R}_{i,k+1} = \Delta\mathbf{R}_{ik}\,\mathrm{Exp}\big((\tilde{\boldsymbol{\omega}}_k - \mathbf{b}_g)\Delta t\big)$, and similarly for $\mathbf{v}$ and $\mathbf{p}$. The white noise from last turn is what corrupts each step.

The trick is to factor each *measured* increment into the noise-free value times a small perturbation, on-manifold for rotation: $\widetilde{\Delta\mathbf{R}}_{ij} = \Delta\mathbf{R}_{ij}\,\mathrm{Exp}(-\delta\boldsymbol{\phi}_{ij})$, and additively for the rest: $\widetilde{\Delta\mathbf{v}}_{ij} = \Delta\mathbf{v}_{ij} + \delta\mathbf{v}_{ij}$, $\widetilde{\Delta\mathbf{p}}_{ij} = \Delta\mathbf{p}_{ij} + \delta\mathbf{p}_{ij}$. Stack the 9-vector $\boldsymbol{\eta}_{ik} = [\delta\boldsymbol{\phi}_{ik};\,\delta\mathbf{v}_{ik};\,\delta\mathbf{p}_{ik}]$ and the error propagates *linearly* one IMU sample at a time:

$$\boldsymbol{\eta}_{i,k+1} = \mathbf{A}_k\,\boldsymbol{\eta}_{ik} + \mathbf{B}_k\,\boldsymbol{\eta}^d_k$$

where $\boldsymbol{\eta}^d_k = [\mathbf{n}_g^d;\,\mathbf{n}_a^d]$ is the discrete IMU noise. So the covariance accumulates recursively, starting from zero:

$$\boldsymbol{\Sigma}_{i,k+1} = \mathbf{A}_k\,\boldsymbol{\Sigma}_{ik}\,\mathbf{A}_k^\top + \mathbf{B}_k\,\boldsymbol{\Sigma}_{\boldsymbol{\eta}}\,\mathbf{B}_k^\top, \qquad \boldsymbol{\Sigma}_{ii} = \mathbf{0}$$

with $\boldsymbol{\Sigma}_{\boldsymbol{\eta}} = \mathrm{diag}\!\big(\tfrac{\sigma_g^2}{\Delta t}\mathbf{I},\ \tfrac{\sigma_a^2}{\Delta t}\mathbf{I}\big)$ — exactly the discrete white-noise covariance from the previous turn, which is why the $\Delta t$ scaling there had to be right.

The structure of $\mathbf{A}_k$ and $\mathbf{B}_k$ is where the physics shows up. In block form (rows $\delta\boldsymbol{\phi}, \delta\mathbf{v}, \delta\mathbf{p}$):

$$\mathbf{A}_k = \begin{bmatrix} \Delta\mathbf{R}_{k,k+1}^\top & \mathbf{0} & \mathbf{0} \\ -\Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \mathbf{b}_a)^\wedge\Delta t & \mathbf{I} & \mathbf{0} \\ -\tfrac{1}{2}\Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \mathbf{b}_a)^\wedge\Delta t^2 & \mathbf{I}\Delta t & \mathbf{I} \end{bmatrix}, \quad \mathbf{B}_k = \begin{bmatrix} \mathbf{J}_r^k\Delta t & \mathbf{0} \\ \mathbf{0} & \Delta\mathbf{R}_{ik}\Delta t \\ \mathbf{0} & \tfrac{1}{2}\Delta\mathbf{R}_{ik}\Delta t^2 \end{bmatrix}$$

where $\mathbf{J}_r^k$ is the $\mathrm{SO}(3)$ right Jacobian at $(\tilde{\boldsymbol{\omega}}_k-\mathbf{b}_g)\Delta t$ and $(\cdot)^\wedge$ is the skew operator. Read the off-diagonal skew terms in the $\delta\mathbf{v}$ and $\delta\mathbf{p}$ rows: they couple *rotation error into velocity and position error*. That's the same gravity-leakage mechanism from the measurement model, now living inside the factor covariance — orientation uncertainty inflates the position uncertainty because acceleration is rotated by $\Delta\mathbf{R}$ before it's integrated.

Three things follow that matter in practice. The covariance is **dense** — those cross-blocks between $\phi$, $v$, $p$ are real correlations; approximating $\boldsymbol{\Sigma}_{ij}$ as block-diagonal to "simplify" the weighting throws away information and degrades the estimate. It **grows with the interval** (the velocity/position blocks roughly with $\Delta t$ and the couplings with higher powers), so a longer preintegration window produces a softer, less informative factor — one reason you keyframe often enough rather than letting intervals stretch. And the bias is deliberately *not* in this 9-D propagation.

The bias enters two other ways. Its slow drift is handled by first-order Jacobians $\partial\Delta\mathbf{R}/\partial\mathbf{b}_g$, $\partial\Delta\mathbf{v}/\partial\mathbf{b}_{g,a}$, $\partial\Delta\mathbf{p}/\partial\mathbf{b}_{g,a}$, which let you correct the preintegrated increment for a changed bias estimate *without re-integrating raw IMU* — the whole point of preintegration. And its random walk adds a separate covariance $\mathrm{diag}(\sigma_{bg}^2\Delta t_{ij},\,\sigma_{ba}^2\Delta t_{ij})$ between $\mathbf{b}_i$ and $\mathbf{b}_j$. GTSAM's `CombinedImuFactor` folds this in, giving a 15-D residual (the 9-D motion error plus the 6-D bias change); the lighter `ImuFactor` is 9-D and needs a separate bias `BetweenFactor`. The final residual is whitened by the accumulated covariance — the optimizer minimizes $\lVert\mathbf{r}_{ij}\rVert^2_{\boldsymbol{\Sigma}_{ij}} = \mathbf{r}_{ij}^\top\boldsymbol{\Sigma}_{ij}^{-1}\mathbf{r}_{ij}$ — with the rotation part of $\mathbf{r}_{ij}$ taken through $\mathrm{Log}(\cdot)$ on the manifold and the bias correction applied via those Jacobians.

## Why accelerometer bias is the troublesome state

Gyro bias and accel bias are both observable in principle, but their *degree* of observability under realistic motion is wildly different, and the asymmetry comes straight out of the measurement model.

Gyro bias directly corrupts integrated rotation, and vision tracks orientation tightly over a window, so even modest rotation pins it down quickly. It converges fast and stays well-behaved.

Accelerometer bias has a fundamental ambiguity with attitude. Take a near-static or constant-velocity case ($\mathbf{a}_w \approx 0$), where the model reduces to $\tilde{\mathbf{a}}_b \approx -\mathbf{R}_{wb}^\top\mathbf{g} + \mathbf{b}_a$. A small attitude error $\delta\boldsymbol{\theta}$ perturbs the gravity term by roughly $(\mathbf{R}_{wb}^\top\mathbf{g})^\wedge\delta\boldsymbol{\theta}$ — a horizontal shift in specific force — and a constant horizontal accel bias produces the *same* horizontal shift. From a single static reading the two are indistinguishable:

![bias and tilt are entangled](accel_bias_gravity_ambiguity.svg)

What breaks the ambiguity is **rotation**. The accel bias is fixed in the body frame, while the gravity projection $\mathbf{R}_{wb}^\top\mathbf{g}$ rotates as the platform reorients. Observe the sensor under several different orientations and the body-fixed offset separates cleanly from the orientation-dependent gravity term. Translational acceleration excitation helps too — a varying true acceleration makes the constant bias stand out, especially with vision supplying metric position constraints — but rotational excitation is the dominant lever. The corollary is that under degenerate motion (no rotation, or constant acceleration, or hovering) accel bias becomes weakly observable or unobservable, and the estimator will quietly trade real attitude error against bias error in whatever combination the prior favors, producing slow position drift that the covariance doesn't warn you about.

This is the same reason monocular VIO initialization demands excitation: the visual-inertial alignment that recovers scale, gravity direction, and biases is solving for exactly these entangled quantities, and a gentle or pure-rotation startup leaves scale and accel bias under-determined — the initializer either fails or converges to a confident wrong answer. Martinelli's closed-form analysis and the Hesch/Kottas/Roumeliotis observability work characterize precisely which motions are degenerate.

It also connects back to consistency. Because the linearized system must share the unobservable subspace of the true nonlinear system, evaluating Jacobians at inconsistent estimates can make the estimator believe a bias or yaw direction is observable when it isn't, injecting spurious information and overconfidence. First-Estimate Jacobians fix the linearization points to preserve the correct null space — and the bias directions are part of what FEJ is protecting.

The practical upshot for tuning: treat accel bias as your least-trustworthy state. Its random-walk parameter and initial uncertainty have outsized effect — too tight and it can't follow real thermal drift, too loose and white noise corrupts it — and you should never read instantaneous accel-bias estimates as ground truth, especially during low-excitation segments. If a VIO run drifts in position while orientation looks fine, a mis-tuned or unobservable accel bias absorbing attitude error is one of the first suspects.

That closes the loop from the raw measurement model through preintegration to the estimator. The natural next step, if you want it, is how this 15-D state and its factor plug into the sliding-window graph — what gets marginalized, and how the marginalization prior interacts with the FEJ requirement to keep the whole thing consistent.

