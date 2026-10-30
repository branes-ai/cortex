# Keeping the covariance physically feasible — practice and a plan for cortex

Is it customary for a deployed state estimator to check its covariance P at run time
(symmetric, positive semi-definite, variances in range), so that a numerical failure
does not take the vehicle or the path planner down with it? What would cortex need?

- Related: `docs-site/src/content/docs/debugging/mixed-precision.mdx` (the float
  divergence this answers), `docs/assessments/covariance-architecture.md` (the
  square-root backend, #187), #450, #533.
- License note: ArduPilot and OpenVINS are GPLv3. They are described here by function
  name and behaviour only; no code was copied, per the no-GPL rule.

---

## 1. Short answer

**Yes in flight autopilots, mostly no in research VIO.** PX4's EKF2 and ArduPilot's
EKF3, both of which run in single precision, guard P in layers:
- they **prevent** with symmetric-by-construction writes and scalar sequential fusion;
- they **detect** with an innovation-variance test (S ≥ R) and a pre-commit
  negative-variance test;
- they **recover** by skipping the fusion, clamping variances to per-state bands, or
  resetting the covariance;
- they **signal** with fault flags that feed a health score, lane or instance
  switching, and vehicle failsafes.

PX4 calls the clamping "a last resort [that] should not be relied on". Research VIO
code does far less. OpenVINS scans the diagonal after every propagation and update,
and exits the process on a negative value. ETH's MSF symmetrizes and logs NaN.
VINS-Fusion's failure detector is disabled.

## 2. What happened in cortex

The MSCKF backend runs each stage in a chosen arithmetic type (#450). On EuRoC, a
stage that **writes P** in float can drive P indefinite, after which the estimate runs
away by 10⁴–10⁵ m:

| Sequence | Writes P in float (S2, S3, S9 or S6e alone) | Reads P only in float (S0, S5 → S6d) |
|---|---|---|
| V1_01 | each is fine alone; S6e diverges together with float S5 | fine |
| MH_05 | **each one alone diverges** | fine |
| V2_03 | fine | fine |

The mechanism is a large information gain on a P that has passed through float. A
stage in float rounds P into float and writes it back, which leaves an absolute error
of about ε_float·‖P‖. When a burst of updates shrinks P by one to three orders of
magnitude, the true small variances fall below that error and P comes out indefinite:
- V1_01: at takeoff, frame 105, P's largest diagonal drops 1.49 → 0.135.
- MH_05: in frames 23–80, the largest diagonal drops 0.53 → 0.018 and the smallest
  4·10⁻⁴ → 9·10⁻⁷.

The next updates then amplify the negative directions. The full diagnosis is in the
mixed-precision page.

## 3. What practice exists

Each finding is marked by how it was checked: **verified** means read in the source
for this report; **reported** means taken from a source reading but not re-checked
line by line.

### Prevent

- **Joseph form with symmetric writes.**
  - PX4 `Ekf::measurementUpdate()` (`ekf2/EKF/ekf_helper.cpp`) is reported to expand
    the Joseph form in two O(n²) steps and write one triangle.
  - ArduPilot `NavEKF3_core::FinishFusion()` (`AP_NavEKF3_core.cpp`) writes
    P − ½(KHP + (KHP)ᵀ) to both triangles. **Verified.**
  - Joseph form keeps P PSD for any gain only in exact arithmetic. Under rounding it
    does not, when one update removes most of the variance, which is the cortex case.
- **Symmetric-by-construction prediction.** PX4 `Ekf::predictCovariance()` takes
  P from `sym::PredictCovariance` (SymForce-generated, upper triangle only) and copies
  the upper triangle to the lower. **Verified.**
- **Scalar sequential fusion.** PX4 and ArduPilot fuse one scalar observation at a
  time: there is no matrix inverse, and each step is a small information gain.
  Reported.
- **Square-root and factored forms.** Potter, then Carlson's triangular form, then
  Bierman's U-D factorization (*Factorization Methods for Discrete Sequential
  Estimation*, 1977). They carry a factor whose condition number is √κ(P). They are
  the standard aerospace answer to short word lengths. NASA/TP-2018-219822
  ("Navigation Filter Best Practices") recommends U-D factorization for flight filters.
  - Wu et al.'s square-root inverse sliding-window filter (RSS 2015) runs a visual-
    inertial sliding-window filter entirely in single precision on a phone. It argues
    that the non-square-root form "necessitates using double-precision arithmetic".

### Detect

- **Pre-commit negative-variance test.**
  - ArduPilot `FinishFusion()` refuses the whole fusion, state and covariance, if any
    (KHP)ᵢᵢ > Pᵢᵢ. **Verified.**
  - Legacy PX4-ECL `Ekf::checkAndFixCovarianceUpdate()` ran the same test.
    **Verified.**
- **Innovation-variance test.** If S < R, then HPHᵀ < 0 and P is indefinite along H.
  PX4 and ArduPilot run this in several fusions (yaw, magnetometer, airspeed), set a
  numerical-fault bit, and reset the covariance. Reported.
- **Post-operation diagonal scan.** OpenVINS `StateHelper::EKFUpdate()` and
  `EKFPropagation()` check every diagonal and call `std::exit(EXIT_FAILURE)` on a
  negative one. **Verified.**
- **Statistical consistency.** NIS / χ² test ratios on every aiding source (PX4,
  ArduPilot, OpenVINS, MSF). They watch integrity, not numerics, but an indefinite P
  shows up in them: ratios that collapse or explode.

### Recover

- **Clamp variances to bands.**
  - PX4 `Ekf::constrainStateVariances()` keeps per-block floors and caps, with the
    "last resort" comment. **Verified.**
  - PX4 `constrainStateVar()` lowers an over-limit variance by fusing a zero
    innovation instead of clipping it, because clipping "would artificially increase
    the correlation between states and destabilize the filter". **Verified.**
  - ArduPilot `ConstrainVariances()` has per-state bands and re-seeds a collapsed bias
    block. Reported.
- **Skip the update.** ArduPilot `FinishFusion()`, above.
- **Reset the covariance**, in full or by block. PX4 `initialiseCovariance()` and
  ArduPilot `CovarianceInit()` run on a numerical fault. Reported.
- **Not used in flight:** nearest-PSD projection (Higham 1988, eigenvalue clipping).
  It is a valid repair, but it silently changes correlations.

### Signal

- **Health score per instance.**
  - PX4 `EKF2Selector` counts an instance healthy only with zero filter-fault flags,
    and switches instances on a fault. Reported.
  - ArduPilot `NavEKF3_core::healthy()` and lane switching on error score. Reported.
- **Failsafe.**
  - PX4 commander's estimator check turns validity and accuracy (`xy_valid`, `eph`)
    into position-invalid flags, which trigger mode failsafes. Reported.
  - ArduCopter's `ekf_check` escalates: yaw reset, then lane switch, then failsafe
    (land or AltHold). Reported.
- **What a consumer sees.** Validity booleans, accuracy scalars, and **reset counters
  with deltas** (PX4 `VehicleLocalPosition`). A controller or planner absorbs a reset
  without a step, and treats "invalid" as a failsafe trigger. Reported.

## 4. How the float filters make float work

PX4 and ArduPilot do not use a square-root form. They keep float safe by keeping the
problem small and well scaled:
- a ~24-state error state;
- per-state variance bands that bound the scale of each variance (not κ(P): two strongly
  correlated states can keep their variances in band while P becomes near-singular, so bands
  do not replace a feasibility check);
- scalar updates, each a small gain;
- symbolically generated prediction, with fewer operations and so less rounding;
- a numerical guard on every fusion.

A sliding-window MSCKF gives most of that up: 81 states with highly correlated clone
blocks, an exactly singular P after every augmentation, and multi-row stacked updates.
The cortex measurements fit this: the stages that write P need double, or a factored P.

## 5. What cortex does today

- **Prevention:** Joseph form and an explicit `symmetrize` after every predict,
  augment and update (`msckf/covariance.hpp`). A square-root backend exists (#187,
  `vio_pipeline --covariance sqrt`). Since #533 it runs every EuRoC sequence, with
  every stage in float (V1_01 0.295 m, MH_05 0.784 m, V2_03 0.282 m).
- **Detection:** the χ² gate rejects an update whose innovation covariance S doesn't
  factor. `spd_solve` returns a zero gain on a non-SPD S in release, and asserts in
  debug. There is no check of P after an operation, and the stage invariants
  (`eval/invariants.hpp`, `--live-assert`) are diagnostics, not production code.
- **Signal:** `VioBackend` exposes no health flag. A diverged estimate is published
  like a good one.

### Measured: a diagonal guard alone is not enough

A scratch probe added ArduPilot's test to S6e: undo any update that leaves a negative
or non-finite variance. On V1_01 with the update in float, the guard rejected 25777
updates and the run still drifted to 10147 m. The indefiniteness starts at frame 105
in the eigenvalues, while every diagonal is still positive. Once the guard first fires
(frame 111), almost every later update trips it, and the filter runs on the IMU alone.
In double the guard never fired.

So the diagonal test is a good **alarm**, but rejecting an update is not a
**recovery**. Recovery needs a covariance reset, or a covariance form that cannot go
indefinite.

## 6. Recommendations for cortex, ranked

1. **Keep every stage that writes P in double, or give it a factored P.** The measured
   rule: S2, S3, S9 and S6e write P; S0 and S5 → S6d only read it.
   `update-float-ekf-double` (S5 → S6d in float) holds on all three sequences (V1_01
   0.263 m, MH_05 1.16 m, V2_03 0.303 m). Every preset that writes P in float diverges
   on MH_05. The square-root backend is that factored P: with #533 fixed, the five presets
   measured run on all three sequences, the whole filter in float included (MH_05
   0.784 m against 0.755 m in double).
2. **Add a covariance health check to the backend and report it; never exit.** After
   each operation that writes P:
   - check that every value is finite and every diagonal is ≥ 0;
   - check S ≥ R row by row (diag(HPHᵀ) ≥ 0) before an update;
   - every N frames, test P for a negative eigenvalue beyond a stated tolerance, to
     detect indefiniteness before a diagonal goes negative. Frame 105 versus frame 111
     on V1_01 shows why the test is worth its cost.

     The tolerance has to be explicit. Clone augmentation makes P exactly singular, so a
     plain Cholesky of P fails on a healthy filter. A factorization of P + λI passes any P
     whose smallest eigenvalue lies in [−λ, 0), so λ is the detection limit, and a large
     ridge would hide exactly the failure the test is for. Set the tolerance from the
     narrowest type P has passed through: τ = 16·ε·max diag(P), the slack
     `is_positive_semidefinite` and the stage-type tests use. Then test P + τI: a Cholesky of the
     shifted matrix, or an LDLᵀ of it whose inertia (the signs of D, 2×2 blocks
     included) is checked. Do not apply −τ as a pivot floor to an LDLᵀ of the unshifted
     P: congruence preserves the signs of the eigenvalues, not their sizes, so a pivot
     says nothing about how negative an eigenvalue is. On V1_01 with the update in float that separates the
     cases cleanly: before takeoff P is indefinite only at about −10⁻⁷ of its largest
     eigenvalue, well inside 16·ε_float ≈ 2·10⁻⁶, while at frame 105 it is −4·10⁻⁵ and
     the test fires.

   Record faults as bits in a backend status. This is cheap compared with the update.
3. **Define a recovery ladder with a reset counter.**
   1. Reject the update, using the pre-commit test.
   2. On repeated faults, reset the affected covariance blocks to conservative priors,
      or reset the estimator (`VioEstimator::reset()` exists) and re-initialize.
   3. Publish a reset counter and the pose delta, so consumers can absorb the jump.

   Variance bands are the last rung. In an MSCKF, clipping a clone variance breaks its
   correlation with the IMU block, so bands belong on the IMU states only, with PX4's
   zero-innovation shrink rather than a hard clip.
4. **Signal validity to consumers.** The daemon's published estimate should carry a
   validity flag, an accuracy scalar (position σ) and the reset counter, derived from
   the health bits and a windowed NIS monitor. The planner and controller treat
   "invalid" as a failsafe trigger. This is the layer that keeps a numerical fault from
   reaching the vehicle, whatever its cause.
5. **Watch the information gain per frame.** Both failures came from a burst of
   updates that collapsed P (at takeoff, and at the first motion on MH_05). A health
   check can flag a frame whose trace(P) drops by more than a factor of, say, 10, as a
   risk indicator for the numerics.

Items 2–4 are independent of the arithmetic type: the same machinery catches a bad
calibration, a sensor fault or a front-end failure, and those are more likely in the
field than float rounding.

## Sources

- PX4 EKF2 `covariance.cpp`:
  https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF/covariance.cpp
- PX4 `ekf_helper.cpp`:
  https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF/ekf_helper.cpp
- PX4 `EKF2Selector.cpp`:
  https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/ekf2/EKF2Selector.cpp
- PX4 `VehicleLocalPosition.msg`:
  https://github.com/PX4/PX4-Autopilot/blob/main/msg/versioned/VehicleLocalPosition.msg
- Legacy PX4-ECL `covariance.cpp`: https://github.com/PX4/PX4-ECL/blob/master/EKF/covariance.cpp
- ArduPilot `AP_NavEKF3_core.cpp`:
  https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_core.cpp
- ArduCopter `ekf_check.cpp`: https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/ekf_check.cpp
- OpenVINS `StateHelper.cpp`:
  https://github.com/rpng/open_vins/blob/master/ov_msckf/src/state/StateHelper.cpp
- ETH MSF: https://github.com/ethz-asl/ethzasl_msf
- VINS-Fusion `estimator.cpp`:
  https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/estimator/estimator.cpp
- K. Wu, A. Ahmed, G. Georgiou, S. Roumeliotis, "A Square Root Inverse Filter for
  Efficient Vision-aided Inertial Navigation on Mobile Devices", RSS 2015:
  https://www.roboticsproceedings.org/rss11/p08.html
- J. R. Carpenter, C. N. D'Souza, "Navigation Filter Best Practices",
  NASA/TP-2018-219822: https://ntrs.nasa.gov/citations/20180003657
- G. J. Bierman, *Factorization Methods for Discrete Sequential Estimation*, Academic
  Press, 1977.
- N. J. Higham, "Computing a nearest symmetric positive semidefinite matrix", *Linear
  Algebra Appl.* 103 (1988) 103–118.
- P. S. Maybeck, *Stochastic Models, Estimation, and Control*, Vol. 1, 1979.
