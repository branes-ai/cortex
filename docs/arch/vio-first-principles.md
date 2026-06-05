# VIO First principles

We have tried to build a VIO pipeline but we do not have the right structure and the pipeline is not working. I asked CC to look at OpenVINS and
clean-room it, but that was insufficient guidance because your clean room pipeline does not work and after three days you have not been able to find what
went wrong and how to fix it. Your girations have been random and this is a very sensitive mathematical pipeline so it is very unlikely that a random shot
approach is going to resolve the core bugs in this pipeline. So we need to go back to first principles. Can you research the OpenVINS repo and distille
the core VIO pipeline that OpenVINS has designed? My plan is to take this pipeline, and for each step define pre- and post-conditions that would need to
hold true for the algorithm to work. Then build instrumentation for that stage, and unit tests for that stage that validate the core pre- and
post-conditions that need to hold with sensitivity analysis. We need to deconstruct this problem with a clear understanding of the pipeline of
transformations, the pre- and post-conditions that need to hold, and the probes and measurement devices that we need to have in place to measure and
validate these pre- and post-conditions. Let's get started with distilling the core VIO pipeline presented by OpenVINS. Then do the same with other
VIO/SLAM repos like MINS. ALso read docs/arch/visual-inertial-odometry.md, an offline analysis we have done with another LLM.

● I'll start by understanding the current state — reading the offline analysis doc, the troubleshooting notes, and the existing VIO code structure. Then I'll
research OpenVINS and MINS from first principles.

● Let me finish reading the architecture doc and explore the existing VIO code in parallel.

● This is excellent context. The offline analysis is deep and correct, and the troubleshooting doc tells the real story: the existing pipeline is
MSCKF/filter-based (OpenVINS-style), it's structurally over-confident (NEES ≫ 1), and FEJ — the textbook fix — was implemented and refuted by measurement.
So the value here is a precise, first-principles distillation of what OpenVINS actually does, decomposed into stages with contracts, so you can find where
your clean-room diverges.

Let me research OpenVINS properly and map your existing code in parallel. First, load the web tools and launch a code-mapping explorer.

● Explore(Map existing MSCKF VIO pipeline)

● I have a precise map of the existing code — it's a faithful OpenVINS-style MSCKF (world-centric error state, right SO3 perturbation, clones, null-space
projection, Joseph update, FEJ already present). Now let me pull the canonical OpenVINS design to compare against. Fetching the core docs pages in
parallel.

● Fetch(https://docs.openvins.com/propagation.html)

● Fetch(https://docs.openvins.com/update-feat.html)

● Fetch(https://docs.openvins.com/)

● Web Search("MINS robust multisensor-aided inertial navigation OpenVINS extension architecture loop closure online calibration")

● The overview pages are thin on the exact mechanics where clean-rooms break. Let me grab the two pages that matter most for the over-confidence question —
the discrete propagation (Φ/Qd form) and the base update page (QR compression + chi-square gating).

● Fetch(https://docs.openvins.com/propagation_discrete.html)

● Fetch(https://docs.openvins.com/update.html)

● I now have the canonical mechanics. One thing already stands out worth flagging: OpenVINS's discrete process noise carries the noise through the noise
Jacobian G (with the Δt, R·Δt, ½R·Δt² blocks that produce θ–v–p cross-correlations), whereas the existing code adds a diagonal σ²·Δt straight onto the
state covariance — a candidate contract violation to probe later. Let me grab the MINS architecture detail to do the comparison justice, then I'll write
the distillation.

● Fetch(https://arxiv.org/abs/2309.15390)

● Fetch(https://docs.openvins.com/update-compress.html)

● I now have the canonical mechanics confirmed against the source. Rather than dump prose into the chat, I'll write the deliverable as a structured reference
document — the pipeline decomposed into stages, each with a type signature, pre/post-conditions, instrumentation probes, and test+sensitivity ideas,
mapped onto your existing code. That's the artifact your stage-by-stage instrumentation plan attaches to.

● Write(docs/arch/vio-pipeline-canonical.md)
⎿  Wrote 684 lines to docs/arch/vio-pipeline-canonical.md
   1 # The Canonical VIO Pipeline — Distilled for Contract-Driven Reconstruction
   2
   3 > **Purpose.** We have a clean-room OpenVINS-style MSCKF that runs but is structurally
   4 > over-confident (NEES ≫ 1) and we have not been able to find the defect by trial and error
   5 > (FEJ was implemented and *refuted* by measurement — see `docs/assessments/vio-troubleshooting.md`).
   6 > This document goes back to first principles: it distills the **canonical** VIO pipeline as
   7 > designed by OpenVINS (and contrasts MINS / VINS-Mono / ROVIO / ORB-SLAM3), decomposed into
   8 > discrete **stages**. Each stage is written as a *contract*: a type signature, the
   9 > **pre-conditions** that must hold on its inputs, the **post-conditions / invariants** that must
  10 > hold on its outputs, the **probes** that measure whether those conditions hold, and the
 … +680 lines (ctrl+o to expand)

● The deliverable is written: docs/arch/vio-pipeline-canonical.md. Let me update the stale memory — the existing cortex-212-overconfidence-structural note
claims FEJ/observability is the fault, but the troubleshooting doc now refutes FEJ. I should correct that.

● The memory already captures the FEJ refutation well. Let me append the new artifact and the ranked candidate hypotheses from this session, and add a
MEMORY.md pointer.

● The second MEMORY.md line points at a repo doc rather than a memory file — that's not what the index is for, and the path is awkward. The #212 memory body
already names the doc. Let me remove it.

● Done. Here's what I produced and what it found.

Deliverable: docs/arch/vio-pipeline-canonical.md

A first-principles distillation of the canonical VIO pipeline, structured exactly for the method you described — every stage is a contract: type signature
→ pre-conditions → post-conditions/invariants → probes → unit tests + sensitivity analysis → mapped onto your existing code with candidate divergences
flagged.

What I did:
1. Read the offline analysis (visual-inertial-odometry.md) and the troubleshooting log — the latter is the key input: FEJ was implemented and refuted by
measurement, so the fault is not the textbook observability fix.
2. Mapped your existing code (via an Explore agent) — confirmed it's a faithful OpenVINS-style MSCKF and, critically, that the algebra is textbook-correct
(null-space projection, Joseph update, marginalization-by-submatrix, FEJ storage all present and right).
3. Researched OpenVINS from its docs + papers (not source — GPL/clean-room rule): the exact discrete Φ/Qd blocks, the four-map measurement model,
null-space projection, QR compression, chi-square gating, and the full state vector including online calibration.
4. Contrasted MINS/VINS-Mono/ROVIO/ORB-SLAM3 stage-by-stage, with a MINS deep-dive.

The pipeline as stages S0–S10: sensor/calib models → init → propagation → augment/clone → frontend → triangulate → MSCKF update (5 sub-contracts) →
SLAM-feat → zero-velocity → marginalize → online calibration. Plus a consolidated contract/probe matrix and the three master invariants (NEES≈dim, NIS≈dof,
white innovations).

What the distillation surfaced (ranked candidates, each with a decisive local test)

Because the algebra checks out and FEJ was refuted, the over-confidence points elsewhere:

1. Unmodeled calibration uncertainty (highest). Your state is [IMU(15)|clones(6n)] only — no camera extrinsics, intrinsics, or time-offset states or
uncertainty; EuRoC T_BS is taken as truth. OpenVINS/MINS estimate these online or account for their uncertainty. R×4 "fixing" NEES is precisely the
fingerprint of treating imperfect calibration as perfect.
2. Diagonal Q_d. Your propagator adds a 12-term diagonal σ²·Δt straight to the state covariance, not the canonical G·Q_meas·Gᵀ — so it's missing the θ–v–p
cross-correlations.
3. No parallax/conditioning gate on triangulated features (prime suspect for the separate V2_03 aggressive-motion divergence).
4. χ² gate too tight under aggressive motion.

These are independent and individually testable — which is the whole point of your stage-contract approach.

I updated memory accordingly (the old note claimed FEJ/observability was the proven fault — now corrected to reflect the refutation and the new candidate
ranking).

Two notes: I have not run any of the decisive tests or touched the estimator (consistent with your "don't edit until traced to a named contract violation"
guidance). And the sweep script sweep_Q_SCALE.sh plus capability-fixes.txt/continuation.txt are untracked scratch files from the prior session — left them
alone. Want me to start building the S0 sensor-model probe (analytic-vs-numeric Jacobian harness + the time-offset/extrinsic sensitivity sweep) as the
first stage instrument, since candidate #1 hinges on it?

## S0 sensor-model

yes, build the S0 sensor-model probe. Let's add assessments of the input and output in metrics that are native to our problem of modeling. 
Also, think about visualizations of these assessments so that we can qualitatively look at these metrics to build intuition.

● Before writing code, I need to ground the probe in the real APIs. Let me read the camera model, IMU model, an eval header for metric conventions, a sample
  test for style, the test CMake wiring, and check what visualization tooling already exists in the repo.

● Bash(cd /home/stillwater/dev/branes/clones/cortex
      echo "=== camera model / intrinsics search ==="…)

## S0 stage tooling

● Everything is in place and verified. Here's what I delivered for your two requests.

S0 sensor-model probe (built, tested, visualized)

Probe library — sdk/include/branes/sdk/eval/sensor_model_probe.hpp (header-only, type-generic, reusable). Computes everything in units native to the
modeling problem:
- projection round-trip residual (px)
- analytic-vs-numeric Jacobian + linearization error (px)
- camera ↔ IMU time-offset sensitivity (px/ms)
- extrinsic sensitivity (px/deg, px/mm)
- IMU static identity / dead-reckoning drift (m/s², mm) — run through the real Propagator.

Regression test — tests/sdk/sensor_model_probe.cpp (7 Catch2 cases, registered in ctest #154–160, all green), each asserting the contract against an
independent oracle (round-trip identity, central differences, closed-form ½·a·t² drift).

Visualizations — docs-site/scripts/gen-sensor-model-figures.mjs (dependency-free SVG: log-scaled image-plane heatmaps + sensitivity/drift line charts) →
docs/assessments/figures/s0/. I cross-checked them against an independent matplotlib render.

The probe found two real issues while building (a fisheye out-of-FOV corner divergence; a bug in my own IMU case where the tilt cancelled) — exactly what
stage-isolation is for. The headline reading: the EuRoC camera model + Jacobian are exact to machine precision (S0 camera contract holds → over-confidence
is not a projection bug), while the calibration sensitivities quantify the uncertainty the filter omits — the S10 candidate for #212. A 0.5° tilt → 4.3 m
of drift in 10 s.

Per-stage study utilities (S0–S10)

A new host-only tools/ area (not tests, not daemons):
- stage_probe.hpp — harness: prints the contract, native-unit results table, CLI (--help/--list/--out/--no-out), CSV writer.
- vio_stage_contracts.hpp — all 11 stages as machine-readable StageInfo (signature, pre/post-conditions, native-unit assessments, artifacts) — single
source of truth.
- 11 executables s0_sensor_model … s10_online_calibration. S0 fully wired; S1–S10 scaffolds that print their real contract + planned assessments with an
explicit fill-in point.

./build/tools/s0_sensor_model --help    # print the contract
./build/tools/s0_sensor_model --list    # the whole pipeline
./build/tools/s0_sensor_model --out build/stage_probes/S0   # run + write artifacts

Docs: tools/README.md, plus a new §6 in docs/arch/vio-pipeline-canonical.md. Memory updated.

Recommended next (the ranked #212 candidates): wire S2 (verify the diagonal Q_d is missing θ–v–p cross-correlations) and S10 (model calibration
uncertainty, test whether it reproduces the empirical R×4). Want me to wire S2 next, following the S0 pattern?

## S2

● Now update memory — the S2 finding meaningfully re-ranks the #212 candidates:

● Final consolidated verification:

● S2 is wired end to end, following the S0 pattern. Everything builds clean and all tests pass.

What I built for S2 (IMU propagation)

Probe — sdk/include/branes/sdk/eval/propagation_probe.hpp (header-only, type-generic). It drives the real Propagator and reconstructs its Φ (validated
against the shipped covariance to 1e-16, so comparisons are exact), then measures the contract in native units:
- Q-structure: cortex diagonal Q_d vs canonical B·Σ·Bᵀ — confirms cortex drops the position-block (¼σ_a²Δt³) and v–p cross (½σ_a²Δt²) terms.
- Covariance growth: position/velocity/attitude σ, cortex-Q vs canonical-Q, in mm/mm·s⁻¹/deg.
- GT-injection: coarse-vs-fine reference on a tumbling trajectory → discretization error.
- NEES vs Q-scale: the #212 consistency lever, with R-orthonormality, P-PSD, and global-position nullspace checks.

Utility — tools/src/s2_propagation.cpp (replaces the scaffold; --help/--list/--out/--no-out). Test — tests/sdk/propagation_probe.cpp (5 cases, 225
assertions). Figures — S2 charts added to the generator → docs/assessments/figures/s2/.

The finding — and it re-ranks #212

S2 is the kind of result this program exists to produce: a candidate measured and cleared.

┌────────────────────────────────────┬─────────────────────────────┬────────────────────────────────────────────────────┐
│              Reading               │            Value            │                      Meaning                       │
├────────────────────────────────────┼─────────────────────────────┼────────────────────────────────────────────────────┤
│ Φ-reconstruction                   │ 1e-16                       │ comparison is exact (real == reference)            │
├────────────────────────────────────┼─────────────────────────────┼────────────────────────────────────────────────────┤
│ Diagonal-Q position-σ under-report │ ~7% inter-frame, 0.7% @0.5s │ real but minor — a correctness fix, not the driver │
├────────────────────────────────────┼─────────────────────────────┼────────────────────────────────────────────────────┤
│ Propagation-only NEES @ default Q  │ 6.4 (6-DoF target 6)        │ IMU process noise is ~consistent on its own        │
├────────────────────────────────────┼─────────────────────────────┼────────────────────────────────────────────────────┤
│ NEES ∝ 1/Q                         │ 0.25→23.5, 1→6.4, 4→1.5     │ clean lever, crosses dof=6 right at Q-scale 1      │
├────────────────────────────────────┼─────────────────────────────┼────────────────────────────────────────────────────┤
│ GT-injection                       │ O(dt), ~15 mm @200 Hz       │ first-order integrator, halves with dt             │
├────────────────────────────────────┼─────────────────────────────┼────────────────────────────────────────────────────┤
│ R on SO(3) / P PSD / nullspace     │ 2e-16 / PSD / 0 leak        │ mean & covariance invariants hold                  │
└────────────────────────────────────┴─────────────────────────────┴────────────────────────────────────────────────────┘

So candidate #2 (diagonal Q_d) is demoted to a minor correctness fix — propagation is approximately consistent at the default noise, which means the #212
over-confidence enters downstream: S10 (calibration uncertainty) is now the leading candidate, then S6 (update: NIS / null-space / χ² gating), then S5
(parallax, for the V2_03 divergence). Docs (vio-pipeline-canonical.md §5/§6, tools/README.md) and memory updated to reflect the new ranking.

Two pipeline stages are now wired (S0, S2); the other nine remain scaffolds with their contracts printing. Given the re-ranking, the highest-value next
stage is S10 (model the extrinsic/time-offset uncertainty S0 already quantified, and test whether it reproduces the empirical R×4). Want me to wire S10
next, or S6?


● #295 merged to main and #294 closed; branch cleaned up, memory updated.

S10 (calibration) landed — the strongest #212 result so far

The leading candidate is now measured, not guessed:

- Analytic: a realistic 1° camera↔IMU extrinsic uncertainty → ~8 px induced reprojection error (> the filter's assumed ~4.6 px) → R-inflation ~4×, derived
from first principles and matching the empirical "R×4 fixes NEES."
- End-to-end: the synthetic backend reproduces the EuRoC over-confidence almost exactly (pose NEES ≈ 43 ≈ MH_05's 43); R is the consistency lever.
- Honest conclusion: calibration cleanly explains the R×4 component, but R-only needs ~×33 for full consistency because the empirical fix paired R×4 with
Q×10 — so #212 is multi-source: calibration + process noise (Q) + the S5 parallax gate.

The fix indicated: model calibration uncertainty (online T_CI/t_d, OpenVINS/MINS-style, or a calibration term in R) rather than a blunt global R-scale.

This CI round also caught — and I fixed — all three of the repo's known gates in one PR (clang-format column limit, MSVC Unicode test-name, commitlint
lowercase-subject).

State of the #212 program (all by measurement)

- S2 (diagonal Q) — measured, minor.
- S10 (calibration) — quantitatively explains the R×4 component.
- Open sources: process-noise Q + S5 parallax gate (the V2_03 divergence).

Wired stages: S0, S1, S2, S10.

Next options

1. S5 (triangulation parallax gate) — the remaining over-confidence/divergence source; reproduced earlier as the static→motion degenerate triangulation.
2. S6 (MSCKF update) — NIS / null-space / χ² gating.
3. The per-stage stream operators you outlined earlier.



## pipeline stages

Can you list me the stages and their purpose, their pre- and post-condition in a nice table?

● Let me pull the exact pre/post-conditions from the contract registry (the single source of truth) so the table is accurate.

  Read 1 file (ctrl+o to expand)

● Here's the VIO pipeline as the contract registry defines it (tools/include/branes/tools/vio_stage_contracts.hpp) — each stage's purpose, the key conditions
  on its inputs (pre) and the invariants it must guarantee on its outputs (post), and whether it's wired yet.

  Stage: S0 Sensor & calibration models
  Purpose: The measurement functions every residual is built from — camera projection, IMU model, extrinsics T_CI, time offset t_d
  Pre-conditions (inputs): Intrinsics valid (fx,fy>0); gravity sign/magnitude pinned (static IMU reads +g); T_CI/t_d known with a stated uncertainty
  Post-conditions / invariants (outputs): unproject∘project ≈ identity in FOV; analytic ∂pixel/∂point = true derivative; cheirality enforced (z>0);
  stationary
    IMU integrates to ~zero drift
  Status: ✅ wired
  ────────────────────────────────────────
  Stage: S1 Initialization
  Purpose: Bootstrap orientation, gyro bias, gravity, velocity, and (dynamic) metric scale
  Pre-conditions (inputs): Static: accel/gyro std below stationarity gates. Dynamic: sufficient acceleration excitation
  Post-conditions / invariants (outputs): R_GI levels gravity (residual ≈0); ‖g‖ in tolerance; scale s>0 & alignment well-conditioned; initial P honestly
    large on the unobservable directions (yaw, accel-bias, scale)
  Status: ✅ wired
  ────────────────────────────────────────
  Stage: S2 IMU propagation
  Purpose: Advance the mean (strapdown) and covariance (P ← ΦPΦᵀ + Q_d) between images
  Pre-conditions (inputs): Δt>0, monotonic timestamps; measurements debiased; P symmetric PSD
  Post-conditions / invariants (outputs): R ∈ SO(3); P stays symmetric PSD; Q_d carries the θ–v–p cross-correlations; the 4-D unobservable subspace stays in
    the null space (FEJ)
  Status: ✅ wired
  ────────────────────────────────────────
  Stage: S3 State augmentation / cloning
  Purpose: Stochastic-clone the current camera pose into the sliding window
  Pre-conditions (inputs): Clone count < window max; current pose valid; P PSD
  Post-conditions / invariants (outputs): P' = GPGᵀ exactly, PSD; clone marginal = cloned-pose marginal, and its cross-cov to every other state matches (a
    deterministic copy — not independent)
  Status: scaffold
  ────────────────────────────────────────
  Stage: S4 Visual frontend
  Purpose: Detect + track features (KLT), reject outliers → 2-D observations
  Pre-conditions (inputs): Image in expected format; previous tracks; optional gyro rotation prior
  Post-conditions / invariants (outputs): Every track passes forward-backward; RANSAC inliers above a floor; spatial coverage across the grid; healthy
    track-length distribution
  Status: scaffold
  ────────────────────────────────────────
  Stage: S5 Feature triangulation
  Purpose: Triangulate a tracked landmark across its observing clones
  Pre-conditions (inputs): ≥2 obs from distinct, sufficiently-parallax clones; cheirality holds
  Post-conditions / invariants (outputs): Reprojection residual below threshold; depth positive & finite; triangulation condition number bounded (low
  parallax
     ⇒ defer/down-weight)
  Status: scaffold
  ────────────────────────────────────────
  Stage: S6 MSCKF update
  Purpose: Null-space-marginalize the feature, compress, χ²-gate, EKF-update
  Pre-conditions (inputs): Triangulated p_f valid (S5); ≥2 obs; H at FEJ, residual at current
  Post-conditions / invariants (outputs): rank(H_f)=3; null-space N orthonormal ⇒ noise stays σ²I; QR compression exact; P⁺ PSD (Joseph), on-manifold;
  MASTER:
     NEES≈dim, NIS≈dof, innovations white
  Status: scaffold
  ────────────────────────────────────────
  Stage: S7 SLAM-feature update (optional)
  Purpose: Keep long-lived landmarks in-state to bound drift
  Pre-conditions (inputs): Feature stochastically initialized into the state with correct cross-covariance
  Post-conditions / invariants (outputs): In-state feature P blocks stay PSD & correlated with their poses; pruning = info-preserving Schur complement
  Status: scaffold
  ────────────────────────────────────────
  Stage: S8 Zero-velocity update (optional)
  Purpose: Arrest drift while stationary via a v=0 pseudo-measurement
  Pre-conditions (inputs): Stationarity detector tuned; not firing during slow motion
  Post-conditions / invariants (outputs): Prevents feature-starved drift while static; detector doesn't false-positive in slow motion (a wrong v=0 injects
    bias)
  Status: scaffold
  ────────────────────────────────────────
  Stage: S9 Marginalization / clone management
  Purpose: Slide the window — drop the oldest clone, preserving information
  Pre-conditions (inputs): Clone selected by policy (oldest, or two-way keyframe/non-keyframe)
  Post-conditions / invariants (outputs): P' PSD; kept-state marginal unchanged by removing a clone (principal-submatrix extraction); does not constrain the
  4
     gauge directions
  Status: scaffold
  ────────────────────────────────────────
  Stage: S10 Online calibration
  Purpose: Treat T_CI / intrinsics / t_d as states (or model their uncertainty in R)
  Pre-conditions (inputs): If estimated: observable only under excitation. If fixed: its true uncertainty must appear in the noise budget
  Post-conditions / invariants (outputs): Estimated calibration states converge & stay consistent. #212 candidate (measured): unmodeled calibration
    uncertainty ⇒ over-confidence (1°≈R×4)
  Status: ✅ wired

  The three master invariants that all of S0–S9 exist to make true (checked end-to-end):

  ┌────────────────────────┬───────────────────────────────────────────────┬───────────────────────────────────────────┐
  │       Invariant        │                    Meaning                    │                  Failure                  │
  ├────────────────────────┼───────────────────────────────────────────────┼───────────────────────────────────────────┤
  │ NEES ≈ dim(state)      │ reported covariance matches actual error      │ ≫ dim ⇒ over-confident (the #212 symptom) │
  ├────────────────────────┼───────────────────────────────────────────────┼───────────────────────────────────────────┤
  │ NIS ≈ dim(measurement) │ each innovation sized as predicted            │ ≫ dof ⇒ R/P understated                   │
  ├────────────────────────┼───────────────────────────────────────────────┼───────────────────────────────────────────┤
  │ Innovations white      │ no information left uncaptured frame-to-frame │ correlated ⇒ unmodeled structure          │
  └────────────────────────┴───────────────────────────────────────────────┴───────────────────────────────────────────┘

  A few notes: S0–S2/S10 are wired (runnable ./build/tools/sN_*); S3–S9 print this contract but their probes aren't built yet. S7/S8 are optional layers (not
  in the core MSCKF). The full prose version with the canonical OpenVINS/MINS derivation behind each row is in docs/arch/vio-pipeline-canonical.md; this
  registry is what --help/--list print and what keeps the tools, tests, figures, and doc from drifting.


