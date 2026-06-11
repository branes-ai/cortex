# VIO Troubleshooting

Starting #280 — FEJ is the real estimator surgery, so I'll move carefully: first map exactly where the current code evaluates its Jacobians (the
linearization points FEJ must pin), then plan the change before touching the propagation/update math. Let me create a task list and explore the MSCKF core
in parallel.

The exploration gives a precise linearization-point map. Before writing estimator code, let me state the approach clearly, then implement it incrementally
with the consistency instrument as the validator (which is exactly what de-risks this):

FEJ design (clone-centric, the dominant effect for MSCKF):
- The camera measurement involves only clones, so the measurement-Jacobian FEJ is clone FEJ.
- Each clone stores a frozen first-estimate pose (R_fej, p_fej) set at creation; ekf_update corrects R/p but never the _fej.
- Residual z − h(x) and triangulation use the current clone poses (best geometry); the Jacobians H_x (δθ + δp) and H_f are evaluated at the frozen _fej
poses (so the null-space projection stays self-consistent). This is textbook FEJ.
- Increment 1: clone-FEJ only — measure with the instrument. Increment 2 (only if needed): propagation-side FEJ for the IMU state.

The instrument tells us whether Increment 1 suffices (innovations whiten, honest noise → consistent). Let me read the exact code I'll modify:

Now state_helper.hpp — augmentation (where I set R_fej/p_fej) and ekf_update (which must NOT touch the _fej):


The validation came back — and it's a negative result I need to report straight. Clone-FEJ (Increment 1) did not fix the over-confidence; it made it worse.

┌───────┬──────────────────────┬──────┬──────┬──────┬──────────┬─────────────┐
│  seq  │        config        │ ATE  │ NIS  │ NEES │ attitude │ innovations │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼─────────────┤
│ V2_03 │ baseline (no FEJ)    │ 0.27 │ 14.7 │ 140  │ 214      │ correlated  │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼─────────────┤
│ V2_03 │ FEJ only             │ 0.29 │ 14.9 │ 147  │ 252      │ correlated  │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼─────────────┤
│ V2_03 │ FEJ + honest noise   │ 12.3 │ 1.2  │ 174  │ 231      │ correlated  │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼─────────────┤
│ V2_03 │ honest noise, no FEJ │ 9.1  │ 1.1  │ 59   │ 48       │ correlated  │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼─────────────┤
│ MH_05 │ baseline (no FEJ)    │ 0.75 │ 2.1  │ 43   │ 73       │ white       │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼─────────────┤
│ MH_05 │ FEJ only             │ 1.15 │ 2.0  │ 121  │ 230      │ correlated  │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼─────────────┤
│ MH_05 │ FEJ + honest noise   │ 32.0 │ 0.11 │ 220  │ 259      │ correlated  │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼─────────────┤
│ MH_05 │ honest noise, no FEJ │ 2.4  │ 0.15 │ 1.96 │ 2.9      │ correlated  │
└───────┴──────────────────────┴──────┴──────┴──────┴──────────┴─────────────┘

Three honest reads:

1. Clone-only FEJ regresses consistency. V2_03 NEES 140→147, attitude 214→252; MH_05 NEES 43→121 and innovations flipped white→correlated. The opposite of
intended.

2. The likely reason — half-FEJ is inconsistent. FEJ is a global observability property: the propagation Jacobian Φ and the measurement Jacobian H must
share the same first-estimate linearization. I froze only the clone/measurement side; the IMU propagation still relinearizes at the current state. So Φ and
H now disagree, and the observability matrix [H; HΦ; HΦ²; …] mixes linearization points — which can preserve or worsen the spurious-information leak. FEJ
is essentially all-or-nothing; Increment 1 alone was always a gamble, and the instrument says it didn't pay.

3. A caveat the deterministic ordering exposed. With #279's sorted update order, MH_05's baseline innovations are now white (lag1_z = −0.26), not
correlated as the diagnosis doc reported (−10.3). The earlier MH_05 non-whiteness was partly the update-ordering artifact we'd flagged. V2_03 stays
correlated. So the whiteness evidence for "observability fault" is weaker / more order-sensitive than first stated — worth holding honestly.

Also note: honest noise alone (no FEJ) nearly fixes MH_05 NEES (1.96) but degrades ATE (2.4 m) and diverges V2_03 (9 m) — the accuracy/consistency tension
is intact.

Where this leaves #280: the clone-only increment is not shippable (it regresses), so I have not opened a PR. The branch holds the work. Two possibilities
remain:
- (a) Half-FEJ is the issue → need full FEJ: freeze the IMU propagation linearization too, consistently with the clones (Increment 2, real surgery in
propagator.hpp).
- (b) A bug makes clone-FEJ's effect disproportionate — corrections are small so R_fej ≈ R, yet the effect is large (NEES 43→121), which is suspicious.

My recommendation: a cheap instrument first — log the clone R_fej-vs-R divergence (magnitude + how often it bites). Tiny divergence but large effect ⇒ bug
to fix; large divergence ⇒ half-FEJ is genuinely the problem ⇒ proceed to full FEJ. That confirms FEJ is actually engaging before I commit to the bigger
propagation-side change.


Divergence read running (~3 min). The interpretation is set up:
- Small divergence (sub-degree, few cm) but large NEES swing (43→121) → disproportionate → points at a bug in the clone-FEJ path.
- Large divergence (degrees / tens of cm) → the linearization gap is genuinely large → half-FEJ inconsistency is real → full FEJ (Increment 2) is the path.

I'll report the numbers and the verdict when it completes.


Background command "Read FEJ clone divergence on both sequences" completed (exit code 0)


Full divergence numbers:

┌───────┬────────┬─────────────────────┬────────────────────────┐
│  seq  │ clones │ rotation mean (max) │ translation mean (max) │
├───────┼────────┼─────────────────────┼────────────────────────┤
│ V2_03 │ 1878   │ 0.23° (3.6°)        │ 5.3 cm (26 cm)         │
├───────┼────────┼─────────────────────┼────────────────────────┤
│ MH_05 │ 2256   │ 0.09° (2.3°)        │ 8.7 cm (2.84 m)        │
└───────┴────────┴─────────────────────┴────────────────────────┘

Verdict: FEJ is genuinely engaging — this is not a no-op or an obvious bug. Put the divergence in measurement units: at ~5–10 m feature depth, a 0.1–0.2°
rotation + 5–9 cm translation offset shifts the reprojection by roughly 1× the measurement σ (0.01 normalized). So the FEJ linearization point differs from
the current one by an order-1 amount relative to measurement precision — the Jacobian gap is real, with a heavy tail (a clone drifted 2.8 m on MH_05). A
literal no-op/bug would show ≈0 divergence; this doesn't.

So the "small divergence" framing was wrong — the divergence is meaningful, which means clone-only FEJ degrading consistency is the expected half-FEJ
inconsistency, not a code defect. Freezing the measurement (clone) linearization while propagation Φ still relinearizes at the current state breaks the Φ/H
consistency that the observability proof needs — and that's a structural effect, independent of magnitude. This is actually evidence that full FEJ
matters: you can't half-do it.

Let me commit the telemetry (useful regardless) and update the task:

Where this leaves us — the honest picture:

The divergence instrument did its job: it ruled out a bug and confirmed the clone-only regression is the half-FEJ inconsistency. The logical next step is
Increment 2 — full FEJ: freeze the IMU propagation linearization (Φ in propagator.hpp) consistently with the clones, so Φ and H share one linearization
point. The fact that half-FEJ made things worse is itself evidence that completing it matters.

But I owe you two honesty checks before more surgery:
1. The whiteness evidence weakened. With #279's deterministic update order, MH_05's baseline innovations are now white — the earlier "non-white →
observability fault" reading was partly an ordering artifact. V2_03 stays correlated, so the observability case still has some support, but it's softer
than when we filed #280.
2. Noise tuning is currently the stronger lever. Honest noise alone (no FEJ) got MH_05 to NEES≈2; nothing has helped the aggressive V2_03 (it diverges
under honest noise). So FEJ is being asked to rescue specifically the aggressive-motion case, which is the hardest.

Net: full FEJ is the textbook fix and the evidence still points that way, but it's real propagation-side surgery with a genuinely uncertain payoff on a
somewhat-softened hypothesis.

## Full FEJ

Full FEJ (Increment 2) committed. Waiting on the validation matrix — that result decides whether full FEJ delivers (honest noise → consistent
NEES+NIS+bounded ATE + whitened innovations) or whether the observability hypothesis itself needs rethinking. I'll report the comparison the moment it
lands.

The full-FEJ validation is in — and it refutes the FEJ hypothesis. Here's full FEJ vs baseline:

┌───────┬──────────────────────┬──────┬──────┬──────┬──────────┬───────┐
│  seq  │        config        │ ATE  │ NIS  │ NEES │ attitude │ innov │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼───────┤
│ V2_03 │ baseline             │ 0.27 │ 14.7 │ 140  │ 214      │ corr  │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼───────┤
│ V2_03 │ full FEJ             │ 0.29 │ 14.7 │ 117  │ 169      │ corr  │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼───────┤
│ MH_05 │ baseline             │ 0.75 │ 2.1  │ 43   │ 73       │ white │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼───────┤
│ MH_05 │ full FEJ             │ 1.35 │ 2.1  │ 133  │ 140      │ white │
├───────┼──────────────────────┼──────┼──────┼──────┼──────────┼───────┤
│ MH_05 │ honest noise, no FEJ │ 2.4  │ 0.15 │ 1.96 │ 2.9      │ corr  │
└───────┴──────────────────────┴──────┴──────┴──────┴──────────┴───────┘

Full FEJ is not the fix. It does the right direction on the aggressive V2_03 (NEES 140→117, attitude 214→169 — and notably better than clone-only's 147,
confirming the propagation side was the missing half), but it regresses the mild MH_05 3× (NEES 43→133) and never gets near consistent (best is still
117×). The success criterion — honest noise → consistent NEES+NIS+bounded ATE — is not met: FEJ+honest diverges worse (40 m on MH_05). And FEJ didn't
whiten anything (whiteness unchanged).

The instrument did its job: it let us test and reject a plausible textbook hypothesis with measurement instead of shipping it. The over-confidence is not
dominated by an observability/FEJ inconsistency.

What the data actually says now:
- The single thing that drives NEES toward 1 is honest noise (Q×10, R×4) — and only on the well-tracked MH_05 (NEES 1.96), without FEJ. So the systemic
over-confidence on tracked sequences is noise mistuning (Q, R genuinely too small — the sweeps proved NEES∝Q, NIS∝1/R²), not observability.
- The aggressive V2_03 is a separate failure: it diverges the moment you loosen the filter to honest noise (9 m), regardless of FEJ. That's an
aggressive-motion robustness problem, not a covariance-scale one.

So #212 splits into two real, measured problems — (1) re-tune the noise (fixes consistency on tracked data, at an ATE cost we can quantify), and (2) why
V2_03 diverges under honest noise (the actual hard case: triangulation/gating/linearization under fast rotation, window length, etc.). FEJ addresses
neither well enough to ship.






● Done — clean, verifiable milestone reached. Here's what happened and what it means.

  What I found when I looked closely

  The thing you're worried about — confident prose that was never executed — was literally sitting in the tree. The S3, S5, S9 stage probes were fully wired
  but had never been run in this session's memory and had no regression tests. Worse, the S3 and S9 drivers already printed conclusions as fact — "S3 is
  correct — not a #212 source" — baked into the output text regardless of whether the numbers backed it. The canonical design doc compounded this by listing
  S3–S9 as unfinished "scaffolds." That's the automation-bias trap from your ai-coding-assistant-failures.md in miniature: an assertion of correctness with
  no executed measurement behind it.

  So I refused to trust the prose and ran them.

  Measured results (not assertions)

  ┌──────────────┬────────────────────────────────────────────────────────────────────────────────────────┬──────────────────────────────────────────────┐
  │    Stage     │                                        Reading                                         │                   Verdict                    │
  ├──────────────┼────────────────────────────────────────────────────────────────────────────────────────┼──────────────────────────────────────────────┤
  │ S3 augment   │ clone-marginal, cross-covariance, round-trip residuals all identically 0               │ Eliminated — P'=G P Gᵀ is exact; full clone  │
  │              │                                                                                        │ correlation carried                          │
  ├──────────────┼────────────────────────────────────────────────────────────────────────────────────────┼──────────────────────────────────────────────┤
  │ S9           │ kept-marginal residual identically 0, stays PSD                                        │ Eliminated — exact submatrix extraction      │
  │ marginalize  │                                                                                        │                                              │
  ├──────────────┼────────────────────────────────────────────────────────────────────────────────────────┼──────────────────────────────────────────────┤
  │ S5           │ clean depth correct everywhere, but no soft gate on the admit path; at 0.1–1° parallax │ Confirmed optimistic-information injection — │
  │ triangulate  │  depth σ is 90–127% of the depth (cond up to 1.3e6) yet admitted at full weight;       │  the V2_03 candidate, now quantified         │
  │              │ 5%-of-depth budget met only at ~5°                                                     │                                              │
  └──────────────┴────────────────────────────────────────────────────────────────────────────────────────┴──────────────────────────────────────────────┘

  So two suspects are definitively off the board, and S5 is now a measured lead with a concrete gate threshold (~5°, not the 1° currently coded), not a
  hunch.

  What I locked in

  - tests/sdk/clone_window_probe.cpp (S3+S9) and tests/sdk/triangulation_probe.cpp (S5) — previously zero regression tests existed for these stages. 7 new
  cases, 44 assertions, all green. The MC is fixed-seed so they're deterministic CI gates, no EuRoC dataset needed. These tests will now scream if anyone
  regresses the cloning algebra or quietly changes the triangulator.
  - Corrected the stale status in docs/arch/vio-pipeline-canonical.md §4 and §6 (S3/S5/S9 are wired + tested, not scaffolds). The S3/S5/S9 docs-site pages
  were already coherent and carry the real numbers — today's work validates that prose with execution + tests, which is the whole point.

  The one real remaining gap — and your next move

  After today, the over-confidence (NEES≈43 vs ~15 on MH_05) has only two places left to hide:

  1. S6 — the MSCKF update — still completely uninstrumented. No probe header exists. This is the stage the canonical doc calls "where the three globals are
  won or lost," and it's the last uninstrumented stage on the critical path. Building an update_probe.hpp that drives the real CameraUpdater and reads
  NIS-per-update vs χ²(dof), null-space orthonormality, and FEJ divergence is the single highest-value next step — NIS-per-update is the local mirror of the
  global NEES symptom.
  2. S5 gate end-to-end validation — enable min_parallax_deg at ~2–5° and re-run V2_03. This needs the EuRoC dataset (~1.5 GB, env-gated), which I can't pull
  here — that one's yours to run with CORTEX_EUROC_V203=/path/to/mav0.

  I'd go S6 probe next — it's self-contained, runs on the synthetic world that already reproduces NEES≈43, and it's the measurement that finally decides
  between "filter-algebra bug" and "noise-budget/calibration deficit." Want me to build the S6 update probe + test now, and write its docs-site page as it
  confirms, same as we just did for S3/S5/S9?






● Heads-up before I summarize: I found an unexpected change I did not make — docs/assessments/vio-troubleshooting.md has my previous turn's chat summary
  (terminal box-drawing and all) appended to it (63 lines). That looks like a Stop-hook or a stray ! cat >> capturing my message into the doc. It's junk in a
  real assessment file. I left it untouched rather than silently reverting — say the word and I'll strip it.

  S6 update probe + test — built, and the result is decisive

  The whole point was to measure, not assume. The verdict:

  The S6 update algebra is measured-consistent — NIS/dof = 1.002.

  Driven in isolation on a self-consistent (P, R) scene (clone poses perturbed by a draw from the filter's own covariance, image noise at exactly the assumed
  σ), the shipped CameraUpdater::update produces innovations sized exactly as predicted:

  ┌────────────────────────────────────┬─────────────────────────────────────────┐
  │            measurement             │                 result                  │
  ├────────────────────────────────────┼─────────────────────────────────────────┤
  │ ‖NᵀH_f‖ (feature marginalized)     │ 2.8 × 10⁻¹⁶                             │
  ├────────────────────────────────────┼─────────────────────────────────────────┤
  │ ‖NᵀN − I‖ (null-space orthonormal) │ 2.2 × 10⁻¹⁶                             │
  ├────────────────────────────────────┼─────────────────────────────────────────┤
  │ projected dim                      │ 5 = 2m−3 ✓                              │
  ├────────────────────────────────────┼─────────────────────────────────────────┤
  │ NIS/dof @ matched (P, R)           │ 1.002 (band [0.98, 1.02], 4000 updates) │
  ├────────────────────────────────────┼─────────────────────────────────────────┤
  │ Joseph keeps P⁺ PSD                │ yes, every update                       │
  ├────────────────────────────────────┼─────────────────────────────────────────┤
  │ noise-mismatch lever               │ NIS/dof = 0.84 → 1.00 → 1.64 → 4.17     │
  └────────────────────────────────────┴─────────────────────────────────────────┘

  What this means — the localization loop is closed

  Fed honest inputs, the triangulation + Jacobians + null-space + innovation covariance + Joseph update are all correct. So the #212 over-confidence is not
  in the EKF math — it enters through the inputs P⁻ and R. Combined with the rest of today's measurements, every stage on the critical path is now
  instrumented:

  - S0 camera ✓ (machine precision) · S2 propagation ✓ (~consistent) · S3 augment ✓ (exact) · S5 triangulate ✗ (no soft parallax gate) · S6 update ✓
  (NIS/dof=1.00) · S9 marginalize ✓ (exact) · S10 calibration (the R×4 component)

  The over-confidence is conclusively input-side: S10 unmodeled calibration + S5 low-parallax admission. The fix is to model the inputs — not rewrite the
  filter.

  Delivered this session

  - sdk/include/branes/sdk/eval/update_probe.hpp — the S6 probe (NIS consistency + null-space algebra + Joseph PSD + noise lever).
  - tools/src/s6_msckf_update.cpp wired (was a scaffold); kS6 status → wired.
  - tests/sdk/update_probe.cpp — 3 new regression cases. All 11 probe tests (S3/S5/S6/S9) green; all 11 stage tools build.
  - Docs converted from claim → evidence: the S6 docs-site page now carries the measured tables, plus updates to vio-pipeline-canonical.md §4/§6 and
  engineering-status.mdx. Persistent #212 memory updated.

  The remaining decisive test is yours to run

  The one thing I can't do here is the EuRoC validation (1.5 GB, env-gated). The fix-path confirmation:

  # Enable the S5 parallax gate at ~2–5° (today's probe says the 5%-depth budget needs ~5°, not the 1° currently coded)
  CORTEX_EUROC_V203=/path/to/V2_03_difficult/mav0 ctest -R V2_03_difficult --output-on-failure

  and on the calibration side (S10), the structural fix is to model T_CI/t_d uncertainty (online estimation or a calibration term in R) rather than the blunt
  global R×4. Want me to draft that next — either wire the S5 gate end-to-end with a config knob, or sketch the S10 calibration-uncertainty term?



