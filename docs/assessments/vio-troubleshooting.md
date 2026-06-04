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

