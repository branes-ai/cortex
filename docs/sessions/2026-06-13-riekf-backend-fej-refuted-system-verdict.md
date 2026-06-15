# Session — 2026-06-13 → 2026-06-15 — R-IEKF backend, FEJ refuted, system-level verdict

> Session arc: built the **Right-Invariant EKF (R-IEKF) backend** end to end and
> proved its observability fix at the unit level; **measured and refuted** the
> surgical FEJ alternative; published the **camera-calibration / projection-Jacobian
> docs** (KaTeX + an interactive 3D triangulation scene); **triaged the Phase-3
> milestone** (the vio_euroc chase had spawned a thicket of issues); and drove the
> invariant backend through the validated front end to get the **system-level NEES
> verdict** — which localized a real continuous-update bug (#366).
> PRs landed: **9** (#353, #354, #356, #357, #358, #360, #362, #364, #367).
> Cortex released through **v0.50.3** via release-please.
> Pair: Theo Omtzigt (Ravenwater) + Claude Opus 4.8.

The through-line, as all session: **measure, don't assume.** Every claim was put on
a number — and the numbers refuted two plausible fixes and confirmed a third only at
the unit level.

---

## What was accomplished

### Arc 1 — The R-IEKF backend, built and unit-proven (#347 Phase B)

The #212 over-confidence was localized (prior session) to a **yaw observability
leak**: the body-frame EKF's Jacobians depend on the estimate, so re-linearizing at
the drifting estimate fabricates information along the unobservable gauge. The cure
(Barrau & Bonnabel): a right-invariant error on SE2(3) makes the Jacobians
state-independent, so the gauge is preserved *by construction*.

| PR | What | Proof (measured) |
|---|---|---|
| #353 | `SE2(3)` extended-pose Lie group (exp/log/adjoint, branch-cut-safe) | round-trips to machine-ε for float/posit |
| #354 | invariant IMU propagation (state-independent nav Φ; gravity `[g]×` replaces `−R̂[a]×`) | `Φ·N = N` (gauge preserved) identically at 3 attitudes |
| #356 | invariant camera measurement update | the **shipped** `H` annihilates the 4-DoF gauge, `H·N = 0` < 1e-12 |
| #357 | `MsckfInvariantBackend` (joint propagation + cloning + update) | the assembled, marginalized joint update annihilates the gauge across nav + all clones |

Each piece is unit-proven. The observability mechanism that drove #212 is eliminated
by construction — **no FEJ**.

### Arc 2 — FEJ implemented, measured, and refuted

A surgical First-Estimates-Jacobians variant (anchor the camera-update Jacobians at
the first estimate) was implemented exactly as directed. It **passed the unit metric**
(yaw leak driven to < 1e-15) but **failed end to end**: the body-frame backend's
pose-NEES went from **42.6 → 77.7** (nearly doubled), and the project-only variant to
**126.7**. Partial measurement-only FEJ is inconsistent with the still-live
propagation linearization, so it makes consistency *worse*. **Discarded.** The
durable, behaviour-neutral part was split out (#358): the observability gauge check
now drives the **production** `CameraUpdater` Jacobian instead of a clean-room mirror.

### Arc 3 — Camera-calibration & projection-Jacobian docs (#362, #364)

Wired **KaTeX** into the Starlight site (`remark-math` + `rehype-katex`), then
rewrote `vio/camera-updaters` as an `.mdx` page with an **interactive Three.js
triangulation scene** (two clones, rays meeting at `p_f`, image-plane parallax dots)
plus the grounded projection-Jacobian walkthrough — symbol table (live `R/p` vs
frozen clones `R_i/p_i` vs the fixed extrinsic `R_ic/p_ic`), a worked two-view
numbers example, and Stages 1–5 of `camera_updater.hpp`. The assessment note
`docs/assessments/current-pipeline.md` carries the calibration map and points at the
page. #364 widened the scene's baseline to 2 m so the parallax reads.

### Arc 4 — Phase-3 milestone triage

The vio_euroc debugging chase had left the milestone illegible. Closed **7** issues
(#337/#348/#296 done; #339/#280 FEJ-refuted; #266/#262 superseded/low-value),
status-commented the two live epics (#347, #261), scoped the verdict as **#365**, and
re-scoped the diagnostic cluster (#268 → P1 with the Allan-variance core already
built; #265 → P2; #263 → P3). Phase-3 open issues went 15 → 8, each legible.

### Arc 5 — The system-level verdict (#365) → a localized bug (#366)

Built `InvariantVioBackend` (#367), which drives `MsckfInvariantBackend` through the
**same feature-track lifecycle as the validated `MsckfBackend::process_camera`** —
replacing the hand-rolled loop (#360) that over-fused. Driving it end to end answered
"is the R-IEKF fix real end-to-end?" with **not yet** — and localized why (filed
**#366**):

- the invariant **measurement update is harmful** in the continuous filter: pure
  propagation drifts ~15 m, with the update ~160 m;
- it is *not* track management (mirrors the validated lifecycle), geometry (more
  feature transit → worse), propagation (no-update is fine), or gating (no-gate still
  diverges to 75 m / 53°);
- **less process noise → worse** divergence — the signature of structural
  inconsistency, not under-tuning;
- a **controlled perfect-sensor experiment** (only a 5.7° initial roll error) showed
  the camera update never converges it (wobbles 5.7° → 9.9° → 5.8°).

## Conventions / discoveries this session

- **The MSCKF camera measurement is camera-only** (the feature is marginalized), so
  its null space is the full **6-DoF SfM gauge — including roll/pitch**. Roll/pitch is
  observable *only* through the temporal coupling (attitude → `[g]×` → velocity →
  camera → attitude). #366 is an inconsistency in that chain.
- **NEES is parameterisation-invariant** — measure it in the filter's own SE2(3)
  right-invariant convention; no change-of-coordinates needed.
- A clean recovery metric must be **gauge-invariant** (re-triangulate the feature),
  else the correctly gauge-blind filter is penalised for leaving the gauge alone.
- **KaTeX in MDX**: `remark-math` tokenises `$…$` before MDX parses `{}` as JSX, so
  brace-LaTeX (`\mathbf{R}`) and embedded components coexist.
- The docs-site (Starlight) had **no math plugin** — the `docs/` LaTeX convention
  only rendered on GitHub until #362.

## State at end of day

- R-IEKF backend merged and unit-proven; **default still the body-frame filter**.
- FEJ refuted and discarded; only the production-driver gauge check kept.
- Camera-calibration docs published with interactive 3D.
- Milestone triaged; verdict (#365) in progress, blocked on the update bug (#366).
- `main` clean, no open PRs, no stale branches.

## Open / next

- **#366 (the blocker)** — fix the continuous-update divergence. Next concrete step:
  build the **finite-difference joint-step validator** (perturb the nav per error
  direction, augment → propagate → update, assert the linearised prediction matches)
  to pin the cross-covariance / SE2(3)-retraction convention error in the
  velocity-coupling chain. Then #365's verdict flips from WARN to a hard assertion.
- **#365** — once #366 lands, report the gauge-anchored attitude-NEES (yaw vs
  roll/pitch split) and, if consistent, flip the default backend and close the #212
  NEES over-confidence.
- Backlog (re-scoped): #268 (P1), #265 (P2), #267, #263; #213 (perf, RTF ~0.10×);
  #332 (S10 calibration — built, didn't resolve over-confidence alone).
