# Session — 2026-06-05 — 3D Visualization + VIO Pipeline Docs Section

> Session arc: from finishing the depth-coloured scene overlay (#300) through a
> two-tier **3D visualization environment** (#302, #303) and a complete,
> instrument-backed **VIO Pipeline documentation section** (#305–#316) that
> publishes the #212 over-confidence analysis as Starlight pages — six data-backed
> stage pages, each measured by its own stage probe.
> Total PRs landed: **10** (#300, #302, #303, #305, #307, #309, #311, #313, #315,
> #316). Cortex released through **v0.40.0** via release-please.
> Pair: Theo Omtzigt (Ravenwater) + Claude Opus 4.8 (1M context).

---

## What was accomplished

### Arc 1 — A 3D visualization environment (#300 → #303)

The goal: visualize drone paths and pose, with videos for educational content and
dissemination. After a requirement analysis (`docs/assessments/3d-viz-options.md`),
we selected a **two-tier** approach — **Three.js** for publication/web,
**Rerun** for internal debug (Phase 2, deferred) — and built tier 1.

| PR | What | Release |
|---|---|---|
| #300 | depth-coloured scene overlay + `vio_pipeline` how-to guide | v0.34.0 |
| #302 | 3D path/pose viewer — **Phase 0** (per-frame position covariance in `run.jsonl`) + **Phase 1** (interactive Three.js viewer) + offline-mp4 render | v0.35.0–v0.36.0 |
| #303 | landmark cloud + camera frustum in the 3D viewer | v0.36.0 |

- **Phase 0** — `vio_pipeline` now emits the 3×3 world-position covariance (`pcov`)
  on every `est` record. This is what makes the over-confidence *visible*: on the
  sample drone run the reported σ collapses to ~13 cm while true error exceeds 2 m.
- **Phase 1** — `docs-site/src/components/scene3d.js` (viewer) + `scene3d-math.js`
  (three-free eigen-decomposition, unit-tested headlessly) + `Scene3D.astro`
  (vanilla island: orbit + scrub + play + HUD). Renders GT/est trajectories,
  moving pose triads, the **3σ covariance ellipsoid**, the landmark cloud, and the
  camera frustum. The HUD flags over-confidence when the true pose escapes the
  ellipsoid.
- **Offline mp4** — `gen-scene3d-video.mjs` drives the *same* viewer in headless
  chromium (swiftshader WebGL) via `scene3d-render.html`, screenshots each frame
  → ffmpeg. Wired as `scripts/vio_scene_video.sh --3d`. The video matches the web
  view exactly because it is the same module.
- Deploys to `branes-ai.github.io/cortex/vio/scene3d/`.

### Arc 2 — The VIO Pipeline documentation section (#305 → #316)

Published the #212 over-confidence analysis — until now reachable only via raw
GitHub links to `docs/assessments` + `docs/arch/vio-pipeline-canonical.md` — as a
proper Starlight section under `docs-site/src/content/docs/pipeline/`. Built
stage-by-stage, each page **backed by its stage probe's measured numbers**.

| PR | Page(s) | Verdict / content | Release |
|---|---|---|---|
| #305 | Overview · Reading the Metrics · **S0** · Engineering Status | section scaffold + issue-ref policy + S0 (camera contract HOLDS) | v0.37.0 |
| #307 | **S10** Online Calibration | **leading** — unmodeled calibration ≈ the R×4 component | v0.38.0 |
| #309 | **S6** MSCKF Update · Consistency Analysis | S6 algebra textbook-correct (defect upstream); the worked Q/R sweep diagnosis | v0.39.0 |
| #311 | **S2** IMU Propagation | **cleared** — diagonal-Q drops 0.35% of Q; propagation NEES consistent | v0.39.4 |
| #313 | **S5** Feature Triangulation (+ probe) | **open** — no soft parallax gate; 0.5° features (σ ~90%) admitted at full weight | v0.40.0 |
| #315 | **S1** Initialization | **sound algorithm, suspect seed** — isotropic `P₀`, yaw under-inflated | v0.40.0 |
| #316 | Diagnostic Methodology | the method primer — why ATE-only debugging fails; migrated, brought current | v0.40.0 |

End-of-day section structure (sidebar group "VIO Pipeline"):
**Overview → Reading the Metrics → Diagnostic Methodology → S0/S1/S2/S5/S6/S10 →
Consistency Analysis → 3D Path & Pose Viewer → Engineering Status.**

Six fully-written, data-backed stage pages spanning the full verdict spectrum of
the #212 hunt: **S0** holds · **S1** sound-but-seed-suspect · **S2** cleared ·
**S5** gate-missing · **S6** algebra-correct · **S10** leading.

---

## Conventions established this session

1. **Issue-reference policy (with the user, then enforced):** *inline the
   substance; GitHub issue numbers are single-sourced to the Engineering Status
   page* via an `<Issue>` component (`src/components/Issue.astro`). Other pages link
   to Engineering Status, never embed `#NNN`. CodeRabbit holds the line on this.
2. **Stage-page workflow:** run the wired stage probe
   (`tools/src/sN_*.cpp` → `build/stage_probes/SN/*.csv`) for **real** numbers,
   regenerate figures via `gen-sensor-model-figures.mjs` into `public/figures/sN/`,
   then write from the canonical contract + measured data. Numbers in prose always
   match the figures because both come from the same probe run.
3. **Migrate brought-current, never verbatim.** The consistency doc's "FEJ is the
   fix" conclusion was corrected with the refutation result; the methodology doc's
   "tools we should build" was reframed as *realized* (a table mapping each failure
   class to the stage probe that now measures it).
4. **Headless verification before commit.** Every page is built with
   `DEPLOY_TARGET=github-pages` and loaded in headless chromium to confirm figures
   render and there are **zero 4xx** before the PR goes up.

---

## Mid-flight discoveries & fixes

- **The S5 probe was a scaffold, not wired.** Unlike S0/S2/S6/S10, `s5_triangulation`
  only printed the contract. Built `eval/triangulation_probe.hpp` from scratch — a
  two-view parallax sweep driving the **shipped** `CameraUpdater::triangulate`, with
  **Monte-Carlo** depth-σ (the honest uncertainty) — and exposed `triangulate`
  (was private) so the probe tests real code. Camera/msckf tests still pass 8/8.
- **The S2 probe sharpened the consistency story.** Propagation-only NEES is ~6.4
  (consistent) at the shipped Q, so the EuRoC "Q×10 helps" is *compensating
  inflation* masking the structural error — **not** a propagation defect. Reconciled
  the Consistency Analysis / Engineering Status wording accordingly.
- **The base-path rehype rewrite only touched `<a href>`.** Embedded figures
  (`<img src>`) would have 404'd under `/cortex/`. Extended the `astro.config.mjs`
  rewrite to images.
- **Recurring CI gates re-confirmed:** commitlint `subject-case` rejected `docs(site):
  VIO …` (uppercase "VIO") and the earlier `3D …`; clang-format 18 column/comment
  rules; CodeRabbit nits on every C++ PR (ffmpeg-spawn timeout, `noreferrer`,
  gate-sentinel honesty, persisting sweep metadata). All addressed before merge.
- **Reading-the-Metrics defined the NEES/NIS *concepts* but never expanded the
  acronyms** — a reader on the page couldn't find the definition. Added the
  expansions (Normalized Estimation/Innovation Error Squared) where the section
  introduces them.

---

## State at end of day

- VIO Pipeline section live with **6 data-backed stage pages** + methodology +
  metrics primer + consistency analysis + 3D viewer + engineering-status index.
- The 3D viewer (interactive web + offline mp4) ships the position-covariance
  ellipsoid — the over-confidence made visible.
- All 10 PRs merged clean; release-please cut through **v0.40.0**.
- New memory: `cortex-vio-pipeline-docs-section`, `cortex-3d-viz-program`.

## Open / next

- **3D viz Phase 2 — Rerun** instrumentation (the internal-debug tier).
- Remaining stage pages: **S3** (cloning), **S4** (frontend), **S7–S9**
  (SLAM-feature / zero-velocity / marginalization) as their probes land.
- Migrate the EuRoC-troubleshooting + failure-mode-taxonomy assessment docs.
