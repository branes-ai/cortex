---
status: complete
issue: branes-ai/cortex#528
---
# Plan: generate the site from docs/, and publish what docs/ already holds

Issue: [#528](https://github.com/branes-ai/cortex/issues/528), under the epic [architecture#3](https://github.com/branes-ai/architecture/issues/3) (one docs-site workflow for every repo, generated from `docs/`). It builds on [docs-kit](https://github.com/branes-ai/docs-kit), the shared generator ([architecture#4](https://github.com/branes-ai/architecture/issues/4)), and its MDX pages ([architecture#5](https://github.com/branes-ai/architecture/issues/5)).

## Goal

cortex has one set of documents. The site is generated from `docs/`, like anatomy's, racing-drone's, reflex's and synapse's: the documents we write and review live in `docs/`, where GitHub renders them, and a broken relative link or anchor fails the PR.

## Two PRs

Today the site's 55 pages live in `docs-site/src/content/docs/`, and `docs/` holds 49 other documents: the records (`adr/`, `plan/`), the session logs, and engineering notes and analyses (`arch/`, `assessments/`, `benchmark/`, `dfm/`, `math/`, `performance-engineering/`) that the site doesn't publish. Moving the pages and deciding what becomes of each note are different reviews, so they are two PRs.

### 1. The move: every route kept

Implemented in [#541](https://github.com/branes-ai/cortex/pull/541).

- **The pages move to `docs/`** at the same paths, so every route stays. `docs/index.mdx` is the landing page.
  - The MDX pages import the site's components and data through docs-kit's `@site/` alias (the site's `src/`) instead of `../../../` paths.
  - Links written as routes (`/vio/overview/`, `./getting-started/introduction/`) become relative links to the documents, so docs-kit checks them.
- **`docs/README.md` is the documentation map:** one row per sidebar group, in today's order, and within a row the pages in today's order. A sidebar label that differs from the page's title moves into the page's front matter (`sidebar.label`); the group labels ("Math Layer (E2)", …) are in the config.
- **One copy of the figures.** The SVGs the pages show were in both `docs-site/public/figures/` and `docs/assessments/figures/`, where the generators write them. The pages now link to `docs/assessments/figures/`, which takes the two folders only `public/` had (`s4/`, `s5/`), and docs-kit publishes the ones linked.
- **Not published yet:** `arch/`, `assessments/`, `benchmark/`, `dfm/` and `performance-engineering/` (`unpublished` in the config), until the second PR. `docs/math/concepts.md` is already in a published folder, so it is listed in the Math row. **Never published:** `sessions/`, the session logs, as on synapse. **Records:** `adr/` and `plan/`, on the [architecture site](https://branes-ai.github.io/architecture/cortex/), with a redirect from each old URL.
- **The site code stays in `docs-site/`:** `Scene3D`, `TriangulationScene`, `Issue`, `data/benchmarks.json` and the `gen-*.mjs` figure, benchmark and video scripts, and the scene data in `public/data/`.
- **`docs-site/docs-kit.config.mjs`** holds what docs-kit doesn't default: the group labels, the perception-blue accent, `math: true`, `src/styles/custom.css`, and the API reference with the doxygen-awesome theme (`api.doxyfile`, new in docs-kit `v0.3.0`; it replaces the `Doxyfile`).
- **The deploy is docs-kit's shared workflow;** `deploy-docs.yml` keeps the triggers and adds `docs/**`. **CI** gets the `Docs site` job.

### 2. The merge: one set of documents

For each document in `arch/`, `assessments/` and the single-document folders, one decision (decided on #528, 2026-10-31). No published page was absorbed, so no route redirects.

**Published as pages:**

| Document | Now |
|---|---|
| `arch/vio-pipeline-canonical.md` | [The Canonical VIO Pipeline](../../pipeline/canonical-pipeline.md): the S0–S10 contracts the code cites (`§S0`–`§S10`). Its dated status sections (the June divergence ranking, which probes were wired) point to [Engineering Status](../../pipeline/engineering-status.mdx) instead; the code's references are repointed. |
| `assessments/vio-pipeline-howto.md` | [Running the VIO Pipeline Demo](../../pipeline/running-the-demo.md), with the tool's current flags. |
| `assessments/covariance-feasibility-safeguards.md` | [Keeping the Covariance Physically Feasible](../../debugging/covariance-safeguards.md), after Mixed Precision on MTL5, which points to it. |

**Merged into pages, and deleted:**

| Document | Into |
|---|---|
| `arch/stage-benches.md` | [Stage Benches](../../debugging/stage-benches.mdx): its developer reference (layout, bench catalogue, fixtures, the bench contract, file formats). The benches' comments point there. |
| `assessments/covariance-architecture.md` | [MSCKF Backend](../../vio/msckf-backend.md): the square-root covariance form (#187). |
| `assessments/filter-consistency-nees-nis.md` | [Reading the Metrics](../../pipeline/metrics.mdx): the gate vs the run-average test, the code map, the VIO subtleties, reading the verdict, references. |
| `assessments/current-pipeline.md` | The [S0](../../pipeline/stages/s0-sensor-models.mdx) and [S10](../../pipeline/stages/s10-online-calibration.mdx) stage pages: how to run their probes. |
| `assessments/vio-diagnostic-methodology.md` | [Diagnostic Methodology](../../pipeline/diagnostic-methodology.mdx): the Sim3-vs-SE3 ATE heuristic; the rest was already there or in Invariants. |
| `benchmark/vio.md`, `assessments/intelligency-per-watt.md` | [Intelligence per Watt](../../benchmarks/intelligence-per-watt.md): the dataset, progress and fallback notes, and two future-work lines. |

**Kept unpublished, as working records:**

- `arch/`: `cortex-repo.md` (the design conversation the ADRs were distilled from; the ADRs, not it, decide when the code is ambiguous), `phase0-foundation/` and `phase1-design/` (summarized by the phase pages), `visual-inertial-odometry.md` (offline background; it recommends libraries cortex chose against).
- `assessments/`: `3d-viz-options.md` (the record of the 3D viewer's tooling choice), `vio-consistency-sweep-diagnosis.md` (the raw sweep data; marked superseded), the `figures/` the pages show.
- Notes outside the site's subject: `arch/flight-controller-analysis.md`, `assessments/ai-coding-assistant-failures.md`, `assessments/embodied-ai-workload-analysis.md`, `dfm/product-engineering.md`, `performance-engineering/vio-pipeline-parallelization.md`. `assessments/workloads.md`, an earlier draft of the workload analysis, is deleted.

**Moved to `sessions/`:** three session transcripts filed as notes, now dated session logs: `2026-06-03-vio-euroc-troubleshooting.md`, `2026-06-04-vio-troubleshooting-fej.md` (the FEJ measurements) and `2026-06-05-vio-first-principles.md`.

**Fixed on the way:** four pages placed the cxx bridge in `core/src/lib.rs` (it is `core/src/bridge.rs`); Stage Benches called the shipped S10 bench "in review"; the S10 page contradicted itself on calibration states; Repository Layout called `cortex-repo.md` the source of truth.

## Acceptance

1. `docs-site/src/content/` has no committed content; every page comes from `docs/`.
2. Every route on today's site still exists, or redirects to the page that absorbed it.
3. Every document in `docs/` is either a page or in a folder listed as unpublished or as records.
4. Figures, 3D scenes and benchmark data render on the deployed site.
5. A broken relative link in `docs/` fails the PR.

The first PR met 1, 4 and 5, and 2 and 3 with today's routes and the unpublished folders above; the second completes 2 and 3. All five were met on 2026-10-31.

## Departures

- **Two figures removed.** The pipeline overview's coordinate-frames diagram and S1's alignment diagram were linked in June but never committed, so the live pages showed a broken image. Their alt text describes them; drawing them is [#540](https://github.com/branes-ai/cortex/issues/540).
- **Light mode works.** cortex's slate backgrounds and grays applied in light mode too, so the light theme stayed dark. They now apply in dark mode only, and light mode is the kit's, with cortex's accent.
- **The sidebar:**
  - "3D Path & Pose Viewer" (`vio/scene3d`) is listed under Visual-Inertial Odometry, its folder, instead of VIO Pipeline. docs-kit builds a group from one folder; the route is unchanged.
  - The "Plans and decisions" link moves to the group docs-kit gives it, and the documentation map and `math/concepts.md` are added.
  - "Maintaining These Docs" comes before the API reference, which docs-kit lists last.
- **Figure URLs:** the figures are served from the site's `/_astro/` assets, as Astro optimizes them, instead of `/figures/`.
- **A favicon:** the site had none (every page asked for a missing `favicon.svg`). It is the hero's tile grid.
