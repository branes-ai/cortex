---
status: in-progress
issue: branes-ai/cortex#528
---
# Plan: generate the site from docs/, and publish what docs/ already holds

Issue: [#528](https://github.com/branes-ai/cortex/issues/528), under the epic [architecture#3](https://github.com/branes-ai/architecture/issues/3) (one docs-site workflow for every repo, generated from `docs/`). It builds on [docs-kit](https://github.com/branes-ai/docs-kit), the shared generator ([architecture#4](https://github.com/branes-ai/architecture/issues/4)), and its MDX pages ([architecture#5](https://github.com/branes-ai/architecture/issues/5)).

## Goal

cortex has one set of documents. The site is generated from `docs/`, like anatomy's, racing-drone's, reflex's and synapse's: the documents we write and review live in `docs/`, where GitHub renders them, and a broken relative link or anchor fails the PR.

## Two PRs

Today the site's 55 pages live in `docs-site/src/content/docs/`, and `docs/` holds 49 other documents: the records (`adr/`, `plan/`), the session logs, and engineering notes and analyses (`arch/`, `assessments/`, `benchmark/`, `dfm/`, `math/`, `performance-engineering/`) that the site doesn't publish. Moving the pages and deciding what becomes of each note are different reviews, so they are two PRs.

### 1. The move: every route kept

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

For each document in `arch/`, `assessments/` and the single-document folders, one decision: merge it into a page that covers its subject, publish it as a page of its own, or leave it unpublished as a working record. Where both sets cover a subject (`vio`, `stage-benches`, the architecture), they become one document, and a page that is absorbed redirects to the one that absorbed it. The second PR lists the decisions in this plan.

## Acceptance

1. `docs-site/src/content/` has no committed content; every page comes from `docs/`.
2. Every route on today's site still exists, or redirects to the page that absorbed it.
3. Every document in `docs/` is either a page or in a folder listed as unpublished or as records.
4. Figures, 3D scenes and benchmark data render on the deployed site.
5. A broken relative link in `docs/` fails the PR.

The first PR meets 1, 4 and 5, and 2 and 3 with today's routes and the unpublished folders above; the second completes 2 and 3.

## Departures

- **Two figures removed.** The pipeline overview's coordinate-frames diagram and S1's alignment diagram were linked in June but never committed, so the live pages showed a broken image. Their alt text describes them; drawing them is [#540](https://github.com/branes-ai/cortex/issues/540).
- **Light mode works.** cortex's slate backgrounds and grays applied in light mode too, so the light theme stayed dark. They now apply in dark mode only, and light mode is the kit's, with cortex's accent.
- **The sidebar:**
  - "3D Path & Pose Viewer" (`vio/scene3d`) is listed under Visual-Inertial Odometry, its folder, instead of VIO Pipeline. docs-kit builds a group from one folder; the route is unchanged.
  - The "Plans and decisions" link moves to the group docs-kit gives it, and the documentation map and `math/concepts.md` are added.
  - "Maintaining These Docs" comes before the API reference, which docs-kit lists last.
- **Figure URLs:** the figures are served from the site's `/_astro/` assets, as Astro optimizes them, instead of `/figures/`.
- **A favicon:** the site had none (every page asked for a missing `favicon.svg`). It is the hero's tile grid.
