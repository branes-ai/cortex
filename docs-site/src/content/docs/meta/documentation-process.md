---
title: Documentation Process
description: How this site is built, published, and kept current — the create / publish / maintain workflow.
---

This page documents the documentation itself: the tooling, the publish pipeline, and
the discipline that keeps it from rotting.

## Tooling

- **[Astro](https://astro.build) + [Starlight](https://starlight.astro.build)** —
  the narrative site (everything in `docs-site/src/content/docs/`), authored in
  Markdown/MDX. Mirrors the sister repo `embodied-ai-architect`.
- **[Doxygen](https://www.doxygen.nl)** — the C++ API reference, generated from the
  `///` comments in `math/`, `sdk/`, and `cv/` headers into `public/api/`, served at
  `<base>api/` next to the prose site.

## Local development

```bash
cd docs-site
npm install            # first time
npm run api            # generate the Doxygen API into public/api/
npm run dev            # live-reloading Starlight at http://localhost:4321/
npm run build          # production build into dist/ (what CI deploys)
npm run build:full     # api + build, the full local equivalent of CI
```

The Doxygen output (`public/api/`), `node_modules/`, `dist/`, and `.astro/` are
gitignored — the API reference is generated, never committed.

## Publishing

`.github/workflows/deploy-docs.yml` builds and deploys to **GitHub Pages** at
`https://branes-ai.github.io/cortex/` on every push to `main` that touches
`docs-site/**` or the documented headers (`math|sdk|cv/include/**`). It runs
`npm ci` → `npm run api` (Doxygen) → `npm run build` (with `DEPLOY_TARGET=github-pages`)
→ `actions/deploy-pages`.

**One-time setup (a repo admin must do this once):** Settings → Pages → Build and
deployment → Source = **GitHub Actions**. Until that is set, the workflow runs but the
deploy step has nowhere to publish.

## The maintenance discipline

Documentation stays accurate only if updating it is part of shipping code, not a
separate chore:

1. **Per-PR docs rule.** A PR that adds or changes a subsystem updates its page here
   (algorithm description, and any benchmark number it moves). The PR template carries
   a checkbox for it. This is the single most important habit — it's why the site
   reflects the code instead of drifting from it.
2. **Single source of truth for numbers.** Benchmark thresholds (ATE gate, latency
   budget) live in the test headers (`FrameLatencyBudget`, the EuRoC ATE gate); the
   docs *cite* them. When a gate is ratcheted in code, the citing page is updated in
   the same PR. (A future enhancement: have CI emit a JSON artifact the page reads, so
   the numbers can't drift at all.)
3. **Per-phase retrospective.** At each epic's close, add or finalize its
   `phases/phase-N` page (paired with the `docs/sessions/` log and the project-memory
   entry). Phases 0–3 are done; Phase 4 (SLAM) gets a page when it closes.
4. **API reference is automatic.** Because Doxygen reads the headers, the C++ API page
   stays current for free — the maintenance cost is keeping the header `///` comments
   good, which review already enforces.

## Adding a page

1. Create `docs-site/src/content/docs/<section>/<slug>.md` with `title` +
   `description` frontmatter.
2. Add it to the `sidebar` in `astro.config.mjs` (or use an `autogenerate` directory
   for a fast-growing section like future tutorials).
3. Use root-absolute internal links (`/vio/overview/`); the base-path rewrite makes
   them resolve under `/cortex/` on Pages and `/` locally.
