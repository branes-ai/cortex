// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import rehypeRewrite from 'rehype-rewrite';

// Deployment configuration.
//   For GitHub Pages: DEPLOY_TARGET=github-pages (sets the project base path).
//   For local dev / other hosts: no env vars needed.
const isGitHubPages = process.env.DEPLOY_TARGET === 'github-pages';
const site = isGitHubPages
  ? 'https://branes-ai.github.io'
  : (process.env.SITE_URL || 'http://localhost:4321');
const base = isGitHubPages ? '/cortex/' : '/';

// The Doxygen C++ API reference is generated into public/api/ and served
// at <base>api/ (so /cortex/api/ on Pages, /api/ in local dev).
const apiHref = base + 'api/';

// Rehype plugin to rewrite absolute internal links with the base path so
// authored `/architecture/...` links resolve under /cortex/ on Pages.
const rehypeBaseLinks = isGitHubPages ? [
  [
    rehypeRewrite,
    {
      rewrite: (node) => {
        if (node.type === 'element' && node.tagName === 'a' && node.properties?.href) {
          const href = node.properties.href;
          if (typeof href === 'string' && href.startsWith('/') && !href.startsWith('/cortex')) {
            node.properties.href = '/cortex' + href;
          }
        }
      },
    },
  ],
] : [];

// https://astro.build/config
export default defineConfig({
  server: { host: '0.0.0.0' },
  site,
  base,
  trailingSlash: 'always',
  markdown: {
    rehypePlugins: rehypeBaseLinks,
  },
  integrations: [
    starlight({
      title: 'Cortex',
      description: 'Core Embodied AI Framework for perception — a layered Rust + C++20 VIO/SLAM stack for spatial-dataflow accelerators.',
      social: [
        { icon: 'github', label: 'GitHub', href: 'https://github.com/branes-ai/cortex' },
      ],
      editLink: {
        baseUrl: 'https://github.com/branes-ai/cortex/edit/main/docs-site/',
      },
      sidebar: [
        {
          label: 'Getting Started',
          items: [
            { label: 'Introduction', slug: 'getting-started/introduction' },
            { label: 'Build & Run', slug: 'getting-started/build-and-run' },
            { label: 'Repository Layout', slug: 'getting-started/repository-layout' },
          ],
        },
        {
          label: 'Architecture',
          items: [
            { label: 'Overview & Topology', slug: 'architecture/overview' },
            { label: 'Layering Invariants', slug: 'architecture/layering' },
            { label: 'The KPU Spacetime Constraint', slug: 'architecture/kpu-spacetime' },
            { label: 'Resource Manager (Rust)', slug: 'architecture/resource-manager' },
          ],
        },
        {
          label: 'Math Layer (E2)',
          items: [
            { label: 'Overview', slug: 'math/overview' },
            { label: 'Non-linear Least Squares', slug: 'math/nls' },
            { label: 'Lie Groups (SO3/SE3/Sim3)', slug: 'math/lie-groups' },
            { label: 'Camera Models', slug: 'math/cameras' },
          ],
        },
        {
          label: 'Visual-Inertial Odometry (E3)',
          items: [
            { label: 'Overview', slug: 'vio/overview' },
            { label: 'Visual Front End', slug: 'vio/front-end' },
            { label: 'IMU Preintegration & Init', slug: 'vio/imu' },
            { label: 'MSCKF State Machinery', slug: 'vio/msckf-state' },
            { label: 'Camera Updaters', slug: 'vio/camera-updaters' },
            { label: 'MSCKF Backend', slug: 'vio/msckf-backend' },
            { label: 'VioEstimator API', slug: 'vio/vio-estimator' },
          ],
        },
        {
          label: 'Benchmarks & Validation',
          items: [
            { label: 'Trajectory Accuracy (ATE/RPE)', slug: 'benchmarks/accuracy' },
            { label: 'Latency Budget', slug: 'benchmarks/latency' },
            { label: 'Coverage & CI Gates', slug: 'benchmarks/ci' },
          ],
        },
        {
          label: 'Algorithms & Provenance',
          items: [
            { label: 'Clean-room Citations', slug: 'algorithms/citations' },
          ],
        },
        {
          label: 'Roadmap & Phases',
          items: [
            { label: 'Roadmap', slug: 'phases/roadmap' },
            { label: 'Phase 0 — Foundation', slug: 'phases/phase-0' },
            { label: 'Phase 1 — Rust Core', slug: 'phases/phase-1' },
            { label: 'Phase 2 — Math', slug: 'phases/phase-2' },
            { label: 'Phase 3 — VIO', slug: 'phases/phase-3' },
          ],
        },
        {
          label: 'API Reference',
          items: [
            { label: 'C++ API (Doxygen)', link: apiHref, attrs: { target: '_blank' } },
          ],
        },
        {
          label: 'Maintaining These Docs',
          items: [
            { label: 'Documentation Process', slug: 'meta/documentation-process' },
          ],
        },
      ],
      customCss: ['./src/styles/custom.css'],
    }),
  ],
});
