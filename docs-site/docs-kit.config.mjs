// The docs site's configuration: everything docs-kit does not default.
// https://github.com/branes-ai/docs-kit
export default {
  repo: 'cortex',
  title: 'Cortex',
  description: 'Core Embodied AI Framework for perception — a layered Rust + C++20 VIO/SLAM stack for spatial-dataflow accelerators.',
  groupLabels: {
    math: 'Math Layer (E2)',
    vio: 'Visual-Inertial Odometry (E3)',
    pipeline: 'VIO Pipeline',
    debugging: 'Stage-Bench Debugging (#444)',
    benchmarks: 'Benchmarks & Validation',
    algorithms: 'Algorithms & Provenance',
    phases: 'Roadmap & Phases',
    meta: 'Maintaining These Docs',
  },
  // Not published yet: the engineering notes and analyses, until each is
  // merged into the site or published as its own folder; and the session logs.
  unpublished: ['arch', 'assessments', 'benchmark', 'dfm', 'performance-engineering', 'sessions'],
  accent: {
    dark: { low: '#14233a', accent: '#2f80ed', high: '#9ec5fe' },
    light: { low: '#c8dcfb', accent: '#1d63d8', high: '#11305f' },
  },
  // LaTeX math (`$…$`, `$$…$$`), rendered with KaTeX at build time.
  math: true,
  customCss: ['./src/styles/custom.css'],
  api: {
    name: 'Cortex C++ API',
    brief: 'branes::math / branes::sdk / branes::cv — header-only perception primitives',
    input: ['math/include', 'sdk/include', 'cv/include'],
    // The doxygen-awesome theme, vendored in docs-site/doxygen-awesome/.
    doxyfile: {
      HTML_EXTRA_STYLESHEET: ['doxygen-awesome/doxygen-awesome.css', 'doxygen-awesome/doxygen-awesome-sidebar-only.css'],
      HTML_COLORSTYLE: 'LIGHT',
    },
  },
};
