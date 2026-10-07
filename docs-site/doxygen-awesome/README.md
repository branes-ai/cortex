# doxygen-awesome-css (vendored)

Theme for the Doxygen C++ API reference (issue #102).

- Upstream: <https://github.com/jothepro/doxygen-awesome-css>
- Version: **v2.5.0** (pinned; bump by replacing these files from a newer release)
- License: MIT (see `LICENSE`, kept with the files as the license requires)
- Files: `doxygen-awesome.css` + `doxygen-awesome-sidebar-only.css` (the
  sidebar-only variant, which matches `GENERATE_TREEVIEW = YES` in `../Doxyfile`)

Vendored rather than fetched so `npm run api` produces the same output locally
and in CI with no extra network step. The theme isn't published on npm.
