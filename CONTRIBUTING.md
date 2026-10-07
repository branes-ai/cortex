# Contributing to cortex

cortex is the perception layer of the Branes.AI platform: a Rust Resource
Manager, header-only C++20 math and CV, the VIO/SLAM/scene-graph SDK, and the
middleware daemons that expose it. This guide covers the rules that keep
that stack coherent. Most are enforced by CI or review, and each links to the
decision record that explains it.

## Layering: what may depend on what

The point of the layering is that **algorithms never see middleware, and
middleware never sees hardware.** Each layer's dependencies are strict:

| Layer | Path | May depend on | Must NOT depend on |
|---|---|---|---|
| Resource Manager | `core/` (Rust) | OS, kernel drivers, `cxx` | C++ SDK, Zenoh, ROS 2 |
| Math | `math/` (header-only C++20) | MTL5, Universal | anything heavier |
| CV | `cv/` (header-only C++20) | `math/`, `stb_image` | OpenCV, middleware |
| Operator SDK | `sdk/` | `math/`, `cv/`, `core/` via the cxx bridge, `std::span` | Zenoh, ROS 2, yaml-cpp |
| Daemons | `daemons/` (executables) | `sdk/`, Zenoh, yaml-cpp | algorithm code of their own |
| Examples, tools | `examples/`, `tools/` | anything above | being depended on |

Rules that follow from the table:

- **Configuration is parsed only at the edge.** YAML/JSON is turned into
  typed POD structs (`VioConfig`, …) at a daemon's or application's entry
  point; SDK operators take those structs by value and never see `yaml-cpp`
  types. `examples/hello_vio` shows the pattern. See
  [ADR-0014](docs/adr/0014-middleware-agnostic-core-zenoh-daemons.md).
- **No dynamic reconfiguration.** Parameters are fixed at `configure`;
  changing them is a teardown/restart through the lifecycle
  (`Unconfigured → Inactive → Active → Teardown`). Don't add setters or
  mutex-guarded parameters on the hot path
  ([ADR-0005](docs/adr/0005-managed-lifecycle-no-dynamic-reconfiguration.md)).
- **Never branch on the build target outside `core/`.** `BUILD_TARGET_KPU`
  (SITL vs. KPU) is confined to the Rust `MemoryProvider` HAL; `sdk/`,
  `math/` and `cv/` must not `#ifdef TARGET_KPU`
  ([ADR-0016](docs/adr/0016-build-target-confined-to-rust-hal.md)).
- **Only the Resource Manager touches the KPU.** No SDK consumer or daemon
  opens `/dev/kpu`; they go through the RM's IPC
  ([ADR-0008](docs/adr/0008-host-broker-thin-client-topology.md)). The RM
  manages *allocations*; scheduling belongs to the DFA graph compiler, a
  separate repo ([ADR-0004](docs/adr/0004-rm-allocates-dfa-compiler-schedules.md)).
- **Tensors cross the FFI as `repr(C)` metadata and are never copied.** The
  C++ side wraps the pointer in `std::span` and hands it to MTL5 views
  ([ADR-0010](docs/adr/0010-repr-c-tensor-boundary.md)).
- **Don't hardcode `double`.** Math and operators are templated on the
  arithmetic type so they run in posits, cfloat, and fixed-point too
  ([ADR-0015](docs/adr/0015-type-generic-arithmetic.md)).
- **The 3D scene graph is data-oriented:** contiguous arrays indexed by
  integer node IDs, not heap nodes with `shared_ptr` children.

C++ code lives in the `branes::` namespace (`branes::math`, `branes::cv`,
`branes::sdk`); the Rust bridge is `branes::core`.

All architectural decisions are in [`docs/adr/`](docs/adr/README.md). Read the
relevant ADR before changing anything structural. If your change contradicts
one, amend or supersede the ADR in the same PR.

## Clean-room code and the no-GPL policy

cortex is MIT-licensed and must stay redistributable by OEM partners, so:

- **No GPL code for any purpose:** not as a dependency, not in tests, not as
  a build tool, not as a ground-truth oracle. LGPL/MPL/SSPL default to
  rejected ([ADR-0001](docs/adr/0001-third-party-license-compatibility.md),
  [ADR-0002](docs/adr/0002-no-gpl-clean-room-policy.md)).
- **Implement from papers and documentation.** Check a reference's license
  *before* reading its code; reading GPL source and then "rewriting" it is
  not clean-room. OpenVINS, MINS, ORB-SLAM3, DBoW3, and GPL parts of
  Sophus/g2o are off-limits as code references.
- **Cite your sources.** New algorithms add their paper(s) to the docs
  site's clean-room citations page.
- **Test against independent oracles:** synthetic worlds and published-paper
  thresholds, never another implementation's output.
- **New dependencies** must be permissively licensed, pinned, fetched via
  `cmake/deps.cmake`, and added to ADR-0001's license matrix. No vcpkg or
  Conan ([ADR-0011](docs/adr/0011-dependencies-via-fetchcontent.md)).

## Development environment

The primary environment is **Visual Studio 2022 on Windows** with CMake
presets, plus **WSL2** for Linux and the aarch64 cross-compile, driven from the
same IDE. Linux-native development works the same way. Profiling uses
cross-platform tools (Tracy, Valgrind), not MSVC-specific ones.

Requirements:

- **CMake ≥ 4.0** and Ninja. Visual Studio 2022's bundled CMake is 3.x, so
  point the IDE at a newer CMake (Tools → Options → CMake).
- **Rust 1.83.0**, pinned by `rust-toolchain.toml`; `rustup` picks it up
  automatically. Toolchain bumps are their own PRs.
- A C++20 compiler: MSVC 2022, GCC 13+, or Clang 18+.

Check and set up a machine with `scripts/bootstrap.sh` (Linux/WSL2, add
`--install-deps` to apt-install missing tools) or `scripts/bootstrap.ps1`
(Windows).

```bash
cmake --preset sitl-debug               # host SITL build (default for development)
cmake --build --preset sitl-debug
ctest --preset sitl-debug
```

| Preset | Use |
|---|---|
| `sitl-debug` / `sitl-release` | Host build, emulated KPU memory (Linux / WSL2) |
| `wsl2-sitl` | `sitl-debug` driven from Visual Studio via WSL2 |
| `msvc` (+ `msvc-debug` / `msvc-release` build presets) | Native Windows, Visual Studio generator |
| `msvc-ninja` / `msvc-ninja-debug` | Windows via Ninja + sccache (what CI runs) |
| `kpu-cross` | aarch64 cross-compile for the KPU target (needs `gcc-aarch64-linux-gnu`) |

To work on MTL5 or Universal alongside cortex, point the build at local
checkouts: `-DFETCHCONTENT_SOURCE_DIR_MTL5=… -DFETCHCONTENT_SOURCE_DIR_UNIVERSAL=…`.

## Tests

- **Drop-in tests:** a `<name>.cpp` Catch2 file in `tests/<layer>/` is built
  and registered with CTest automatically (`cmake/compile_all.cmake`). File
  stems must be globally unique. Shared helpers go in `tests/include/` as
  headers, never as `.cpp` in the globbed directories.
- **Build and test with two compilers** for C++ changes. CI runs GCC (Linux)
  and MSVC (Windows, Release and Debug); test with Clang locally too.
- **Keep Catch2 test-case names ASCII.** Unicode in `TEST_CASE` names breaks
  the MSVC CI job through CTest's codepage handling.
- **Formatting:** run `clang-format` (18.x, `.clang-format` at the root) on
  C/C++ files, plus `cargo fmt` and `cargo clippy -D warnings` for `core/`.
  CI checks all three. Rust coverage in `core/` is gated at ≥ 85% lines.
- Dataset-dependent tests (EuRoC) skip unless their environment variable
  points at a sequence's `mav0/`; see `tests/sdk/vio_euroc.cpp`.

## Commits, pull requests, and releases

**Every change goes through a pull request**, including small ones.

- **Commit messages and PR titles follow
  [Conventional Commits](https://www.conventionalcommits.org/):**
  `type(scope): subject`, enforced by commitlint in CI.
  - Types: `feat`, `fix`, `perf`, `refactor`, `build`, `ci`, `docs`, `test`,
    `chore`, `style`, `revert`.
  - Scope is optional and **lowercase** (e.g. `sdk`, `math`, `core`, `tools`,
    `deps`).
  - **The subject starts lowercase.** `feat(sdk): add camera updaters`
    passes; `feat(sdk): Add camera updaters` fails.
  - Header ≤ 100 characters.
- **PRs are squash-merged**, so the PR title becomes the commit on `main`.
  Fill in the PR template (summary, changes, test results, layering, clean-room,
  and docs checklist).
- **Review:** CodeRabbit reviews ready PRs automatically (drafts are skipped).
  Open a draft while iterating, mark it ready when CI is green, and address
  the review before merging.
- **Link issues:** `Resolves #N` closes an issue on merge; use `Relates to #N`
  for partial work, and never `Resolves` an epic.
- **Releases are automatic.** [release-please](https://github.com/googleapis/release-please)
  reads the Conventional Commit types on `main` and maintains a release PR with
  the next **SemVer** version and the `CHANGELOG.md` entry. `feat` bumps the
  minor version and `fix` the patch (pre-1.0, breaking changes bump the minor).
  Merging the release PR tags `vX.Y.Z` and publishes the GitHub release.

## Documentation

- **The docs site** (`docs-site/`, Starlight plus a Doxygen API reference)
  publishes to <https://branes-ai.github.io/cortex/> on every push to `main`
  that touches it or the public headers. If a PR changes a subsystem,
  benchmark, or public API, update the matching page in the same PR.
  `npm run build:full` in `docs-site/` builds it locally.
- Some docs-site generators read test and header files by path and match
  their contents with regexes (e.g. benchmark gates in
  `tests/sdk/vio_euroc.cpp`). **Moving or reformatting a pinned file breaks
  the deploy** after merge, because the deploy runs only on `main`. Grep
  `docs-site/scripts/` for a file's path before moving or restructuring it.
- **Decisions** go in `docs/adr/` ([how to write one](docs/adr/README.md));
  assessments in `docs/assessments/`; long-form architecture in `docs/arch/`.

## Issues, labels, and milestones

Work is tracked as GitHub issues in this repo, grouped into epics and phases.

- **Titles use the same Conventional Commit form** as PRs
  (`feat(sdk): …`, `fix(core): …`, `docs: …`). Epics are titled
  `Epic: E<n> · <name>` and list their sub-issues.
- **Issue type** follows the title prefix: `feat` → *Feature*, `fix` → *Bug*,
  `chore` / `build` / `ci` / `test` / `docs` → *Task*. Epics are untyped.
- **Phase label → milestone:** each issue carries one `phase-N-…` label
  (`phase-0-foundation` through `phase-11-docs`) and sits in the matching
  milestone (`Phase N: …`). KPU-silicon-dependent work that is not on the MVP
  path carries the `phase-soc-deferred` label and goes in the milestone named
  `SoC (deferred)`, which wins if both apply.
- **Cross-cutting labels:** `epic`, `mvp-blocker`, `clean-room`, `no-gpl`,
  `cv-stack`, `decision-needed` (needs a maintainer call before work starts).
- Issues are tracked on the *Branes CORTEX* project board with an estimate.
