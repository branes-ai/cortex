# ADR 0011 — C++ dependencies via CMake FetchContent; no package manager

**Status:** Accepted (amended 2026-10-07: CMake 4.0 minimum; Stillwater header-only fetch pattern)
**Date:** 2026-05-20
**Source:** `docs/arch/cortex-repo.md`, *Why FetchContent Wins for Custom Silicon*; PRs #434, #436
**Related:** [ADR-0001](0001-third-party-license-compatibility.md), [ADR-0007](0007-dual-delivery-oci-and-yocto.md)

## Context

cortex has to cross-compile for custom silicon (KPU target) with custom
flags. vcpkg and Conan would each need custom triplets/profiles to cross-compile
every dependency. The core math (MTL5, Universal) is header-only, and early
development often needs to patch or co-develop dependencies.

## Decision

- **All C++ third-party dependencies come in through CMake `FetchContent`**,
  pinned to release tags (or a commit SHA where a project has no releases,
  e.g. stb), declared in `cmake/deps.cmake`. **No vcpkg, no Conan.**
- **CI compile time is solved with `sccache`**, not a package manager. It
  caches both `rustc` and C/C++ objects.
- **Exception: MLIR/LLVM** is never FetchContent'd. Building LLVM from
  source destroys iteration time; it is installed as prebuilt binaries.
- **Amendment (#436):** the header-only Stillwater sister projects (MTL5,
  Universal) use the pattern from the Stillwater mixed-precision repos.
  `find_package` runs first **with no version constraint**, so any installed
  package wins. If none is found, a header-only fetch (`SOURCE_SUBDIR` with
  no `CMakeLists.txt`) pulls the release tag pinned in `cmake/deps.cmake`,
  so their own CMake projects are never added.
  `FETCHCONTENT_SOURCE_DIR_*` is an explicit local-checkout override that
  bypasses the pin, for co-development. All paths expose `MTL5::mtl5` /
  `universal::universal`.
- **Amendment (#434):** CMake **4.0** is the minimum version. CI installs a
  pinned CMake 4.0.x rather than trusting runner images.

## Consequences

- Every dependency builds under the master toolchain, so cross-compilation
  and the Yocto delivery see one consistent build.
- Dependency bumps are dedicated PRs, so release-please records them.
- Developers need CMake ≥ 4.0. Visual Studio 2022's bundled CMake is 3.x, so
  point the IDE at a newer CMake.
