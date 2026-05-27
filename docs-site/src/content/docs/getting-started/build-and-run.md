---
title: Build & Run
description: Building Cortex for the SITL host target and the KPU cross-compile target, and running the test suite.
---

The master build is **CMake-driven**. [Corrosion](https://github.com/corrosion-rs/corrosion)
invokes Cargo to compile the Rust `core/` into a static library that links into the
C++ targets. Third-party C++ dependencies come in via CMake `FetchContent` (MTL5,
Universal, Catch2, yaml-cpp, stb) — no vcpkg/Conan, because the cross-compilation
story for the custom SoC depends on every dependency living under the master
toolchain.

## SITL (host x86, emulated MemoryProvider) — default

```bash
cmake -B build -DBUILD_TARGET_KPU=OFF
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure
```

## KPU target (cross-compile, real KPU/DMA kernel bindings)

```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cross-toolchain.cmake -DBUILD_TARGET_KPU=ON
cmake --build build -j$(nproc)
```

## The `BUILD_TARGET_KPU` flag

This one flag controls **both sides of the FFI**:

- it sets `TARGET_KPU` / `TARGET_SITL` for the C++ targets, and
- it exports `RUST_HAL_TARGET=KPU|SITL` so the Rust `MemoryProvider` selects between
  the real kernel-driver implementation and the POSIX-shm emulation.

The C++ operators (`sdk/`) and math (`math/`) **never** branch on the build target —
that distinction stays confined to the Rust `core/` HAL.

## CMake presets

Four presets remove the need to memorize flag chains:

| Preset | Build type | Purpose |
|---|---|---|
| `sitl-debug` | Debug | Primary host build; **what CI runs** |
| `sitl-release` | Release | Optimized host build (latency budgets are enforced here) |
| `kpu-cross` | Release | aarch64 cross-compile for KPU silicon |
| `wsl2-sitl` | Debug | Linux build from the Windows/Visual Studio IDE via WSL2 |

```bash
cmake --preset sitl-release
cmake --build --preset sitl-release
ctest --preset sitl-release
```

## Running specific tests

The suite is [Catch2](https://github.com/catchorg/Catch2)-based and registered with
`catch_discover_tests`, so `ctest -R` filters by name:

```bash
# the VIO trajectory metrics + EuRoC harness
ctest -R vio_euroc --output-on-failure

# the MSCKF and estimator suites
ctest -R 'sdk_msckf|sdk_vio' --output-on-failure
```

The EuRoC accuracy benchmark and the latency budget are **dataset-/optimized-build
gated** — they run when you point `CORTEX_EUROC_V101` at a sequence and build under
`NDEBUG`, and otherwise skip cleanly so CI stays green without the dataset. See
[Benchmarks & Validation](/benchmarks/accuracy/).

## Toolchain notes

- **C++20 is required** (`std::span`, the concepts MTL5 leans on). Do not lower it.
- The **Rust toolchain is pinned** in `rust-toolchain.toml` (currently `1.83.0`);
  bumps are deliberate PRs of their own.
- If CI compile time grows, the answer is **sccache**, not a package manager.
- MLIR/LLVM (when introduced) are installed as pre-built binaries, never
  `FetchContent`'d — building LLVM from source destroys iteration time.
