# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Status

This repository is an **early skeleton**. The root build configures and produces an empty `libcortex_core.a`; the subdirectory `CMakeLists.txt` files are deliberate placeholders, and the Rust files under `core/src/` are stubs pointing at the issues that will implement them. Expect to be filling in stubs rather than refactoring existing code.

The canonical design document is `docs/arch/cortex-repo.md` — read it before making non-trivial structural changes. It records architectural decisions that the directory layout alone does not capture (e.g., the "Host Broker + Thin Client" topology, the crash-only RM recovery model, the dual OCI/Yocto delivery strategy).

## Build commands

The master build is CMake-driven; Corrosion invokes Cargo to compile the Rust `core/` into a static library that links into the C++ targets.

```bash
# SITL (host x86, emulated MemoryProvider) — default
cmake -B build -DBUILD_TARGET_KPU=OFF
cmake --build build -j$(nproc)
cd build && ctest --output-on-failure

# KPU target (cross-compile, real KPU/DMA kernel bindings) — needs cross-toolchain.cmake (issue #6)
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cross-toolchain.cmake -DBUILD_TARGET_KPU=ON
cmake --build build -j$(nproc)
```

The `BUILD_TARGET_KPU` flag controls *both* sides of the FFI: it sets `TARGET_KPU`/`TARGET_SITL` for C++ and exports `RUST_HAL_TARGET=KPU|SITL` for the Rust `MemoryProvider` to select between the real kernel-driver implementation and the POSIX-shm emulation. Never branch on the build target inside the C++ operators (`sdk/`) or math (`math/`) — that distinction must stay confined to the Rust `core/` HAL.

The cross-compile CI job is currently disabled (commented out in `.github/workflows/ci.yml`) and re-enabled when `cross-toolchain.cmake` lands in issue #6.

## Architectural boundaries (load-bearing — do not collapse)

The whole point of the layering is that **algorithms never see middleware, and middleware never sees hardware**. Each layer's allowed dependencies are strict:

| Layer | Path | May depend on | Must NOT depend on |
|---|---|---|---|
| Resource Manager | `core/` (Rust) | OS, kernel drivers, `cxx` | C++ SDK, Zenoh, ROS 2 |
| Math | `math/` (header-only C++20) | Universal, MTL5 | Anything heavier |
| Operator SDK | `sdk/` (C++) | `math/`, `core/` via cxx bridge, `std::span` | Zenoh, ROS 2, YAML parsing |
| Daemons | `daemons/` (C++ executables) | `sdk/`, Zenoh, yaml-cpp | (top of stack) |

The cxx bridge is defined in `core/src/lib.rs` under `#[cxx::bridge(namespace = "branes::core")]`. Tensor metadata crosses the boundary as a `repr(C)` struct (`TensorMetadata { data_ptr, rows, cols, ... }`) — raw pointer plus shape, no Arrow-style metadata, no ownership semantics. The C++ side immediately wraps the pointer in `std::span<T>` and hands it to MTL5 views; it never copies.

**Configuration discipline:** YAML/JSON are parsed only at the daemon entry point into typed C++ POD structs (`VioConfig`, etc.). The SDK operators take those structs by value — they never see `yaml-cpp` types. The Rust layer uses `serde` to parse the *same* config file for memory-pool sizing.

**No dynamic reconfiguration.** Parameter changes go through a managed lifecycle (`Unconfigured → Inactive → Active → teardown`). Do not add mutex-guarded mutable parameters to the hot path — this was an explicit architectural decision to preserve real-time determinism.

## The KPU spacetime constraint

The KPU is a spatial dataflow fabric, not a time-sliced accelerator. Tensors are physically distributed across the compute tiles; shape and location *are* part of the schedule, baked in offline by the DFA graph compiler (a separate repo, not `cortex`). The compiler emits **domain flow programs** — scheduled, fused subgraphs that the Rust RM places on allocated hardware resources. Once a domain flow program is loaded, the KPU runs its own dataflow internally; no runtime scheduling happens on the host.

This has three consequences for any code in this repo:

1. **The Rust RM manages *allocations*, not *schedules*.** Its job is the live map of which domain flow programs occupy which physical tiles/memory regions, conflict detection, and release-on-completion. Don't add scheduler logic here — that belongs in the DFA compiler.
2. **There is exactly one Resource Manager process** with access to `/dev/kpu`. Operator containers are unprivileged and request execution via Unix Domain Socket + POSIX shared memory IPC. Never give an SDK consumer or daemon direct hardware access.
3. **The fabric is treated as stateless.** Recovery from an RM crash is by client replay, not checkpointing — the SDK's IPC layer must enter a brief "Holding Pattern" on socket close, then replay pending requests when the RM respawns. Don't add disk-based checkpoint paths.

## What lives where

- `core/` — Rust Resource Manager. `memory_provider.rs` is the SITL-vs-KPU HAL trait; expand here, not in C++.
- `math/include/branes/math/` — header-only MTL5-based non-linear least squares (Gauss-Newton, Levenberg-Marquardt), Krylov inner solvers via MTL5 ITL, sparse direct solvers. First-class support for custom arithmetic (Universal posits, mixed-precision) — do not hardcode `double`.
- `sdk/include/branes/sdk/` + `sdk/src/` — VIO, SLAM, SfM, 3D Scene Graph operators. The 3DSG backing store must be contiguous arrays indexed by integer node IDs (data-oriented), not heap-allocated nodes with `shared_ptr` children — pointer chasing kills cache locality on the SoC.
- `daemons/src/` — one executable per operator (`vio_daemon.cpp`, etc.). These are thin: subscribe to Zenoh, deserialize config, call SDK, publish results. No algorithm code here.
- `config/` — unified YAML schemas consumed by both Rust (`serde`) and C++ (`yaml-cpp`).
- `docs/arch/cortex-repo.md` — full architectural record; treat as authoritative when the code is ambiguous.

## Dependency management

Use CMake `FetchContent` for C++ third-party deps (Zenoh C++, yaml-cpp, MTL5, Universal). Do *not* introduce vcpkg/Conan — the cross-compilation story for the custom SoC is built around having every dependency under the master CMake toolchain. The one exception is MLIR/LLVM: those must be installed as pre-built binaries, never `FetchContent`'d, because building LLVM from source destroys developer iteration time.

If a CI compile-time problem appears, the answer is `sccache`, not a package manager.

## Conventions worth knowing

- Target C++20 (required for `std::span` and the C++20 concepts MTL5 leans on). Do not lower the standard.
- Rust toolchain is pinned via `rust-toolchain.toml` at `1.83.0`. When adding Rust targets, also add them there so `rustup` picks them up automatically. Bumps are deliberate PRs of their own.
- Primary developer IDE is **Visual Studio 2022 on Windows** with CMakePresets. WSL2 is used for Linux cross-compilation from the same IDE. Profiling is cross-platform (Tracy + Valgrind), never MSVC-specific tools.
