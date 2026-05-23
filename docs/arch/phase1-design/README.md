# Phase 1 — Rust Resource Manager + cxx Bridge — Design

> Status: **proposed** (2026-05-23). Tracks issues [#13–#20](https://github.com/branes-ai/cortex/milestone/2) (MVP) and [#21–#23](https://github.com/branes-ai/cortex/issues?q=is%3Aissue+label%3Aphase-soc-deferred) (deferred).
> Companion documents: [`CLAUDE.md`](../../../CLAUDE.md) (layering invariants), [`docs/arch/cortex-repo.md`](../cortex-repo.md) (original architecture), [`docs/arch/phase0-foundation/README.md`](../phase0-foundation/README.md) (what landed in Phase 0).

This document **pre-decides every judgment call** for Phase 1 so the hands-free `/phase-implement 1` loop can land issues #13–#20 without stopping to ask. Each section below names the decision, the rationale, and the concrete API/code shape the implementing PR is expected to produce.

---

## 1. Goals

A SITL-only Rust Resource Manager exposing a strictly typed memory-allocation surface to C++ via the `cxx::bridge`, plus a lifecycle state machine that gates the configure/activate/teardown flow. By the end of Phase 1, a C++ program can:

1. Call into Rust to request a zero-copy buffer of arbitrary size and alignment.
2. Lock the buffer to obtain a raw pointer + `TensorMetadata`, write into it, and have the bytes be visible from the Rust side after release.
3. Walk an operator through `Unconfigured → Inactive → Active → Teardown` transitions, with invalid transitions returning typed errors (never panicking).

## 2. Non-goals (deferred to E1-DEFERRED, `phase-soc-deferred` label)

- IPC broker over Unix Domain Socket + POSIX shm — issue #21.
- Crash-only recovery / client replay protocol — issue #22.
- KPU resource allocation arbiter (live tile/memory map, conflict detection) — issue #23.
- Any KPU silicon driver code. The SoC `MemoryProvider` stub returns `Err(NotYetImplemented)` for every call.

## 3. Architecture (Phase 1 only)

```
         ┌──────────────────────────────────────────────┐
         │  C++ SDK / tests (consumer)                  │
         │   #include <branes/core/lib.rs.h>            │
         └─────────────────┬────────────────────────────┘
                           │  (cxx-generated headers)
                           ▼
         ┌──────────────────────────────────────────────┐
         │  cortex_core (Rust staticlib + cxx bridge)   │
         │                                              │
         │   pub use bridge::*;                         │
         │   ┌──────────────────────────────────────┐   │
         │   │  MemoryProvider trait (object-safe)  │   │
         │   └──────────┬───────────────────────────┘   │
         │              │                               │
         │   ┌──────────▼─────────┐  ┌───────────────┐  │
         │   │ SitlMemoryProvider │  │ SocMemoryProv │  │
         │   │  (shm + mmap;      │  │  (stub: Err)  │  │
         │   │   heap fallback)   │  │               │  │
         │   └────────────────────┘  └───────────────┘  │
         │                                              │
         │   ┌──────────────────────────────────────┐   │
         │   │  Lifecycle state machine             │   │
         │   │  Unconfigured→Inactive→Active        │   │
         │   │       →Teardown→Inactive             │   │
         │   └──────────────────────────────────────┘   │
         └──────────────────────────────────────────────┘
```

Selection between SITL and SoC providers happens via the `RUST_HAL_TARGET` env var set by the CMake preset (`SITL` or `KPU`). The unselected impl still compiles — both are present in the binary, picked at construction time.

## 4. `MemoryProvider` trait

The single allocation/release/lock surface every backend must implement.

### 4.1 Trait definition

```rust
// core/src/memory_provider.rs

use crate::error::MemErr;

/// Opaque handle to a buffer owned by a MemoryProvider. The bit pattern
/// is provider-private; never inspect or compare the inner field
/// outside the provider that issued it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct BufferHandle(pub u64);

/// Properties of an allocated buffer that the lock step exposes back
/// to the caller (so the C++ side can construct a `std::span` without
/// re-querying the provider).
#[derive(Debug, Clone, Copy)]
pub struct BufferProps {
    pub byte_size: usize,
    pub alignment: usize,
}

/// Source of zero-copy buffers. Implementations must be Send + Sync —
/// the future IPC broker will share a single provider across threads.
pub trait MemoryProvider: Send + Sync {
    /// Allocate `byte_size` bytes aligned to `alignment`. Returns an
    /// opaque handle the caller stores; the underlying memory is not
    /// guaranteed to be mapped into the calling process until `lock`.
    fn request(&self, byte_size: usize, alignment: usize) -> Result<BufferHandle, MemErr>;

    /// Release the buffer. After this call the handle is invalid; any
    /// subsequent `lock`/`release` on it returns `MemErr::InvalidHandle`.
    fn release(&self, handle: BufferHandle) -> Result<(), MemErr>;

    /// Map the buffer into the calling process and return a raw
    /// pointer + the buffer's properties. The pointer remains valid
    /// until `release` is called. Calling `lock` twice on the same
    /// handle returns the same pointer (idempotent).
    fn lock(&self, handle: BufferHandle) -> Result<(*mut u8, BufferProps), MemErr>;
}
```

**Decisions encoded here:**
- `BufferHandle` is `repr(transparent) u64`. Opaque to the caller; provider-internal interpretation. Stable across the FFI boundary (the cxx bridge sees it as a plain integer).
- `lock` returns the pointer **and** the buffer's properties (size + alignment). The C++ side needs both to construct a `std::span<T>`. Avoids a second round-trip.
- `lock` is idempotent — the same handle returns the same pointer. Avoids accidental double-mapping.
- `Send + Sync` is required, not optional. SITL provider uses `Arc<Mutex<...>>` internally.
- No async / no futures. Phase 1 is synchronous; async lands when the IPC broker arrives.

### 4.2 What the trait is *not*

- No `realloc`. Buffers are fixed-size; resize means allocate-new + copy + release-old, owned by the caller.
- No `flush` / `sync`. SITL is host-memory only; KPU sync becomes a separate trait method in E1-DEFERRED if needed.
- No `cancel`. The lifecycle state machine handles cancellation by transitioning to Teardown.

## 5. `BufferHandle` lifetime and ownership

- Allocation creates a handle; the provider tracks it internally.
- `Clone + Copy` because the handle is `u64`. Copying a handle does NOT duplicate the underlying buffer — both copies refer to the same allocation. Double-release of the same handle returns `MemErr::InvalidHandle` (not an undefined-behavior corruption).
- No `Drop` impl on `BufferHandle` — that would require carrying a back-reference to the provider, breaking `Copy`. Callers explicitly release.
- Future improvement (post-Phase 1): an owning `OwnedBuffer<'a>` wrapper that holds a `&'a dyn MemoryProvider` and releases on Drop, for ergonomic Rust-side usage.

## 6. Error taxonomy (`MemErr`)

Single enum across the trait surface. Avoids `Box<dyn Error>`, avoids panics.

```rust
// core/src/error.rs

use thiserror::Error;

#[derive(Debug, Error)]
pub enum MemErr {
    #[error("buffer handle {0:?} is invalid or already released")]
    InvalidHandle(BufferHandle),

    #[error("requested alignment {alignment} is not a power of two")]
    AlignmentNotPow2 { alignment: usize },

    #[error("requested byte_size {byte_size} exceeds provider limit {limit}")]
    SizeExceeded { byte_size: usize, limit: usize },

    #[error("out of memory: requested {byte_size} bytes (alignment {alignment})")]
    OutOfMemory { byte_size: usize, alignment: usize },

    #[error("operation not supported by this provider (target = {target})")]
    NotSupported { target: &'static str },

    #[error("operation not yet implemented for this provider (target = {target})")]
    NotYetImplemented { target: &'static str },

    #[error("operating system error: {source}")]
    Os {
        #[from]
        source: std::io::Error,
    },
}
```

**Decisions encoded here:**
- Variants are exhaustive and named after the failure mode, not the call site.
- `NotYetImplemented` is for the SoC stub — every method returns this. Distinct from `NotSupported` (which means "this backend will never do that").
- `Os` variant absorbs `std::io::Error` for OS-call failures (shm_open, mmap, etc.).
- Across the cxx bridge: errors become `Result<T, String>` where the string is `format!("{}", err)`. C++ side checks for `Err` and propagates as a `std::expected<T, std::string>` or throws — see §10.

## 7. Lifecycle state machine

```
    Unconfigured ──configure(yaml)──▶ Inactive
                                       │
                                       │ activate()
                                       ▼
                                     Active
                                       │
                                       │ deactivate()
                                       ▼
                                     Inactive
                                       │
                                       │ teardown()
                                       ▼
                                    Teardown
                                       │
                                       │ on-drop / reset()
                                       ▼
                                  Unconfigured
```

### 7.1 Implementation: enum-backed, not type-state

```rust
// core/src/lifecycle.rs

use crate::error::MemErr;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleState {
    Unconfigured,
    Inactive,
    Active,
    Teardown,
}

#[derive(Debug, Error)]
pub enum LifecycleErr {
    #[error("invalid transition: from {from:?} to {to:?}")]
    InvalidTransition { from: LifecycleState, to: LifecycleState },
}

pub struct Lifecycle {
    state: LifecycleState,
    // future: hold the parsed config + the provider here
}

impl Lifecycle {
    pub fn new() -> Self {
        Self { state: LifecycleState::Unconfigured }
    }

    pub fn state(&self) -> LifecycleState { self.state }

    pub fn configure(&mut self) -> Result<(), LifecycleErr> { /* Unconfigured → Inactive */ }
    pub fn activate(&mut self)  -> Result<(), LifecycleErr> { /* Inactive    → Active */ }
    pub fn deactivate(&mut self) -> Result<(), LifecycleErr> { /* Active     → Inactive */ }
    pub fn teardown(&mut self)  -> Result<(), LifecycleErr> { /* Inactive    → Teardown */ }
    pub fn reset(&mut self)     -> Result<(), LifecycleErr> { /* Teardown    → Unconfigured */ }
}
```

**Why enum-backed, not type-state:**
Type-state (encoding state in the type system via `Lifecycle<Unconfigured>` etc.) is the prettier Rust pattern. But the cxx bridge sees a single C++-visible class — type parameters can't cross the boundary. Enum-backed is the natural fit for FFI, and we still get exhaustive matching + compile-time enforcement that every state has handlers.

### 7.2 Transition rules (exhaustive)

| From | Method | To | Invalid? |
|---|---|---|---|
| Unconfigured | `configure` | Inactive | |
| Unconfigured | anything else | — | yes (`InvalidTransition`) |
| Inactive | `activate` | Active | |
| Inactive | `teardown` | Teardown | |
| Inactive | `configure` | Inactive (no-op? or error? — **decision: error**, requires explicit `reset` first) | |
| Active | `deactivate` | Inactive | |
| Active | new-config arrival | Teardown (then Inactive on next configure) | |
| Active | anything else | — | yes |
| Teardown | `reset` | Unconfigured | |
| Teardown | anything else | — | yes |

Invalid transitions return `Err(LifecycleErr::InvalidTransition { from, to })`. No panics.

## 8. SITL `MemoryProvider` implementation

### 8.1 Backends

| OS | Backend | Selected when |
|---|---|---|
| Linux | `shm_open` + `mmap` (anonymous shared fd) | `cfg(target_os = "linux")` and `CORTEX_MEM_BACKEND != "heap"` |
| Windows | `CreateFileMappingA` + `MapViewOfFile` (paging-file-backed) | `cfg(target_os = "windows")` and `CORTEX_MEM_BACKEND != "heap"` |
| Any | Heap (`std::alloc::alloc_zeroed`) | `CORTEX_MEM_BACKEND=heap` (or fallback if shm unavailable) |

Heap fallback is the simplest path for unit tests where shm setup costs aren't worth it.

### 8.2 Internal state shape

```rust
struct SitlProviderInner {
    backends: Mutex<HashMap<BufferHandle, BackendEntry>>,
    next_id: AtomicU64,
}

enum BackendEntry {
    Shm { ptr: NonNull<u8>, byte_size: usize, alignment: usize, /* OS-specific fd/handle */ },
    Heap { ptr: NonNull<u8>, layout: std::alloc::Layout },
}
```

`Arc<SitlProviderInner>` is the public type; the trait impl is on `Arc<...>`.

### 8.3 Invariants the implementation must enforce

- `alignment` must be a power of two ≥ 1. Otherwise `Err(AlignmentNotPow2)`.
- `byte_size == 0` is allowed (returns a valid handle whose `lock` produces a non-null pointer to a zero-byte region — the underlying alloc rounds up to page size).
- Maximum single allocation: 4 GiB. Above that returns `Err(SizeExceeded { limit: 1 << 32 })`. Document in code; bump later if needed.
- Concurrent `request` from multiple threads is safe; the mutex serializes registration.
- `release` is idempotent only in the "InvalidHandle on second call" sense — it does NOT silently succeed twice.
- Drop of `SitlProviderInner` releases all outstanding buffers (RAII cleanup).

## 9. SoC `MemoryProvider` stub

Per #15 — trait conformance only. Every method returns `Err(MemErr::NotYetImplemented { target: "soc" })`.

```rust
pub struct SocMemoryProvider;

impl MemoryProvider for SocMemoryProvider {
    fn request(&self, _byte_size: usize, _alignment: usize) -> Result<BufferHandle, MemErr> {
        Err(MemErr::NotYetImplemented { target: "soc" })
    }
    fn release(&self, _handle: BufferHandle) -> Result<(), MemErr> {
        Err(MemErr::NotYetImplemented { target: "soc" })
    }
    fn lock(&self, _handle: BufferHandle) -> Result<(*mut u8, BufferProps), MemErr> {
        Err(MemErr::NotYetImplemented { target: "soc" })
    }
}
```

The accompanying `core/src/soc_provider.rs` includes a `// IMPLEMENTATION PLAN` block at the top of the file outlining:
- The kernel device node (`/dev/kpu`, expected ioctls).
- The expected DMA-buf protocol.
- Page-locking strategy.
- Concurrency requirements (likely a single-process arbiter).

This block becomes the implementation guide when the SoC issue (#21–#23 region) lands.

## 10. cxx::bridge surface

The single bridge module in `core/src/lib.rs`.

```rust
// core/src/lib.rs (Phase 1 final form)

mod error;
mod memory_provider;
mod sitl_provider;
mod soc_provider;
mod lifecycle;
mod bridge;

pub use bridge::ffi;

// The bridge module re-exports the types/functions C++ sees.
```

### 10.1 Bridge module

```rust
// core/src/bridge.rs

#[cxx::bridge(namespace = "branes::core")]
pub mod ffi {
    /// Data types we publish into TensorMetadata. Must stay in sync
    /// with the C++ side; cxx generates a mirrored enum.
    #[repr(u8)]
    enum DataType {
        Uint8,  Uint16, Uint32, Uint64,
        Int8,   Int16,  Int32,  Int64,
        Float32, Float64,
    }

    /// View of a buffer + its shape, suitable for wrapping in std::span
    /// on the C++ side. Allocated via request_buffer; released via
    /// release_buffer. POD — no destructors run across the bridge.
    struct TensorMetadata {
        handle: u64,        // matches BufferHandle internally
        data_ptr: usize,    // raw pointer cast to usize
        byte_size: u64,
        rows: u32,
        cols: u32,
        stride: u32,        // in elements, not bytes
        dtype: DataType,
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum BridgeLifecycleState {
        Unconfigured = 0,
        Inactive    = 1,
        Active      = 2,
        Teardown    = 3,
    }

    extern "Rust" {
        type ResourceManager;

        /// Construct a new RM bound to the SITL or SoC provider, picked
        /// by the `RUST_HAL_TARGET` env var at construction time.
        fn make_resource_manager() -> Result<Box<ResourceManager>>;

        /// Request a buffer; returns metadata you can pass to lock().
        fn request_buffer(
            self: &ResourceManager,
            byte_size: u64,
            alignment: u64,
            rows: u32,
            cols: u32,
            stride: u32,
            dtype: DataType,
        ) -> Result<TensorMetadata>;

        /// Release. Idempotent error: second release returns Err.
        fn release_buffer(self: &ResourceManager, handle: u64) -> Result<()>;

        // Lifecycle transitions
        fn lifecycle_state(self: &ResourceManager) -> BridgeLifecycleState;
        fn configure(self: &mut ResourceManager) -> Result<()>;
        fn activate(self: &mut ResourceManager) -> Result<()>;
        fn deactivate(self: &mut ResourceManager) -> Result<()>;
        fn teardown(self: &mut ResourceManager) -> Result<()>;
        fn reset(self: &mut ResourceManager) -> Result<()>;
    }
}
```

### 10.2 Error propagation across the bridge

cxx wraps `Result<T, E>` into C++ exceptions if `E: Display`. Strategy: every Rust function exposed across the bridge returns `Result<T, MemErr>` or `Result<T, LifecycleErr>`. cxx converts to C++ `rust::Error` (which is a `std::exception`). C++ tests catch and inspect via `e.what()`.

## 11. Build system

### 11.1 Cargo.toml

```toml
[package]
name = "cortex-core"
version = "0.1.0"
edition = "2021"
publish = false
license = "MIT"
description = "Cortex Rust Resource Manager — KPU memory pools + cxx bridge"

[lib]
crate-type = ["staticlib", "rlib"]

[dependencies]
cxx = "1.0"
thiserror = "1.0"

[target.'cfg(target_os = "linux")'.dependencies]
nix = { version = "0.29", features = ["mman", "fs"] }

[target.'cfg(target_os = "windows")'.dependencies]
windows-sys = { version = "0.59", features = [
    "Win32_System_Memory",
    "Win32_Foundation",
] }

[build-dependencies]
cxx-build = "1.0"
```

### 11.2 build.rs

```rust
fn main() {
    cxx_build::bridge("src/bridge.rs")
        .compile("cortex_core_bridge");
    println!("cargo:rerun-if-changed=src/bridge.rs");
}
```

(The bridge module is in `bridge.rs`, not `lib.rs`, so the `#[cxx::bridge]` macro and its generated headers can be developed in isolation from the public-facing lib.rs.)

### 11.3 Generated header path

Corrosion exports the generated header at `$<TARGET_PROPERTY:cortex_core,INTERFACE_INCLUDE_DIRECTORIES>` automatically. C++ consumers:

```cpp
#include <branes/core/bridge.rs.h>
```

(cxx names the generated header after the source file: `bridge.rs` → `bridge.rs.h`. The `branes/core/` prefix comes from the namespace declaration in the bridge.)

## 12. Concurrency and safety invariants

- All public `core/` APIs are safe Rust. The only `unsafe` blocks are inside `sitl_provider` (for shm_open / mmap / CreateFileMapping) and at the FFI boundary (raw pointer return from `lock`).
- The SITL provider's `Arc<Mutex<...>>` guards the handle table. Mutex held only for the registration/lookup; the actual allocation happens outside the lock (call shm_open without the lock, then take the lock to insert into the map).
- The lifecycle state machine is `!Send` until wrapped in `Box`. The bridge wraps it; raw `Lifecycle` instances can't accidentally cross threads.
- **No panics** in production code paths. The only `panic!` permitted is in `#[cfg(test)]` modules. Clippy enforced via the lint job.

## 13. Test plan

### 13.1 Rust unit tests (#19)

Per file:

| File | Required coverage |
|---|---|
| `error.rs` | Display formatting for each variant; `From<io::Error>` works |
| `lifecycle.rs` | All valid transitions; all 12 invalid transitions emit `InvalidTransition`; state observability |
| `memory_provider.rs` | Trait is object-safe (dyn-compatible — compile-time check via `let _: &dyn MemoryProvider = ...`) |
| `sitl_provider.rs` | request/release/lock happy path; double-release returns `InvalidHandle`; non-pow2 alignment returns `AlignmentNotPow2`; size > 4GiB returns `SizeExceeded`; lock returns same pointer twice |
| `soc_provider.rs` | Every method returns `NotYetImplemented { target: "soc" }` |
| `bridge.rs` | `make_resource_manager()` succeeds; bridge enum values match Rust enum values |

Total: ~20 tests. `cargo llvm-cov` should report ≥ 85% line coverage of `core/src/*` after this lands. **The PR closing #20 turns on `--fail-under-lines 85` in the coverage CI step.**

### 13.2 C++↔Rust round-trip integration test (#20)

`core/tests/cxx_bridge_test.cpp` (or similar location):

```cpp
TEST_CASE("ResourceManager round-trip", "[bridge][integration]") {
    auto rm = branes::core::make_resource_manager();
    REQUIRE(rm);
    REQUIRE(rm->lifecycle_state() == BridgeLifecycleState::Unconfigured);

    rm->configure();
    REQUIRE(rm->lifecycle_state() == BridgeLifecycleState::Inactive);

    rm->activate();
    REQUIRE(rm->lifecycle_state() == BridgeLifecycleState::Active);

    auto meta = rm->request_buffer(/*byte_size=*/1<<20, /*alignment=*/64,
                                   /*rows=*/1024, /*cols=*/1024, /*stride=*/1024,
                                   DataType::Uint8);

    // Wrap in std::span, write a pattern, release.
    auto* ptr = reinterpret_cast<uint8_t*>(meta.data_ptr);
    std::span<uint8_t> view(ptr, meta.byte_size);
    std::fill(view.begin(), view.end(), 0xA5);

    REQUIRE(view[0] == 0xA5);
    REQUIRE(view[meta.byte_size - 1] == 0xA5);

    rm->release_buffer(meta.handle);
    rm->deactivate();
    rm->teardown();
    rm->reset();
}

TEST_CASE("invalid lifecycle transitions return Err", "[bridge][lifecycle]") {
    auto rm = branes::core::make_resource_manager();
    // Cannot activate from Unconfigured
    REQUIRE_THROWS_AS(rm->activate(), rust::Error);
}
```

## 14. Acceptance-criteria-to-test map (driver for `/phase-implement`)

| Issue | Acceptance criterion | Test(s) that prove it |
|---|---|---|
| #13 | Trait compiles + dyn-compatible | `memory_provider::tests::object_safety` |
| #14 | shm/Windows/heap backends work; no leaks | `sitl_provider::tests::*` + Valgrind step on the integration test |
| #15 | SoC stub returns NotYetImplemented | `soc_provider::tests::all_methods_unimplemented` |
| #16 | C++ test calls bridge, writes, Rust verifies | `cxx_bridge_test.cpp::ResourceManager round-trip` |
| #17 | `#include <branes/core/bridge.rs.h>` works | The bridge integration test's `#include` line is the test |
| #18 | Lifecycle transitions valid + invalid handled | `lifecycle::tests::*` |
| #19 | Unit tests cover provider + lifecycle, ≥ 85% | Coverage CI step + `cargo test` count |
| #20 | C++↔Rust round-trip integration | `cxx_bridge_test.cpp::ResourceManager round-trip` |

## 15. Forward links

- **Phase 2 (Math)** does not depend on Phase 1's runtime — math is pure header-only and operates on caller-owned buffers. But Phase 2's tensor view types (#24) will use the `TensorMetadata` struct as the canonical shape descriptor.
- **Phase 3 (VIO)** consumes `TensorMetadata` directly to wrap zero-copy camera frames + IMU samples. The `make_resource_manager()` factory becomes the bootstrap call for the VIO daemon.
- **E1-DEFERRED** (IPC broker, crash recovery, KPU arbiter, #21–#23) builds on the trait and lifecycle defined here. The trait surface is intentionally narrow so the deferred work can wrap rather than replace.

---

## Appendix A — Crates the implementation will pull in

| Crate | Version | Purpose |
|---|---|---|
| cxx | 1.0 | FFI bridge generation |
| cxx-build | 1.0 | build-time codegen |
| thiserror | 1.0 | derive `Error` for `MemErr`/`LifecycleErr` |
| nix | 0.29 | Linux shm_open / mmap wrappers |
| windows-sys | 0.59 | Windows CreateFileMapping wrappers |

All MIT or dual MIT/Apache-2.0. No GPL.

## Appendix B — What `/phase-implement 1` does (recap)

1. Read `docs/arch/phase1-design/README.md` (this file) for context. The loop never asks judgment-call questions while the design doc covers them.
2. Query the Phase 1 milestone for open issues. Sort by issue number for dependency order (#13 → #14 → #15 → #16 → #17 → #18 → #19 → #20). Deferred sub-issues #21–#23 skipped (label filter).
3. For each issue: branch → implement per this doc's spec → `tests/` per §13 → push → wait for CI green → mark ready → wait for CR clean → `gh pr merge --auto --squash`.
4. On any test/CI failure that the loop can't self-resolve in one fix attempt, stop and report to Theo.
5. On all issues closed: report Phase 1 complete; prompt Theo to confirm before starting Phase 2 (which has its own design doc).
