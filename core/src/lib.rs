//! cortex-core — Rust Resource Manager.
//!
//! Public surface is established here. Concrete `MemoryProvider`
//! implementations (SITL, KPU stub), the lifecycle state machine, and
//! the `cxx::bridge` land in subsequent Phase 1 issues — see
//! `docs/arch/phase1-design/README.md`.

mod error;
mod memory_provider;

pub use error::MemErr;
pub use memory_provider::{BufferHandle, BufferProps, MemoryProvider};
