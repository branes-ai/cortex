//! cxx::bridge — C++ ⇄ Rust FFI surface for `cortex-core`.
//!
//! See `docs/arch/phase1-design/README.md` § 10 for the design.
//! The generated header lives at `branes/core/bridge.rs.h` and is
//! `#include`d by C++ TUs that talk to the Resource Manager.

use crate::error::MemErr;
use crate::kpu_provider::KpuMemoryProvider;
use crate::lifecycle::{Lifecycle, LifecycleErr, LifecycleState};
use crate::memory_provider::{BufferHandle, MemoryProvider};
use crate::sitl_provider::SitlMemoryProvider;
use std::sync::Arc;

#[cxx::bridge(namespace = "branes::core")]
pub mod ffi {
    /// Data-type tag mirrored on the C++ side. Must stay in sync with
    /// the C++ enum cxx emits in `bridge.rs.h`.
    #[repr(u8)]
    enum DataType {
        Uint8,
        Uint16,
        Uint32,
        Uint64,
        Int8,
        Int16,
        Int32,
        Int64,
        Float32,
        Float64,
    }

    /// Bridge view of the operator lifecycle state. Mirrors
    /// `crate::lifecycle::LifecycleState`.
    #[derive(Debug)]
    #[repr(u8)]
    enum BridgeLifecycleState {
        Unconfigured = 0,
        Inactive = 1,
        Active = 2,
        Teardown = 3,
    }

    /// View of an allocated buffer + its shape, designed to be wrapped
    /// in `std::span<T>` on the C++ side. POD — no destructors run
    /// across the bridge.
    struct TensorMetadata {
        handle: u64,
        data_ptr: usize,
        byte_size: u64,
        rows: u32,
        cols: u32,
        stride: u32,
        dtype: DataType,
    }

    extern "Rust" {
        type ResourceManager;

        /// Construct a new RM bound to either the SITL or the KPU
        /// provider, selected by the `RUST_HAL_TARGET` env var at
        /// construction time (case-insensitive `"KPU"` → KPU; anything
        /// else → SITL).
        fn make_resource_manager() -> Result<Box<ResourceManager>>;

        /// Allocate a buffer + lock it, returning the metadata C++
        /// needs to construct a `std::span<T>`.
        fn request_buffer(
            self: &ResourceManager,
            byte_size: u64,
            alignment: u64,
            rows: u32,
            cols: u32,
            stride: u32,
            dtype: DataType,
        ) -> Result<TensorMetadata>;

        /// Release the buffer. Second release returns an error rather
        /// than corrupting state.
        fn release_buffer(self: &ResourceManager, handle: u64) -> Result<()>;

        /// Observe current lifecycle state.
        fn lifecycle_state(self: &ResourceManager) -> BridgeLifecycleState;

        /// Lifecycle transitions. Invalid transitions return an error;
        /// cxx surfaces these as C++ exceptions.
        fn configure(self: &mut ResourceManager) -> Result<()>;
        fn activate(self: &mut ResourceManager) -> Result<()>;
        fn deactivate(self: &mut ResourceManager) -> Result<()>;
        fn teardown(self: &mut ResourceManager) -> Result<()>;
        fn reset(self: &mut ResourceManager) -> Result<()>;
    }
}

/// The bridge-exposed Resource Manager. Wraps a chosen
/// `MemoryProvider` (SITL or KPU) and an operator lifecycle.
pub struct ResourceManager {
    provider: Arc<dyn MemoryProvider>,
    lifecycle: Lifecycle,
}

/// Read the `RUST_HAL_TARGET` env var at construction time to pick a
/// provider. The CMake preset (`sitl-debug`, `kpu-cross`) sets this
/// before invoking cargo.
fn select_provider() -> Arc<dyn MemoryProvider> {
    if std::env::var("RUST_HAL_TARGET")
        .map(|v| v.eq_ignore_ascii_case("KPU"))
        .unwrap_or(false)
    {
        Arc::new(KpuMemoryProvider)
    } else {
        Arc::new(SitlMemoryProvider::new())
    }
}

/// Free function exposed across the bridge to construct an RM.
pub fn make_resource_manager() -> Result<Box<ResourceManager>, MemErr> {
    Ok(Box::new(ResourceManager {
        provider: select_provider(),
        lifecycle: Lifecycle::new(),
    }))
}

impl ResourceManager {
    pub fn request_buffer(
        &self,
        byte_size: u64,
        alignment: u64,
        rows: u32,
        cols: u32,
        stride: u32,
        dtype: ffi::DataType,
    ) -> Result<ffi::TensorMetadata, MemErr> {
        let handle = self
            .provider
            .request(byte_size as usize, alignment as usize)?;
        let (ptr, props) = self.provider.lock(handle)?;
        Ok(ffi::TensorMetadata {
            handle: handle.as_raw(),
            data_ptr: ptr as usize,
            byte_size: props.byte_size as u64,
            rows,
            cols,
            stride,
            dtype,
        })
    }

    pub fn release_buffer(&self, handle: u64) -> Result<(), MemErr> {
        self.provider.release(BufferHandle::from_raw(handle))
    }

    pub fn lifecycle_state(&self) -> ffi::BridgeLifecycleState {
        match self.lifecycle.state() {
            LifecycleState::Unconfigured => ffi::BridgeLifecycleState::Unconfigured,
            LifecycleState::Inactive => ffi::BridgeLifecycleState::Inactive,
            LifecycleState::Active => ffi::BridgeLifecycleState::Active,
            LifecycleState::Teardown => ffi::BridgeLifecycleState::Teardown,
        }
    }

    pub fn configure(&mut self) -> Result<(), LifecycleErr> {
        self.lifecycle.configure()
    }
    pub fn activate(&mut self) -> Result<(), LifecycleErr> {
        self.lifecycle.activate()
    }
    pub fn deactivate(&mut self) -> Result<(), LifecycleErr> {
        self.lifecycle.deactivate()
    }
    pub fn teardown(&mut self) -> Result<(), LifecycleErr> {
        self.lifecycle.teardown()
    }
    pub fn reset(&mut self) -> Result<(), LifecycleErr> {
        self.lifecycle.reset()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rm_for_test() -> Box<ResourceManager> {
        // Force the heap backend so tests don't pollute /dev/shm.
        std::env::set_var("CORTEX_MEM_BACKEND", "heap");
        // Force SITL (not KPU) regardless of CI env.
        std::env::remove_var("RUST_HAL_TARGET");
        make_resource_manager().expect("make_resource_manager")
    }

    #[test]
    fn make_resource_manager_succeeds() {
        let rm = rm_for_test();
        assert_eq!(
            rm.lifecycle_state(),
            ffi::BridgeLifecycleState::Unconfigured,
        );
    }

    #[test]
    fn request_buffer_returns_valid_metadata() {
        let rm = rm_for_test();
        let meta = rm
            .request_buffer(1024, 64, 32, 32, 32, ffi::DataType::Float32)
            .expect("request_buffer");
        assert!(meta.byte_size >= 1024);
        assert_ne!(meta.data_ptr, 0);
        assert_eq!(meta.rows, 32);
        assert_eq!(meta.cols, 32);
        assert_eq!(meta.stride, 32);
        rm.release_buffer(meta.handle).expect("release_buffer");
    }

    #[test]
    fn release_twice_returns_error() {
        let rm = rm_for_test();
        let meta = rm
            .request_buffer(64, 8, 1, 1, 1, ffi::DataType::Uint8)
            .unwrap();
        rm.release_buffer(meta.handle).unwrap();
        assert!(rm.release_buffer(meta.handle).is_err());
    }

    #[test]
    fn lifecycle_full_cycle_through_bridge() {
        let mut rm = rm_for_test();
        assert_eq!(
            rm.lifecycle_state(),
            ffi::BridgeLifecycleState::Unconfigured
        );
        rm.configure().unwrap();
        assert_eq!(rm.lifecycle_state(), ffi::BridgeLifecycleState::Inactive);
        rm.activate().unwrap();
        assert_eq!(rm.lifecycle_state(), ffi::BridgeLifecycleState::Active);
        rm.deactivate().unwrap();
        assert_eq!(rm.lifecycle_state(), ffi::BridgeLifecycleState::Inactive);
        rm.teardown().unwrap();
        assert_eq!(rm.lifecycle_state(), ffi::BridgeLifecycleState::Teardown);
        rm.reset().unwrap();
        assert_eq!(
            rm.lifecycle_state(),
            ffi::BridgeLifecycleState::Unconfigured
        );
    }

    #[test]
    fn invalid_lifecycle_transition_returns_error() {
        let mut rm = rm_for_test();
        // From Unconfigured, activate is invalid.
        assert!(rm.activate().is_err());
    }

    #[test]
    fn enum_values_align_with_lifecycle_state() {
        // Sanity: the bridge enum discriminants match the design doc.
        assert_eq!(ffi::BridgeLifecycleState::Unconfigured.repr, 0);
        assert_eq!(ffi::BridgeLifecycleState::Inactive.repr, 1);
        assert_eq!(ffi::BridgeLifecycleState::Active.repr, 2);
        assert_eq!(ffi::BridgeLifecycleState::Teardown.repr, 3);
    }
}
