//! Error types for the cortex Resource Manager.
//!
//! Single, exhaustive enum across the `MemoryProvider` trait surface
//! plus future lifecycle / IPC operations. See
//! `docs/arch/phase1-design/README.md` § 6 for the rationale: no
//! `Box<dyn Error>`, no panics on fallible paths, every variant is
//! named for the failure mode rather than the call site.

use crate::memory_provider::BufferHandle;
use thiserror::Error;

/// Errors surfaced by `MemoryProvider` implementations.
///
/// Variants intentionally cover both backend-agnostic failures
/// (`InvalidHandle`, `AlignmentNotPow2`, `OutOfMemory`) and
/// backend-specific OS failures (`Os`). The `NotYetImplemented` and
/// `NotSupported` variants distinguish "stub still to be filled in"
/// from "this backend will never do that".
#[derive(Debug, Error)]
pub enum MemErr {
    /// The buffer handle is unknown to this provider — either never
    /// issued, or already released.
    #[error("buffer handle {0:?} is invalid or already released")]
    InvalidHandle(BufferHandle),

    /// The requested alignment is zero or not a power of two.
    #[error("requested alignment {alignment} is not a power of two")]
    AlignmentNotPow2 { alignment: usize },

    /// The requested allocation exceeds the provider's configured
    /// per-request size limit.
    #[error("requested byte_size {byte_size} exceeds provider limit {limit}")]
    SizeExceeded { byte_size: usize, limit: usize },

    /// The provider could not satisfy the request: out of available
    /// memory / address space.
    #[error("out of memory: requested {byte_size} bytes (alignment {alignment})")]
    OutOfMemory { byte_size: usize, alignment: usize },

    /// The operation is not supported by this provider and will never
    /// be (e.g. asking the SITL provider to enrol a KPU device fd).
    #[error("operation not supported by this provider (target = {target})")]
    NotSupported { target: &'static str },

    /// The operation is not yet implemented for this provider — used
    /// by the KPU stub until the silicon ships.
    #[error("operation not yet implemented for this provider (target = {target})")]
    NotYetImplemented { target: &'static str },

    /// An OS-level call (shm_open / mmap / CreateFileMapping / …)
    /// failed. The wrapped `std::io::Error` carries the OS errno.
    #[error("operating system error: {source}")]
    Os {
        #[from]
        source: std::io::Error,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_handle_display_mentions_handle_and_invalid() {
        let err = MemErr::InvalidHandle(BufferHandle::from_raw(0xABCD));
        let s = format!("{}", err);
        assert!(s.contains("BufferHandle"), "got: {s}");
        assert!(s.contains("invalid"), "got: {s}");
    }

    #[test]
    fn alignment_not_pow2_display_mentions_value() {
        let err = MemErr::AlignmentNotPow2 { alignment: 3 };
        let s = format!("{}", err);
        assert!(s.contains('3'), "got: {s}");
        assert!(s.contains("power of two"), "got: {s}");
    }

    #[test]
    fn size_exceeded_display_includes_both_values() {
        let err = MemErr::SizeExceeded {
            byte_size: 9_000_000_000,
            limit: 4_294_967_296,
        };
        let s = format!("{}", err);
        assert!(s.contains("9000000000"), "got: {s}");
        assert!(s.contains("4294967296"), "got: {s}");
    }

    #[test]
    fn out_of_memory_display_includes_request() {
        let err = MemErr::OutOfMemory {
            byte_size: 1024,
            alignment: 64,
        };
        let s = format!("{}", err);
        assert!(s.contains("1024"), "got: {s}");
        assert!(s.contains("64"), "got: {s}");
    }

    #[test]
    fn not_supported_display_includes_target() {
        let err = MemErr::NotSupported { target: "sitl" };
        let s = format!("{}", err);
        assert!(s.contains("sitl"), "got: {s}");
    }

    #[test]
    fn not_yet_implemented_display_includes_target() {
        let err = MemErr::NotYetImplemented { target: "kpu" };
        let s = format!("{}", err);
        assert!(s.contains("kpu"), "got: {s}");
    }

    #[test]
    fn from_io_error_yields_os_variant() {
        let io_err = std::io::Error::new(std::io::ErrorKind::OutOfMemory, "test failure");
        let mem_err: MemErr = io_err.into();
        match mem_err {
            MemErr::Os { source } => {
                assert!(format!("{}", source).contains("test failure"));
            }
            other => panic!("expected MemErr::Os, got {other:?}"),
        }
    }
}
