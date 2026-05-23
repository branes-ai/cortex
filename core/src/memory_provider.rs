//! The [`MemoryProvider`] trait — the single allocation/release/lock
//! surface every Resource Manager backend implements.
//!
//! Concrete implementations live in:
//!
//! - `sitl_provider.rs` — host-emulated zero-copy buffers via POSIX
//!   shm or Windows file mappings (issue #14).
//! - `kpu_provider.rs` — stub for the custom silicon target (issue
//!   #15).
//!
//! See `docs/arch/phase1-design/README.md` § 4 for the rationale
//! behind the API shape (`Send + Sync`, lock-returns-properties, no
//! realloc, opaque `BufferHandle`, …).

use crate::error::MemErr;

/// Opaque handle to a buffer owned by a [`MemoryProvider`].
///
/// The inner `u64` is provider-private; never inspect or compare it
/// outside the provider that issued it. `repr(transparent)` so the
/// cxx bridge can pass it across the FFI boundary as a plain integer.
///
/// `Copy` because the handle is a raw `u64` — copying does **not**
/// duplicate the underlying buffer. Two copies of the same handle
/// refer to the same allocation; double-release of the same handle
/// returns [`MemErr::InvalidHandle`] (per trait invariant 3) rather
/// than corrupting state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
pub struct BufferHandle(pub u64);

/// Properties of an allocated buffer, returned by
/// [`MemoryProvider::lock`] so the caller (typically the C++ SDK)
/// can construct a `std::span<T>` without re-querying the provider.
#[derive(Debug, Clone, Copy)]
pub struct BufferProps {
    /// Byte size of the buffer. May be larger than the requested
    /// size if the provider rounds up to a page boundary.
    pub byte_size: usize,

    /// Alignment of the returned pointer, in bytes. Power-of-two.
    /// At least as large as the `alignment` argument that was passed
    /// to [`MemoryProvider::request`].
    pub alignment: usize,
}

/// Source of zero-copy buffers.
///
/// # Invariants
///
/// Implementations must uphold:
///
/// 1. **Thread safety** (`Send + Sync`): the IPC broker
///    (E1-DEFERRED, #21) will share a single provider across
///    threads. Every method must be safe to call concurrently from
///    multiple threads.
/// 2. **Alignment**: the pointer returned by [`lock`](Self::lock) is
///    aligned to at least the `alignment` argument that was passed
///    to [`request`](Self::request). Providers may over-align (e.g.
///    round up to a page boundary).
/// 3. **Handle uniqueness**: a [`BufferHandle`] returned from
///    [`request`](Self::request) is unique for the lifetime of the
///    provider — even after [`release`](Self::release), a
///    previously-issued handle is not reused. Re-releasing the same
///    handle returns [`MemErr::InvalidHandle`] rather than silently
///    succeeding.
/// 4. **Lock idempotence**: calling [`lock`](Self::lock) twice on
///    the same valid handle returns the same pointer + properties;
///    no observable state changes between calls.
/// 5. **No panics on fallible paths**: every operation returns a
///    typed `Err` for failure modes (including allocation failure
///    and OS errors). Implementations must not `panic!`, `unwrap`,
///    or `expect` in production code paths.
pub trait MemoryProvider: Send + Sync {
    /// Allocate `byte_size` bytes aligned to `alignment`. Returns an
    /// opaque [`BufferHandle`] the caller stores; the underlying
    /// memory is not guaranteed to be mapped into the calling
    /// process until [`lock`](Self::lock).
    ///
    /// # Errors
    ///
    /// - [`MemErr::AlignmentNotPow2`] if `alignment` is zero or not
    ///   a power of two.
    /// - [`MemErr::SizeExceeded`] if `byte_size` exceeds the
    ///   provider-defined per-request limit.
    /// - [`MemErr::OutOfMemory`] if the provider cannot satisfy the
    ///   request.
    /// - [`MemErr::Os`] for backend-specific OS failures.
    fn request(&self, byte_size: usize, alignment: usize) -> Result<BufferHandle, MemErr>;

    /// Release the buffer. After this call the handle is invalid; any
    /// subsequent [`lock`](Self::lock) or [`release`](Self::release)
    /// on it returns [`MemErr::InvalidHandle`].
    fn release(&self, handle: BufferHandle) -> Result<(), MemErr>;

    /// Map the buffer into the calling process and return a raw
    /// pointer + the buffer's [`BufferProps`]. The pointer remains
    /// valid until [`release`](Self::release) is called. Idempotent —
    /// see invariant 4.
    ///
    /// # Safety
    ///
    /// The returned pointer is owned by the provider; the caller may
    /// read and write up to `props.byte_size` bytes through it but
    /// must not `free` / `munmap` it directly. Dereferencing the
    /// pointer after [`release`](Self::release) is undefined
    /// behavior.
    fn lock(&self, handle: BufferHandle) -> Result<(*mut u8, BufferProps), MemErr>;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Compile-time proof that [`MemoryProvider`] is dyn-compatible
    /// (formerly known as "object-safe"). If the trait gains a
    /// generic method or a `Self`-typed return, this fails to compile.
    #[allow(dead_code)]
    fn assert_dyn_compatible() {
        let _: Option<&dyn MemoryProvider> = None;
    }

    /// Minimal no-op impl used to verify the trait surface plus the
    /// `Send + Sync` bound at compile time. The real SITL provider
    /// lands in issue #14; the KPU stub in #15.
    struct NoopProvider;

    impl MemoryProvider for NoopProvider {
        fn request(&self, _byte_size: usize, _alignment: usize) -> Result<BufferHandle, MemErr> {
            Err(MemErr::NotYetImplemented { target: "noop" })
        }
        fn release(&self, _handle: BufferHandle) -> Result<(), MemErr> {
            Err(MemErr::NotYetImplemented { target: "noop" })
        }
        fn lock(&self, _handle: BufferHandle) -> Result<(*mut u8, BufferProps), MemErr> {
            Err(MemErr::NotYetImplemented { target: "noop" })
        }
    }

    #[test]
    fn noop_impl_returns_not_yet_implemented() {
        let provider = NoopProvider;
        match provider.request(1024, 64) {
            Err(MemErr::NotYetImplemented { target: "noop" }) => {}
            other => panic!("expected NotYetImplemented {{ target: \"noop\" }}, got {other:?}"),
        }
        match provider.release(BufferHandle(0)) {
            Err(MemErr::NotYetImplemented { target: "noop" }) => {}
            other => panic!("expected NotYetImplemented {{ target: \"noop\" }}, got {other:?}"),
        }
        match provider.lock(BufferHandle(0)) {
            Err(MemErr::NotYetImplemented { target: "noop" }) => {}
            other => panic!("expected NotYetImplemented {{ target: \"noop\" }}, got {other:?}"),
        }
    }

    #[test]
    fn provider_is_send_and_sync() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<NoopProvider>();
        // And via dyn trait object — proves the trait itself permits
        // boxing across thread boundaries.
        let _: Box<dyn MemoryProvider + Send + Sync> = Box::new(NoopProvider);
    }

    #[test]
    fn buffer_handle_is_copy_and_eq() {
        fn assert_copy<T: Copy>() {}
        assert_copy::<BufferHandle>();
        let a = BufferHandle(42);
        let b = a;
        assert_eq!(a, b);
    }

    #[test]
    fn buffer_handle_repr_transparent_size() {
        // repr(transparent) guarantees same layout as u64. The cxx
        // bridge depends on this for the FFI handoff.
        assert_eq!(
            std::mem::size_of::<BufferHandle>(),
            std::mem::size_of::<u64>(),
        );
        assert_eq!(
            std::mem::align_of::<BufferHandle>(),
            std::mem::align_of::<u64>(),
        );
    }

    #[test]
    fn buffer_props_construction() {
        let props = BufferProps {
            byte_size: 1024,
            alignment: 64,
        };
        assert_eq!(props.byte_size, 1024);
        assert_eq!(props.alignment, 64);
        // Copy works.
        let copy = props;
        assert_eq!(copy.byte_size, 1024);
    }
}
