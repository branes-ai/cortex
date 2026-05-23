//! KPU `MemoryProvider` — trait-conformance stub.
//!
//! Every method returns `Err(MemErr::NotYetImplemented { target: "kpu" })`.
//! Exists so downstream code (the `cxx::bridge` in #16, the lifecycle
//! state machine in #18, the future operator SDK) can refer to a
//! concrete `MemoryProvider` impl for the KPU target without waiting
//! for the silicon work to land.
//!
//! # Implementation plan (deferred — gated on `phase-soc-deferred`)
//!
//! When the KPU device driver and the IPC broker (#21) come online,
//! this file fills in around the following skeleton. Tracking work in
//! issues #21–#23 (see `docs/arch/phase1-design/README.md` § 2 for the
//! non-goals scope).
//!
//! 1. **Kernel device node.** The Rust resource manager opens
//!    `/dev/kpu0` once (held in `Inner` behind an `Arc`). The fd is
//!    process-private; the IPC broker (#21) hands client containers
//!    references via `SCM_RIGHTS` rather than passing the fd itself.
//!
//! 2. **Allocation ioctl protocol.** Single ioctl `KPU_IOC_ALLOC`
//!    takes `{ byte_size, alignment, flags }` and returns
//!    `{ buffer_id, dma_addr, host_va }`. The `buffer_id` becomes the
//!    inner `u64` of `BufferHandle::from_raw`.
//!
//! 3. **DMA-buf protocol.** For inter-process sharing the kernel
//!    driver exports a dma-buf fd via `KPU_IOC_EXPORT_DMABUF`. The
//!    arbiter (#23) routes the fd to the requesting client over the
//!    IPC broker's `SCM_RIGHTS` channel. This crate keeps the fds
//!    process-private; cross-process sharing is the arbiter's job.
//!
//! 4. **Page-locking.** Buffers are pinned at allocation time
//!    (`mlock` equivalent) so DMA never sees an unmapped page.
//!    Release `munlock`s before the deallocation ioctl.
//!
//! 5. **Concurrency.** The KPU driver supports concurrent allocation
//!    from multiple threads, but a single-process arbiter (#23) is the
//!    sole client in production. Local impl still uses `Arc<Mutex>`
//!    for the handle table so direct (non-broker) use stays safe.
//!
//! 6. **Lifecycle integration.** Buffers are torn down on
//!    [`crate::Lifecycle::teardown`] (#18). The arbiter additionally
//!    flushes the KPU fabric.

use crate::error::MemErr;
use crate::memory_provider::{BufferHandle, BufferProps, MemoryProvider};

const KPU_TARGET: &str = "kpu";

/// Stub `MemoryProvider` for the KPU silicon target.
///
/// All methods return `Err(MemErr::NotYetImplemented { target: "kpu" })`.
/// Selected at construction time when `RUST_HAL_TARGET=KPU`; otherwise
/// [`crate::SitlMemoryProvider`] is used.
#[derive(Debug, Default, Clone, Copy)]
pub struct KpuMemoryProvider;

impl MemoryProvider for KpuMemoryProvider {
    fn request(&self, _byte_size: usize, _alignment: usize) -> Result<BufferHandle, MemErr> {
        Err(MemErr::NotYetImplemented { target: KPU_TARGET })
    }

    fn release(&self, _handle: BufferHandle) -> Result<(), MemErr> {
        Err(MemErr::NotYetImplemented { target: KPU_TARGET })
    }

    fn lock(&self, _handle: BufferHandle) -> Result<(*mut u8, BufferProps), MemErr> {
        Err(MemErr::NotYetImplemented { target: KPU_TARGET })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_returns_not_yet_implemented() {
        let p = KpuMemoryProvider;
        match p.request(1024, 64) {
            Err(MemErr::NotYetImplemented { target: "kpu" }) => {}
            other => panic!("expected NotYetImplemented {{ target: \"kpu\" }}, got {other:?}"),
        }
    }

    #[test]
    fn release_returns_not_yet_implemented() {
        let p = KpuMemoryProvider;
        // The handle is a placeholder — the stub never reads it.
        match p.release(BufferHandle::from_raw(42)) {
            Err(MemErr::NotYetImplemented { target: "kpu" }) => {}
            other => panic!("expected NotYetImplemented {{ target: \"kpu\" }}, got {other:?}"),
        }
    }

    #[test]
    fn lock_returns_not_yet_implemented() {
        let p = KpuMemoryProvider;
        match p.lock(BufferHandle::from_raw(42)) {
            Err(MemErr::NotYetImplemented { target: "kpu" }) => {}
            other => panic!("expected NotYetImplemented {{ target: \"kpu\" }}, got {other:?}"),
        }
    }

    #[test]
    fn dyn_compatible_send_sync_box() {
        // Same shape the cxx bridge (#16) will use: stub stored behind
        // a Box<dyn MemoryProvider + Send + Sync>.
        let p: Box<dyn MemoryProvider + Send + Sync> = Box::new(KpuMemoryProvider);
        assert!(p.request(64, 8).is_err());
    }

    #[test]
    fn default_constructs() {
        let _p: KpuMemoryProvider = Default::default();
    }
}
