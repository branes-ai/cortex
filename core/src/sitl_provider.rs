//! Host-emulated `MemoryProvider` implementation.
//!
//! Two backends:
//!
//! - **shm** (Linux default): `shm_open` + `mmap` over an anonymous
//!   shared file descriptor; `shm_unlink` immediately so the name
//!   isn't visible in `/dev/shm`. Forward-compatible with the
//!   E1-DEFERRED IPC broker (#21), which will pass these fds across
//!   process boundaries via `SCM_RIGHTS`.
//! - **heap** (always-available fallback): `std::alloc::alloc_zeroed`.
//!   Selected when the env var `CORTEX_MEM_BACKEND=heap` is set, or
//!   automatically on non-Linux targets where the shm backend is
//!   absent. Cheap for unit tests where shm-setup costs aren't worth
//!   it.
//!
//! See `docs/arch/phase1-design/README.md` § 8 for the design
//! rationale (invariants, error contract, max alloc size, etc.).
//!
//! Windows support (CreateFileMappingA + MapViewOfFile) is a follow-up
//! — the Phase 0 CI matrix is Linux-only and the design-doc Windows
//! snippet has not been exercised yet. Tracked as a TODO inline.

use crate::error::MemErr;
use crate::memory_provider::{BufferHandle, BufferProps, MemoryProvider};
use std::alloc::{alloc_zeroed, dealloc, Layout};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// Maximum single allocation: 4 GiB. Documented in design-doc § 8.3.
/// Larger requests return [`MemErr::SizeExceeded`].
const MAX_ALLOC: usize = 1usize << 32;

/// Environment variable that forces the heap backend over the OS shm
/// backend. Useful for unit tests where shm setup overhead isn't
/// justified, and for non-Linux targets where shm isn't available.
const BACKEND_ENV: &str = "CORTEX_MEM_BACKEND";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Backend {
    Shm,
    Heap,
}

fn default_backend() -> Backend {
    if let Ok(v) = std::env::var(BACKEND_ENV) {
        if v.eq_ignore_ascii_case("heap") {
            return Backend::Heap;
        }
    }
    if cfg!(target_os = "linux") {
        Backend::Shm
    } else {
        // No shm impl for non-Linux yet. Heap is the universal fallback.
        Backend::Heap
    }
}

/// One row of the provider's handle table. Holds whatever the
/// concrete backend needs to release the allocation on `release` /
/// `Drop`.
enum Entry {
    /// Heap-backed allocation. Layout retained so `dealloc` matches
    /// the original `alloc_zeroed` shape.
    Heap { ptr: *mut u8, layout: Layout },

    /// Linux shm-backed allocation. `ptr` is the mmap'd region; we
    /// keep the byte size for `munmap`. Caller-requested alignment
    /// lives in the sibling `BufferProps`; not stored here because the
    /// release path doesn't need it. The fd has already been
    /// `shm_unlink`'d at allocation time.
    #[cfg(target_os = "linux")]
    Shm { ptr: *mut u8, byte_size: usize },
}

// SAFETY: Entry holds raw pointers that the provider treats as
// allocation tokens, not as dereferenceable Rust references. The
// provider's Mutex serialises all access; the underlying memory is
// not shared between threads except via the explicit `lock` API,
// which the caller is responsible for using correctly.
unsafe impl Send for Entry {}
unsafe impl Sync for Entry {}

struct Inner {
    backend: Backend,
    table: Mutex<HashMap<u64, EntryWithProps>>,
    next_id: AtomicU64,
}

struct EntryWithProps {
    entry: Entry,
    props: BufferProps,
}

/// Host-side `MemoryProvider`. Cheap to clone (one `Arc` bump).
#[derive(Clone)]
pub struct SitlMemoryProvider {
    inner: Arc<Inner>,
}

impl SitlMemoryProvider {
    /// Construct a provider using the backend selected by env var
    /// (defaults to shm on Linux, heap elsewhere).
    pub fn new() -> Self {
        Self::with_backend(default_backend())
    }

    fn with_backend(backend: Backend) -> Self {
        Self {
            inner: Arc::new(Inner {
                backend,
                table: Mutex::new(HashMap::new()),
                next_id: AtomicU64::new(1),
            }),
        }
    }

    /// Force the heap backend regardless of env. Used by tests so
    /// they don't pollute `/dev/shm` or require root.
    #[doc(hidden)]
    pub fn new_heap() -> Self {
        Self::with_backend(Backend::Heap)
    }
}

impl Default for SitlMemoryProvider {
    fn default() -> Self {
        Self::new()
    }
}

fn validate_alignment(alignment: usize) -> Result<(), MemErr> {
    if alignment == 0 || !alignment.is_power_of_two() {
        return Err(MemErr::AlignmentNotPow2 { alignment });
    }
    Ok(())
}

fn validate_size(byte_size: usize) -> Result<(), MemErr> {
    if byte_size > MAX_ALLOC {
        return Err(MemErr::SizeExceeded {
            byte_size,
            limit: MAX_ALLOC,
        });
    }
    Ok(())
}

impl MemoryProvider for SitlMemoryProvider {
    fn request(&self, byte_size: usize, alignment: usize) -> Result<BufferHandle, MemErr> {
        validate_alignment(alignment)?;
        validate_size(byte_size)?;

        // Heap path is universal — used both for the heap backend and
        // when running on a non-Linux host.
        let (entry, props) = match self.inner.backend {
            Backend::Heap => alloc_heap(byte_size, alignment)?,
            #[cfg(target_os = "linux")]
            Backend::Shm => alloc_shm(byte_size, alignment)?,
            #[cfg(not(target_os = "linux"))]
            Backend::Shm => return Err(MemErr::NotSupported { target: "shm" }),
        };

        let id = self.inner.next_id.fetch_add(1, Ordering::Relaxed);
        let mut table = self.inner.table.lock().unwrap_or_else(|p| p.into_inner());
        table.insert(id, EntryWithProps { entry, props });
        Ok(BufferHandle::from_raw(id))
    }

    fn release(&self, handle: BufferHandle) -> Result<(), MemErr> {
        let mut table = self.inner.table.lock().unwrap_or_else(|p| p.into_inner());
        let entry_props = table
            .remove(&handle.as_raw())
            .ok_or(MemErr::InvalidHandle(handle))?;
        drop(table); // free the lock before the (potentially-blocking) syscall
        free_entry(entry_props.entry);
        Ok(())
    }

    fn lock(&self, handle: BufferHandle) -> Result<(*mut u8, BufferProps), MemErr> {
        let table = self.inner.table.lock().unwrap_or_else(|p| p.into_inner());
        let entry_props = table
            .get(&handle.as_raw())
            .ok_or(MemErr::InvalidHandle(handle))?;
        let ptr = match &entry_props.entry {
            Entry::Heap { ptr, .. } => *ptr,
            #[cfg(target_os = "linux")]
            Entry::Shm { ptr, .. } => *ptr,
        };
        Ok((ptr, entry_props.props))
    }
}

impl Drop for Inner {
    fn drop(&mut self) {
        // Release every still-outstanding allocation. We don't return
        // errors here because Drop can't; if the OS munmap fails the
        // process is exiting anyway.
        if let Ok(mut table) = self.table.lock() {
            for (_, entry_props) in table.drain() {
                free_entry(entry_props.entry);
            }
        }
    }
}

fn alloc_heap(byte_size: usize, alignment: usize) -> Result<(Entry, BufferProps), MemErr> {
    // Layout::from_size_align rejects size == 0 and non-power-of-two
    // alignment, both of which we've already filtered. But size == 0
    // is allowed by the trait contract — round up to the alignment
    // so we get a usable distinct pointer.
    let effective_size = byte_size.max(alignment).max(1);
    let layout = Layout::from_size_align(effective_size, alignment)
        .map_err(|_| MemErr::AlignmentNotPow2 { alignment })?;

    // SAFETY: alloc_zeroed with a valid non-zero-sized Layout.
    let ptr = unsafe { alloc_zeroed(layout) };
    if ptr.is_null() {
        return Err(MemErr::OutOfMemory {
            byte_size,
            alignment,
        });
    }

    let props = BufferProps {
        byte_size: effective_size,
        alignment,
    };
    Ok((Entry::Heap { ptr, layout }, props))
}

#[cfg(target_os = "linux")]
fn alloc_shm(byte_size: usize, alignment: usize) -> Result<(Entry, BufferProps), MemErr> {
    use nix::fcntl::OFlag;
    use nix::sys::mman::{mmap, shm_open, shm_unlink, MapFlags, ProtFlags};
    use nix::sys::stat::Mode;
    use nix::unistd::ftruncate;
    use std::num::NonZeroUsize;
    use std::os::fd::AsRawFd;

    // shm_open requires a name. Generate one unique to this process +
    // allocation count; shm_unlink it immediately so /dev/shm doesn't
    // accumulate stale entries.
    static SHM_COUNTER: AtomicU64 = AtomicU64::new(0);
    let name = format!(
        "/cortex-{}-{}",
        std::process::id(),
        SHM_COUNTER.fetch_add(1, Ordering::Relaxed),
    );

    let fd = shm_open(
        name.as_str(),
        OFlag::O_RDWR | OFlag::O_CREAT | OFlag::O_EXCL,
        Mode::S_IRUSR | Mode::S_IWUSR,
    )
    .map_err(|errno| MemErr::Os {
        source: std::io::Error::from_raw_os_error(errno as i32),
    })?;

    // Mark the name for deletion now; the mapping survives via the fd.
    let _ = shm_unlink(name.as_str());

    // Round size up to the page boundary for mmap; alignment beyond
    // that is satisfied automatically (mmap returns page-aligned
    // pointers, which is at least 4096-byte alignment).
    let page = page_size();
    let effective_size = byte_size.next_multiple_of(page).max(page);

    let raw_fd = fd.as_raw_fd();
    if let Err(errno) = ftruncate(&fd, effective_size as libc::off_t) {
        return Err(MemErr::Os {
            source: std::io::Error::from_raw_os_error(errno as i32),
        });
    }

    let size_nz = NonZeroUsize::new(effective_size).expect("effective_size >= page > 0");
    // SAFETY: fd was just opened and ftruncated to effective_size; we
    // are the only owner. The mmap returns a fresh mapping in our
    // address space.
    let map = unsafe {
        mmap(
            None,
            size_nz,
            ProtFlags::PROT_READ | ProtFlags::PROT_WRITE,
            MapFlags::MAP_SHARED,
            &fd,
            0,
        )
    }
    .map_err(|errno| MemErr::Os {
        source: std::io::Error::from_raw_os_error(errno as i32),
    })?;

    // Verify the mmap pointer meets the requested alignment. mmap
    // always page-aligns; if the caller asked for ≤ page alignment,
    // we're fine. If they asked for more than a page, fall through to
    // an error rather than silently mis-aligning.
    let ptr = map.as_ptr() as *mut u8;
    if (ptr as usize) % alignment != 0 {
        // Unmap and fail. SAFETY: ptr is the recent successful mmap.
        unsafe {
            let _ = nix::sys::mman::munmap(map, effective_size);
        }
        return Err(MemErr::NotSupported {
            target: "shm-overalignment",
        });
    }

    // Keep fd alive by leaking it: the mmap holds the underlying
    // resource. Closing fd is fine as long as the mapping persists,
    // but leaking is simpler and matches anonymous-shm conventions.
    let _ = raw_fd; // explicitly unused
    std::mem::forget(fd);

    let props = BufferProps {
        byte_size: effective_size,
        alignment: alignment.max(page),
    };
    Ok((
        Entry::Shm {
            ptr,
            byte_size: effective_size,
        },
        props,
    ))
}

#[cfg(target_os = "linux")]
fn page_size() -> usize {
    // SAFETY: sysconf with a defined parameter name returns a long.
    let n = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
    if n <= 0 {
        4096 // sane fallback for any conceivable host
    } else {
        n as usize
    }
}

fn free_entry(entry: Entry) {
    match entry {
        Entry::Heap { ptr, layout } => {
            if !ptr.is_null() {
                // SAFETY: matches the alloc_zeroed in alloc_heap.
                unsafe { dealloc(ptr, layout) };
            }
        }
        #[cfg(target_os = "linux")]
        Entry::Shm { ptr, byte_size } => {
            use std::ptr::NonNull;
            if let Some(nn) = NonNull::new(ptr.cast()) {
                // SAFETY: ptr came from a successful mmap of byte_size.
                unsafe {
                    let _ = nix::sys::mman::munmap(nn, byte_size);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn heap_provider() -> SitlMemoryProvider {
        SitlMemoryProvider::new_heap()
    }

    #[test]
    fn alloc_lock_write_release_round_trip() {
        let p = heap_provider();
        let h = p.request(16 * 1024 * 1024, 64).expect("16 MiB alloc");
        let (ptr, props) = p.lock(h).expect("lock");
        assert!(props.byte_size >= 16 * 1024 * 1024);
        assert!(props.alignment >= 64);
        // SAFETY: provider owns ptr, byte_size is correct by props.
        let slice = unsafe { std::slice::from_raw_parts_mut(ptr, props.byte_size) };
        slice.fill(0xA5);
        assert_eq!(slice[0], 0xA5);
        assert_eq!(slice[props.byte_size - 1], 0xA5);
        p.release(h).expect("release");
    }

    #[test]
    fn lock_is_idempotent() {
        let p = heap_provider();
        let h = p.request(4096, 8).unwrap();
        let (ptr1, props1) = p.lock(h).unwrap();
        let (ptr2, props2) = p.lock(h).unwrap();
        assert_eq!(ptr1, ptr2);
        assert_eq!(props1.byte_size, props2.byte_size);
        assert_eq!(props1.alignment, props2.alignment);
        p.release(h).unwrap();
    }

    #[test]
    fn double_release_returns_invalid_handle() {
        let p = heap_provider();
        let h = p.request(64, 8).unwrap();
        p.release(h).unwrap();
        match p.release(h) {
            Err(MemErr::InvalidHandle(_)) => {}
            other => panic!("expected InvalidHandle, got {other:?}"),
        }
    }

    #[test]
    fn lock_after_release_fails() {
        let p = heap_provider();
        let h = p.request(64, 8).unwrap();
        p.release(h).unwrap();
        match p.lock(h) {
            Err(MemErr::InvalidHandle(_)) => {}
            other => panic!("expected InvalidHandle, got {other:?}"),
        }
    }

    #[test]
    fn non_power_of_two_alignment_rejected() {
        let p = heap_provider();
        match p.request(64, 3) {
            Err(MemErr::AlignmentNotPow2 { alignment: 3 }) => {}
            other => panic!("expected AlignmentNotPow2, got {other:?}"),
        }
    }

    #[test]
    fn zero_alignment_rejected() {
        let p = heap_provider();
        match p.request(64, 0) {
            Err(MemErr::AlignmentNotPow2 { alignment: 0 }) => {}
            other => panic!("expected AlignmentNotPow2, got {other:?}"),
        }
    }

    #[test]
    fn size_over_4gib_rejected() {
        let p = heap_provider();
        match p.request(MAX_ALLOC + 1, 8) {
            Err(MemErr::SizeExceeded { limit, .. }) => assert_eq!(limit, MAX_ALLOC),
            other => panic!("expected SizeExceeded, got {other:?}"),
        }
    }

    #[test]
    fn zero_byte_size_returns_valid_handle() {
        let p = heap_provider();
        let h = p.request(0, 8).expect("zero-size alloc");
        let (ptr, props) = p.lock(h).expect("lock zero-size");
        assert!(!ptr.is_null());
        // Round up to at least the requested alignment.
        assert!(props.byte_size >= 8);
        p.release(h).unwrap();
    }

    #[test]
    fn handle_ids_are_unique() {
        let p = heap_provider();
        let a = p.request(64, 8).unwrap();
        let b = p.request(64, 8).unwrap();
        assert_ne!(a, b);
        p.release(a).unwrap();
        let c = p.request(64, 8).unwrap();
        // Unique even after release — invariant 3.
        assert_ne!(a, c);
        assert_ne!(b, c);
        p.release(b).unwrap();
        p.release(c).unwrap();
    }

    #[test]
    fn concurrent_requests_are_safe() {
        use std::thread;
        let p = heap_provider();
        let threads: Vec<_> = (0..8)
            .map(|_| {
                let p = p.clone();
                thread::spawn(move || {
                    let mut handles = Vec::new();
                    for _ in 0..32 {
                        handles.push(p.request(1024, 16).unwrap());
                    }
                    for h in handles {
                        p.release(h).unwrap();
                    }
                })
            })
            .collect();
        for t in threads {
            t.join().unwrap();
        }
    }

    #[test]
    fn drop_releases_outstanding_allocations() {
        let p = heap_provider();
        // Allocate without releasing.
        let _ = p.request(4096, 8).unwrap();
        let _ = p.request(4096, 8).unwrap();
        // Drop the provider; Inner's Drop impl should clean up the
        // table. Valgrind / Miri would catch a leak here; for the unit
        // test we just verify we don't panic.
        drop(p);
    }

    // Linux-only shm-backend round-trip. Compiled out on other targets.
    #[cfg(target_os = "linux")]
    #[test]
    fn shm_backend_round_trip() {
        let p = SitlMemoryProvider::with_backend(Backend::Shm);
        let h = p.request(16 * 1024 * 1024, 64).unwrap();
        let (ptr, props) = p.lock(h).unwrap();
        // SAFETY: provider owns ptr, byte_size is correct.
        let slice = unsafe { std::slice::from_raw_parts_mut(ptr, props.byte_size) };
        slice.fill(0x5A);
        assert_eq!(slice[0], 0x5A);
        assert_eq!(slice[props.byte_size - 1], 0x5A);
        p.release(h).unwrap();
    }

    #[test]
    fn provider_is_dyn_compatible_and_send_sync() {
        let p: Box<dyn MemoryProvider + Send + Sync> = Box::new(heap_provider());
        let h = p.request(64, 8).unwrap();
        p.release(h).unwrap();
    }
}
