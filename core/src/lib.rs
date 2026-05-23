//! cortex-core — Rust Resource Manager.
//!
//! Placeholder crate. The `MemoryProvider` trait, the SITL and KPU
//! implementations, the lifecycle state machine, and the `cxx::bridge`
//! land in issues #13-#20 (Phase 1).

#[cfg(test)]
mod tests {
    /// Placeholder so the test binary has at least one symbol and
    /// `cargo-llvm-cov` can emit a profile. Phase 1 (#13-#20) replaces
    /// this with real lifecycle / provider / bridge tests.
    #[test]
    fn placeholder_compiles() {}
}
