//! Operator lifecycle state machine — `Unconfigured → Inactive →
//! Active → Teardown → Unconfigured`.
//!
//! Enum-backed (not type-state) because the cxx bridge surfaces this
//! as a single C++-visible type; type parameters can't cross the FFI.
//! See `docs/arch/phase1-design/README.md` § 7 for the rationale.
//!
//! Transitions return `Err(LifecycleErr::InvalidTransition)` rather
//! than panicking; the bridge maps the error to a C++ exception.
//!
//! This module ships the *transition skeleton*. Future work (deferred
//! to Phase 6 / #62) extends `configure` to accept a typed YAML
//! config struct + reactively trigger teardown when a new config
//! arrives in `Active`.

use thiserror::Error;

/// Observable state of an operator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleState {
    /// No configuration loaded; no resources allocated.
    Unconfigured,
    /// Configured + resources pre-allocated; not yet processing data.
    Inactive,
    /// Actively processing the data stream. Hot path.
    Active,
    /// Resources being released; transient state between Active /
    /// Inactive and the next Unconfigured.
    Teardown,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum LifecycleErr {
    #[error("invalid transition: from {from:?} to {to:?}")]
    InvalidTransition {
        from: LifecycleState,
        to: LifecycleState,
    },
}

/// Holder for the current lifecycle state. Each method enforces the
/// state-machine transition rules; invalid transitions return a typed
/// `Err` rather than panicking.
#[derive(Debug)]
pub struct Lifecycle {
    state: LifecycleState,
}

impl Lifecycle {
    /// Construct a fresh `Lifecycle` in `Unconfigured`.
    pub fn new() -> Self {
        Self {
            state: LifecycleState::Unconfigured,
        }
    }

    /// Observe the current state.
    pub fn state(&self) -> LifecycleState {
        self.state
    }

    /// `Unconfigured → Inactive`. Configuration consumption itself is
    /// deferred to #62 (Phase 6) — this method currently performs the
    /// state transition only.
    pub fn configure(&mut self) -> Result<(), LifecycleErr> {
        self.expect(LifecycleState::Unconfigured, LifecycleState::Inactive)
    }

    /// `Inactive → Active`.
    pub fn activate(&mut self) -> Result<(), LifecycleErr> {
        self.expect(LifecycleState::Inactive, LifecycleState::Active)
    }

    /// `Active → Inactive`. Use when pausing without releasing
    /// allocated resources.
    pub fn deactivate(&mut self) -> Result<(), LifecycleErr> {
        self.expect(LifecycleState::Active, LifecycleState::Inactive)
    }

    /// Allowed from `Inactive` *or* `Active`. Sets `Teardown`; release
    /// the underlying resources before transitioning out via `reset`.
    pub fn teardown(&mut self) -> Result<(), LifecycleErr> {
        match self.state {
            LifecycleState::Inactive | LifecycleState::Active => {
                self.state = LifecycleState::Teardown;
                Ok(())
            }
            from => Err(LifecycleErr::InvalidTransition {
                from,
                to: LifecycleState::Teardown,
            }),
        }
    }

    /// `Teardown → Unconfigured`. Closes the cycle so the operator
    /// can be re-configured with new parameters.
    pub fn reset(&mut self) -> Result<(), LifecycleErr> {
        self.expect(LifecycleState::Teardown, LifecycleState::Unconfigured)
    }

    fn expect(&mut self, from: LifecycleState, to: LifecycleState) -> Result<(), LifecycleErr> {
        if self.state == from {
            self.state = to;
            Ok(())
        } else {
            Err(LifecycleErr::InvalidTransition {
                from: self.state,
                to,
            })
        }
    }
}

impl Default for Lifecycle {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_lifecycle_is_unconfigured() {
        let l = Lifecycle::new();
        assert_eq!(l.state(), LifecycleState::Unconfigured);
    }

    #[test]
    fn full_round_trip() {
        let mut l = Lifecycle::new();
        l.configure().unwrap();
        assert_eq!(l.state(), LifecycleState::Inactive);
        l.activate().unwrap();
        assert_eq!(l.state(), LifecycleState::Active);
        l.deactivate().unwrap();
        assert_eq!(l.state(), LifecycleState::Inactive);
        l.teardown().unwrap();
        assert_eq!(l.state(), LifecycleState::Teardown);
        l.reset().unwrap();
        assert_eq!(l.state(), LifecycleState::Unconfigured);
    }

    #[test]
    fn teardown_from_active() {
        let mut l = Lifecycle::new();
        l.configure().unwrap();
        l.activate().unwrap();
        l.teardown().unwrap();
        assert_eq!(l.state(), LifecycleState::Teardown);
    }

    #[test]
    fn invalid_from_unconfigured() {
        // From Unconfigured, only `configure` is valid.
        let mut l = Lifecycle::new();
        assert!(matches!(
            l.activate(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
        assert!(matches!(
            l.deactivate(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
        assert!(matches!(
            l.teardown(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
        assert!(matches!(
            l.reset(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
    }

    #[test]
    fn invalid_from_inactive() {
        let mut l = Lifecycle::new();
        l.configure().unwrap();
        // From Inactive: only activate + teardown are valid.
        assert!(matches!(
            l.configure(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
        assert!(matches!(
            l.deactivate(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
        assert!(matches!(
            l.reset(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
    }

    #[test]
    fn invalid_from_active() {
        let mut l = Lifecycle::new();
        l.configure().unwrap();
        l.activate().unwrap();
        // From Active: only deactivate + teardown are valid.
        assert!(matches!(
            l.configure(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
        assert!(matches!(
            l.activate(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
        assert!(matches!(
            l.reset(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
    }

    #[test]
    fn invalid_from_teardown() {
        let mut l = Lifecycle::new();
        l.configure().unwrap();
        l.teardown().unwrap();
        // From Teardown: only reset is valid.
        assert!(matches!(
            l.configure(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
        assert!(matches!(
            l.activate(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
        assert!(matches!(
            l.deactivate(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
        assert!(matches!(
            l.teardown(),
            Err(LifecycleErr::InvalidTransition { .. })
        ));
    }

    #[test]
    fn err_carries_from_and_to() {
        let mut l = Lifecycle::new();
        match l.activate() {
            Err(LifecycleErr::InvalidTransition { from, to }) => {
                assert_eq!(from, LifecycleState::Unconfigured);
                assert_eq!(to, LifecycleState::Active);
            }
            other => panic!("expected InvalidTransition, got {other:?}"),
        }
    }
}
