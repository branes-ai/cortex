# Architecture Decision Records

One page per load-bearing architectural decision in cortex: what was
decided, why, and what it commits us to. These are the short, updateable
form of the decisions captured in the long-form architecture record
[`docs/arch/cortex-repo.md`](../arch/cortex-repo.md) and the judgment calls in
[`docs/plan/bootstrap-plan.md`](../plan/bootstrap-plan.md). When the code is
ambiguous, the ADRs say what was intended.

## Index

| ADR | Decision | Status |
|---|---|---|
| [0001](0001-third-party-license-compatibility.md) | Third-party license compatibility (MIT core; permissive deps only) | Accepted |
| [0002](0002-no-gpl-clean-room-policy.md) | No GPL contact for any purpose; clean-room from papers | Accepted |
| [0003](0003-hybrid-sdk-header-only-operators-compiled-glue.md) | Hybrid SDK: header-only templated operators, compiled glue | Accepted |
| [0004](0004-rm-allocates-dfa-compiler-schedules.md) | The Resource Manager allocates; the DFA compiler schedules | Accepted |
| [0005](0005-managed-lifecycle-no-dynamic-reconfiguration.md) | Managed lifecycle; no dynamic reconfiguration | Accepted |
| [0006](0006-clean-room-cv-stack.md) | Clean-room CV stack with an OpenCV-shaped API; no OpenCV | Accepted |
| [0007](0007-dual-delivery-oci-and-yocto.md) | Dual delivery: OCI containers (with CDI) and Yocto recipes | Accepted |
| [0008](0008-host-broker-thin-client-topology.md) | Host Broker + Thin Client runtime topology | Accepted |
| [0009](0009-crash-only-rm-recovery.md) | Crash-only RM recovery by client replay (no checkpointing) | Accepted |
| [0010](0010-repr-c-tensor-boundary.md) | `repr(C)` tensor metadata across the Rust/C++ boundary (not Arrow) | Accepted |
| [0011](0011-dependencies-via-fetchcontent.md) | C++ dependencies via FetchContent; no package manager | Accepted (amended) |
| [0012](0012-msckf-primary-pluggable-vio-backend.md) | MSCKF first, behind a pluggable VIO backend interface | Accepted |
| [0013](0013-analytical-jacobians.md) | Analytical Jacobians only (no AD, for now) | Accepted |
| [0014](0014-middleware-agnostic-core-zenoh-daemons.md) | Middleware-agnostic core; Zenoh daemons as the black-box surface | Accepted |
| [0015](0015-type-generic-arithmetic.md) | Arithmetic type is a first-class template parameter | Accepted |
| [0016](0016-build-target-confined-to-rust-hal.md) | SITL vs. KPU is one build flag, confined to the Rust HAL | Accepted |

**Planned:** A/B-partition + pull-based OTA updates (issue #99, Phase 10).

## By concern

- **Licensing and provenance:** 0001, 0002, 0006
- **Runtime architecture (KPU, RM):** 0004, 0008, 0009, 0010, 0016
- **SDK and math design:** 0003, 0012, 0013, 0015
- **Integration and configuration:** 0005, 0014
- **Build and delivery:** 0007, 0011

## Writing an ADR

- Copy [`template.md`](template.md) to `NNNN-short-kebab-title.md` with the
  next free number, and add it to the index above.
- One decision per ADR, about one page. Link the source discussion (issue,
  PR, or section of the architecture record) instead of repeating it.
- ADRs are append-mostly. To change a decision, either amend the ADR and say
  so in its **Status** line (small refinements), or write a new ADR and mark
  the old one **Superseded by NNNN** (reversals). Never silently rewrite
  history.
- Keep the **Current state** line honest. An ADR records intent; say plainly
  what is implemented and which issue tracks the rest.
