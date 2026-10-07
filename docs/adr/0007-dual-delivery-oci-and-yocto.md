# ADR 0007 — Dual delivery: OCI containers and Yocto recipes

**Status:** Accepted
**Date:** 2026-05-20
**Source:** `docs/arch/cortex-repo.md`, *The AV/OEM Standard* and *Deployment architecture*
**Related:** [ADR-0008](0008-host-broker-thin-client-topology.md); issues #96 (OCI), #97 (CDI), #98 (Yocto), #99 (OTA ADR)

## Context

cortex is not the robot. It has to drop onto a partner's platform without
forcing a "Branes.ai OS". Partners split into two camps:

- **Agile robotics companies, startups and researchers** want something
  they can pull and run in minutes.
- **AV makers and certified OEMs** build immutable, safety-certified OS
  images (Yocto/BitBake), often don't run containers in production, and
  reject tooling their DevSecOps pipelines don't already use. Nix/NixOS was
  considered and rejected for this reason.

## Decision

Ship cortex two ways from the same source:

- **Delivery A — OCI containers** (Docker/Podman) for rapid adoption,
  e.g. `cortex-vio-daemon`.
- **Delivery B — Yocto recipes (`meta-branes`)** for OEMs. The recipe
  cross-compiles cortex against the partner's kernel headers and installs
  it as native `systemd` services in their image, with no container overhead.

For hardware access from containers, use the **Container Device Interface
(CDI)**, a static `/etc/cdi/branes.json` spec, rather than a custom OCI
runtime hook. CDI is a CNCF standard supported by Docker, Podman,
containerd, and Kubernetes, and has no custom daemon to maintain.

**Scope of CDI under the broker topology (ADR-0008):** only the privileged
Resource Manager maps `/dev/kpu`. Operator containers are unprivileged and
reach the KPU through the RM's IPC, never through device mapping.

## Consequences

- The build must cross-compile cleanly under an external toolchain, one
  reason dependencies stay under the master CMake build
  ([ADR-0011](0011-dependencies-via-fetchcontent.md)).
- Third-party license notices travel with both artifacts (ADR-0001).
- For the strictest automotive isolation, microVMs with VFIO passthrough
  remain an option but are not the default.
- **Current state:** design only. Implementation is tracked in #96–#99.
