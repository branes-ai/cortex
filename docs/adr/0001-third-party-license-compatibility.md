# ADR 0001 — Third-party license compatibility

**Status:** Accepted
**Date:** 2026-05-21
**Issue:** #12

## Context

`cortex` is MIT-licensed (see top-level `LICENSE`). It links against a small set of third-party libraries fetched via CMake `FetchContent` and one Rust crate (`cxx`) brought in by Corrosion. Before any of those dependencies actually land, this ADR records the license posture so distribution packaging (issues #96 OCI containers, #98 Yocto recipe) inherits a clear plan.

## Dependency license matrix

| Dependency        | License                         | Compatible with MIT cortex? | Distribution obligation |
|-------------------|---------------------------------|-----------------------------|--------------------------|
| MTL5              | MIT                             | Yes                         | Bundle their LICENSE in distribution. |
| Universal         | MIT                             | Yes                         | Bundle their LICENSE in distribution. |
| yaml-cpp          | MIT                             | Yes                         | Bundle their LICENSE in distribution. |
| Catch2 v3         | BSL-1.0                         | Yes                         | BSL-1.0 waives notice requirements for object-code redistribution; bundling for source distribution is best practice anyway. |
| Tracy             | BSD-3-Clause                    | Yes                         | Bundle their LICENSE; do not use authors' names for endorsement (3rd clause). |
| stb (stb_image)   | MIT **or** Public Domain (dual) | Yes                         | We pick the MIT branch; bundle the in-file notice from `stb_image.h`. |
| Zenoh-cpp / -c    | Apache-2.0 **or** EPL-2.0 (dual)| Yes (Apache-2.0 branch)     | **Preserve upstream `NOTICE` file** in distribution (Apache-2.0 §4 requirement). |
| `cxx` (Rust crate)| MIT **or** Apache-2.0 (dual)    | Yes                         | Bundle MIT notice with the Rust component. |

No GPL / LGPL / MPL / SSPL dependencies are accepted in this stack — see [[cortex-no-gpl]] memory record and bootstrap plan judgment call #3.

## Decisions

1. **Keep cortex MIT-licensed.** No re-license needed; all chosen deps are compatible with MIT relicensing of the combined work.
2. **Preserve a `THIRD_PARTY_LICENSES/` directory in release artifacts** (OCI container, Yocto image, source tarballs). The contents are populated automatically by a packaging step that copies each dep's `LICENSE` (and `NOTICE` where present) out of the FetchContent source dirs. Concrete implementation lands with issue #96 (Docker) and #98 (Yocto).
3. **Apache-2.0 NOTICE handling.** Zenoh's `NOTICE` file must be copied verbatim into `THIRD_PARTY_LICENSES/zenoh/NOTICE`. If we ever modify Zenoh sources (e.g. for a custom transport), Apache-2.0 §4(b) requires us to add an attribution line — track that in a follow-up if/when we vendor a patched Zenoh.
4. **No CLA requirement on external contributions** for now. Revisit if cortex becomes the target of patches from contributors whose employers require explicit copyright assignment.

## Reviewers

When adding a new third-party dependency, the PR must update this table and the `THIRD_PARTY_LICENSES/` plan. Strong-copyleft licenses (GPL family, AGPL, SSPL) are rejected outright by the no-GPL policy.
