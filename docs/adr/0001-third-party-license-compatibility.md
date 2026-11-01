# ADR 0001 — Third-party license compatibility

**Status:** Accepted (amended 2026-11-01: third-party data and published facts)
**Date:** 2026-05-21
**Issue:** #12
**Related:** [ADR-0002](0002-no-gpl-clean-room-policy.md), [ADR-0011](0011-dependencies-via-fetchcontent.md)

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

No GPL / LGPL / AGPL / MPL / SSPL dependencies are accepted in this stack. The broader no-GPL policy (which also rules out GPL code as a reference or test oracle) is [ADR-0002](0002-no-gpl-clean-room-policy.md), from bootstrap plan judgment call #3.

**Data is someone's work too.** Synthetic worlds, trajectories and fixtures made by cortex are its own. A third-party dataset or set of pretrained weights may be committed or fetched only under a license recorded here; data without one, or under a non-commercial or share-alike license (CC BY-NC, CC BY-SA, …), stays outside the repo. A dataset's published facts are not its data: calibration parameters, sensor specifications and results printed in a paper may be written into the repo's own files, with a citation to their source. A file copied from a dataset is the dataset's, and follows the rule above.

EuRoC MAV, the dataset the VIO accuracy gates run on, is "In Copyright - Non-Commercial Use Permitted" ([ETH Research Collection](https://doi.org/10.3929/ethz-b-000690084)). cortex never commits or fetches its sequences: the tests and scripts read a copy the developer supplies (`CORTEX_EUROC_*`, `CORTEX_EUROC_ROOT`). Its cam0 and IMU calibration is quoted as published facts, from Burri et al., "The EuRoC micro aerial vehicle datasets" (IJRR, 2016), in cortex-written `sensor.yaml` files (`examples/hello_vio/testdata/`) and in the tests.

## Decisions

1. **Keep cortex MIT-licensed.** No re-license needed; all chosen deps are compatible with MIT relicensing of the combined work.
2. **Preserve a `THIRD_PARTY_LICENSES/` directory in release artifacts** (OCI container, Yocto image, source tarballs). The contents are populated automatically by a packaging step that copies each dep's `LICENSE` (and `NOTICE` where present) out of the FetchContent source dirs. Concrete implementation lands with issue #96 (Docker) and #98 (Yocto).
3. **Apache-2.0 NOTICE handling.** Zenoh's `NOTICE` file must be copied verbatim into `THIRD_PARTY_LICENSES/zenoh/NOTICE`. If we ever modify Zenoh sources (e.g. for a custom transport), Apache-2.0 §4(b) requires us to add an attribution line — track that in a follow-up if/when we vendor a patched Zenoh.
4. **Third-party data and weights** are committed or fetched only under a license recorded here; the rest stays outside the repo. Published facts (calibrations, specifications, a paper's results) may be quoted, with a citation.
5. **No CLA requirement on external contributions** for now. Revisit if cortex becomes the target of patches from contributors whose employers require explicit copyright assignment.

## Reviewers

When adding a new third-party dependency, the PR must update this table and the `THIRD_PARTY_LICENSES/` plan. GPL, LGPL, AGPL, MPL and SSPL are rejected outright by the no-GPL policy ([ADR-0002](0002-no-gpl-clean-room-policy.md)). The table covers what cortex ships, the code that builds it and the tests, not the docs site's toolchain.
