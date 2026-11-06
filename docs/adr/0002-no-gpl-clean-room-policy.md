# ADR 0002 — No GPL code; clean-room implementation from papers

**Status:** Accepted (amended 2026-10-31: scope, the structure allowance, weak copyleft rejected; amended 2026-11-06: black-box benchmarking)
**Date:** 2026-05-21
**Source:** `docs/plan/bootstrap-plan.md`, judgment call #3
**Related:** [ADR-0001](0001-third-party-license-compatibility.md) (dependency licenses), [ADR-0006](0006-clean-room-cv-stack.md)

## Context

The reference implementations for VIO and SLAM are mostly GPL-licensed:
OpenVINS and MINS (GPL-3.0), ORB-SLAM3, DBoW3, and the GPL parts of the
Sophus/g2o ecosystem. Vendoring OpenVINS as a test-only oracle was proposed,
since a separate test target would not link GPL code into the SDK.

cortex is meant to be redistributed by OEM partners (AV makers, robotics
vendors) inside their own certified products. Any GPL obligation, even one
that seems confined to tests, is something those partners' legal reviews
would have to clear.

## Decision

**No GPL-licensed code anywhere, for any purpose.** Not as an SDK
dependency, not in a test target, not as a build-time tool in what cortex
builds, and not as a ground-truth oracle. The scope is what cortex ships,
the code that builds it and the tests; the docs site's toolchain is not
part of it (see the amendment below).

- Operators are implemented **clean-room from papers and documentation**,
  plus permissively licensed (MIT / BSD / Apache-2.0) references.
- **Reading GPL source and then "writing from scratch" is not clean-room.**
  Check a reference's license *before* opening its code. A GPL project's
  structure may be consulted, meaning its directory layout and how it splits
  the problem into modules (e.g. sensor-modular decomposition), and nothing
  else: never its code, algorithm details or data (tuning tables, gains,
  coefficients, vocabularies, weights).
- **Test oracles are in-house:** synthetic trajectories, landmarks, and IMU
  noise models (`sdk/include/branes/sdk/eval/synthetic_world.hpp`), plus
  published-paper numerical thresholds. No test, CI job or build step runs a
  GPL binary or reads its output.
- **Black-box benchmarking is allowed** (amendment 2026-11-06, below): an
  unmodified GPL estimator may be run outside every Branes repository, and
  the accuracy it scores may be recorded as a target.
- **LGPL, MPL, AGPL and SSPL sources are rejected,** as dependencies and as
  references to port from, as ADR-0001 rejects them. An exception would need
  an ADR of its own that supersedes this one.
- **Amendment (2026-10-31):** the three rules above replace looser wording
  ("no GPL contact", "structure only", LGPL / MPL / SSPL "flagged for review
  and default to rejected"), to match the same policy as recorded in
  synapse, reflex, anatomy, racing-drone and scout-drone. The scope rule
  matters for the docs site: Astro brings in sharp (Apache-2.0), and sharp
  brings prebuilt libvips binaries, the `@img/sharp-libvips-*` packages
  (LGPL-3.0-or-later). They only build the documentation, in CI and on
  developers' machines, and are not part of anything cortex ships.

### Amendment (2026-11-06): black-box benchmarking

The goal for the VIO pipeline is accuracy competitive with OpenVINS on
EuRoC. Published tables don't settle that: they use their own sequence
subsets, start offsets, alignments and evaluators, so a gap between our
number and a paper's mixes the estimator with the protocol. Running the
other estimator ourselves, through our own evaluator, removes the protocol
from the comparison. GPL-3.0 governs distribution and modification, not
running a program, so running one creates no obligation for cortex.
Mixing what we learn from its internals into cortex is the risk this ADR
exists to prevent, and the conditions below keep the two apart.

**Allowed.** Running the **unmodified upstream binary** of a GPL estimator
on public datasets, built from its upstream release, **outside every Branes
repository** (for example in a container on a developer machine). An
in-house evaluator scores its output trajectories, cortex's output by the
same code, and the evaluator may live in cortex. The results may be
recorded in cortex docs, issues and PRs as **aggregate metrics**: ATE, RPE
and NEES per sequence, with the estimator's name, release and the run mode
(e.g. "OpenVINS v2.7, monocular, upstream defaults, upstream start
offsets"). Those numbers are targets, in the same standing as a paper's
published table.

**Still forbidden.**
- **Reading the estimator's source, configuration files or tuning values.**
  Running it with its shipped defaults needs none of them; only its launch
  arguments (dataset, bag, start offset, camera count, output path) may be
  set. Its settings are data in this ADR's sense, so they are never
  read, quoted or copied into cortex.
- **Taking algorithm details from it**, its source, its runs or its
  outputs. Ideas come from published papers and documentation, as before,
  and are cited.
- **Anything GPL in what cortex builds, ships or tests.** The scripts that
  fetch, build or run the binary stay outside the Branes repositories. No
  test, CI job or build step runs it, and no test asserts against its
  output; a regression target in cortex is a recorded number, like a
  published threshold.
- **Committing its raw outputs** (trajectories, logs). Only the aggregate
  metrics are recorded.

**Disclosure.** The first baseline (2026-11-06) was run before this
amendment, under the reading that an external binary was outside the rule.
In setting it up, lines of OpenVINS's EuRoC configuration file were read
(initialization thresholds and feature counts), and so was the argument
list of its launch file, including the start offset it suggests for each
sequence. The offsets were used to run it; none of the configuration values
is used in cortex. One suggestion that came from them, an initialization
trigger, was removed from issue #591. That run's comparison results are
admitted under this amendment.

The other repositories that record this policy (synapse, reflex, anatomy,
racing-drone, scout-drone) are not changed: none of them benchmarks
against a GPL estimator.

## Consequences

- Every operator costs more to build: the math has to come from the
  literature, and we have to build our own simulators to validate it.
- Each algorithm records its citations so provenance can be audited (docs
  site: *Algorithms & Provenance → Clean-room Citations*).
- OEMs can redistribute cortex under MIT terms with no copyleft review.

## Enforcement

- PR review and the CodeRabbit path instructions flag forbidden references
  (OpenVINS, MINS, ORB-SLAM3, DBoW3, Sophus/g2o) and non-permissive
  FetchContent dependencies. A benchmark result that names one of them is
  allowed under the 2026-11-06 amendment; code, settings or algorithm
  details attributed to one are not.
- Issues touching provenance carry the `clean-room` / `no-gpl` labels.
