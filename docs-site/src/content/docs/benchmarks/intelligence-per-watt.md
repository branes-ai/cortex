---
title: Intelligence per Watt (vio_bench)
description: The one-command VIO benchmark that ties trajectory accuracy and throughput to energy — empirically (RAPL) and, later, via graphs model estimation.
---

`vio_bench` runs one EuRoC sequence through the [`VioEstimator`](/vio/vio-estimator/)
and reports the industry VIO metrics **plus empirical energy**, so accuracy and
throughput can be weighed against power — the embodied-AI figure of merit. It lives
in `bench/` (`branes::bench`) and is part of epic #199.

## The metric

There is no single industry-standard energy number for VIO, so `vio_bench` reports a
**triple plus a derived ratio**:

- **Accuracy** — ATE (m), RPE RMSE (m), and KITTI-style RPE drift (translation %,
  rotation deg/m), via [`branes::sdk::eval`](/benchmarks/accuracy/).
- **Performance** — per-frame `feed_image` latency p50/p99, real-time factor
  (sequence duration ÷ processing time), throughput (fps).
- **Energy (empirical)** — Linux **RAPL/powercap** energy measured around the run →
  J/frame and average W.
- **Efficiency** — **accuracy-bounded fps/W**: throughput per Watt, only counted when
  ATE ≤ the gate, so you can't "win" by running fast and wrong.

## Running it

```bash
# Build the optimized tool (energy budgets are only meaningful with -O2).
cmake --preset sitl-release && cmake --build --preset sitl-release --target vio_bench

# Run a sequence (the mav0 directory of an EuRoC sequence).
./build/sitl-release/bench/vio_bench /data/EuRoC/V1_01_easy/mav0 --out v101

# v101.json (machine-readable) + v101.md (human-readable) are written.
```

**Energy access:** RAPL's `energy_uj` is often **root-readable only** (since
CVE-2020-8694). Run with energy-read access (e.g. `sudo`, or relax the powercap
permissions) to get J/W; otherwise the report degrades gracefully and marks energy
*unavailable* while still reporting accuracy/latency/RTF.

## How energy is (and will be) measured

| Backend | Status | What it gives |
|---|---|---|
| **RAPL / powercap** (x86 host) | ✅ Phase 1 | Measured CPU package + DRAM energy → J/frame, avg W |
| **`graphs` model estimation** | ⏳ Phase 2 | Modeled energy/power for CPU/GPU SKUs from the per-operator profile; **validated against the RAPL measurement** |
| **KPU model (`graphs`)** + Jetson `tegrastats` | ⏳ Phase 3 | Projected intelligence/W on the KPU target; on-device measurement |

The bridge to model-based estimation is the **per-operator profile** `vio_bench`
emits in its JSON (`operator_profile`: first-order FLOPs/bytes per stage — pyramid,
FAST, KLT, MSCKF propagate/update — from the observed runtime counts). The Phase-2
harness maps each stage to a [`graphs`](https://github.com/branes-ai/graphs)
`SubgraphDescriptor` and runs its `EnergyAnalyzer`; comparing **modeled-CPU vs
measured-RAPL** validates the models before their **KPU** projections are trusted.

## What's first-order today

The operator FLOP/byte coefficients are analytic estimates (ops per pixel / per
feature / per matrix element), not measured FLOP counts, and the per-stage timing is
not yet broken out (only aggregate `feed_image` latency). Both are honest inputs for
energy *modeling* and are slated to be refined with hardware performance counters.
