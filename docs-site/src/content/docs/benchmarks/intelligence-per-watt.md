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

### Energy backend (`--energy`)

The empirical energy source is pluggable, so the same benchmark runs on the host
and on the edge/KPU target:

| `--energy` | Where | How |
|---|---|---|
| `rapl` (default) | x86 host | Linux RAPL/powercap counter delta |
| `tegrastats` | Jetson | spawns `tegrastats --interval <ms>` (`--energy-source <ms>`) and integrates total board power over the run |
| `external` | instrumented rig | a cumulative-µJ counter file (`--energy-source <path>`) from an INA/Joulescope exporter or a board rail |

```bash
# Jetson: sample power every 50 ms while replaying
vio_bench .../mav0 --energy tegrastats --energy-source 50 --out v101

# External meter exposing cumulative micro-joules at a path
vio_bench .../mav0 --energy external --energy-source /run/power/energy_uj --out v101
```

Each backend degrades gracefully (energy marked *unavailable*) when its source can't
be read, so accuracy/latency/RTF are always reported.

**Energy access:** RAPL's `energy_uj` is often **root-readable only** (since
CVE-2020-8694). Run with energy-read access (e.g. `sudo`, or relax the powercap
permissions) to get J/W; otherwise the report degrades gracefully and marks energy
*unavailable* while still reporting accuracy/latency/RTF.

## How energy is (and will be) measured

| Backend | Status | What it gives |
|---|---|---|
| **RAPL / powercap** (x86 host) | ✅ Phase 1 | Measured CPU package + DRAM energy → J/frame, avg W |
| **`graphs` model estimation** | ✅ Phase 2 | Modeled energy/power/latency for CPU, GPU, **and KPU (T64)** SKUs from the per-operator profile; CPU **validated against the RAPL measurement** |
| **Jetson `tegrastats` + external meter** | ✅ Phase 3 | On-device empirical measurement on the edge target, via `--energy tegrastats` / `--energy external` (see above) |

The bridge to model-based estimation is the **per-operator profile** `vio_bench`
emits in its JSON (`operator_profile`: first-order FLOPs/bytes per stage — pyramid,
FAST, KLT, MSCKF propagate/update — from the observed runtime counts).

### Modeled energy across SKUs (Phase 2)

`bench/energy_model/vio_energy_model.py` reads that JSON, maps each stage to a
[`graphs`](https://github.com/branes-ai/graphs) `SubgraphDescriptor`, and runs its
`EnergyAnalyzer` for each SKU:

```bash
python bench/energy_model/vio_energy_model.py v101.json --skus cpu,gpu,kpu_t64 --out v101_energy
```

It reports modeled energy / power / latency per SKU (CPU `i7-12700k`, GPU
`Jetson Orin AGX`, KPU `T64`), and compares **modeled-CPU vs measured-RAPL** so the
model is validated on hardware we can measure before its **KPU** projection is
trusted. The harness needs `graphs` importable, so it's a local tool, not a CI gate
(see `bench/energy_model/README.md`).

> The RAPL measurement is the **whole CPU package** over the run; the model estimates
> only the **VIO compute**, so `modeled ≤ measured` is expected — the ratio is a
> sanity check, not a strict equality.

## What's first-order today

The operator FLOP/byte coefficients are analytic estimates (ops per pixel / per
feature / per matrix element), not measured FLOP counts, and the per-stage timing is
not yet broken out (only aggregate `feed_image` latency). Both are honest inputs for
energy *modeling* and are slated to be refined with hardware performance counters.
