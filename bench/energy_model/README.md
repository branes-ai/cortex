# vio_energy_model — modeled VIO energy via `branes-ai/graphs`

Phase 2 of the VIO intelligence-per-Watt benchmark (epic #199). Turns
[`vio_bench`](../) 's per-operator profile into **modeled** energy across
hardware SKUs — CPU, GPU, and **KPU (T64)** — and validates the model
against the empirical RAPL measurement.

## Prerequisites

`branes-ai/graphs` must be importable (it's a sibling repo, **not** a cortex
dependency, so this tool is not part of cortex CI):

```bash
pip install -e /path/to/branes/clones/graphs
```

## Usage

```bash
# 1) produce a vio_bench report (Phase 1)
vio_bench /data/EuRoC/V1_01_easy/mav0 --out v101      # writes v101.json

# 2) model its energy across SKUs
python bench/energy_model/vio_energy_model.py v101.json --skus cpu,gpu,kpu_t64 --out v101_energy
```

Output (Markdown + JSON) gives, per SKU, the modeled **energy (mJ)**, **average
power (W)**, and **latency (ms)**, plus a CPU **modeled-vs-measured** ratio.

### SKUs

| name | graphs mapper | precision |
|---|---|---|
| `cpu` | `create_i7_12700k_mapper` | FP32 |
| `gpu` | `create_jetson_orin_agx_64gb_mapper` | FP16 |
| `kpu_t64` | `create_kpu_t64_mapper` | INT8 |
| `kpu_t256` | `create_kpu_t256_mapper` | INT8 |

## How it works

Each stage in the report's `operator_profile` (pyramid / FAST / KLT / MSCKF
propagate+update, with first-order FLOPs+bytes) becomes a `graphs`
`SubgraphDescriptor` (`total_macs = 0`, no weights — VIO is hand-written, not
a neural net). `graphs`' `EnergyAnalyzer` then estimates energy/power/latency
for the chosen SKU's resource model.

## On modeled-vs-measured

The RAPL measurement from `vio_bench` is the **whole CPU package** over the
run (idle + OS + the estimator), while the model estimates only the **VIO
compute**. So `modeled ≤ measured` is expected; the ratio is a sanity check,
not a strict equality. Tightening it (isolating VIO compute energy as a delta
over an idle baseline, and per-stage measured timing) is future work — see the
Phase-1 first-order caveats.

## Tests

```bash
cd bench/energy_model && python -m unittest -v
```

The pure helpers always run; the `graphs`-backed cases are skipped when
`graphs` isn't importable.
