#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Model VIO per-operator energy via branes-ai/graphs (epic #199, Phase 2).

Reads a `vio_bench` JSON report (its `operator_profile` + measured RAPL
energy), maps each pipeline stage to a `graphs` SubgraphDescriptor, runs the
EnergyAnalyzer for each requested SKU (CPU / GPU / KPU-T64), and emits the
modeled energy/power/latency per SKU plus a modeled-vs-measured validation
against the empirical RAPL measurement.

This is a local analysis tool: it needs `branes-ai/graphs` importable
(`pip install -e` the graphs repo). It is intentionally not wired into
cortex CI — graphs is a sibling repo, not a cortex dependency. See
bench/energy_model/README.md.

    vio_energy_model.py <bench.json> [--skus cpu,gpu,kpu_t64] [--out prefix]
"""
from __future__ import annotations

import argparse
import importlib
import json
import sys

# SKU registry: name -> (graphs module, mapper factory, precision name).
# CPU/GPU default to edge-relevant parts; KPU is the T64 tile config.
SKUS = {
    "cpu": ("graphs.hardware.mappers.cpu", "create_i7_12700k_mapper", "FP32"),
    "gpu": ("graphs.hardware.mappers.gpu", "create_jetson_orin_agx_64gb_mapper", "FP16"),
    "kpu_t64": ("graphs.hardware.mappers.accelerators.kpu", "create_kpu_t64_mapper", "INT8"),
    "kpu_t256": ("graphs.hardware.mappers.accelerators.kpu", "create_kpu_t256_mapper", "INT8"),
}

# ── Pure helpers (no graphs dependency; unit-tested) ─────────────────────


def load_report(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def stage_specs(report):
    """Extract [{name, flops, bytes}] from a vio_bench operator_profile."""
    specs = []
    for s in report.get("operator_profile", []):
        specs.append({"name": str(s["name"]), "flops": float(s["flops"]), "bytes": float(s["bytes"])})
    return specs


def measured_energy(report):
    """(available, energy_j, avg_power_w) from the empirical RAPL section."""
    e = report.get("energy", {})
    return bool(e.get("rapl_available", False)), float(e.get("energy_j", 0.0)), float(e.get("avg_power_w", 0.0))


def modeled_over_measured(measured_j, modeled_j):
    """Ratio modeled/measured, or None when there is no measurement."""
    if measured_j and measured_j > 0.0:
        return modeled_j / measured_j
    return None


# ── graphs-backed estimation ─────────────────────────────────────────────


def _generic_op_type():
    ot = importlib.import_module("graphs.core.structures").OperationType
    for name in ("UNKNOWN", "ELEMENTWISE", "OTHER", "GENERIC", "CUSTOM"):
        if hasattr(ot, name):
            return getattr(ot, name)
    return list(ot)[-1]


def build_subgraphs(specs):
    """One graphs SubgraphDescriptor per VIO stage (non-NN: macs/weights 0)."""
    st = importlib.import_module("graphs.core.structures")
    op = _generic_op_type()
    subgraphs = []
    for i, s in enumerate(specs):
        subgraphs.append(
            st.SubgraphDescriptor(
                subgraph_id=i,
                node_ids=[s["name"]],
                node_names=[s["name"]],
                operation_types=[op],
                fusion_pattern=f"VIO_{s['name']}",
                total_flops=int(round(s["flops"])),
                total_macs=0,
                total_input_bytes=int(round(s["bytes"])),
                total_output_bytes=0,
                total_weight_bytes=0,
            )
        )
    return subgraphs


def make_analyzer(sku):
    module, factory, precision_name = SKUS[sku]
    mapper = getattr(importlib.import_module(module), factory)()
    rm = importlib.import_module("graphs.hardware.resource_model")
    en = importlib.import_module("graphs.estimation.energy")
    precision = getattr(rm.Precision, precision_name)
    return en.EnergyAnalyzer(mapper.resource_model, precision=precision), precision_name


def estimate_sku(specs, sku):
    analyzer, precision = make_analyzer(sku)
    report = analyzer.analyze(build_subgraphs(specs))
    per_stage = [
        {"name": s["name"], "energy_j": float(getattr(d, "total_energy_j", 0.0))}
        for s, d in zip(specs, getattr(report, "energy_descriptors", []))
    ]
    return {
        "sku": sku,
        "precision": precision,
        "modeled_energy_j": float(report.total_energy_j),
        "modeled_avg_power_w": float(report.average_power_w),
        "modeled_latency_s": float(getattr(report, "total_latency_s", 0.0)),
        "per_stage": per_stage,
    }


def run(report, skus):
    specs = stage_specs(report)
    available, measured_j, measured_w = measured_energy(report)
    results = [estimate_sku(specs, sku) for sku in skus]
    out = {
        "sequence": report.get("sequence", ""),
        "measured": {"rapl_available": available, "energy_j": measured_j, "avg_power_w": measured_w},
        "skus": results,
    }
    cpu = next((r for r in results if r["sku"] == "cpu"), None)
    if cpu and available:
        out["validation"] = {
            "cpu_modeled_energy_j": cpu["modeled_energy_j"],
            "measured_energy_j": measured_j,
            "modeled_over_measured": modeled_over_measured(measured_j, cpu["modeled_energy_j"]),
        }
    return out


# ── Report formatting (pure) ─────────────────────────────────────────────


def to_markdown(out):
    lines = [f"# VIO energy model — {out.get('sequence', '')}", ""]
    lines += ["| SKU | precision | energy (mJ) | avg power (W) | latency (ms) |", "|---|---|---|---|---|"]
    for r in out["skus"]:
        lines.append(
            f"| {r['sku']} | {r['precision']} | {r['modeled_energy_j'] * 1e3:.4g} | "
            f"{r['modeled_avg_power_w']:.4g} | {r['modeled_latency_s'] * 1e3:.4g} |"
        )
    v = out.get("validation")
    if v and v.get("modeled_over_measured") is not None:
        lines += [
            "",
            "## Modeled vs measured (CPU, RAPL)",
            "",
            f"- measured: {v['measured_energy_j'] * 1e3:.4g} mJ",
            f"- modeled:  {v['cpu_modeled_energy_j'] * 1e3:.4g} mJ",
            f"- modeled / measured: {v['modeled_over_measured']:.3f}×",
        ]
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description="Model VIO per-operator energy via branes-ai/graphs.")
    ap.add_argument("report", help="vio_bench JSON report")
    ap.add_argument("--skus", default="cpu,gpu,kpu_t64", help="comma-separated SKU names")
    ap.add_argument("--out", default="", help="write <out>.json and <out>.md")
    args = ap.parse_args(argv)

    skus = [s.strip() for s in args.skus.split(",") if s.strip()]
    unknown = [s for s in skus if s not in SKUS]
    if unknown:
        ap.error(f"unknown SKU(s): {unknown}; known: {sorted(SKUS)}")

    out = run(load_report(args.report), skus)
    sys.stdout.write(to_markdown(out))
    if args.out:
        with open(args.out + ".json", "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2)
        with open(args.out + ".md", "w", encoding="utf-8") as f:
            f.write(to_markdown(out))
        sys.stderr.write(f"wrote {args.out}.json and {args.out}.md\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
