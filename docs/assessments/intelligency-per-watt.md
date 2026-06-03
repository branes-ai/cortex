# Intelligence per Watt benchmark

We need a benchmark that measured intelligence per Watt for the VIO. 
What does the industry articulate as accuracy or performance to track vio capability? 
Do we have an easy to run benchmark to measure these metrics with our stack? With a competing stack like OpenVINS or MINS?

 Confirmed the licensing point first, because it shapes the answer: OpenVINS and MINS are both GPL-3.0 (RPNG / U. Delaware). Under cortex's strict no-GPL
rule, that means we can use them only as external black-box baselines — run their binary on a dataset, take the trajectory file it emits, and compare —
never vendor, link, or copy their source. Running a GPL program and comparing its output doesn't make cortex a derivative work; copying code would.

## What the industry actually tracks for VIO

Three axes, and serious papers report all three:

1. Accuracy (the headline)
- ATE — Absolute Trajectory Error, translational RMSE after a rigid (SE3) / similarity (Sim3, for mono) alignment. The de-facto VIO number, reported per
EuRoC sequence and averaged. Current code contains an ARE metric.
- RPE — Relative Pose Error over a fixed time/distance step; KITTI-style reports it as translational drift % and rotational drift deg/m. Captures local
drift independent of global alignment. Current code contains an RPE metric.
- Scale error for monocular; NEES / consistency (normalized estimation error squared vs. the χ² bound) — does the filter's covariance tell the truth, not
just its mean. OpenVINS leans on NEES heavily. (We don't have NEES yet.)

2. Robustness
- Completion/divergence rate across a dataset, and across aggressive datasets (UZH-FPV drone racing, TUM-VI) — many stacks ace EuRoC and fall over on fast
motion.

3. Performance
- Per-frame latency (median / p99) (we have this), real-time factor (dataset duration ÷ processing time; >1 = faster than real time), CPU %, memory,
throughput (fps). (RTF we don't report yet.)

## Standard datasets 

EuRoC MAV (the benchmark everyone reports), TUM-VI, UZH-FPV, KITTI, 4Seasons, OpenLORIS.

## "Intelligence per Watt" for VIO

There is no single industry-standard energy metric for VIO — academia rarely reports power. This is precisely the Branes gap to own. The honest formulation
is a triple plus a derived ratio, not one magic number:

- measure (ATE, throughput fps, average power W) on a fixed sequence/platform, then derive:
- energy per pose = J / frame (the cleanest "cost of one estimate"),
- accuracy-bounded efficiency = fps ÷ W subject to ATE ≤ threshold (so you can't "win" by going fast and wrong) — the VIO analog of TOPS/W.

Measuring power is the missing piece, and it's easy on the host: Linux RAPL / powercap exposes cumulative package + DRAM energy at
/sys/class/powercap/intel-rapl:*/energy_uj (AMD has equivalents on recent kernels). Read it before/after the processing loop → ΔJ; divide by frames and
wall-time → J/frame and average W, no extra hardware. On the KPU/embedded target you'd use the SoC's power telemetry or an external rail meter
(Joulescope/INA219); that's the same harness with a different energy backend.

## What we have vs. the gaps

┌───────────────────────────────────────────────────────────┬────────────────────────────────────────────────────┐
│                        Capability                         │                       Status                       │
├───────────────────────────────────────────────────────────┼────────────────────────────────────────────────────┤
│ ATE / RPE metrics (clean-room)                            │ ✅ branes::sdk::eval                               │
├───────────────────────────────────────────────────────────┼────────────────────────────────────────────────────┤
│ EuRoC replay harness                                      │ ✅ branes::sdk::euroc (gated on CORTEX_EUROC_V101) │
├───────────────────────────────────────────────────────────┼────────────────────────────────────────────────────┤
│ Per-frame latency (median/p99)                            │ ✅ sdk_vio_latency_test                            │
├───────────────────────────────────────────────────────────┼────────────────────────────────────────────────────┤
│ Real-time factor                                          │ ❌                                                 │
├───────────────────────────────────────────────────────────┼────────────────────────────────────────────────────┤
│ Energy / power (J/frame, W) via RAPL                      │ ❌                                                 │
├───────────────────────────────────────────────────────────┼────────────────────────────────────────────────────┤
│ One-command "run sequence → report" tool + JSON/MD output │ ❌ (it's split across tests)                       │
├───────────────────────────────────────────────────────────┼────────────────────────────────────────────────────┤
│ NEES / consistency                                        │ ❌                                                 │
├───────────────────────────────────────────────────────────┼────────────────────────────────────────────────────┤
│ OpenVINS/MINS black-box comparison                        │ ❌                                                 │
└───────────────────────────────────────────────────────────┴────────────────────────────────────────────────────┘

So today you can already get ATE, RPE, and latency on EuRoC with our stack — but there's no single command that produces the intelligence-per-Watt report,
and nothing measures energy or compares against a baseline.

## The benchmark I'd build

A small vio_bench driver (extends the existing EuRoC replay) that runs a sequence through VioEstimator once and emits a JSON + Markdown report with: ATE,
RPE (% and deg/m), median/p99 latency, real-time factor, average power (RAPL), J/frame, and the accuracy-bounded fps/W. Optionally a --baseline mode that
runs OpenVINS as an external process on the same sequence, captures its TUM trajectory, and scores it with our metrics for an apples-to-apples table.


## What should the first benchmark deliverable cover? → Our-stack intelligence/W report
Where should power be measured for the first version? → Host CPU via RAPL, Jetson via tegrastats, and external power meter, and all the the energy
models available in branes-ai/graphs. They were built for exactly this purpose: estimating energy for the computational graph of an embodied AI. So, we
can do the emprical measurement (RAPL/tegrastats) on existing hardware, and do model estimations with graphs to see how well we model cpu and gpu
executions. Once we have that integration figured out, we can also hook in the kpu energy/performance models that are also available in
branes-ai/graphs.

That changes the design meaningfully — you want both empirical measurement (RAPL/tegrastats/external) and model-based estimation via branes-ai/graphs, with
the benchmark able to compare modeled vs. measured energy (and later plug in the KPU models). That's a pluggable energy-backend architecture, and a
cross-repo integration. Let me study graphs before proposing anything concrete.

`graphs` is a Python library whose SubgraphDescriptor → EnergyAnalyzer.analyze(...) path is exactly built to estimate
energy/power/latency for an arbitrary operator graph on a chosen hardware SKU (CPU/GPU/KPU mappers, incl. kpu_t64/t256/t768) — so a hand-written VIO
pipeline fits as a per-operator FLOP/byte profile, no ONNX needed. That makes a clean two-backend design possible.

Proposed architecture — pluggable energy backends

                     ┌──────────────── vio_bench (C++, cortex) ───────────────┐
 EuRoC sequence ───▶ │ VioEstimator run → ATE/RPE, latency (p50/p99), RTF,    │
                     │ + per-stage operator profile (counts/shapes + time)    │
                     │ + EMPIRICAL energy (RAPL) measured around the run      │
                     └───────────────┬───────────────────────┬────────────────┘
                                     │ run.json               │ (measured)
                                     ▼                        │
            ┌──────── vio_energy_model.py (Python) ────────┐  │
            │ profile → graphs SubgraphDescriptors →       │  │
            │ EnergyAnalyzer over SKUs (CPU/GPU now,       │  │
            │ KPU t64/t256/t768 later) → MODELED energy    │  │
            └───────────────┬──────────────────────────────┘  │
                            ▼                                  ▼
                  report.{json,md}:  ATE/RPE · latency · RTF · J/frame · avg W
                  · accuracy-bounded fps/W · MODELED vs MEASURED (model validation)

Empirical backends (measure on real silicon): RAPL now (x86 host), tegrastats (Jetson), external meter (Joulescope/INA) later — each just wraps the run and
captures energy. Modeled backend: graphs estimates energy/power/latency from the operator profile for any SKU — and crucially, modeled-CPU vs
measured-RAPL on the same host is a model-validation check, which is the real prize: once graphs matches measurement on CPU/GPU, its KPU projections become
trustworthy intelligence/W numbers for the actual target.

The seam is the per-operator profile (frames, #features, KLT window/iters, #clones, residual rows → analytic FLOP/byte per stage). C++ stays Python-free;
graphs stays Python; they meet at a JSON file.

Phasing (each a PR through the normal gate)

1. vio_bench (C++, self-contained in cortex) — EuRoC run → ATE/RPE + latency + real-time factor + RAPL energy (J/frame, avg W) + accuracy-bounded fps/W,
JSON+MD report. Dataset-gated like the existing harness; the RAPL reader, FLOP/byte model, and report serialization are unit-tested in CI on synthetic
data. Immediately useful, no cross-repo dependency.
2. graphs model integration (Python) — consume the profile, build SubgraphDescriptors, run EnergyAnalyzer for CPU/GPU SKUs, fold modeled energy into the
report, and emit the modeled-vs-measured delta.
3. KPU + Jetson — graphs KPU SKUs (the target-hardware projection) + the tegrastats empirical backend.
4. (optional) OpenVINS/MINS baseline — run as an external black-box (both are GPL-3.0 — verified — so we never vendor/link their code), scored with our
metrics for a side-by-side table.

Two flags worth knowing: RAPL's energy_uj is often root-readable only (post-2019 side-channel CVE), so the bench degrades gracefully when it can't read it;
and the FLOP/byte profile starts as honest first-order analytic models (refine later with perf counters).

Start on Phase 1 (the self-contained C++ vio_bench) — unless you'd rather I adjust the home (I'd put it in a new bench/ dir with its own target) or
sanity-check the analytic FLOP/byte models with you first. Want me to proceed, or review the Phase-1 design in more detail before I build?

Sources for the license check: open_vins LICENSE (GPL-3.0) (https://github.com/rpng/open_vins/blob/master/LICENSE), MINS (rpng) — GPL-3.0
(https://github.com/rpng/MINS/blob/master/ReadMe.md).

