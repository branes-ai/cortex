# VIO Benchmark

How to run the benchmark

It runs on a EuRoC MAV sequence (ASL format). End to end:

1. Get a sequence (e.g. V1_01_easy) — download the ASL .zip from the ETH EuRoC page
(https://projects.asl.ethz.ch/datasets/) and unzip; you'll get a mav0/ folder (imu0/, cam0/,
state_groundtruth_estimate0/).

2. Build the tool (optimized — latency/energy are only meaningful with -O2):
cmake --preset sitl-release
cmake --build --preset sitl-release --target vio_bench
./build/sitl-release/bench/vio_bench /path/to/V1_01_easy/mav0 --out v101
# prints a Markdown report; writes v101.json + v101.md
RAPL energy_uj is usually root-only, so for real Joules run it with sudo (otherwise it reports energy unavailable but still gives ATE/RPE/latency/RTF).

While it runs it prints a `processed N/total frames (P%)` progress line to stderr (the Markdown report goes to stdout). Runtime is roughly the sequence
duration divided by the real-time factor it reports — a few minutes for V1_01_easy (~2.9k frames). It's silent on stdout until the run finishes, so the
stderr counter is how you tell it's working rather than hung.

4. Model energy across SKUs (Phase 2 — needs the graphs repo installed):
pip install -e /home/stillwater/dev/branes/clones/graphs
python bench/energy_model/vio_energy_model.py v101.json --skus cpu,gpu,kpu_t64 --out v101_energy

No dataset? The component unit tests still run without one: ctest -R bench (C++) and cd bench/energy_model && python -m unittest (Python).

On-device energy backends (merged in #207): on a Jetson add --energy tegrastats --energy-source 50 (the sampling interval in ms), or on an instrumented
rig --energy external --energy-source <cumulative-µJ counter file>. Without a flag it defaults to --energy rapl.

