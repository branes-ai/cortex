// SPDX-License-Identifier: MIT
//
// s10_online_calibration_bench — the S10_online_calibration stage bench (issue #458). Runs
// stages::s10_online_calibration::apply on its fixtures, in several arithmetic types,
// and prints the invariant report. See docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s10_online_calibration_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S10OnlineCalibrationBench>(argc, argv);
}
