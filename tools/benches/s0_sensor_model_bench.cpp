// SPDX-License-Identifier: MIT
//
// s0_sensor_model_bench — the S0_sensor_model stage bench (issue #454). Runs
// stages::s0_sensor_model::apply through the EuRoC pinhole-radtan camera on its
// fixtures, in several arithmetic types, and prints the invariant report. See
// docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s0_sensor_model_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S0SensorModelBench>(argc, argv);
}
