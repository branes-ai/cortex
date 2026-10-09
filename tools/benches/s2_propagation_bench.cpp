// SPDX-License-Identifier: MIT
//
// s2_propagation_bench — the S2_propagation stage bench (issue #455). Runs
// stages::s2_propagation::apply over IMU segments on its
// fixtures, in several arithmetic types, and prints the invariant report. See
// docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s2_propagation_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S2PropagationBench>(argc, argv);
}
