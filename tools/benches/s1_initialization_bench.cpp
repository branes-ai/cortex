// SPDX-License-Identifier: MIT
//
// s1_initialization_bench — the S1_initialization stage bench (issue #454). Runs
// the ImuInitializer + S1 seeding (static, gravity-align, dynamic) on its
// fixtures, in several arithmetic types, and prints the invariant report. See
// docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s1_initialization_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S1InitializationBench>(argc, argv);
}
