// SPDX-License-Identifier: MIT
//
// c_filter_loop_bench — the composition bench for the MSCKF loop (issue #447).
// Runs a tape of S2/S3/S6/S9 operations through the shipped stages and checks
// each operation's stage contract plus the loop's own invariants.

#include <branes/tools/bench/c_filter_loop_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::CFilterLoopBench>(argc, argv);
}
