// SPDX-License-Identifier: MIT
//
// s6d_gating_bench — the S6d_gating stage bench (issue #457). Runs
// stages::s6d_gating::apply on its fixtures, in several arithmetic types,
// and prints the invariant report. See docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s6d_gating_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S6dGatingBench>(argc, argv);
}
