// SPDX-License-Identifier: MIT
//
// s5_triangulation_bench — the S5_triangulation stage bench (issue #456). Runs
// stages::s5_triangulation::apply on its
// fixtures, in several arithmetic types, and prints the invariant report. See
// docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s5_triangulation_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S5TriangulationBench>(argc, argv);
}
