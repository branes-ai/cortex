// SPDX-License-Identifier: MIT
//
// s6b_nullspace_projection_bench — the S6b_nullspace_projection stage bench (issue #457). Runs
// stages::s6b_nullspace_projection::apply on its fixtures, in several arithmetic types,
// and prints the invariant report. See docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s6b_nullspace_projection_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S6bNullspaceProjectionBench>(argc, argv);
}
