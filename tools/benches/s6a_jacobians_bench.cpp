// SPDX-License-Identifier: MIT
//
// s6a_jacobians_bench — the S6a_jacobians stage bench (issue #457). Runs
// stages::s6a_jacobians::apply on its fixtures, in several arithmetic types,
// and prints the invariant report. See docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s6a_jacobians_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S6aJacobiansBench>(argc, argv);
}
