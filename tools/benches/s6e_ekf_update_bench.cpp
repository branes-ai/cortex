// SPDX-License-Identifier: MIT
//
// s6e_ekf_update_bench — the S6e_ekf_update stage bench (issue #457). Runs
// stages::s6e_ekf_update::apply on its fixtures, in several arithmetic types,
// and prints the invariant report. See docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s6e_ekf_update_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S6eEkfUpdateBench>(argc, argv);
}
