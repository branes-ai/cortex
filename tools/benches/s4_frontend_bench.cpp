// SPDX-License-Identifier: MIT
//
// s4_frontend_bench — the S4_frontend stage bench (issue #456). Runs
// the image-domain front end (KLT, FB gate, FAST) on its
// fixtures, in several arithmetic types, and prints the invariant report. See
// docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s4_frontend_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S4FrontendBench>(argc, argv);
}
