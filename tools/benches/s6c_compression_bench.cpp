// SPDX-License-Identifier: MIT
//
// s6c_compression_bench — the S6c_compression stage bench (issue #457). Runs
// stages::s6c_compression::apply on its fixtures, in several arithmetic types,
// and prints the invariant report. See docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s6c_compression_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S6cCompressionBench>(argc, argv);
}
