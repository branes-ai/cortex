// SPDX-License-Identifier: MIT
//
// s9_marginalization_bench — the S9_marginalization stage bench (issue #453).
// Runs stages::s9_marginalization::apply on its fixtures in double, float and
// posit<32,2>, prints the stage's invariant report, and optionally writes CSV
// reports, sweeps, and the built-in fixtures as files.
//
//   ./s9_marginalization_bench                          # built-in fixtures, default types
//   ./s9_marginalization_bench --types double,posit16   # pick arithmetic types
//   ./s9_marginalization_bench --variant all            # shipped vs research variants
//   ./s9_marginalization_bench --fixture F.json         # a fixture file (e.g. captured, #446)
//   ./s9_marginalization_bench --sweep --csv DIR        # characterization sweep → CSV
//   ./s9_marginalization_bench --capture DIR            # write the built-in fixtures

#include <branes/tools/bench/s9_marginalization_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S9MarginalizationBench>(argc, argv);
}
