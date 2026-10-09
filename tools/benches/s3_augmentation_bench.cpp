// SPDX-License-Identifier: MIT
//
// s3_augmentation_bench — the S3_augmentation stage bench (issue #455). Runs
// stages::s3_augmentation::apply (stochastic cloning) on its
// fixtures, in several arithmetic types, and prints the invariant report. See
// docs/arch/stage-benches.md for the flags.

#include <branes/tools/bench/s3_augmentation_bench.hpp>

int main(int argc, char** argv) {
    return branes::tools::bench::bench_main<branes::tools::bench::S3AugmentationBench>(argc, argv);
}
