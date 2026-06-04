// SPDX-License-Identifier: MIT
//
// s8_zero_velocity — study stage in isolation: Zero-velocity update (scaffold).
//
// Prints the stage contract (pre/post-conditions + native-unit assessments) so
// the stage is studyable now; wire its probe in sdk/eval/ and drive it here,
// following tools/src/s0_sensor_model.cpp. See docs/arch/vio-pipeline-canonical.md.

#include <branes/tools/vio_stage_contracts.hpp>

int main(int argc, char** argv) {
    const auto pl = branes::tools::pipeline();
    return branes::tools::run_scaffold(argc, argv, branes::tools::kS8, &pl);
}
