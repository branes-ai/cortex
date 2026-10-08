// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s10_online_calibration.hpp — stage
// S10_online_calibration as an explicit transformation (issue #452,
// docs/arch/vio-pipeline-canonical.md §S10).
//
//   apply(state, extrinsics, (σ_rot, σ_trans)) → (state' with calibration states, diagnostics)
//
// Puts each camera↔IMU extrinsic into the error state (6 per camera) with the
// stated prior. Must run on a fresh state, before any clone. The calibration
// states are then estimated by the regular update: S6a contributes their
// Jacobian columns and S6e box-plusses their correction.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S10_ONLINE_CALIBRATION_HPP
#define BRANES_SDK_MSCKF_STAGES_S10_ONLINE_CALIBRATION_HPP

#include <branes/sdk/msckf/state.hpp>

#include <cstddef>
#include <utility>
#include <vector>

namespace branes::sdk::msckf::stages::s10_online_calibration {

struct Diagnostics {
    std::size_t cameras = 0;    ///< calibration blocks added
    std::size_t dim_after = 0;  ///< error-state dimension including them
};

template <math::Scalar T, class Cov>
Diagnostics
apply(State<T, Cov>& s, std::vector<typename State<T, Cov>::CalibState> extrinsics, T rot_sigma, T trans_sigma) {
    Diagnostics d;
    d.cameras = extrinsics.size();
    s.enable_calibration(std::move(extrinsics), rot_sigma, trans_sigma);
    d.dim_after = s.dim();
    return d;
}

}  // namespace branes::sdk::msckf::stages::s10_online_calibration

#endif  // BRANES_SDK_MSCKF_STAGES_S10_ONLINE_CALIBRATION_HPP
