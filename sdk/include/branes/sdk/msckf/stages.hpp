// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages.hpp — the VIO pipeline stages as explicit
// transformations (issue #452, epic #444). Each stage lives in its own header
// and namespace, msckf::stages::<S<N>_transformation>, with an `apply` of the
// shape (state, inputs, params) → (state, diagnostics): the state passed in is
// the only thing mutated, and what happened is returned rather than logged. The
// MSCKF backend is the call sequence of these transformations; a stage bench
// calls one in isolation.
//
//   S0_sensor_model         stages/s0_sensor_model.hpp         pixel → normalized image point
//   S1_initialization       stages/s1_initialization.hpp       init result → seeded state
//   S2_propagation          stages/s2_propagation.hpp          IMU sample → propagated state
//   S3_augmentation         stages/s3_augmentation.hpp         state → state + clone
//   S4_frontend             stages/s4_frontend.hpp             frame observations → tracks, ended tracks
//   S5_triangulation        stages/s5_triangulation.hpp        track → feature position
//   S6a–S6e (MSCKF update)  stages/s6_msckf_update.hpp         track → updated state
//   S8_zero_velocity        stages/s8_zero_velocity.hpp        IMU + frames at rest → v = 0, b_g = ω̄
//   S9_marginalization      stages/s9_marginalization.hpp      state → state − clone
//   S10_online_calibration  stages/s10_online_calibration.hpp  state → state + calibration states
//
// S7 (SLAM features) is not implemented in cortex; it gets a stage header when
// it is.
//
// Header-only, C++20.

#ifndef BRANES_SDK_MSCKF_STAGES_HPP
#define BRANES_SDK_MSCKF_STAGES_HPP

#include <branes/sdk/msckf/stages/s0_sensor_model.hpp>
#include <branes/sdk/msckf/stages/s10_online_calibration.hpp>
#include <branes/sdk/msckf/stages/s1_initialization.hpp>
#include <branes/sdk/msckf/stages/s2_propagation.hpp>
#include <branes/sdk/msckf/stages/s3_augmentation.hpp>
#include <branes/sdk/msckf/stages/s4_frontend.hpp>
#include <branes/sdk/msckf/stages/s5_triangulation.hpp>
#include <branes/sdk/msckf/stages/s6_msckf_update.hpp>
#include <branes/sdk/msckf/stages/s8_zero_velocity.hpp>
#include <branes/sdk/msckf/stages/s9_marginalization.hpp>

#endif  // BRANES_SDK_MSCKF_STAGES_HPP
