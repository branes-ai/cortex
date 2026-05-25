// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf.hpp — umbrella include for the MSCKF state machinery:
// the State vector, the IMU Propagator (mean + covariance), the
// StateHelper (clone augmentation, marginalization, EKF update), and the
// CameraUpdater (feature triangulation + null-space MSCKF update).

#ifndef BRANES_SDK_MSCKF_HPP
#define BRANES_SDK_MSCKF_HPP

#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/msckf/state_helper.hpp>

#endif  // BRANES_SDK_MSCKF_HPP
