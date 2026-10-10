// SPDX-License-Identifier: MIT
//
// branes/tools/euroc_cam0.hpp — the EuRoC MAV cam0 calibration (pinhole-radtan
// intrinsics and the cam0→IMU extrinsic T_BS), from the published calibration.
//
// Shared by every tool that runs EuRoC (vio_pipeline's MSCKF and R-IEKF runs,
// vio_trajectory_report), so they are always driven from byte-identical
// calibration. EuRoC's cam0 is rotated ~90° from the IMU: an identity
// extrinsic makes the measurement model grossly wrong (#245).
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_EUROC_CAM0_HPP
#define BRANES_TOOLS_EUROC_CAM0_HPP

#include <branes/math/cameras/pinhole_radtan.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/sfm/init_window.hpp>  // so3_from_matrix

namespace branes::tools {

template <math::Scalar T>
struct EurocCam0 {
    math::cameras::PinholeRadtanCamera<T> intrinsics;
    math::lie::SO3<T> R_imu_cam;
    math::lie::detail::Vec<T, 3> p_imu_cam;
};

template <math::Scalar T>
[[nodiscard]] inline EurocCam0<T> euroc_cam0() {
    math::lie::detail::Mat<T, 3, 3> R{};
    R(0, 0) = 0.0148655429818;
    R(0, 1) = -0.999880929698;
    R(0, 2) = 0.00414029679422;
    R(1, 0) = 0.999557249008;
    R(1, 1) = 0.0149672133247;
    R(1, 2) = 0.025715529948;
    R(2, 0) = -0.0257744366974;
    R(2, 1) = 0.00375618835797;
    R(2, 2) = 0.999660727178;
    return EurocCam0<T>{math::cameras::PinholeRadtanCamera<T>(
                            458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05),
                        sdk::sfm::so3_from_matrix<T>(R),
                        math::lie::detail::Vec<T, 3>{{-0.0216401454975, -0.064676986768, 0.00981073058949}}};
}

}  // namespace branes::tools

#endif  // BRANES_TOOLS_EUROC_CAM0_HPP
