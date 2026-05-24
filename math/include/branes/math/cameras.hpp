// SPDX-License-Identifier: MIT
//
// branes/math/cameras.hpp — umbrella include for the camera models used
// by the VIO/SLAM front end: pinhole+radtan (EuRoC), equidistant fisheye
// (Kannala–Brandt), and the unified omnidirectional model (Mei–Rives).
// Each is header-only, type-generic, and provides project / unproject /
// distort / undistort plus the analytical d(pixel)/d(point) Jacobian.

#ifndef BRANES_MATH_CAMERAS_HPP
#define BRANES_MATH_CAMERAS_HPP

#include <branes/math/cameras/equidistant.hpp>
#include <branes/math/cameras/pinhole_radtan.hpp>
#include <branes/math/cameras/unified.hpp>

#endif  // BRANES_MATH_CAMERAS_HPP
