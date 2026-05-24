// SPDX-License-Identifier: MIT
//
// branes/math/lie.hpp — umbrella include for the header-only Lie groups
// used by the estimation stack: SO(3), SE(3), Sim(3).
//
// Each group provides exp / log / composition / inverse / matrix /
// adjoint and the left/right Jacobians of the exponential (with
// inverses), is type-generic over the scalar (double, float, Universal
// posit, ...), and is implemented clean-room from published Lie theory.
// See the individual headers for conventions and references.

#ifndef BRANES_MATH_LIE_HPP
#define BRANES_MATH_LIE_HPP

#include <branes/math/lie/se3.hpp>
#include <branes/math/lie/sim3.hpp>
#include <branes/math/lie/so3.hpp>

#endif  // BRANES_MATH_LIE_HPP
