// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s0_sensor_model.hpp — stage S0_sensor_model as an
// explicit transformation (issue #452, docs/arch/vio-pipeline-canonical.md §S0).
//
//   apply(camera, (u, v)) → (normalized image point x/z, y/z | rejected)
//
// The camera model's unprojection, normalized to the image plane: the one place
// pixels become the intrinsics-free coordinates the estimator works in. A ray that
// is not in front of the camera (z ≤ 0) is rejected, never normalized.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S0_SENSOR_MODEL_HPP
#define BRANES_SDK_MSCKF_STAGES_S0_SENSOR_MODEL_HPP

#include <branes/math/cameras/detail.hpp>
#include <branes/math/lie/detail.hpp>

namespace branes::sdk::msckf::stages::s0_sensor_model {

template <math::Scalar T>
struct Result {
    bool valid = false;                 ///< false: the ray is not in front of the camera
    math::lie::detail::Vec<T, 2> xy{};  ///< normalized image point (x/z, y/z)
};

/// Unproject pixel (u, v) through `camera` and normalize to the image plane.
/// `Camera` is any model with `unproject(Vec2) → +Z-forward bearing`.
template <math::Scalar T, class Camera>
[[nodiscard]] Result<T> apply(const Camera& camera, T u, T v) {
    Result<T> out;
    const auto bearing = camera.unproject(math::cameras::Vec2<T>{{u, v}});
    if (!(bearing[2] > T{0}))
        return out;
    out.xy = math::lie::detail::Vec<T, 2>{{bearing[0] / bearing[2], bearing[1] / bearing[2]}};
    out.valid = true;
    return out;
}

}  // namespace branes::sdk::msckf::stages::s0_sensor_model

#endif  // BRANES_SDK_MSCKF_STAGES_S0_SENSOR_MODEL_HPP
