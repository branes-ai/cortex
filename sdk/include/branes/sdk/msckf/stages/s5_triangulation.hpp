// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s5_triangulation.hpp — stage S5_triangulation as an
// explicit transformation (issue #452, docs/arch/vio-pipeline-canonical.md §S5).
//
//   apply(state, updater, track) → (p_f ∈ ℝ³ | rejected)
//
// Validates the track (≥ 2 observations, clone and camera indices in the
// window) and triangulates the feature from the clone poses: the linear
// ray-perpendicular solve, the optional parallax gate, then Gauss-Newton
// reprojection refinement. Reads the state; never mutates it.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S5_TRIANGULATION_HPP
#define BRANES_SDK_MSCKF_STAGES_S5_TRIANGULATION_HPP

#include <branes/sdk/msckf/camera_updater.hpp>

namespace branes::sdk::msckf::stages::s5_triangulation {

template <math::Scalar T>
struct Result {
    bool ok = false;                     ///< false: invalid track, degenerate geometry, or gated parallax
    math::lie::detail::Vec<T, 3> p_f{};  ///< feature position in the world frame
};

template <math::Scalar T, class Cov>
[[nodiscard]] Result<T> apply(const State<T, Cov>& s, const CameraUpdater<T>& updater, const FeatureTrack<T>& track) {
    Result<T> out;
    if (!updater.validate_track(s, track))
        return out;
    out.ok = updater.triangulate(s, track.observations, out.p_f);
    return out;
}

}  // namespace branes::sdk::msckf::stages::s5_triangulation

#endif  // BRANES_SDK_MSCKF_STAGES_S5_TRIANGULATION_HPP
