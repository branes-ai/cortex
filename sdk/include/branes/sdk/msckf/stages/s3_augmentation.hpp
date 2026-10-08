// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s3_augmentation.hpp — stage S3_augmentation as an
// explicit transformation (issue #452, docs/arch/vio-pipeline-canonical.md §S3).
//
//   apply(state, t) → (state' with a clone of the pose at t, diagnostics)
//
// Stamps the state at the frame time and stochastically clones the IMU pose
// into the sliding window: P' = G P Gᵀ with G = [I; J], so the clone carries the
// exact cross-covariance of the pose it copies.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S3_AUGMENTATION_HPP
#define BRANES_SDK_MSCKF_STAGES_S3_AUGMENTATION_HPP

#include <branes/sdk/msckf/state_helper.hpp>

#include <cstddef>

namespace branes::sdk::msckf::stages::s3_augmentation {

struct Diagnostics {
    std::size_t dim_before = 0;
    std::size_t dim_after = 0;  ///< dim_before + 6
    std::size_t clones = 0;     ///< window size after the clone
};

template <math::Scalar T, class Cov>
Diagnostics apply(State<T, Cov>& s, double t) {
    Diagnostics d;
    d.dim_before = s.dim();
    s.timestamp = t;
    StateHelper<T>::augment_clone(s);
    d.dim_after = s.dim();
    d.clones = s.clones.size();
    return d;
}

}  // namespace branes::sdk::msckf::stages::s3_augmentation

#endif  // BRANES_SDK_MSCKF_STAGES_S3_AUGMENTATION_HPP
