// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stage_types.hpp — the arithmetic type each MSCKF stage
// computes in (issue #450, epic #444 §C).
//
// The backend keeps its state in one storage type T. A stage-type policy names,
// per stage, the type that stage computes in; a stage whose type is not T runs on
// the state rebound into its type and hands the result back rounded to T
// (rebind.hpp). The policy is a struct of type aliases, one per stage:
//
//   sensor_model     S0   pixel → normalized image point
//   propagation      S2   IMU sample → propagated mean and covariance
//   augmentation     S3   clone of the pose into the window
//   update           S5 → S6e   triangulation, Jacobians, null space, gate, EKF update
//   marginalization  S9   removal of a clone
//   calibration      S10  calibration states added to the fresh state
//
// UniformStageTypes<T> is every stage in T, the default, and compiles to the
// single-type backend. A mixed policy derives from it and overrides the stages
// it changes, by name:
//
//   struct UpdateInDouble : msckf::UniformStageTypes<float> {
//       using update = double;
//   };
//   MsckfBackendT<float, FullCovariance<float>, UpdateInDouble> backend;
//
// S1 (initialization) and S4 (the front end) are not in the policy: S1 runs once,
// in the storage type, and S4 hands the backend storage-type observations.
//
// Header-only, C++20.

#ifndef BRANES_SDK_MSCKF_STAGE_TYPES_HPP
#define BRANES_SDK_MSCKF_STAGE_TYPES_HPP

#include <branes/math/arithmetic.hpp>

namespace branes::sdk::msckf {

/// Every stage in T.
template <math::Scalar T>
struct UniformStageTypes {
    using sensor_model = T;
    using propagation = T;
    using augmentation = T;
    using update = T;
    using marginalization = T;
    using calibration = T;
};

/// A policy names a Scalar type for every stage.
template <class P>
concept StageTypePolicy = math::Scalar<typename P::sensor_model> && math::Scalar<typename P::propagation> &&
                          math::Scalar<typename P::augmentation> && math::Scalar<typename P::update> &&
                          math::Scalar<typename P::marginalization> && math::Scalar<typename P::calibration>;

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_STAGE_TYPES_HPP
