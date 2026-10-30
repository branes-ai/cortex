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
//   ekf_update       S6e  the EKF update alone (optional: defaults to `update`)
//   marginalization  S9   removal of a clone
//   calibration      S10  calibration states added to the fresh state
//
// `ekf_update` splits S6e (the Kalman gain, the mean correction and the
// covariance update) off the rest of the update: with it, S5 → S6d run in
// `update` and S6e in `ekf_update`. A policy that does not name
// it runs S6e in `update`, so `using update = float;` puts the whole update in
// float. S6e is the step a float update fails at on real data (#450,
// debugging/mixed-precision.mdx), hence its own alias:
//
//   struct UpdateFloatEkfDouble : msckf::UniformStageTypes<double> {
//       using update = float;
//       using ekf_update = double;
//   };
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

namespace detail {
template <class P>
struct ekf_update_of {
    using type = typename P::update;
};
template <class P>
    requires requires { typename P::ekf_update; }
struct ekf_update_of<P> {
    using type = typename P::ekf_update;
};
}  // namespace detail

/// The type S6e computes in: `P::ekf_update` when the policy names it, else
/// `P::update`.
template <class P>
using ekf_update_t = typename detail::ekf_update_of<P>::type;

/// A policy names a Scalar type for every stage.
template <class P>
concept StageTypePolicy =
    math::Scalar<typename P::sensor_model> && math::Scalar<typename P::propagation> &&
    math::Scalar<typename P::augmentation> && math::Scalar<typename P::update> && math::Scalar<ekf_update_t<P>> &&
    math::Scalar<typename P::marginalization> && math::Scalar<typename P::calibration>;

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_STAGE_TYPES_HPP
