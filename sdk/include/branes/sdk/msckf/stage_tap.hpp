// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stage_tap.hpp — an opt-in observer of the MSCKF stage
// boundaries (issue #446, epic #444 §C).
//
// The backend (msckf_backend.hpp) runs the stages as a call sequence. A tap set
// on it (`set_stage_tap`) sees, at each boundary it asks for, everything the
// stage consumed and produced: the state before, the stage's inputs, its
// diagnostics, and the state after. The tools layer turns those into captured
// bench fixtures (tools/include/branes/tools/bench/stage_recorder.hpp); the SDK
// itself never sees the fixture format.
//
// Cost: with no tap set, each boundary costs one null-pointer test. With a tap
// set, a boundary the tap does not want (`wants` returns false) costs one
// virtual call; only a wanted boundary copies the state.
//
// A tap observes; it must not mutate the backend. The `after` state is the
// backend's live state, valid only during the call.
//
// Header-only, C++20.

#ifndef BRANES_SDK_MSCKF_STAGE_TAP_HPP
#define BRANES_SDK_MSCKF_STAGE_TAP_HPP

#include <branes/math/arithmetic.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/stages/s0_sensor_model.hpp>
#include <branes/sdk/msckf/stages/s2_propagation.hpp>
#include <branes/sdk/msckf/stages/s3_augmentation.hpp>
#include <branes/sdk/msckf/stages/s6_msckf_update.hpp>
#include <branes/sdk/msckf/stages/s9_marginalization.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <cstdint>

namespace branes::sdk::msckf {

/// The stage boundaries a tap can observe.
enum class TapStage {
    S0_sensor_model,     ///< one pixel unprojected and normalized
    S2_propagation,      ///< one IMU step (or the zero-order hold to a frame time)
    S3_augmentation,     ///< the frame's pose cloned into the window
    S6_msckf_update,     ///< one feature track through S5 → S6e
    S9_marginalization,  ///< one clone removed from the window
};

template <math::Scalar T, class Cov>
class StageTap {
public:
    using St = State<T, Cov>;
    using Vec3 = typename St::Vec3;

    virtual ~StageTap() = default;

    /// A camera frame at `t` enters the initialized filter: after the S2 steps
    /// that propagate to it (IMU samples, then the hold to `t`), before its
    /// S6/S9 window maintenance, S3 clone, S0/S4 ingest and S6 updates.
    virtual void on_frame(double /*t*/) {}

    /// Whether the next boundary of `stage` should be reported. Asked before
    /// the stage runs; only a `true` makes the backend copy the state.
    [[nodiscard]] virtual bool wants(TapStage /*stage*/) const {
        return false;
    }

    virtual void
    on_s0(std::uint32_t /*camera_id*/, T /*u*/, T /*v*/, const stages::s0_sensor_model::Result<T>& /*out*/) {}

    virtual void on_s2(const St& /*before*/,
                       const Propagator<T>& /*propagator*/,
                       const Vec3& /*gyro*/,
                       const Vec3& /*accel*/,
                       T /*dt*/,
                       const St& /*after*/) {}

    virtual void on_s3(const St& /*before*/,
                       double /*t*/,
                       const stages::s3_augmentation::Diagnostics& /*diag*/,
                       const St& /*after*/) {}

    virtual void on_s6(const St& /*before*/,
                       const CameraUpdater<T>& /*updater*/,
                       std::uint64_t /*feature_id*/,
                       const FeatureTrack<T>& /*track*/,
                       const stages::s6_msckf_update::Diagnostics<T>& /*diag*/,
                       const St& /*after*/) {}

    virtual void on_s9(const St& /*before*/,
                       std::size_t /*clone_index*/,
                       const stages::s9_marginalization::Diagnostics& /*diag*/,
                       const St& /*after*/) {}
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_STAGE_TAP_HPP
