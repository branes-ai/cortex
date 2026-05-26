// SPDX-License-Identifier: MIT
//
// branes/sdk/sliding_window_backend.hpp — sliding-window optimization
// backend *skeleton*.
//
// This is a deliberate placeholder that implements the VioBackend
// interface but performs no estimation: the measurement-processing and
// state-query methods raise NotImplemented. Its purpose is to prove the
// backend abstraction is real and substitutable — a VioEstimator (or any
// consumer) can be templated on it and link, before the MSCKF backend
// accretes implementation-specific assumptions. The real sliding-window
// math (keyframe marginalization, factor-graph / Schur-complement solve
// over the IMU-preintegration and reprojection factors) lands post-MVP.
//
// Header-only, C++20, type-generic. Lives next to msckf_backend.hpp.

#ifndef BRANES_SDK_SLIDING_WINDOW_BACKEND_HPP
#define BRANES_SDK_SLIDING_WINDOW_BACKEND_HPP

#include <branes/sdk/vio_backend.hpp>

#include <span>
#include <stdexcept>

namespace branes::sdk {

/// Raised by not-yet-implemented backend operations. Distinct type so a
/// substitution test (or a caller) can observe the placeholder response
/// unambiguously.
struct NotImplemented : std::logic_error {
    using std::logic_error::logic_error;
};

template <math::Scalar T>
class SlidingWindowBackend final : public VioBackend<T> {
public:
    using Scalar = T;

    /// Marks this backend as a non-functional skeleton. Real backends
    /// (e.g. MsckfBackend) report true.
    static constexpr bool kImplemented = false;

    /// Configuration is accepted and retained (so the object is
    /// constructible/configurable and clearly substitutable), but no
    /// estimation is performed.
    void initialize(const VioConfig& config) override {
        config_ = config;
    }

    void process_imu(const ImuMeasurement<T>&) override {
        throw NotImplemented("SlidingWindowBackend::process_imu is not implemented yet (post-MVP)");
    }

    void process_camera(double, std::span<const FrontendObservation<T>>) override {
        throw NotImplemented("SlidingWindowBackend::process_camera is not implemented yet (post-MVP)");
    }

    [[nodiscard]] NavState<T> current_state() const override {
        throw NotImplemented("SlidingWindowBackend::current_state is not implemented yet (post-MVP)");
    }

    /// Non-throwing introspection: whether this backend actually estimates.
    [[nodiscard]] bool implemented() const noexcept {
        return kImplemented;
    }

    [[nodiscard]] const VioConfig& config() const noexcept {
        return config_;
    }

private:
    VioConfig config_{};
};

}  // namespace branes::sdk

#endif  // BRANES_SDK_SLIDING_WINDOW_BACKEND_HPP
