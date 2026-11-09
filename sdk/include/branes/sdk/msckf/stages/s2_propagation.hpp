// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s2_propagation.hpp — stage S2_propagation as an
// explicit transformation (issue #452, docs/pipeline/canonical-pipeline.md §S2).
//
//   apply(state, propagator, interval) → (state', diagnostics)
//
// One IMU interval of strapdown mean integration and covariance propagation,
// P ← Φ P Φᵀ + Q_d, in whichever covariance representation the state carries.
// A non-positive Δt leaves the state untouched.
//
// The convention both backends follow (#616):
//   • an interval [t_{k−1}, t_k] between two IMU samples is integrated from
//     both of them: the input is the line between the two samples, integrated
//     by RK4 (Propagator::propagate, rk4_strapdown). A sample is the rate at
//     its timestamp; holding either one over the interval would integrate the
//     IMU half a sample early or late against the camera.
//   • an image time t_c between two samples: propagate to t_c holding the
//     last sample (the next one has not arrived), then, when it arrives,
//     integrate [t_c, t_k] from the rate interpolated at t_c
//     (interpolate_sample) to the new sample. The residual skew lives only on
//     the partial interval before the image; it is zero when image and IMU
//     stamps coincide, as on EuRoC.
// docs/first-principles/sample-alignment.mdx is the failure mode this guards.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S2_PROPAGATION_HPP
#define BRANES_SDK_MSCKF_STAGES_S2_PROPAGATION_HPP

#include <branes/sdk/msckf/propagator.hpp>

#include <cstddef>

namespace branes::sdk::msckf::stages::s2_propagation {

template <math::Scalar T>
struct Diagnostics {
    bool applied = false;  ///< false: Δt ≤ 0, nothing propagated
    T dt = T{0};
    std::size_t dim = 0;  ///< error-state dimension (unchanged by propagation)
};

/// Propagate over one IMU interval (both of its samples).
template <math::Scalar T, class Cov>
Diagnostics<T> apply(State<T, Cov>& s, const Propagator<T>& propagator, const ImuInterval<T>& interval) {
    Diagnostics<T> d;
    d.dt = interval.dt;
    d.applied = interval.dt > T{0};
    propagator.propagate(s, interval);
    d.dim = s.dim();
    return d;
}

/// Propagate over `dt` on one sample held constant: a constant input, or the
/// hold up to an image time (ImuInterval::held).
template <math::Scalar T, class Cov>
Diagnostics<T> apply(State<T, Cov>& s,
                     const Propagator<T>& propagator,
                     const typename State<T, Cov>::Vec3& gyro,
                     const typename State<T, Cov>::Vec3& accel,
                     T dt) {
    return apply(s, propagator, ImuInterval<T>::held(gyro, accel, dt));
}

}  // namespace branes::sdk::msckf::stages::s2_propagation

#endif  // BRANES_SDK_MSCKF_STAGES_S2_PROPAGATION_HPP
