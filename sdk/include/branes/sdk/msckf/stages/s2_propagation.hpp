// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s2_propagation.hpp — stage S2_propagation as an
// explicit transformation (issue #452, docs/arch/vio-pipeline-canonical.md §S2).
//
//   apply(state, propagator, (ω̃, ã), Δt) → (state', diagnostics)
//
// One IMU sample of strapdown mean integration and covariance propagation,
// P ← Φ P Φᵀ + Q_d, in whichever covariance representation the state carries.
// A non-positive Δt leaves the state untouched.
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

template <math::Scalar T, class Cov>
Diagnostics<T> apply(State<T, Cov>& s,
                     const Propagator<T>& propagator,
                     const typename State<T, Cov>::Vec3& gyro,
                     const typename State<T, Cov>::Vec3& accel,
                     T dt) {
    Diagnostics<T> d;
    d.dt = dt;
    d.applied = dt > T{0};
    propagator.propagate(s, gyro, accel, dt);
    d.dim = s.dim();
    return d;
}

}  // namespace branes::sdk::msckf::stages::s2_propagation

#endif  // BRANES_SDK_MSCKF_STAGES_S2_PROPAGATION_HPP
