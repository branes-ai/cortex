// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s6_msckf_update.hpp — stage S6 (the MSCKF update) as
// explicit transformations, one per sub-step (issue #452,
// docs/arch/vio-pipeline-canonical.md §S6), so the update can be benched piece
// by piece:
//
//   s6a_jacobians::apply(state, updater, track, p_f)  → (H_f, H_x, r)
//   s6b_nullspace_projection::apply(H_f, H_x, r)      → (H₀, r₀), feature marginalized
//   s6c_compression::apply(H₀, r₀)                    → (H_c, r_c)
//   s6d_gating::apply(state, updater, H_c, r_c)       → (NIS, accept | reject)
//   s6e_ekf_update::apply(state, updater, H_c, r_c)   → (state', δx)
//
// and s6_msckf_update::apply(state, updater, track), the sequence S5 → S6e for
// one feature track. Every sub-step reads the state; only S6e mutates it.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S6_MSCKF_UPDATE_HPP
#define BRANES_SDK_MSCKF_STAGES_S6_MSCKF_UPDATE_HPP

#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/stages/s5_triangulation.hpp>

#include <cstddef>
#include <utility>
#include <vector>

namespace branes::sdk::msckf::stages {

// ── S6a: measurement Jacobians and residual ───────────────────────────────
namespace s6a_jacobians {

template <math::Scalar T>
struct Result {
    bool ok = false;  ///< false: the feature projects behind an observing camera
    MeasurementSystem<T> system;
};

/// Stack H_f (2m×3), H_x (2m×n) and r (2m) for `track` at the triangulated p_f.
template <math::Scalar T, class Cov>
[[nodiscard]] Result<T> apply(const State<T, Cov>& s,
                              const CameraUpdater<T>& updater,
                              const FeatureTrack<T>& track,
                              const math::lie::detail::Vec<T, 3>& p_f) {
    Result<T> out;
    out.ok = updater.measurement_system(s, track.observations, p_f, out.system);
    return out;
}

}  // namespace s6a_jacobians

// ── S6b: left null-space projection (marginalize the feature) ─────────────
namespace s6b_nullspace_projection {

template <math::Scalar T>
struct Result {
    bool ok = false;  ///< false: no rows survive the projection
    ProjectedMeasurement<T> projected;
};

template <math::Scalar T>
[[nodiscard]] Result<T> apply(const MeasurementSystem<T>& system) {
    Result<T> out;
    out.ok = CameraUpdater<T>::nullspace_project(system, out.projected);
    return out;
}

}  // namespace s6b_nullspace_projection

// ── S6c: measurement compression ──────────────────────────────────────────
namespace s6c_compression {

template <math::Scalar T>
struct Result {
    ProjectedMeasurement<T> compressed;
    std::size_t rows_in = 0;
    std::size_t rows_out = 0;
    bool compressed_applied = false;  ///< cortex applies none yet (see below)
};

/// The identity today: cortex updates one feature at a time, whose projected
/// system has 2m − 3 rows — fewer than the state dimension — so there is
/// nothing for a QR compression to remove. The stage is explicit so a bench can
/// slot in a compressing variant (stacked multi-feature updates) and check it
/// against the inv::check_compression invariants (#457).
template <math::Scalar T>
[[nodiscard]] Result<T> apply(ProjectedMeasurement<T> projected) {
    Result<T> out;
    out.rows_in = projected.H.rows;
    out.rows_out = projected.H.rows;
    out.compressed = std::move(projected);
    return out;
}

}  // namespace s6c_compression

// ── S6d: innovation NIS and χ² gating ─────────────────────────────────────
namespace s6d_gating {

/// NIS γ = rᵀ S⁻¹ r with dof = rows, and the Mahalanobis gate γ ≤
/// χ²_per_dof · rows. With `want_nis` false and gating disabled, nothing is
/// computed and the measurement is accepted.
template <math::Scalar T, class Cov>
[[nodiscard]] GateDecision<T> apply(const State<T, Cov>& s,
                                    const CameraUpdater<T>& updater,
                                    const ProjectedMeasurement<T>& measurement,
                                    bool want_nis = true) {
    return updater.gate(s, measurement, want_nis);
}

}  // namespace s6d_gating

// ── S6e: EKF update ───────────────────────────────────────────────────────
namespace s6e_ekf_update {

template <math::Scalar T>
struct Result {
    std::vector<T> dx;  ///< the error-state correction box-plussed onto the mean
};

/// Kalman gain, covariance update (Joseph form, or the QR array form for the
/// square-root covariance) and box-plus of δx onto the mean.
template <math::Scalar T, class Cov>
Result<T> apply(State<T, Cov>& s, const CameraUpdater<T>& updater, const ProjectedMeasurement<T>& measurement) {
    return Result<T>{updater.apply_update(s, measurement)};
}

}  // namespace s6e_ekf_update

// ── S6: the update of one feature track, S5 → S6e ─────────────────────────
namespace s6_msckf_update {

/// The step a track stopped at; `Applied` means the state was updated.
enum class Outcome {
    Rejected,      ///< S5: invalid track or triangulation failed
    BehindCamera,  ///< S6a: the feature projects behind an observing camera
    NoNullspace,   ///< S6b: no rows survive the projection
    Gated,         ///< S6d: ill-conditioned innovation or χ² gate
    Applied,       ///< S6e ran
};

template <math::Scalar T>
struct Diagnostics {
    Outcome outcome = Outcome::Rejected;
    NisSample<T> nis{};    ///< valid once S6d ran on a well-conditioned innovation
    std::size_t rows = 0;  ///< projected residual dimension (2m − 3)
    [[nodiscard]] bool accepted() const noexcept {
        return outcome == Outcome::Applied;
    }
};

template <math::Scalar T, class Cov>
Diagnostics<T> apply(State<T, Cov>& s, const CameraUpdater<T>& updater, const FeatureTrack<T>& track) {
    Diagnostics<T> d;
    const auto tri = s5_triangulation::apply(s, updater, track);
    if (!tri.ok)
        return d;
    const auto jac = s6a_jacobians::apply(s, updater, track, tri.p_f);
    if (!jac.ok) {
        d.outcome = Outcome::BehindCamera;
        return d;
    }
    auto proj = s6b_nullspace_projection::apply(jac.system);
    if (!proj.ok) {
        d.outcome = Outcome::NoNullspace;
        return d;
    }
    const auto comp = s6c_compression::apply(std::move(proj.projected));
    d.rows = comp.rows_out;
    const auto gate = s6d_gating::apply(s, updater, comp.compressed);
    d.nis = gate.nis;
    if (!gate.accepted) {
        d.outcome = Outcome::Gated;
        return d;
    }
    (void)s6e_ekf_update::apply(s, updater, comp.compressed);
    d.outcome = Outcome::Applied;
    return d;
}

}  // namespace s6_msckf_update

}  // namespace branes::sdk::msckf::stages

#endif  // BRANES_SDK_MSCKF_STAGES_S6_MSCKF_UPDATE_HPP
