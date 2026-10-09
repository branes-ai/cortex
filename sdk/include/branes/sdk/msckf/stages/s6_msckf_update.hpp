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
//
// S6b and S6c also take an alternative method for the benches to compare (#457):
// Givens rotations instead of Householder reflectors for the null space, and a
// QR compression of a stacked multi-feature system instead of the identity.
// The shipped sequence below uses neither.
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
#include <branes/sdk/msckf/qr.hpp>
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

/// How the left null space of H_f is applied.
enum class Method {
    Householder,  ///< shipped: three reflectors (CameraUpdater::nullspace_project)
    Givens,       ///< plane rotations, column by column, bottom-up
};

/// Givens variant: zero the sub-diagonal of H_f's three columns with plane
/// rotations applied across [H_f | H_x | r], and keep the bottom rows − 3. The
/// rows span the same left null space as the Householder projection; they differ
/// from it by an orthogonal change of basis, so HᵀH and Hᵀr agree.
template <math::Scalar T>
[[nodiscard]] Result<T> apply_givens(const MeasurementSystem<T>& sys) {
    Result<T> out;
    const std::size_t m = sys.rows, n = sys.cols, cols = 3 + n + 1;
    if (m <= 3)
        return out;
    std::vector<T> a(m * cols, T{0});
    for (std::size_t i = 0; i < m; ++i) {
        for (std::size_t j = 0; j < 3; ++j)
            a[i * cols + j] = sys.Hf[i * 3 + j];
        for (std::size_t j = 0; j < n; ++j)
            a[i * cols + 3 + j] = sys.Hx[i * n + j];
        a[i * cols + 3 + n] = sys.r[i];
    }
    using math::lie::detail::sqrt_;
    for (std::size_t c = 0; c < 3; ++c)
        for (std::size_t i = m - 1; i > c; --i) {
            // Rotate rows (i−1, i) so that a(i, c) = 0.
            const T x = a[(i - 1) * cols + c], y = a[i * cols + c];
            if (y == T{0})
                continue;
            const T rr = sqrt_(x * x + y * y);
            const T cs = x / rr, sn = y / rr;
            for (std::size_t j = 0; j < cols; ++j) {
                const T u = a[(i - 1) * cols + j], v = a[i * cols + j];
                a[(i - 1) * cols + j] = cs * u + sn * v;
                a[i * cols + j] = cs * v - sn * u;
            }
        }
    out.projected.H = DynMat<T>(m - 3, n);
    out.projected.r.resize(m - 3);
    for (std::size_t i = 0; i < m - 3; ++i) {
        for (std::size_t j = 0; j < n; ++j)
            out.projected.H(i, j) = a[(i + 3) * cols + 3 + j];
        out.projected.r[i] = a[(i + 3) * cols + 3 + n];
    }
    out.ok = true;
    return out;
}

template <math::Scalar T>
[[nodiscard]] Result<T> apply(const MeasurementSystem<T>& system, Method method) {
    return method == Method::Givens ? apply_givens(system) : apply(system);
}

}  // namespace s6b_nullspace_projection

// ── S6c: measurement compression ──────────────────────────────────────────
namespace s6c_compression {

template <math::Scalar T>
struct Result {
    ProjectedMeasurement<T> compressed;
    std::size_t rows_in = 0;
    std::size_t rows_out = 0;
    bool compressed_applied = false;  ///< the shipped path compresses nothing (see below)
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

/// The compression applied.
enum class Method {
    Identity,  ///< shipped: no compression (one feature at a time)
    Qr,        ///< QR of the stacked [H | r]: at most n + 1 rows survive
};

/// QR variant: [H | r] = Q·R, keep R's leading min(rows, n + 1) rows. With n + 1
/// columns the residual column is kept whole, so ‖r_c‖ = ‖r‖, and HᵀH and Hᵀr
/// are preserved exactly. Only a system taller than n + 1 rows (a stack of
/// features) actually shrinks.
template <math::Scalar T>
[[nodiscard]] Result<T> apply(ProjectedMeasurement<T> projected, Method method) {
    if (method != Method::Qr || projected.H.rows <= projected.H.cols + 1)
        return apply(std::move(projected));
    const std::size_t k = projected.H.rows, n = projected.H.cols;
    DynMat<T> a(k, n + 1);
    for (std::size_t i = 0; i < k; ++i) {
        for (std::size_t j = 0; j < n; ++j)
            a(i, j) = projected.H(i, j);
        a(i, n) = projected.r[i];
    }
    const DynMat<T> rf = householder_qr_r(a);
    Result<T> out;
    out.rows_in = k;
    out.rows_out = rf.rows;
    out.compressed.H = DynMat<T>(rf.rows, n);
    out.compressed.r.resize(rf.rows);
    for (std::size_t i = 0; i < rf.rows; ++i) {
        for (std::size_t j = 0; j < n; ++j)
            out.compressed.H(i, j) = rf(i, j);
        out.compressed.r[i] = rf(i, n);
    }
    out.compressed_applied = true;
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
