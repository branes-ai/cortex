// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/invariant_update.hpp — the camera measurement update for the
// Right-Invariant EKF backend (issue #347, Phase B). The complement of
// invariant_propagator.hpp: it marginalizes a feature and corrects a window of
// cloned poses in the LEFT-INVARIANT (world-frame) error.
//
// Two pieces differ from the body-frame CameraUpdater:
//   • the measurement Jacobian is built w.r.t. the world-frame clone twist
//     ξ_c = (φ, ρ) — R̂_c = Exp(φ)·R_c, p̂_c = Exp(φ)·p_c + ρ — giving
//       ∂h/∂φ = dh·R_cᵀ·[p_f]×,  ∂h/∂ρ = −dh·R_cᵀ,  ∂h/∂p_f = dh·R_cᵀ.
//     The gauge directions this annihilates are STATE-INDEPENDENT constants, so it
//     never fabricates yaw information (Phase A: flat yaw leak at every σ);
//   • the EKF correction is retracted by the left-invariant box-plus
//     R_c ← Exp(δφ)·R_c, p_c ← Exp(δφ)·p_c + δρ (world-frame), not the body-frame
//     R_c ← R_c·Exp(δθ).
//
// The left-null-space feature marginalization and the Joseph covariance update are
// shared with the body-frame path. Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_INVARIANT_UPDATE_HPP
#define BRANES_SDK_MSCKF_INVARIANT_UPDATE_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/features/msckf_nullspace.hpp>
#include <branes/sdk/msckf/dense.hpp>

#include <cstddef>
#include <span>
#include <vector>

namespace branes::sdk::msckf {

/// A world-frame clone pose for the invariant window.
template <math::Scalar T>
struct InvariantClone {
    using SO3 = branes::math::lie::SO3<T>;
    using Vec3 = branes::math::lie::detail::Vec<T, 3>;

    SO3 R{};
    Vec3 p{};
};

/// One observation: which clone saw the feature, in normalized image coords.
template <math::Scalar T>
struct InvariantObs {
    std::size_t clone_index = 0;
    std::size_t camera_index = 0;
    branes::math::lie::detail::Vec<T, 2> xy{};
};

/// Camera↔IMU extrinsic calibration state.
template <math::Scalar T>
struct InvariantCalib {
    using SO3 = branes::math::lie::SO3<T>;
    using Vec3 = branes::math::lie::detail::Vec<T, 3>;

    SO3 R_imu_cam{};
    Vec3 p_imu_cam{};
};

/// Left-invariant box-plus on a clone pose: R ← Exp(φ)·R, p ← Exp(φ)·p + ρ.
template <math::Scalar T>
void retract_invariant(InvariantClone<T>& c,
                       const branes::math::lie::detail::Vec<T, 3>& phi,
                       const branes::math::lie::detail::Vec<T, 3>& rho) {
    const branes::math::lie::SO3<T> dR = branes::math::lie::SO3<T>::exp(phi);
    c.R = dR * c.R;
    c.p = dR * c.p + rho;
}

/// Linear (DLT-style) triangulation of a feature from the clone window: solve the
/// least-squares intersection Σ (I − d̂d̂ᵀ)(p_f − p_c) = 0, where d = R_c·[xy;1] is
/// the world-frame bearing. `ok` is false on degenerate (near-coplanar) geometry.
template <math::Scalar T>
struct InvariantTriangulation {
    branes::math::lie::detail::Vec<T, 3> p_f{};
    bool ok = false;
};

template <math::Scalar T>
[[nodiscard]] InvariantTriangulation<T> triangulate_invariant(const std::vector<InvariantClone<T>>& clones,
                                                              const std::vector<InvariantObs<T>>& obs,
                                                              const std::vector<InvariantCalib<T>>& calib = {}) {
    using Vec3 = branes::math::lie::detail::Vec<T, 3>;
    using Mat3 = branes::math::lie::detail::Mat<T, 3, 3>;
    Mat3 A{};
    Vec3 b{};
    for (const auto& o : obs) {
        if (o.clone_index >= clones.size())
            return {};
        // Early validation: ensure camera_index is within bounds of calib if provided, or 0 if empty.
        if ((!calib.empty() && o.camera_index >= calib.size()) || (calib.empty() && o.camera_index != 0))
            return {};

        const auto& c = clones[o.clone_index];
        const auto R_imu_cam = !calib.empty() && o.camera_index < calib.size() ? calib[o.camera_index].R_imu_cam
                                                                               : branes::math::lie::SO3<T>{};
        const auto p_imu_cam =
            !calib.empty() && o.camera_index < calib.size() ? calib[o.camera_index].p_imu_cam : Vec3{};
        const auto R_cam = c.R * R_imu_cam;
        const auto p_cam = c.p + c.R * p_imu_cam;
        const Vec3 d = R_cam.matrix() * Vec3{{o.xy[0], o.xy[1], T{1}}};
        const T n2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        const Vec3& pc = p_cam;
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j) {
                const T P = (i == j ? T{1} : T{0}) - d[i] * d[j] / n2;
                A(i, j) += P;
                b[i] += P * pc[j];
            }
    }
    const T dt = A(0, 0) * (A(1, 1) * A(2, 2) - A(1, 2) * A(2, 1)) - A(0, 1) * (A(1, 0) * A(2, 2) - A(1, 2) * A(2, 0)) +
                 A(0, 2) * (A(1, 0) * A(2, 1) - A(1, 1) * A(2, 0));
    const T mean_eig = (A(0, 0) + A(1, 1) + A(2, 2)) / T{3};
    // 1e-6 conditioning floor. Stricter thresholds (like 1e-4) falsely reject
    // highly-clustered yet valid tracks in specific geometries (such as diagnostic worlds).
    const T eps_rel = T{1} / T{1000000};
    if (!(dt > eps_rel * mean_eig * mean_eig * mean_eig))
        return {};
    auto cof = [&](std::size_t i, std::size_t j) {
        const std::size_t a1 = (j + 1) % 3, a2 = (j + 2) % 3, b1 = (i + 1) % 3, b2 = (i + 2) % 3;
        return (A(b1, a1) * A(b2, a2) - A(b1, a2) * A(b2, a1)) / dt;
    };
    Vec3 x{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            x[i] += cof(i, j) * b[j];
    return {x, true};
}

/// The stacked invariant measurement of one feature track over the clone window,
/// before feature marginalization: H_x (2m×6·#clones, world-frame clone twists),
/// H_f (2m×3, feature), and the residual r (2m). `ok` is false on a cheirality
/// violation (a clone with the feature behind it). Exposed so observability tests
/// can confirm the SHIPPED H annihilates the gauge.
template <math::Scalar T>
struct InvariantMeasurement {
    std::vector<T> H_x, H_f, r;
    std::size_t rows2 = 0, n = 0;
    bool ok = false;
};

template <math::Scalar T>
[[nodiscard]] InvariantMeasurement<T> build_invariant_measurement(const std::vector<InvariantClone<T>>& clones,
                                                                  const std::vector<InvariantObs<T>>& obs,
                                                                  const branes::math::lie::detail::Vec<T, 3>& p_f,
                                                                  const std::vector<InvariantCalib<T>>& calib = {},
                                                                  bool estimate_calib = false) {
    using Vec3 = branes::math::lie::detail::Vec<T, 3>;
    using Mat3 = branes::math::lie::detail::Mat<T, 3, 3>;
    const std::size_t m = obs.size();
    const std::size_t n = estimate_calib ? (6 * clones.size() + 6 * calib.size()) : (6 * clones.size());
    const std::size_t rows2 = 2 * m;
    InvariantMeasurement<T> M;
    M.rows2 = rows2;
    M.n = n;
    M.H_f.reserve(rows2 * 3);
    M.H_f.assign(rows2 * 3, T{0});
    M.H_x.reserve(rows2 * n);
    M.H_x.assign(rows2 * n, T{0});
    M.r.reserve(rows2);
    M.r.assign(rows2, T{0});

    constexpr T kMinDepth = T{1} / T{1000};  // 1 mm

    for (std::size_t i = 0; i < m; ++i) {
        if (obs[i].clone_index >= clones.size())
            return M;  // out-of-range clone reference; ok stays false
        // Early validation: ensure camera_index is within bounds of calib if provided, or 0 if empty.
        if ((!calib.empty() && obs[i].camera_index >= calib.size()) || (calib.empty() && obs[i].camera_index != 0))
            return M;  // invalid camera index, reject track

        const auto& cl = clones[obs[i].clone_index];
        const auto R_imu_cam = !calib.empty() && obs[i].camera_index < calib.size()
                                   ? calib[obs[i].camera_index].R_imu_cam
                                   : branes::math::lie::SO3<T>{};
        const auto p_imu_cam =
            !calib.empty() && obs[i].camera_index < calib.size() ? calib[obs[i].camera_index].p_imu_cam : Vec3{};
        const auto R_cam = cl.R * R_imu_cam;
        const auto p_cam = cl.p + cl.R * p_imu_cam;
        const Mat3 Rct = R_cam.inverse().matrix();  // R_cᵀ
        const Vec3 y = Rct * (p_f - p_cam);
        if (!(y[2] > kMinDepth))
            return M;  // ok stays false

        const T inv = T{1} / y[2];
        branes::math::lie::detail::Mat<T, 2, 3> dh{};
        dh(0, 0) = inv;
        dh(0, 2) = -y[0] * inv * inv;
        dh(1, 1) = inv;
        dh(1, 2) = -y[1] * inv * inv;

        const branes::math::lie::detail::Mat<T, 2, 3> dhRct = dh * Rct;  // ∂h/∂p_f = dh·R_cᵀ
        const branes::math::lie::detail::Mat<T, 2, 3> Hphi =
            (dhRct * branes::math::lie::detail::hat(p_f));  // ∂h/∂φ = dh·R_cᵀ·[p_f]×
        const std::size_t row = 2 * i, off = 6 * obs[i].clone_index;

        // Compute calibration Jacobians once per observation above the nested loop.
        // Hext_theta is derived assuming right-perturbation convention: R_ic' = R_ic * Exp(dtheta).
        // This makes the Jacobian consistent with the right-multiplication retraction in the backend.
        // ∂h_c/∂δθ_ic = dh·[y]×  where y is the camera-frame landmark point.
        // ∂h_c/∂δp_ic = −dh·R_icᵀ
        const Mat3 Rict = R_imu_cam.inverse().matrix();
        const branes::math::lie::detail::Mat<T, 2, 3> Hext_theta = dh * branes::math::lie::detail::hat(y);
        const branes::math::lie::detail::Mat<T, 2, 3> Hext_p = (dh * Rict) * T{-1};
        const std::size_t off_calib =
            estimate_calib && !calib.empty() ? (6 * clones.size() + 6 * obs[i].camera_index) : 0;

        for (std::size_t a = 0; a < 2; ++a) {
            M.r[row + a] = obs[i].xy[a] - y[a] * inv;
            for (std::size_t b = 0; b < 3; ++b) {
                M.H_f[(row + a) * 3 + b] = dhRct(a, b);             // feature
                M.H_x[(row + a) * n + off + b] = Hphi(a, b);        // φ_c
                M.H_x[(row + a) * n + off + 3 + b] = -dhRct(a, b);  // ρ_c
            }
            if (estimate_calib && !calib.empty() && obs[i].camera_index < calib.size()) {
                for (std::size_t b = 0; b < 3; ++b) {
                    M.H_x[(row + a) * n + off_calib + b] = Hext_theta(a, b);
                    M.H_x[(row + a) * n + off_calib + 3 + b] = Hext_p(a, b);
                }
            }
        }
    }
    M.ok = true;
    return M;
}

/// Apply one feature track to the invariant clone window. `clones`/`cov` carry the
/// world-frame state; `p_f` is the (already triangulated) world feature. Builds the
/// invariant H, marginalizes the feature, runs the EKF update, and retracts each
/// clone. Returns true iff the state was updated. `cov` is the right-invariant
/// covariance over [φ_c, ρ_c] per clone (6·#clones).
template <math::Scalar T, class Cov>
bool invariant_update(std::vector<InvariantClone<T>>& clones,
                      Cov& cov,
                      const std::vector<InvariantObs<T>>& obs,
                      const branes::math::lie::detail::Vec<T, 3>& p_f,
                      T normalized_sigma = T{1} / T{100},
                      const std::vector<InvariantCalib<T>>& calib = {}) {
    using Vec3 = branes::math::lie::detail::Vec<T, 3>;
    if (obs.size() < 2)
        return false;
    if (!calib.empty())
        return false;  // reject calibration to preserve EKF consistency in stand-alone update helper

    const std::size_t n = 6 * clones.size();
    const InvariantMeasurement<T> M = build_invariant_measurement<T>(clones, obs, p_f);
    if (!M.ok)
        return false;
    const std::size_t rows2 = M.rows2;
    const auto proj = features::msckf_left_nullspace_project<T>(M.H_f, M.H_x, M.r, M.rows2, n);
    if (proj.rows == 0)
        return false;
    DynMat<T> H(proj.rows, n);
    for (std::size_t i = 0; i < proj.rows; ++i)
        for (std::size_t j = 0; j < n; ++j)
            H(i, j) = proj.H_x[i * n + j];
    const T r2 = normalized_sigma * normalized_sigma;
    const std::vector<T> R_diag(proj.rows, r2);
    const std::vector<T> dx = cov.update(H, std::span<const T>{proj.r}, std::span<const T>{R_diag});

    // Retract each clone by its world-frame correction [δφ; δρ].
    for (std::size_t c = 0; c < clones.size(); ++c) {
        const std::size_t off = 6 * c;
        retract_invariant<T>(
            clones[c], Vec3{{dx[off], dx[off + 1], dx[off + 2]}}, Vec3{{dx[off + 3], dx[off + 4], dx[off + 5]}});
    }
    return true;
}

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_INVARIANT_UPDATE_HPP
