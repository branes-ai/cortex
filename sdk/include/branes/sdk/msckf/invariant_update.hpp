// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/invariant_update.hpp — the camera measurement update for the
// Right-Invariant EKF backend (issue #347, Phase B). The complement of
// invariant_propagator.hpp: it marginalizes a feature and corrects a window of
// cloned poses in the RIGHT-INVARIANT (world-frame) error.
//
// Two pieces differ from the body-frame CameraUpdater:
//   • the measurement Jacobian is built w.r.t. the world-frame clone twist
//     ξ_c = (φ, ρ) — R̂_c = Exp(φ)·R_c, p̂_c = Exp(φ)·p_c + ρ — giving
//       ∂h/∂φ = dh·R_cᵀ·[p_f]×,  ∂h/∂ρ = −dh·R_cᵀ,  ∂h/∂p_f = dh·R_cᵀ.
//     The gauge directions this annihilates are STATE-INDEPENDENT constants, so it
//     never fabricates yaw information (Phase A: flat yaw leak at every σ);
//   • the EKF correction is retracted by the right-invariant box-plus
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
    math::lie::SO3<T> R{};
    math::lie::detail::Vec<T, 3> p{};
};

/// One observation: which clone saw the feature, in normalized image coords.
template <math::Scalar T>
struct InvariantObs {
    std::size_t clone_index = 0;
    math::lie::detail::Vec<T, 2> xy{};
};

/// Right-invariant box-plus on a clone pose: R ← Exp(φ)·R, p ← Exp(φ)·p + ρ.
template <math::Scalar T>
void retract_invariant(InvariantClone<T>& c,
                       const math::lie::detail::Vec<T, 3>& phi,
                       const math::lie::detail::Vec<T, 3>& rho) {
    const math::lie::SO3<T> dR = math::lie::SO3<T>::exp(phi);
    c.R = dR * c.R;
    const auto p = dR * c.p;
    c.p = math::lie::detail::Vec<T, 3>{{p[0] + rho[0], p[1] + rho[1], p[2] + rho[2]}};
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
                                                                  const math::lie::detail::Vec<T, 3>& p_f) {
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Mat3 = math::lie::detail::Mat<T, 3, 3>;
    const std::size_t m = obs.size();
    const std::size_t n = 6 * clones.size();
    const std::size_t rows2 = 2 * m;
    InvariantMeasurement<T> M;
    M.rows2 = rows2;
    M.n = n;
    M.H_f.assign(rows2 * 3, T{0});
    M.H_x.assign(rows2 * n, T{0});
    M.r.assign(rows2, T{0});

    for (std::size_t i = 0; i < m; ++i) {
        const auto& c = clones[obs[i].clone_index];
        const Mat3 Rct = c.R.inverse().matrix();  // R_cᵀ (identity extrinsic)
        const Vec3 y = Rct * Vec3{{p_f[0] - c.p[0], p_f[1] - c.p[1], p_f[2] - c.p[2]}};
        if (!(y[2] > T{0}))
            return M;  // ok stays false
        const T inv = T{1} / y[2];
        math::lie::detail::Mat<T, 2, 3> dh{};
        dh(0, 0) = inv;
        dh(0, 2) = -y[0] * inv * inv;
        dh(1, 1) = inv;
        dh(1, 2) = -y[1] * inv * inv;
        const math::lie::detail::Mat<T, 2, 3> dhRct = dh * Rct;                            // ∂h/∂p_f = dh·R_cᵀ
        const math::lie::detail::Mat<T, 2, 3> Hphi = dhRct * math::lie::detail::hat(p_f);  // dh·R_cᵀ·[p_f]×
        const std::size_t row = 2 * i, off = 6 * obs[i].clone_index;
        for (std::size_t a = 0; a < 2; ++a) {
            M.r[row + a] = obs[i].xy[a] - y[a] * inv;
            for (std::size_t b = 0; b < 3; ++b) {
                M.H_f[(row + a) * 3 + b] = dhRct(a, b);             // feature
                M.H_x[(row + a) * n + off + b] = Hphi(a, b);        // φ_c
                M.H_x[(row + a) * n + off + 3 + b] = -dhRct(a, b);  // ρ_c
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
                      const math::lie::detail::Vec<T, 3>& p_f,
                      T normalized_sigma = T{1} / T{100}) {
    using Vec3 = math::lie::detail::Vec<T, 3>;
    if (obs.size() < 2)
        return false;
    const std::size_t n = 6 * clones.size();
    const InvariantMeasurement<T> M = build_invariant_measurement<T>(clones, obs, p_f);
    if (!M.ok)
        return false;
    const std::size_t rows2 = M.rows2;
    const auto proj = features::msckf_left_nullspace_project<T>(M.H_f, M.H_x, M.r, rows2, n);
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
