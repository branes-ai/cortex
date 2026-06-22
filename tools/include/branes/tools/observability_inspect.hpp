// SPDX-License-Identifier: MIT
//
// branes/tools/observability_inspect.hpp — the testable core of the real-data
// observability / null-space-leak diagnostic (obs_inspect.cpp, #212 / #337).
//
// The synthetic observability_probe (sdk/eval/observability_probe.hpp) proves the
// gauge-leak MECHANISM on a synthetic scene. This header is its real-data
// companion: the same three operations — build the stacked camera Jacobian H by
// driving the SHIPPED CameraUpdater::projection_jacobians over a clone window,
// build the analytic 4-DoF gauge N (global translation x3 + rotation-about-gravity
// x1), and measure the per-direction leak ‖H·N‖ — but parameterized so the
// obs_inspect tool can feed it the LIVE filter's clone window and the gate test
// can feed it a constructed one. Unlike the eval probe it takes the production
// updater (so it respects the real extrinsic) and uses fully-qualified types (no
// `using namespace`) so it composes with the backend headers without the
// Vec3-ambiguity that bites a `using namespace obs_detail` TU.
//
// A consistent filter must never gain information along the gauge: H·N ≈ 0. That
// holds only at a single consistent linearization — so the gate test asserts the
// consistent leak is ~machine-ε (the shipped Jacobian annihilates the gauge), and
// that under a perturbed clone window the TRANSLATION leak stays ≈ 0 (structurally
// protected by the ±Hf cancellation) while the YAW leak grows. Header-only,
// C++20, type-generic.

#ifndef BRANES_TOOLS_OBSERVABILITY_INSPECT_HPP
#define BRANES_TOOLS_OBSERVABILITY_INSPECT_HPP

#include <branes/math/lie/detail.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/invariant_update.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace branes::tools {

/// One pose in the clone window the leak machinery operates on: the live filter's
/// current clone estimate, or a perturbed copy of it.
template <math::Scalar T>
struct ObsPose {
    math::lie::SO3<T> R{};
    math::lie::detail::Vec<T, 3> p{};
};

/// Stacked camera measurement Jacobian over the clone window, driving the SHIPPED
/// CameraUpdater::projection_jacobians (so the gauge property of the REAL filter
/// code is what gets measured). Columns: [ per clone δθ(3) δp(3) | feature δp(3) ];
/// rows: 2 per clone. δθ block = Htheta, δp block = −Hf, feature = Hf — exactly the
/// layout CameraUpdater::update stacks into H.
template <math::Scalar T>
[[nodiscard]] sdk::msckf::DynMat<T> obs_build_H(const sdk::msckf::CameraUpdater<T>& updater,
                                                const std::vector<ObsPose<T>>& cl,
                                                const math::lie::detail::Vec<T, 3>& p_f) {
    const std::size_t m = cl.size();
    sdk::msckf::State<T> s(T{1});
    s.clones.reserve(m);
    for (std::size_t c = 0; c < m; ++c)
        s.clones.push_back({cl[c].R, cl[c].p, static_cast<double>(c)});
    sdk::msckf::DynMat<T> H(2 * m, 6 * m + 3);
    for (std::size_t c = 0; c < m; ++c) {
        sdk::msckf::CameraObservation<T> o;
        o.clone_index = c;
        o.camera_index = 0;
        const auto J = updater.projection_jacobians(s, o, p_f);
        for (std::size_t a = 0; a < 2; ++a)
            for (std::size_t b = 0; b < 3; ++b) {
                H(2 * c + a, 6 * c + b) = J.Htheta(a, b);   // δθ_c
                H(2 * c + a, 6 * c + 3 + b) = -J.Hf(a, b);  // δp_c = −Hf
                H(2 * c + a, 6 * m + b) = J.Hf(a, b);       // feature δp
            }
    }
    return H;
}

/// The analytic 4-DoF gauge over [ clones | feature ], at poses `cl` / feature
/// `p_f`. Cols 0–2: global translation (shift every clone + the feature by e_k).
/// Col 3: rotation about gravity g — δθ_c = R_c^T g, δp_c = [g]× p_c, δp_f = [g]× p_f.
template <math::Scalar T>
[[nodiscard]] sdk::msckf::DynMat<T> obs_build_N(const std::vector<ObsPose<T>>& cl,
                                                const math::lie::detail::Vec<T, 3>& p_f,
                                                const math::lie::detail::Vec<T, 3>& g) {
    const std::size_t m = cl.size();
    sdk::msckf::DynMat<T> N(6 * m + 3, 4);
    for (std::size_t c = 0; c < m; ++c)
        for (std::size_t k = 0; k < 3; ++k)
            N(6 * c + 3 + k, k) = T{1};  // translation: clone δp
    for (std::size_t k = 0; k < 3; ++k)
        N(6 * m + k, k) = T{1};  // translation: feature δp
    const math::lie::detail::Mat<T, 3, 3> gx = math::lie::detail::hat(g);
    for (std::size_t c = 0; c < m; ++c) {
        const math::lie::detail::Vec<T, 3> dth = cl[c].R.inverse() * g;  // R_c^T g
        const math::lie::detail::Vec<T, 3> dp = gx * cl[c].p;            // [g]× p_c
        for (std::size_t k = 0; k < 3; ++k) {
            N(6 * c + k, 3) = dth[k];
            N(6 * c + 3 + k, 3) = dp[k];
        }
    }
    const math::lie::detail::Vec<T, 3> dpf = gx * p_f;
    for (std::size_t k = 0; k < 3; ++k)
        N(6 * m + k, 3) = dpf[k];
    return N;
}

/// The RIGHT-INVARIANT (R-IEKF) stacked Jacobian over the clone window, driving the
/// SHIPPED build_invariant_measurement (invariant_update.hpp) — the world-frame
/// twist parameterization ξ_c = (φ, ρ). Same column layout as obs_build_H so it
/// composes with obs_build_N_invariant / obs_leak: [ per clone φ(3) ρ(3) | feature
/// δp(3) ]. This is the candidate fix's Jacobian, measured on the same real geometry.
template <math::Scalar T>
[[nodiscard]] sdk::msckf::DynMat<T> obs_build_H_invariant(const std::vector<ObsPose<T>>& cl,
                                                          const math::lie::detail::Vec<T, 3>& p_f,
                                                          const sdk::msckf::CameraExtrinsics<T>& ext) {
    const std::size_t m = cl.size();
    std::vector<sdk::msckf::InvariantClone<T>> ic;
    ic.reserve(m);
    for (const auto& c : cl)
        ic.push_back({c.R, c.p});
    std::vector<sdk::msckf::InvariantObs<T>> obs;
    obs.reserve(m);
    for (std::size_t c = 0; c < m; ++c)
        obs.push_back({c, 0, {}});  // xy only enters the residual, not H
    const std::vector<sdk::msckf::InvariantCalib<T>> cal{{ext.R_imu_cam, ext.p_imu_cam}};
    const auto M = sdk::msckf::build_invariant_measurement<T>(ic, obs, p_f, cal, /*estimate_calib=*/false);
    sdk::msckf::DynMat<T> H(2 * m, 6 * m + 3);
    if (!M.ok)
        return H;               // degenerate window ⇒ zeros (leak 0)
    const std::size_t n = M.n;  // 6*m (no calib columns)
    for (std::size_t row = 0; row < 2 * m; ++row) {
        for (std::size_t cc = 0; cc < 6 * m; ++cc)
            H(row, cc) = M.H_x[row * n + cc];  // [φ_c ρ_c] per clone
        for (std::size_t b = 0; b < 3; ++b)
            H(row, 6 * m + b) = M.H_f[row * 3 + b];  // feature
    }
    return H;
}

/// The invariant 4-DoF gauge. Crucially the CLONE directions are CONSTANTS (no
/// estimate): translation ρ = e_k, yaw φ = ĝ for EVERY clone — so a perturbed
/// window cannot break the annihilation. Only the (marginalized) feature carries
/// the estimate. Cols 0–2 translation, col 3 yaw.
template <math::Scalar T>
[[nodiscard]] sdk::msckf::DynMat<T> obs_build_N_invariant(const std::vector<ObsPose<T>>& cl,
                                                          const math::lie::detail::Vec<T, 3>& p_f,
                                                          const math::lie::detail::Vec<T, 3>& g) {
    const std::size_t m = cl.size();
    sdk::msckf::DynMat<T> N(6 * m + 3, 4);
    for (std::size_t c = 0; c < m; ++c)
        for (std::size_t k = 0; k < 3; ++k) {
            N(6 * c + 3 + k, k) = T{1};  // translation: clone ρ = e_k (constant)
            N(6 * c + k, 3) = g[k];      // yaw: clone φ = ĝ (constant, estimate-independent)
        }
    for (std::size_t k = 0; k < 3; ++k)
        N(6 * m + k, k) = T{1};  // translation: feature δp
    const math::lie::detail::Vec<T, 3> dpf = math::lie::detail::hat(g) * p_f;
    for (std::size_t k = 0; k < 3; ++k)
        N(6 * m + k, 3) = dpf[k];  // yaw: feature [g]× p_f
    return N;
}

/// Per-gauge-direction leak: column norms of H·N. `.first` = translation (RSS of
/// cols 0–2), `.second` = yaw (norm of col 3).
template <math::Scalar T>
[[nodiscard]] std::pair<T, T> obs_leak(const sdk::msckf::DynMat<T>& H, const sdk::msckf::DynMat<T>& N) {
    const sdk::msckf::DynMat<T> HN = sdk::msckf::mul(H, N);
    using std::sqrt;
    T tr = T{0}, yaw = T{0};
    for (std::size_t i = 0; i < HN.rows; ++i) {
        for (std::size_t k = 0; k < 3; ++k)
            tr += HN(i, k) * HN(i, k);
        yaw += HN(i, 3) * HN(i, 3);
    }
    return {sqrt(tr), sqrt(yaw)};
}

/// Portable reproducible draw in [−1, 1) from a 64-bit engine (matches
/// observability_probe::urand), so the perturbation sweep is deterministic
/// everywhere.
template <math::Scalar T, class Rng>
[[nodiscard]] T obs_urand(Rng& rng) {
    return T(static_cast<double>(rng() >> 11) * (2.0 / 9007199254740992.0) - 1.0);
}

}  // namespace branes::tools

#endif  // BRANES_TOOLS_OBSERVABILITY_INSPECT_HPP
