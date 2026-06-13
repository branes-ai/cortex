// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/observability_probe.hpp — the VIO observability null-space
// probe (issue #337). The S2.5 propagation probe checks that Φ preserves the
// global-position null space, but explicitly defers the *yaw* direction as "a
// JOINT property requiring the S6 measurement Jacobian". This is that probe.
//
// A monocular VIO system has exactly four unobservable degrees of freedom under
// general motion: global translation (3) and rotation about gravity — yaw (1).
// (With the IMU, scale is observable; it degrades only under low acceleration.)
// A consistent filter must NEVER gain information along these directions: the
// stacked camera measurement Jacobian H must annihilate the null space, H·N ≈ 0,
// so the corresponding covariance is not spuriously shrunk.
//
// The catch the OC-VIO literature (Huang/Hesch) formalizes: H·N = 0 holds only
// when H is linearized at a SINGLE consistent point. A standard EKF re-linearizes
// at the evolving estimate every step, so across a window the gauge direction at
// clone i's linearization point disagrees with clone j's, and H·N ≠ 0 — the
// filter fabricates information along the null space and becomes over-confident.
// That is the mechanism behind the measured EuRoC attitude-NEES ≈ 993 on the slow
// V1_01 sequence (NIS a healthy 1.5): innovations look fine, the *state* drifts
// off the unobservable manifold while the covariance refuses to grow.
//
// This probe MEASURES the leak directly. On a synthetic clone window + features:
//   • build the analytic 4-DoF null space N (translation ×3, gravity-yaw ×1);
//   • CONSISTENT case — H linearized at the true poses ⇒ ‖H·N‖ ≈ 0 (machine ε):
//     the measurement model has the correct null space when linearized honestly;
//   • INCONSISTENT case — H linearized at a PERTURBED estimate, N at the true
//     gauge ⇒ translation stays ≈ 0 (the clone-δp and feature columns are ±Hf and
//     cancel for any single H), but YAW leaks and grows with the perturbation —
//     the filter gains spurious information about its rotation-about-gravity.
//
// The yaw-leak-vs-perturbation curve is the acceptance test any FEJ / OC-EKF fix
// must drive back to ~0. Native units: dimensionless Jacobian-row norms.
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_OBSERVABILITY_PROBE_HPP
#define BRANES_SDK_EVAL_OBSERVABILITY_PROBE_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/dense.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace branes::sdk::eval {

namespace obs_detail {
template <math::Scalar T>
using Vec3 = math::lie::detail::Vec<T, 3>;
template <math::Scalar T>
using Mat3 = math::lie::detail::Mat<T, 3, 3>;
template <math::Scalar T>
using SO3 = math::lie::SO3<T>;

template <math::Scalar T>
struct Pose {
    SO3<T> R{};
    Vec3<T> p{};
};

template <math::Scalar T>
struct Scene {
    std::vector<Pose<T>> clones;
    std::vector<Vec3<T>> feats;
};

// A clone window with mild rotation diversity along a translating baseline, and
// features spread across the field of view at moderate depth (healthy parallax).
template <math::Scalar T>
[[nodiscard]] Scene<T> make_scene(std::size_t m, std::size_t k) {
    Scene<T> s;
    for (std::size_t c = 0; c < m; ++c) {
        const T u = static_cast<T>(c);
        s.clones.push_back(
            {SO3<T>::exp(
                 Vec3<T>{{T{4} / T{100} * std::sin(u), T{3} / T{100} * u - T{1} / T{10}, T{5} / T{100} * std::cos(u)}}),
             Vec3<T>{{T{3} / T{10} * u, T{1} / T{10} * std::sin(u), T{5} / T{100} * u}}});
    }
    // Spread `k` features across a repeating 5×5 bearing grid at increasing depth —
    // honours any requested count (not just a 3×3 = 9 cap).
    for (std::size_t made = 0; made < k; ++made) {
        const int ix = static_cast<int>(made % 5) - 2;
        const int iy = static_cast<int>((made / 5) % 5) - 2;
        s.feats.push_back(Vec3<T>{{static_cast<T>(ix), static_cast<T>(iy), T{6} + T{1} / T{4} * static_cast<T>(made)}});
    }
    return s;
}

// Stacked camera measurement Jacobian (identity extrinsic; the gauge argument is
// independent of T_CI). Columns: [ per clone δθ(3) δp(3) | per feature δp(3) ].
// Rows: 2 per (clone, feature) observation. This is exactly what the MSCKF update
// builds (camera_updater.hpp): δθ block = dh·[y]×, δp block = −Hf, feature = Hf.
template <math::Scalar T>
[[nodiscard]] msckf::DynMat<T> build_H(const std::vector<Pose<T>>& cl, const std::vector<Vec3<T>>& ff) {
    const std::size_t m = cl.size(), K = ff.size();
    msckf::DynMat<T> H(2 * m * K, 6 * m + 3 * K);
    std::size_t row = 0;
    for (std::size_t c = 0; c < m; ++c) {
        const Mat3<T> Rct = cl[c].R.inverse().matrix();  // identity extrinsic ⇒ R_cam^T = R_clone^T
        for (std::size_t f = 0; f < K; ++f, row += 2) {
            const Vec3<T> y = Rct * (ff[f] - cl[c].p);  // feature in the camera frame
            const T inv = T{1} / y[2];
            math::lie::detail::Mat<T, 2, 3> dh{};
            dh(0, 0) = inv;
            dh(0, 2) = -y[0] * inv * inv;
            dh(1, 1) = inv;
            dh(1, 2) = -y[1] * inv * inv;
            const math::lie::detail::Mat<T, 2, 3> Hf = dh * Rct;                         // ∂h/∂p_f
            const math::lie::detail::Mat<T, 2, 3> Hth = dh * math::lie::detail::hat(y);  // ∂h/∂δθ_c
            for (std::size_t a = 0; a < 2; ++a)
                for (std::size_t b = 0; b < 3; ++b) {
                    H(row + a, 6 * c + b) = Hth(a, b);         // δθ_c
                    H(row + a, 6 * c + 3 + b) = -Hf(a, b);     // δp_c = −Hf
                    H(row + a, 6 * m + 3 * f + b) = Hf(a, b);  // δp_f
                }
        }
    }
    return H;
}

// The analytic 4-DoF unobservable subspace over [clones | features], evaluated at
// `cl`/`ff` (reality's gauge). Cols 0–2: global translation (shift every clone and
// feature position by e_k). Col 3: rotation about gravity g — δθ_c = R_c^T g,
// δp_c = [g]× p_c, δp_f = [g]× p_f.
template <math::Scalar T>
[[nodiscard]] msckf::DynMat<T>
build_N(const std::vector<Pose<T>>& cl, const std::vector<Vec3<T>>& ff, const Vec3<T>& g) {
    const std::size_t m = cl.size(), K = ff.size();
    msckf::DynMat<T> N(6 * m + 3 * K, 4);
    for (std::size_t c = 0; c < m; ++c)
        for (std::size_t k = 0; k < 3; ++k)
            N(6 * c + 3 + k, k) = T{1};  // translation: clone δp
    for (std::size_t f = 0; f < K; ++f)
        for (std::size_t k = 0; k < 3; ++k)
            N(6 * m + 3 * f + k, k) = T{1};  // translation: feature δp
    const Mat3<T> gx = math::lie::detail::hat(g);
    for (std::size_t c = 0; c < m; ++c) {
        const Vec3<T> dth = cl[c].R.inverse() * g;  // R_c^T g
        const Vec3<T> dp = gx * cl[c].p;            // [g]× p_c
        for (std::size_t k = 0; k < 3; ++k) {
            N(6 * c + k, 3) = dth[k];
            N(6 * c + 3 + k, 3) = dp[k];
        }
    }
    for (std::size_t f = 0; f < K; ++f) {
        const Vec3<T> dp = gx * ff[f];  // [g]× p_f
        for (std::size_t k = 0; k < 3; ++k)
            N(6 * m + 3 * f + k, 3) = dp[k];
    }
    return N;
}

// Per-gauge-direction leak: column norms of H·N. Translation = RSS of cols 0–2,
// yaw = norm of col 3.
template <math::Scalar T>
[[nodiscard]] std::pair<T, T> leak(const msckf::DynMat<T>& H, const msckf::DynMat<T>& N) {
    const msckf::DynMat<T> HN = msckf::mul(H, N);
    using std::sqrt;
    T tr = T{0}, yaw = T{0};
    for (std::size_t i = 0; i < HN.rows; ++i) {
        for (std::size_t k = 0; k < 3; ++k)
            tr += HN(i, k) * HN(i, k);
        yaw += HN(i, 3) * HN(i, 3);
    }
    return {sqrt(tr), sqrt(yaw)};
}

// ── Right-invariant (R-IEKF) parameterization, issue #348 ──────────────────
//
// The body-frame error above leaks yaw because its gauge direction is R_c^T·ĝ —
// it DEPENDS on each clone's estimate, so across a drifted window the per-clone
// directions disagree and the stacked H stops annihilating them. The RIGHT-
// INVARIANT (world-frame) error replaces the clone perturbation with a twist
// ξ_c = (φ, ρ): R̂_c = Exp(φ)·R_c, p̂_c = Exp(φ)·p_c + ρ. Then the gauge directions
// become STATE-INDEPENDENT constants — global yaw is φ = ĝ for EVERY clone, global
// translation is ρ = e_k — and H annihilates them regardless of the linearization
// point. That is R-IEKF's "observable by construction".
//
// Invariant measurement Jacobian (identity extrinsic). Per (clone,feature), with
// y = R_c^T(p_f − p_c) and dh = ∂(π)/∂p_c: derived from
//   ŷ ≈ y + R_c^T[p_f]×·φ − R_c^T·ρ  (clone twist),  ∂h/∂p_f = dh·R_c^T:
//   ∂h/∂φ_c = dh·R_c^T·[p_f]×   ∂h/∂ρ_c = −dh·R_c^T   ∂h/∂δp_f = dh·R_c^T.
// Columns: [ per clone φ(3) ρ(3) | per feature δp(3) ].
template <math::Scalar T>
[[nodiscard]] msckf::DynMat<T> build_H_invariant(const std::vector<Pose<T>>& cl, const std::vector<Vec3<T>>& ff) {
    const std::size_t m = cl.size(), K = ff.size();
    msckf::DynMat<T> H(2 * m * K, 6 * m + 3 * K);
    std::size_t row = 0;
    for (std::size_t c = 0; c < m; ++c) {
        const Mat3<T> Rct = cl[c].R.inverse().matrix();
        for (std::size_t f = 0; f < K; ++f, row += 2) {
            const Vec3<T> y = Rct * (ff[f] - cl[c].p);
            const T inv = T{1} / y[2];
            math::lie::detail::Mat<T, 2, 3> dh{};
            dh(0, 0) = inv;
            dh(0, 2) = -y[0] * inv * inv;
            dh(1, 1) = inv;
            dh(1, 2) = -y[1] * inv * inv;
            const math::lie::detail::Mat<T, 2, 3> dhRct = dh * Rct;  // ∂h/∂p_f = dh·R_c^T
            const math::lie::detail::Mat<T, 2, 3> Hphi = dhRct * math::lie::detail::hat(ff[f]);  // dh·R_c^T·[p_f]×
            for (std::size_t a = 0; a < 2; ++a)
                for (std::size_t b = 0; b < 3; ++b) {
                    H(row + a, 6 * c + b) = Hphi(a, b);           // φ_c
                    H(row + a, 6 * c + 3 + b) = -dhRct(a, b);     // ρ_c = −dh·R_c^T
                    H(row + a, 6 * m + 3 * f + b) = dhRct(a, b);  // δp_f
                }
        }
    }
    return H;
}

// The invariant 4-DoF gauge. Crucially the CLONE directions are constants (no
// estimate): translation ρ = e_k; yaw φ = ĝ. Only the feature term carries the
// feature position (δp_f = [g]× p_f) — and the feature is the marginalized
// quantity, not a persistent state that drifts.
template <math::Scalar T>
[[nodiscard]] msckf::DynMat<T>
build_N_invariant(const std::vector<Pose<T>>& cl, const std::vector<Vec3<T>>& ff, const Vec3<T>& g) {
    const std::size_t m = cl.size(), K = ff.size();
    msckf::DynMat<T> N(6 * m + 3 * K, 4);
    for (std::size_t c = 0; c < m; ++c)
        for (std::size_t k = 0; k < 3; ++k)
            N(6 * c + 3 + k, k) = T{1};  // translation: clone ρ = e_k (constant)
    for (std::size_t f = 0; f < K; ++f)
        for (std::size_t k = 0; k < 3; ++k)
            N(6 * m + 3 * f + k, k) = T{1};  // translation: feature δp
    const Mat3<T> gx = math::lie::detail::hat(g);
    for (std::size_t c = 0; c < m; ++c)
        for (std::size_t k = 0; k < 3; ++k)
            N(6 * c + k, 3) = g[k];  // yaw: clone φ = ĝ (constant, estimate-independent)
    for (std::size_t f = 0; f < K; ++f) {
        const Vec3<T> dp = gx * ff[f];  // [g]× p_f
        for (std::size_t k = 0; k < 3; ++k)
            N(6 * m + 3 * f + k, 3) = dp[k];
    }
    return N;
}
}  // namespace obs_detail

template <math::Scalar T>
struct ObservabilityProbe {
    std::size_t gauge_dim = 4;           ///< analytic unobservable DoF (3 translation + 1 yaw)
    T trans_leak_consistent = T{0};      ///< ‖H·N_trans‖ at a consistent linearization (≈ 0)
    T yaw_leak_consistent = T{0};        ///< ‖H·N_yaw‖   at a consistent linearization (≈ 0)
    T trans_leak_inconsistent = T{0};    ///< translation stays ≈ 0 even at the perturbed estimate
    T yaw_leak_inconsistent = T{0};      ///< YAW leaks: spurious information along rotation-about-gravity
    std::vector<std::pair<T, T>> sweep;  ///< (perturbation σ, yaw leak) — the FEJ acceptance curve
};

/// Drive the null-space audit. `sigma` is the pose-estimate error the filter
/// linearizes at in the inconsistent case (rad / m).
template <math::Scalar T>
[[nodiscard]] ObservabilityProbe<T>
observability_probe(std::size_t m = 5, std::size_t k = 6, T sigma = T{2} / T{100}, std::uint64_t seed = 0x0B5E11ull) {
    using namespace obs_detail;
    const Vec3<T> g{{T{0}, T{0}, T{1}}};  // gravity axis: world up (config gravity is −z)
    const Scene<T> truth = make_scene<T>(m, k);
    const msckf::DynMat<T> Ntrue = build_N<T>(truth.clones, truth.feats, g);

    ObservabilityProbe<T> out;
    // Consistent linearization (FEJ-like): H and N at the same true poses.
    {
        const auto [tr, yaw] = leak<T>(build_H<T>(truth.clones, truth.feats), Ntrue);
        out.trans_leak_consistent = tr;
        out.yaw_leak_consistent = yaw;
    }

    std::mt19937_64 rng(seed);
    // Portable uniform draw in [−1, 1): mt19937_64's output IS reproducible across
    // standard-library implementations, whereas std::normal/uniform_distribution
    // is NOT — so the probe gates deterministically everywhere. Sign and growth
    // with σ are all the leak metric needs (it is not a statistical estimate).
    auto urand = [&]() -> T {
        const std::uint64_t bits = rng() >> 11;  // 53 high bits
        return T(static_cast<double>(bits) * (2.0 / 9007199254740992.0) - 1.0);
    };
    auto perturbed = [&](T s) {
        Scene<T> e = truth;
        for (auto& c : e.clones) {
            c.R = c.R * SO3<T>::exp(Vec3<T>{{s * urand(), s * urand(), s * urand()}});
            c.p = Vec3<T>{{c.p[0] + s * urand(), c.p[1] + s * urand(), c.p[2] + s * urand()}};
        }
        for (auto& f : e.feats)
            f = Vec3<T>{{f[0] + s * urand(), f[1] + s * urand(), f[2] + s * urand()}};
        return e;
    };

    // Inconsistent (standard EKF): H at the perturbed estimate, N at the true gauge.
    {
        const Scene<T> e = perturbed(sigma);
        const auto [tr, yaw] = leak<T>(build_H<T>(e.clones, e.feats), Ntrue);
        out.trans_leak_inconsistent = tr;
        out.yaw_leak_inconsistent = yaw;
    }
    // Yaw-leak-vs-perturbation curve — the acceptance target for any FEJ / OC-EKF fix.
    for (const T s : {T{0}, T{5} / T{1000}, T{1} / T{100}, T{2} / T{100}, T{4} / T{100}}) {
        const Scene<T> e = perturbed(s);
        const auto [tr, yaw] = leak<T>(build_H<T>(e.clones, e.feats), Ntrue);
        out.sweep.push_back({s, yaw});
    }
    return out;
}

// ── R-IEKF Phase-A gate (issue #348): the invariant yaw leak goes flat ─────────
//
// The decisive go/no-go for the R-IEKF epic (#347). Perturb the CLONE estimates
// (the drift that diverged the standard filter) while keeping features at truth,
// and compare the yaw leak ‖H·N‖_yaw of the body-frame EKF vs the right-invariant
// parameterization. The standard curve RISES with σ (the measured leak); the
// invariant curve must stay at ~machine-ε for EVERY σ — yaw observable by
// construction, because its gauge direction (φ = ĝ) is estimate-independent.
template <math::Scalar T>
struct InvariantObservabilityProbe {
    T std_yaw_consistent = T{0};             ///< standard yaw leak at the true poses (≈ 0)
    T inv_yaw_consistent = T{0};             ///< invariant yaw leak at the true poses (≈ 0)
    std::vector<std::pair<T, T>> std_sweep;  ///< (σ, standard yaw leak) — rises
    std::vector<std::pair<T, T>> inv_sweep;  ///< (σ, invariant yaw leak) — flat at ~ε
};

template <math::Scalar T>
[[nodiscard]] InvariantObservabilityProbe<T>
invariant_observability_probe(std::size_t m = 5, std::size_t k = 6, std::uint64_t seed = 0x0B5E11ull) {
    using namespace obs_detail;
    const Vec3<T> g{{T{0}, T{0}, T{1}}};
    const Scene<T> truth = make_scene<T>(m, k);
    const msckf::DynMat<T> N_std = build_N<T>(truth.clones, truth.feats, g);
    const msckf::DynMat<T> N_inv = build_N_invariant<T>(truth.clones, truth.feats, g);

    InvariantObservabilityProbe<T> out;
    out.std_yaw_consistent = leak<T>(build_H<T>(truth.clones, truth.feats), N_std).second;
    out.inv_yaw_consistent = leak<T>(build_H_invariant<T>(truth.clones, truth.feats), N_inv).second;

    std::mt19937_64 rng(seed);
    auto urand = [&]() -> T {
        const std::uint64_t bits = rng() >> 11;
        return T(static_cast<double>(bits) * (2.0 / 9007199254740992.0) - 1.0);
    };
    // Perturb the CLONES only (features at truth) — the per-clone linearization
    // drift that breaks the body-frame null space across the window.
    auto perturb_clones = [&](T s) {
        Scene<T> e = truth;
        for (auto& c : e.clones) {
            c.R = c.R * SO3<T>::exp(Vec3<T>{{s * urand(), s * urand(), s * urand()}});
            c.p = Vec3<T>{{c.p[0] + s * urand(), c.p[1] + s * urand(), c.p[2] + s * urand()}};
        }
        return e;
    };
    for (const T s : {T{0}, T{5} / T{1000}, T{1} / T{100}, T{2} / T{100}, T{4} / T{100}}) {
        const Scene<T> e = perturb_clones(s);
        out.std_sweep.push_back({s, leak<T>(build_H<T>(e.clones, e.feats), N_std).second});
        out.inv_sweep.push_back({s, leak<T>(build_H_invariant<T>(e.clones, e.feats), N_inv).second});
    }
    return out;
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_OBSERVABILITY_PROBE_HPP
