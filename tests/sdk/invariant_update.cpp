// R-IEKF invariant camera measurement update (issue #347, Phase B).
//
// The measurement complement of invariant_propagator.cpp. Locks three properties
// of the right-invariant feature update on a clone window:
//   (1) consistency — at truth with exact observations the correction is ~0;
//   (2) recovery — an OBSERVABLE clone error is corrected (residual collapses);
//   (3) the decisive one — the SHIPPED measurement Jacobian H = [H_x | H_f]
//       annihilates the 4-DoF gauge (global yaw + translation) at the actual clone
//       poses: H·N = 0. The update therefore injects no information along the
//       unobservable manifold — the structural fix for #212, with no FEJ.

#include <branes/sdk/msckf/covariance.hpp>
#include <branes/sdk/msckf/invariant_update.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {
namespace ms = branes::sdk::msckf;
namespace det = branes::math::lie::detail;
using T = double;
using Vec3 = det::Vec<T, 3>;
using Vec2 = det::Vec<T, 2>;
using SO3 = branes::math::lie::SO3<T>;

// A small synthetic scene: clones along a sideways sweep all viewing a cloud of
// points in front of the trajectory (so every feature is well-conditioned).
struct Scene {
    std::vector<ms::InvariantClone<T>> clones;
    std::vector<Vec3> feats;
};
Scene make_scene() {
    Scene s;
    for (int c = 0; c < 5; ++c) {
        ms::InvariantClone<T> cl;
        cl.R = SO3::exp(Vec3{{0.02 * c, -0.01 * c, 0.015 * c}});  // gentle attitude change
        cl.p = Vec3{{0.25 * c, 0.0, 0.0}};                        // slide along +x
        s.clones.push_back(cl);
    }
    for (int i = 0; i < 16; ++i) {
        const T u = static_cast<T>((i % 4) - 2) * T{0.4};
        const T v = static_cast<T>((i / 4) - 2) * T{0.4};
        s.feats.push_back(Vec3{{u, v, 3.0 + 0.1 * i}});  // ~3 m ahead, in front of every clone
    }
    return s;
}

// Exact normalized-image observation of `pf` from clone `c` (identity extrinsic).
Vec2 project(const ms::InvariantClone<T>& c, const Vec3& pf) {
    const Vec3 y = c.R.inverse().matrix() * Vec3{{pf[0] - c.p[0], pf[1] - c.p[1], pf[2] - c.p[2]}};
    return Vec2{{y[0] / y[2], y[1] / y[2]}};
}
std::vector<ms::InvariantObs<T>> track(const Scene& s, const Vec3& pf) {
    std::vector<ms::InvariantObs<T>> obs;
    for (std::size_t c = 0; c < s.clones.size(); ++c)
        obs.push_back({c, project(s.clones[c], pf)});
    return obs;
}

// Linear (DLT-style) triangulation of a feature from the current clones: solve
// Σ (I − d̂d̂ᵀ)(p_f − p_c) = 0, d = R_c·[xy;1] the world bearing.
Vec3 triangulate(const Scene& s, const std::vector<ms::InvariantObs<T>>& obs) {
    det::Mat<T, 3, 3> A{};
    Vec3 b{};
    for (const auto& o : obs) {
        const Vec3 d = s.clones[o.clone_index].R.matrix() * Vec3{{o.xy[0], o.xy[1], T{1}}};
        const T n2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        const Vec3& pc = s.clones[o.clone_index].p;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                const T P = (i == j ? T{1} : T{0}) - d[i] * d[j] / n2;
                A(i, j) += P;
                b[i] += P * pc[j];
            }
    }
    const T dt = A(0, 0) * (A(1, 1) * A(2, 2) - A(1, 2) * A(2, 1)) - A(0, 1) * (A(1, 0) * A(2, 2) - A(1, 2) * A(2, 0)) +
                 A(0, 2) * (A(1, 0) * A(2, 1) - A(1, 1) * A(2, 0));
    assert(std::abs(dt) > T{1e-12} && "triangulate: singular geometry (coplanar rays?)");
    auto cof = [&](int i, int j) {
        const int a1 = (j + 1) % 3, a2 = (j + 2) % 3, b1 = (i + 1) % 3, b2 = (i + 2) % 3;
        return (A(b1, a1) * A(b2, a2) - A(b1, a2) * A(b2, a1)) / dt;
    };
    Vec3 x{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            x[i] += cof(i, j) * b[j];
    return x;
}

// Gauge-INVARIANT inconsistency: for each observed track, re-triangulate the
// feature from the current clones and sum the reprojection residual. A global
// yaw/translation shift of the clones leaves this unchanged, so it measures only
// the OBSERVABLE error — what the filter is allowed to remove.
T inconsistency(const Scene& s) {
    const Scene truth = make_scene();
    T cost = T{0};
    for (const auto& pf0 : truth.feats) {
        const auto obs = track(truth, pf0);
        const Vec3 pf = triangulate(s, obs);
        for (const auto& o : obs) {
            const Vec2 rr = project(s.clones[o.clone_index], pf);
            cost += (rr[0] - o.xy[0]) * (rr[0] - o.xy[0]) + (rr[1] - o.xy[1]) * (rr[1] - o.xy[1]);
        }
    }
    return cost;
}
}  // namespace

TEST_CASE("invariant update: exact observations at truth produce no correction", "[sdk][riekf][update]") {
    const Scene s = make_scene();
    ms::FullCovariance<T> cov(0.5, 6 * s.clones.size());
    Scene work = s;
    for (const auto& pf : s.feats)
        ms::invariant_update<T>(work.clones, cov, track(s, pf), pf);
    // Residuals were exactly zero ⇒ the clones must not have moved at all.
    T moved = T{0};
    for (std::size_t c = 0; c < work.clones.size(); ++c) {
        moved += branes::math::lie::detail::norm((s.clones[c].R.inverse() * work.clones[c].R).log());
        moved += branes::math::lie::detail::norm(Vec3{{work.clones[c].p[0] - s.clones[c].p[0],
                                                       work.clones[c].p[1] - s.clones[c].p[1],
                                                       work.clones[c].p[2] - s.clones[c].p[2]}});
    }
    REQUIRE(moved < 1e-12);
    REQUIRE(ms::is_positive_semidefinite(cov.covariance()));
}

TEST_CASE("invariant update: an observable clone error is corrected", "[sdk][riekf][update]") {
    const Scene truth = make_scene();
    Scene work = truth;
    // Perturb ONE clone's pose: with the others anchoring the frame this is a purely
    // OBSERVABLE error (orthogonal to the global yaw/translation gauge).
    ms::retract_invariant<T>(work.clones[2], Vec3{{0.03, -0.02, 0.025}}, Vec3{{0.04, -0.03, 0.02}});
    const T pre = inconsistency(work);
    REQUIRE(pre > 1e-3);

    // Iterated relinearization: re-inflate the prior each sweep (Gauss-Newton on the
    // batch) — the legitimate way to reach the fixed point of a linearized update.
    for (int sweep = 0; sweep < 4; ++sweep) {
        ms::FullCovariance<T> cov(0.5, 6 * work.clones.size());
        for (const auto& pf : truth.feats)
            ms::invariant_update<T>(work.clones, cov, track(truth, pf), pf);
    }
    REQUIRE(inconsistency(work) < 1e-6);  // observable inconsistency driven out
}

TEST_CASE("invariant update: H annihilates the 4-DoF gauge at the actual clone poses",
          "[sdk][riekf][update][observability]") {
    const Scene s = make_scene();
    const std::size_t nc = s.clones.size();
    const std::size_t n = 6 * nc;

    // Vertical axis = gravity axis; the global-yaw gauge rotates every clone AND
    // the feature about it. Translation gauge shifts every clone AND the feature.
    const Vec3 a{{0.0, 0.0, 1.0}};

    auto leak = [&](const Vec3& pf) {
        const auto M = ms::build_invariant_measurement<T>(s.clones, track(s, pf), pf);
        REQUIRE(M.ok);
        // Gauge columns of the FULL Jacobian [H_x | H_f]: 3 translation + 1 yaw.
        // Per clone: translation ρ_c = e_k (φ_c = 0); yaw φ_c = a (ρ_c = 0).
        // Feature: translation δp_f = e_k; yaw δp_f = a × p_f.
        T worst = T{0};
        for (int g = 0; g < 4; ++g) {
            Vec3 fcol{};  // feature column of this gauge direction
            std::vector<T> xcol(n, T{0});
            if (g < 3) {
                for (std::size_t c = 0; c < nc; ++c)
                    xcol[6 * c + 3 + g] = T{1};  // ρ_c = e_g
                fcol[g] = T{1};
            } else {
                for (std::size_t c = 0; c < nc; ++c)
                    for (int k = 0; k < 3; ++k)
                        xcol[6 * c + k] = a[k];  // φ_c = a
                fcol = det::hat(a) * pf;         // a × p_f
            }
            for (std::size_t row = 0; row < M.rows2; ++row) {
                T hv = T{0};
                for (std::size_t j = 0; j < n; ++j)
                    hv += M.H_x[row * n + j] * xcol[j];
                for (int b = 0; b < 3; ++b)
                    hv += M.H_f[row * 3 + b] * fcol[b];
                worst = std::max(worst, std::abs(hv));
            }
        }
        return worst;
    };

    // Holds for every feature, at the real (non-identity) clone attitudes.
    REQUIRE(leak(s.feats[0]) < 1e-12);
    REQUIRE(leak(s.feats[7]) < 1e-12);
    REQUIRE(leak(s.feats[15]) < 1e-12);
}
