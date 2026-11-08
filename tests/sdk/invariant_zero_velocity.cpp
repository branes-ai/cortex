// SPDX-License-Identifier: MIT
//
// The R-IEKF's real-data pieces added for EuRoC (#608), on MsckfInvariantBackend:
//
//   * S8 zero-velocity update in the right-invariant error: its posterior equals
//     the Kalman update with the Jacobian of v = Exp(δθ)·v̂ + ν taken by finite
//     differences of the retraction, so H_θ = −[v̂]× and H_ν = I are checked
//     against the filter's own box-plus, not against a restatement of them;
//   * the S8 gyro-bias update (additive bias);
//   * seed_covariance (P = G·Gᵀ, only before any clone);
//   * the update's NIS telemetry (valid whether or not the gate accepts);
//   * the gate rule's validation (the chi-square quantile needs alpha in (0,1)).
//
// ASCII-only TEST_CASE names (the Windows MSVC ctest job rejects non-ASCII).

#include <branes/math/lie/detail.hpp>
#include <branes/math/lie/se23.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/msckf_invariant_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <random>
#include <stdexcept>
#include <vector>

namespace {
namespace ms = branes::sdk::msckf;
namespace lie = branes::math::lie;
using T = double;
using Backend = ms::MsckfInvariantBackend<T>;
using Nav = Backend::Nav;
using Vec3 = lie::detail::Vec<T, 3>;
using SO3 = lie::SO3<T>;
using SE23 = lie::SE23<T>;

// A dense, well-conditioned G so the seeded prior couples attitude, velocity and
// the biases (an isotropic prior would leave the attitude columns of H untested).
ms::DynMat<T> random_factor(unsigned seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<T> n(0.0, 1.0);
    ms::DynMat<T> G(Nav::kDim, Nav::kDim);
    for (std::size_t i = 0; i < Nav::kDim; ++i) {
        for (std::size_t j = 0; j < Nav::kDim; ++j)
            G(i, j) = 0.02 * n(rng);
        G(i, i) += 0.1;
    }
    return G;
}

// The world velocity after the filter's own nav retraction by δx (15-vector):
// v = Exp(δθ)·v̂ + ν (MsckfInvariantBackend::retract).
Vec3 velocity_after(const SE23& X, const std::vector<T>& dx) {
    const SO3 dR = SO3::exp(Vec3{{dx[Nav::kTheta], dx[Nav::kTheta + 1], dx[Nav::kTheta + 2]}});
    return dR * X.velocity() + Vec3{{dx[Nav::kVel], dx[Nav::kVel + 1], dx[Nav::kVel + 2]}};
}

// P⁺ = P − P Hᵀ (H P Hᵀ + σ² I)⁻¹ H P.
ms::DynMat<T> kalman_posterior(const ms::DynMat<T>& P, const ms::DynMat<T>& H, T sigma) {
    ms::DynMat<T> S = ms::mul(ms::mul(H, P), ms::transpose(H));
    for (std::size_t i = 0; i < S.rows; ++i)
        S(i, i) += sigma * sigma;
    ms::DynMat<T> L;
    REQUIRE(ms::cholesky(S, L));
    const ms::DynMat<T> HP = ms::mul(H, P);
    const ms::DynMat<T> X = ms::cholesky_solve(L, HP);  // S⁻¹ H P
    ms::DynMat<T> Pn = P;
    const ms::DynMat<T> D = ms::mul(ms::transpose(HP), X);
    for (std::size_t i = 0; i < Pn.rows; ++i)
        for (std::size_t j = 0; j < Pn.cols; ++j)
            Pn(i, j) -= D(i, j);
    return Pn;
}

T max_abs_diff(const ms::DynMat<T>& a, const ms::DynMat<T>& b) {
    T m = 0;
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t j = 0; j < a.cols; ++j)
            m = std::max(m, std::abs(a(i, j) - b(i, j)));
    return m;
}
}  // namespace

TEST_CASE("invariant S8: the zero-velocity posterior matches the retraction's Jacobian",
          "[sdk][msckf][invariant][s8]") {
    Backend be;
    const SO3 R = SO3::exp(Vec3{{0.3, -0.2, 1.1}});
    const SE23 X(R, Vec3{{0.8, -0.5, 0.3}}, Vec3{{2.0, 1.0, -0.5}});
    be.set_nav(X, Vec3{}, Vec3{}, 0.0);
    be.seed_covariance(random_factor(7));
    const ms::DynMat<T> P = be.covariance();

    // H by central differences of v(δx) at δx = 0.
    ms::DynMat<T> H(3, Nav::kDim);
    const T eps = 1e-6;
    for (std::size_t k = 0; k < Nav::kDim; ++k) {
        std::vector<T> dp(Nav::kDim, 0.0), dm(Nav::kDim, 0.0);
        dp[k] = eps;
        dm[k] = -eps;
        const Vec3 vp = velocity_after(X, dp), vm = velocity_after(X, dm);
        for (std::size_t i = 0; i < 3; ++i)
            H(i, k) = (vp[i] - vm[i]) / (2 * eps);
    }
    const T sigma = 0.01;
    const ms::DynMat<T> expected = kalman_posterior(P, H, sigma);

    REQUIRE(be.zero_velocity_update(sigma));
    CHECK(max_abs_diff(be.covariance(), expected) < 1e-12);
    // The mean moves toward v = 0: the prior's velocity σ (~0.1 m/s) is much
    // larger than the pseudo-measurement's, so most of |v̂| = 1 m/s is removed.
    const Vec3 v = be.nav().X.velocity();
    CHECK(lie::detail::norm(v) < 0.1 * lie::detail::norm(X.velocity()));
}

TEST_CASE("invariant S8: the gyro-bias update is the additive Kalman update", "[sdk][msckf][invariant][s8]") {
    Backend be;
    be.set_nav(SE23(SO3{}, Vec3{{0.2, 0.0, 0.0}}, Vec3{}), Vec3{{0.01, -0.02, 0.005}}, Vec3{}, 0.0);
    be.seed_covariance(random_factor(11));
    const ms::DynMat<T> P = be.covariance();
    ms::DynMat<T> H(3, Nav::kDim);
    for (std::size_t i = 0; i < 3; ++i)
        H(i, Nav::kBg + i) = 1.0;
    const T sigma = 0.001;
    const Vec3 rate{{0.003, -0.004, 0.002}};
    const ms::DynMat<T> expected = kalman_posterior(P, H, sigma);

    REQUIRE(be.gyro_bias_update(rate, sigma));
    CHECK(max_abs_diff(be.covariance(), expected) < 1e-12);
    for (std::size_t i = 0; i < 3; ++i)
        CHECK(std::abs(be.nav().bg[i] - rate[i]) < 1e-3);
}

TEST_CASE("invariant backend: seed_covariance sets G G^T before any clone", "[sdk][msckf][invariant]") {
    Backend be;
    const ms::DynMat<T> G = random_factor(3);
    be.seed_covariance(G);
    CHECK(max_abs_diff(be.covariance(), ms::mul(G, ms::transpose(G))) < 1e-15);
    CHECK_THROWS_AS(be.seed_covariance(ms::DynMat<T>(3, 3)), std::invalid_argument);
    be.augment_clone();
    CHECK_THROWS_AS(be.seed_covariance(G), std::logic_error);
}

TEST_CASE("invariant backend: every update reports its NIS, gated or not", "[sdk][msckf][invariant]") {
    // Two clones 1 m apart looking down +z at a landmark 4 m ahead (identity
    // extrinsic): an exact track has NIS ~ 0 with dof 2m - 3 = 1.
    Backend::Config c;
    c.gate_rule = ms::GateRule::chi2_quantile;
    Backend be(c);
    be.set_nav(SE23(SO3{}, Vec3{}, Vec3{}), Vec3{}, Vec3{}, 0.0);
    be.augment_clone();
    be.set_nav(SE23(SO3{}, Vec3{}, Vec3{{1.0, 0.0, 0.0}}), Vec3{}, Vec3{}, 0.1);
    be.augment_clone();
    const Vec3 L{{0.3, -0.2, 4.0}};
    // `dy` offsets the observation across the epipolar line (the baseline is
    // along x, so an x offset would only move the triangulated depth).
    const auto obs_at = [&](std::size_t clone, double px, T dy) {
        ms::InvariantObs<T> o;
        o.clone_index = clone;
        o.camera_index = 0;
        o.xy = lie::detail::Vec<T, 2>{{(L[0] - px) / L[2], L[1] / L[2] + dy}};
        return o;
    };

    REQUIRE(be.update(ms::InvariantTrack<T>{obs_at(0, 0.0, 0.0), obs_at(1, 1.0, 0.0)}));
    CHECK(be.last_nis().valid);
    CHECK(be.last_nis().dof == 1);
    CHECK(be.last_nis().value < 1e-6);

    // A gross outlier (0.2 normalized, ~90 px, across the epipolar line):
    // rejected by the gate, NIS still reported.
    CHECK_FALSE(be.update(ms::InvariantTrack<T>{obs_at(0, 0.0, 0.0), obs_at(1, 1.0, 0.2)}));
    CHECK(be.last_nis().valid);
    CHECK(be.last_nis().value > 3.841);  // above chi-square(1) at 0.95: why the gate rejected it

    // A one-observation track never reaches the gate: no NIS.
    CHECK_FALSE(be.update(ms::InvariantTrack<T>{obs_at(0, 0.0, 0.0)}));
    CHECK_FALSE(be.last_nis().valid);
}

TEST_CASE("invariant backend: the chi-square gate needs alpha in (0,1)", "[sdk][msckf][invariant]") {
    Backend::Config c;
    c.gate_rule = ms::GateRule::chi2_quantile;
    c.gate_alpha = 0.05;
    CHECK_NOTHROW(Backend(c));
    c.gate_alpha = 0.0;
    CHECK_THROWS_AS(Backend(c), std::invalid_argument);
    c.gate_alpha = 1.0;
    CHECK_THROWS_AS(Backend(c), std::invalid_argument);
    c.enable_gating = false;  // no gate, nothing to validate
    CHECK_NOTHROW(Backend(c));
    Backend::Config d;
    d.chi2_per_dof = 0.0;
    CHECK_THROWS_AS(Backend(d), std::invalid_argument);
}
