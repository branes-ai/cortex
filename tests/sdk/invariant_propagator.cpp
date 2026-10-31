// R-IEKF invariant IMU propagation (issue #347, Phase B).
//
// Locks two properties of the right-invariant propagation:
//   (1) the SE₂(3) mean strapdown is the SAME physics as the shipped body-frame
//       propagator (identical R/v/p trajectory);
//   (2) the right-invariant error-transition Φ PRESERVES the 4-DoF unobservable
//       gauge (global translation + gravity-yaw) — Φ·N = N — and does so at ANY
//       rotation state, because the nav block depends only on (g, dt). This is the
//       propagation analogue of the flat measurement-yaw leak (Phase A): the
//       filter cannot fabricate information along the gauge while propagating;
//   (3) the predicted covariance matches a Monte-Carlo of the right-invariant
//       error far from the origin, where the gyro noise reaches position through
//       the [p̂]× lever arm (#537).

#include <branes/sdk/msckf/invariant_propagator.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <random>

namespace {
namespace ms = branes::sdk::msckf;
using T = double;
using Vec3 = ms::InvariantNavState<T>::Vec3;
using SO3 = branes::math::lie::SO3<T>;
using SE23 = ms::InvariantNavState<T>::SE23;

T vdiff(const Vec3& a, const Vec3& b) {
    return branes::math::lie::detail::norm(Vec3{{a[0] - b[0], a[1] - b[1], a[2] - b[2]}});
}
}  // namespace

TEST_CASE("invariant propagation: the SE2(3) mean matches the body-frame strapdown", "[sdk][riekf][propagation]") {
    const ms::ImuNoise<T> noise{2e-3, 1.6e-4, 3e-3, 1.9e-5};
    const Vec3 gravity{{T{0}, T{0}, T{-981} / T{100}}};

    // Same initial nav state and biases for both filters.
    const SO3 R0 = SO3::exp(Vec3{{0.1, -0.05, 0.2}});
    const Vec3 v0{{0.3, -0.1, 0.05}}, p0{{0.5, -0.3, 0.2}}, bg{{0.01, -0.02, 0.0}}, ba{{0.0, 0.01, -0.01}};

    ms::State<T> body(0.1);
    body.R = R0;
    body.v = v0;
    body.p = p0;
    body.bg = bg;
    body.ba = ba;
    ms::Propagator<T> bprop(noise, gravity);

    ms::InvariantNavState<T> inv;
    inv.X = SE23(R0, v0, p0);
    inv.bg = bg;
    inv.ba = ba;
    ms::FullCovariance<T> P(0.1, ms::InvariantNavState<T>::kDim);
    ms::InvariantPropagator<T> iprop(noise, gravity);

    for (int k = 0; k < 50; ++k) {
        const Vec3 gyro{{0.02, 0.01, -0.01}}, accel{{0.1, -0.05, 9.8}};
        bprop.propagate(body, gyro, accel, 0.01);
        iprop.propagate(inv, P, gyro, accel, 0.01);
    }
    // Identical physics ⇒ identical mean trajectory.
    REQUIRE(branes::math::lie::detail::norm((body.R.inverse() * inv.X.rotation()).log()) < 1e-12);
    REQUIRE(vdiff(inv.X.velocity(), body.v) < 1e-12);
    REQUIRE(vdiff(inv.X.position(), body.p) < 1e-12);
    REQUIRE(ms::is_positive_semidefinite(P.covariance()));
}

TEST_CASE("invariant propagation: Phi preserves the gauge null space at any state", "[sdk][riekf][propagation]") {
    using St = ms::InvariantNavState<T>;
    const ms::InvariantPropagator<T> prop;  // default gravity (0,0,-9.81)
    const T dt = T{1} / T{200};

    // The 4-DoF gauge in the right-invariant error [δθ; δv; δp; δbg; δba]:
    // translation = δp e_k (constant), gravity-yaw = δθ about the vertical axis.
    ms::DynMat<T> N(St::kDim, 4);
    for (std::size_t k = 0; k < 3; ++k)
        N(St::kPos + k, k) = T{1};  // global translation
    N(St::kTheta + 2, 3) = T{1};    // yaw: δθ = (0,0,1), the gravity axis

    // Nonzero v̂, p̂ so the gyro-bias lever-arm terms (−[v̂]×R̂, −[p̂]×R̂) are live; the
    // gauge must still be preserved (those terms multiply δbg, which is zero on N).
    const Vec3 vv{{0.3, -0.1, 0.05}}, pp{{0.5, -0.3, 0.2}};
    auto gauge_leak = [&](const SO3& R) {
        const ms::DynMat<T> PhiN = ms::mul(prop.phi(R, vv, pp, dt), N);
        T s = T{0};
        for (std::size_t i = 0; i < PhiN.rows; ++i)
            for (std::size_t j = 0; j < 4; ++j) {
                const T d = PhiN(i, j) - N(i, j);  // Φ·N should equal N
                s += d * d;
            }
        return std::sqrt(s);
    };

    // Φ preserves the gauge — and does so identically at two very different
    // attitudes (the nav block is state-independent; no FEJ needed).
    REQUIRE(gauge_leak(SO3{}) < 1e-12);
    REQUIRE(gauge_leak(SO3::exp(Vec3{{0.8, -0.6, 1.2}})) < 1e-12);
    REQUIRE(gauge_leak(SO3::exp(Vec3{{-1.5, 0.9, -0.3}})) < 1e-12);
}

TEST_CASE("invariant propagation: the predicted covariance matches a Monte-Carlo far from the origin",
          "[sdk][riekf][propagation]") {
    using St = ms::InvariantNavState<T>;
    // Strong gyro noise and a pose 20 m out, moving at 5 m/s: the right-invariant
    // error then takes the gyro noise into position as ≈ |p̂|²σ_g²·t, which a
    // diagonal Q (gyro noise in δθ only) under-predicts by more than 10×.
    const ms::ImuNoise<T> noise{1e-2, 1e-3, 0.0, 0.0};
    const ms::InvariantPropagator<T> prop(noise);
    const T dt = T{1} / T{100};
    const int steps = 100;
    const Vec3 gyro{{0.05, -0.02, 0.1}}, accel{{0.2, 0.1, 9.81}};
    const SE23 X0(SO3::exp(Vec3{{0.1, -0.2, 0.7}}), Vec3{{5.0, 0.0, 0.0}}, Vec3{{20.0, -5.0, 2.0}});

    // Predicted: P from zero through Φ P Φᵀ + L Lᵀ, along the noise-free mean.
    St est;
    est.X = X0;
    ms::FullCovariance<T> P(0.0, St::kDim);
    for (int k = 0; k < steps; ++k)
        prop.propagate(est, P, gyro, accel, dt);

    // Empirical: the truth integrates the noisy rates; its right-invariant error
    // against the mean is ξ = (log(R R̂ᵀ), v − ΔR v̂, p − ΔR p̂).
    std::mt19937 rng(0x537u);
    std::normal_distribution<T> nd(0, 1);
    const T sg = noise.gyro / std::sqrt(dt), sa = noise.accel / std::sqrt(dt);
    const int runs = 4000;
    ms::DynMat<T> C(9, 9);
    for (int n = 0; n < runs; ++n) {
        St tru;
        tru.X = X0;
        for (int k = 0; k < steps; ++k) {
            const Vec3 ng{{nd(rng), nd(rng), nd(rng)}};
            const Vec3 na{{nd(rng), nd(rng), nd(rng)}};
            prop.propagate_mean(tru, gyro + ng * sg, accel + na * sa, dt);
        }
        const SO3 dR = tru.X.rotation() * est.X.rotation().inverse();
        const Vec3 th = dR.log();
        const Vec3 nu = tru.X.velocity() - dR * est.X.velocity();
        const Vec3 rho = tru.X.position() - dR * est.X.position();
        const std::array<T, 9> e{{th[0], th[1], th[2], nu[0], nu[1], nu[2], rho[0], rho[1], rho[2]}};
        for (std::size_t i = 0; i < 9; ++i)
            for (std::size_t j = 0; j < 9; ++j)
                C(i, j) += e[i] * e[j] / T{runs};
    }

    // Each 3×3 block's trace agrees within the 4000-run sampling error (≈ 3%).
    const ms::DynMat<T> Pp = P.covariance();
    for (const std::size_t off : {St::kTheta, St::kVel, St::kPos}) {
        T pred = 0, emp = 0;
        for (std::size_t i = 0; i < 3; ++i) {
            pred += Pp(off + i, off + i);
            emp += C(off + i, off + i);
        }
        INFO("block " << off << ": predicted trace " << pred << ", empirical " << emp);
        CHECK(emp / pred > 0.85);
        CHECK(emp / pred < 1.15);
    }
}
