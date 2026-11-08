// SPDX-License-Identifier: MIT
//
// Gate for the R-IEKF EuRoC adapter (branes/tools/invariant_backend_adapter.hpp,
// #347/#348). The full end-to-end comparison runs in vio_pipeline over EuRoC
// (not vendored in CI); here we pin the adapter's contract bridges that the
// harness relies on, on synthetic input:
//
//   * since #608 it initializes through the shipped MsckfBackend: at the same
//     sample, with the same seed, and with the body-frame prior mapped into the
//     right-invariant error by body_to_invariant_jacobian (checked against the
//     exact error by finite differences);
//   * the legacy bootstrap (Options::legacy_init) still comes up from a static
//     IMU window of 150 samples, levelling roll/pitch with zero velocity;
//   * S8: at rest the frame adds no clone and the velocity is held at zero;
//     with legacy_zupt it is cloned;
//   * it satisfies the VioEstimator<T, Backend> duck-typed contract (process_imu /
//     process_camera / current_state) and maps the SE2(3) nav back to a NavState.
//
// ASCII-only TEST_CASE names (the Windows MSVC ctest job rejects non-ASCII).

#include <branes/math/cameras/pinhole_radtan.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/vio_backend.hpp>
#include <branes/tools/invariant_backend_adapter.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

namespace bt = branes::tools;
namespace bs = branes::sdk;
using Catch::Matchers::WithinAbs;

namespace {
using T = double;
using Adapter = bt::InvariantBackendAdapter<T>;
using SO3 = branes::math::lie::SO3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;

Adapter::Camera euroc_like_camera() {
    return Adapter::Camera(458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
}

Adapter make_adapter(Adapter::Options opt = {}) {
    // EuRoC-like intrinsics; identity extrinsic keeps the test about the bridge.
    return Adapter(euroc_like_camera(), SO3{}, Vec3{}, opt);
}

// A static start tilted by a few degrees, so the seeded attitude is not the
// identity and the covariance map has a rotation to carry.
bs::ImuMeasurement<T> tilted_sample(double t) {
    const SO3 R = SO3::exp(Vec3{{0.05, -0.03, 0.0}});
    const Vec3 f = R.inverse() * Vec3{{0.0, 0.0, 9.81}};
    bs::ImuMeasurement<T> m;
    m.timestamp_s = t;
    m.angular_velocity = {0.001, -0.002, 0.0015};  // a gyro bias the static init recovers
    m.linear_acceleration = {f[0], f[1], f[2]};
    return m;
}

std::vector<bs::FrontendObservation<T>> grid_obs() {
    std::vector<bs::FrontendObservation<T>> obs;
    for (int k = 0; k < 16; ++k) {
        bs::FrontendObservation<T> o;
        o.feature_id = static_cast<std::uint64_t>(k);
        o.camera_id = 0;
        o.u = 200.0 + 60.0 * (k % 4);
        o.v = 150.0 + 60.0 * (k / 4);
        obs.push_back(o);
    }
    return obs;
}

T max_abs_diff(const branes::sdk::msckf::DynMat<T>& a, const branes::sdk::msckf::DynMat<T>& b) {
    T m = 0;
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t j = 0; j < a.cols; ++j)
            m = std::max(m, std::abs(a(i, j) - b(i, j)));
    return m;
}

bs::ImuMeasurement<T> stationary_sample(double t) {
    bs::ImuMeasurement<T> m;
    m.timestamp_s = t;
    m.angular_velocity = {0.0, 0.0, 0.0};
    m.linear_acceleration = {0.0, 0.0, 9.81};  // at rest: specific force = +g (world up)
    return m;
}
}  // namespace

TEST_CASE("invariant adapter: legacy init bootstraps from a static IMU window", "[tools][invariant_adapter]") {
    Adapter a = make_adapter(Adapter::Options{false, true, false});
    bs::VioConfig cfg;
    a.initialize(cfg);
    REQUIRE_FALSE(a.initialized());

    // Pin the bootstrap boundary exactly: one sample short of the window
    // (kInitSamples = 150 in the adapter) it must still be down, and it must come
    // up as soon as the window fills. Asserting at the off-by-one catches both an
    // early bootstrap and a silently-lowered threshold.
    for (int i = 0; i < 149; ++i)
        a.process_imu(stationary_sample(0.005 * i));
    REQUIRE_FALSE(a.initialized());
    for (int i = 149; i < 200; ++i)
        a.process_imu(stationary_sample(0.005 * i));
    REQUIRE(a.initialized());

    const auto ns = a.current_state();
    // Zero-velocity seed.
    REQUIRE_THAT(ns.velocity_world[0], WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(ns.velocity_world[1], WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(ns.velocity_world[2], WithinAbs(0.0, 1e-9));
    // Gravity-aligned attitude: rotating the measured specific force (+z body) into
    // world should land on world up (+z), i.e. the seed levels roll/pitch.
    const Vec3 up_body{{0.0, 0.0, 1.0}};
    const Vec3 up_world = ns.T_world_imu.rotation() * up_body;
    REQUIRE_THAT(up_world[2], WithinAbs(1.0, 1e-6));
}

TEST_CASE("invariant adapter: satisfies the VioEstimator backend contract", "[tools][invariant_adapter]") {
    Adapter a = make_adapter();
    a.initialize(bs::VioConfig{});
    for (int i = 0; i < 200; ++i)
        a.process_imu(stationary_sample(0.005 * i));
    REQUIRE(a.initialized());

    // process_camera takes pixel FrontendObservations; with no real parallax the
    // update may decline, but the call path (unprojection → backend) must be safe.
    std::vector<bs::FrontendObservation<T>> obs;
    for (int k = 0; k < 4; ++k) {
        bs::FrontendObservation<T> o;
        o.feature_id = static_cast<std::uint64_t>(k);
        o.camera_id = 0;
        o.u = 300.0 + 40.0 * k;
        o.v = 240.0;
        obs.push_back(o);
    }
    const auto before_t = a.current_state().timestamp_s;
    a.process_camera(1.0, std::span<const bs::FrontendObservation<T>>{obs});

    // covariance() is a square PSD-sized matrix; and process_camera must have
    // advanced the state stamp (propagation to the frame time), not merely left
    // the positive bootstrap stamp in place.
    const auto P = a.covariance();
    REQUIRE(P.rows == P.cols);
    REQUIRE(P.rows >= 15);
    REQUIRE(a.current_state().timestamp_s > before_t);
}

TEST_CASE("invariant adapter: the body-to-invariant map matches the exact error", "[tools][invariant_adapter]") {
    namespace ms = branes::sdk::msckf;
    using Body = ms::State<T>;
    using Inv = ms::InvariantNavState<T>;
    const SO3 R = SO3::exp(Vec3{{0.4, -0.7, 2.0}});
    const Vec3 v{{1.2, -0.4, 0.3}}, p{{3.0, -1.0, 0.5}};
    const auto J = bt::body_to_invariant_jacobian<T>(R, v, p);
    // The exact right-invariant error of a body-frame perturbation δb:
    // R⁺ = R·Exp(δθ_b), p⁺ = p + δp, v⁺ = v + δv; φ = Log(R⁺Rᵀ), ν = v⁺ − Exp(φ)v,
    // ρ = p⁺ − Exp(φ)p. Its derivative at δb = 0, by central differences.
    const auto invariant_error = [&](const std::vector<T>& db) {
        const SO3 Rp = R * SO3::exp(Vec3{{db[Body::kTheta], db[Body::kTheta + 1], db[Body::kTheta + 2]}});
        const Vec3 vp = v + Vec3{{db[Body::kVel], db[Body::kVel + 1], db[Body::kVel + 2]}};
        const Vec3 pp = p + Vec3{{db[Body::kPos], db[Body::kPos + 1], db[Body::kPos + 2]}};
        const Vec3 phi = (Rp * R.inverse()).log();
        const SO3 E = SO3::exp(phi);
        const Vec3 nu = vp - E * v, rho = pp - E * p;
        std::vector<T> e(Inv::kDim, 0.0);
        for (std::size_t i = 0; i < 3; ++i) {
            e[Inv::kTheta + i] = phi[i];
            e[Inv::kVel + i] = nu[i];
            e[Inv::kPos + i] = rho[i];
            e[Inv::kBg + i] = db[Body::kBg + i];
            e[Inv::kBa + i] = db[Body::kBa + i];
        }
        return e;
    };
    const T eps = 1e-6;
    T worst = 0;
    for (std::size_t k = 0; k < Body::kImuDim; ++k) {
        std::vector<T> dp(Body::kImuDim, 0.0), dm(Body::kImuDim, 0.0);
        dp[k] = eps;
        dm[k] = -eps;
        const auto ep = invariant_error(dp), em = invariant_error(dm);
        for (std::size_t i = 0; i < Inv::kDim; ++i)
            worst = std::max(worst, std::abs((ep[i] - em[i]) / (2 * eps) - J(i, k)));
    }
    CHECK(worst < 1e-8);
}

TEST_CASE("invariant adapter: initializes through the shipped MSCKF with its seed", "[tools][invariant_adapter]") {
    namespace ms = branes::sdk::msckf;
    using Boot = bs::MsckfBackend<T>;
    Adapter a = make_adapter();
    Boot boot(std::vector<Boot::CameraCalibration>{
        Boot::CameraCalibration{euroc_like_camera(), ms::CameraExtrinsics<T>{SO3{}, Vec3{}}}});
    const bs::VioConfig cfg;
    a.initialize(cfg);
    boot.initialize(cfg);

    int boot_at = -1, adapter_at = -1;
    for (int i = 0; i < 400 && (boot_at < 0 || adapter_at < 0); ++i) {
        const auto m = tilted_sample(0.005 * i);
        if (boot_at < 0) {
            boot.process_imu(m);
            if (boot.initialized())
                boot_at = i;
        }
        if (adapter_at < 0) {
            a.process_imu(m);
            if (a.initialized())
                adapter_at = i;
        }
    }
    REQUIRE(boot_at >= 0);
    CHECK(adapter_at == boot_at);
    CHECK(a.init_diagnostics().method == bs::InitMethod::Static);

    // The same mean.
    const auto& s = boot.state();
    const auto& nav = a.impl().nav();
    CHECK(nav.timestamp == s.timestamp);
    const auto Ra = nav.X.rotation().matrix(), Rb = s.R.matrix();
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            CHECK(Ra(i, j) == Rb(i, j));
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK(nav.X.velocity()[i] == s.v[i]);
        CHECK(nav.X.position()[i] == s.p[i]);
        CHECK(nav.bg[i] == s.bg[i]);
        CHECK(nav.ba[i] == s.ba[i]);
    }
    // The prior, mapped: P_inv = J P_body Jᵀ.
    const auto P = s.covariance();
    ms::DynMat<T> Pn(15, 15);
    for (std::size_t i = 0; i < 15; ++i)
        for (std::size_t j = 0; j < 15; ++j)
            Pn(i, j) = P(i, j);
    const auto J = bt::body_to_invariant_jacobian<T>(s.R, s.v, s.p);
    const auto expected = ms::mul(ms::mul(J, Pn), ms::transpose(J));
    CHECK(max_abs_diff(a.covariance(), expected) < 1e-15);
}

TEST_CASE("invariant adapter: S8 holds a frame at rest out of the window", "[tools][invariant_adapter]") {
    const auto run = [](Adapter::Options opt) {
        Adapter a = make_adapter(opt);
        a.initialize(bs::VioConfig{});
        const auto obs = grid_obs();
        int i = 0;
        for (int f = 0; f < 40; ++f) {
            const double tf = 0.5 + 0.05 * f;
            for (; 0.005 * i <= tf; ++i)
                a.process_imu(tilted_sample(0.005 * i));
            a.process_camera(tf, std::span<const bs::FrontendObservation<T>>{obs});
        }
        REQUIRE(a.initialized());
        return a;
    };
    const Adapter on = run(Adapter::Options{});
    CHECK(on.zero_velocity_frames() > 20);
    CHECK(on.num_clones() < 6);  // only the frames before the detector's 0.25 s windows filled
    const auto v = on.current_state().velocity_world;
    CHECK(std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) < 1e-3);

    const Adapter off = run(Adapter::Options{false, false, true});
    CHECK(off.zero_velocity_frames() == 0);
    CHECK(off.num_clones() > on.num_clones());
}
