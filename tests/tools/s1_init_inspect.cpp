// SPDX-License-Identifier: MIT
//
// Gate for the S1 initialization inspector (branes/tools/s1_init_inspect.hpp,
// issue #376). The real inspector captures a live EuRoC bootstrap, which CI does
// not vendor, so here we drive the pure record builder on a constructed seeded
// state + a ground-truth nav sample and pin the behaviour the filter-internal
// figures rely on: the gravity-direction (roll/pitch) error is recovered
// yaw-invariantly, velocity/bias errors are the vector norms, the per-block
// initial σ is read from √diag(P), the isotropic seed is flagged, the dynamic
// scale error is computed against the metric truth, and the record serializes.

#include <branes/sdk/eval/nav_consistency.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/tools/s1_init_inspect.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

namespace bt = branes::tools;
namespace ms = branes::sdk::msckf;
namespace ev = branes::sdk::eval;
namespace lie = branes::math::lie;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
using SO3 = lie::SO3<double>;
using Vec3 = lie::detail::Vec<double, 3>;
constexpr double kDeg = 3.14159265358979323846 / 180.0;
}  // namespace

TEST_CASE("S1 inspector: static init - gravity-dir error, biases, isotropic seed", "[tools][s1_inspect]") {
    ms::State<double> s(0.1);                      // isotropic 0.1·I over the 15 IMU states
    s.R = SO3::exp(Vec3{{8.0 * kDeg, 0.0, 0.0}});  // 8° roll tilt vs the GT level frame
    s.v = Vec3{{0.0, 0.0, 0.0}};                   // static init seeds zero velocity
    s.bg = Vec3{{0.01, -0.005, 0.008}};
    s.ba = Vec3{{0.02, 0.0, 0.0}};

    branes::sdk::InitDiagnostics<double> diag;
    diag.method = branes::sdk::InitMethod::Static;
    diag.t_s = 1.37;
    diag.gravity_residual = 0.002;

    ev::NavSample<double> gt;  // GT: level, at rest, no bias
    gt.R = SO3{};
    gt.v = Vec3{{0.0, 0.0, 0.0}};
    gt.bg = Vec3{{0.0, 0.0, 0.0}};
    gt.ba = Vec3{{0.0, 0.0, 0.0}};

    const auto r = bt::S1InitInspector().build(diag, s, &gt);

    REQUIRE(r.method == "static");
    REQUIRE_THAT(r.t_s, WithinAbs(1.37, 1e-12));
    REQUIRE(r.have_gt);
    REQUIRE_THAT(r.gravity_dir_error_deg, WithinAbs(8.0, 1e-6));  // the 8° tilt, yaw-invariant
    REQUIRE_THAT(r.roll_deg, WithinAbs(8.0, 1e-6));
    REQUIRE_THAT(r.velocity_error_ms, WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(r.gyro_bias_error, WithinRel(std::sqrt(0.01 * 0.01 + 0.005 * 0.005 + 0.008 * 0.008), 1e-9));
    REQUIRE_THAT(r.accel_bias_error, WithinRel(0.02, 1e-9));
    REQUIRE(r.scale == 1.0);            // static ⇒ scale is trivially 1
    REQUIRE(r.scale_error_pct == 0.0);  // not a dynamic init

    // Per-block initial σ from √diag(P): θ in deg, the rest native.
    REQUIRE_THAT(r.sigma_theta_deg, WithinRel(0.1 * 180.0 / 3.14159265358979323846, 1e-6));
    REQUIRE_THAT(r.sigma_pos_m, WithinRel(0.1, 1e-9));
    REQUIRE_THAT(r.sigma_vel_ms, WithinRel(0.1, 1e-9));
    REQUIRE(r.isotropic_seed);  // σ·I seed — the S1.3 contract observation
    REQUIRE(r.dim == ms::State<double>::kImuDim);
    REQUIRE(r.cov.size() == r.dim * r.dim);
}

TEST_CASE("S1 inspector: dynamic init - scale error vs metric truth", "[tools][s1_inspect]") {
    ms::State<double> s(0.1);
    s.R = SO3{};
    s.v = Vec3{{1.2, -0.3, 0.05}};  // dynamic init recovers a velocity
    s.bg = Vec3{{0.0, 0.0, 0.0}};
    s.ba = Vec3{{0.0, 0.0, 0.0}};

    branes::sdk::InitDiagnostics<double> diag;
    diag.method = branes::sdk::InitMethod::Dynamic;
    diag.t_s = 2.5;
    diag.dyn_scale = 1.08;  // 8% off the metric truth
    diag.dyn_seed_speed = 1.24;
    diag.dyn_keyframes = 6;

    ev::NavSample<double> gt;
    gt.R = SO3{};
    gt.v = Vec3{{1.0, 0.0, 0.0}};

    const auto r = bt::S1InitInspector().build(diag, s, &gt);
    REQUIRE(r.method == "dynamic");
    REQUIRE_THAT(r.scale, WithinRel(1.08, 1e-9));
    REQUIRE_THAT(r.scale_error_pct, WithinAbs(8.0, 1e-6));  // |1.08 − 1| · 100, EuRoC is metric
    REQUIRE(r.dyn_keyframes == 6);
    REQUIRE_THAT(r.dyn_seed_speed, WithinRel(1.24, 1e-9));
    REQUIRE(r.velocity_error_ms > 0.0);
}

TEST_CASE("S1 inspector: no ground truth - error fields stay zero; record serializes", "[tools][s1_inspect]") {
    ms::State<double> s(0.1);
    branes::sdk::InitDiagnostics<double> diag;
    diag.method = branes::sdk::InitMethod::GravityAlign;
    diag.t_s = 0.9;

    const auto r = bt::S1InitInspector().build(diag, s, nullptr);
    REQUIRE(r.method == "gravity_align");
    REQUIRE_FALSE(r.have_gt);
    REQUIRE(r.gravity_dir_error_deg == 0.0);
    REQUIRE(r.velocity_error_ms == 0.0);

    const auto j = bt::to_json(r);
    REQUIRE(j.at("method") == "gravity_align");
    REQUIRE(j.at("attitude_deg").contains("roll"));
    REQUIRE(j.at("sigma").contains("theta_deg"));
    REQUIRE(j.at("cov").is_array());
    REQUIRE(j.at("cov").size() == r.dim * r.dim);
    REQUIRE(j.at("isotropic_seed") == true);
    REQUIRE(j.contains("gravity_dir_error_deg"));
}
