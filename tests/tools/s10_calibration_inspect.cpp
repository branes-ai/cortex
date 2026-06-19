// SPDX-License-Identifier: MIT
//
// Gate for the S10 online-calibration inspector (branes/tools/s10_calibration_inspect.hpp,
// issue #384). The real inspector tracks the in-state extrinsic converging over a
// live EuRoC run, which CI does not vendor, so here we drive the pure per-frame
// sampler on a constructed calibration state and pin the behaviour the convergence
// figure relies on: the rotation/translation error against a reference extrinsic is
// the geodesic / vector distance, the σ is read from the calibration covariance
// block, a state with no calibration block reads as empty, and the record
// serializes.

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/tools/s10_calibration_inspect.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

namespace bt = branes::tools;
namespace ms = branes::sdk::msckf;
namespace lie = branes::math::lie;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
using SO3 = lie::SO3<double>;
using Vec3 = lie::detail::Vec<double, 3>;
constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
}  // namespace

TEST_CASE("S10 inspector: samples extrinsic error and sigma against a reference", "[tools][s10_inspect]") {
    ms::State<double> s(0.1);
    ms::State<double>::CalibState cs;
    cs.R_imu_cam = SO3::exp(Vec3{{5.0 * kDeg, 0.0, 0.0}});  // 5° off the reference
    cs.p_imu_cam = Vec3{{0.01, 0.0, 0.0}};                  // 10 mm off the reference
    const double rot_sigma = 0.2, trans_sigma = 0.2;        // prior σ seeded into the calib block
    s.enable_calibration({cs}, rot_sigma, trans_sigma);

    const auto smp = bt::S10CalibrationInspector().sample(s, SO3{}, Vec3{{0.0, 0.0, 0.0}}, /*t=*/1.5);

    REQUIRE_THAT(smp.t, WithinAbs(1.5, 1e-12));
    REQUIRE_THAT(smp.rot_err_deg, WithinAbs(5.0, 1e-6));    // geodesic angle to the reference
    REQUIRE_THAT(smp.trans_err_mm, WithinRel(10.0, 1e-9));  // ‖p − ref‖ in mm
    // σ = √trace of the 3×3 block; the seed is rot_sigma²·I / trans_sigma²·I.
    REQUIRE_THAT(smp.rot_sigma_deg, WithinRel(std::sqrt(3.0) * rot_sigma * kRad2Deg, 1e-6));
    REQUIRE_THAT(smp.trans_sigma_mm, WithinRel(std::sqrt(3.0) * trans_sigma * 1000.0, 1e-6));
}

TEST_CASE("S10 inspector: a state with no calibration block reads empty", "[tools][s10_inspect]") {
    ms::State<double> s(0.1);  // online calibration NOT enabled
    REQUIRE(s.calib.empty());
    const auto smp = bt::S10CalibrationInspector().sample(s, SO3{}, Vec3{{0.0, 0.0, 0.0}}, 2.0);
    REQUIRE_THAT(smp.t, WithinAbs(2.0, 1e-12));
    REQUIRE(smp.rot_err_deg == 0.0);
    REQUIRE(smp.trans_err_mm == 0.0);
    REQUIRE(smp.rot_sigma_deg == 0.0);
}

TEST_CASE("S10 inspector: record serializes to the figure schema", "[tools][s10_inspect]") {
    ms::State<double> s(0.1);
    ms::State<double>::CalibState cs;
    cs.R_imu_cam = SO3::exp(Vec3{{0.0, 3.0 * kDeg, 0.0}});
    s.enable_calibration({cs}, 0.1, 0.1);
    const bt::S10CalibrationInspector insp;

    bt::S10CalibRecord rec;
    rec.has_calib = true;
    rec.ref_rot_deg_init = 3.0;
    for (int k = 0; k < 4; ++k)
        rec.curve.push_back(insp.sample(s, SO3{}, Vec3{{0.0, 0.0, 0.0}}, 0.1 * k));

    const auto j = bt::to_json(rec);
    REQUIRE(j.at("has_calib") == true);
    REQUIRE(j.at("curve").is_array());
    REQUIRE(j.at("curve").size() == 4);
    const auto& s0 = j.at("curve").at(0);
    REQUIRE(s0.contains("rot_err_deg"));
    REQUIRE(s0.contains("trans_err_mm"));
    REQUIRE(s0.contains("rot_sigma_deg"));
    REQUIRE(s0.contains("trans_sigma_mm"));
}
