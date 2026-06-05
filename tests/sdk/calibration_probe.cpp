// S10 calibration probe (docs/arch/vio-pipeline-canonical.md) — the leading
// #212 candidate: the filter treats camera↔IMU calibration as perfectly known.
//
// Locks two findings: (1) the analytic noise budget — a realistic ~1° extrinsic
// uncertainty induces a reprojection error larger than the filter's assumed
// pixel noise, an R-inflation of order the empirical R×4; (2) the end-to-end
// R-inflation sweep — the synthetic backend is strongly over-confident even with
// perfect calibration (pose NEES ≫ dof), and inflating the measurement noise R
// drives NEES back toward consistency (R is the lever).

#include <branes/sdk/eval/calibration_probe.hpp>

#include <catch2/catch_test_macros.hpp>

namespace {
namespace ev = branes::sdk::eval;
using T = double;
}  // namespace

TEST_CASE("S10 calibration noise budget: ~1deg extrinsic uncertainty ~ the empirical Rx4", "[sdk][s10][calibration]") {
    // 1° extrinsic, 10 mm translation, slow motion (small time-offset term).
    const auto b = ev::noise_budget<T>(/*ext_rot_deg=*/1.0,
                                       /*ext_trans_mm=*/10.0,
                                       /*time_offset_ms=*/0.0,
                                       /*px_per_ms=*/0.12,
                                       /*depth=*/5.0);
    // The calibration-induced reprojection error exceeds the assumed pixel noise…
    REQUIRE(b.total_calib_px > b.assumed_px);
    // …so the required measurement-noise inflation is of order the empirical R×4.
    REQUIRE(b.r_inflation_factor > 3.0);
    REQUIRE(b.r_inflation_factor < 6.0);

    // Monotone in the extrinsic uncertainty.
    const auto half = ev::noise_budget<T>(0.5, 10.0, 0.0, 0.12, 5.0);
    const auto two = ev::noise_budget<T>(2.0, 10.0, 0.0, 0.12, 5.0);
    REQUIRE(half.r_inflation_factor < b.r_inflation_factor);
    REQUIRE(two.r_inflation_factor > b.r_inflation_factor);
    // Translation sensitivity falls with depth (a near feature is more affected).
    REQUIRE(ev::noise_budget<T>(0.0, 10.0, 0.0, 0.0, 2.0).ext_trans_px >
            ev::noise_budget<T>(0.0, 10.0, 0.0, 0.0, 10.0).ext_trans_px);
}

TEST_CASE("S10 end-to-end: backend over-confident with perfect calibration; R is the lever",
          "[sdk][s10][calibration]") {
    const auto sw = ev::r_inflation_sweep<T>();
    REQUIRE(sw.curve.size() >= 5);
    // With PERFECT calibration and clean sensors the pose NEES is far above the
    // 6-DoF target — the structural #212 over-confidence, reproduced.
    REQUIRE(sw.curve.front().r_scale == 1.0);
    REQUIRE(sw.curve.front().nees > 15.0);
    // Inflating R monotonically reduces NEES (R is the consistency lever)…
    for (std::size_t i = 1; i < sw.curve.size(); ++i)
        REQUIRE(sw.curve[i].nees < sw.curve[i - 1].nees);
    // …and a finite R-scale restores NEES to the dof (the empirical deficit).
    REQUIRE(sw.r_scale_for_consistency > 1.0);
    // The 1° calibration budget is a real fraction of that deficit (the R×4 part).
    REQUIRE(sw.budget_r_inflation_1deg > 3.0);
    REQUIRE(sw.budget_r_inflation_1deg < 6.0);
}
