// S10 calibration probe (docs/pipeline/canonical-pipeline.md) — the leading
// #212 candidate: the filter treats camera↔IMU calibration as perfectly known.
//
// Locks two findings: (1) the analytic noise budget — a realistic ~1° extrinsic
// uncertainty induces a reprojection error larger than the filter's assumed
// pixel noise, an R-inflation of order the empirical R×4; (2) the end-to-end
// R-inflation sweep — with perfect calibration and clean sensors the synthetic
// backend is NOT over-confident (pose NEES < dof), so inflating R only adds
// pessimism. Before #537 the world's velocity step made it look over-confident
// (NEES ≈ 43, "reproducing" EuRoC MH_05); that reproduction was the defect.

#include <branes/sdk/eval/calibration_probe.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

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

TEST_CASE("S10 end-to-end: with perfect calibration and exact sensors the world is self-consistent",
          "[sdk][s10][calibration]") {
    const auto sw = ev::r_inflation_sweep<T>();
    REQUIRE(sw.curve.size() >= 5);
    // With PERFECT calibration and EXACT sensors the pose NEES is below the
    // 6-DoF target (0.04 measured) and the pose tracks to centimetres. The
    // assumed R (4.6 px) is pessimistic against zero noise, so this checks the
    // world and the plumbing, not consistency under noise: that is the
    // matched-noise control below (#551).
    REQUIRE(sw.curve.front().r_scale == 1.0);
    REQUIRE(sw.curve.front().nees < 6.0);
    REQUIRE(sw.curve.front().ate < 0.05);
    // Inflating R only makes the filter less confident…
    for (std::size_t i = 1; i < sw.curve.size(); ++i)
        REQUIRE(sw.curve[i].nees <= sw.curve[i - 1].nees);
    // …and no R-scale is needed to reach consistency.
    REQUIRE(sw.r_scale_for_consistency == 0.0);
    // The 1° calibration budget is a real fraction of that deficit (the R×4 part).
    REQUIRE(sw.budget_r_inflation_1deg > 3.0);
    REQUIRE(sw.budget_r_inflation_1deg < 6.0);
}

// #551: with camera and IMU noise matched to what the filter assumes, the
// body-frame MSCKF is over-confident, and the cause is the yaw observability
// leak (#212): yaw is unobservable, yet its σ collapses from the prior while the
// error it should cover grows. This pins the defect as a characterization; a
// fix (or the R-IEKF taking over as the reference, #347) flips both checks.
TEST_CASE("S10 matched-noise control: the body-frame MSCKF leaks information into yaw", "[sdk][s10][calibration]") {
    std::vector<T> nees;
    for (const std::uint32_t seed : {1u, 2u, 3u}) {
        const auto r = ev::matched_noise_control<T>(seed);
        INFO("seed " << seed << ": pose NEES " << r.nees << " (rp " << r.nees_rp << ", yaw " << r.nees_yaw << ", pos "
                     << r.nees_pos << "), yaw sigma " << r.yaw_sigma_first << " -> " << r.yaw_sigma_last << " rad, ATE "
                     << r.ate << " m");
        REQUIRE(r.frames > 100);
        CHECK(r.ate < 0.5);  // it still tracks: the defect is the covariance
        // The leak signature: an unobservable direction gained information.
        CHECK(r.yaw_sigma_last < r.yaw_sigma_first / 3);
        // Position is gauge too, but its σ stays at the prior and is honest.
        CHECK(r.nees_pos < 6.0);
        nees.push_back(r.nees);
    }
    std::sort(nees.begin(), nees.end());
    CHECK(nees[nees.size() / 2] > 12.0);  // median 6-DoF pose NEES; consistent ≈ 6
}
