// S5 (feature triangulation) probe (docs/arch/vio-pipeline-canonical.md).
//
// Locks the measured S5 finding: the shipped triangulator (CameraUpdater::
// triangulate) recovers correct depth from clean observations across the whole
// parallax range, but its ONLY guard is the hard Cholesky breakdown at
// near-parallel rays — there is no soft parallax/conditioning gate on the path
// that admits a feature to the EKF update. So a feature triangulated at low
// parallax (depth σ a large fraction of the depth itself under 1 px noise) is
// handed to the update at full weight, injecting optimistic information. This is
// the #212 candidate for the aggressive-motion (V2_03) divergence.
//
// The probe's Monte-Carlo uses a fixed seed, so these readings are deterministic
// and gate in CI without the EuRoC dataset.

#include <branes/sdk/eval/triangulation_probe.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>

namespace {
namespace ev = branes::sdk::eval;
using T = double;
}  // namespace

TEST_CASE("S5 triangulate recovers correct depth from clean observations at every parallax",
          "[sdk][s5][triangulation]") {
    const T depth = 5.0;
    const auto sweep = ev::triangulation_parallax_sweep<T>(depth, /*px_noise=*/1.0, /*mc=*/400);
    REQUIRE(sweep.curve.size() == 7);

    for (const auto& p : sweep.curve) {
        INFO("parallax = " << p.parallax_deg << " deg");
        // The geometry is correct everywhere: clean-observation depth error and
        // reprojection are both ~0. The danger is the UNCERTAINTY, not the point.
        REQUIRE(p.status_ok);
        REQUIRE(p.depth_error_mm < 1.0);   // sub-mm on a 5 m point
        REQUIRE(p.reproj_rms_px < 0.05);   // the solution reprojects on top of the obs
    }
}

TEST_CASE("S5 triangulation condition number improves monotonically with parallax",
          "[sdk][s5][triangulation]") {
    const auto sweep = ev::triangulation_parallax_sweep<T>(5.0, 1.0, 400);
    // cond(A) is a deterministic function of the geometry — strictly decreasing as
    // the rays separate. This is the geometric conditioner of depth.
    for (std::size_t i = 1; i < sweep.curve.size(); ++i) {
        INFO("level " << i << ": " << sweep.curve[i].parallax_deg << " deg");
        REQUIRE(sweep.curve[i].condition_number < sweep.curve[i - 1].condition_number);
    }
}

TEST_CASE("S5 low-parallax depth is uncertain past budget while high-parallax is within it",
          "[sdk][s5][triangulation]") {
    const T depth = 5.0;
    const auto sweep = ev::triangulation_parallax_sweep<T>(depth, 1.0, 400);
    const T budget_mm = 0.05 * depth * 1000.0;  // 5% of depth = 250 mm

    const auto& lo = sweep.curve.front();  // 0.1 deg — near-parallel rays
    const auto& hi = sweep.curve.back();   // 10 deg — wide baseline
    INFO("lo σ = " << lo.depth_sigma_mm << " mm, hi σ = " << hi.depth_sigma_mm << " mm, budget = " << budget_mm);

    // At grazing parallax the depth σ blows well past the budget (here ~metres on
    // a 5 m point) — yet triangulate() still returned success above (no soft gate).
    REQUIRE(lo.depth_sigma_mm > budget_mm);
    REQUIRE(lo.condition_number > 1e4);  // ill-conditioned
    // At wide baseline the depth is well-determined and within budget.
    REQUIRE(hi.depth_sigma_mm < budget_mm);
    REQUIRE(hi.condition_number < 1e4);  // well-conditioned
}

TEST_CASE("S5 the suggested soft parallax gate sits at a finite, sane threshold",
          "[sdk][s5][triangulation]") {
    const auto sweep = ev::triangulation_parallax_sweep<T>(5.0, 1.0, 400);
    // The probe reports the smallest swept parallax whose depth σ is within the 5%
    // budget. It must be a real, finite threshold inside the swept band — the basis
    // for setting CameraUpdaterOptions::min_parallax_deg if the gate is enabled.
    REQUIRE(std::isfinite(sweep.gate_parallax_deg));
    REQUIRE(sweep.gate_parallax_deg > 2.0);   // 2 deg σ still exceeds budget
    REQUIRE(sweep.gate_parallax_deg <= 5.0);  // 5 deg σ is within budget
}
