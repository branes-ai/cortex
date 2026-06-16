// SPDX-License-Identifier: MIT
//
// Gate for the S5 triangulation inspector (branes/tools/s5_triangulation_inspect.hpp,
// issue #379). The real inspector runs over EuRoC, which CI does not vendor, so
// here we drive S5TriangulationInspector with exact synthetic multi-view geometry
// and pin the behaviour the 3-D overlay + downstream study rely on: the shipped
// triangulator recovers the known point, the reprojection residual is ~0 on clean
// observations, a wider baseline (more parallax) yields a tighter covariance, a
// too-short track is rejected, and the decoupled S5 I/O struct round-trips JSON.

#include <branes/tools/s5_triangulation_inspect.hpp>
#include <branes/tools/vio_trace.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <vector>

namespace bt = branes::tools;
namespace tr = branes::tools::trace;
namespace lie = branes::math::lie;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

using SE3 = lie::SE3<double>;
using SO3 = lie::SO3<double>;
using Vec3 = lie::detail::Vec<double, 3>;

// Two world-aligned cameras (look +z) offset along x by ±half_b, both at z=0,
// observing a point at (0,0,depth). Identity extrinsics. Returns the S5 input.
tr::S5Input<double> two_view(double depth, double half_b, std::uint64_t fid = 7) {
    tr::S5Input<double> in;
    const Vec3 centers[2] = {Vec3{{-half_b, 0.0, 0.0}}, Vec3{{half_b, 0.0, 0.0}}};
    for (const auto& c : centers)
        in.clone_poses.push_back(SE3(SO3{}, c));
    // extrinsics empty → identity (camera == IMU)
    const Vec3 P{{0.0, 0.0, depth}};
    tr::S5Track<double> t;
    t.feature_id = fid;
    for (std::uint32_t i = 0; i < 2; ++i) {
        const Vec3 pc{{P[0] - centers[i][0], P[1] - centers[i][1], P[2] - centers[i][2]}};  // R = I
        t.obs.push_back(tr::S5Observation<double>{i, 0, {pc[0] / pc[2], pc[1] / pc[2]}});
    }
    in.tracks.push_back(std::move(t));
    return in;
}

double trace3(const std::array<double, 9>& c) {
    return c[0] + c[4] + c[8];
}

}  // namespace

TEST_CASE("S5 inspector: recovers a known point with ~0 reprojection residual", "[tools][s5_inspect]") {
    const double depth = 5.0, half_b = 0.5;  // baseline 1 m at 5 m → ~11.4° parallax
    bt::S5TriangulationInspector insp;       // EuRoC focal default
    const auto out = insp.run(two_view(depth, half_b));

    REQUIRE(out.landmarks.size() == 1);
    const auto& lm = out.landmarks[0];
    REQUIRE(lm.feature_id == 7);
    REQUIRE(lm.success);
    REQUIRE(lm.n_obs == 2);
    REQUIRE_THAT(lm.p_world[0], WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(lm.p_world[1], WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(lm.p_world[2], WithinRel(depth, 1e-6));
    REQUIRE(lm.reproj_rms_px < 1e-6);  // clean observations triangulate exactly

    // Parallax: 2·atan(half_b/depth) in degrees.
    const double expect_deg = 2.0 * std::atan(half_b / depth) * (180.0 / 3.14159265358979323846);
    REQUIRE_THAT(lm.parallax_deg, WithinRel(expect_deg, 1e-3));

    // Covariance is finite, symmetric, positive-definite-ish (positive diagonal).
    REQUIRE(std::isfinite(trace3(lm.cov)));
    REQUIRE(lm.cov[0] > 0.0);
    REQUIRE(lm.cov[4] > 0.0);
    REQUIRE(lm.cov[8] > 0.0);
    REQUIRE_THAT(lm.cov[1], WithinAbs(lm.cov[3], 1e-12));  // symmetric
    REQUIRE_THAT(lm.cov[2], WithinAbs(lm.cov[6], 1e-12));
    REQUIRE(lm.condition_number > 1.0);
}

TEST_CASE("S5 inspector: more parallax gives tighter covariance and lower condition number", "[tools][s5_inspect]") {
    bt::S5TriangulationInspector insp;
    const auto narrow = insp.run(two_view(5.0, 0.1)).landmarks[0];  // ~2.3° parallax
    const auto wide = insp.run(two_view(5.0, 1.0)).landmarks[0];    // ~22.6° parallax

    REQUIRE(narrow.success);
    REQUIRE(wide.success);
    REQUIRE(wide.parallax_deg > narrow.parallax_deg);
    // Depth is far better observed with a wide baseline: smaller covariance,
    // better-conditioned system — the core S5 intuition.
    REQUIRE(trace3(wide.cov) < trace3(narrow.cov));
    REQUIRE(wide.condition_number < narrow.condition_number);
}

TEST_CASE("S5 inspector: a single-observation track cannot triangulate", "[tools][s5_inspect]") {
    auto in = two_view(5.0, 0.5);
    in.tracks[0].obs.resize(1);  // drop to one view
    const auto out = bt::S5TriangulationInspector().run(in);
    REQUIRE(out.landmarks.size() == 1);
    REQUIRE_FALSE(out.landmarks[0].success);
    REQUIRE(out.landmarks[0].n_obs == 1);
}

TEST_CASE("S5 I/O struct round-trips JSON (the decoupled S5 boundary)", "[tools][s5_inspect]") {
    const auto in = two_view(5.0, 0.5);

    tr::json ji;
    tr::to_json(ji, in);
    tr::S5Input<double> back;
    tr::from_json(ji, back);
    REQUIRE(back.clone_poses.size() == in.clone_poses.size());
    REQUIRE(back.tracks.size() == 1);
    REQUIRE(back.tracks[0].feature_id == 7);
    REQUIRE(back.tracks[0].obs.size() == 2);
    REQUIRE_THAT(back.tracks[0].obs[1].xy[0], WithinAbs(in.tracks[0].obs[1].xy[0], 1e-12));

    const auto out = bt::S5TriangulationInspector().run(in);
    tr::json jo;
    tr::to_json(jo, out);
    REQUIRE(jo.at("landmarks").is_array());
    const auto& lm = jo.at("landmarks").at(0);
    REQUIRE(lm.at("success") == true);
    REQUIRE(lm.at("p_world").is_array());
    REQUIRE(lm.at("cov").is_array());
    REQUIRE(lm.at("cov").size() == 9);
    REQUIRE(lm.contains("parallax_deg"));
    REQUIRE(lm.contains("reproj_rms_px"));

    tr::S5Output<double> out_back;
    tr::from_json(jo, out_back);
    REQUIRE(out_back.landmarks.size() == 1);
    REQUIRE(out_back.landmarks[0].feature_id == 7);
    REQUIRE_THAT(out_back.landmarks[0].p_world[2], WithinRel(out.landmarks[0].p_world[2], 1e-9));
}
