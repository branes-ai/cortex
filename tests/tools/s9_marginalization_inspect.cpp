// SPDX-License-Identifier: MIT
//
// Gate for the S9 marginalization inspector (branes/tools/s9_marginalization_inspect.hpp,
// issue #383). The real inspector drops a clone from a live EuRoC keyframe state,
// which CI does not vendor, so here we drive S9MarginalizationInspector::run on a
// synthetic but genuinely-correlated state (augment + propagate a few clones, as
// the probe does) and pin the behaviour the before/after heatmap relies on:
// dropping a clone is exact principal-submatrix extraction (the kept-state marginal
// is unchanged), the dimension shrinks by exactly one clone block, the reduced
// covariance stays PSD, the dropped clone's information is reported, the caller's
// state is untouched, and the record serializes.

#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/msckf/state_helper.hpp>
#include <branes/tools/s9_marginalization_inspect.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>

namespace bt = branes::tools;
namespace ms = branes::sdk::msckf;
using Catch::Matchers::WithinAbs;

namespace {
using Vec3 = ms::State<double>::Vec3;

ms::State<double> correlated_state(std::size_t nclones) {
    ms::State<double> s(0.3);
    ms::Propagator<double> prop;
    for (std::size_t i = 0; i < nclones; ++i) {
        s.p = Vec3{{static_cast<double>(i) / 2.0, static_cast<double>(i) / 10.0, static_cast<double>(i) / 5.0}};
        ms::StateHelper<double>::augment_clone(s);
        for (int k = 0; k < 10; ++k)
            prop.propagate(s, Vec3{{0.01, -0.005, 0.008}}, Vec3{{0.05, 0.02, 9.81}}, 0.01);
    }
    return s;
}

}  // namespace

TEST_CASE("S9 inspector: marginalization is exact principal-submatrix extraction", "[tools][s9_inspect]") {
    bt::S9MarginalizationInspector::S9Input in{correlated_state(3), /*idx=*/0};  // drop the oldest clone
    const std::size_t d = in.state.dim();
    const std::size_t clones0 = in.state.clones.size();

    const auto r = bt::S9MarginalizationInspector().run(in);

    REQUIRE(r.valid);
    REQUIRE(r.clone_dim == ms::State<double>::kCloneDim);  // reported block size bound to the SDK constant
    REQUIRE(r.dim_before == d);
    REQUIRE(r.dim_after == d - ms::State<double>::kCloneDim);  // shrank by exactly one 6-DoF clone
    REQUIRE(r.n_clones_after == clones0 - 1);
    // The defining marginalization invariant: the kept-state marginal is unchanged.
    REQUIRE_THAT(r.kept_marginal_err, WithinAbs(0.0, 1e-12));
    REQUIRE(r.psd);  // the reduced covariance stays positive-semidefinite
    // The dropped clone carried real information (uncertainty + correlations).
    REQUIRE(r.dropped_sigma > 0.0);
    REQUIRE(r.max_cross_dropped > 0.0);
    REQUIRE(r.cov_before.size() == d * d);
    REQUIRE(r.cov_after.size() == r.dim_after * r.dim_after);
}

TEST_CASE("S9 inspector: dropping a middle clone leaves the others' marginal exact", "[tools][s9_inspect]") {
    const auto r = bt::S9MarginalizationInspector().run({correlated_state(3), /*idx=*/1});
    REQUIRE(r.dropped_index == 1);
    REQUIRE_THAT(r.kept_marginal_err, WithinAbs(0.0, 1e-12));
    REQUIRE(r.psd);
}

TEST_CASE("S9 inspector: run does not mutate the caller's state", "[tools][s9_inspect]") {
    bt::S9MarginalizationInspector::S9Input in{correlated_state(3), 0};
    const std::size_t d = in.state.dim();
    const std::size_t clones = in.state.clones.size();
    const auto cov_before = in.state.covariance();

    (void)bt::S9MarginalizationInspector().run(in);

    REQUIRE(in.state.dim() == d);
    REQUIRE(in.state.clones.size() == clones);
    const auto cov_after = in.state.covariance();
    double maxdiff = 0.0;
    for (std::size_t i = 0; i < cov_before.rows; ++i)
        for (std::size_t j = 0; j < cov_before.cols; ++j)
            maxdiff = std::max(maxdiff, std::abs(cov_after(i, j) - cov_before(i, j)));
    REQUIRE(maxdiff == 0.0);  // marginalization ran entirely on a copy
}

TEST_CASE("S9 inspector: an out-of-range clone index is a no-op", "[tools][s9_inspect]") {
    auto in = bt::S9MarginalizationInspector::S9Input{correlated_state(2), 99};
    const auto r = bt::S9MarginalizationInspector().run(in);
    REQUIRE_FALSE(r.valid);     // fails closed — an out-of-range index is not a clean pass
    REQUIRE(r.dim_after == 0);  // signalled invalid — nothing dropped
    REQUIRE(r.kept_marginal_err == 0.0);
}

TEST_CASE("S9 inspector: record serializes to the figure schema", "[tools][s9_inspect]") {
    const auto r = bt::S9MarginalizationInspector().run({correlated_state(3), 0});
    const auto j = bt::to_json(r);
    REQUIRE(j.at("valid") == true);
    REQUIRE(j.at("dim_after") == r.dim_after);
    REQUIRE(j.at("dropped_offset") == r.dropped_offset);
    REQUIRE(j.at("psd") == true);
    REQUIRE(j.at("cov_before").is_array());
    REQUIRE(j.at("cov_after").size() == r.dim_after * r.dim_after);
    REQUIRE(j.contains("kept_marginal_err"));
    REQUIRE(j.contains("dropped_sigma"));
    REQUIRE(j.contains("max_cross_dropped"));
}
