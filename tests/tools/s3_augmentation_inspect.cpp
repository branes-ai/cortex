// SPDX-License-Identifier: MIT
//
// Gate for the S3 augmentation inspector (branes/tools/s3_augmentation_inspect.hpp,
// issue #378). The real inspector clones a live EuRoC keyframe state, which CI does
// not vendor, so here we drive S3AugmentationInspector::run on a synthetic but
// genuinely-correlated state (augment + propagate a few clones, as the probe does)
// and pin the behaviour the before/after heatmap relies on: the new clone block is
// a faithful stochastic copy (its marginal and its cross-covariance with every
// existing state equal the cloned pose's), the dimension grows by exactly one clone
// block, the augmented covariance stays PSD, and the record serializes.

#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/msckf/state_helper.hpp>
#include <branes/tools/s3_augmentation_inspect.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace bt = branes::tools;
namespace ms = branes::sdk::msckf;
using Catch::Matchers::WithinAbs;

namespace {
using Vec3 = ms::State<double>::Vec3;

// A genuinely-correlated state: distinct clone poses + a few IMU steps between
// augmentations build real cross-covariance (mirrors clone_window_probe).
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

TEST_CASE("S3 inspector: a clone is a faithful stochastic copy of the pose", "[tools][s3_inspect]") {
    bt::S3AugmentationInspector::S3Input in{correlated_state(2)};
    const std::size_t d = in.state.dim();
    const std::size_t clones0 = in.state.clones.size();

    const auto r = bt::S3AugmentationInspector().run(in);

    REQUIRE(r.dim_before == d);
    REQUIRE(r.dim_after == d + ms::State<double>::kCloneDim);  // grew by exactly one 6-DoF clone
    REQUIRE(r.clone_offset == d);
    REQUIRE(r.n_clones_after == clones0 + 1);
    // The defining stochastic-cloning invariants: marginal and cross match the pose.
    REQUIRE_THAT(r.clone_marginal_err, WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(r.clone_cross_err, WithinAbs(0.0, 1e-12));
    REQUIRE(r.psd);  // a deterministic copy keeps P positive-semidefinite
    REQUIRE(r.cov_before.size() == d * d);
    REQUIRE(r.cov_after.size() == r.dim_after * r.dim_after);
}

TEST_CASE("S3 inspector: run does not mutate the caller's state", "[tools][s3_inspect]") {
    bt::S3AugmentationInspector::S3Input in{correlated_state(2)};
    const std::size_t d = in.state.dim();
    (void)bt::S3AugmentationInspector().run(in);
    REQUIRE(in.state.dim() == d);  // augmentation ran on a copy
}

TEST_CASE("S3 inspector: record serializes to the figure schema", "[tools][s3_inspect]") {
    const auto r = bt::S3AugmentationInspector().run({correlated_state(1)});
    const auto j = bt::to_json(r);
    REQUIRE(j.at("dim_after") == r.dim_after);
    REQUIRE(j.at("clone_offset") == r.clone_offset);
    REQUIRE(j.at("psd") == true);
    REQUIRE(j.at("cov_before").is_array());
    REQUIRE(j.at("cov_after").is_array());
    REQUIRE(j.at("cov_after").size() == r.dim_after * r.dim_after);
    REQUIRE(j.contains("clone_marginal_err"));
    REQUIRE(j.contains("clone_cross_err"));
}
