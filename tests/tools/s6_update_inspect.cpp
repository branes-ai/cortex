// SPDX-License-Identifier: MIT
//
// Gate for the S6 MSCKF-update inspector (branes/tools/s6_update_inspect.hpp,
// issue #380). The real inspector observes a live EuRoC run, which CI does not
// vendor, so here we drive S6UpdateInspector's self-contained path on an exact,
// deterministic self-consistent (P, R) scene and pin the behaviour the
// filter-internal figures rely on: a clean, well-conditioned update is accepted,
// shrinks the covariance, keeps P PSD, and reports dof = 2m−3 with ~0 residual;
// an outlier observation drives NIS past the χ² gate and is rejected with the
// covariance left untouched; and the record serializes to the figure schema.

#include <branes/math/lie/se3.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/tools/s6_update_inspect.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <vector>

namespace bt = branes::tools;
namespace ms = branes::sdk::msckf;
namespace lie = branes::math::lie;
using Catch::Matchers::WithinAbs;

namespace {

using SE3 = lie::SE3<double>;
using SO3 = lie::SO3<double>;
using Vec3 = lie::detail::Vec<double, 3>;

// A well-conditioned mono scene: m world-aligned clones on the x baseline looking
// +z at a feature F (mirrors update_probe's geometry, ~13° parallax). `outlier`
// perturbs the last observation by that many normalized units (0 = clean).
bt::S6UpdateInspector::S6Input scene(std::size_t m, double sigma_meas, double outlier = 0.0) {
    const double baseline = 0.4;
    const double sigma_theta = 0.0087, sigma_pos = 0.02, sigma_imu = 0.01;
    const Vec3 F{{0.3, -0.2, 5.0}};

    bt::S6UpdateInspector::S6Input in;
    const std::size_t dim = ms::State<double>::kImuDim + ms::State<double>::kCloneDim * m;
    ms::DynMat<double> P(dim, dim);
    for (std::size_t i = 0; i < ms::State<double>::kImuDim; ++i)
        P(i, i) = sigma_imu * sigma_imu;

    ms::FeatureTrack<double> track;
    for (std::size_t c = 0; c < m; ++c) {
        const std::size_t off = ms::State<double>::kImuDim + ms::State<double>::kCloneDim * c;
        for (std::size_t a = 0; a < 3; ++a) {
            P(off + a, off + a) = sigma_theta * sigma_theta;
            P(off + 3 + a, off + 3 + a) = sigma_pos * sigma_pos;
        }
        const Vec3 p_true{{baseline * static_cast<double>(c), 0.0, 0.0}};
        in.clone_poses.push_back(SE3(SO3{}, p_true));  // belief = truth (clean, well-conditioned)
        const Vec3 pc{{F[0] - p_true[0], F[1] - p_true[1], F[2] - p_true[2]}};
        double x = pc[0] / pc[2], y = pc[1] / pc[2];
        if (outlier != 0.0 && c + 1 == m)
            x += outlier;  // corrupt the last observation
        track.observations.push_back(ms::CameraObservation<double>{static_cast<std::size_t>(c), 0, {x, y}});
    }
    (void)sigma_meas;
    in.P = std::move(P);
    in.track = std::move(track);
    return in;
}

bt::S6UpdateInspector make_inspector(double sigma_meas, bool gating) {
    ms::CameraUpdaterOptions<double> opts;
    opts.normalized_sigma = sigma_meas;
    opts.enable_gating = gating;
    opts.min_parallax_deg = 0.0;
    return bt::S6UpdateInspector(std::vector<ms::CameraExtrinsics<double>>(1), opts);
}

}  // namespace

TEST_CASE("S6 inspector: a clean update is accepted, shrinks covariance, stays PSD", "[tools][s6_inspect]") {
    const std::size_t m = 4;  // dof = 2m−3 = 5
    const double sigma = 0.005;
    const auto rec = make_inspector(sigma, /*gating=*/true).run(scene(m, sigma, /*outlier=*/0.0));

    REQUIRE(rec.n_obs == m);
    REQUIRE(rec.dof == 2 * m - 3);
    REQUIRE(rec.valid);
    REQUIRE(rec.accepted);
    REQUIRE_FALSE(rec.gated);
    REQUIRE(rec.residual_rms_px < 1e-6);                  // belief == truth, clean obs ⇒ ~0 innovation
    REQUIRE(rec.cov_trace_after < rec.cov_trace_before);  // the update gains information
    REQUIRE(rec.cov_trace_ratio < 1.0);
    REQUIRE(rec.psd_after);  // Joseph keeps P positive-definite
    REQUIRE(rec.residuals.size() == m);
    REQUIRE(rec.chi2_threshold > 0.0);
}

TEST_CASE("S6 inspector: an outlier observation is chi2-gated and leaves covariance untouched", "[tools][s6_inspect]") {
    const std::size_t m = 4;
    const double sigma = 0.005;
    // A 0.1-normalized (~20σ) corruption on one observation ⇒ NIS ≫ chi2_per_dof·dof.
    const auto rec = make_inspector(sigma, /*gating=*/true).run(scene(m, sigma, /*outlier=*/0.1));

    REQUIRE(rec.valid);           // S is well-conditioned…
    REQUIRE_FALSE(rec.accepted);  // …but the innovation fails the gate
    REQUIRE(rec.gated);
    REQUIRE(rec.nis > rec.chi2_threshold);
    REQUIRE(rec.residual_rms_px > 1.0);                        // a real, large reprojection residual
    REQUIRE_THAT(rec.cov_trace_ratio, WithinAbs(1.0, 1e-12));  // rejected ⇒ covariance unchanged
    REQUIRE_THAT(rec.cov_trace_after, WithinAbs(rec.cov_trace_before, 1e-12));
}

TEST_CASE("S6 inspector: the same outlier is admitted when the gate is OFF", "[tools][s6_inspect]") {
    const std::size_t m = 4;
    const double sigma = 0.005;
    const auto rec = make_inspector(sigma, /*gating=*/false).run(scene(m, sigma, /*outlier=*/0.1));
    REQUIRE(rec.valid);
    REQUIRE(rec.accepted);  // gate disabled ⇒ the outlier is fused
    REQUIRE_FALSE(rec.gated);
    REQUIRE(rec.nis > rec.chi2_threshold);  // it WOULD have been gated — the inspector still reports it
}

TEST_CASE("S6 inspector: record serializes to the figure schema", "[tools][s6_inspect]") {
    const auto rec = make_inspector(0.005, true).run(scene(4, 0.005, 0.0));
    const auto j = bt::to_json(rec);
    REQUIRE(j.at("dof") == 5);
    REQUIRE(j.at("accepted") == true);
    REQUIRE(j.contains("nis_over_dof"));
    REQUIRE(j.contains("cov_trace_before"));
    REQUIRE(j.contains("cov_trace_after"));
    REQUIRE(j.at("psd_after") == true);
    REQUIRE(j.at("residuals").is_array());
    REQUIRE(j.at("residuals").size() == 4);
    for (const auto& o : j.at("residuals")) {
        REQUIRE(o.contains("clone"));
        REQUIRE(o.contains("err"));
    }
}
