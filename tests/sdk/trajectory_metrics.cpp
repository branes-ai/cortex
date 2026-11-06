// Trajectory metrics for comparing VIO estimators: the position + yaw (4-DoF)
// ATE and RPE over a fixed travelled distance (docs/benchmarks/euroc-baseline.mdx).
// Known answers on synthetic trajectories; the rigid ATE and the fixed-step RPE
// are covered in vio_euroc.cpp.

#include <branes/sdk/eval/trajectory_metrics.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
using T = double;
using SE3 = branes::math::lie::SE3<T>;
using SO3 = branes::math::lie::SO3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using Catch::Matchers::WithinAbs;

ev::StampedPose<T> pose_at(double t, const Vec3& p, const SO3& R = {}) {
    return ev::StampedPose<T>{t, SE3(R, p)};
}

/// A curved 3-D path (so the alignments are well-posed) with a turning heading.
std::vector<ev::StampedPose<T>> curved_path(int n = 60) {
    std::vector<ev::StampedPose<T>> gt;
    for (int k = 0; k < n; ++k) {
        const T s = static_cast<T>(k);
        gt.push_back(pose_at(0.1 * k,
                             Vec3{{2.0 * std::cos(0.1 * s), 1.5 * std::sin(0.1 * s), 0.02 * s}},
                             SO3::exp(Vec3{{0.01 * s, -0.005 * s, 0.1 * s}})));
    }
    return gt;
}

std::vector<ev::StampedPose<T>> transformed(const SE3& g, const std::vector<ev::StampedPose<T>>& in) {
    std::vector<ev::StampedPose<T>> out;
    for (const auto& s : in)
        out.push_back(ev::StampedPose<T>{s.t_s, g * s.pose});
    return out;
}

}  // namespace

TEST_CASE("4-DoF ATE removes yaw and position, and only those", "[eval][trajectory]") {
    const auto gt = curved_path();

    // Yaw + translation is the VIO gauge: the 4-DoF alignment absorbs it.
    const auto yawed = transformed(SE3(SO3::exp(Vec3{{0.0, 0.0, 1.2}}), Vec3{{3.0, -1.0, 0.5}}), gt);
    REQUIRE_THAT(ev::ate_rmse_4dof(yawed, gt), WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(ev::ate_rmse(yawed, gt), WithinAbs(0.0, 1e-7));

    // A roll is observable to a VIO: the rigid alignment absorbs it, the 4-DoF
    // one must not.
    const auto rolled = transformed(SE3(SO3::exp(Vec3{{0.2, 0.0, 0.0}}), Vec3{{0.0, 0.0, 0.0}}), gt);
    REQUIRE_THAT(ev::ate_rmse(rolled, gt), WithinAbs(0.0, 1e-7));
    REQUIRE(ev::ate_rmse_4dof(rolled, gt) > 0.05);
}

TEST_CASE("align_yaw recovers a known heading offset", "[eval][trajectory]") {
    const auto gt = curved_path();
    std::vector<Vec3> pe, pr;
    for (const auto& s : gt) {
        pr.push_back(s.pose.translation());
        pe.push_back(SO3::exp(Vec3{{0.0, 0.0, -0.7}}) * s.pose.translation() + Vec3{{1.0, 2.0, 3.0}});
    }
    const auto a = ev::align_yaw<T>(pe, pr);
    const auto phi = a.R.log();
    REQUIRE_THAT(phi[0], WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(phi[1], WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(phi[2], WithinAbs(0.7, 1e-9));
    REQUIRE_THROWS_AS(ev::align_yaw<T>({}, {}), std::invalid_argument);
}

TEST_CASE("4-DoF ATE is never below the rigid ATE", "[eval][trajectory]") {
    const auto gt = curved_path();
    std::vector<ev::StampedPose<T>> est;
    for (std::size_t k = 0; k < gt.size(); ++k) {
        const T w = static_cast<T>(k % 7) - 3.0;  // a non-rigid wobble
        est.push_back(ev::StampedPose<T>{gt[k].t_s,
                                         SE3(SO3::exp(Vec3{{0.1, 0.0, 0.3}}), Vec3{{0.0, 0.0, 0.0}}) * gt[k].pose *
                                             SE3(SO3{}, Vec3{{0.01 * w, -0.02 * w, 0.005 * w}})});
    }
    REQUIRE(ev::ate_rmse_4dof(est, gt) >= ev::ate_rmse(est, gt) - 1e-12);
}

TEST_CASE("distance RPE: a 10% scale error over a straight path", "[eval][trajectory]") {
    // Ground truth moves 1 m per pose along x; the estimate 1.1 m. Over L metres
    // of travel the relative-motion error is exactly 0.1·L, with no rotation.
    std::vector<ev::StampedPose<T>> gt, est;
    for (int k = 0; k <= 50; ++k) {
        gt.push_back(pose_at(0.1 * k, Vec3{{1.0 * k, 0.0, 0.0}}));
        est.push_back(pose_at(0.1 * k, Vec3{{1.1 * k, 0.0, 0.0}}));
    }
    const auto r = ev::rpe_by_distance<T>(est, gt, {8.0, 40.0, 100.0});
    REQUIRE(r.size() == 2);  // the path is 50 m: no 100 m segment
    REQUIRE(r[0].length_m == 8.0);
    REQUIRE(r[0].pairs == 43);  // starts 0..42
    REQUIRE_THAT(r[0].translation_m, WithinAbs(0.8, 1e-12));
    REQUIRE_THAT(r[0].rotation_deg, WithinAbs(0.0, 1e-12));
    REQUIRE(r[1].pairs == 11);
    REQUIRE_THAT(r[1].translation_m, WithinAbs(4.0, 1e-12));
}

TEST_CASE("distance RPE: a constant heading-rate error", "[eval][trajectory]") {
    // The estimate's heading drifts 0.01 rad per pose; over 8 poses (8 m) the
    // relative rotation is off by 0.08 rad, whatever the start.
    std::vector<ev::StampedPose<T>> gt, est;
    for (int k = 0; k <= 30; ++k) {
        gt.push_back(pose_at(0.1 * k, Vec3{{1.0 * k, 0.0, 0.0}}));
        est.push_back(pose_at(0.1 * k, Vec3{{1.0 * k, 0.0, 0.0}}, SO3::exp(Vec3{{0.0, 0.0, 0.01 * k}})));
    }
    const auto r = ev::rpe_by_distance<T>(est, gt, {8.0});
    REQUIRE(r.size() == 1);
    REQUIRE_THAT(r[0].rotation_deg, WithinAbs(0.08 * 180.0 / std::numbers::pi, 1e-9));
}

TEST_CASE("distance RPE is invariant to a global rigid transform", "[eval][trajectory]") {
    const auto gt = curved_path();
    const auto est = transformed(SE3(SO3::exp(Vec3{{0.3, -0.2, 1.0}}), Vec3{{5.0, 1.0, -2.0}}), gt);
    for (const auto& s : ev::rpe_by_distance<T>(est, gt, {1.0, 2.0, 4.0})) {
        REQUIRE(s.pairs > 0);
        REQUIRE_THAT(s.translation_m, WithinAbs(0.0, 1e-9));
        REQUIRE_THAT(s.rotation_deg, WithinAbs(0.0, 1e-6));
    }
}

TEST_CASE("distance RPE rejects bad input", "[eval][trajectory]") {
    const auto gt = curved_path(10);
    REQUIRE_THROWS_AS(ev::rpe_by_distance<T>(gt, {}, {1.0}), std::invalid_argument);
    REQUIRE_THROWS_AS(ev::rpe_by_distance<T>(gt, gt, {0.0}), std::invalid_argument);
}
