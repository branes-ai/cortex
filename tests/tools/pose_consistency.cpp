// Pose consistency against ground truth (issue #605).
//
// The EuRoC consistency gate reads the yaw / roll-pitch / position NEES from
// branes::tools::PoseConsistency. These cases lock what it computes: errors
// injected in each filter's convention come back exactly in the world frame,
// the gauge anchor absorbs yaw and position but not roll, errors drawn from the
// covariance give an ANEES equal to each block's dimension, and frames without
// ground truth or with a broken covariance are counted, not scored.

#include <branes/tools/pose_consistency.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

namespace {

using T = double;
using SE3 = branes::math::lie::SE3<T>;
using SO3 = branes::math::lie::SO3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using Mat = branes::sdk::msckf::DynMat<T>;
using Pose = branes::sdk::eval::StampedPose<T>;
using branes::tools::PoseConsistency;
using branes::tools::PoseErrorConvention;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// The covariance layouts of the two backends: body-frame MSCKF [δθ δp ...],
// R-IEKF [φ ν ρ ...].
constexpr std::size_t kBodyTheta = 0, kBodyPos = 3, kInvTheta = 0, kInvPos = 6;

Vec3 v3(T x, T y, T z) {
    return Vec3{{x, y, z}};
}

/// A gravity-aligned gauge: yaw about +z and a translation.
SE3 gauge(T yaw, const Vec3& t) {
    return SE3(SO3::exp(v3(0, 0, yaw)), t);
}

/// Ground truth at 20 Hz, rotating and moving.
std::vector<Pose> truth(std::size_t n) {
    std::vector<Pose> gt;
    for (std::size_t k = 0; k < n; ++k) {
        const T s = static_cast<T>(k) * T(0.05);
        gt.push_back(Pose{1000.0 + s, SE3(SO3::exp(v3(0.1 * std::sin(s), -0.05, 0.3 * s)), v3(s, 2 * std::cos(s), 1))});
    }
    return gt;
}

/// A diagonal covariance of dimension n with the given attitude and position σ² at the blocks.
Mat diag_cov(std::size_t n, std::size_t th, std::size_t p, const Vec3& s2_th, T s2_p) {
    Mat P(n, n);
    for (std::size_t i = 0; i < n; ++i)
        P(i, i) = T(1);
    for (std::size_t i = 0; i < 3; ++i) {
        P(th + i, th + i) = s2_th[i];
        P(p + i, p + i) = s2_p;
    }
    return P;
}

}  // namespace

TEST_CASE("Pose consistency: a body-frame error is scored in the world frame", "[tools][consistency]") {
    const auto gt = truth(2);
    PoseConsistency<T> pc(gt);
    const SE3 G = gauge(0.7, v3(3, -1, 0.2));
    const Mat P = diag_cov(15, kBodyTheta, kBodyPos, v3(1e-4, 1e-4, 1e-4), 4e-2);

    // Frame 0 fixes the gauge: the estimate is the truth carried by G.
    REQUIRE(pc.add(gt[0].t_s, G * gt[0].pose, P, kBodyTheta, kBodyPos, PoseErrorConvention::body_right));
    // Frame 1: truth = R̂·Exp(θ), p = p̂ + δp, in the estimate's world.
    const Vec3 th = v3(0.01, -0.02, 0.03), dp = v3(0.1, 0.2, -0.3);
    const SE3 Tt = G * gt[1].pose;
    const SO3 Rh = Tt.rotation() * SO3::exp(v3(-th[0], -th[1], -th[2]));
    const Vec3 pt = Tt.translation();
    const SE3 est(Rh, v3(pt[0] - dp[0], pt[1] - dp[1], pt[2] - dp[2]));
    REQUIRE(pc.add(gt[1].t_s, est, P, kBodyTheta, kBodyPos, PoseErrorConvention::body_right));

    // P's attitude block is isotropic, so in the world frame it stays 1e-4·I and
    // the world error is R̂·θ.
    const Vec3 ew = Rh * th;
    const auto s = pc.summary();
    REQUIRE(s.frames == 2);
    CHECK_THAT(s.nees_yaw, WithinRel(ew[2] * ew[2] / 1e-4 / 2, 1e-9));
    CHECK_THAT(s.nees_rp, WithinRel((ew[0] * ew[0] + ew[1] * ew[1]) / 1e-4 / 2, 1e-9));
    CHECK_THAT(s.nees_pos, WithinRel((0.01 + 0.04 + 0.09) / 4e-2 / 2, 1e-9));
    CHECK_THAT(s.nees_pose, WithinRel(s.nees_rp + s.nees_yaw + s.nees_pos, 1e-9));
    CHECK_THAT(s.yaw_err_last, WithinRel(std::abs(ew[2]), 1e-9));
}

TEST_CASE("Pose consistency: a right-invariant error is scored as the filter defines it", "[tools][consistency]") {
    const auto gt = truth(2);
    PoseConsistency<T> pc(gt);
    const SE3 G = gauge(-1.2, v3(0.5, 4, -2));
    const Mat P = diag_cov(15, kInvTheta, kInvPos, v3(4e-4, 4e-4, 9e-4), 1e-2);

    REQUIRE(pc.add(gt[0].t_s, G * gt[0].pose, P, kInvTheta, kInvPos, PoseErrorConvention::right_invariant));
    // truth = Exp(φ)·R̂, p = Exp(φ)·p̂ + ρ.
    const Vec3 phi = v3(-0.01, 0.015, 0.05), rho = v3(0.05, -0.1, 0.02);
    const SE3 Tt = G * gt[1].pose;
    const SO3 Ei = SO3::exp(v3(-phi[0], -phi[1], -phi[2]));
    const Vec3 pt = Tt.translation();
    const SE3 est(Ei * Tt.rotation(), Ei * v3(pt[0] - rho[0], pt[1] - rho[1], pt[2] - rho[2]));
    REQUIRE(pc.add(gt[1].t_s, est, P, kInvTheta, kInvPos, PoseErrorConvention::right_invariant));

    const auto s = pc.summary();
    CHECK_THAT(s.nees_yaw, WithinRel(phi[2] * phi[2] / 9e-4 / 2, 1e-9));
    CHECK_THAT(s.nees_rp, WithinRel((phi[0] * phi[0] + phi[1] * phi[1]) / 4e-4 / 2, 1e-9));
    CHECK_THAT(s.nees_pos, WithinRel((rho[0] * rho[0] + rho[1] * rho[1] + rho[2] * rho[2]) / 1e-2 / 2, 1e-9));
    CHECK_THAT(s.yaw_sigma_last, WithinRel(0.03, 1e-12));
}

TEST_CASE("Pose consistency: the gauge anchor absorbs yaw and position, not roll", "[tools][consistency]") {
    const auto gt = truth(40);
    const Mat P = diag_cov(15, kBodyTheta, kBodyPos, v3(1e-4, 1e-4, 1e-4), 1e-2);

    SECTION("a constant yaw and position offset scores zero") {
        PoseConsistency<T> pc(gt);
        const SE3 G = gauge(2.5, v3(-7, 3, 1));
        for (const auto& g : gt)
            REQUIRE(pc.add(g.t_s, G * g.pose, P, kBodyTheta, kBodyPos, PoseErrorConvention::body_right));
        const auto s = pc.summary();
        CHECK_THAT(s.nees_pose, WithinAbs(0.0, 1e-12));
        CHECK_THAT(s.yaw_err_max, WithinAbs(0.0, 1e-9));
    }
    SECTION("a constant world roll offset stays in the roll/pitch NEES") {
        PoseConsistency<T> pc(gt);
        const SO3 roll = SO3::exp(v3(0.02, 0, 0));
        for (const auto& g : gt)
            REQUIRE(pc.add(g.t_s,
                           SE3(roll * g.pose.rotation(), roll * g.pose.translation()),
                           P,
                           kBodyTheta,
                           kBodyPos,
                           PoseErrorConvention::body_right));
        const auto s = pc.summary();
        // ~0.02 rad of roll against σ = 0.01: NEES ~4 per frame.
        CHECK(s.nees_rp > 3.0);
        CHECK(s.nees_yaw < 0.1 * s.nees_rp);
    }
}

TEST_CASE("Pose consistency: errors drawn from the covariance give an ANEES of each block's dimension",
          "[tools][consistency]") {
    const auto gt = truth(4001);
    PoseConsistency<T> pc(gt);
    const SE3 G = gauge(0.4, v3(1, 1, 1));
    // Anisotropic in the BODY frame, so the world-frame split depends on the rotation being applied.
    const Vec3 s2 = v3(1e-4, 9e-4, 4e-6);
    const Mat P = diag_cov(15, kBodyTheta, kBodyPos, s2, 2.5e-3);
    std::mt19937_64 rng(605);
    std::normal_distribution<T> n01(0.0, 1.0);

    REQUIRE(pc.add(gt[0].t_s, G * gt[0].pose, P, kBodyTheta, kBodyPos, PoseErrorConvention::body_right));
    for (std::size_t k = 1; k < gt.size(); ++k) {
        Vec3 th{}, dp{};
        for (std::size_t i = 0; i < 3; ++i) {
            th[i] = std::sqrt(s2[i]) * n01(rng);
            dp[i] = 0.05 * n01(rng);
        }
        const SE3 Tt = G * gt[k].pose;
        const Vec3 pt = Tt.translation();
        const SE3 est(Tt.rotation() * SO3::exp(v3(-th[0], -th[1], -th[2])),
                      v3(pt[0] - dp[0], pt[1] - dp[1], pt[2] - dp[2]));
        REQUIRE(pc.add(gt[k].t_s, est, P, kBodyTheta, kBodyPos, PoseErrorConvention::body_right));
    }
    const auto s = pc.summary();
    // 4000 draws: the ANEES of a k-DoF block has σ = √(2k/4000).
    CHECK_THAT(s.nees_yaw, WithinAbs(1.0, 0.1));
    CHECK_THAT(s.nees_rp, WithinAbs(2.0, 0.15));
    CHECK_THAT(s.nees_pos, WithinAbs(3.0, 0.2));
    CHECK_THAT(s.nees_pose, WithinAbs(6.0, 0.3));
}

TEST_CASE("Pose consistency: unmatched frames and broken covariances are counted, not scored", "[tools][consistency]") {
    const auto gt = truth(3);
    PoseConsistency<T> pc(gt);
    const Mat P = diag_cov(15, kBodyTheta, kBodyPos, v3(1e-4, 1e-4, 1e-4), 1e-2);
    REQUIRE(pc.add(gt[0].t_s, gt[0].pose, P, kBodyTheta, kBodyPos, PoseErrorConvention::body_right));

    // 30 ms from the nearest ground truth: past the 20 ms association window.
    CHECK_FALSE(pc.add(gt[2].t_s + 0.03, gt[2].pose, P, kBodyTheta, kBodyPos, PoseErrorConvention::body_right));
    // 10 ms away: matched.
    CHECK(pc.add(gt[1].t_s + 0.01, gt[1].pose, P, kBodyTheta, kBodyPos, PoseErrorConvention::body_right));

    Mat bad = P;
    bad(kBodyTheta + 2, kBodyTheta + 2) = -1e-4;
    CHECK_FALSE(pc.add(gt[2].t_s, gt[2].pose, bad, kBodyTheta, kBodyPos, PoseErrorConvention::body_right));
    Mat nan = P;
    nan(kBodyPos, kBodyPos) = std::nan("");
    CHECK_FALSE(pc.add(gt[2].t_s, gt[2].pose, nan, kBodyTheta, kBodyPos, PoseErrorConvention::body_right));

    const auto s = pc.summary();
    CHECK(s.frames == 2);
    CHECK(s.unmatched == 1);
    CHECK(s.not_pd == 2);
    CHECK_THAT(s.yaw_sigma_first, WithinRel(0.01, 1e-12));
}
