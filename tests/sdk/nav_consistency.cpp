// NEES-against-ground-truth helpers (issue #264): the nav-state error vector,
// the gauge anchor, and the core-covariance extraction.
//
// Verified against an independent oracle: a known error injected as
// truth = est ⊞ δ must be recovered exactly by nav_error (the convention test),
// the gauge anchor must reproduce the estimate pose at the matched frame (the
// algebra test), and nav_error composed with `nees` must equal the
// hand-computed quadratic form.

#include <branes/sdk/eval/nav_consistency.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>

namespace {

namespace ev = branes::sdk::eval;
using T = double;
using SO3 = branes::math::lie::SO3<T>;
using SE3 = branes::math::lie::SE3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using ev::DynMat;

}  // namespace

TEST_CASE("nav_error recovers a known injected error (convention test)", "[sdk][eval][nav-consistency]") {
    ev::NavSample<T> est;
    est.R = SO3::exp(Vec3{{0.2, -0.3, 0.5}});
    est.p = Vec3{{1.0, -2.0, 3.0}};
    est.v = Vec3{{0.4, 0.1, -0.2}};
    est.bg = Vec3{{0.01, -0.02, 0.005}};
    est.ba = Vec3{{0.1, 0.05, -0.03}};

    // Inject a known 15-dim error: truth = est ⊞ δ. For the right-perturbation
    // attitude convention R_true = R̂·Exp(δθ), and additive for the rest.
    const Vec3 dtheta{{0.03, -0.04, 0.02}};
    const Vec3 dp{{0.10, -0.20, 0.30}};
    const Vec3 dv{{-0.05, 0.06, 0.07}};
    const Vec3 dbg{{0.001, 0.002, -0.003}};
    const Vec3 dba{{0.02, -0.01, 0.04}};

    ev::NavSample<T> truth;
    truth.R = est.R * SO3::exp(dtheta);
    truth.p = est.p + dp;
    truth.v = est.v + dv;
    truth.bg = est.bg + dbg;
    truth.ba = est.ba + dba;

    const auto e = ev::nav_error<T>(est, truth);
    const std::array<Vec3, 5> expect{dtheta, dp, dv, dbg, dba};
    for (std::size_t blk = 0; blk < 5; ++blk)
        for (std::size_t k = 0; k < 3; ++k)
            REQUIRE_THAT(e[blk * 3 + k], Catch::Matchers::WithinAbs(expect[blk][k], 1e-12));

    // Zero error ⇒ zero vector.
    const auto e0 = ev::nav_error<T>(est, est);
    for (T x : e0)
        REQUIRE_THAT(x, Catch::Matchers::WithinAbs(0.0, 1e-12));
}

TEST_CASE("gauge_align anchors the truth pose onto the estimate", "[sdk][eval][nav-consistency]") {
    // Estimate and truth in different world frames.
    const SE3 est_pose(SO3::exp(Vec3{{0.1, 0.2, 0.9}}), Vec3{{5.0, -1.0, 2.0}});
    const SE3 truth_pose(SO3::exp(Vec3{{-0.3, 0.05, 0.4}}), Vec3{{-2.0, 3.0, 1.0}});

    const SE3 T_align = ev::gauge_align<T>(est_pose, truth_pose);

    ev::NavSample<T> truth;
    truth.R = truth_pose.rotation();
    truth.p = truth_pose.translation();
    truth.v = Vec3{{0.3, -0.1, 0.2}};
    truth.bg = Vec3{{0.001, 0.0, -0.001}};
    truth.ba = Vec3{{0.02, 0.01, 0.0}};

    const ev::NavSample<T> aligned = ev::align_truth<T>(T_align, truth);

    // The aligned truth pose must equal the estimate pose at the anchor frame.
    const Vec3 dtheta = (est_pose.rotation().inverse() * aligned.R).log();
    for (std::size_t k = 0; k < 3; ++k) {
        REQUIRE_THAT(dtheta[k], Catch::Matchers::WithinAbs(0.0, 1e-12));
        REQUIRE_THAT(aligned.p[k], Catch::Matchers::WithinAbs(est_pose.translation()[k], 1e-12));
    }
    // Velocity rotates by R_align; biases (IMU-frame) are unchanged.
    const Vec3 v_expect = T_align.rotation() * truth.v;
    for (std::size_t k = 0; k < 3; ++k) {
        REQUIRE_THAT(aligned.v[k], Catch::Matchers::WithinAbs(v_expect[k], 1e-12));
        REQUIRE_THAT(aligned.bg[k], Catch::Matchers::WithinAbs(truth.bg[k], 1e-12));
        REQUIRE_THAT(aligned.ba[k], Catch::Matchers::WithinAbs(truth.ba[k], 1e-12));
    }
}

TEST_CASE("nav_error feeds nees as a quadratic form", "[sdk][eval][nav-consistency]") {
    ev::NavSample<T> est;  // default (identity R, zero vectors)
    ev::NavSample<T> truth;
    truth.p = Vec3{{0.3, 0.0, 0.0}};  // pure 0.3 m position error, nothing else
    const auto e = ev::nav_error<T>(est, truth);
    // Identity 15×15 covariance ⇒ NEES = ‖e‖² = 0.09.
    DynMat<T> P(ev::kNavErrorDim, ev::kNavErrorDim);
    for (std::size_t i = 0; i < ev::kNavErrorDim; ++i)
        P(i, i) = 1.0;
    REQUIRE_THAT(ev::nees<T>(e, P), Catch::Matchers::WithinAbs(0.09, 1e-12));
}

TEST_CASE("core_covariance extracts the top-left block", "[sdk][eval][nav-consistency]") {
    DynMat<T> full(18, 18);  // 15 core + one 3-dim clone-ish tail
    for (std::size_t i = 0; i < 18; ++i)
        for (std::size_t j = 0; j < 18; ++j)
            full(i, j) = static_cast<T>(i * 100 + j);
    const auto c = ev::core_covariance<T>(full);
    REQUIRE(c.rows == 15);
    REQUIRE(c.cols == 15);
    REQUIRE_THAT(c(0, 0), Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(c(14, 14), Catch::Matchers::WithinAbs(14.0 * 100 + 14, 1e-12));
    REQUIRE_THROWS_AS(ev::core_covariance<T>(DynMat<T>(10, 10)), std::invalid_argument);
}
