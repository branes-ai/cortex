// Two-view SfM bootstrap tests (issue #228, epic #211).
//
// Plant a known relative pose and a cloud of 3D points, project them into two
// normalized cameras, and check that estimate_relative_pose recovers the
// rotation and the (up-to-scale) translation direction with the correct
// cheirality — first noise-free, then with pixel noise and gross outliers to
// exercise the RANSAC.

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/sfm/two_view.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace {

namespace sfm = branes::sdk::sfm;
namespace ld = branes::math::lie::detail;
using T = double;
using Vec2 = sfm::Vec2<T>;
using Vec3 = sfm::Vec3<T>;
using Mat3 = sfm::Mat3<T>;
using SO3 = branes::math::lie::SO3<T>;

// A deterministic, genuinely non-coplanar spread of 3D points in front of both
// cameras. Depth must NOT be an affine function of (x, y): coplanar points are
// a degeneracy for the 8-point essential matrix (the solution is not unique).
std::vector<Vec3> scene_points() {
    std::vector<Vec3> pts;
    for (int i = -2; i <= 2; ++i)
        for (int j = -2; j <= 2; ++j) {
            const T z = 3.0 + 0.7 * static_cast<T>((i * 3 + j * 5 + 7) % 4) + 0.2 * (i + 2);
            pts.push_back(Vec3{{0.6 * i, 0.5 * j, z}});
        }
    return pts;
}

Vec2 project(const Vec3& X) {
    return Vec2{{X[0] / X[2], X[1] / X[2]}};
}

T frob_diff(const Mat3& A, const Mat3& B) {
    T s = 0;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            s += (A(i, j) - B(i, j)) * (A(i, j) - B(i, j));
    return std::sqrt(s);
}

// Build matched normalized correspondences for a relative pose (R, t_metric):
// camera 0 is [I|0]; camera 1 sees R·X + t_metric.
void make_correspondences(const Mat3& R, const Vec3& t_metric, std::vector<Vec2>& x0, std::vector<Vec2>& x1) {
    for (const Vec3& X : scene_points()) {
        const Vec3 Y = R * X + t_metric;
        if (!(X[2] > 0.0) || !(Y[2] > 0.0))
            continue;
        x0.push_back(project(X));
        x1.push_back(project(Y));
    }
}

}  // namespace

TEST_CASE("two-view SfM recovers a known relative pose (noise-free)", "[sdk][sfm][two-view]") {
    const Mat3 R_true = SO3::exp(Vec3{{0.05, 0.18, -0.04}}).matrix();
    const Vec3 t_unit_true = Vec3{{1.0, 0.2, 0.1}} * (1.0 / ld::norm(Vec3{{1.0, 0.2, 0.1}}));
    const Vec3 t_metric = t_unit_true * 0.35;  // metric baseline, unobservable to vision

    std::vector<Vec2> x0, x1;
    make_correspondences(R_true, t_metric, x0, x1);
    REQUIRE(x0.size() >= 16);

    const auto r = sfm::estimate_relative_pose<T>(x0, x1);
    REQUIRE(r.success);
    REQUIRE(r.inliers.size() == x0.size());  // every (clean) correspondence is an inlier

    // Rotation recovered tightly.
    CAPTURE(frob_diff(r.R_1_0, R_true));
    REQUIRE(frob_diff(r.R_1_0, R_true) < 1e-6);

    // Translation direction recovered (unit, correct sign via cheirality).
    const T align = ld::dot(r.t_1_0_unit, t_unit_true);
    CAPTURE(align);
    REQUIRE(align > 0.999999);

    // Triangulated points sit in front of camera 0.
    REQUIRE(r.points_cam0.size() == x0.size());
    for (const Vec3& X : r.points_cam0)
        REQUIRE(X[2] > 0.0);
}

TEST_CASE("two-view SfM rejects gross outliers via RANSAC", "[sdk][sfm][two-view]") {
    const Mat3 R_true = SO3::exp(Vec3{{0.0, 0.12, 0.0}}).matrix();
    const Vec3 t_unit_true = Vec3{{1.0, 0.0, 0.0}};
    const Vec3 t_metric = t_unit_true * 0.30;

    std::vector<Vec2> x0, x1;
    make_correspondences(R_true, t_metric, x0, x1);
    const std::size_t n_clean = x0.size();
    REQUIRE(n_clean >= 16);

    // Inject mismatched correspondences (wrong x1) — deterministic garbage.
    for (std::size_t k = 0; k < 5; ++k) {
        x0.push_back(Vec2{{0.1 * static_cast<T>(k) - 0.2, 0.05 * static_cast<T>(k)}});
        x1.push_back(Vec2{{-0.3 + 0.07 * static_cast<T>(k), 0.4 - 0.06 * static_cast<T>(k)}});
    }

    const auto r = sfm::estimate_relative_pose<T>(x0, x1);
    REQUIRE(r.success);
    // The clean correspondences dominate the consensus; nearly all are kept and
    // the pose is recovered. A stray outlier may coincidentally fall on an
    // epipolar line, so allow a small slack on the inlier count.
    REQUIRE(r.inliers.size() >= n_clean - 2);
    REQUIRE(r.inliers.size() <= n_clean + 2);
    REQUIRE(frob_diff(r.R_1_0, R_true) < 1e-3);
    REQUIRE(ld::dot(r.t_1_0_unit, t_unit_true) > 0.999);
}

TEST_CASE("two-view SfM degrades gracefully on too few points", "[sdk][sfm][two-view]") {
    std::vector<Vec2> x0(4, Vec2{{0.1, 0.1}}), x1(4, Vec2{{0.1, 0.1}});
    const auto r = sfm::estimate_relative_pose<T>(x0, x1);
    REQUIRE_FALSE(r.success);
}
