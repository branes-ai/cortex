// PnP resectioning tests (issue #235, epic #211).
//
// Plant a known camera pose and a cloud of 3D landmarks, project them into the
// camera (normalized coordinates), and check that estimate_pose recovers the
// pose — noise-free to high precision, then with gross 2D↔3D mismatches to
// exercise the RANSAC, plus graceful failure on too few points.

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/sfm/pnp.hpp>

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

T frob_diff(const Mat3& A, const Mat3& B) {
    T s = 0;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            s += (A(i, j) - B(i, j)) * (A(i, j) - B(i, j));
    return std::sqrt(s);
}

// A deterministic, volumetric landmark cloud (non-coplanar).
std::vector<Vec3> landmarks() {
    std::vector<Vec3> pts;
    for (int i = -2; i <= 2; ++i)
        for (int j = -2; j <= 2; ++j) {
            const T z = 4.0 + 0.7 * static_cast<T>((i * 3 + j * 5 + 7) % 4) + 0.2 * (i + 2);
            pts.push_back(Vec3{{0.7 * i, 0.6 * j, z}});
        }
    return pts;
}

// Project landmarks (landmark frame) through the camera pose (R, t).
void project_all(
    const Mat3& R, const Vec3& t, const std::vector<Vec3>& X, std::vector<Vec3>& kept3d, std::vector<Vec2>& obs) {
    for (const Vec3& p : X) {
        const Vec3 Xc = R * p + t;
        if (!(Xc[2] > 0.0))
            continue;
        kept3d.push_back(p);
        obs.push_back(Vec2{{Xc[0] / Xc[2], Xc[1] / Xc[2]}});
    }
}

}  // namespace

TEST_CASE("PnP recovers a known camera pose (noise-free)", "[sdk][sfm][pnp]") {
    const Mat3 R_true = SO3::exp(Vec3{{0.1, -0.2, 0.15}}).matrix();
    const Vec3 t_true{{0.3, -0.1, 0.2}};

    std::vector<Vec3> X3;
    std::vector<Vec2> obs;
    project_all(R_true, t_true, landmarks(), X3, obs);
    REQUIRE(X3.size() >= 12);

    const auto r = sfm::estimate_pose<T>(X3, obs);
    REQUIRE(r.success);
    REQUIRE(r.inliers.size() == X3.size());

    CAPTURE(frob_diff(r.R, R_true));
    REQUIRE(frob_diff(r.R, R_true) < 1e-6);
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(r.t[i], Catch::Matchers::WithinAbs(t_true[i], 1e-6));
}

TEST_CASE("PnP rejects gross 2D-3D mismatches via RANSAC", "[sdk][sfm][pnp]") {
    const Mat3 R_true = SO3::exp(Vec3{{0.0, 0.12, -0.05}}).matrix();
    const Vec3 t_true{{0.2, 0.0, 0.1}};

    std::vector<Vec3> X3;
    std::vector<Vec2> obs;
    project_all(R_true, t_true, landmarks(), X3, obs);
    const std::size_t n_clean = X3.size();
    REQUIRE(n_clean >= 16);  // headroom so the corrupted indices below stay in range

    // Corrupt several observations with deterministic garbage (spread across the
    // set; the index guard keeps the writes in bounds regardless of geometry).
    std::size_t corrupted = 0;
    for (std::size_t k = 0; k < 5 && k * 3 < n_clean; ++k, ++corrupted)
        obs[k * 3] = Vec2{{-0.4 + 0.1 * static_cast<T>(k), 0.35 - 0.08 * static_cast<T>(k)}};

    const auto r = sfm::estimate_pose<T>(X3, obs);
    REQUIRE(r.success);
    REQUIRE(r.inliers.size() >= n_clean - corrupted - 2);
    REQUIRE(r.inliers.size() <= n_clean - corrupted + 2);
    REQUIRE(frob_diff(r.R, R_true) < 1e-3);
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(r.t[i], Catch::Matchers::WithinAbs(t_true[i], 1e-2));
}

TEST_CASE("PnP fails gracefully on too few points", "[sdk][sfm][pnp]") {
    std::vector<Vec3> X3(4, Vec3{{0.0, 0.0, 3.0}});
    std::vector<Vec2> obs(4, Vec2{{0.0, 0.0}});
    REQUIRE_FALSE(sfm::estimate_pose<T>(X3, obs).success);
}
