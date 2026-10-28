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

    // inliers and points_cam0 are aligned 1:1 (every kept correspondence has a
    // triangulated landmark), and the landmarks sit in front of camera 0.
    REQUIRE(r.points_cam0.size() == r.inliers.size());
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

TEST_CASE("jacobi_eigh diagonalizes to working precision with orthonormal eigenvectors", "[sdk][sfm][jacobi]") {
    // A 9x9 normal matrix like the 8-point solver's: AtA of a 27x9 system with
    // a fixed pseudo-random fill.
    constexpr std::size_t N = 9;
    ld::Mat<T, N, N> M{};
    std::vector<T> A(27 * N);
    for (std::size_t i = 0; i < A.size(); ++i) {
        const T h = std::sin(static_cast<T>(i) * 12.9898) * 43758.5453;
        A[i] = h - std::floor(h) - 0.5;
    }
    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j)
            for (std::size_t r = 0; r < 27; ++r)
                M(i, j) += A[r * N + i] * A[r * N + j];
    ld::Mat<T, N, N> V;
    ld::Vec<T, N> w;
    sfm::jacobi_eigh<T, N>(M, V, w);
    for (std::size_t k = 1; k < N; ++k)
        REQUIRE(w[k - 1] <= w[k]);  // ascending
    // Vt V = I.
    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j) {
            T s = 0;
            for (std::size_t k = 0; k < N; ++k)
                s += V(k, i) * V(k, j);
            REQUIRE(std::abs(s - (i == j ? T{1} : T{0})) < 1e-14);
        }
    // M = V diag(w) Vt, relative to |M|_F.
    T frob = 0, res = 0;
    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j) {
            T s = 0;
            for (std::size_t k = 0; k < N; ++k)
                s += V(i, k) * w[k] * V(j, k);
            res += (M(i, j) - s) * (M(i, j) - s);
            frob += M(i, j) * M(i, j);
        }
    REQUIRE(std::sqrt(res / frob) < 1e-14);
}

TEST_CASE("jacobi_eigh keeps the null vector of a badly scaled normal matrix", "[sdk][sfm][jacobi]") {
    // M = S G S with G = BtB, the rows of B orthogonal to a unit vector u, and
    // S diagonal over six decades: M's null vector is S^-1 u. A Frobenius-mass
    // stopping test leaves the two smallest eigenvalues coupled here: the
    // hand-rolled Jacobi it replaced missed this null vector by 5.6e-10, the
    // relative test lands at 2e-16.
    constexpr std::size_t N = 9, R = 27;
    T u[N], s[N], un = 0;
    for (std::size_t j = 0; j < N; ++j) {
        const T h = std::sin(static_cast<T>(j + 1) * 78.233) * 43758.5453;
        u[j] = h - std::floor(h) - 0.5;
        un += u[j] * u[j];
        s[j] = std::pow(T{10}, T{6} * (static_cast<T>(j) / (N - 1) - T{0.5}));
    }
    for (auto& x : u)
        x /= std::sqrt(un);
    std::vector<T> B(R * N);
    for (std::size_t r = 0; r < R; ++r) {
        T d = 0;
        for (std::size_t j = 0; j < N; ++j) {
            const T h = std::sin(static_cast<T>(r * N + j) * 12.9898) * 43758.5453;
            B[r * N + j] = h - std::floor(h) - 0.5;
            d += B[r * N + j] * u[j];
        }
        for (std::size_t j = 0; j < N; ++j)
            B[r * N + j] = (B[r * N + j] - d * u[j]) * s[j];  // rows of B S
    }
    ld::Mat<T, N, N> M{};
    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j)
            for (std::size_t r = 0; r < R; ++r)
                M(i, j) += B[r * N + i] * B[r * N + j];
    T z[N], zn = 0;
    for (std::size_t j = 0; j < N; ++j) {
        z[j] = u[j] / s[j];
        zn += z[j] * z[j];
    }
    for (auto& x : z)
        x /= std::sqrt(zn);

    ld::Mat<T, N, N> V;
    ld::Vec<T, N> w;
    sfm::jacobi_eigh<T, N>(M, V, w);
    for (std::size_t k = 1; k < N; ++k)
        REQUIRE(w[k - 1] <= w[k]);
    T dot = 0;
    for (std::size_t i = 0; i < N; ++i)
        dot += V(i, 0) * z[i];
    const T sg = dot < 0 ? T{-1} : T{1};
    T err = 0;
    for (std::size_t i = 0; i < N; ++i)
        err += (V(i, 0) - sg * z[i]) * (V(i, 0) - sg * z[i]);
    REQUIRE(std::sqrt(err) < 1e-12);
}

TEST_CASE("jacobi_eigh converges on a matrix whose squared entries overflow", "[sdk][sfm][jacobi]") {
    // Entries near 1e160: their squares overflow double, so a solver that
    // squares an entry (an unscaled off-diagonal mass, theta^2) would stop
    // early or produce infinities. MTL5's Jacobi squares none.
    constexpr std::size_t N = 4;
    ld::Mat<T, N, N> M{};
    const T base[N][N] = {{4, 1, 0.5, 0.2}, {1, 3, 0.3, 0.1}, {0.5, 0.3, 2, 0.4}, {0.2, 0.1, 0.4, 1}};
    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j)
            M(i, j) = base[i][j] * 1e160;
    ld::Mat<T, N, N> V;
    ld::Vec<T, N> w;
    sfm::jacobi_eigh<T, N>(M, V, w);
    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j) {
            T s = 0;
            for (std::size_t k = 0; k < N; ++k)
                s += V(i, k) * (w[k] / 1e160) * V(j, k);
            REQUIRE(std::abs(base[i][j] - s) < 1e-14);
        }
}
