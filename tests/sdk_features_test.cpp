// Feature representation tests (issue #41).
//
// Round-trips XYZ ↔ anchored-inverse-depth and checks all three
// representations agree on a synthetic landmark, then verifies the MSCKF
// left-null-space projection: it annihilates the feature Jacobian
// (NᵀH_f = 0) and removes the feature term from a synthetic measurement.

#include <branes/sdk/features.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace {

namespace ft = branes::sdk::features;
using T = double;
using Vec3 = ft::Vec3<T>;
using SE3 = ft::SE3<T>;

void require_vec3_close(const Vec3& a, const Vec3& b, double tol) {
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE(std::abs(a[i] - b[i]) < tol);
}

}  // namespace

TEST_CASE("XYZ <-> inverse-depth round-trip and agreement", "[sdk][features]") {
    const SE3 anchor = SE3::exp(branes::math::lie::detail::Vec<T, 6>{{0.5, -0.3, 1.2, 0.2, -0.1, 0.4}});
    const Vec3 p_world{{2.0, -1.5, 8.0}};

    ft::XyzFeature<T> xyz{p_world};
    const auto inv = ft::to_inverse_depth(anchor, xyz);
    require_vec3_close(inv.world_point(), p_world, 1e-12);

    const auto back = ft::to_xyz(inv);
    require_vec3_close(back.position, p_world, 1e-12);

    // All three representations agree on the world point.
    const auto msckf = ft::to_msckf(xyz);
    require_vec3_close(msckf.world_point(), p_world, 1e-12);
    require_vec3_close(inv.world_point(), msckf.world_point(), 1e-12);
}

TEST_CASE("inverse depth equals 1 / anchor-frame depth", "[sdk][features]") {
    const SE3 anchor;  // identity: anchor frame == world
    const Vec3 p{{1.0, 2.0, 4.0}};
    const auto inv = ft::to_inverse_depth(anchor, ft::XyzFeature<T>{p});
    REQUIRE(std::abs(inv.rho - 0.25) < 1e-12);    // 1/z, z=4
    REQUIRE(std::abs(inv.alpha - 0.25) < 1e-12);  // x/z
    REQUIRE(std::abs(inv.beta - 0.5) < 1e-12);    // y/z
}

TEST_CASE("MSCKF null-space projection annihilates the feature Jacobian", "[sdk][features]") {
    constexpr std::size_t m = 8, n = 5;
    std::vector<T> Hf(m * 3), Hx(m * n);
    for (std::size_t i = 0; i < m; ++i) {
        for (std::size_t j = 0; j < 3; ++j)
            Hf[i * 3 + j] = std::sin(0.7 * i + 1.3 * j) + 0.1 * i;
        for (std::size_t j = 0; j < n; ++j)
            Hx[i * n + j] = std::cos(0.4 * i - 0.9 * j) + 0.05 * j;
    }
    // Project H_f through its own null space: Nᵀ H_f must be ~0.
    const auto proj = ft::msckf_left_nullspace_project<T>(
        std::span<const T>{Hf}, std::span<const T>{Hf}, std::vector<T>(m, 0.0), m, 3);
    REQUIRE(proj.rows == m - 3);
    REQUIRE(proj.cols == 3);
    for (T v : proj.H_x)
        REQUIRE(std::abs(v) < 1e-9);
}

TEST_CASE("MSCKF projection rejects mismatched span sizes", "[sdk][features]") {
    constexpr std::size_t m = 6, n = 4;
    std::vector<T> Hf(m * 3, 1.0), Hx_short((m - 1) * n, 1.0), r(m, 0.0);
    REQUIRE_THROWS_AS(ft::msckf_left_nullspace_project<T>(
                          std::span<const T>{Hf}, std::span<const T>{Hx_short}, std::span<const T>{r}, m, n),
                      std::invalid_argument);
}

TEST_CASE("MSCKF projection removes the feature term from a measurement", "[sdk][features]") {
    constexpr std::size_t m = 9, n = 6;
    std::vector<T> Hf(m * 3), Hx(m * n), r(m);
    std::vector<T> x_true(n), f_true(3);
    for (std::size_t j = 0; j < n; ++j)
        x_true[j] = 0.5 - 0.2 * j;
    for (std::size_t j = 0; j < 3; ++j)
        f_true[j] = 1.0 + 0.3 * j;
    for (std::size_t i = 0; i < m; ++i) {
        T ri = 0;
        for (std::size_t j = 0; j < 3; ++j) {
            Hf[i * 3 + j] = std::sin(0.5 * i + j);
            ri += Hf[i * 3 + j] * f_true[j];
        }
        for (std::size_t j = 0; j < n; ++j) {
            Hx[i * n + j] = std::cos(0.3 * i + 0.7 * j);
            ri += Hx[i * n + j] * x_true[j];
        }
        r[i] = ri;  // r = H_x x_true + H_f f_true
    }
    const auto proj = ft::msckf_left_nullspace_project<T>(
        std::span<const T>{Hf}, std::span<const T>{Hx}, std::span<const T>{r}, m, n);
    REQUIRE(proj.rows == m - 3);
    // r_proj must equal H_x_proj · x_true (the feature term is gone).
    for (std::size_t i = 0; i < proj.rows; ++i) {
        T pred = 0;
        for (std::size_t j = 0; j < n; ++j)
            pred += proj.H_x[i * n + j] * x_true[j];
        REQUIRE(std::abs(pred - proj.r[i]) < 1e-9);
    }
}
