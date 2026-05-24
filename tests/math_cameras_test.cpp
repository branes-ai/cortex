// Camera-model tests (issue #39).
//
// For each of the three models (pinhole+radtan, equidistant fisheye,
// unified omnidirectional) we verify:
//   - round-trip: unproject(project(ray)) recovers the original unit
//     bearing, and project(unproject(pixel)) recovers the pixel;
//   - distort/undistort invert each other on normalized coords;
//   - the analytical d(pixel)/d(point) Jacobian matches central finite
//     differences.
// Across double and (looser-tolerance) float.

#include <branes/math/cameras.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>

namespace {

namespace cam = branes::math::cameras;

template <class T>
cam::Vec3<T> normalize3(cam::Vec3<T> v) {
    const T n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return {v[0] / n, v[1] / n, v[2] / n};
}

// Finite-difference d(pixel)/d(point) (central differences).
template <class Camera, class T>
cam::Mat23<T> fd_jacobian(const Camera& c, const cam::Vec3<T>& p, T h) {
    cam::Mat23<T> J{};
    for (std::size_t k = 0; k < 3; ++k) {
        cam::Vec3<T> pp = p, pm = p;
        pp[k] += h;
        pm[k] -= h;
        const auto up = c.project(pp);
        const auto um = c.project(pm);
        J[0 * 3 + k] = (up[0] - um[0]) / (2 * h);
        J[1 * 3 + k] = (up[1] - um[1]) / (2 * h);
    }
    return J;
}

template <class Camera, class T>
void check_jacobian(const Camera& c, const cam::Vec3<T>& p, T h, double tol) {
    const auto Ja = c.project_jacobian(p);
    const auto Jn = fd_jacobian(c, p, h);
    for (std::size_t i = 0; i < 6; ++i)
        REQUIRE(std::abs(double(Ja[i]) - double(Jn[i])) < tol);
}

// A spread of forward-facing rays (Z > 0) used for round-trip checks.
template <class T>
std::array<cam::Vec3<T>, 5> test_rays() {
    return {normalize3<T>({T{0}, T{0}, T{1}}),
            normalize3<T>({T{0.2}, T{0.1}, T{1}}),
            normalize3<T>({T{-0.3}, T{0.25}, T{1}}),
            normalize3<T>({T{0.4}, T{-0.35}, T{1}}),
            normalize3<T>({T{-0.15}, T{-0.2}, T{1}})};
}

}  // namespace

TEST_CASE("pinhole+radtan round-trip and Jacobian", "[math][cameras]") {
    // EuRoC cam0-like intrinsics + distortion.
    cam::PinholeRadtanCamera<double> c(
        458.654, 457.296, 367.215, 248.375, -0.283408, 0.0739591, 0.00019359, 1.76187e-5);
    for (const auto& ray : test_rays<double>()) {
        const auto px = c.project(ray);
        const auto bearing = c.unproject(px);
        // bearing should be parallel to ray (both unit, same hemisphere).
        REQUIRE(std::abs(bearing[0] - ray[0]) < 1e-9);
        REQUIRE(std::abs(bearing[1] - ray[1]) < 1e-9);
        REQUIRE(std::abs(bearing[2] - ray[2]) < 1e-9);
        check_jacobian(c, ray, 1e-6, 1e-4);
    }
    // distort/undistort invert on normalized coords.
    const cam::Vec2<double> n{0.12, -0.08};
    const auto rt = c.undistort(c.distort(n));
    REQUIRE(std::abs(rt[0] - n[0]) < 1e-10);
    REQUIRE(std::abs(rt[1] - n[1]) < 1e-10);
}

TEST_CASE("equidistant fisheye round-trip and Jacobian", "[math][cameras]") {
    cam::EquidistantCamera<double> c(190.0, 190.0, 254.0, 256.0, -0.01, 0.003, -0.001, 0.0002);
    for (const auto& ray : test_rays<double>()) {
        const auto px = c.project(ray);
        const auto bearing = c.unproject(px);
        REQUIRE(std::abs(bearing[0] - ray[0]) < 1e-9);
        REQUIRE(std::abs(bearing[1] - ray[1]) < 1e-9);
        REQUIRE(std::abs(bearing[2] - ray[2]) < 1e-9);
        check_jacobian(c, ray, 1e-6, 1e-4);
    }
    const cam::Vec2<double> n{0.3, 0.2};
    const auto rt = c.undistort(c.distort(n));
    REQUIRE(std::abs(rt[0] - n[0]) < 1e-9);
    REQUIRE(std::abs(rt[1] - n[1]) < 1e-9);
}

TEST_CASE("unified omnidirectional round-trip and Jacobian", "[math][cameras]") {
    cam::OmnidirectionalCamera<double> c(460.0, 460.0, 376.0, 240.0, 0.85, -0.01, 0.002, 0.0001, -0.0002);
    for (const auto& ray : test_rays<double>()) {
        const auto px = c.project(ray);
        const auto bearing = c.unproject(px);
        REQUIRE(std::abs(bearing[0] - ray[0]) < 1e-8);
        REQUIRE(std::abs(bearing[1] - ray[1]) < 1e-8);
        REQUIRE(std::abs(bearing[2] - ray[2]) < 1e-8);
        check_jacobian(c, ray, 1e-6, 1e-4);
    }
}

TEST_CASE("pixel round-trip project(unproject(px)) == px", "[math][cameras]") {
    cam::PinholeRadtanCamera<double> c(
        458.654, 457.296, 367.215, 248.375, -0.283408, 0.0739591, 0.00019359, 1.76187e-5);
    const cam::Vec2<double> px{420.0, 300.0};
    const auto ray = c.unproject(px);
    const auto px2 = c.project(ray);
    REQUIRE(std::abs(px2[0] - px[0]) < 1e-6);
    REQUIRE(std::abs(px2[1] - px[1]) < 1e-6);
}

TEST_CASE("camera models instantiate and round-trip for float", "[math][cameras]") {
    cam::PinholeRadtanCamera<float> c(
        458.654f, 457.296f, 367.215f, 248.375f, -0.283408f, 0.0739591f, 0.00019359f, 1.76187e-5f);
    const auto ray = normalize3<float>({0.2f, -0.1f, 1.0f});
    const auto px = c.project(ray);
    const auto bearing = c.unproject(px);
    REQUIRE(std::abs(bearing[0] - ray[0]) < 1e-3f);
    REQUIRE(std::abs(bearing[1] - ray[1]) < 1e-3f);
    check_jacobian(c, ray, 1e-3f, 1e-1);
}
