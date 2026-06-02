// SfM init-window builder tests (issue #230, epic #211).
//
// Plant a moving body trajectory (frame 0 at the origin, gravity-aligned world),
// a cloud of landmarks, and per-frame normalized observations + metric IMU
// preintegration. build_init_window should recover an up-to-scale camera
// trajectory and assemble DynInitKeyframes that drive ImuInitializer::try_dynamic
// to the correct gravity, metric velocities, and the metric scale of the window.

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/sfm/init_window.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

namespace sfm = branes::sdk::sfm;
namespace bs = branes::sdk;
namespace ld = branes::math::lie::detail;
using T = double;
using Vec2 = sfm::Vec2<T>;
using Vec3 = sfm::Vec3<T>;
using Mat3 = sfm::Mat3<T>;
using SO3 = branes::math::lie::SO3<T>;

std::vector<Vec3> landmarks() {
    std::vector<Vec3> pts;
    for (int i = -2; i <= 2; ++i)
        for (int j = -2; j <= 2; ++j) {
            const T z = 5.0 + 0.8 * static_cast<T>((i * 3 + j * 5 + 7) % 4) + 0.3 * (i + 2);
            pts.push_back(Vec3{{0.9 * i, 0.8 * j, z}});
        }
    return pts;
}

T angle_between(const Vec3& a, const Vec3& b) {
    const T c = ld::dot(a, b) / (ld::norm(a) * ld::norm(b));
    return std::acos(std::max(-1.0, std::min(1.0, c)));
}

}  // namespace

TEST_CASE("init window drives try_dynamic to scale, gravity, and velocities", "[sdk][sfm][init-window]") {
    const Vec3 g_world{{0.0, 0.0, -9.81}};
    const T dt = 0.1;
    const std::size_t n = 8;

    // Body trajectory, world == frame-0 body frame (R[0]=I, p[0]=0), gravity
    // along world −z so the recovered gravity is directly comparable.
    std::vector<SO3> R(n);
    std::vector<Vec3> p(n), v(n);
    for (std::size_t i = 0; i < n; ++i) {
        const T s = static_cast<T>(i);
        R[i] = SO3::exp(Vec3{{0.18 * std::sin(0.6 * s), 0.15 * std::cos(0.5 * s), 0.12 * s}});
        p[i] = Vec3{{0.4 * s + 0.2 * std::sin(s), 0.05 * s * s, 0.15 * std::cos(0.5 * s) + 0.1 * s}};
        v[i] = Vec3{{0.4 + 0.15 * std::cos(s), 0.1 * s, -0.08 * std::sin(0.5 * s) + 0.05}};
    }
    p[0] = Vec3{};  // anchor frame 0 at the origin (world == frame-0 frame)

    const auto X = landmarks();
    std::vector<sfm::InitFrame<T>> frames(n);
    for (std::size_t f = 0; f < n; ++f) {
        frames[f].timestamp = static_cast<double>(f) * dt;
        const Mat3 Rt = R[f].inverse().matrix();
        for (std::size_t m = 0; m < X.size(); ++m) {
            const Vec3 Xc = Rt * (X[m] - p[f]);  // camera == body (identity extrinsics)
            if (!(Xc[2] > 0.0))
                continue;
            frames[f].ids.push_back(static_cast<std::uint64_t>(m));
            frames[f].obs.push_back(Vec2{{Xc[0] / Xc[2], Xc[1] / Xc[2]}});
        }
        // Metric IMU preintegration since the previous frame.
        if (f > 0) {
            const Mat3 Rit = R[f - 1].inverse().matrix();
            frames[f].dR = R[f - 1].inverse() * R[f];
            frames[f].dv = Rit * (v[f] - v[f - 1] + g_world * dt);
            frames[f].dp = Rit * (p[f] - p[f - 1] - v[f - 1] * dt + g_world * (0.5 * dt * dt));
            frames[f].dR_dbg = Mat3::identity();
            frames[f].dt = dt;
        }
    }

    const auto win = sfm::build_init_window<T>(frames);
    REQUIRE(win.success);
    REQUIRE(win.keyframes.size() == n);

    bs::ImuInitializer<T> init;
    const auto r = init.try_dynamic(win.keyframes);
    REQUIRE(r.success);

    // Metric scale check, independent of which frame pair the bootstrap chose
    // for its unit baseline: scale · (vision displacement) = metric displacement.
    // Frame 1's metric displacement from frame 0 (the origin) is |p[1]|.
    const Vec3 vis1 = win.keyframes[1].p_world_imu - win.keyframes[0].p_world_imu;
    REQUIRE_THAT(r.scale * ld::norm(vis1), Catch::Matchers::WithinRel(ld::norm(p[1]), 0.05));

    // Gravity recovered (world is gravity-aligned, so directly comparable).
    REQUIRE_THAT(ld::norm(r.gravity_world), Catch::Matchers::WithinAbs(9.81, 1e-2));
    REQUIRE(angle_between(r.gravity_world, g_world) * 180.0 / std::acos(-1.0) < 2.0);

    // Metric velocities recovered.
    REQUIRE(r.velocities_world.size() == n);
    for (std::size_t i = 0; i < n; ++i)
        REQUIRE(ld::norm(Vec3{{r.velocities_world[i][0] - v[i][0],
                               r.velocities_world[i][1] - v[i][1],
                               r.velocities_world[i][2] - v[i][2]}}) < 0.1);
}

TEST_CASE("init window fails gracefully with too few frames", "[sdk][sfm][init-window]") {
    std::vector<sfm::InitFrame<T>> frames(2);
    REQUIRE_FALSE(sfm::build_init_window<T>(frames).success);
}
