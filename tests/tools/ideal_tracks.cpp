// Ideal tracks (#602): observations synthesized from ground truth. Without
// noise they must be exactly the projection of a fixed world landmark through
// the interpolated pose, S4's spacing and feature count must hold, and no
// observation may appear where the ground truth does not cover the stamp.

#include <branes/tools/euroc_cam0.hpp>
#include <branes/tools/ideal_tracks.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <map>
#include <utility>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
using T = double;
using SE3 = branes::math::lie::SE3<T>;
using SO3 = branes::math::lie::SO3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using Tracks = branes::tools::IdealTracks<T>;

/// Ground truth at 200 Hz over [0, 1] s: the body yaws at `rate` rad/s and
/// moves along x at `speed` m/s.
std::vector<ev::StampedPose<T>> gt(double rate, double speed) {
    std::vector<ev::StampedPose<T>> g;
    for (int k = 0; k <= 200; ++k) {
        const double t = 0.005 * k;
        g.push_back({t, SE3(SO3::exp({0.0, 0.0, rate * t}), Vec3{{speed * t, 0.0, 0.0}})});
    }
    return g;
}

branes::tools::IdealTrackOptions clean() {
    branes::tools::IdealTrackOptions o;
    o.sigma_px = 0.0;
    return o;
}

}  // namespace

TEST_CASE("ideal tracks fill the view with spaced features", "[tools][ideal_tracks]") {
    const auto cam = branes::tools::euroc_cam0<T>();
    Tracks tr(gt(0.0, 0.0), cam.intrinsics, SE3(cam.R_imu_cam, cam.p_imu_cam), clean());
    const auto a = tr.observe(0.1);
    REQUIRE(a.size() == 150);
    for (std::size_t i = 0; i < a.size(); ++i)
        for (std::size_t j = i + 1; j < a.size(); ++j)
            REQUIRE(std::hypot(a[i].u - a[j].u, a[i].v - a[j].v) >= 15.0);
    // At rest and without noise the next frame sees the same landmarks at the
    // same pixels.
    const auto b = tr.observe(0.15);
    REQUIRE(b.size() == a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        REQUIRE(b[i].feature_id == a[i].feature_id);
        REQUIRE_THAT(b[i].u, Catch::Matchers::WithinAbs(a[i].u, 1e-9));
        REQUIRE_THAT(b[i].v, Catch::Matchers::WithinAbs(a[i].v, 1e-9));
    }
    REQUIRE(tr.tracks() == 150);
}

TEST_CASE("ideal tracks are projections of fixed landmarks", "[tools][ideal_tracks]") {
    // Two frames of a moving, yawing camera: a landmark triangulated from its
    // two clean observations must reproject onto a third frame's observation.
    const auto cam = branes::tools::euroc_cam0<T>();
    const SE3 T_ic(cam.R_imu_cam, cam.p_imu_cam);
    Tracks tr(gt(0.3, 0.5), cam.intrinsics, T_ic, clean());
    const double ts[3] = {0.2, 0.4625, 0.7};  // the middle stamp falls between ground-truth samples
    std::map<std::uint64_t, std::vector<std::pair<T, T>>> seen;
    for (const double t : ts)
        for (const auto& o : tr.observe(t))
            seen[o.feature_id].emplace_back(o.u, o.v);
    std::size_t checked = 0;
    for (const auto& [id, px] : seen) {
        if (px.size() != 3)
            continue;
        // Midpoint triangulation from the first two views (world frame).
        Vec3 c[2], d[2];
        for (int k = 0; k < 2; ++k) {
            const SE3 T_wc = *tr.pose_at(ts[k]) * T_ic;
            const auto b = cam.intrinsics.unproject({px[k].first, px[k].second});
            c[k] = T_wc.translation();
            d[k] = T_wc.rotation() * Vec3{{b[0], b[1], b[2]}};
        }
        const auto dot = [](const Vec3& x, const Vec3& y) { return x[0] * y[0] + x[1] * y[1] + x[2] * y[2]; };
        const Vec3 w{{c[0][0] - c[1][0], c[0][1] - c[1][1], c[0][2] - c[1][2]}};
        const double a = dot(d[0], d[0]), b = dot(d[0], d[1]), cc = dot(d[1], d[1]);
        const double dd = dot(d[0], w), e = dot(d[1], w), den = a * cc - b * b;
        const double s = (b * e - cc * dd) / den, u = (a * e - b * dd) / den;
        const Vec3 X{{0.5 * (c[0][0] + s * d[0][0] + c[1][0] + u * d[1][0]),
                      0.5 * (c[0][1] + s * d[0][1] + c[1][1] + u * d[1][1]),
                      0.5 * (c[0][2] + s * d[0][2] + c[1][2] + u * d[1][2])}};
        const SE3 T_cw = (*tr.pose_at(ts[2]) * T_ic).inverse();
        const Vec3 p = T_cw * X;
        const auto q = cam.intrinsics.project({p[0], p[1], p[2]});
        REQUIRE_THAT(q[0], Catch::Matchers::WithinAbs(px[2].first, 1e-4));
        REQUIRE_THAT(q[1], Catch::Matchers::WithinAbs(px[2].second, 1e-4));
        ++checked;
    }
    REQUIRE(checked > 50);
}

TEST_CASE("ideal tracks need ground truth at the stamp", "[tools][ideal_tracks]") {
    const auto cam = branes::tools::euroc_cam0<T>();
    Tracks tr(gt(0.0, 0.0), cam.intrinsics, SE3(cam.R_imu_cam, cam.p_imu_cam));
    REQUIRE(tr.observe(-0.01).empty());
    REQUIRE(tr.observe(1.5).empty());
    REQUIRE_FALSE(tr.pose_at(1.5).has_value());
    // With noise the pixels move but the count does not.
    REQUIRE(tr.observe(0.5).size() == 150);
}
