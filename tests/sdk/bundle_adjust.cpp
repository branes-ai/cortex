// SfM window bundle adjustment tests (issue #501).
//
// Plant cameras and points, observe them, perturb the estimate, and check that
// bundle_adjust returns to the truth: exactly with noise-free observations (up
// to the free scale gauge, with frame 0 held), and robustly with a gross outlier
// under the Huber weights. Plus the N-view linear triangulation it starts from.

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/sfm/bundle_adjust.hpp>

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

/// A planted scene: world→camera poses (frame 0 is the identity) and points.
struct Scene {
    std::vector<Mat3> R;
    std::vector<Vec3> t;
    std::vector<Vec3> X;
    std::vector<sfm::BaObservation<T>> obs;
};

Scene scene(std::size_t frames = 6) {
    Scene s;
    for (std::size_t f = 0; f < frames; ++f) {
        const T a = static_cast<T>(f);
        const Mat3 R = SO3::exp(Vec3{{0.02 * a, -0.03 * a, 0.015 * a}}).matrix();
        const Vec3 c{{0.12 * a, 0.03 * a * a, 0.02 * a}};  // camera centre in the world
        s.R.push_back(R);
        s.t.push_back(R * c * (-1.0));
    }
    for (int i = -3; i <= 3; ++i)
        for (int j = -2; j <= 2; ++j)
            s.X.push_back(Vec3{{0.5 * i, 0.4 * j, 4.0 + 0.6 * static_cast<T>((i * 5 + j * 3 + 11) % 5)}});
    for (std::size_t f = 0; f < frames; ++f)
        for (std::size_t p = 0; p < s.X.size(); ++p) {
            const Vec3 c = s.R[f] * s.X[p] + s.t[f];
            s.obs.push_back(sfm::BaObservation<T>{f, p, Vec2{{c[0] / c[2], c[1] / c[2]}}});
        }
    return s;
}

/// The scene with every pose but frame 0 and every point perturbed.
void perturb(Scene& s) {
    for (std::size_t f = 1; f < s.R.size(); ++f) {
        const T a = static_cast<T>(f);
        s.R[f] = SO3::exp(Vec3{{0.01 * std::sin(a), -0.008 * std::cos(a), 0.006}}).matrix() * s.R[f];
        s.t[f] = s.t[f] + Vec3{{0.02 * std::cos(2.0 * a), -0.015, 0.01 * std::sin(a)}};
    }
    for (std::size_t p = 0; p < s.X.size(); ++p) {
        const T a = static_cast<T>(p);
        s.X[p] = s.X[p] + Vec3{{0.05 * std::sin(a), 0.04 * std::cos(1.3 * a), 0.1 * std::sin(0.7 * a)}};
    }
}

T rot_err(const Mat3& A, const Mat3& B) {
    T s = 0;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            s += (A(i, j) - B(i, j)) * (A(i, j) - B(i, j));
    return std::sqrt(s);
}

std::vector<char> hold_first(std::size_t n) {
    std::vector<char> fixed(n, 0);
    fixed[0] = 1;
    return fixed;
}

}  // namespace

TEST_CASE("N-view triangulation recovers a point exactly", "[sdk][sfm][bundle-adjust]") {
    const auto s = scene(4);
    const Vec3& X = s.X[7];
    std::vector<Vec2> x;
    for (std::size_t f = 0; f < 4; ++f)
        x.push_back(s.obs[f * s.X.size() + 7].x);
    Vec3 Y;
    REQUIRE(sfm::triangulate_views<T>(s.R, s.t, x, Y));
    REQUIRE(ld::norm(Y - X) < 1e-9);

    // A point consistent with both views but behind one of them is rejected:
    // camera 1 sits beyond the point along camera 0's axis, looking the same
    // way, and its observation is the point's (mirrored) projection.
    const Vec3 P{{0.3, -0.2, 4.0}};
    const std::vector<Mat3> R2{Mat3::identity(), Mat3::identity()};
    const std::vector<Vec3> t2{Vec3{}, Vec3{{0.0, 0.0, -8.0}}};  // camera 1 at z = 8
    const Vec3 c1 = P + t2[1];                                   // depth −4
    const std::vector<Vec2> x2{Vec2{{P[0] / P[2], P[1] / P[2]}}, Vec2{{c1[0] / c1[2], c1[1] / c1[2]}}};
    Vec3 Z;
    REQUIRE_FALSE(sfm::triangulate_views<T>(R2, t2, x2, Z));
}

TEST_CASE("bundle adjustment returns a perturbed window to the truth", "[sdk][sfm][bundle-adjust]") {
    const auto truth = scene();
    auto s = truth;
    perturb(s);
    const auto fixed = hold_first(s.R.size());
    const auto r = sfm::bundle_adjust<T>(s.R, s.t, s.X, s.obs, fixed);
    REQUIRE(r.success);
    REQUIRE(r.initial_rms > 1e-3);
    REQUIRE(r.final_rms < 1e-9);
    REQUIRE(r.iterations > 0);

    // Frame 0 is held; the scale gauge is free, so translations match the truth
    // up to one factor, read off frame 1.
    REQUIRE(rot_err(s.R[0], truth.R[0]) == 0.0);
    const T k = ld::norm(s.t[1]) / ld::norm(truth.t[1]);
    REQUIRE_THAT(k, Catch::Matchers::WithinAbs(1.0, 0.1));
    for (std::size_t f = 1; f < s.R.size(); ++f) {
        REQUIRE(rot_err(s.R[f], truth.R[f]) < 1e-8);
        REQUIRE(ld::norm(s.t[f] - truth.t[f] * k) < 1e-8);
    }
    for (std::size_t p = 0; p < s.X.size(); ++p)
        REQUIRE(ld::norm(s.X[p] - truth.X[p] * k) < 1e-7);
}

TEST_CASE("bundle adjustment with Huber weights resists a gross outlier", "[sdk][sfm][bundle-adjust]") {
    const auto truth = scene();
    auto obs = truth.obs;
    obs[3 * truth.X.size() + 5].x = obs[3 * truth.X.size() + 5].x + Vec2{{0.08, -0.05}};  // ~40 px off

    auto run = [&](T huber) {
        auto s = truth;
        perturb(s);
        sfm::BundleAdjustOptions<T> opt;
        opt.huber = huber;
        const auto r = sfm::bundle_adjust<T>(s.R, s.t, s.X, obs, hold_first(s.R.size()), opt);
        REQUIRE(r.success);
        const T k = ld::norm(s.t[1]) / ld::norm(truth.t[1]);
        T worst = 0;
        for (std::size_t f = 1; f < s.R.size(); ++f)
            worst = std::max(worst, ld::norm(s.t[f] - truth.t[f] * k));
        return worst;
    };
    const T plain = run(0.0);
    const T robust = run(2e-3);  // ~1 px at EuRoC's focal length
    REQUIRE(robust < plain);
    REQUIRE(robust < 1e-3);
}

TEST_CASE("bundle adjustment holds a fully fixed window and rejects bad input", "[sdk][sfm][bundle-adjust]") {
    auto s = scene(3);
    perturb(s);
    const auto before = s;
    const std::vector<char> all(s.R.size(), 1);
    const auto r = sfm::bundle_adjust<T>(s.R, s.t, s.X, s.obs, all);
    REQUIRE(r.success);
    REQUIRE(r.iterations == 0);
    for (std::size_t f = 0; f < s.R.size(); ++f)
        REQUIRE(rot_err(s.R[f], before.R[f]) == 0.0);

    // An observation of a point that does not exist.
    auto bad = s.obs;
    bad[0].point = s.X.size();
    REQUIRE_FALSE(sfm::bundle_adjust<T>(s.R, s.t, s.X, bad, hold_first(s.R.size())).success);
}
