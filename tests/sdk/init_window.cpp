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

/// The planted window: a moving body (frame 0 at the origin, gravity-aligned
/// world), landmark observations and metric preintegration, plus the truth.
struct Planted {
    Vec3 g_world;
    std::size_t n = 0;
    std::vector<SO3> R;
    std::vector<Vec3> p, v;
    std::vector<sfm::InitFrame<T>> frames;
};

Planted planted() {
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
            // Gravity-free specific-force preintegration, matching
            // ImuPreintegrator/predict(): Δv = Rᵢᵀ(Δv_world − g Δt),
            // Δp = Rᵢᵀ(Δp_world − vᵢ Δt − ½ g Δt²). (Earlier this used +g, which
            // matched a sign bug in solve_alignment and hid the gravity
            // inversion — #247.)
            const Mat3 Rit = R[f - 1].inverse().matrix();
            frames[f].dR = R[f - 1].inverse() * R[f];
            frames[f].dv = Rit * (v[f] - v[f - 1] - g_world * dt);
            frames[f].dp = Rit * (p[f] - p[f - 1] - v[f - 1] * dt - g_world * (0.5 * dt * dt));
            frames[f].dR_dbg = Mat3::identity();
            frames[f].dt = dt;
        }
    }

    return Planted{g_world, n, R, p, v, frames};
}

}  // namespace

TEST_CASE("init window drives try_dynamic to scale, gravity, and velocities", "[sdk][sfm][init-window]") {
    const auto [g_world, n, R, p, v, frames] = planted();
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

TEST_CASE("init window composes preintegration into spaced keyframes", "[sdk][sfm][init-window]") {
    // Frames 0.1 s apart, keyframes >= 0.2 s apart: frames 0, 2, 4, 6 and the
    // last (7) become keyframes, each carrying the composed preintegration since
    // the previous one; the alignment on them recovers the same metric scale.
    const auto pl = planted();
    sfm::InitWindowOptions<T> opt;
    opt.min_keyframe_dt = 0.2;
    const auto win = sfm::build_init_window<T>(pl.frames, {}, opt);
    REQUIRE(win.success);
    REQUIRE(win.keyframe_frames == std::vector<std::size_t>{0, 2, 4, 6, 7});
    REQUIRE(win.keyframes.size() == 5);
    REQUIRE_THAT(win.keyframes[1].dt, Catch::Matchers::WithinAbs(0.2, 1e-12));
    REQUIRE_THAT(win.keyframes[4].dt, Catch::Matchers::WithinAbs(0.1, 1e-12));
    // ΔR composes to the relative rotation between the keyframes.
    const SO3 dR_true = pl.R[2].inverse() * pl.R[4];
    REQUIRE(ld::norm((win.keyframes[2].dR.inverse() * dR_true).log()) < 1e-12);

    const auto r = bs::ImuInitializer<T>{}.try_dynamic(win.keyframes);
    REQUIRE(r.success);
    const Vec3 vis = win.keyframes[1].p_world_imu - win.keyframes[0].p_world_imu;
    REQUIRE_THAT(r.scale * ld::norm(vis), Catch::Matchers::WithinRel(ld::norm(pl.p[2]), 0.05));
}

TEST_CASE("init window gates come from the track noise (#500)", "[sdk][sfm][init-window]") {
    const T sigma = 1.0 / 458.0;  // 1 px at EuRoC's focal length
    const auto o = sfm::init_window_options_for_noise<T>(sigma);
    REQUIRE_THAT(o.two_view.sampson_threshold, Catch::Matchers::WithinRel(2.576 * sigma, 1e-12));  // √χ²₁(0.99)
    REQUIRE_THAT(o.pnp.reprojection_threshold, Catch::Matchers::WithinRel(3.035 * sigma, 1e-12));  // √χ²₂(0.99)
    REQUIRE_THAT(o.min_bootstrap_parallax, Catch::Matchers::WithinRel(20.0 * sigma, 1e-12));
    REQUIRE(o.two_view.confidence > 0.0);
    REQUIRE(o.refine);
    // The defaults keep the fixed gates and build no refinement.
    const sfm::InitWindowOptions<T> d;
    REQUIRE_FALSE(d.refine);
    REQUIRE(d.min_bootstrap_parallax == 0.0);
}

TEST_CASE("init window declines a bootstrap without translational parallax (#500)", "[sdk][sfm][init-window]") {
    // The body only rotates: the frames have raw disparity but no baseline.
    auto pl = planted();
    const auto X = landmarks();
    for (std::size_t f = 0; f < pl.n; ++f) {
        pl.frames[f].ids.clear();
        pl.frames[f].obs.clear();
        const Mat3 Rt = pl.R[f].inverse().matrix();
        for (std::size_t m = 0; m < X.size(); ++m) {
            const Vec3 Xc = Rt * X[m];  // the camera stays at the origin
            if (!(Xc[2] > 0.0))
                continue;
            pl.frames[f].ids.push_back(static_cast<std::uint64_t>(m));
            pl.frames[f].obs.push_back(Vec2{{Xc[0] / Xc[2], Xc[1] / Xc[2]}});
        }
    }
    const auto win = sfm::build_init_window<T>(pl.frames, {}, sfm::init_window_options_for_noise<T>(1e-3));
    REQUIRE_FALSE(win.success);
    REQUIRE(win.status == sfm::InitWindowStatus::NoBootstrapPair);
    REQUIRE(win.bootstrap_parallax > 20 * 1e-3);  // the raw disparity passes the floor…
}

TEST_CASE("init window refinement lowers a noisy window's track error (#501)", "[sdk][sfm][init-window]") {
    // Deterministic noise of amplitude 1e-3 (normalized, ~0.5 px) on every
    // observation.
    auto pl = planted();
    std::uint32_t k = 0;
    for (auto& f : pl.frames)
        for (auto& o : f.obs) {
            o[0] += 1e-3 * std::sin(1.7 * static_cast<T>(k++));
            o[1] += 1e-3 * std::cos(2.3 * static_cast<T>(k++));
        }
    // The camera track's error after a scale-and-offset fit to the truth,
    // relative to its spread. The window's world is camera 0 (camera == body
    // here), so the truth is expressed in frame 0 first.
    std::vector<Vec3> truth(pl.n);
    for (std::size_t i = 0; i < pl.n; ++i)
        truth[i] = pl.R[0].inverse() * (pl.p[i] - pl.p[0]);
    const auto track_error = [&](const sfm::InitWindowResult<T>& w) {
        Vec3 me{}, mt{};
        for (std::size_t i = 0; i < pl.n; ++i) {
            me = me + w.keyframes[i].p_world_imu;
            mt = mt + truth[i];
        }
        me = me * (1.0 / static_cast<T>(pl.n));
        mt = mt * (1.0 / static_cast<T>(pl.n));
        T se = 0, st = 0;
        for (std::size_t i = 0; i < pl.n; ++i) {
            se += ld::dot(w.keyframes[i].p_world_imu - me, w.keyframes[i].p_world_imu - me);
            st += ld::dot(truth[i] - mt, truth[i] - mt);
        }
        const T s = std::sqrt(st / se);
        T e = 0;
        for (std::size_t i = 0; i < pl.n; ++i) {
            const Vec3 d = (w.keyframes[i].p_world_imu - me) * s - (truth[i] - mt);
            e += ld::dot(d, d);
        }
        return std::sqrt(e / st);
    };
    // Gates for twice the noise amplitude: PnP's 3σ gate does not model the
    // bootstrap landmarks' own error, and at the noise itself it rejects frame 1.
    auto opt = sfm::init_window_options_for_noise<T>(2e-3);
    opt.refine = false;
    const auto raw = sfm::build_init_window<T>(pl.frames, {}, opt);
    opt.refine = true;
    const auto refined = sfm::build_init_window<T>(pl.frames, {}, opt);
    REQUIRE(raw.success);
    REQUIRE(refined.success);
    REQUIRE(refined.refined_landmarks >= raw.landmarks);
    REQUIRE(refined.refine_rms_after < refined.refine_rms_before);
    CAPTURE(track_error(raw), track_error(refined));
    REQUIRE(track_error(refined) < 0.5 * track_error(raw));
}

TEST_CASE("init window composes the accelerometer-bias Jacobians (#501)", "[sdk][sfm][init-window]") {
    // Per frame: ∂Δv/∂ba = −Δt·I and ∂Δp/∂ba = −½Δt²·I (constant attitude).
    // Composed over two 0.1 s frames with ΔR = I they must be −0.2·I and −0.02·I,
    // as for one 0.2 s interval.
    auto pl = planted();
    for (std::size_t f = 1; f < pl.n; ++f) {
        pl.frames[f].dR = SO3{};
        pl.frames[f].dv_dba = Mat3::identity() * (-0.1);
        pl.frames[f].dp_dba = Mat3::identity() * (-0.005);
    }
    sfm::InitWindowOptions<T> opt;
    opt.min_keyframe_dt = 0.2;
    const auto win = sfm::build_init_window<T>(pl.frames, {}, opt);
    REQUIRE(win.success);
    REQUIRE_THAT(win.keyframes[1].dv_dba(0, 0), Catch::Matchers::WithinAbs(-0.2, 1e-12));
    REQUIRE_THAT(win.keyframes[1].dp_dba(1, 1), Catch::Matchers::WithinAbs(-0.02, 1e-12));
    REQUIRE(win.keyframes[1].dp_dba(0, 1) == 0.0);
}
