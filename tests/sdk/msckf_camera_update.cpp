// MSCKF camera-updater tests (issue #44).
//
// Builds a small sliding window of clones at known poses, places synthetic
// world features in front of the cameras, and generates the exact
// normalized observations. Checks that one update reduces the covariance
// while keeping it positive semidefinite, that a too-short or behind-the-
// camera track is rejected, and that the filter stays stable with a
// well-conditioned innovation over 1000 consecutive updates.

#include <branes/sdk/msckf.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace {

namespace ms = branes::sdk::msckf;
using T = double;
using Vec3 = ms::CameraUpdater<T>::Vec3;
using Vec2 = ms::CameraUpdater<T>::Vec2;
using SO3 = ms::CameraUpdater<T>::SO3;

T trace(const ms::DynMat<T>& m) {
    T s = 0;
    for (std::size_t i = 0; i < m.rows; ++i)
        s += m(i, i);
    return s;
}

// Build a state with clones at the given world poses (identity extrinsics,
// camera == IMU). Propagation between augmentations gives a full-rank P.
ms::State<T> make_state_with_clones(const std::vector<SO3>& R, const std::vector<Vec3>& p) {
    ms::State<T> s(0.3);
    ms::Propagator<T> prop;
    for (std::size_t i = 0; i < R.size(); ++i) {
        s.R = R[i];
        s.p = p[i];
        ms::StateHelper<T>::augment_clone(s);
        for (int k = 0; k < 10; ++k)
            prop.propagate(s, Vec3{{0.01, -0.005, 0.008}}, Vec3{{0.05, 0.02, 9.81}}, 0.01);
    }
    return s;
}

// Exact normalized observation of world point `f` from clone `i`
// (identity extrinsics). Returns false if the point is behind the camera.
bool observe(const ms::State<T>& s, std::size_t i, const Vec3& f, Vec2& xy) {
    const auto& cl = s.clones[i];
    const Vec3 pc = cl.R.inverse() * (f - cl.p);
    if (!(pc[2] > T{0}))
        return false;
    xy = Vec2{{pc[0] / pc[2], pc[1] / pc[2]}};
    return true;
}

ms::CameraUpdater<T> make_updater() {
    std::vector<ms::CameraExtrinsics<T>> cams(1);  // identity: camera == IMU
    return ms::CameraUpdater<T>(cams);
}

ms::FeatureTrack<T> make_track(const ms::State<T>& s, const Vec3& f) {
    ms::FeatureTrack<T> tr;
    for (std::size_t i = 0; i < s.clones.size(); ++i) {
        Vec2 xy;
        if (observe(s, i, f, xy))
            tr.observations.push_back({i, 0, xy});
    }
    return tr;
}

}  // namespace

TEST_CASE("a camera update reduces covariance and keeps it PSD", "[sdk][msckf][camera]") {
    // Four clones along x, all looking down +z, with a feature in front.
    std::vector<SO3> R(4);
    std::vector<Vec3> p = {{{0.0, 0.0, 0.0}}, {{0.5, 0.0, 0.0}}, {{1.0, 0.0, 0.0}}, {{1.5, 0.0, 0.0}}};
    auto s = make_state_with_clones(R, p);
    const Vec3 f{{0.7, 0.1, 4.0}};

    auto upd = make_updater();
    const auto track = make_track(s, f);
    REQUIRE(track.observations.size() == 4);

    const T tr_before = trace(s.covariance());
    REQUIRE(upd.update(s, track));
    REQUIRE(ms::is_positive_semidefinite(s.covariance()));
    REQUIRE(trace(s.covariance()) <= tr_before + 1e-9);
}

TEST_CASE("a single-observation track is rejected", "[sdk][msckf][camera]") {
    std::vector<SO3> R(2);
    std::vector<Vec3> p = {{{0.0, 0.0, 0.0}}, {{0.5, 0.0, 0.0}}};
    auto s = make_state_with_clones(R, p);
    auto upd = make_updater();

    ms::FeatureTrack<T> tr;
    Vec2 xy;
    REQUIRE(observe(s, 0, Vec3{{0.2, 0.0, 3.0}}, xy));
    tr.observations.push_back({0, 0, xy});  // only one observation
    REQUIRE_FALSE(upd.update(s, tr));
}

TEST_CASE("a behind-the-camera track is rejected", "[sdk][msckf][camera]") {
    std::vector<SO3> R(2);
    std::vector<Vec3> p = {{{0.0, 0.0, 0.0}}, {{0.5, 0.0, 0.0}}};
    auto s = make_state_with_clones(R, p);
    auto upd = make_updater();

    // Fabricate observations for a point behind the cameras (z < 0): the
    // updater must triangulate it behind and drop the track.
    ms::FeatureTrack<T> tr;
    tr.observations.push_back({0, 0, Vec2{{0.1, 0.0}}});
    tr.observations.push_back({1, 0, Vec2{{-0.1, 0.0}}});
    // Point them so the rays converge behind the cameras.
    tr.observations[0].xy = Vec2{{-0.2, 0.0}};
    tr.observations[1].xy = Vec2{{0.2, 0.0}};
    // Either triangulation fails or the projection is behind ⇒ no update.
    const bool applied = upd.update(s, tr);
    REQUIRE_FALSE(applied);
}

TEST_CASE("the filter stays stable over 1000 updates", "[sdk][msckf][camera]") {
    // A fixed, well-baselined window of five clones looking down +z.
    std::vector<SO3> R(5);
    std::vector<Vec3> p = {
        {{0.0, 0.0, 0.0}}, {{0.4, 0.0, 0.0}}, {{0.8, 0.0, 0.0}}, {{1.2, 0.0, 0.0}}, {{1.6, 0.0, 0.0}}};
    auto s = make_state_with_clones(R, p);
    auto upd = make_updater();

    // A handful of features spread across the field of view.
    const std::vector<Vec3> feats = {{{0.4, 0.2, 3.0}}, {{0.9, -0.3, 5.0}}, {{1.3, 0.4, 4.0}}, {{0.7, -0.1, 6.0}}};

    T tr_prev = trace(s.covariance());
    std::size_t applied = 0;
    for (int step = 0; step < 1000; ++step) {
        const Vec3& f = feats[static_cast<std::size_t>(step) % feats.size()];
        const auto track = make_track(s, f);
        if (upd.update(s, track))
            ++applied;
        // Innovation stays well-conditioned ⇒ covariance never goes
        // indefinite and never blows up. With no propagation between
        // updates, every step only removes information, so the trace is
        // monotonically non-increasing — assert that per step.
        REQUIRE(ms::is_positive_semidefinite(s.covariance()));
        const T tr = trace(s.covariance());
        REQUIRE(std::isfinite(tr));
        REQUIRE(tr <= tr_prev + 1e-9);
        REQUIRE(tr > 0.0);
        tr_prev = tr;
    }
    // The consistent (zero-residual) measurements should keep being
    // accepted by the Mahalanobis gate throughout.
    REQUIRE(applied == 1000);
}

TEST_CASE("S5 parallax gate rejects low-parallax triangulations", "[sdk][msckf][camera]") {
    using Opts = ms::CameraUpdater<T>::Options;
    const std::vector<ms::CameraExtrinsics<T>> cams(1);  // identity extrinsics

    // Low parallax: 6 cm baseline, feature 5 m ahead → ~0.7°, below the 1° gate
    // but above the numerical (Cholesky) breakdown.
    {
        const std::vector<SO3> R(2);
        const std::vector<Vec3> p = {{{0.0, 0.0, 0.0}}, {{0.06, 0.0, 0.0}}};
        auto s = make_state_with_clones(R, p);
        const Vec3 f{{0.0, 0.0, 5.0}};
        const auto tr = make_track(s, f);
        REQUIRE(tr.observations.size() == 2);
        Vec3 pf;
        Opts gated;
        gated.min_parallax_deg = 1.0;
        REQUIRE_FALSE(ms::CameraUpdater<T>(cams, gated).triangulate(s, tr.observations, pf));
        Opts off;
        off.min_parallax_deg = 0.0;  // gate disabled → the (ill-conditioned) solve still runs
        REQUIRE(ms::CameraUpdater<T>(cams, off).triangulate(s, tr.observations, pf));
    }

    // Sufficient parallax: 1.5 m baseline, ~4 m ahead → ~20°, accepted by the gate.
    {
        const std::vector<SO3> R(2);
        const std::vector<Vec3> p = {{{0.0, 0.0, 0.0}}, {{1.5, 0.0, 0.0}}};
        auto s = make_state_with_clones(R, p);
        const Vec3 f{{0.7, 0.1, 4.0}};
        const auto tr = make_track(s, f);
        Vec3 pf;
        Opts gated;
        gated.min_parallax_deg = 1.0;
        REQUIRE(ms::CameraUpdater<T>(cams, gated).triangulate(s, tr.observations, pf));
    }
}

TEST_CASE("S6d gate line: the chi-square quantile by default, 5 per dof on request", "[sdk][msckf][camera][gate]") {
    // #481: the shipped gate is the chi-square quantile at alpha = 0.05, so a
    // consistent inlier passes with probability 0.95 whatever its dof. The old
    // line, 5 per dof, sat far above the quantile at low and at high dof.
    const std::vector<ms::CameraExtrinsics<T>> cams(1);
    using Opts = ms::CameraUpdaterOptions<T>;
    const ms::CameraUpdater<T> shipped(cams, Opts{});
    REQUIRE(shipped.options().gate_rule == ms::GateRule::chi2_quantile);
    REQUIRE(std::abs(shipped.gate_threshold(1) - 3.841459) < 1e-5);
    REQUIRE(std::abs(shipped.gate_threshold(3) - 7.814728) < 1e-5);
    REQUIRE(std::abs(shipped.gate_threshold(21) - 32.670573) < 1e-5);

    Opts strict;
    strict.gate_alpha = 0.01;
    REQUIRE(std::abs(ms::CameraUpdater<T>(cams, strict).gate_threshold(3) - 11.344867) < 1e-5);

    Opts legacy;
    legacy.gate_rule = ms::GateRule::per_dof;
    const ms::CameraUpdater<T> per_dof(cams, legacy);
    REQUIRE(per_dof.gate_threshold(1) == 5.0);
    REQUIRE(per_dof.gate_threshold(21) == 105.0);

    Opts off;
    off.enable_gating = false;
    REQUIRE(ms::CameraUpdater<T>(cams, off).gate_threshold(3) == 0.0);

    Opts bad;
    bad.gate_alpha = 0.0;
    REQUIRE_THROWS_AS(ms::CameraUpdater<T>(cams, bad), std::invalid_argument);
    bad.gate_alpha = 1.0;
    REQUIRE_THROWS_AS(ms::CameraUpdater<T>(cams, bad), std::invalid_argument);
    bad.gate_alpha = 1e-17;  // 1 - alpha rounds to 1: refused at construction, not mid-run
    REQUIRE_THROWS_AS(ms::CameraUpdater<T>(cams, bad), std::invalid_argument);
}
