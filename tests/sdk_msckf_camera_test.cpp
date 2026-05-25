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

    const T tr_before = trace(s.P);
    REQUIRE(upd.update(s, track));
    REQUIRE(ms::is_positive_semidefinite(s.P));
    REQUIRE(trace(s.P) <= tr_before + 1e-9);
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

    T tr_prev = trace(s.P);
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
        REQUIRE(ms::is_positive_semidefinite(s.P));
        const T tr = trace(s.P);
        REQUIRE(std::isfinite(tr));
        REQUIRE(tr <= tr_prev + 1e-9);
        REQUIRE(tr > 0.0);
        tr_prev = tr;
    }
    // The consistent (zero-residual) measurements should keep being
    // accepted by the Mahalanobis gate throughout.
    REQUIRE(applied == 1000);
}
