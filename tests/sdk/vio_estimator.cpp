// VioEstimator end-to-end tests (issue #45).
//
// Exercises the lifecycle gate (measurements ignored unless Active) and a
// full front-end → backend run on synthetic frames: a high-contrast
// pattern of bright squares translating across a grayscale image is FAST-
// detected and KLT-tracked, the resulting observations drive the MSCKF
// backend, and the estimator produces a finite, stable navigation state.
// Trajectory accuracy on real EuRoC is the harness's job (#46).

#include <branes/sdk/vio_estimator.hpp>
#include <branes/test/synthetic_frames.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <universal/number/posit/posit.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <vector>

namespace {

namespace bs = branes::sdk;
using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T>;
using DVec3 = Backend::DVec3;

// Render bright 8×8 squares on a black background, on a 40px grid, shifted
// by (ox, oy) (tests/include/branes/test/synthetic_frames.hpp).
branes::cv::OwnedImage<std::uint8_t> render(double ox, double oy) {
    return branes::test::render_squares(ox, oy);
}

bs::ImuMeasurement<T> level_sample(double t) {
    return branes::test::level_sample<T>(t);
}

// A calibrated backend (so pixel→bearing unprojection is sane).
Estimator make_estimator() {
    return branes::test::make_estimator<T>();
}

template <class S>
bool finite(const S& x) {
    return std::isfinite(static_cast<double>(x));
}

}  // namespace

TEST_CASE("the estimator gates measurements on its lifecycle", "[sdk][vio]") {
    auto est = make_estimator();
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Unconfigured);

    // Feeds before configuration are ignored (no crash, no state change).
    const auto frame = render(0, 0);
    est.feed_image(0.0, frame.view());
    REQUIRE(est.num_tracked_features() == 0);

    bs::VioConfig cfg;
    cfg.max_clones = 8;
    est.configure(cfg);
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Inactive);

    // Still inactive ⇒ still ignored.
    est.feed_image(0.01, frame.view());
    REQUIRE(est.num_tracked_features() == 0);

    est.activate();
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Active);
    est.feed_image(0.02, frame.view());
    REQUIRE(est.num_tracked_features() > 0);

    // No reconfiguration while Active (ADR-0005): ignored, state kept.
    const auto tracked = est.num_tracked_features();
    est.configure(cfg);
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Active);
    REQUIRE(est.num_tracked_features() == tracked);

    est.deactivate();
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Inactive);

    est.reset();
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Inactive);
    REQUIRE(est.num_tracked_features() == 0);

    est.teardown();
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Teardown);
    est.activate();  // no-op from Teardown
    REQUIRE(est.lifecycle() == Estimator::Lifecycle::Teardown);
}

TEST_CASE("the estimator runs the front end and backend end-to-end", "[sdk][vio]") {
    auto est = make_estimator();
    bs::VioConfig cfg;
    cfg.max_clones = 8;
    est.configure(cfg);
    est.activate();

    // 1) Stationary IMU window to initialize the backend.
    double t = 0.0;
    std::vector<bs::ImuMeasurement<T>> batch;
    for (int k = 0; k < 80; ++k, t += 0.005)
        batch.push_back(level_sample(t));
    est.feed_imu(std::span<const bs::ImuMeasurement<T>>{batch});
    REQUIRE(est.backend().initialized());

    // 2) Feed translating frames interleaved with IMU. The front end
    // should detect and then persistently track the square corners.
    std::size_t max_clones_seen = 0;
    for (int f = 0; f < 20; ++f) {
        std::vector<bs::ImuMeasurement<T>> imus;
        for (int k = 0; k < 10; ++k, t += 0.005)
            imus.push_back(level_sample(t));
        est.feed_imu(std::span<const bs::ImuMeasurement<T>>{imus});

        const auto img = render(1.0 * f, 0.5 * f);
        est.feed_image(t, img.view());

        REQUIRE(est.num_tracked_features() > 0);
        // tracked_features() (for overlaying the live tracks on the scene video)
        // is consistent with the count, and every position is finite.
        const auto feats = est.tracked_features();
        REQUIRE(feats.size() == est.num_tracked_features());
        for (const auto& tf : feats) {
            REQUIRE(std::isfinite(tf.x));
            REQUIRE(std::isfinite(tf.y));
        }
        REQUIRE(branes::sdk::msckf::is_positive_semidefinite(est.backend().state().covariance()));
        max_clones_seen = std::max(max_clones_seen, est.backend().state().clones.size());

        const auto pose = est.current_pose().translation();
        for (std::size_t i = 0; i < 3; ++i)
            REQUIRE(std::isfinite(pose[i]));
    }

    // The sliding window grew but stayed bounded by max_clones.
    REQUIRE(max_clones_seen > 1);
    REQUIRE(max_clones_seen <= static_cast<std::size_t>(cfg.max_clones));

    // Features tracked across many frames (KLT actually followed them, not
    // just re-detected fresh each time).
    REQUIRE(est.num_tracked_features() > 0);
}

TEST_CASE("S4 forward-backward gate drops tracks that fail the round trip", "[sdk][vio]") {
    // Two estimators fed the same clean translating sequence: one with the FB
    // gate disabled, one so strict (1e-9 px) that any sub-pixel round-trip error
    // rejects the track. With FB disabled, tracks persist across frames (their
    // ids carry over); with the strict gate, every tracked feature is dropped and
    // re-detected fresh, so no id survives frame-to-frame.
    auto build = [](double fb) {
        bs::FrontendParams fe;
        fe.fb_max_residual = fb;
        Backend::CameraCalibration cal;
        cal.intrinsics = Backend::Camera(200.0, 200.0, 120.0, 90.0, 0.0, 0.0, 0.0, 0.0);
        return Estimator(Backend(std::vector<Backend::CameraCalibration>{cal}), fe);
    };

    auto ids_after_two_frames = [](Estimator& est) {
        bs::VioConfig cfg;
        est.configure(cfg);
        est.activate();
        auto frame_ids = [&](double t, double ox, double oy) {
            const auto img = render(ox, oy);
            est.feed_image(t, img.view());
            std::vector<std::uint32_t> ids;
            for (const auto& f : est.tracked_features())
                ids.push_back(f.id);
            std::sort(ids.begin(), ids.end());
            return ids;
        };
        const auto first = frame_ids(0.0, 0.0, 0.0);
        const auto second = frame_ids(0.05, 1.5, 1.0);  // small translation KLT can follow
        std::vector<std::uint32_t> persisted;
        std::set_intersection(first.begin(), first.end(), second.begin(), second.end(), std::back_inserter(persisted));
        return persisted.size();
    };

    auto off = build(0.0);
    auto strict = build(1e-9);
    REQUIRE(ids_after_two_frames(off) > 0);      // FB disabled → tracks persist
    REQUIRE(ids_after_two_frames(strict) == 0);  // strict FB → all dropped + re-detected
}

// Genericity end to end (#449): the same front end → backend run in every
// arithmetic type the stage benches use. The KLT front end stays IEEE inside by
// design and hands T-valued observations to a backend that runs entirely in T.
// Fewer frames than the double run above: posit32 is emulated in software.
// strict_double, bit for bit against double, is in tests/tools/strict_double.cpp
// (it lives in branes::tools, which the SDK tests do not link).
using posit32 = sw::universal::posit<32, 2>;

// The double run of the same scene, per frame: position (x, y, z).
const std::vector<std::array<double, 3>>& double_positions() {
    static const std::vector<std::array<double, 3>> ref = [] {
        auto est = branes::test::make_estimator<double>();
        bs::VioConfig cfg;
        cfg.max_clones = 5;
        est.configure(cfg);
        est.activate();
        std::vector<std::array<double, 3>> out;
        branes::test::drive_estimator(est, 8, [&](int, double) {
            const auto p = est.current_pose().translation();
            out.push_back({p[0], p[1], p[2]});
        });
        return out;
    }();
    return ref;
}

TEMPLATE_TEST_CASE(
    "the estimator runs end to end in every arithmetic type", "[sdk][vio][types]", float, posit32, long double) {
    using S = TestType;
    auto est = branes::test::make_estimator<S>();
    bs::VioConfig cfg;
    cfg.max_clones = 5;
    est.configure(cfg);
    est.activate();

    // Observations must reach the filter: count the S6 updates the backend runs.
    std::size_t updates = 0, accepted = 0;
    est.backend().set_update_observer([&](const auto&, const auto&, const auto&, bool ok, const auto&) {
        ++updates;
        accepted += ok ? 1 : 0;
    });

    std::size_t max_clones_seen = 0;
    double worst = 0.0;
    const auto& ref = double_positions();
    branes::test::drive_estimator(est, 8, [&](int f, double t) {
        REQUIRE(est.backend().initialized());
        REQUIRE(est.num_tracked_features() > 0);
        REQUIRE(est.current_state().timestamp_s == t);  // the clock stays double
        REQUIRE(branes::sdk::msckf::is_positive_semidefinite(est.backend().state().covariance()));
        max_clones_seen = std::max(max_clones_seen, est.backend().state().clones.size());
        const auto pose = est.current_pose().translation();
        for (std::size_t i = 0; i < 3; ++i)
            REQUIRE(finite(pose[i]));
        const auto& r = ref[static_cast<std::size_t>(f)];
        const double err = std::hypot(static_cast<double>(pose[0]) - r[0],
                                      static_cast<double>(pose[1]) - r[1],
                                      static_cast<double>(pose[2]) - r[2]);
        REQUIRE(std::isfinite(err));  // a NaN would slip past std::max
        worst = std::max(worst, err);
    });
    // Each type tracks the double run to within its own rounding, amplified by
    // the filter. The bound is absolute: 1000·max(ε_T, ε_double) metres, i.e.
    // 1000 ulps at the scene's 1 m unit, not ulps of each position. Positions
    // start at 0, and the error does not scale with them: it comes from rounding
    // in the covariance and gain, carried through the updates. max(·, ε_double)
    // is for long double, whose deviation is double's own rounding. Measured
    // over the 16 cm trajectory: float 1.4e-5 m (bound 1.2e-4), posit32 5.4e-7 m
    // (bound 7.5e-6), long double 9.0e-15 m (bound 2.2e-13).
    const double eps =
        std::max(static_cast<double>(std::numeric_limits<S>::epsilon()), std::numeric_limits<double>::epsilon());
    INFO("worst |p - p_double| " << worst << " m");
    REQUIRE(worst <= 1e3 * eps);
    REQUIRE(max_clones_seen > 1);
    REQUIRE(max_clones_seen <= static_cast<std::size_t>(cfg.max_clones));
    INFO("updates " << updates << ", accepted " << accepted);
    REQUIRE(accepted > 0);
}
