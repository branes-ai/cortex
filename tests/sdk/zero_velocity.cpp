// S8 zero-velocity update tests (issue #600).
//
// The stillness detector must find rest under rotor-like vibration and must
// not fire on motion its IMU test cannot see (a constant-velocity glide); the
// pseudo-measurements pull the velocity to zero and the gyro bias to the mean
// rate; and the backend, at rest with an accelerometer error it never saw at
// init, holds its velocity only with S8 on.

#include <branes/sdk/msckf/stages/s8_zero_velocity.hpp>
#include <branes/sdk/msckf_backend.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace s8 = branes::sdk::msckf::stages::s8_zero_velocity;
namespace ld = branes::math::lie::detail;
using T = double;
using Vec3 = ld::Vec<T, 3>;

/// 50 gyro samples: a bias plus a rotor-like tone near Nyquist (the low-pass
/// removes it) plus an optional slow ramp (which it keeps).
void feed_gyro(s8::Detector<T>& d, T tone, T ramp) {
    for (int k = 0; k < 50; ++k) {
        const T n = tone * std::sin(2.7 * k);
        d.push_imu(Vec3{{0.01 + n, -0.02 - n, 0.005 + ramp * k}});
    }
}

/// Six frames of 30 features on a grid, moved by `step` pixels per frame along
/// u, with deterministic sub-pixel jitter.
void feed_frames(s8::Detector<T>& d, T step, std::size_t features = 30) {
    for (int f = 0; f < 6; ++f) {
        std::vector<bs::FrontendObservation<T>> obs;
        for (std::size_t i = 0; i < features; ++i) {
            bs::FrontendObservation<T> o;
            o.feature_id = i;
            o.u = 100.0 + 20.0 * static_cast<T>(i % 6) + step * f + 0.2 * std::sin(1.3 * (i + f));
            o.v = 80.0 + 25.0 * static_cast<T>(i / 6) + 0.2 * std::cos(0.7 * (i + 2 * f));
            obs.push_back(o);
        }
        d.push_frame(obs);
    }
}

}  // namespace

TEST_CASE("S8 finds rest under vibration", "[sdk][msckf][s8]") {
    s8::Detector<T> d;
    feed_gyro(d, 0.03, 0.0);  // a per-sample spread of ~0.02 rad/s, as on EuRoC MH_03 (#591)
    feed_frames(d, 0.0);
    const auto r = d.check();
    REQUIRE(r.imu_still);
    REQUIRE(r.vision_still);
    REQUIRE(r.still);
    REQUIRE(r.features == 30);
    REQUIRE(r.disparity_px < 1.0);
    // At rest the mean rate is the gyro bias (the tone averages out).
    REQUIRE_THAT(d.mean_rate()[0], Catch::Matchers::WithinAbs(0.01, 2e-3));
}

TEST_CASE("S8 does not fire on a constant-velocity glide", "[sdk][msckf][s8]") {
    // The IMU sees nothing (no rotation, no acceleration) but the image moves
    // 2 px a frame: 10 px over the window.
    s8::Detector<T> d;
    feed_gyro(d, 0.0, 0.0);
    feed_frames(d, 2.0);
    const auto r = d.check();
    REQUIRE(r.imu_still);
    REQUIRE_FALSE(r.vision_still);
    REQUIRE_FALSE(r.still);
    REQUIRE_THAT(r.disparity_px, Catch::Matchers::WithinAbs(10.0, 0.5));
}

TEST_CASE("S8 does not fire while the rate changes", "[sdk][msckf][s8]") {
    // A slow change of rate survives the low-pass: 1 mrad/s per sample, a
    // spread of the block means of ~0.014 rad/s.
    s8::Detector<T> d;
    feed_gyro(d, 0.0, 1e-3);
    feed_frames(d, 0.0);
    const auto r = d.check();
    REQUIRE_FALSE(r.imu_still);
    REQUIRE_FALSE(r.still);
}

TEST_CASE("S8 gives no verdict without enough data", "[sdk][msckf][s8]") {
    s8::Detector<T> d;
    REQUIRE_FALSE(d.check().still);  // nothing yet
    feed_gyro(d, 0.0, 0.0);
    REQUIRE_FALSE(d.check().still);  // no frames
    feed_frames(d, 0.0, 5);          // too few shared features
    const auto r = d.check();
    REQUIRE(r.imu_still);
    REQUIRE(r.features == 5);
    REQUIRE_FALSE(r.vision_still);
    d.reset();
    REQUIRE_FALSE(d.check().still);
}

TEST_CASE("S8 pseudo-measurements pull v to zero and b_g to the mean rate", "[sdk][msckf][s8]") {
    bs::msckf::State<T> s(0.5);  // σ 0.5 on every error state
    s.v = Vec3{{0.3, -0.2, 0.1}};
    s.bg = Vec3{{0.0, 0.0, 0.0}};
    const T var_v = s.covariance()(bs::msckf::State<T>::kVel, bs::msckf::State<T>::kVel);
    (void)s8::apply(s, 0.01);
    REQUIRE(ld::norm(s.v) < 1e-3);  // σ 0.01 against a prior σ of 0.5
    REQUIRE(s.covariance()(bs::msckf::State<T>::kVel, bs::msckf::State<T>::kVel) < 1e-3 * var_v);

    const Vec3 rate{{0.004, -0.002, 0.003}};
    (void)s8::apply_gyro_bias(s, rate, 0.001);
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(s.bg[i], Catch::Matchers::WithinAbs(rate[i], 1e-5));
    REQUIRE(bs::msckf::is_positive_semidefinite(s.covariance()));
}

TEST_CASE("the backend holds its velocity at rest only with S8", "[sdk][msckf][s8]") {
    // Rest after a static init, with an accelerometer error that appears after
    // init (so the init could not absorb it into the attitude), in front of a
    // static scene. Without S8 nothing holds the velocity: the features have no
    // parallax. With it the velocity stays near zero and no clone is added.
    const auto run = [](bool zupt) {
        bs::MsckfBackend<T> backend;
        bs::VioConfig cfg;
        cfg.max_clones = 8;
        cfg.zero_velocity_update = zupt;
        cfg.zupt_max_disparity_px = 2e-3;  // identity pinhole: a pixel is a normalized unit
        backend.initialize(cfg);
        const auto imu = [](double t, const Vec3& a) {
            bs::ImuMeasurement<T> m;
            m.timestamp_s = t;
            m.angular_velocity = bs::Vec3<T>{{0.0, 0.0, 0.0}};
            m.linear_acceleration = bs::Vec3<T>{{a[0], a[1], a[2]}};
            return m;
        };
        double t = 0.0;
        for (int k = 0; k < 80; ++k, t += 0.005)
            backend.process_imu(imu(t, Vec3{{0.0, 0.0, 9.81}}));
        REQUIRE(backend.initialized());
        const std::vector<Vec3> X = {{{1.0, 0.3, 4.0}},
                                     {{-1.5, -0.2, 5.0}},
                                     {{2.0, 0.4, 6.0}},
                                     {{0.8, -0.4, 3.5}},
                                     {{-2.5, 0.1, 5.5}},
                                     {{1.2, 0.9, 4.5}},
                                     {{-0.6, -0.8, 4.2}},
                                     {{0.3, 0.6, 7.0}},
                                     {{-1.1, 0.5, 3.8}},
                                     {{1.7, -0.7, 6.4}},
                                     {{0.0, 0.0, 5.2}},
                                     {{-2.0, -0.9, 6.8}}};
        for (int step = 0; step < 600; ++step, t += 0.005) {       // 3 s
            backend.process_imu(imu(t, Vec3{{0.05, 0.0, 9.81}}));  // a 0.05 m/s² error along x
            if (step % 10 == 0) {
                std::vector<bs::FrontendObservation<T>> obs;
                for (std::size_t f = 0; f < X.size(); ++f) {  // the camera at the origin, looking +z
                    bs::FrontendObservation<T> o;
                    o.feature_id = f;
                    o.u = X[f][0] / X[f][2];
                    o.v = X[f][1] / X[f][2];
                    obs.push_back(o);
                }
                backend.process_camera(t, obs);
            }
        }
        const auto v = backend.current_state().velocity_world;
        return std::pair<T, std::size_t>{std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]),
                                         backend.zero_velocity_frames()};
    };
    const auto [v_on, n_on] = run(true);
    const auto [v_off, n_off] = run(false);
    CAPTURE(v_on, v_off, n_on);
    REQUIRE(n_off == 0);
    REQUIRE(n_on > 40);    // most of the 60 frames (the first few fill the windows)
    REQUIRE(v_on < 0.02);  // held
    REQUIRE(v_off > 0.1);  // 0.05 m/s² over 3 s, unheld
}
