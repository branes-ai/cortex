// SPDX-License-Identifier: MIT
//
// The consistency gate's smooth truth and its synthesized IMU (#613,
// branes/tools/gt_spline.hpp; nodes G1, G2 and the s2_closure invariant of
// docs/methodology/specs/euroc-consistency-gate.yaml), on a smooth analytic
// "ground truth" sampled at 200 Hz:
//
//   * the spline's analytic velocity, acceleration and body angular velocity
//     match finite differences of its own pose;
//   * it stays close to the ground truth it was built from;
//   * s2_closure, with a bound taken from what the camera resolves (a tenth of
//     0.5 px over a clone window), not from what S2 achieves: averaging each
//     interval's two samples meets it for rotation; the shipped S2 does not
//     (#616), and a [!shouldfail] tripwire records that until it does;
//   * the injected noise has the configured densities and is white, and the
//     biases walk at the configured rate.
//
// ASCII-only TEST_CASE names (the Windows MSVC ctest job rejects non-ASCII).

#include <branes/math/lie/se3.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/stages/s2_propagation.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/tools/gt_spline.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace {
namespace bt = branes::tools;
namespace ms = branes::sdk::msckf;
namespace lie = branes::math::lie;
using T = double;
using Vec3 = lie::detail::Vec<T, 3>;
using SO3 = lie::SO3<T>;
using SE3 = lie::SE3<T>;
using Spline = bt::PoseSpline<T>;

// A smooth flight: a slow loop with a climb, turning and rocking as it goes.
SE3 truth(double t) {
    const Vec3 p{{1.5 * std::sin(0.6 * t), 1.0 * std::cos(0.45 * t), 0.2 * t + 0.1 * std::sin(1.3 * t)}};
    const Vec3 r{{0.15 * std::sin(0.9 * t), 0.1 * std::cos(0.7 * t), 0.5 * t}};
    return SE3(SO3::exp(r), p);
}

std::vector<branes::sdk::eval::StampedPose<T>> sampled(double t_end, double rate = 200.0) {
    std::vector<branes::sdk::eval::StampedPose<T>> gt;
    for (double t = 0.0; t <= t_end; t += 1.0 / rate)
        gt.push_back({t, truth(t)});
    return gt;
}

double norm(const Vec3& v) {
    return lie::detail::norm(v);
}
}  // namespace

TEST_CASE("gt spline: velocity, acceleration and body rate are the derivatives of its pose", "[tools][gt_spline]") {
    const Spline s(sampled(10.0), 0.05);
    const double h = 1e-4;
    double worst_v = 0, worst_a = 0, worst_w = 0;
    for (double t = s.t_begin() + 0.01; t < s.t_end() - 0.01; t += 0.137) {
        const SE3 Pm = s.pose(t - h), P0 = s.pose(t), Pp = s.pose(t + h);
        const Vec3 v_fd = (Pp.translation() - Pm.translation()) * (1.0 / (2 * h));
        const Vec3 a_fd = (Pp.translation() - P0.translation() * 2.0 + Pm.translation()) * (1.0 / (h * h));
        const Vec3 w_fd = (Pm.rotation().inverse() * Pp.rotation()).log() * (1.0 / (2 * h));
        worst_v = std::max(worst_v, norm(s.velocity(t) - v_fd));
        worst_a = std::max(worst_a, norm(s.acceleration(t) - a_fd));
        worst_w = std::max(worst_w, norm(s.angular_velocity(t) - w_fd));
    }
    CHECK(worst_v < 1e-6);
    CHECK(worst_a < 1e-3);  // a second difference at h = 1e-4 carries ~1e-8 / h^2 of rounding
    CHECK(worst_w < 1e-6);
}

TEST_CASE("gt spline: it stays close to the ground truth it was built from", "[tools][gt_spline]") {
    const auto gt = sampled(10.0);
    const Spline s(gt, 0.05);
    double worst_p = 0, worst_r = 0;
    for (const auto& g : gt) {
        if (!s.contains(g.t_s))
            continue;
        const SE3 P = s.pose(g.t_s);
        worst_p = std::max(worst_p, norm(P.translation() - g.pose.translation()));
        worst_r = std::max(worst_r, norm((g.pose.rotation().inverse() * P.rotation()).log()));
    }
    // A cubic B-spline smooths by about knot_dt^2 / 6 times the second derivative.
    CHECK(worst_p < 1e-3);
    CHECK(worst_r < 1e-3);
    CHECK(s.t_begin() == 0.05);
    CHECK_THROWS(Spline(sampled(0.1), 0.05));  // fewer than four knots
}

namespace {
// A fast flight: rolling and pitching at about 1.3 rad/s, EuRoC V2_03's 95th
// percentile, where a sample held over the wrong interval shows (#616).
SE3 fast_truth(double t) {
    const Vec3 p{{1.5 * std::sin(0.8 * t), 1.0 * std::cos(0.6 * t), 0.3 * std::sin(1.5 * t)}};
    const Vec3 r{{0.3 * std::sin(4.3 * t), 0.26 * std::cos(5.0 * t), 0.5 * t}};
    return SE3(SO3::exp(r), p);
}

// The S2-closure bound, from what the camera resolves, not from what S2
// achieves: over a clone window (11 frames at 20 Hz, 0.55 s), S2's error must
// stay under a tenth of the gate's finest track noise, 0.5 px at cam0's
// fx = 458.65, i.e. 1.09e-4 rad of rotation, and 0.22 mm of position at the
// nearest landmark depth (2 m).
std::vector<branes::sdk::eval::StampedPose<T>> fast_sampled() {
    std::vector<branes::sdk::eval::StampedPose<T>> gt;
    for (double t = 0.0; t <= 10.0; t += 0.005)
        gt.push_back({t, fast_truth(t)});
    return gt;
}

constexpr double kWindowS = 0.55;
constexpr double kRotBound = 0.1 * 0.5 / 458.654;
constexpr double kPosBound = 2.0 * kRotBound;

enum class Hold { end_sample, both_samples };

// Integrate a window of noise-free samples at `rate` Hz with the shipped S2,
// seeded from the spline at the first sample, and return the {rotation,
// position} error against the spline at the last. end_sample feeds each
// interval the sample that ends it (MsckfBackend today); both_samples feeds it
// the average of its two samples.
std::pair<double, double> closure(const Spline& s, double rate, Hold hold) {
    std::vector<double> times;
    for (int k = 0; k <= static_cast<int>(std::lround(kWindowS * rate)); ++k)
        times.push_back(2.0 + k / rate);
    const auto imu = bt::synthesize_imu<T>(s, times, bt::ImuSynthesisOptions{});
    REQUIRE(imu.size() == times.size());
    ms::State<T> st(0.1);
    const SE3 P0 = s.pose(imu.front().timestamp_s);
    st.R = P0.rotation();
    st.p = P0.translation();
    st.v = s.velocity(imu.front().timestamp_s);
    const ms::Propagator<T> prop;
    const auto vec = [](const auto& a) { return Vec3{{a[0], a[1], a[2]}}; };
    for (std::size_t k = 1; k < imu.size(); ++k) {
        Vec3 w = vec(imu[k].angular_velocity), f = vec(imu[k].linear_acceleration);
        if (hold == Hold::both_samples) {
            w = (w + vec(imu[k - 1].angular_velocity)) * 0.5;
            f = (f + vec(imu[k - 1].linear_acceleration)) * 0.5;
        }
        (void)ms::stages::s2_propagation::apply(st, prop, w, f, imu[k].timestamp_s - imu[k - 1].timestamp_s);
    }
    const SE3 P1 = s.pose(imu.back().timestamp_s);
    return {norm((P1.rotation().inverse() * st.R).log()), norm(st.p - P1.translation())};
}
}  // namespace

TEST_CASE("gt spline: S2 closure, averaging each interval's two samples, holds the rotation bound",
          "[tools][gt_spline]") {
    // The rotation bound is attainable at EuRoC's 200 Hz: the shipped S2 step,
    // fed the average of each interval's two samples, closes on a fast spline
    // within it, and converges at second order. This also checks the
    // synthesized IMU against the spline it came from. Its position still
    // drifts at first order (about 5 mm over the window): the specific force is
    // rotated with the attitude at the start of the interval, which #616's
    // integrator must fix as well.
    const Spline s(fast_sampled(), 0.05);
    const auto [rot, pos] = closure(s, 200.0, Hold::both_samples);
    const auto [rot2, pos2] = closure(s, 400.0, Hold::both_samples);
    CAPTURE(rot, rot2, pos, pos2, kRotBound, kPosBound);
    CHECK(rot < kRotBound);
    CHECK(rot / rot2 > 3.5);  // second order: a quarter at half the step
}

// What the shipped S2 does today (#616), pinned so the tripwire below can only
// turn green because S2 changed: [!shouldfail] passes on any failing check, so
// this case asserts the specific miss. On the fast flight the end-sample hold
// misses the rotation bound by about 28x and converges at first order (the
// error halves with the step). When #616 fixes S2 this case fails, and both it
// and the tripwire's tag must be updated together.
TEST_CASE("gt spline: S2 closure, the shipped S2 misses the rotation bound at first order (#616)",
          "[tools][gt_spline]") {
    const Spline s(fast_sampled(), 0.05);
    const auto [rot, pos] = closure(s, 200.0, Hold::end_sample);
    const auto [rot2, pos2] = closure(s, 400.0, Hold::end_sample);
    CAPTURE(rot, rot2, pos, pos2, kRotBound);
    CHECK(rot > 20.0 * kRotBound);
    CHECK(rot < 40.0 * kRotBound);
    CHECK(rot / rot2 > 1.8);
    CHECK(rot / rot2 < 2.2);
}

// A tripwire for #616: the shipped S2, fed each interval the sample that ends
// it as MsckfBackend does, must close within what the camera resolves, at
// second order or better. Today it misses the rotation bound 28x (a sample held
// over the wrong interval is a 2.5 ms IMU-camera skew) and converges at first
// order, so this case fails and [!shouldfail] reports that as a pass. When #616
// fixes S2 the case passes, [!shouldfail] turns that into a failure, and the
// tag must come off: the test then guards the fix.
TEST_CASE("gt spline: S2 closure, the shipped S2 within what the camera resolves (#616)",
          "[tools][gt_spline][!shouldfail]") {
    const Spline s(fast_sampled(), 0.05);
    const auto [rot, pos] = closure(s, 200.0, Hold::end_sample);
    const auto [rot2, pos2] = closure(s, 400.0, Hold::end_sample);
    CAPTURE(rot, rot2, pos, pos2, kRotBound, kPosBound);
    CHECK(rot < kRotBound);
    CHECK(pos < kPosBound);
    CHECK(rot / rot2 > 3.5);
    CHECK(pos / pos2 > 3.5);
}

TEST_CASE("gt spline: the synthesized noise has the configured densities", "[tools][gt_spline]") {
    const Spline s(sampled(60.0), 0.05);
    std::vector<double> times;
    for (double t = 1.0; t <= 59.0; t += 0.005)
        times.push_back(t);
    const auto clean = bt::synthesize_imu<T>(s, times, bt::ImuSynthesisOptions{});

    bt::ImuSynthesisOptions o;
    o.gyro_noise_density = 1.6e-4;
    o.accel_noise_density = 2.0e-3;
    o.seed = 7;
    const auto noisy = bt::synthesize_imu<T>(s, times, o);
    double vg = 0, va = 0, lag1 = 0;
    for (std::size_t k = 0; k < clean.size(); ++k) {
        const double eg = noisy[k].angular_velocity[0] - clean[k].angular_velocity[0];
        const double ea = noisy[k].linear_acceleration[1] - clean[k].linear_acceleration[1];
        vg += eg * eg;
        va += ea * ea;
        if (k > 0)
            lag1 += eg * (noisy[k - 1].angular_velocity[0] - clean[k - 1].angular_velocity[0]);
    }
    const double n = static_cast<double>(clean.size());
    const double sg2 = o.gyro_noise_density * o.gyro_noise_density / 0.005;
    const double sa2 = o.accel_noise_density * o.accel_noise_density / 0.005;
    CHECK(std::abs(vg / n / sg2 - 1.0) < 0.05);
    CHECK(std::abs(va / n / sa2 - 1.0) < 0.05);
    CHECK(std::abs(lag1 / vg) < 0.03);  // white: no correlation from sample to sample

    // Bias walk only: the gyro bias after T seconds has variance rw^2 * T per axis.
    bt::ImuSynthesisOptions w;
    w.gyro_bias_random_walk = 1.9e-5;
    double var = 0;
    const int seeds = 200;
    for (int seed = 1; seed <= seeds; ++seed) {
        w.seed = static_cast<std::uint64_t>(seed);
        const auto b = bt::synthesize_imu<T>(s, times, w);
        const double d = b.back().angular_velocity[2] - clean.back().angular_velocity[2];
        var += d * d;
    }
    const double expected = w.gyro_bias_random_walk * w.gyro_bias_random_walk * (times.back() - times.front());
    CHECK(std::abs(var / seeds / expected - 1.0) < 0.25);  // 200 draws: about 10% standard error
}
