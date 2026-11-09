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
//     0.5 px over a clone window), not from what S2 achieves: the shipped S2
//     (both samples, RK4, #616) meets it at second order; the pre-#616 held
//     sample misses it at first order, and the two errors a held sample makes
//     (the input's skew, the integrator's start attitude) are pinned apart;
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

// The S2 schemes the closure compares. The pre-#616 step is reproduced here,
// one Euler step on a held sample with the specific force rotated at the start
// attitude, so the test can pin that it misses the bound: S2 no longer ships it.
enum class Scheme {
    euler_end,      // pre-#616 MsckfBackend: the sample that ends the interval, held
    euler_start,    // pre-#616 InvariantVioBackend: the sample that starts it, held
    euler_average,  // the #616 probe: the two samples' average, held, one Euler step
    rk4_average,    // the two samples' average, held, through the shipped RK4
    shipped,        // the shipped S2: both samples, interpolated, RK4 (#616)
};

void euler_step(ms::State<T>& st, const Vec3& w, const Vec3& f, double dt) {
    const Vec3 g{{0.0, 0.0, -9.81}};
    const Vec3 a = st.R * f + g;
    st.p = st.p + st.v * dt + a * (0.5 * dt * dt);
    st.v = st.v + a * dt;
    st.R = st.R * SO3::exp(w * dt);
}

// Integrate a window of noise-free samples at `rate` Hz with `scheme`, seeded
// from the spline at the first sample, and return the {rotation, position}
// error against the spline at the last.
std::pair<double, double> closure(const Spline& s, double rate, Scheme scheme) {
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
        const Vec3 w0 = vec(imu[k - 1].angular_velocity), f0 = vec(imu[k - 1].linear_acceleration);
        const Vec3 w1 = vec(imu[k].angular_velocity), f1 = vec(imu[k].linear_acceleration);
        const double dt = imu[k].timestamp_s - imu[k - 1].timestamp_s;
        switch (scheme) {
        case Scheme::euler_end:
            euler_step(st, w1, f1, dt);
            break;
        case Scheme::euler_start:
            euler_step(st, w0, f0, dt);
            break;
        case Scheme::euler_average:
            euler_step(st, (w0 + w1) * 0.5, (f0 + f1) * 0.5, dt);
            break;
        case Scheme::rk4_average:
            (void)ms::stages::s2_propagation::apply(st, prop, (w0 + w1) * 0.5, (f0 + f1) * 0.5, dt);
            break;
        case Scheme::shipped:
            (void)ms::stages::s2_propagation::apply(st, prop, ms::ImuInterval<T>{w0, f0, w1, f1, dt});
            break;
        }
    }
    const SE3 P1 = s.pose(imu.back().timestamp_s);
    return {norm((P1.rotation().inverse() * st.R).log()), norm(st.p - P1.translation())};
}
}  // namespace

// #616's acceptance test: the shipped S2, fed both samples of each interval as
// both backends now do, closes on the fast flight within what the camera
// resolves, in rotation and in position, and converges at second order. Second,
// not fourth: RK4 integrates the line between the samples to fourth order, but
// the line itself follows the true rate only to second order, and that is the
// error left (about a quarter of the bound in rotation, a tenth in position).
TEST_CASE("gt spline: S2 closure, the shipped S2 within what the camera resolves (#616)", "[tools][gt_spline]") {
    const Spline s(fast_sampled(), 0.05);
    const auto [rot, pos] = closure(s, 200.0, Scheme::shipped);
    const auto [rot2, pos2] = closure(s, 400.0, Scheme::shipped);
    CAPTURE(rot, rot2, pos, pos2, kRotBound, kPosBound);
    CHECK(rot < kRotBound);
    CHECK(pos < kPosBound);
    CHECK(rot / rot2 > 3.5);  // second order: a quarter at half the step
    CHECK(pos / pos2 > 3.5);
}

// The pre-#616 S2, pinned: one Euler step on a held sample, the end sample
// (MsckfBackend) or the start sample (InvariantVioBackend). Either hold is a
// half-sample IMU-camera skew, which misses the rotation bound about 28x and
// converges at first order: the error halves with the step.
TEST_CASE("gt spline: S2 closure, a held sample misses the rotation bound at first order", "[tools][gt_spline]") {
    const Spline s(fast_sampled(), 0.05);
    for (const Scheme scheme : {Scheme::euler_end, Scheme::euler_start}) {
        const auto [rot, pos] = closure(s, 200.0, scheme);
        const auto [rot2, pos2] = closure(s, 400.0, scheme);
        CAPTURE(static_cast<int>(scheme), rot, rot2, pos, pos2, kRotBound, kPosBound);
        CHECK(rot > 20.0 * kRotBound);
        CHECK(pos > kPosBound);
        CHECK(rot / rot2 > 1.8);
        CHECK(rot / rot2 < 2.2);
    }
}

// The two errors a held sample makes, separated. Centring the input (the
// average of the interval's two samples) removes the rotation error. The
// position error is the integrator's: one Euler step rotates the specific force
// with the attitude at the start of the interval, which misses the position
// bound and converges at first order; RK4 rotates it along the interval and
// meets the bound. The shipped S2 interpolates rather than averaging: the same
// order, a little closer (tests above).
TEST_CASE("gt spline: S2 closure, centring fixes the rotation and the integrator the position", "[tools][gt_spline]") {
    const Spline s(fast_sampled(), 0.05);
    const auto [rot_e, pos_e] = closure(s, 200.0, Scheme::euler_average);
    const auto [rot_e2, pos_e2] = closure(s, 400.0, Scheme::euler_average);
    CAPTURE(rot_e, pos_e, pos_e2, kRotBound, kPosBound);
    CHECK(rot_e < kRotBound);
    CHECK(pos_e > 10.0 * kPosBound);
    CHECK(pos_e / pos_e2 > 1.8);
    CHECK(pos_e / pos_e2 < 2.2);

    const auto [rot_r, pos_r] = closure(s, 200.0, Scheme::rk4_average);
    CAPTURE(rot_r, pos_r);
    CHECK(rot_r < kRotBound);
    CHECK(pos_r < kPosBound);
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
