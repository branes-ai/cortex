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
//   * s2_closure: the noise-free synthesized IMU, integrated by the shipped
//     S2, reproduces the spline's pose over a clone window;
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

TEST_CASE("gt spline: the noise-free IMU integrated by the shipped S2 reproduces the spline", "[tools][gt_spline]") {
    const Spline s(sampled(10.0), 0.05);
    // Integrate 0.6 s (a clone window) of noise-free samples at `rate` Hz with
    // the shipped S2, seeded from the spline, the way MsckfBackend does: each
    // sample over the interval it ends. Returns {rotation, position} error.
    const auto closure = [&](double rate) {
        std::vector<double> times;
        for (int k = 0; k <= static_cast<int>(0.6 * rate); ++k)
            times.push_back(2.0 + k / rate);
        const auto imu = bt::synthesize_imu<T>(s, times, bt::ImuSynthesisOptions{});
        REQUIRE(imu.size() == times.size());
        ms::State<T> st(0.1);
        const SE3 P0 = s.pose(imu.front().timestamp_s);
        st.R = P0.rotation();
        st.p = P0.translation();
        st.v = s.velocity(imu.front().timestamp_s);
        const ms::Propagator<T> prop;
        for (std::size_t k = 1; k < imu.size(); ++k) {
            const auto& m = imu[k];
            const Vec3 w{{m.angular_velocity[0], m.angular_velocity[1], m.angular_velocity[2]}};
            const Vec3 f{{m.linear_acceleration[0], m.linear_acceleration[1], m.linear_acceleration[2]}};
            (void)ms::stages::s2_propagation::apply(st, prop, w, f, m.timestamp_s - imu[k - 1].timestamp_s);
        }
        const SE3 P1 = s.pose(imu.back().timestamp_s);
        return std::pair{norm((P1.rotation().inverse() * st.R).log()), norm(st.p - P1.translation())};
    };
    const auto [rot200, pos200] = closure(200.0);
    const auto [rot400, pos400] = closure(400.0);
    // At EuRoC's 200 Hz: about 0.14 px of rotation at most, against the gate's
    // 0.5 px tracks, and well under a millimetre over the window.
    CHECK(rot200 < 3e-4);
    CHECK(pos200 < 1e-3);
    // The residual is S2's first-order discretization, not a disagreement
    // between the synthesized IMU and the spline: it halves with the step.
    CHECK(rot200 / rot400 > 1.8);
    CHECK(rot200 / rot400 < 2.2);
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
