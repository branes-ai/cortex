// SPDX-License-Identifier: MIT
//
// Gate for the S2 propagation inspector (branes/tools/s2_propagation_inspect.hpp,
// issue #377). The real inspector runs over an EuRoC IMU window, which CI does not
// vendor, so here we drive S2PropagationInspector::run on an exact synthetic
// window and pin the behaviour the filter-internal figures rely on: the
// covariance grows monotonically from the seed under process noise, the mean
// strapdown integrates a known constant-acceleration window to ~½at², the
// attitude stays a unit quaternion, the diagonals stay positive (PSD sanity), and
// the drift helper measures position/attitude error in native units.

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/vio_backend.hpp>
#include <branes/tools/s2_propagation_inspect.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>

namespace bt = branes::tools;
namespace ms = branes::sdk::msckf;
namespace bs = branes::sdk;
namespace lie = branes::math::lie;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
using SO3 = lie::SO3<double>;
using Vec3 = lie::detail::Vec<double, 3>;
using ImuVec = bs::Vec3<double>;  // ImuMeasurement fields are std::array, not the lie Vec

// A level, still IMU window at 200 Hz: gyro 0, accel = -g (specific force of a
// body at rest holding station), so the strapdown integrates to zero motion and
// the only covariance change is process-noise growth.
bt::S2PropagationInspector::S2Input still_window(double duration_s, double sigma0) {
    bt::S2PropagationInspector::S2Input in;
    in.prior = ms::State<double>(sigma0);  // P0 = sigma0²·I, mean at origin, level
    const double dt = 1.0 / 200.0;
    const auto n = static_cast<std::size_t>(duration_s / dt);
    in.imu.reserve(n);
    const double t0 = 100.0;
    for (std::size_t k = 0; k < n; ++k) {
        bs::ImuMeasurement<double> m;
        m.timestamp_s = t0 + static_cast<double>(k) * dt;
        m.angular_velocity = ImuVec{{0.0, 0.0, 0.0}};
        m.linear_acceleration = ImuVec{{0.0, 0.0, 9.81}};  // specific force at rest, level
        in.imu.push_back(m);
    }
    return in;
}

}  // namespace

TEST_CASE("S2 inspector: covariance grows from the seed under process noise", "[tools][s2_inspect]") {
    const double sigma0 = 0.01;
    const auto rec = bt::S2PropagationInspector().run(still_window(0.5, sigma0));

    REQUIRE(rec.steps.size() > 2);
    REQUIRE_THAT(rec.rate_hz, WithinRel(200.0, 1e-6));
    const auto& first = rec.steps.front();
    const auto& last = rec.steps.back();

    // The first recorded step is the prior at t=0 (no propagation yet): P0 = sigma0²·I,
    // and block σ = √trace of the 3×3 block ⇒ exactly √3·sigma0 (pos in mm).
    REQUIRE_THAT(first.t, WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(first.pos_sigma_mm, WithinRel(std::sqrt(3.0) * sigma0 * 1000.0, 1e-9));
    // Process noise strictly grows the attitude/velocity/position σ over the window.
    REQUIRE(last.att_sigma_deg > first.att_sigma_deg);
    REQUIRE(last.vel_sigma_mm_s > first.vel_sigma_mm_s);
    REQUIRE(last.pos_sigma_mm > first.pos_sigma_mm);
    // Monotonic non-decreasing position σ (covariance only grows during propagation).
    for (std::size_t i = 1; i < rec.steps.size(); ++i)
        REQUIRE(rec.steps[i].pos_sigma_mm >= rec.steps[i - 1].pos_sigma_mm - 1e-9);
    // PSD sanity: every covariance diagonal stays positive.
    for (const auto& s : rec.steps)
        REQUIRE(s.min_diag > 0.0);
    // The still, level window integrates to ~no motion and a unit quaternion.
    REQUIRE_THAT(last.p[0], WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(last.p[1], WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(last.p[2], WithinAbs(0.0, 1e-6));
    const double qn =
        std::sqrt(last.q[0] * last.q[0] + last.q[1] * last.q[1] + last.q[2] * last.q[2] + last.q[3] * last.q[3]);
    REQUIRE_THAT(qn, WithinAbs(1.0, 1e-9));
}

TEST_CASE("S2 inspector: mean integrates a constant-acceleration window to ~0.5*a*t^2", "[tools][s2_inspect]") {
    // Level, still gyro; specific force = (a, 0, g) ⇒ world accel (a, 0, 0).
    auto in = still_window(1.0, 0.01);
    const double a = 2.0;
    for (auto& m : in.imu)
        m.linear_acceleration = ImuVec{{a, 0.0, 9.81}};

    const auto rec = bt::S2PropagationInspector().run(in);
    const auto& last = rec.steps.back();
    const double t = last.t;
    // x(t) = ½ a t²  (a small strapdown discretization error is expected).
    REQUIRE_THAT(last.p[0], WithinRel(0.5 * a * t * t, 0.02));
    REQUIRE_THAT(last.p[1], WithinAbs(0.0, 1e-6));
    REQUIRE_THAT(last.v[0], WithinRel(a * t, 0.02));
}

TEST_CASE("S2 inspector: drift helper measures position and attitude error", "[tools][s2_inspect]") {
    const Vec3 p_est{{0.10, 0.0, 0.0}}, p_gt{{0.0, 0.0, 0.0}};  // 100 mm apart
    const SO3 R_gt{};
    const SO3 R_est = SO3::exp(Vec3{{0.0, 0.0, 5.0 * 3.14159265358979323846 / 180.0}});  // 5° yaw

    const auto d = bt::S2PropagationInspector::drift(p_est, R_est, p_gt, R_gt);
    REQUIRE_THAT(d.pos_mm, WithinRel(100.0, 1e-9));
    REQUIRE_THAT(d.att_deg, WithinRel(5.0, 1e-6));
}

TEST_CASE("S2 inspector: record serializes to the figure schema", "[tools][s2_inspect]") {
    auto rec = bt::S2PropagationInspector().run(still_window(0.3, 0.01));
    // Attach a drift sample (as the tool would) so the GT branch serializes too.
    rec.steps.front().have_gt = true;
    rec.steps.front().pos_drift_mm = 12.3;
    rec.steps.front().pos_within_3sigma = true;

    const auto j = bt::to_json(rec);
    REQUIRE(j.at("dim") == 15);
    REQUIRE(j.at("steps").is_array());
    REQUIRE(j.at("have_gt") == true);
    const auto& s0 = j.at("steps").at(0);
    REQUIRE(s0.contains("pos_sigma_mm"));
    REQUIRE(s0.at("diag_sigma").is_array());
    REQUIRE(s0.at("diag_sigma").size() == 15);
    REQUIRE(s0.contains("pos_drift_mm"));
    REQUIRE(s0.at("q").size() == 4);
}
