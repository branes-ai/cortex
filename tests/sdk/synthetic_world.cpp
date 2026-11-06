// Synthetic VIO world + end-to-end backend integration (the noise-demo source).
//
// Validates that the synthetic world generator (sdk/eval/synthetic_world.hpp)
// produces a self-consistent set of sensor streams + exact ground truth that the
// real MSCKF backend can track: with zero injected noise on a gentle (ground-
// robot) trajectory the filter stays initialized, tracks within a bound, keeps a
// PSD covariance, and is over-conservative (NIS < 1) on perfectly clean sensors.
// This is the foundation the vio_pipeline noise→robustness demo builds on.
//
// The A1 cases (#560) check the world on its own, before any filter: its clean
// IMU dead-reckons to its ground truth and is continuous at the motion onset.

#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

namespace {
namespace ev = branes::sdk::eval;
using T = double;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;

double norm3(const Vec3& a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}
}  // namespace

TEST_CASE("synthetic world generates a trackable VIO stream with exact ground truth",
          "[sdk][synthetic][pipeline][slow]") {
    ev::SyntheticConfig<T> cfg;  // gentle "ground robot" so the baseline is tight
    cfg.trans_amp = 0.5;
    cfg.motion_rate = 0.6;
    cfg.yaw_amp = 0.3;
    const auto world = ev::generate_world<T>(cfg);

    // The world is non-degenerate: high-rate IMU, frame-rate camera, and the
    // camera actually sees a healthy number of landmarks per frame.
    REQUIRE(world.imu.size() > 1000);
    REQUIRE(world.frames.size() > 100);
    REQUIRE(world.frames.size() == world.gt.size());
    std::size_t feat = 0;
    for (const auto& f : world.frames)
        feat += f.obs.size();
    REQUIRE(feat / world.frames.size() > 20);  // mean features per frame

    using Backend = branes::sdk::MsckfBackend<T>;
    Backend::CameraCalibration cal;
    cal.intrinsics = world.camera;
    cal.extrinsics.R_imu_cam = world.R_imu_cam;
    cal.extrinsics.p_imu_cam = world.p_imu_cam;
    Backend backend(std::vector<Backend::CameraCalibration>{cal});
    branes::sdk::VioConfig vcfg;
    backend.initialize(vcfg);

    std::size_t imu_idx = 0, tracked = 0;
    double sq = 0;
    for (std::size_t f = 0; f < world.frames.size(); ++f) {
        const double t = world.frames[f].t;
        for (; imu_idx < world.imu.size() && world.imu[imu_idx].timestamp_s <= t; ++imu_idx)
            backend.process_imu(world.imu[imu_idx]);
        backend.process_camera(t, std::span<const branes::sdk::FrontendObservation<T>>{world.frames[f].obs});
        if (!backend.initialized())
            continue;
        const auto est = backend.current_state();
        const Vec3 ep = est.T_world_imu.translation();
        for (int i = 0; i < 3; ++i)
            REQUIRE(std::isfinite(ep[i]));  // no NaN/Inf
        const Vec3& gp = world.gt[f].p;
        const double e = norm3(Vec3{{ep[0] - gp[0], ep[1] - gp[1], ep[2] - gp[2]}});
        sq += e * e;
        ++tracked;
    }

    REQUIRE(backend.initialized());
    REQUIRE(tracked > world.frames.size() * 3 / 4);  // filter ran most of the sequence
    const double ate = std::sqrt(sq / static_cast<double>(tracked));
    INFO("zero-noise ATE (RMS position) = " << ate << " m");
    REQUIRE(ate < 0.5);  // gentle motion, clean sensors ⇒ tracks within half a metre
    // Covariance stays PSD, and on perfectly clean sensors the filter is
    // over-conservative (innovations smaller than the assumed noise ⇒ NIS < 1).
    REQUIRE(branes::sdk::msckf::is_positive_semidefinite(backend.state().covariance()));
    const double nis = backend.nis_consistency().report().normalized;
    REQUIRE(std::isfinite(nis));
    REQUIRE(nis < 1.0);
}

// ── A1 (#560): the world's IMU integrates to its own ground truth ────────────
//
// #475 stepped the velocity by ≈1 m/s at the end of warm-up with no
// accelerometer sample recording it. Nothing checked the world directly, so the
// defect surfaced as roll/pitch NEES ≈ 3000 in the R-IEKF seed sweep and was
// first blamed on the filter. These cases check the world itself, with an
// integrator that shares no code with the filters under test.

namespace {

/// Dead reckoning of the clean IMU stream (true biases removed, no noise) from
/// the first ground-truth sample, compared with the ground truth at every frame.
struct DeadReckoning {
    double horizon_s = 0;        ///< gt.back().t − gt.front().t
    double max_pos_err = 0;      ///< m
    double max_rot_err = 0;      ///< rad
    double max_dv_residual = 0;  ///< m/s: |Δv_gt − ∫a dt| between consecutive frames
    double one_sample_dv = 0;    ///< m/s: max |a_world|·dt, the integration of one IMU sample
};

DeadReckoning dead_reckon(const ev::SyntheticData<T>& w, T gravity) {
    using SO3 = branes::math::lie::SO3<T>;
    const Vec3 g{{0, 0, -gravity}};
    DeadReckoning out;
    REQUIRE(!w.gt.empty());
    REQUIRE(w.imu.size() > 1);

    // Start at the IMU sample that coincides with the first ground-truth frame.
    const double t0 = w.gt.front().t;
    std::size_t k = 0;
    while (k + 1 < w.imu.size() && w.imu[k + 1].timestamp_s <= t0 + 1e-9)
        ++k;
    REQUIRE(std::abs(w.imu[k].timestamp_s - t0) < 1e-9);

    auto omega = [&](std::size_t i) {
        const auto& m = w.imu[i].angular_velocity;
        return Vec3{{m[0] - w.gyro_bias[0], m[1] - w.gyro_bias[1], m[2] - w.gyro_bias[2]}};
    };
    auto accel_world = [&](std::size_t i, const SO3& R) {
        const auto& m = w.imu[i].linear_acceleration;
        return R * Vec3{{m[0] - w.accel_bias[0], m[1] - w.accel_bias[1], m[2] - w.accel_bias[2]}} + g;
    };

    SO3 R = w.gt.front().R;
    Vec3 p = w.gt.front().p;
    Vec3 v = w.gt.front().v;
    Vec3 a = accel_world(k, R);
    Vec3 v_prev_dr = v, v_prev_gt = v;
    std::size_t f = 1;
    double max_a = norm3(a), max_dt = 0;
    for (; k + 1 < w.imu.size() && f < w.gt.size(); ++k) {
        const double dt = w.imu[k + 1].timestamp_s - w.imu[k].timestamp_s;
        max_dt = std::max(max_dt, dt);
        // Trapezoidal rotation rate; acceleration linear over the interval, so
        // the position update is exact for a piecewise-linear a(t).
        const Vec3 wm = (omega(k) + omega(k + 1)) * (T{1} / T{2});
        const SO3 R1 = R * SO3::exp(wm * dt);
        const Vec3 a1 = accel_world(k + 1, R1);
        p = p + v * dt + (a * (T{1} / T{3}) + a1 * (T{1} / T{6})) * (dt * dt);
        v = v + (a + a1) * (dt / T{2});
        R = R1;
        a = a1;
        max_a = std::max(max_a, norm3(a));

        if (std::abs(w.imu[k + 1].timestamp_s - w.gt[f].t) < 1e-9) {
            const auto& gt = w.gt[f];
            out.max_pos_err = std::max(out.max_pos_err, norm3(p - gt.p));
            out.max_rot_err = std::max(out.max_rot_err, norm3((gt.R.inverse() * R).log()));
            // The velocity the IMU recorded over this frame interval must be the
            // velocity the ground truth gained: a step with no accelerometer
            // impulse shows up here, wherever it happens.
            const Vec3 dv_gt = gt.v - v_prev_gt, dv_dr = v - v_prev_dr;
            out.max_dv_residual = std::max(out.max_dv_residual, norm3(dv_gt - dv_dr));
            v_prev_gt = gt.v;
            v_prev_dr = v;
            ++f;
        }
    }
    REQUIRE(f == w.gt.size());  // every frame was visited
    out.horizon_s = w.gt.back().t - t0;
    out.one_sample_dv = max_a * max_dt;
    return out;
}

/// Largest sample-to-sample change of the raw IMU (accel, gyro) in the window
/// around the motion onset.
struct OnsetIncrements {
    double accel = 0;  ///< m/s²
    double gyro = 0;   ///< rad/s
};

OnsetIncrements onset_increments(const ev::SyntheticData<T>& w, double t_lo, double t_hi) {
    OnsetIncrements out;
    for (std::size_t k = 0; k + 1 < w.imu.size(); ++k) {
        if (w.imu[k].timestamp_s < t_lo || w.imu[k + 1].timestamp_s > t_hi)
            continue;
        const auto& a0 = w.imu[k].linear_acceleration;
        const auto& a1 = w.imu[k + 1].linear_acceleration;
        const auto& g0 = w.imu[k].angular_velocity;
        const auto& g1 = w.imu[k + 1].angular_velocity;
        out.accel = std::max(out.accel, norm3(Vec3{{a1[0] - a0[0], a1[1] - a0[1], a1[2] - a0[2]}}));
        out.gyro = std::max(out.gyro, norm3(Vec3{{g1[0] - g0[0], g1[1] - g0[1], g1[2] - g0[2]}}));
    }
    return out;
}

}  // namespace

TEST_CASE("A1: the synthetic world's clean IMU dead-reckons to its ground truth", "[sdk][synthetic][A1]") {
    for (const T onset : {ev::SyntheticConfig<T>{}.onset_s, T{1}, T{2}}) {
        ev::SyntheticConfig<T> cfg;
        cfg.onset_s = onset;
        const auto world = ev::generate_world<T>(cfg);
        const DeadReckoning dr = dead_reckon(world, cfg.gravity);
        INFO("onset_s = " << onset << ": horizon " << dr.horizon_s << " s, max |dp| = " << dr.max_pos_err
                          << " m, max |dR| = " << dr.max_rot_err << " rad, max dv residual = " << dr.max_dv_residual
                          << " m/s (one IMU sample integrates " << dr.one_sample_dv << " m/s)");
        REQUIRE(dr.horizon_s >= 12.0);
        CHECK(dr.max_pos_err < 0.05);  // 5 cm over 12 s
        CHECK(dr.max_rot_err < 1e-3);
        // Velocity is continuous: no frame interval gains more velocity than the
        // accelerometer recorded, beyond what one IMU sample integrates.
        CHECK(dr.max_dv_residual < dr.one_sample_dv);
    }
}

TEST_CASE("A1: the synthetic world's IMU is continuous at the motion onset", "[sdk][synthetic][A1]") {
    // A continuous signal's sample-to-sample change shrinks with the sample
    // interval; a step does not. Generate the same world at 200 Hz and 800 Hz and
    // require the largest increment around the onset to shrink with the rate
    // (ideal ×4 for a C⁰ signal; a step gives ×1).
    for (const T onset : {ev::SyntheticConfig<T>{}.onset_s, T{1}, T{2}}) {
        ev::SyntheticConfig<T> cfg;
        cfg.onset_s = onset;
        cfg.duration_s = cfg.warmup_s + onset + T{1};
        const double lo = static_cast<double>(cfg.warmup_s) - 0.1;
        const double hi = static_cast<double>(cfg.warmup_s + onset) + 0.1;
        const OnsetIncrements slow = onset_increments(ev::generate_world<T>(cfg), lo, hi);
        cfg.imu_rate_hz = T{4} * cfg.imu_rate_hz;
        const OnsetIncrements fast = onset_increments(ev::generate_world<T>(cfg), lo, hi);
        INFO("onset_s = " << onset << ": max |da| " << slow.accel << " -> " << fast.accel << " m/s^2, max |dw| "
                          << slow.gyro << " -> " << fast.gyro << " rad/s (200 -> 800 Hz)");
        REQUIRE(slow.accel > 0);
        REQUIRE(slow.gyro > 0);
        CHECK(fast.accel < slow.accel / 3);
        CHECK(fast.gyro < slow.gyro / 3);
    }
}

TEST_CASE("A1: an unrecorded velocity step (#475) fails the dead-reckoning checks", "[sdk][synthetic][A1]") {
    // Negative control: reinstate the #475 defect on the ground-truth side — from
    // the onset on, the world moves with an extra ≈1 m/s that no accelerometer
    // sample recorded. Both A1 criteria must reject it.
    ev::SyntheticConfig<T> cfg;
    auto world = ev::generate_world<T>(cfg);
    const Vec3 dv{{0, cfg.trans_amp * cfg.motion_rate, T{2} * cfg.vbob_amp * cfg.motion_rate}};  // dp/dσ at σ = 0
    const double t_step = world.gt.front().t;
    for (auto& s : world.gt) {
        s.v = s.v + dv;
        s.p = s.p + dv * static_cast<T>(s.t - t_step);
    }
    world.gt.front().v = world.gt.front().v - dv;  // at rest when the step happens
    const DeadReckoning dr = dead_reckon(world, cfg.gravity);
    INFO("defective world: max |dp| = " << dr.max_pos_err << " m, max dv residual = " << dr.max_dv_residual << " m/s");
    CHECK(dr.max_pos_err > 1.0);
    CHECK(dr.max_dv_residual > 10 * dr.one_sample_dv);
}

// ── A4 (#467): the synthetic camera is the camera it imitates ────────────────
//
// The world is documented as using EuRoC cam0, and every synthetic result is
// read as "on the EuRoC camera". Its p2 was 1000x the published value and
// nothing failed, because the world is self-consistent: it projects with the
// same camera the backend is handed. Hold each parameter to the published
// calibration (examples/hello_vio/testdata/mav0/cam0/sensor.yaml) exactly: each
// is a ratio of exactly representable integers, so IEEE division rounds it to
// the same double as the decimal literal.
TEST_CASE("A4: the synthetic world's camera is EuRoC cam0", "[sdk][synthetic][A4]") {
    const auto cam = ev::generate_world<T>(ev::SyntheticConfig<T>{}).camera;
    CHECK(cam.fx() == 458.654);
    CHECK(cam.fy() == 457.296);
    CHECK(cam.cx() == 367.215);
    CHECK(cam.cy() == 248.375);
    const auto& d = cam.radtan();
    CHECK(d.k1 == -0.28340811);
    CHECK(d.k2 == 0.07395907);
    CHECK(d.p1 == 0.00019359);
    CHECK(d.p2 == 1.76187114e-05);
    CHECK(d.k3 == 0.0);
}
