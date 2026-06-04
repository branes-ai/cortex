// S0 sensor-&-calibration-model probe (docs/arch/vio-pipeline-canonical.md).
//
// These tests assert the S0 contracts in units native to the modeling problem —
// pixels, px/ms, px/deg, px/mm, m/s², mm — each against an oracle INDEPENDENT of
// the implementation (the projection model round-trips against itself only as an
// identity; the analytic Jacobian is checked against central differences; the
// IMU integrator is checked against the closed-form ½·a·t² drift). That
// independence is deliberate: the methodology post-mortem traced a real bug to
// synthetic tests that re-encoded the code's own assumptions.
//
// When the env var CORTEX_PROBE_OUT names a directory, each test also writes a
// CSV artifact there; docs-site/scripts/gen-sensor-model-figures.mjs renders
// those to SVG heatmaps and curves so the metrics can be inspected qualitatively.
// Without the env var the tests still run and assert — only the artifacts are skipped.

#include <branes/sdk/eval/sensor_model_probe.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
namespace cam = branes::math::cameras;
using T = double;
using SO3 = branes::math::lie::SO3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;

// EuRoC MAV cam0 — pinhole + radial-tangential (the canonical VIO benchmark).
cam::PinholeRadtanCamera<T> euroc_cam0() {
    return {458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
}
constexpr T kW = 752, kH = 480;

// A representative wide-FOV fisheye (Kannala–Brandt), TUM-VI-like, to exercise
// the equidistant model where distortion is strong.
cam::EquidistantCamera<T> fisheye() {
    return {190.0, 190.0, 254.0, 256.0, 0.003, 0.001, -0.0005, 0.0001};
}

// EuRoC cam0 extrinsic translation (camera origin in the IMU/body frame). The
// rotation is a documented stand-in (identity) so the px/mm slope reads on the
// camera x-axis; the magnitude is representative of the real rig.
Vec3 euroc_p_imu_cam() {
    return Vec3{{-0.0216, -0.0647, 0.00981}};
}

// ── CSV artifact helpers (only when CORTEX_PROBE_OUT is set) ──────────────

const char* probe_out_dir() {
    return std::getenv("CORTEX_PROBE_OUT");
}

std::ofstream open_artifact(const std::string& name) {
    const char* dir = probe_out_dir();
    std::ofstream f;
    if (dir)
        f.open(std::string(dir) + "/" + name);
    return f;  // closed/!is_open when no dir — writes become no-ops via guards
}

}  // namespace

// ── S0.1  Projection round-trip (pixels) ─────────────────────────────────

TEST_CASE("S0 camera projection round-trips within sub-pixel tolerance", "[sdk][s0][camera]") {
    const auto c = euroc_cam0();
    const auto rt = ev::camera_round_trip(c, kW, kH, 40u, 26u);

    INFO("radtan round-trip residual: max=" << rt.residual_px.max << " px  rms=" << rt.residual_px.rms
                                            << " px  over incidence [" << rt.min_incidence_deg << ", "
                                            << rt.max_incidence_deg << "] deg");
    // Contract: the distortion model is its own inverse to well under a pixel.
    REQUIRE(rt.residual_px.count > 100);
    REQUIRE(rt.residual_px.max < 1e-2);

    if (auto f = open_artifact("roundtrip_radtan.csv"); f.is_open()) {
        f << "x_px,y_px,residual_px,incidence_deg\n";
        for (const auto& s : rt.field)
            f << s.x << ',' << s.y << ',' << s.value << ',' << s.incidence_deg << '\n';
    }
}

TEST_CASE("S0 fisheye projection round-trips within sub-pixel tolerance", "[sdk][s0][camera]") {
    const auto c = fisheye();
    // Cap sampling at 70° incidence — the forward-FOV boundary for this lens
    // (a corner pixel's distorted radius exceeds θ_d(π/2) and is simply outside
    // the lens's forward field, where undistort is undefined). Within the FOV the
    // round-trip is machine-precision; the heatmap renders the valid disk.
    const auto rt = ev::camera_round_trip(c,
                                          508.0,
                                          512.0,
                                          40u,
                                          40u,
                                          /*margin_px=*/4.0,
                                          /*incidence_cap_deg=*/70.0);
    if (auto f = open_artifact("roundtrip_fisheye.csv"); f.is_open()) {
        f << "x_px,y_px,residual_px,incidence_deg\n";
        for (const auto& s : rt.field)
            f << s.x << ',' << s.y << ',' << s.value << ',' << s.incidence_deg << '\n';
    }
    INFO("fisheye round-trip residual: max=" << rt.residual_px.max << " px  rms=" << rt.residual_px.rms
                                             << " px  max incidence=" << rt.max_incidence_deg << " deg");
    REQUIRE(rt.residual_px.count > 100);
    REQUIRE(rt.residual_px.max < 1e-2);
}

// ── S0.2  Projection-Jacobian consistency (pixels & %) ───────────────────

TEST_CASE("S0 analytic projection Jacobian matches central differences", "[sdk][s0][camera][jacobian]") {
    const auto c = euroc_cam0();
    const auto jr = ev::camera_jacobian_consistency(c, kW, kH, 40u, 26u);

    INFO("radtan Jacobian: max rel-Frobenius=" << jr.rel_frob.max << "  max lin-px-err(1cm)=" << jr.lin_px.max
                                               << " px");
    // Contract: the analytic Jacobian is the true derivative — relative error is
    // at the finite-difference floor, not a modeling-sized discrepancy.
    REQUIRE(jr.rel_frob.count > 100);
    REQUIRE(jr.rel_frob.max < 1e-4);

    if (auto f = open_artifact("jacobian_radtan.csv"); f.is_open()) {
        f << "x_px,y_px,lin_px_err\n";
        for (const auto& s : jr.field)
            f << s.x << ',' << s.y << ',' << s.value << '\n';
    }
}

TEST_CASE("S0 analytic fisheye Jacobian matches central differences", "[sdk][s0][camera][jacobian]") {
    const auto c = fisheye();
    const auto jr = ev::camera_jacobian_consistency(c, 508.0, 512.0, 36u, 36u, /*depth_m=*/4.0);
    INFO("fisheye Jacobian: max rel-Frobenius=" << jr.rel_frob.max);
    REQUIRE(jr.rel_frob.count > 100);
    REQUIRE(jr.rel_frob.max < 1e-3);

    if (auto f = open_artifact("jacobian_fisheye.csv"); f.is_open()) {
        f << "x_px,y_px,lin_px_err\n";
        for (const auto& s : jr.field)
            f << s.x << ',' << s.y << ',' << s.value << '\n';
    }
}

// ── S0.3  Time-offset sensitivity (pixels per millisecond) ───────────────

TEST_CASE("S0 quantifies the pixel cost of an unmodeled camera-IMU time offset", "[sdk][s0][calib][timeoffset]") {
    const auto c = euroc_cam0();
    const cam::Vec3<T> p_c{1.0, 0.5, 5.0};  // a point 5 m out, off the optical axis

    const std::vector<ev::MotionRegime<T>> regimes = {
        {"slow", Vec3{{0.3, 0, 0}}, Vec3{{0, 0.2, 0}}},
        {"medium", Vec3{{1.0, 0, 0}}, Vec3{{0, 0.6, 0}}},
        {"fast_aggressive", Vec3{{2.0, 0, 0}}, Vec3{{0, 1.5, 0}}},
    };

    auto f = open_artifact("timeoffset.csv");
    if (f.is_open())
        f << "regime,dt_ms,error_px,px_per_ms\n";

    for (const auto& m : regimes) {
        const auto r = ev::time_offset_sensitivity(c, p_c, m);
        INFO(std::string(m.name) << ": " << r.px_per_ms << " px/ms;  1px budget = " << r.dt_budget_ms_at_1px << " ms");
        // Sensitivity must rise monotonically with motion, and aggressive motion
        // must spend a real pixel budget on even a few ms of offset.
        REQUIRE(r.px_per_ms > 0.0);
        if (f.is_open())
            for (const auto& pt : r.curve)
                f << m.name << ',' << pt.dt_ms << ',' << pt.error_px << ',' << r.px_per_ms << '\n';
    }

    // The aggressive regime must dominate the slow one (the whole point: fast
    // motion makes the unmodeled offset expensive).
    const auto slow = ev::time_offset_sensitivity(c, p_c, regimes[0]);
    const auto fast = ev::time_offset_sensitivity(c, p_c, regimes[2]);
    REQUIRE(fast.px_per_ms > 3.0 * slow.px_per_ms);
}

// ── S0.4  Extrinsic-calibration sensitivity (px/deg, px/mm) ───────────────

TEST_CASE("S0 quantifies the pixel bias from extrinsic-calibration error", "[sdk][s0][calib][extrinsic]") {
    const auto c = euroc_cam0();
    const SO3 R_ic{};  // identity stand-in (see euroc_p_imu_cam note)
    const Vec3 p_ic = euroc_p_imu_cam();

    auto f = open_artifact("extrinsic.csv");
    if (f.is_open())
        f << "depth_m,type,perturb,error_px\n";

    struct Case {
        const char* tag;
        T depth;
    };
    T near_px_per_mm = 0, far_px_per_mm = 0;
    for (const Case cs : {Case{"near", 2.0}, Case{"far", 10.0}}) {
        const Vec3 p_feat{{p_ic[0] + 0.6, p_ic[1] + 0.3, p_ic[2] + cs.depth}};
        const auto r = ev::extrinsic_sensitivity(c, p_feat, R_ic, p_ic);
        INFO(std::string(cs.tag) << " (" << cs.depth << " m): " << r.px_per_deg << " px/deg, " << r.px_per_mm
                                 << " px/mm");
        REQUIRE(r.px_per_deg > 0.0);
        REQUIRE(r.px_per_mm > 0.0);
        if (std::string(cs.tag) == "near")
            near_px_per_mm = r.px_per_mm;
        else
            far_px_per_mm = r.px_per_mm;
        if (f.is_open()) {
            for (const auto& pt : r.rot_curve)
                f << cs.depth << ",rot_deg," << pt.perturb << ',' << pt.error_px << '\n';
            for (const auto& pt : r.trans_curve)
                f << cs.depth << ",trans_mm," << pt.perturb << ',' << pt.error_px << '\n';
        }
    }
    // Translation sensitivity scales with 1/depth: a near feature is more
    // sensitive to a mm of extrinsic error than a far one.
    REQUIRE(near_px_per_mm > far_px_per_mm);
}

// ── S0.5  IMU static identity & gravity leakage (m/s², mm) ────────────────

TEST_CASE("S0 IMU integrates a stationary stream to ~zero drift, and leaks small "
          "orientation/bias errors into large position drift",
          "[sdk][s0][imu]") {
    const auto cases = ev::imu_static_identity(/*duration_s=*/10.0, /*rate_hz=*/200.0);

    auto f = open_artifact("imu_drift.csv");
    if (f.is_open())
        f << "case,t_s,pos_drift_mm,vel_drift_mm_s\n";

    const ev::ImuStaticCase<T>* ideal = nullptr;
    const ev::ImuStaticCase<T>* grav = nullptr;
    const ev::ImuStaticCase<T>* tilt = nullptr;
    const ev::ImuStaticCase<T>* bias = nullptr;
    for (const auto& cse : cases) {
        const std::string n = cse.name;
        if (n == "ideal")
            ideal = &cse;
        else if (n == "gravity_sign")
            grav = &cse;
        else if (n == "tilt_0.5deg")
            tilt = &cse;
        else if (n == "accel_bias")
            bias = &cse;
        if (f.is_open())
            for (const auto& pt : cse.curve)
                f << cse.name << ',' << pt.t_s << ',' << pt.pos_drift_mm << ',' << pt.vel_drift_mm_s << '\n';
    }
    REQUIRE(ideal);
    REQUIRE(grav);
    REQUIRE(tilt);
    REQUIRE(bias);

    INFO("ideal residual accel = " << ideal->residual_accel_ms2 << " m/s²,  drift = " << ideal->final_pos_drift_mm
                                   << " mm");
    // Ideal: correct R + correct gravity ⇒ specific force cancels exactly.
    REQUIRE(ideal->residual_accel_ms2 < 1e-9);
    REQUIRE(ideal->final_pos_drift_mm < 1e-6);

    // Wrong gravity sign ⇒ ~2g residual ⇒ catastrophic (hundreds of metres / 10 s).
    REQUIRE(grav->final_pos_drift_mm > 1e5);

    // 0.5° tilt leaks gravity into the horizontal: ½·g·sin(0.5°)·t² ≈ 4.3 m.
    INFO("tilt 0.5° drift = " << tilt->final_pos_drift_mm << " mm (expected ~4300)");
    REQUIRE(tilt->final_pos_drift_mm > 2000.0);
    REQUIRE(tilt->final_pos_drift_mm < 8000.0);

    // Unmodeled accel bias 0.05 m/s² ⇒ ½·0.05·t² = 2.5 m.
    INFO("accel-bias drift = " << bias->final_pos_drift_mm << " mm (expected ~2500)");
    REQUIRE(bias->final_pos_drift_mm > 1500.0);
    REQUIRE(bias->final_pos_drift_mm < 4000.0);
}
