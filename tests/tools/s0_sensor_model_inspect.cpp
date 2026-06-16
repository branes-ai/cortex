// SPDX-License-Identifier: MIT
//
// Gate for the S0 sensor-model inspector (branes/tools/s0_sensor_model_inspect.hpp,
// issue #375). The real inspector runs over EuRoC, which CI does not vendor, so
// here we drive the operators directly: the camera model on the known EuRoC cam0
// intrinsics (and a zero-distortion pinhole), and the IMU characterizer on a
// synthetic stationary stream with a known per-channel noise level. We pin the
// behaviour the overlay + downstream study rely on: the projection round-trip
// holds the S0.1 contract, the distortion field is ~0 at the centre and grows to
// the corners (and is ~0 for a distortion-free lens), the white-noise density
// recovers the injected σ·√dt, and both reports serialize to the overlay schema.

#include <branes/math/cameras.hpp>
#include <branes/sdk/vio_backend.hpp>
#include <branes/tools/s0_sensor_model_inspect.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

namespace bt = branes::tools;
namespace cam = branes::math::cameras;
namespace bs = branes::sdk;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

cam::PinholeRadtanCamera<double> euroc_cam0() {
    return {458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
}

// The lens displacement of the grid sample nearest a pixel (cx,cy)-ish centre.
double centre_distortion(const bt::S0CameraReport& r) {
    double best = 1e300, val = 0.0;
    for (const auto& s : r.grid) {
        const double d = (s.u - r.cx) * (s.u - r.cx) + (s.v - r.cy) * (s.v - r.cy);
        if (d < best) {
            best = d;
            val = s.distortion_px;
        }
    }
    return val;
}

}  // namespace

TEST_CASE("S0 inspector: camera round-trip holds and distortion grows to the edge", "[tools][s0_inspect]") {
    const auto c = euroc_cam0();
    const auto rep =
        bt::inspect_camera_model(c, "pinhole-radtan", 458.654, 457.296, 367.215, 248.375, 752, 480, 24, 16);

    REQUIRE(rep.model == "pinhole-radtan");
    REQUIRE(rep.width == 752);
    REQUIRE(rep.height == 480);
    REQUIRE(rep.grid_cols == 24);
    REQUIRE(rep.grid_rows == 16);
    REQUIRE(rep.grid.size() > 0);

    // The S0.1 contract: project(unproject(px)) ≈ px to well under a pixel.
    REQUIRE(rep.max_roundtrip_px < 1e-2);
    for (const auto& s : rep.grid)
        REQUIRE(s.roundtrip_px < 1e-2);

    // Distortion is the lens displacement: small near the principal point, large
    // at the corners (EuRoC's k1 = −0.28 is appreciable barrel distortion).
    REQUIRE(centre_distortion(rep) < 2.0);
    REQUIRE(rep.max_distortion_px > 5.0);
}

TEST_CASE("S0 inspector: a distortion-free pinhole shows ~zero displacement", "[tools][s0_inspect]") {
    const cam::PinholeRadtanCamera<double> pinhole{458.654, 457.296, 367.215, 248.375, 0.0, 0.0, 0.0, 0.0};
    const auto rep =
        bt::inspect_camera_model(pinhole, "pinhole-radtan", 458.654, 457.296, 367.215, 248.375, 752, 480, 12, 8);

    REQUIRE(rep.grid.size() > 0);
    REQUIRE(rep.max_distortion_px < 1e-6);  // no distortion → ideal == observed
    REQUIRE(rep.max_roundtrip_px < 1e-6);
    // With no distortion the ideal pinhole projection lands on the sampled pixel.
    for (const auto& s : rep.grid) {
        REQUIRE_THAT(s.ideal_u, WithinAbs(s.u, 1e-6));
        REQUIRE_THAT(s.ideal_v, WithinAbs(s.v, 1e-6));
    }
}

TEST_CASE("S0 inspector: IMU characterizer recovers the injected noise and rate", "[tools][s0_inspect]") {
    // Synthetic stationary stream: a deterministic, portable LCG of iid samples
    // (no <random> distributions — those are not identical across stdlibs). Each
    // channel gets centred uniform noise of a known amplitude; for iid noise the
    // Allan white-noise density is N = σ·√dt.
    const double dt = 1.0 / 200.0;  // 200 Hz, like EuRoC imu0
    const std::size_t n = 16384;
    std::vector<bs::ImuMeasurement<double>> imu;
    imu.reserve(n);

    std::uint64_t state = 0x9e3779b97f4a7c15ULL;
    auto u = [&]() {  // uniform in [-1, 1)
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        const std::uint32_t hi = static_cast<std::uint32_t>(state >> 32);
        return (static_cast<double>(hi) / 2147483648.0) - 1.0;
    };
    const double gyro_amp = 0.01, accel_amp = 0.05;  // half-ranges
    for (std::size_t k = 0; k < n; ++k) {
        bs::ImuMeasurement<double> m;
        m.timestamp_s = static_cast<double>(k) * dt;
        m.angular_velocity = bs::Vec3<double>{{gyro_amp * u(), gyro_amp * u(), gyro_amp * u()}};
        m.linear_acceleration = bs::Vec3<double>{{accel_amp * u(), accel_amp * u(), accel_amp * u()}};
        imu.push_back(m);
    }

    const auto rep = bt::characterize_imu<double>(std::span<const bs::ImuMeasurement<double>>(imu));

    REQUIRE(rep.n_samples == n);
    REQUIRE_THAT(rep.rate_hz, WithinRel(200.0, 1e-9));
    REQUIRE_THAT(rep.dt_s, WithinRel(dt, 1e-9));
    REQUIRE(rep.channels.size() == 6);

    REQUIRE(rep.channels[0].name == "gyro_x");
    REQUIRE(rep.channels[0].unit == "rad/s");
    REQUIRE(rep.channels[3].name == "accel_x");
    REQUIRE(rep.channels[3].unit == "m/s^2");

    for (const auto& ch : rep.channels) {
        REQUIRE(ch.taus.size() > 0);
        REQUIRE(ch.allan_dev.size() == ch.taus.size());
        REQUIRE(ch.white_noise_density > 0.0);
        // For iid noise N = σ·√dt. Recover it from the channel's own empirical σ.
        const double expected_N = ch.stddev * std::sqrt(dt);
        REQUIRE_THAT(ch.white_noise_density, WithinRel(expected_N, 0.20));
        REQUIRE_THAT(ch.mean, WithinAbs(0.0, 5e-4));  // centred noise
    }
}

TEST_CASE("S0 inspector: reports serialize to the overlay schema", "[tools][s0_inspect]") {
    const auto c = euroc_cam0();
    const auto cam_rep =
        bt::inspect_camera_model(c, "pinhole-radtan", 458.654, 457.296, 367.215, 248.375, 752, 480, 8, 6, "cam0/x.png");
    const auto jc = bt::to_json(cam_rep);
    REQUIRE(jc.at("kind") == "s0_camera");
    REQUIRE(jc.at("stage") == "S0");
    REQUIRE(jc.at("image") == "cam0/x.png");
    REQUIRE(jc.at("samples").is_array());
    REQUIRE(jc.at("grid").at("cols") == 8);
    REQUIRE(jc.contains("max_distortion_px"));
    for (const auto& s : jc.at("samples")) {
        REQUIRE(s.contains("u"));
        REQUIRE(s.contains("iu"));
        REQUIRE(s.contains("dist"));
        REQUIRE(s.contains("rt"));
    }

    // Minimal two-sample IMU stream still serializes the full channel schema.
    std::vector<bs::ImuMeasurement<double>> imu(4);
    for (std::size_t k = 0; k < imu.size(); ++k)
        imu[k].timestamp_s = static_cast<double>(k) * 0.005;
    const auto imu_rep = bt::characterize_imu<double>(std::span<const bs::ImuMeasurement<double>>(imu));
    const auto ji = bt::to_json(imu_rep);
    REQUIRE(ji.at("kind") == "s0_imu");
    REQUIRE(ji.at("channels").is_array());
    REQUIRE(ji.at("channels").size() == 6);
    for (const auto& ch : ji.at("channels")) {
        REQUIRE(ch.contains("name"));
        REQUIRE(ch.contains("N"));
        REQUIRE(ch.at("taus").is_array());
        REQUIRE(ch.at("allan").is_array());
    }
}
