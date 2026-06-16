// SPDX-License-Identifier: MIT
//
// branes/tools/s0_sensor_model_inspect.hpp — the S0 (sensor & calibration
// model) inspector (epic #371, issue #375). The real-data companion to the
// synthetic S0 contract probe (sdk/eval/sensor_model_probe.hpp), built on the
// S4 inspector template (issue #374).
//
// An inspector re-runs the REAL stage operator on real data and exposes the
// quantities the production path hides, for study. S0 is the substrate every
// later stage trusts — the camera projection/distortion model and the IMU
// measurement/noise model — so this inspector runs exactly those two operators:
//
//   1. the camera model (branes/math/cameras): over a regular pixel grid it
//      unprojects each pixel to its UNDISTORTED normalized bearing, projects an
//      ideal (distortion-free) pinhole of that bearing, and reports the lens
//      DISPLACEMENT (distorted − ideal) plus the project∘unproject ROUND-TRIP
//      residual — the S0.1 contract, measured on the actual frame so the
//      distortion grid can be drawn over real texture;
//   2. the IMU noise model (branes/sdk/eval/allan_variance): per channel it
//      reports mean/std, the white-noise density N the filter's measurement
//      noise needs, and the Allan-deviation curve — the per-channel noise
//      characterization, computed from the real imu0 stream.
//
// The computation lives in this header (not the .cpp driver) so it is
// unit-testable on a known camera model and a synthetic IMU stream without the
// ~1.5 GB EuRoC dataset.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_S0_SENSOR_MODEL_INSPECT_HPP
#define BRANES_TOOLS_S0_SENSOR_MODEL_INSPECT_HPP

#include <branes/math/arithmetic.hpp>  // math::Scalar
#include <branes/math/cameras.hpp>     // Vec2/Vec3 + camera models
#include <branes/sdk/eval/allan_variance.hpp>
#include <branes/sdk/vio_backend.hpp>  // branes::sdk::ImuMeasurement

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace branes::tools {

// ── S0 camera-model record (distortion grid + round-trip) ─────────────────

/// One grid sample on the image plane, run through the real camera model.
struct S0GridSample {
    double u = 0.0, v = 0.0;              ///< the sampled (distorted/observed) pixel
    double ideal_u = 0.0, ideal_v = 0.0;  ///< pinhole projection of the bearing with NO distortion
    double nx = 0.0, ny = 0.0;            ///< undistorted normalized image coords (bearing x/z, y/z)
    double distortion_px = 0.0;           ///< |(u,v) − (ideal_u,ideal_v)| — what the lens displaced
    double roundtrip_px = 0.0;            ///< |project(unproject(u,v)) − (u,v)| — the S0.1 contract
    double incidence_deg = 0.0;           ///< bearing angle from the optical axis (input coverage)
};

/// Everything the camera operator produced over the grid — one inspector record.
struct S0CameraReport {
    std::string model;       ///< "pinhole-radtan" | "equidistant" | ...
    std::string image_path;  ///< EuRoC frame the grid is drawn over (may be empty)
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    std::uint32_t width = 0, height = 0;
    int grid_cols = 0, grid_rows = 0;
    std::vector<S0GridSample> grid;
    double max_distortion_px = 0.0;  ///< worst lens displacement over the grid
    double max_roundtrip_px = 0.0;   ///< worst round-trip residual (contract)
};

// ── S0 IMU-model record (per-channel noise characterization) ──────────────

/// One IMU axis characterized from the real stream.
struct S0ImuChannel {
    std::string name;  ///< "gyro_x" .. "accel_z"
    std::string unit;  ///< "rad/s" (gyro) | "m/s^2" (accel)
    double mean = 0.0;
    double stddev = 0.0;
    double white_noise_density = 0.0;  ///< N — the continuous-time density (unit/√Hz)
    std::vector<double> taus;          ///< averaging times τ (s), octave-spaced
    std::vector<double> allan_dev;     ///< σ_A(τ) — the Allan-deviation curve
};

/// Everything the IMU-noise operator produced — one inspector record.
struct S0ImuReport {
    double rate_hz = 0.0;
    double dt_s = 0.0;
    std::uint64_t n_samples = 0;
    double duration_s = 0.0;
    std::vector<S0ImuChannel> channels;  ///< gyro x/y/z then accel x/y/z (6)
};

// ── Camera operator ───────────────────────────────────────────────────────

/// Run the real camera model over an `cols × ny` pixel grid (inset by
/// `margin_px`). `Cam` is any model exposing `project(Vec3)`/`unproject(Vec2)`
/// (pinhole-radtan, equidistant, unified); `(fx,fy,cx,cy)` are the pinhole
/// intrinsics, used to form the distortion-free reference projection. Pixels
/// whose bearing folds behind the camera, or whose incidence exceeds
/// `incidence_cap_deg` (the fisheye FOV boundary), are dropped from the grid.
template <class Cam>
[[nodiscard]] inline S0CameraReport inspect_camera_model(const Cam& cam,
                                                         std::string model,
                                                         double fx,
                                                         double fy,
                                                         double cx,
                                                         double cy,
                                                         std::uint32_t width,
                                                         std::uint32_t height,
                                                         int cols,
                                                         int rows,
                                                         std::string image_path = {},
                                                         double margin_px = 2.0,
                                                         double incidence_cap_deg = 89.0) {
    using branes::math::cameras::Vec2;
    using branes::math::cameras::Vec3;
    constexpr double kPi = 3.14159265358979323846;

    S0CameraReport out;
    out.model = std::move(model);
    out.image_path = std::move(image_path);
    out.fx = fx;
    out.fy = fy;
    out.cx = cx;
    out.cy = cy;
    out.width = width;
    out.height = height;
    out.grid_cols = cols;
    out.grid_rows = rows;
    out.grid.reserve(static_cast<std::size_t>(std::max(0, cols)) * static_cast<std::size_t>(std::max(0, rows)));

    const double W = static_cast<double>(width), H = static_cast<double>(height);
    for (int iy = 0; iy < rows; ++iy) {
        for (int ix = 0; ix < cols; ++ix) {
            const double fxr = cols > 1 ? static_cast<double>(ix) / static_cast<double>(cols - 1) : 0.5;
            const double fyr = rows > 1 ? static_cast<double>(iy) / static_cast<double>(rows - 1) : 0.5;
            const Vec2<double> px{margin_px + fxr * (W - 2.0 * margin_px), margin_px + fyr * (H - 2.0 * margin_px)};

            const Vec3<double> bearing = cam.unproject(px);
            if (!(bearing[2] > 0.0))  // folds behind the camera (extreme fisheye corner)
                continue;
            const double nx = bearing[0] / bearing[2];
            const double ny = bearing[1] / bearing[2];
            const double inc =
                std::atan2(std::sqrt(bearing[0] * bearing[0] + bearing[1] * bearing[1]), bearing[2]) * (180.0 / kPi);
            if (inc > incidence_cap_deg)  // past the lens FOV the inverse is meaningless
                continue;

            // Ideal (distortion-free) pinhole projection of the same bearing.
            const double iu = fx * nx + cx;
            const double iv = fy * ny + cy;

            // Round-trip the real model: project(unproject(px)) should return px.
            const Vec2<double> back = cam.project(bearing);
            const double rt = std::sqrt((back[0] - px[0]) * (back[0] - px[0]) + (back[1] - px[1]) * (back[1] - px[1]));
            const double disp = std::sqrt((px[0] - iu) * (px[0] - iu) + (px[1] - iv) * (px[1] - iv));

            out.grid.push_back(S0GridSample{px[0], px[1], iu, iv, nx, ny, disp, rt, inc});
            if (disp > out.max_distortion_px)
                out.max_distortion_px = disp;
            if (rt > out.max_roundtrip_px)
                out.max_roundtrip_px = rt;
        }
    }
    return out;
}

// ── IMU operator ──────────────────────────────────────────────────────────

namespace s0_detail {

/// Mean and (population) standard deviation of a series.
template <class T>
inline void mean_std(const std::vector<T>& x, double& mean, double& stddev) {
    mean = 0.0;
    stddev = 0.0;
    if (x.empty())
        return;
    for (T v : x)
        mean += static_cast<double>(v);
    mean /= static_cast<double>(x.size());
    double acc = 0.0;
    for (T v : x) {
        const double d = static_cast<double>(v) - mean;
        acc += d * d;
    }
    stddev = std::sqrt(acc / static_cast<double>(x.size()));
}

inline S0ImuChannel characterize_channel(std::string name,
                                         std::string unit,
                                         const std::vector<double>& series,
                                         double dt_s,
                                         const std::vector<double>& taus,
                                         std::size_t n_small) {
    S0ImuChannel ch;
    ch.name = std::move(name);
    ch.unit = std::move(unit);
    mean_std(series, ch.mean, ch.stddev);
    ch.taus = taus;
    ch.allan_dev = branes::sdk::eval::allan_deviation<double>(
        std::span<const double>(series), dt_s, std::span<const double>(taus));
    ch.white_noise_density =
        branes::sdk::eval::white_noise_density<double>(std::span<const double>(series), dt_s, n_small);
    return ch;
}

}  // namespace s0_detail

/// Characterize the real IMU stream: split into the 6 axes, and for each report
/// mean/std, the Allan-deviation curve, and the white-noise density N. `dt_s` is
/// taken from the mean inter-sample time of `imu` (EuRoC imu0 is a regular
/// 200 Hz stream). `n_small` is the number of smallest octave averaging factors
/// the white-noise density averages over (see allan_variance.hpp).
template <branes::math::Scalar T>
[[nodiscard]] inline S0ImuReport characterize_imu(std::span<const branes::sdk::ImuMeasurement<T>> imu,
                                                  std::size_t n_small = 4) {
    S0ImuReport out;
    out.n_samples = static_cast<std::uint64_t>(imu.size());
    if (imu.size() < 2)
        return out;

    out.duration_s = imu.back().timestamp_s - imu.front().timestamp_s;
    out.dt_s = out.duration_s / static_cast<double>(imu.size() - 1);
    out.rate_hz = out.dt_s > 0.0 ? 1.0 / out.dt_s : 0.0;

    std::vector<double> gx, gy, gz, ax, ay, az;
    const std::size_t n = imu.size();
    gx.reserve(n);
    gy.reserve(n);
    gz.reserve(n);
    ax.reserve(n);
    ay.reserve(n);
    az.reserve(n);
    for (const auto& m : imu) {
        gx.push_back(static_cast<double>(m.angular_velocity[0]));
        gy.push_back(static_cast<double>(m.angular_velocity[1]));
        gz.push_back(static_cast<double>(m.angular_velocity[2]));
        ax.push_back(static_cast<double>(m.linear_acceleration[0]));
        ay.push_back(static_cast<double>(m.linear_acceleration[1]));
        az.push_back(static_cast<double>(m.linear_acceleration[2]));
    }

    const auto taus = branes::sdk::eval::octave_taus<double>(n, out.dt_s);
    out.channels.reserve(6);
    out.channels.push_back(s0_detail::characterize_channel("gyro_x", "rad/s", gx, out.dt_s, taus, n_small));
    out.channels.push_back(s0_detail::characterize_channel("gyro_y", "rad/s", gy, out.dt_s, taus, n_small));
    out.channels.push_back(s0_detail::characterize_channel("gyro_z", "rad/s", gz, out.dt_s, taus, n_small));
    out.channels.push_back(s0_detail::characterize_channel("accel_x", "m/s^2", ax, out.dt_s, taus, n_small));
    out.channels.push_back(s0_detail::characterize_channel("accel_y", "m/s^2", ay, out.dt_s, taus, n_small));
    out.channels.push_back(s0_detail::characterize_channel("accel_z", "m/s^2", az, out.dt_s, taus, n_small));
    return out;
}

// ── Serialization (the schema the overlay renderer reads) ─────────────────

[[nodiscard]] inline nlohmann::json to_json(const S0CameraReport& r) {
    using nlohmann::json;
    json samples = json::array();
    for (const auto& s : r.grid)
        samples.push_back(json{{"u", s.u},
                               {"v", s.v},
                               {"iu", s.ideal_u},
                               {"iv", s.ideal_v},
                               {"nx", s.nx},
                               {"ny", s.ny},
                               {"dist", s.distortion_px},
                               {"rt", s.roundtrip_px},
                               {"inc", s.incidence_deg}});
    return json{{"kind", "s0_camera"},
                {"stage", "S0"},
                {"frame", 0},
                {"model", r.model},
                {"image", r.image_path},
                {"width", r.width},
                {"height", r.height},
                {"intrinsics", json{{"fx", r.fx}, {"fy", r.fy}, {"cx", r.cx}, {"cy", r.cy}}},
                {"grid", json{{"cols", r.grid_cols}, {"rows", r.grid_rows}}},
                {"samples", samples},
                {"max_distortion_px", r.max_distortion_px},
                {"max_roundtrip_px", r.max_roundtrip_px}};
}

[[nodiscard]] inline nlohmann::json to_json(const S0ImuReport& r) {
    using nlohmann::json;
    json channels = json::array();
    for (const auto& c : r.channels)
        channels.push_back(json{{"name", c.name},
                                {"unit", c.unit},
                                {"mean", c.mean},
                                {"std", c.stddev},
                                {"N", c.white_noise_density},
                                {"taus", c.taus},
                                {"allan", c.allan_dev}});
    return json{{"kind", "s0_imu"},
                {"stage", "S0"},
                {"frame", 1},
                {"rate_hz", r.rate_hz},
                {"dt_s", r.dt_s},
                {"n_samples", r.n_samples},
                {"duration_s", r.duration_s},
                {"channels", channels}};
}

}  // namespace branes::tools

#endif  // BRANES_TOOLS_S0_SENSOR_MODEL_INSPECT_HPP
