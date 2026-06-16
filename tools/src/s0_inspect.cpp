// SPDX-License-Identifier: MIT
//
// s0_inspect — the S0 (sensor & calibration model) stage inspector (epic #371,
// issue #375). The real-data companion to the synthetic S0 contract probe
// (s0_sensor_model.cpp); built on the S4 inspector template (s4_inspect.cpp).
//
// It runs the REAL S0 operators on real EuRoC data: the camera model over a
// pixel grid on an actual cam0 frame (undistorted normalized points + lens
// distortion field + round-trip residual), and the IMU-noise characterizer over
// the imu0 stream (per-channel mean/std, white-noise density, Allan curve). The
// result is an enriched JSONL the image-domain overlay renderer
// (docs-site/scripts/gen-overlay.mjs) draws: the distortion grid over the frame,
// and the Allan-deviation / noise plot.
//
// Usage:
//   s0_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--frame K]
//              [--grid-cols N] [--grid-rows N] [--allan-small N]
//
//   --dataset      sequence mav0 directory (cam0/, imu0/). Required.
//   --out          output dir (default build/stage_probes/S0)
//   --frame        cam0 frame index to draw the distortion grid over (default 0)
//   --grid-cols    distortion-grid columns                       (default 24)
//   --grid-rows    distortion-grid rows                          (default 16)
//   --allan-small  octave factors averaged for white-noise N     (default 4)
//
// Then render the overlay:
//   node docs-site/scripts/gen-overlay.mjs <out>
//
// The camera is the EuRoC cam0 pinhole-radtan model (the same intrinsics the
// synthetic S0 probe uses). EuRoC (~1.5 GB) is not vendored, so this is a
// developer tool; the inspector logic is gated by
// tests/tools/s0_sensor_model_inspect.cpp.

#include <branes/cv/image_io.hpp>
#include <branes/math/cameras.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/tools/s0_sensor_model_inspect.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bs = branes::sdk;
namespace cam = branes::math::cameras;
namespace cv = branes::cv;
namespace bt = branes::tools;

namespace {

struct Args {
    std::string dataset;
    std::string out = "build/stage_probes/S0";
    std::uint64_t frame = 0;
    int grid_cols = 24;
    int grid_rows = 16;
    std::size_t allan_small = 4;
    bool help = false;
};

// EuRoC MAV cam0 pinhole-radtan intrinsics (matches s0_sensor_model.cpp).
cam::PinholeRadtanCamera<double> euroc_cam0() {
    return {458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
}

std::uint64_t to_u64(const std::string& raw) {
    // std::stoull silently wraps a leading '-' (e.g. "-1" → ULLONG_MAX). Reject signs.
    if (!raw.empty() && (raw.front() == '-' || raw.front() == '+'))
        throw std::runtime_error("s0_inspect: expected a non-negative integer, got '" + raw + "'");
    std::size_t pos = 0;
    unsigned long long v = 0;
    try {
        v = std::stoull(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s0_inspect: invalid integer value '" + raw + "'");
    return v;
}

Args parse(int argc, char** argv) {
    Args a;
    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("s0_inspect: missing value after " + std::string(argv[i]));
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        std::string_view v = argv[i];
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--dataset")
            a.dataset = next(i);
        else if (v == "--out")
            a.out = next(i);
        else if (v == "--frame")
            a.frame = to_u64(next(i));
        else if (v == "--grid-cols")
            a.grid_cols = static_cast<int>(to_u64(next(i)));
        else if (v == "--grid-rows")
            a.grid_rows = static_cast<int>(to_u64(next(i)));
        else if (v == "--allan-small")
            a.allan_small = static_cast<std::size_t>(to_u64(next(i)));
        else
            throw std::runtime_error("s0_inspect: unknown argument '" + std::string(v) + "'");
    }
    return a;
}

void usage() {
    std::cout << "s0_inspect — S0 sensor-model inspector (EuRoC camera distortion + IMU noise)\n\n"
                 "  s0_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--frame K]\n"
                 "             [--grid-cols N] [--grid-rows N] [--allan-small N]\n\n"
                 "  Then: node docs-site/scripts/gen-overlay.mjs <out>\n";
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        args = parse(argc, argv);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 2;
    }
    if (args.help) {
        usage();
        return 0;
    }
    if (args.dataset.empty()) {
        std::cerr << "s0_inspect: --dataset <EuRoC mav0 dir> is required.\n\n";
        usage();
        return 2;
    }

    // ── Inputs: a cam0 frame + the imu0 stream ──────────────────────────────
    std::vector<bs::euroc::ImageEntry> images;
    std::vector<bs::ImuMeasurement<double>> imu;
    try {
        images = bs::euroc::parse_images(args.dataset);
        imu = bs::euroc::parse_imu<double>(args.dataset);
    } catch (const std::exception& ex) {
        std::cerr << "s0_inspect: " << ex.what() << "\n";
        return 1;
    }
    if (images.empty()) {
        std::cerr << "s0_inspect: no images under " << args.dataset << "/cam0 — is this an EuRoC mav0 directory?\n";
        return 1;
    }
    if (args.frame >= images.size()) {
        std::cerr << "s0_inspect: --frame " << args.frame << " out of range (" << images.size() << " frames)\n";
        return 1;
    }

    const auto& entry = images[static_cast<std::size_t>(args.frame)];
    cv::OwnedImage<std::uint8_t> img;
    try {
        img = cv::read_png(entry.path);
    } catch (const std::exception& ex) {
        std::cerr << "s0_inspect: cannot read frame " << entry.path << ": " << ex.what() << "\n";
        return 1;
    }
    const auto width = static_cast<std::uint32_t>(img.width());
    const auto height = static_cast<std::uint32_t>(img.height());

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    const std::string out_path = args.out + "/frames.jsonl";
    std::ofstream os(out_path);
    if (!os) {
        std::cerr << "s0_inspect: cannot write " << out_path << "\n";
        return 1;
    }

    // ── Operator 1: the camera model over the frame ─────────────────────────
    const auto c = euroc_cam0();
    const auto cam_report = bt::inspect_camera_model(c,
                                                     "pinhole-radtan",
                                                     458.654,
                                                     457.296,
                                                     367.215,
                                                     248.375,
                                                     width,
                                                     height,
                                                     args.grid_cols,
                                                     args.grid_rows,
                                                     entry.path);
    os << bt::to_json(cam_report).dump() << '\n';

    // ── Operator 2: the IMU noise characterizer over the stream ─────────────
    const auto imu_report =
        bt::characterize_imu<double>(std::span<const bs::ImuMeasurement<double>>(imu), args.allan_small);
    os << bt::to_json(imu_report).dump() << '\n';

    std::cout << "s0_inspect: frame " << args.frame << " (" << width << "x" << height << "), " << imu.size()
              << " IMU samples\n"
              << "  camera:  distortion max " << cam_report.max_distortion_px << " px, round-trip max "
              << cam_report.max_roundtrip_px << " px (" << cam_report.grid.size() << " grid points)\n";
    if (!imu_report.channels.empty())
        std::cout << "  imu:     " << imu_report.rate_hz
                  << " Hz, gyro_x N=" << imu_report.channels[0].white_noise_density
                  << " rad/s/sqrt(Hz), accel_x N=" << imu_report.channels[3].white_noise_density << " m/s^2/sqrt(Hz)\n";
    std::cout << "  frames:  " << out_path << "\n"
              << "  render:  node docs-site/scripts/gen-overlay.mjs " << args.out << "\n";
    return 0;
}
