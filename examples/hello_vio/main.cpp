// hello_vio — the smallest complete "white-box" consumer of the cortex SDK.
//
// Runs visual-inertial odometry on a EuRoC MAV sequence by calling the C++ SDK
// directly: no daemon, no Zenoh, no Resource Manager. It shows the four things
// every SDK consumer does:
//
//   1. Turn a configuration file into typed SDK structs (here: the EuRoC
//      sequence's own cam0/imu0 sensor.yaml -> camera calibration + VioConfig).
//      YAML stays in the application; the SDK only ever sees the structs.
//   2. Construct a VioEstimator and walk it through its lifecycle
//      (configure -> activate -> ... -> teardown).
//   3. Feed it time-ordered measurements: all IMU samples up to a frame's
//      timestamp, then the frame.
//   4. Read the pose back after every frame.
//
// Usage:  hello_vio <path/to/sequence/mav0> [max_frames=200]
//
// See README.md next to this file for a walkthrough.

#include <branes/cv/image_io.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/sfm/init_window.hpp>  // so3_from_matrix
#include <branes/sdk/vio_estimator.hpp>

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace sdk = branes::sdk;
using T = double;
using Backend = sdk::MsckfBackend<T>;
using Estimator = sdk::VioEstimator<T, Backend>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using Mat3 = branes::math::lie::detail::Mat<T, 3, 3>;

// Calibration files written by OpenCV's FileStorage (common for re-exported
// EuRoC calibrations) begin with "%YAML:1.0", which is not a valid YAML
// directive. Drop such a line so both the dataset's own sensor.yaml and those
// re-exports load.
YAML::Node load_euroc_yaml(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open " + path);
    }
    std::stringstream body;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("%YAML", 0) != 0) {
            body << line << '\n';
        }
    }
    return YAML::Load(body.str());
}

// cam0/sensor.yaml -> the backend's camera calibration: pinhole intrinsics
// (fu, fv, cu, cv), radial-tangential distortion (k1, k2, p1, p2), and the
// camera->body extrinsics T_BS (a row-major 4x4).
Backend::CameraCalibration load_camera(const std::string& mav0) {
    const YAML::Node cam = load_euroc_yaml(mav0 + "/cam0/sensor.yaml");
    const auto k = cam["intrinsics"].as<std::vector<double>>();
    const auto d = cam["distortion_coefficients"].as<std::vector<double>>();
    const auto t_bs = cam["T_BS"]["data"].as<std::vector<double>>();
    if (k.size() != 4 || d.size() != 4 || t_bs.size() != 16) {
        throw std::runtime_error("cam0/sensor.yaml: expected 4 intrinsics, 4 distortion coefficients, 16 T_BS entries");
    }

    Backend::CameraCalibration cal;
    cal.intrinsics = Backend::Camera(k[0], k[1], k[2], k[3], d[0], d[1], d[2], d[3]);
    Mat3 R{};
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            R(r, c) = t_bs[r * 4 + c];
        }
    }
    cal.extrinsics.R_imu_cam = sdk::sfm::so3_from_matrix<T>(R);
    cal.extrinsics.p_imu_cam = Vec3{{t_bs[3], t_bs[7], t_bs[11]}};

    std::printf("camera: fu=%.3f fv=%.3f cu=%.3f cv=%.3f  k1=%.5f k2=%.5f  p_imu_cam=(%.4f, %.4f, %.4f)\n",
                k[0],
                k[1],
                k[2],
                k[3],
                d[0],
                d[1],
                t_bs[3],
                t_bs[7],
                t_bs[11]);
    return cal;
}

// imu0/sensor.yaml -> the IMU noise model in VioConfig.
sdk::VioConfig load_vio_config(const std::string& mav0) {
    const YAML::Node imu = load_euroc_yaml(mav0 + "/imu0/sensor.yaml");
    sdk::VioConfig cfg;
    cfg.gyro_noise_density = imu["gyroscope_noise_density"].as<double>();
    cfg.gyro_bias_random_walk = imu["gyroscope_random_walk"].as<double>();
    cfg.accel_noise_density = imu["accelerometer_noise_density"].as<double>();
    cfg.accel_bias_random_walk = imu["accelerometer_random_walk"].as<double>();
    cfg.max_clones = 11;  // MSCKF sliding-window length
    return cfg;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <path/to/sequence/mav0> [max_frames=200]\n", argv[0]);
        return 2;
    }
    const std::string mav0 = argv[1];
    const std::size_t max_frames = argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 200;

    try {
        // 1. Configuration file -> typed SDK structs.
        const Backend::CameraCalibration cam = load_camera(mav0);
        const sdk::VioConfig config = load_vio_config(mav0);
        std::printf(
            "imu:    gyro noise %.3e  accel noise %.3e\n", config.gyro_noise_density, config.accel_noise_density);

        // 2. Construct and bring the estimator up: Unconfigured -> Inactive -> Active.
        Estimator vio(Backend(std::vector<Backend::CameraCalibration>{cam}));
        vio.configure(config);
        vio.activate();

        // 3. Replay: IMU up to each frame's timestamp, then the frame.
        const auto imu = sdk::euroc::parse_imu<T>(mav0);
        const auto frames = sdk::euroc::parse_images(mav0);
        std::printf(
            "sequence: %zu IMU samples, %zu frames (replaying up to %zu)\n", imu.size(), frames.size(), max_frames);
        std::printf("%12s %10s %10s %10s %9s %9s %9s %9s\n", "t[s]", "x[m]", "y[m]", "z[m]", "qw", "qx", "qy", "qz");

        std::size_t next_imu = 0;
        std::size_t replayed = 0;
        for (const auto& frame : frames) {
            if (replayed == max_frames) {
                break;
            }
            std::size_t end = next_imu;
            while (end < imu.size() && imu[end].timestamp_s <= frame.t_s) {
                ++end;
            }
            vio.feed_imu(std::span<const branes::sdk::ImuMeasurement<T>>{imu.data() + next_imu, end - next_imu});
            next_imu = end;

            const branes::cv::OwnedImage<std::uint8_t> image = branes::cv::read_png(frame.path);
            vio.feed_image(frame.t_s, image.view());

            // 4. Read the pose back: T_world_imu.
            const auto pose = vio.current_pose();
            const auto& p = pose.translation();
            const auto& q = pose.rotation().quaternion();  // (w, x, y, z)
            std::printf("%12.4f %10.4f %10.4f %10.4f %9.5f %9.5f %9.5f %9.5f\n",
                        frame.t_s,
                        p[0],
                        p[1],
                        p[2],
                        q[0],
                        q[1],
                        q[2],
                        q[3]);
            ++replayed;
        }

        // Leaving Active: deactivate, then release everything.
        vio.deactivate();
        vio.teardown();
        std::printf("replayed %zu frames\n", replayed);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "hello_vio: %s\n", e.what());
        return 1;
    }
    return 0;
}
