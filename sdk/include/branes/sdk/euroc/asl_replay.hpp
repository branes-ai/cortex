// SPDX-License-Identifier: MIT
//
// branes/sdk/euroc/asl_replay.hpp — EuRoC MAV (ASL-format) dataset replay.
//
// Parses the ASL CSV layout — imu0/data.csv (gyro+accel), cam0/data.csv
// (image timestamps + filenames), and state_groundtruth_estimate0/data.csv
// (T_world_body) — and drives a VioEstimator with the time-ordered stream:
// for each image, all IMU samples up to its timestamp are fed first, then
// the frame, and the resulting pose is recorded. Returns the estimated
// trajectory for comparison against ground truth (see eval/).
//
// `dataset_root` is the sequence's `mav0` directory. Clean-room from the
// published EuRoC ASL format description. Header-only, C++20.

#ifndef BRANES_SDK_EUROC_ASL_REPLAY_HPP
#define BRANES_SDK_EUROC_ASL_REPLAY_HPP

#include <branes/cv/image.hpp>
#include <branes/cv/image_io.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace branes::sdk::euroc {

/// One camera-frame entry: timestamp (s) and the image file path.
struct ImageEntry {
    double t_s = 0.0;
    std::string path;
};

namespace detail {

/// Split one CSV line into trimmed comma-separated fields.
inline std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        const auto a = field.find_first_not_of(" \t\r\n");
        const auto b = field.find_last_not_of(" \t\r\n");
        out.push_back(a == std::string::npos ? std::string{} : field.substr(a, b - a + 1));
    }
    return out;
}

/// Read non-comment, non-empty data lines (ASL files start with a '#'
/// header row).
inline std::vector<std::string> data_lines(const std::string& path) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("asl_replay: cannot open " + path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        lines.push_back(line);
    }
    return lines;
}

inline double ns_to_s(const std::string& ns) {
    return static_cast<double>(std::stoll(ns)) * 1e-9;
}

/// Parse a finite floating-point field; reject NaN/Inf so malformed CSV
/// values can't propagate into the estimator's integration or the
/// trajectory alignment.
inline double parse_finite(const std::string& field) {
    const double v = std::stod(field);
    if (!std::isfinite(v))
        throw std::runtime_error("asl_replay: non-finite value '" + field + "'");
    return v;
}

}  // namespace detail

/// Parse imu0/data.csv → ascending IMU samples (gyro rad/s, accel m/s²).
template <math::Scalar T>
[[nodiscard]] std::vector<ImuMeasurement<T>> parse_imu(const std::string& dataset_root) {
    std::vector<ImuMeasurement<T>> out;
    for (const auto& ln : detail::data_lines(dataset_root + "/imu0/data.csv")) {
        const auto f = detail::split_csv(ln);
        if (f.size() < 7)
            continue;
        ImuMeasurement<T> m;
        m.timestamp_s = detail::ns_to_s(f[0]);
        m.angular_velocity = Vec3<T>{{static_cast<T>(detail::parse_finite(f[1])),
                                      static_cast<T>(detail::parse_finite(f[2])),
                                      static_cast<T>(detail::parse_finite(f[3]))}};
        m.linear_acceleration = Vec3<T>{{static_cast<T>(detail::parse_finite(f[4])),
                                         static_cast<T>(detail::parse_finite(f[5])),
                                         static_cast<T>(detail::parse_finite(f[6]))}};
        out.push_back(m);
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.timestamp_s < b.timestamp_s; });
    return out;
}

/// Parse cam0/data.csv → ascending image entries (path = cam0/data/<file>).
[[nodiscard]] inline std::vector<ImageEntry> parse_images(const std::string& dataset_root) {
    std::vector<ImageEntry> out;
    for (const auto& ln : detail::data_lines(dataset_root + "/cam0/data.csv")) {
        const auto f = detail::split_csv(ln);
        if (f.size() < 2)
            continue;
        out.push_back(ImageEntry{detail::ns_to_s(f[0]), dataset_root + "/cam0/data/" + f[1]});
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.t_s < b.t_s; });
    return out;
}

/// Parse state_groundtruth_estimate0/data.csv → T_world_body poses.
template <math::Scalar T>
[[nodiscard]] std::vector<eval::StampedPose<T>> parse_groundtruth(const std::string& dataset_root) {
    std::vector<eval::StampedPose<T>> out;
    for (const auto& ln : detail::data_lines(dataset_root + "/state_groundtruth_estimate0/data.csv")) {
        const auto f = detail::split_csv(ln);
        if (f.size() < 8)
            continue;
        const math::lie::detail::Vec<T, 3> p{{static_cast<T>(detail::parse_finite(f[1])),
                                              static_cast<T>(detail::parse_finite(f[2])),
                                              static_cast<T>(detail::parse_finite(f[3]))}};
        // ASL quaternion order is (w, x, y, z).
        const double qw = detail::parse_finite(f[4]), qx = detail::parse_finite(f[5]);
        const double qy = detail::parse_finite(f[6]), qz = detail::parse_finite(f[7]);
        if (!(qw * qw + qx * qx + qy * qy + qz * qz > 0.0))
            throw std::runtime_error("asl_replay: zero-norm ground-truth quaternion");
        const typename math::lie::SO3<T>::Quaternion q{
            {static_cast<T>(qw), static_cast<T>(qx), static_cast<T>(qy), static_cast<T>(qz)}};
        eval::StampedPose<T> sp;
        sp.t_s = detail::ns_to_s(f[0]);
        sp.pose = math::lie::SE3<T>(math::lie::SO3<T>(q), p);
        out.push_back(sp);
    }
    std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.t_s < b.t_s; });
    return out;
}

/// Drive `est` through the sequence and return its estimated trajectory.
/// The estimator is configured and activated here. Images are loaded as
/// grayscale PNGs on demand.
template <class Estimator>
[[nodiscard]] std::vector<eval::StampedPose<typename Estimator::Scalar>>
replay(const std::string& dataset_root, Estimator& est, const VioConfig& config) {
    using T = typename Estimator::Scalar;
    const auto imu = parse_imu<T>(dataset_root);
    const auto images = parse_images(dataset_root);

    est.configure(config);
    est.activate();

    std::vector<eval::StampedPose<T>> traj;
    traj.reserve(images.size());
    std::size_t imu_idx = 0;
    for (const auto& frame : images) {
        // Feed all IMU samples up to (and including) the frame time.
        std::size_t end = imu_idx;
        while (end < imu.size() && imu[end].timestamp_s <= frame.t_s)
            ++end;
        if (end > imu_idx) {
            est.feed_imu(std::span<const ImuMeasurement<T>>{imu.data() + imu_idx, end - imu_idx});
            imu_idx = end;
        }
        // A single unreadable/corrupt frame shouldn't abort the whole
        // sequence — skip it (IMU has already advanced) and continue.
        cv::OwnedImage<std::uint8_t> img;
        try {
            img = cv::read_png(frame.path);
        } catch (const std::exception&) {
            continue;
        }
        est.feed_image(frame.t_s, std::as_const(img).view());

        eval::StampedPose<T> sp;
        sp.t_s = frame.t_s;
        sp.pose = est.current_pose();
        traj.push_back(sp);
    }
    return traj;
}

}  // namespace branes::sdk::euroc

#endif  // BRANES_SDK_EUROC_ASL_REPLAY_HPP
