// SPDX-License-Identifier: MIT
//
// s10_inspect — the S10 (online calibration) stage inspector (epic #371, issue
// #384). The real-data companion to the synthetic S10 probe
// (s10_online_calibration.cpp); renders on the filter-internal tier, built on the
// S4 inspector template (s4_inspect.cpp).
//
// It runs the real estimator (VioEstimator + MsckfBackend) over a real EuRoC
// sequence with online extrinsic estimation ON (estimate_extrinsics=true), seeded
// from a deliberately PERTURBED camera↔IMU extrinsic. Each frame it samples (via
// branes/tools/s10_calibration_inspect.hpp) the in-state extrinsic estimate's
// error against the dataset's true extrinsic and the estimate's σ — the
// convergence curve. The figure (docs-site/scripts/gen-calibration-figures.mjs)
// draws the rotation/translation error decaying toward the reference inside the
// shrinking ±σ band.
//
// No SDK change: online calibration is a shipped config (estimate_extrinsics) and
// backend().state() exposes the calibration block.
//
// Usage:
//   s10_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--perturb-rot-deg D]
//               [--perturb-trans-mm M] [--max-frames N]
//
//   --dataset          mav0 directory (cam0/, imu0/). Required.
//   --out              output dir (default build/stage_probes/S10)
//   --perturb-rot-deg  seed extrinsic-rotation error to converge from (default 2)
//   --perturb-trans-mm seed extrinsic-translation error to converge from (default 30)
//   --max-frames       cap frames processed
//
// Then render:
//   node docs-site/scripts/gen-calibration-figures.mjs <out>
//
// EuRoC (~1.5 GB) is not vendored, so this is a developer tool; the inspector
// logic is gated by tests/tools/s10_calibration_inspect.cpp.

#include <branes/cv/image_io.hpp>
#include <branes/math/lie/detail.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/sfm/init_window.hpp>  // so3_from_matrix
#include <branes/sdk/vio_estimator.hpp>
#include <branes/tools/s10_calibration_inspect.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bs = branes::sdk;
namespace cv = branes::cv;
namespace bt = branes::tools;
namespace lie = branes::math::lie;
using json = nlohmann::json;
using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T, Backend>;
using Vec3 = lie::detail::Vec<T, 3>;
using Mat3 = lie::detail::Mat<T, 3, 3>;
using SO3 = lie::SO3<T>;

namespace {

struct Args {
    std::string dataset;
    std::string out = "build/stage_probes/S10";
    double perturb_rot_deg = 2.0;
    double perturb_trans_mm = 30.0;
    std::uint64_t max_frames = std::numeric_limits<std::uint64_t>::max();
    bool help = false;
};

std::uint64_t to_u64(const std::string& raw) {
    if (!raw.empty() && (raw.front() == '-' || raw.front() == '+'))
        throw std::runtime_error("s10_inspect: expected a non-negative integer, got '" + raw + "'");
    std::size_t pos = 0;
    unsigned long long v = 0;
    try {
        v = std::stoull(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s10_inspect: invalid integer value '" + raw + "'");
    return v;
}
double to_f64(const std::string& raw) {
    std::size_t pos = 0;
    double v = 0;
    try {
        v = std::stod(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s10_inspect: invalid number value '" + raw + "'");
    if (!std::isfinite(v))
        throw std::runtime_error("s10_inspect: non-finite number value '" + raw + "'");
    return v;
}

Args parse(int argc, char** argv) {
    Args a;
    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("s10_inspect: missing value after " + std::string(argv[i]));
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
        else if (v == "--perturb-rot-deg") {
            a.perturb_rot_deg = to_f64(next(i));
            if (a.perturb_rot_deg < 0.0)
                throw std::runtime_error("s10_inspect: --perturb-rot-deg must be >= 0");
        } else if (v == "--perturb-trans-mm") {
            a.perturb_trans_mm = to_f64(next(i));
            if (a.perturb_trans_mm < 0.0)
                throw std::runtime_error("s10_inspect: --perturb-trans-mm must be >= 0");
        } else if (v == "--max-frames")
            a.max_frames = to_u64(next(i));
        else
            throw std::runtime_error("s10_inspect: unknown argument '" + std::string(v) + "'");
    }
    return a;
}

void usage() {
    std::cout << "s10_inspect — S10 online-calibration inspector (real EuRoC → extrinsic convergence)\n\n"
                 "  s10_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--perturb-rot-deg D]\n"
                 "              [--perturb-trans-mm M] [--max-frames N]\n\n"
                 "  Then: node docs-site/scripts/gen-calibration-figures.mjs <out>\n";
}

// The dataset's true cam0 extrinsic (the convergence reference).
SO3 euroc_R_imu_cam() {
    Mat3 R{};
    R(0, 0) = 0.0148655429818;
    R(0, 1) = -0.999880929698;
    R(0, 2) = 0.00414029679422;
    R(1, 0) = 0.999557249008;
    R(1, 1) = 0.0149672133247;
    R(1, 2) = 0.025715529948;
    R(2, 0) = -0.0257744366974;
    R(2, 1) = 0.00375618835797;
    R(2, 2) = 0.999660727178;
    return bs::sfm::so3_from_matrix<T>(R);
}
Vec3 euroc_p_imu_cam() {
    return Vec3{{-0.0216401454975, -0.064676986768, 0.00981073058949}};
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
        std::cerr << "s10_inspect: --dataset <EuRoC mav0 dir> is required.\n\n";
        usage();
        return 2;
    }

    std::vector<bs::ImuMeasurement<T>> imu;
    std::vector<bs::euroc::ImageEntry> images;
    try {
        imu = bs::euroc::parse_imu<T>(args.dataset);
        images = bs::euroc::parse_images(args.dataset);
    } catch (const std::exception& ex) {
        std::cerr << "s10_inspect: " << ex.what() << "\n";
        return 1;
    }
    if (images.empty() || imu.empty()) {
        std::cerr << "s10_inspect: empty EuRoC streams (" << images.size() << " images, " << imu.size() << " IMU)\n";
        return 1;
    }

    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    const SO3 R_ic_true = euroc_R_imu_cam();
    const Vec3 p_ic_true = euroc_p_imu_cam();
    // Seed a deliberately wrong extrinsic to converge FROM.
    const SO3 R_ic_seed = R_ic_true * SO3::exp(Vec3{{args.perturb_rot_deg * kDeg, 0.0, 0.0}});
    const Vec3 p_ic_seed{{p_ic_true[0] + args.perturb_trans_mm / 1000.0, p_ic_true[1], p_ic_true[2]}};

    Backend::CameraCalibration cal;
    cal.intrinsics =
        Backend::Camera(458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    cal.extrinsics.R_imu_cam = R_ic_seed;
    cal.extrinsics.p_imu_cam = p_ic_seed;

    bs::VioConfig cfg;
    cfg.estimate_extrinsics = true;
    // Let the prior covary the actual perturbation so the filter is free to move.
    cfg.calib_ext_rot_prior_deg = args.perturb_rot_deg * 1.5 + 0.5;
    cfg.calib_ext_trans_prior_mm = args.perturb_trans_mm * 1.5 + 5.0;

    Estimator est(Backend(std::vector<Backend::CameraCalibration>{cal}));
    est.configure(cfg);
    est.activate();

    const bt::S10CalibrationInspector inspector;
    bt::S10CalibRecord record;
    record.ref_rot_deg_init = args.perturb_rot_deg;
    record.ref_trans_mm_init = args.perturb_trans_mm;

    std::uint64_t processed = 0;
    std::size_t imu_idx = 0;
    double t0 = -1.0;
    for (const auto& frame : images) {
        if (processed >= args.max_frames)
            break;
        std::size_t end = imu_idx;
        while (end < imu.size() && imu[end].timestamp_s <= frame.t_s)
            ++end;
        if (end > imu_idx) {
            est.feed_imu(std::span<const bs::ImuMeasurement<T>>{imu.data() + imu_idx, end - imu_idx});
            imu_idx = end;
        }
        cv::OwnedImage<std::uint8_t> img;
        try {
            img = cv::read_png(frame.path);
        } catch (const std::exception&) {
            continue;
        }
        est.feed_image(frame.t_s, std::as_const(img).view());
        ++processed;
        if (!est.backend().initialized())
            continue;
        if (t0 < 0.0)
            t0 = frame.t_s;
        if (!est.backend().state().calib.empty())
            record.has_calib = true;
        record.curve.push_back(inspector.sample(est.backend().state(), R_ic_true, p_ic_true, frame.t_s - t0));
    }

    if (record.curve.empty()) {
        std::cerr << "s10_inspect: filter did not initialize within " << processed << " frames\n";
        return 1;
    }
    if (!record.has_calib) {
        std::cerr << "s10_inspect: no calibration block was created (estimate_extrinsics off?)\n";
        return 1;
    }

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    const std::string out_path = args.out + "/calibration.json";
    std::ofstream os(out_path);
    if (!os) {
        std::cerr << "s10_inspect: cannot write " << out_path << "\n";
        return 1;
    }
    os << bt::to_json(record).dump(1) << '\n';
    if (!os) {
        std::cerr << "s10_inspect: error writing " << out_path << "\n";
        return 1;
    }

    const auto& first = record.curve.front();
    const auto& last = record.curve.back();
    std::cout << "s10_inspect: processed " << processed << " frames, " << record.curve.size() << " post-init samples\n"
              << "  seeded:   extrinsic perturbed by " << args.perturb_rot_deg << " deg / " << args.perturb_trans_mm
              << " mm\n"
              << "  converge: rot err " << first.rot_err_deg << " → " << last.rot_err_deg << " deg, trans err "
              << first.trans_err_mm << " → " << last.trans_err_mm << " mm\n"
              << "  sigma:    rot σ " << first.rot_sigma_deg << " → " << last.rot_sigma_deg << " deg\n"
              << "  out:      " << out_path << "\n"
              << "  render:   node docs-site/scripts/gen-calibration-figures.mjs " << args.out << "\n";
    return 0;
}
