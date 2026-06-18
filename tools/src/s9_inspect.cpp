// SPDX-License-Identifier: MIT
//
// s9_inspect — the S9 (marginalization / clone management) stage inspector (epic
// #371, issue #383). The real-data companion to the synthetic S9 probe
// (s9_marginalization.cpp); renders on the filter-internal tier, built on the S4
// inspector template (s4_inspect.cpp). It is the inverse of s3_inspect.
//
// It runs the real estimator (VioEstimator + MsckfBackend) over a real EuRoC
// sequence until the window holds a genuinely-correlated covariance (initialized,
// with a few clones), then takes a COPY of that keyframe state and runs the real
// `StateHelper::marginalize_clone` on it (via branes/tools/
// s9_marginalization_inspect.hpp). It records the before/after covariance and the
// marginalization invariant — the kept-state marginal must be unchanged — plus the
// information the dropped clone carried. The figure (docs-site/scripts/
// gen-marginalization-figures.mjs) draws the before/after heatmap with the dropped
// block outlined.
//
// No SDK change: marginalize_clone is already standalone and backend().state() is
// exposed, so the inspector marginalizes a copy of the live state.
//
// Usage:
//   s9_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--min-clones N] [--idx K] [--max-frames N]
//
//   --dataset     mav0 directory (cam0/, imu0/). Required.
//   --out         output dir (default build/stage_probes/S9)
//   --min-clones  capture once the window holds this many clones (default 5)
//   --idx         which clone (window index) to drop (default 0 = oldest, as the backend does)
//   --max-frames  give up waiting after this many frames
//
// Then render:
//   node docs-site/scripts/gen-marginalization-figures.mjs <out>
//
// EuRoC (~1.5 GB) is not vendored, so this is a developer tool; the inspector
// logic is gated by tests/tools/s9_marginalization_inspect.cpp.

#include <branes/cv/image_io.hpp>
#include <branes/math/lie/detail.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/sfm/init_window.hpp>  // so3_from_matrix
#include <branes/sdk/vio_estimator.hpp>
#include <branes/tools/s9_marginalization_inspect.hpp>

#include <nlohmann/json.hpp>

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
namespace ms = branes::sdk::msckf;
using json = nlohmann::json;
using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T, Backend>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using Mat3 = branes::math::lie::detail::Mat<T, 3, 3>;

namespace {

struct Args {
    std::string dataset;
    std::string out = "build/stage_probes/S9";
    std::size_t min_clones = 5;
    std::size_t idx = 0;
    std::uint64_t max_frames = std::numeric_limits<std::uint64_t>::max();
    bool help = false;
};

std::uint64_t to_u64(const std::string& raw) {
    if (!raw.empty() && (raw.front() == '-' || raw.front() == '+'))
        throw std::runtime_error("s9_inspect: expected a non-negative integer, got '" + raw + "'");
    std::size_t pos = 0;
    unsigned long long v = 0;
    try {
        v = std::stoull(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s9_inspect: invalid integer value '" + raw + "'");
    return v;
}

Args parse(int argc, char** argv) {
    Args a;
    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("s9_inspect: missing value after " + std::string(argv[i]));
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
        else if (v == "--min-clones")
            a.min_clones = static_cast<std::size_t>(to_u64(next(i)));
        else if (v == "--idx")
            a.idx = static_cast<std::size_t>(to_u64(next(i)));
        else if (v == "--max-frames")
            a.max_frames = to_u64(next(i));
        else
            throw std::runtime_error("s9_inspect: unknown argument '" + std::string(v) + "'");
    }
    return a;
}

void usage() {
    std::cout << "s9_inspect — S9 marginalization inspector (real EuRoC clone drop → reduced covariance)\n\n"
                 "  s9_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--min-clones N] [--idx K] [--max-frames N]\n\n"
                 "  Then: node docs-site/scripts/gen-marginalization-figures.mjs <out>\n";
}

Backend::CameraCalibration euroc_cam0() {
    Backend::CameraCalibration cal;
    cal.intrinsics =
        Backend::Camera(458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
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
    cal.extrinsics.R_imu_cam = bs::sfm::so3_from_matrix<T>(R);
    cal.extrinsics.p_imu_cam = Vec3{{-0.0216401454975, -0.064676986768, 0.00981073058949}};
    return cal;
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
        std::cerr << "s9_inspect: --dataset <EuRoC mav0 dir> is required.\n\n";
        usage();
        return 2;
    }

    std::vector<bs::ImuMeasurement<T>> imu;
    std::vector<bs::euroc::ImageEntry> images;
    try {
        imu = bs::euroc::parse_imu<T>(args.dataset);
        images = bs::euroc::parse_images(args.dataset);
    } catch (const std::exception& ex) {
        std::cerr << "s9_inspect: " << ex.what() << "\n";
        return 1;
    }
    if (images.empty() || imu.empty()) {
        std::cerr << "s9_inspect: empty EuRoC streams (" << images.size() << " images, " << imu.size() << " IMU)\n";
        return 1;
    }
    if (args.idx >= args.min_clones) {
        std::cerr << "s9_inspect: --idx " << args.idx << " must be < --min-clones " << args.min_clones << "\n";
        return 2;
    }

    const auto cal = euroc_cam0();
    bs::VioConfig cfg;
    Estimator est(Backend(std::vector<Backend::CameraCalibration>{cal}));
    est.configure(cfg);
    est.activate();

    std::uint64_t processed = 0;
    std::size_t imu_idx = 0;
    bool captured = false;
    bt::S9MarginalizeRecord record;
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

        if (est.backend().initialized() && est.backend().state().clones.size() >= args.min_clones) {
            bt::S9MarginalizationInspector::S9Input in{est.backend().state(), args.idx};  // copy the live state
            record = bt::S9MarginalizationInspector().run(in);
            captured = true;
            break;
        }
    }

    if (!captured) {
        std::cerr << "s9_inspect: did not reach " << args.min_clones << " clones within " << processed
                  << " frames (try a longer sequence or fewer --min-clones)\n";
        return 1;
    }

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    const std::string out_path = args.out + "/marginalization.json";
    std::ofstream os(out_path);
    if (!os) {
        std::cerr << "s9_inspect: cannot write " << out_path << "\n";
        return 1;
    }
    os << bt::to_json(record).dump(1) << '\n';
    if (!os) {
        std::cerr << "s9_inspect: error writing " << out_path << "\n";
        return 1;
    }

    std::cout << "s9_inspect: captured at " << processed << " frames, " << record.n_clones_before
              << " clones in window\n"
              << "  marginalize: drop clone " << record.dropped_index << " (offset " << record.dropped_offset
              << "); dim " << record.dim_before << " → " << record.dim_after << "\n"
              << "  contract:    kept-marginal err " << record.kept_marginal_err << ", PSD "
              << (record.psd ? "yes" : "NO") << "\n"
              << "  dropped:     σ " << record.dropped_sigma << ", max cross-cov " << record.max_cross_dropped
              << " (info discarded)\n"
              << "  out:         " << out_path << "\n"
              << "  render:      node docs-site/scripts/gen-marginalization-figures.mjs " << args.out << "\n";
    return 0;
}
