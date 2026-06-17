// SPDX-License-Identifier: MIT
//
// s1_inspect — the S1 (initialization / bootstrap) stage inspector (epic #371,
// issue #376). The real-data companion to the synthetic S1 probe
// (s1_initialization.cpp); renders on the filter-internal tier, built on the S4
// inspector template (s4_inspect.cpp).
//
// It runs the REAL estimator (VioEstimator + MsckfBackend) over a real EuRoC
// sequence and watches backend().initialized() flip false→true. At that moment
// it snapshots the seeded state + 15×15 covariance + init diagnostics, compares
// the recovered gravity / scale / velocity / biases against EuRoC ground truth
// (parse_groundtruth_states, which is metric), and writes init.json. The figures
// (docs-site/scripts/gen-init-figures.mjs) draw the recovered-state-vs-GT panel
// and the initial-covariance heatmap.
//
// No SDK change: the init operators are already decoupled and the backend exposes
// initialized()/state()/init_diagnostics() — polling per frame is sufficient.
//
// Usage:
//   s1_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N] [--prefer-dynamic]
//
//   --dataset         sequence mav0 directory (cam0/, imu0/, state_groundtruth_estimate0/). Required.
//   --out             output dir (default build/stage_probes/S1)
//   --max-frames      cap frames fed while waiting for init
//   --prefer-dynamic  skip the static window and bootstrap from the dynamic VI alignment
//
// Then render:
//   node docs-site/scripts/gen-init-figures.mjs <out>
//
// EuRoC (~1.5 GB) is not vendored, so this is a developer tool; the inspector
// logic is gated by tests/tools/s1_init_inspect.cpp.

#include <branes/cv/image_io.hpp>
#include <branes/math/lie/detail.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/sfm/init_window.hpp>  // so3_from_matrix
#include <branes/sdk/vio_estimator.hpp>
#include <branes/tools/s1_init_inspect.hpp>

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
namespace ev = branes::sdk::eval;
using json = nlohmann::json;
using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T, Backend>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using Mat3 = branes::math::lie::detail::Mat<T, 3, 3>;

namespace {

struct Args {
    std::string dataset;
    std::string out = "build/stage_probes/S1";
    std::uint64_t max_frames = std::numeric_limits<std::uint64_t>::max();
    bool prefer_dynamic = false;
    bool help = false;
};

std::uint64_t to_u64(const std::string& raw) {
    if (!raw.empty() && (raw.front() == '-' || raw.front() == '+'))
        throw std::runtime_error("s1_inspect: expected a non-negative integer, got '" + raw + "'");
    std::size_t pos = 0;
    unsigned long long v = 0;
    try {
        v = std::stoull(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s1_inspect: invalid integer value '" + raw + "'");
    return v;
}

Args parse(int argc, char** argv) {
    Args a;
    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("s1_inspect: missing value after " + std::string(argv[i]));
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
        else if (v == "--max-frames")
            a.max_frames = to_u64(next(i));
        else if (v == "--prefer-dynamic")
            a.prefer_dynamic = true;
        else
            throw std::runtime_error("s1_inspect: unknown argument '" + std::string(v) + "'");
    }
    return a;
}

void usage() {
    std::cout << "s1_inspect — S1 initialization inspector (real EuRoC bootstrap → initial state & covariance)\n\n"
                 "  s1_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N] [--prefer-dynamic]\n\n"
                 "  Then: node docs-site/scripts/gen-init-figures.mjs <out>\n";
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

// Nearest ground-truth nav state to time `t_s` (GT is dense vs the camera rate).
const ev::NavSample<T>* nearest_nav(const std::vector<bs::euroc::GroundTruthState<T>>& gt, double t_s) {
    if (gt.empty())
        return nullptr;
    std::size_t best = 0;
    double bestd = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < gt.size(); ++i) {
        const double d = gt[i].t_s - t_s < 0 ? t_s - gt[i].t_s : gt[i].t_s - t_s;
        if (d < bestd) {
            bestd = d;
            best = i;
        }
    }
    return &gt[best].nav;
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
        std::cerr << "s1_inspect: --dataset <EuRoC mav0 dir> is required.\n\n";
        usage();
        return 2;
    }

    std::vector<bs::ImuMeasurement<T>> imu;
    std::vector<bs::euroc::ImageEntry> images;
    std::vector<bs::euroc::GroundTruthState<T>> gt;
    try {
        imu = bs::euroc::parse_imu<T>(args.dataset);
        images = bs::euroc::parse_images(args.dataset);
        try {
            gt = bs::euroc::parse_groundtruth_states<T>(args.dataset);  // optional — comparison only
        } catch (const std::exception& gex) {
            // Ground truth is optional, but a parse FAILURE must not look like an
            // intentional no-GT run — surface it so a malformed file is visible.
            gt.clear();
            std::cerr << "s1_inspect: ground truth unavailable (" << gex.what()
                      << ") — proceeding without GT comparison\n";
        }
    } catch (const std::exception& ex) {
        std::cerr << "s1_inspect: " << ex.what() << "\n";
        return 1;
    }
    if (images.empty() || imu.empty()) {
        std::cerr << "s1_inspect: empty EuRoC streams (" << images.size() << " images, " << imu.size() << " IMU)\n";
        return 1;
    }

    const auto cal = euroc_cam0();
    bs::VioConfig cfg;
    cfg.prefer_dynamic_init = args.prefer_dynamic;

    Estimator est(Backend(std::vector<Backend::CameraCalibration>{cal}));
    est.configure(cfg);
    est.activate();

    // Replay until init flips false→true, then capture the SEEDED state. The check
    // runs both right after IMU ingestion and right after image processing: on the
    // static path init completes during process_imu (before any clone is augmented),
    // so capturing post-IMU snapshots the pristine 15×15 seed rather than a state
    // already grown by the first frame's clone; the dynamic path completes during
    // process_camera, caught post-image.
    std::uint64_t processed = 0;
    std::size_t imu_idx = 0;
    bool captured = false;
    bt::S1InitRecord record;
    auto try_capture = [&](double t_at) -> bool {
        if (captured || !est.backend().initialized())
            return false;
        const auto& diag = est.backend().init_diagnostics();
        const ev::NavSample<T>* gt_nav = nearest_nav(gt, diag.t_s > 0 ? diag.t_s : t_at);
        record = bt::S1InitInspector().build(diag, est.backend().state(), gt_nav);
        captured = true;
        return true;
    };
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
        if (try_capture(frame.t_s))  // static path: init during IMU ingestion → pristine seed
            break;
        cv::OwnedImage<std::uint8_t> img;
        try {
            img = cv::read_png(frame.path);
        } catch (const std::exception&) {
            continue;
        }
        est.feed_image(frame.t_s, std::as_const(img).view());
        ++processed;
        if (try_capture(frame.t_s))  // dynamic path: init during image processing
            break;
    }

    if (!captured) {
        std::cerr << "s1_inspect: filter did not initialize within " << processed << " frames "
                  << "(try more --max-frames, or --prefer-dynamic for a moving start)\n";
        return 1;
    }

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    const std::string out_path = args.out + "/init.json";
    std::ofstream os(out_path);
    if (!os) {
        std::cerr << "s1_inspect: cannot write " << out_path << "\n";
        return 1;
    }
    os << bt::to_json(record).dump(1) << '\n';
    if (!os) {
        std::cerr << "s1_inspect: error writing " << out_path << "\n";
        return 1;
    }

    std::cout << "s1_inspect: initialized via '" << record.method << "' at t=" << record.t_s << "s after " << processed
              << " frames\n";
    std::cout << "  attitude: roll " << record.roll_deg << "  pitch " << record.pitch_deg << "  yaw " << record.yaw_deg
              << " deg\n";
    if (record.have_gt)
        std::cout << "  vs GT:    gravity-dir err " << record.gravity_dir_error_deg << " deg, speed err "
                  << record.velocity_error_ms << " m/s, gyro-bias err " << record.gyro_bias_error << " rad/s"
                  << (record.scale_error_pct > 0 ? ", scale err " + std::to_string(record.scale_error_pct) + "%" : "")
                  << "\n";
    std::cout << "  seed:     initial-P is " << (record.isotropic_seed ? "ISOTROPIC σ·I" : "structured") << " (θ σ "
              << record.sigma_theta_deg << " deg, pos σ " << record.sigma_pos_m << " m)\n"
              << "  init:     " << out_path << "\n"
              << "  render:   node docs-site/scripts/gen-init-figures.mjs " << args.out << "\n";
    return 0;
}
