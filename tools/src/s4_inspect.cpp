// SPDX-License-Identifier: MIT
//
// s4_inspect — the S4 (visual frontend) stage inspector (epic #371, issue #374).
// The TEMPLATE per-stage inspector: real EuRoC frames → the shipped FAST + KLT
// frontend, run with full instrumentation → an enriched per-frame JSONL the
// image-domain overlay renderer (docs-site/scripts/gen-overlay.mjs) draws.
//
// It runs the real cv/ frontend operator directly (S4FrontendInspector mirrors
// VioEstimator::track_frame) so it can expose what production hides: per-track
// forward-backward residual, status, age; the FAST detections added each frame;
// the pyramid geometry; and a spatial-coverage grid.
//
// Usage:
//   s4_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N]
//              [--fast-threshold F] [--target-features N] [--pyramid-levels N]
//              [--fb-max PX] [--min-dist PX]
//
//   --dataset        sequence mav0 directory (cam0/, imu0/). Required.
//   --out            output dir (default build/stage_probes/S4)
//   --max-frames     cap frames processed
//   --fast-threshold FAST contrast threshold              (default 20)
//   --target-features re-detect below this many tracks    (default 150)
//   --pyramid-levels  KLT pyramid levels                  (default 3)
//   --fb-max         forward-backward GATE residual (px); 0 = measure but don't cull (default 0)
//   --min-dist       min pixel spacing for new detections (default 15)
//
// Then render the overlay:
//   node docs-site/scripts/gen-overlay.mjs <out>
//
// EuRoC (~1.5 GB) is not vendored, so this is a developer tool; the inspector
// logic is gated by tests/tools/s4_frontend_inspect.cpp.

#include <branes/cv/image_io.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/vio_estimator.hpp>
#include <branes/tools/s4_frontend_inspect.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>

namespace bs = branes::sdk;
namespace cv = branes::cv;

namespace {

struct Args {
    std::string dataset;
    std::string out = "build/stage_probes/S4";
    std::uint64_t max_frames = std::numeric_limits<std::uint64_t>::max();
    bs::FrontendParams fe;
    bool help = false;
};

// Parse an unsigned/decimal flag value, turning malformed input into a clean
// CLI error rather than an uncaught std::sto* exception.
std::uint64_t to_u64(const std::string& raw) {
    // std::stoull silently wraps a leading '-' (e.g. "-1" → ULLONG_MAX), which
    // would then narrow-cast into garbage for --pyramid-levels etc. Reject signs.
    if (!raw.empty() && (raw.front() == '-' || raw.front() == '+'))
        throw std::runtime_error("s4_inspect: expected a non-negative integer, got '" + raw + "'");
    std::size_t pos = 0;
    unsigned long long v = 0;
    try {
        v = std::stoull(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s4_inspect: invalid integer value '" + raw + "'");
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
        throw std::runtime_error("s4_inspect: invalid number value '" + raw + "'");
    return v;
}

Args parse(int argc, char** argv) {
    Args a;
    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("s4_inspect: missing value after " + std::string(argv[i]));
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
        else if (v == "--fast-threshold")
            a.fe.fast_threshold = to_f64(next(i));
        else if (v == "--target-features")
            a.fe.target_features = static_cast<std::size_t>(to_u64(next(i)));
        else if (v == "--pyramid-levels")
            a.fe.pyramid_levels = static_cast<int>(to_u64(next(i)));
        else if (v == "--fb-max")
            a.fe.fb_max_residual = to_f64(next(i));
        else if (v == "--min-dist")
            a.fe.min_feature_distance = to_f64(next(i));
        else
            throw std::runtime_error("s4_inspect: unknown argument '" + std::string(v) + "'");
    }
    return a;
}

void usage() {
    std::cout << "s4_inspect — S4 visual-frontend inspector (FAST + pyramidal KLT on EuRoC)\n\n"
                 "  s4_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N]\n"
                 "             [--fast-threshold F] [--target-features N] [--pyramid-levels N]\n"
                 "             [--fb-max PX] [--min-dist PX]\n\n"
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
        std::cerr << "s4_inspect: --dataset <EuRoC mav0 dir> is required.\n\n";
        usage();
        return 2;
    }

    std::vector<bs::euroc::ImageEntry> images;
    try {
        images = bs::euroc::parse_images(args.dataset);
    } catch (const std::exception& ex) {
        std::cerr << "s4_inspect: " << ex.what() << "\n";
        return 1;
    }
    if (images.empty()) {
        std::cerr << "s4_inspect: no images under " << args.dataset << "/cam0 — is this an EuRoC mav0 directory?\n";
        return 1;
    }

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    const std::string out_path = args.out + "/frames.jsonl";
    std::ofstream os(out_path);
    if (!os) {
        std::cerr << "s4_inspect: cannot write " << out_path << "\n";
        return 1;
    }

    branes::tools::S4FrontendInspector inspector(args.fe);
    std::uint64_t processed = 0, skipped = 0;
    for (const auto& frame : images) {
        if (processed >= args.max_frames)
            break;
        cv::OwnedImage<std::uint8_t> img;
        try {
            img = cv::read_png(frame.path);  // a single bad frame shouldn't abort the run
        } catch (const std::exception&) {
            ++skipped;
            continue;
        }
        const auto report = inspector.step(std::as_const(img).view(), frame.t_s, frame.path);
        os << branes::tools::to_json(report).dump() << '\n';
        ++processed;
    }

    std::cout << "s4_inspect: processed " << processed << " frames";
    if (skipped)
        std::cout << " (" << skipped << " unreadable, skipped)";
    std::cout << "\n  frames:  " << out_path << "\n"
              << "  render:  node docs-site/scripts/gen-overlay.mjs " << args.out << "\n";
    return 0;
}
