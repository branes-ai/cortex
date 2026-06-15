// SPDX-License-Identifier: MIT
//
// asl_trace — dump per-stage VIO trace records from ONE real EuRoC run
// (epic #371, issue #373).
//
// Runs the real estimator (VioEstimator + MsckfBackend) over an EuRoC ASL
// sequence via branes::sdk::euroc::replay, taps each stage boundary through the
// replay's on_frame hook, and writes trace-bus JSONL (branes/tools/vio_trace.hpp)
// — the real, ground-truthed traces every per-stage inspector replays from.
//
// Layering: this lives in the tools layer because the trace bus does. The SDK
// must not depend on tools, so the tap drives the estimator from the OUTSIDE
// (the on_frame hook + the estimator's public accessors) rather than
// instrumenting it. As each operator is decoupled (#374–#384) its boundary tap
// joins S4 here; today the frontend (S4) is the worked template.
//
// Usage:
//   asl_trace --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N]
//
//   --dataset   path to a sequence's mav0 directory (with cam0/, imu0/). Required.
//   --out       output dir for the trace files   (default build/stage_probes/trace)
//   --max-frames  cap emitted records (the run still completes; for quick peeks)
//
// EuRoC is ~1.5 GB and not vendored, so this is a developer tool, not a CI test;
// the schema/tap logic is gated by tests/tools/vio_trace*.cpp.

#include <branes/cv/image_io.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/vio_estimator.hpp>
#include <branes/tools/vio_trace.hpp>
#include <branes/tools/vio_trace_tap.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace bs = branes::sdk;
namespace cv = branes::cv;
namespace tr = branes::tools::trace;

namespace {

using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T, Backend>;

struct Args {
    std::string dataset;
    std::string out = "build/stage_probes/trace";
    std::uint64_t max_frames = std::numeric_limits<std::uint64_t>::max();
    bool help = false;
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string_view v = argv[i];
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--dataset" && i + 1 < argc)
            a.dataset = argv[++i];
        else if (v == "--out" && i + 1 < argc)
            a.out = argv[++i];
        else if (v == "--max-frames" && i + 1 < argc)
            a.max_frames = std::stoull(argv[++i]);
    }
    return a;
}

void usage() {
    std::cout << "asl_trace — dump per-stage VIO trace records from one EuRoC run\n\n"
                 "  asl_trace --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N]\n\n"
                 "  --dataset     sequence mav0 directory (cam0/, imu0/). Required.\n"
                 "  --out         output dir (default build/stage_probes/trace)\n"
                 "  --max-frames  cap emitted records (the run still completes)\n";
}

// The frontend reports features as id + pixel; lift them to the backend-agnostic
// observation the trace bus speaks (camera 0 — this driver is mono).
std::vector<bs::FrontendObservation<T>> to_observations(const Estimator& est) {
    std::vector<bs::FrontendObservation<T>> obs;
    const auto feats = est.tracked_features();
    obs.reserve(feats.size());
    for (const auto& f : feats)
        obs.push_back(bs::FrontendObservation<T>{f.id, 0, static_cast<T>(f.x), static_cast<T>(f.y)});
    return obs;
}

}  // namespace

int main(int argc, char** argv) {
    Args args = parse(argc, argv);
    if (args.help) {
        usage();
        return 0;
    }
    if (args.dataset.empty()) {
        std::cerr << "asl_trace: --dataset <EuRoC mav0 dir> is required.\n\n";
        usage();
        return 2;
    }

    // Resolve the image list up front: it gives the path for each frame (the
    // on_frame hook only hands us a timestamp) and the image dimensions.
    // parse_images throws if the CSV is missing — report it instead of aborting.
    std::vector<bs::euroc::ImageEntry> images;
    try {
        images = bs::euroc::parse_images(args.dataset);
    } catch (const std::exception& ex) {
        std::cerr << "asl_trace: " << ex.what() << "\n";
        return 1;
    }
    if (images.empty()) {
        std::cerr << "asl_trace: no images under " << args.dataset << "/cam0 — is this an EuRoC mav0 directory?\n";
        return 1;
    }
    std::unordered_map<double, std::string> path_at;
    path_at.reserve(images.size());
    for (const auto& e : images)
        path_at.emplace(e.t_s, e.path);

    // EuRoC cameras are fixed-resolution per sequence; read the first frame once
    // for the dimensions rather than re-decoding every frame in the hook.
    std::uint32_t width = 0, height = 0;
    try {
        const auto first = cv::read_png(images.front().path);
        width = static_cast<std::uint32_t>(first.width());
        height = static_cast<std::uint32_t>(first.height());
    } catch (const std::exception& ex) {
        std::cerr << "asl_trace: cannot read " << images.front().path << ": " << ex.what() << "\n";
        return 1;
    }

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    const std::string s4_path = args.out + "/s4_frontend.trace.jsonl";
    std::ofstream s4_os(s4_path);
    if (!s4_os) {
        std::cerr << "asl_trace: cannot write " << s4_path << "\n";
        return 1;
    }

    tr::TraceWriter s4_writer(s4_os);
    s4_writer.write_banner("S4", "Visual frontend (track generation) — real EuRoC run, per-frame I/O");
    tr::S4FrameTap s4_tap(width, height, /*camera_id=*/0);

    const auto on_frame = [&](double t_s, const Estimator& est) {
        if (s4_tap.frames_emitted() >= args.max_frames)
            return;
        auto it = path_at.find(t_s);
        const std::string path = it != path_at.end() ? it->second : std::string{};
        s4_writer.write(s4_tap.step(t_s, path, to_observations(est)));
    };

    Estimator est;
    bs::VioConfig cfg;  // defaults — the frontend tracks regardless of backend init
    std::size_t frames = 0;
    try {
        // replay also parses imu0/data.csv, which throws if the sequence is
        // incomplete; surface that cleanly (the partial trace stays on disk).
        frames = bs::euroc::replay(args.dataset, est, cfg, on_frame).size();
    } catch (const std::exception& ex) {
        std::cerr << "asl_trace: " << ex.what() << "\n";
        return 1;
    }

    std::cout << "asl_trace: replayed " << frames << " frames; wrote " << s4_tap.frames_emitted() << " S4 records\n"
              << "  trace:   " << s4_path << "\n"
              << "  inspect: head -2 " << s4_path << "\n";
    return 0;
}
