// SPDX-License-Identifier: MIT
//
// vio_trace_demo — write a tiny, self-describing VIO trace and read it back.
//
// A study companion to branes/tools/vio_trace.hpp (the inter-stage trace bus,
// #372): it fabricates a couple of S4 (visual frontend) stage-boundary records,
// writes them to a JSONL file with the self-describing banner, then replays the
// file and prints what it recovered. The point is to SEE the schema — run it,
// then `cat` the .jsonl and read a record without reading any code.
//
// Usage:  vio_trace_demo [--out <dir>]      (default build/stage_probes/trace)
//
// This is NOT a test (that gate is tests/tools/vio_trace.cpp); it is the
// human-readable demonstration of the format.

#include <branes/tools/stage_probe.hpp>  // ProbeArgs / parse_args (the tools CLI shell)
#include <branes/tools/vio_trace.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>

using namespace branes::tools;
namespace tr = branes::tools::trace;

namespace {

// A plausible S4 boundary: a frame of tracks carried in, a slightly-moved set
// surviving as the output observations.
tr::TraceRecord make_frame(std::uint64_t frame, double t_s) {
    tr::S4Input<double> in;
    in.image_path = "euroc/MH_05/cam0/" + std::to_string(frame) + ".png";
    in.width = 752;
    in.height = 480;
    in.camera_id = 0;
    in.prev_tracks = {{10, 0, 320.5, 240.0}, {11, 0, 410.0, 180.25}, {12, 0, 150.75, 360.5}};
    in.gyro_prior = branes::math::lie::SO3<double>{};  // identity prior

    tr::S4Output<double> out;
    out.observations = {{10, 0, 322.0, 241.5}, {11, 0, 408.5, 181.0}};  // 12 dropped by RANSAC

    return tr::make_record(tr::TraceHeader{frame, t_s, "S4"}, in, out);
}

}  // namespace

int main(int argc, char** argv) {
    // A throwaway StageInfo drives the shared CLI shell (parse_args): its id is
    // the default artifact dir (build/stage_probes/trace) and its title shows in
    // --help. The S4 banner text below is passed separately.
    StageInfo meta{"trace", "VIO trace bus demo (S4 frontend records)", "", {}, {}, {}, {}, "", ""};
    ProbeArgs args = parse_args(argc, argv, meta);
    if (args.out.empty())  // --no-out: still write somewhere the demo can read back
        args.out = "build/stage_probes/trace";

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);
    const std::string path = args.out + "/s4_frontend.trace.jsonl";

    // ── Write ───────────────────────────────────────────────────────────────
    {
        std::ofstream os(path);
        if (!os) {
            std::cerr << "vio_trace_demo: cannot write " << path << "\n";
            return 1;
        }
        tr::TraceWriter w(os);
        w.write_banner("S4", "Visual frontend (track generation) — per-frame input/output");
        for (std::uint64_t f = 0; f < 3; ++f)
            w.write(make_frame(f, 1.40 + 0.05 * static_cast<double>(f)));
    }
    std::cout << "wrote trace: " << path << "\n";
    std::cout << "  inspect it:  cat " << path << "\n\n";

    // ── Read back ─────────────────────────────────────────────────────────────
    std::ifstream is(path);
    if (!is) {
        std::cerr << "vio_trace_demo: cannot read " << path << "\n";
        return 1;
    }
    tr::TraceReader r(is);
    auto records = r.read_all();
    std::cout << "replayed " << records.size() << " S4 records:\n";
    for (const auto& rec : records) {
        auto in = tr::get_input<tr::S4Input<double>>(rec);
        auto out = tr::get_output<tr::S4Output<double>>(rec);
        std::cout << "  frame " << rec.header.frame << " @ " << rec.header.t_s << "s  [" << rec.header.stage << "]  "
                  << in.prev_tracks.size() << " tracks in (" << in.image_path << ") → " << out.observations.size()
                  << " observations out\n";
    }
    return 0;
}
