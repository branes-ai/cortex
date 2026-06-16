// SPDX-License-Identifier: MIT
//
// Gate for the per-frame trace taps (branes/tools/vio_trace_tap.hpp, issue #373).
// The real tap runs over a ~1.5 GB EuRoC sequence that CI does not vendor, so
// here we drive S4FrameTap with synthetic frames and pin the behaviour every
// inspector relies on: the previous frame's features become THIS frame's input,
// the first frame has no input, the frame index advances, and the records
// survive a write→read round-trip through the bus.

#include <branes/tools/vio_trace.hpp>
#include <branes/tools/vio_trace_tap.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <sstream>
#include <vector>

namespace tr = branes::tools::trace;
using Catch::Matchers::WithinAbs;
using Obs = branes::sdk::FrontendObservation<double>;

TEST_CASE("S4FrameTap chains previous-frame features as the next input", "[tools][vio_trace_tap]") {
    tr::S4FrameTap tap(752, 480, /*camera_id=*/0);

    std::vector<Obs> f0 = {{1, 0, 100.5, 200.5}, {2, 0, 110.0, 210.0}};
    std::vector<Obs> f1 = {{1, 0, 101.5, 201.5}, {3, 0, 300.0, 300.0}};

    REQUIRE(tap.frames_emitted() == 0);
    tr::TraceRecord r0 = tap.step(1.40, "cam0/0.png", f0);
    tr::TraceRecord r1 = tap.step(1.45, "cam0/1.png", f1);
    REQUIRE(tap.frames_emitted() == 2);

    auto in0 = tr::get_input<tr::S4Input<double>>(r0);
    auto out0 = tr::get_output<tr::S4Output<double>>(r0);
    auto in1 = tr::get_input<tr::S4Input<double>>(r1);
    auto out1 = tr::get_output<tr::S4Output<double>>(r1);

    SECTION("headers carry frame index, timestamp, and stage") {
        REQUIRE(r0.header.frame == 0);
        REQUIRE(r1.header.frame == 1);
        REQUIRE(r0.header.stage == "S4");
        REQUIRE_THAT(r1.header.t_s, WithinAbs(1.45, 0.0));
    }

    SECTION("image metadata is stamped on the input") {
        REQUIRE(in0.image_path == "cam0/0.png");
        REQUIRE(in0.width == 752);
        REQUIRE(in0.height == 480);
        REQUIRE(in0.camera_id == 0);
    }

    SECTION("first frame has no input tracks; output is this frame's features") {
        REQUIRE(in0.prev_tracks.empty());
        REQUIRE(out0.observations.size() == 2);
        REQUIRE(out0.observations[0].feature_id == 1);
        REQUIRE_FALSE(in0.gyro_prior.has_value());  // not exposed by the public API yet
    }

    SECTION("second frame's input == first frame's output") {
        REQUIRE(in1.prev_tracks.size() == f0.size());
        REQUIRE(in1.prev_tracks[0].feature_id == 1);
        REQUIRE_THAT(in1.prev_tracks[1].u, WithinAbs(110.0, 0.0));
        REQUIRE(out1.observations.size() == 2);
        REQUIRE(out1.observations[1].feature_id == 3);
    }
}

TEST_CASE("S4FrameTap records survive the JSONL round-trip", "[tools][vio_trace_tap]") {
    std::stringstream ss;
    {
        tr::TraceWriter w(ss);
        w.write_banner("S4", "tap self-test");
        tr::S4FrameTap tap(640, 480);
        std::vector<Obs> prev;
        for (std::uint64_t f = 0; f < 4; ++f) {
            std::vector<Obs> feats = {{f, 0, 10.0 + static_cast<double>(f), 20.0}};
            w.write(tap.step(1.0 + 0.1 * static_cast<double>(f), "f" + std::to_string(f) + ".png", feats));
        }
    }

    tr::TraceReader r(ss);
    auto records = r.read_all();
    REQUIRE(records.size() == 4);
    // frame f's input must equal frame f-1's output (the chaining invariant),
    // verified after a full serialize/deserialize cycle.
    for (std::size_t f = 1; f < records.size(); ++f) {
        auto in = tr::get_input<tr::S4Input<double>>(records[f]);
        auto prev_out = tr::get_output<tr::S4Output<double>>(records[f - 1]);
        REQUIRE(records[f].header.frame == f);
        REQUIRE(in.prev_tracks.size() == prev_out.observations.size());
        REQUIRE(in.prev_tracks[0].feature_id == prev_out.observations[0].feature_id);
        REQUIRE_THAT(in.prev_tracks[0].u, WithinAbs(prev_out.observations[0].u, 0.0));
    }
}
