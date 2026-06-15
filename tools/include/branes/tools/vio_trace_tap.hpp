// SPDX-License-Identifier: MIT
//
// branes/tools/vio_trace_tap.hpp — stateful per-frame taps that turn a live VIO
// replay into trace-bus records (epic #371, issue #373).
//
// The trace bus (vio_trace.hpp) defines the schema; the taps here are the glue
// that captures a real run into it. They live in the tools layer on purpose: the
// SDK must not depend on tools (the dependency is tools→sdk only), so the tap
// drives the SDK from the outside — through the `on_frame(t_s, est)` hook that
// `branes::sdk::euroc::replay` already exposes — rather than instrumenting the
// estimator. As each stage operator is decoupled (#374–#384), its boundary gets
// a tap here alongside S4.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_VIO_TRACE_TAP_HPP
#define BRANES_TOOLS_VIO_TRACE_TAP_HPP

#include <branes/tools/vio_trace.hpp>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace branes::tools::trace {

/// Per-frame tap for the S4 (visual frontend) boundary.
///
/// S4's contract is `track(image_t, prev_tracks) → observations`, but a forward
/// replay only ever surfaces the CURRENT frame's tracked features. So the tap
/// remembers the previous frame's features and replays them as this frame's
/// input — making the boundary self-contained for an inspector that re-runs the
/// frontend. The first frame therefore has empty `prev_tracks` (nothing has been
/// tracked in yet), which is the honest state at sequence start.
///
/// `gyro_prior` is left unset: the public estimator API does not expose the
/// per-frame rotation prior (S0/S2 → frontend) yet, and inventing one would lie
/// about what the run actually had. It becomes available when S2's tap lands.
class S4FrameTap {
public:
    S4FrameTap(std::uint32_t width, std::uint32_t height, std::uint32_t camera_id = 0)
        : width_(width), height_(height), camera_id_(camera_id) {}

    /// Build the S4 record for one frame. `features` are the features the
    /// frontend is tracking AFTER this frame (stable id + pixel u,v). Returns
    /// the record (the caller writes it), then rolls `features` into the prev
    /// slot and advances the frame counter.
    [[nodiscard]] TraceRecord
    step(double t_s, std::string image_path, std::vector<sdk::FrontendObservation<double>> features) {
        S4Input<double> in;
        in.image_path = std::move(image_path);
        in.width = width_;
        in.height = height_;
        in.camera_id = camera_id_;
        in.prev_tracks = prev_;  // empty on the first frame

        S4Output<double> out;
        out.observations = features;

        TraceRecord rec = make_record(TraceHeader{frame_, t_s, "S4"}, in, out);
        prev_ = std::move(features);
        ++frame_;
        return rec;
    }

    /// Number of frames tapped so far (also the next frame's index).
    [[nodiscard]] std::uint64_t frames_emitted() const noexcept {
        return frame_;
    }

private:
    std::uint32_t width_;
    std::uint32_t height_;
    std::uint32_t camera_id_;
    std::uint64_t frame_ = 0;
    std::vector<sdk::FrontendObservation<double>> prev_;
};

}  // namespace branes::tools::trace

#endif  // BRANES_TOOLS_VIO_TRACE_TAP_HPP
