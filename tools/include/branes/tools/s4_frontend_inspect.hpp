// SPDX-License-Identifier: MIT
//
// branes/tools/s4_frontend_inspect.hpp — the S4 (visual frontend) inspector
// (epic #371, issue #374). The TEMPLATE per-stage inspector.
//
// An inspector re-runs the REAL stage operator on real data and exposes the
// intermediate quantities the production path hides, for study. For S4 the
// operator is `msckf::stages::s4_frontend::track` — FAST + pyramidal KLT + the
// optional forward-backward gate — the exact function VioEstimator calls per
// frame (#456). `S4FrontendInspector::step` calls it, unmodified, and derives the
// study quantities around it:
//
//   1. a forward-backward residual for every surviving track, by re-tracking the
//      survivors back into the previous pyramid (production only does this when
//      the gate is enabled; the KLT is per-point, so the residual is the one the
//      gate would see);
//   2. an enriched per-frame report (S4FrameReport) — every track's
//      previous/current pixel, FB residual, age and status, the FAST detections
//      added this frame, the pyramid geometry, and a coverage grid — which the
//      overlay renderer (docs-site/scripts/gen-overlay.mjs) draws.
//
// Because the inspector observes the stage instead of copying it, what it shows
// is what the estimator does; there is no second implementation to drift.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_S4_FRONTEND_INSPECT_HPP
#define BRANES_TOOLS_S4_FRONTEND_INSPECT_HPP

#include <branes/cv/image.hpp>
#include <branes/cv/klt.hpp>
#include <branes/cv/pyramid.hpp>
#include <branes/sdk/msckf/stages/s4_frontend.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace branes::tools {

/// One track's state at a frame boundary, enriched for study.
struct S4Track {
    std::uint64_t id = 0;
    double u = 0.0, v = 0.0;    ///< position in THIS frame (px)
    double pu = 0.0, pv = 0.0;  ///< position in the PREVIOUS frame (== u,v if new)
    double fb_residual = -1.0;  ///< forward-backward round-trip error (px); -1 if not computed
    std::uint32_t age = 0;      ///< frames survived (0 = detected this frame)
    std::string status;         ///< "new" | "tracked"
};

/// Everything one frame of the frontend produced — the inspector's record.
struct S4FrameReport {
    std::uint64_t frame = 0;
    double t_s = 0.0;
    std::string image_path;
    std::uint32_t width = 0, height = 0;
    int pyramid_levels = 0;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> pyramid_sizes;  ///< per-level (w,h)
    std::vector<S4Track> tracks;                                         ///< surviving + newly detected
    std::vector<std::pair<float, float>> detections;                     ///< FAST corners added this frame
    std::uint32_t n_tracked = 0, n_new = 0, n_lost = 0, n_fb_culled = 0;
    int grid_cols = 8, grid_rows = 6;
    std::uint32_t grid_occupied = 0;  ///< grid cells holding ≥1 track (spatial coverage)
    double fb_max = 0.0;              ///< the FB gate threshold in force (0 = disabled)
};

[[nodiscard]] inline nlohmann::json to_json(const S4FrameReport& r) {
    using nlohmann::json;
    json tracks = json::array();
    for (const auto& t : r.tracks)
        tracks.push_back(json{{"id", t.id},
                              {"u", t.u},
                              {"v", t.v},
                              {"pu", t.pu},
                              {"pv", t.pv},
                              {"fb", t.fb_residual},
                              {"age", t.age},
                              {"status", t.status}});
    json dets = json::array();
    for (const auto& d : r.detections)
        dets.push_back(json::array({d.first, d.second}));
    json sizes = json::array();
    for (const auto& s : r.pyramid_sizes)
        sizes.push_back(json::array({s.first, s.second}));
    // Keys `frame`/`t`/`image`/`nfeat` match the existing overlay schema so the
    // shared HUD keeps working; the rest is the frontend-specific enrichment.
    return json{
        {"frame", r.frame},
        {"t", r.t_s},
        {"image", r.image_path},
        {"width", r.width},
        {"height", r.height},
        {"nfeat", r.tracks.size()},
        {"pyramid", json{{"levels", r.pyramid_levels}, {"sizes", sizes}}},
        {"tracks", tracks},
        {"detections", dets},
        {"counts", json{{"tracked", r.n_tracked}, {"new", r.n_new}, {"lost", r.n_lost}, {"fb_culled", r.n_fb_culled}}},
        {"grid", json{{"cols", r.grid_cols}, {"rows", r.grid_rows}, {"occupied", r.grid_occupied}}},
        {"fb_max", r.fb_max}};
}

/// Runs the S4_frontend stage frame by frame, recording what it did.
class S4FrontendInspector {
public:
    explicit S4FrontendInspector(branes::sdk::FrontendParams fe = {}) : fe_(fe) {}

    /// Track `image` against the previous frame with the stage and report.
    [[nodiscard]] S4FrameReport step(cv::Image<const std::uint8_t> image, double t_s, std::string path) {
        namespace s4 = branes::sdk::msckf::stages::s4_frontend;

        // What the stage consumes and then discards: the previous pyramid (for
        // the backward pass) and each live track's previous position.
        const cv::Pyramid<std::uint8_t> prev = st_.prev_pyramid;
        const bool had_prev = st_.have_prev;
        std::unordered_map<std::uint64_t, std::pair<float, float>> before;
        for (const auto& t : st_.tracks)
            before.emplace(t.id, std::pair{t.x, t.y});

        const auto res = s4::track<double>(st_, image, fe_);
        const auto& next = st_.prev_pyramid;  // the stage leaves this frame's pyramid here

        S4FrameReport rep;
        rep.frame = frame_;
        rep.t_s = t_s;
        rep.image_path = std::move(path);
        rep.width = static_cast<std::uint32_t>(image.width());
        rep.height = static_cast<std::uint32_t>(image.height());
        rep.pyramid_levels = next.num_levels();
        for (int l = 0; l < next.num_levels(); ++l)
            rep.pyramid_sizes.emplace_back(static_cast<std::uint32_t>(next.level_width(l)),
                                           static_cast<std::uint32_t>(next.level_height(l)));
        rep.fb_max = fe_.fb_max_residual;
        rep.n_lost = static_cast<std::uint32_t>(res.diag.klt_lost);
        rep.n_fb_culled = static_cast<std::uint32_t>(res.diag.fb_rejected);
        rep.n_new = static_cast<std::uint32_t>(res.diag.detected);
        rep.n_tracked = static_cast<std::uint32_t>(res.diag.tracks_out - res.diag.detected);

        // Backward pass over the survivors, for the FB residual we display.
        std::vector<cv::KeyPoint> fwd;
        std::vector<std::size_t> idx;
        if (had_prev)
            for (std::size_t i = 0; i < st_.tracks.size(); ++i)
                if (before.contains(st_.tracks[i].id)) {
                    fwd.push_back(cv::KeyPoint{st_.tracks[i].x, st_.tracks[i].y, 0.0f});
                    idx.push_back(i);
                }
        std::vector<double> fb(st_.tracks.size(), -1.0);
        if (!fwd.empty()) {
            const auto back = cv::track_klt_pyramidal(next, prev, fwd, fe_.klt);
            for (std::size_t k = 0; k < idx.size(); ++k)
                if (back[k].status == cv::TrackStatus::Tracked) {
                    const auto [px, py] = before.at(st_.tracks[idx[k]].id);
                    const double dx = static_cast<double>(back[k].x) - px;
                    const double dy = static_cast<double>(back[k].y) - py;
                    fb[idx[k]] = std::sqrt(dx * dx + dy * dy);
                }
        }

        std::unordered_map<std::uint64_t, std::uint32_t> ages;
        for (std::size_t i = 0; i < st_.tracks.size(); ++i) {
            const auto& t = st_.tracks[i];
            const auto it = before.find(t.id);
            if (it == before.end()) {
                rep.detections.emplace_back(t.x, t.y);
                rep.tracks.push_back(S4Track{t.id, t.x, t.y, t.x, t.y, -1.0, 0, "new"});
                ages.emplace(t.id, 0);
            } else {
                const std::uint32_t age = age_.at(t.id) + 1;
                rep.tracks.push_back(
                    S4Track{t.id, t.x, t.y, it->second.first, it->second.second, fb[i], age, "tracked"});
                ages.emplace(t.id, age);
            }
        }
        age_ = std::move(ages);

        rep.grid_occupied = coverage(rep.grid_cols, rep.grid_rows, image.width(), image.height());
        ++frame_;
        return rep;
    }

    [[nodiscard]] std::uint64_t frames_emitted() const noexcept {
        return frame_;
    }

    /// The stage state the inspector drives (read-only, for cross-checks).
    [[nodiscard]] const branes::sdk::msckf::stages::s4_frontend::FrontendState& state() const noexcept {
        return st_;
    }

private:
    /// Count grid cells holding ≥1 current track — the spatial-coverage metric.
    [[nodiscard]] std::uint32_t coverage(int cols, int rows, std::size_t w, std::size_t h) const {
        std::vector<char> occ(static_cast<std::size_t>(cols) * rows, 0);
        for (const auto& t : st_.tracks) {
            const int cx = std::clamp(static_cast<int>(t.x / static_cast<double>(w) * cols), 0, cols - 1);
            const int cy = std::clamp(static_cast<int>(t.y / static_cast<double>(h) * rows), 0, rows - 1);
            occ[static_cast<std::size_t>(cy) * cols + cx] = 1;
        }
        std::uint32_t n = 0;
        for (char c : occ)
            n += static_cast<std::uint32_t>(c);
        return n;
    }

    branes::sdk::FrontendParams fe_;
    branes::sdk::msckf::stages::s4_frontend::FrontendState st_;
    std::unordered_map<std::uint64_t, std::uint32_t> age_;  ///< frames survived, per live track id
    std::uint64_t frame_ = 0;
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_S4_FRONTEND_INSPECT_HPP
