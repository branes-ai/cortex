// SPDX-License-Identifier: MIT
//
// branes/tools/s4_frontend_inspect.hpp — the S4 (visual frontend) inspector
// (epic #371, issue #374). The TEMPLATE per-stage inspector.
//
// An inspector re-runs the REAL stage operator on real data and exposes the
// intermediate quantities the production path hides, for study. For S4 the
// operator is the shipped cv/ frontend — `detect_fast` + pyramidal KLT — and
// the hidden quantities are the per-track forward-backward residual, track
// status, and track age. `S4FrontendInspector::step` mirrors the production
// `VioEstimator::track_frame` (sdk/include/branes/sdk/vio_estimator.hpp) call
// for call, with two deliberate differences for visualization:
//
//   1. it ALWAYS runs the backward KLT pass to compute a forward-backward
//      residual per track (production only does this when the gate is enabled);
//   2. it emits an enriched per-frame report (S4FrameReport) — every track's
//      previous/current pixel, FB residual, age and status, the FAST detections
//      added this frame, the pyramid geometry, and a coverage grid — which the
//      overlay renderer (docs-site/scripts/gen-overlay.mjs) draws.
//
// The instrumented tracking lives in this header (not the .cpp driver) so it is
// unit-testable on synthetic frames without the ~1.5 GB EuRoC dataset.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_S4_FRONTEND_INSPECT_HPP
#define BRANES_TOOLS_S4_FRONTEND_INSPECT_HPP

#include <branes/cv/fast.hpp>
#include <branes/cv/image.hpp>
#include <branes/cv/klt.hpp>
#include <branes/cv/pyramid.hpp>
#include <branes/sdk/vio_estimator.hpp>  // branes::sdk::FrontendParams

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
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

/// Runs the shipped frontend operator frame by frame, recording what it did.
class S4FrontendInspector {
public:
    explicit S4FrontendInspector(branes::sdk::FrontendParams fe = {}) : fe_(fe) {}

    /// Track `image` against the previous frame and report. Mirrors
    /// VioEstimator::track_frame, instrumented.
    [[nodiscard]] S4FrameReport step(cv::Image<const std::uint8_t> image, double t_s, std::string path) {
        cv::Pyramid<std::uint8_t> next(image, std::max(1, fe_.pyramid_levels));

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

        if (have_prev_ && !tracks_.empty()) {
            std::vector<cv::KeyPoint> pts;
            pts.reserve(tracks_.size());
            for (const auto& t : tracks_)
                pts.push_back(cv::KeyPoint{t.x, t.y, 0.0f});
            const auto res = cv::track_klt_pyramidal(prev_pyramid_, next, pts, fe_.klt);

            // Backward pass — ALWAYS, for the FB residual we display (production
            // only does this when fe_.fb_max_residual > 0).
            std::vector<cv::KeyPoint> back_pts;
            back_pts.reserve(res.size());
            for (const auto& rr : res)
                back_pts.push_back(cv::KeyPoint{rr.x, rr.y, 0.0f});
            const auto back = cv::track_klt_pyramidal(next, prev_pyramid_, back_pts, fe_.klt);
            const double fb2 = fe_.fb_max_residual * fe_.fb_max_residual;

            std::vector<Track> kept;
            kept.reserve(tracks_.size());
            for (std::size_t i = 0; i < res.size(); ++i) {
                if (res[i].status != cv::TrackStatus::Tracked) {
                    ++rep.n_lost;
                    continue;
                }
                double fb = -1.0;
                if (back[i].status == cv::TrackStatus::Tracked) {
                    const double dx = static_cast<double>(back[i].x) - pts[i].x;
                    const double dy = static_cast<double>(back[i].y) - pts[i].y;
                    fb = std::sqrt(dx * dx + dy * dy);
                }
                if (fe_.fb_max_residual > 0.0) {
                    if (back[i].status != cv::TrackStatus::Tracked || fb * fb > fb2) {
                        ++rep.n_fb_culled;
                        continue;
                    }
                }
                Track t = tracks_[i];
                t.x = res[i].x;
                t.y = res[i].y;
                ++t.age;
                rep.tracks.push_back(S4Track{t.id, res[i].x, res[i].y, pts[i].x, pts[i].y, fb, t.age, "tracked"});
                kept.push_back(t);
                ++rep.n_tracked;
            }
            tracks_ = std::move(kept);
        }

        if (tracks_.size() < fe_.target_features) {
            for (const auto& t : detect_new(next.level(0))) {
                rep.detections.emplace_back(t.x, t.y);
                rep.tracks.push_back(S4Track{t.id, t.x, t.y, t.x, t.y, -1.0, 0, "new"});
                ++rep.n_new;
            }
        }

        rep.grid_occupied = coverage(rep.grid_cols, rep.grid_rows, image.width(), image.height());

        prev_pyramid_ = std::move(next);
        have_prev_ = true;
        ++frame_;
        return rep;
    }

    [[nodiscard]] std::uint64_t frames_emitted() const noexcept {
        return frame_;
    }

private:
    struct Track {
        std::uint64_t id = 0;
        float x = 0.0f, y = 0.0f;
        std::uint32_t age = 0;
    };

    /// FAST detect + NMS-by-distance against existing tracks, exactly as
    /// VioEstimator::detect_new; returns the tracks it appended.
    [[nodiscard]] std::vector<Track> detect_new(cv::Image<const std::uint8_t> level0) {
        auto kps = cv::detect_fast(level0, fe_.fast_threshold);
        std::sort(kps.begin(), kps.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
            return a.response > b.response;
        });
        const double min_d2 = fe_.min_feature_distance * fe_.min_feature_distance;
        std::vector<Track> added;
        for (const auto& kp : kps) {
            if (tracks_.size() >= fe_.target_features)
                break;
            bool too_close = false;
            for (const auto& t : tracks_) {
                const double dx = static_cast<double>(kp.x) - t.x;
                const double dy = static_cast<double>(kp.y) - t.y;
                if (dx * dx + dy * dy < min_d2) {
                    too_close = true;
                    break;
                }
            }
            if (!too_close) {
                Track t{next_id_++, kp.x, kp.y, 0};
                tracks_.push_back(t);
                added.push_back(t);
            }
        }
        return added;
    }

    /// Count grid cells holding ≥1 current track — the spatial-coverage metric.
    [[nodiscard]] std::uint32_t coverage(int cols, int rows, std::size_t w, std::size_t h) const {
        std::vector<char> occ(static_cast<std::size_t>(cols) * rows, 0);
        for (const auto& t : tracks_) {
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
    std::vector<Track> tracks_;
    cv::Pyramid<std::uint8_t> prev_pyramid_;
    bool have_prev_ = false;
    std::uint64_t next_id_ = 0;
    std::uint64_t frame_ = 0;
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_S4_FRONTEND_INSPECT_HPP
