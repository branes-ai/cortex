// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s4_frontend.hpp — stage S4_frontend as an explicit
// transformation on the backend side (issue #452,
// docs/arch/vio-pipeline-canonical.md §S4).
//
//   track(frontend state, image, params)               → (state', observations, diagnostics)
//   apply(tracks, t, frame observations, S0 normalize) → (tracks', diagnostics: ended tracks)
//   assemble(track records, state)                     → FeatureTrack for S5/S6
//
// `track` is the image-domain front end (#456): pyramidal KLT of the existing
// tracks into the new frame, the optional forward-backward gate, and FAST
// replenishment up to the target count — producing one pixel observation per
// surviving track with a stable id. VioEstimator runs it per frame.
//
// `apply` is the hand-off to the backend: each observation is normalized by
// S0_sensor_model and appended to its feature's track, keyed by the clone TIME it
// was taken at (clone indices shift on marginalization; times do not). A
// feature not observed in this frame has ended; its id is released, in feature-id
// order, to the update.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S4_FRONTEND_HPP
#define BRANES_SDK_MSCKF_STAGES_S4_FRONTEND_HPP

#include <branes/cv/fast.hpp>
#include <branes/cv/image.hpp>
#include <branes/cv/klt.hpp>
#include <branes/cv/pyramid.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace branes::sdk {

/// Front-end tuning. Defaults target a ~480p grayscale stream.
struct FrontendParams {
    int pyramid_levels = 3;
    double fast_threshold = 20.0;        ///< FAST-9 contrast threshold
    std::size_t target_features = 150;   ///< re-detect when tracked count drops below
    double min_feature_distance = 15.0;  ///< px; suppress detections near existing tracks
    /// S4 forward-backward gate: re-track each survivor back to the previous frame
    /// and drop it if the round-trip error exceeds this (px). Rejects tracks that
    /// drifted to a confidently-wrong location before they reach the backend.
    /// **Default 0 (disabled)**: enabling it (e.g. ~1 px) doubles the KLT cost and
    /// should be validated end-to-end before being turned on.
    double fb_max_residual = 0.0;
    cv::KltParams klt{};
};

}  // namespace branes::sdk

namespace branes::sdk::msckf::stages::s4_frontend {

using Pixel = std::uint8_t;

/// One live image-domain track: a stable id and its current pixel position.
struct FrontendTrack {
    std::uint64_t id = 0;
    float x = 0.0f;
    float y = 0.0f;
};

/// The front end's state between frames: the previous frame's pyramid and the
/// live tracks, with the next id to assign.
struct FrontendState {
    cv::Pyramid<Pixel> prev_pyramid{};
    bool have_prev = false;
    std::vector<FrontendTrack> tracks;
    std::uint64_t next_id = 0;
};

struct TrackDiagnostics {
    std::size_t tracks_in = 0;    ///< live tracks entering the frame
    std::size_t klt_lost = 0;     ///< lost or out of bounds in forward KLT
    std::size_t fb_rejected = 0;  ///< failed the forward-backward gate (when enabled)
    std::size_t detected = 0;     ///< new tracks from FAST replenishment
    std::size_t tracks_out = 0;   ///< live tracks after the frame (= observations)
};

template <math::Scalar T>
struct TrackResult {
    std::vector<FrontendObservation<T>> observations;  ///< one per surviving track, camera 0
    TrackDiagnostics diag;
};

namespace detail {
// Add the strongest FAST corners that are far enough from every existing
// track, up to the target count.
inline std::size_t detect_new(FrontendState& st, cv::Image<const Pixel> level0, const FrontendParams& fe) {
    auto kps = cv::detect_fast(level0, fe.fast_threshold);
    std::sort(
        kps.begin(), kps.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });
    const double min_d2 = fe.min_feature_distance * fe.min_feature_distance;
    std::size_t added = 0;
    for (const auto& kp : kps) {
        if (st.tracks.size() >= fe.target_features)
            break;
        bool too_close = false;
        for (const auto& t : st.tracks) {
            const double dx = static_cast<double>(kp.x) - t.x;
            const double dy = static_cast<double>(kp.y) - t.y;
            if (dx * dx + dy * dy < min_d2) {
                too_close = true;
                break;
            }
        }
        if (!too_close) {
            st.tracks.push_back(FrontendTrack{st.next_id++, kp.x, kp.y});
            ++added;
        }
    }
    return added;
}
}  // namespace detail

/// Track the live features into `image`, replenish with new FAST detections,
/// and return one observation per surviving track.
template <math::Scalar T>
TrackResult<T> track(FrontendState& st, cv::Image<const Pixel> image, const FrontendParams& fe) {
    TrackResult<T> out;
    out.diag.tracks_in = st.tracks.size();
    // At least one level, so level(0) is always valid even if a caller sets a
    // non-positive pyramid_levels.
    cv::Pyramid<Pixel> next(image, std::max(1, fe.pyramid_levels));

    if (st.have_prev && !st.tracks.empty()) {
        std::vector<cv::KeyPoint> pts;
        pts.reserve(st.tracks.size());
        for (const auto& t : st.tracks)
            pts.push_back(cv::KeyPoint{t.x, t.y, 0.0f});
        const auto res = cv::track_klt_pyramidal(st.prev_pyramid, next, pts, fe.klt);

        // Forward-backward consistency: re-track the forward survivors back to
        // the previous frame; a self-consistent track returns to its origin.
        std::vector<cv::KeyPoint> back_pts;
        std::vector<cv::KltResult> back;
        if (fe.fb_max_residual > 0.0) {
            back_pts.reserve(res.size());
            for (const auto& rr : res)
                back_pts.push_back(cv::KeyPoint{rr.x, rr.y, 0.0f});
            back = cv::track_klt_pyramidal(next, st.prev_pyramid, back_pts, fe.klt);
        }
        const double fb2 = fe.fb_max_residual * fe.fb_max_residual;

        std::vector<FrontendTrack> kept;
        kept.reserve(st.tracks.size());
        for (std::size_t i = 0; i < res.size(); ++i) {
            if (res[i].status != cv::TrackStatus::Tracked) {
                ++out.diag.klt_lost;
                continue;
            }
            if (fe.fb_max_residual > 0.0) {
                if (back[i].status != cv::TrackStatus::Tracked) {
                    ++out.diag.fb_rejected;
                    continue;  // can't verify the round trip → drop
                }
                const double dx = static_cast<double>(back[i].x) - pts[i].x;
                const double dy = static_cast<double>(back[i].y) - pts[i].y;
                if (dx * dx + dy * dy > fb2) {
                    ++out.diag.fb_rejected;
                    continue;  // failed forward-backward → drop the outlier
                }
            }
            FrontendTrack t = st.tracks[i];
            t.x = res[i].x;
            t.y = res[i].y;
            kept.push_back(t);
        }
        st.tracks = std::move(kept);
    }

    if (st.tracks.size() < fe.target_features)
        out.diag.detected = detail::detect_new(st, next.level(0), fe);

    out.observations.reserve(st.tracks.size());
    for (const auto& t : st.tracks) {
        FrontendObservation<T> o;
        o.feature_id = t.id;
        o.camera_id = 0;
        o.u = static_cast<T>(t.x);
        o.v = static_cast<T>(t.y);
        out.observations.push_back(o);
    }
    out.diag.tracks_out = st.tracks.size();

    st.prev_pyramid = std::move(next);
    st.have_prev = true;
    return out;
}

/// One stored observation of a feature, tagged by the clone time it was taken at.
template <math::Scalar T>
struct ObsRec {
    double clone_time = 0.0;
    std::uint32_t camera_id = 0;
    math::lie::detail::Vec<T, 2> xy{};  ///< normalized image point
};

/// The live feature tracks, by feature id.
template <math::Scalar T>
using TrackTable = std::unordered_map<std::uint64_t, std::vector<ObsRec<T>>>;

struct Diagnostics {
    std::size_t ingested = 0;          ///< observations appended to a track
    std::size_t rejected = 0;          ///< observations S0 rejected (behind the camera, unknown camera)
    std::vector<std::uint64_t> ended;  ///< features not seen this frame, ascending id
};

/// Ingest one frame's observations at clone time `t`. `normalize(camera_id, u,
/// v, xy) → bool` is the S0 sensor model for that camera. Returns the features
/// whose track ended, sorted by id: tracks is an unordered_map whose iteration
/// order is hash-driven and varies across STL implementations, and sorting keeps
/// the EKF update sequence — and the order-dependent innovation-whiteness lag-1
/// telemetry — reproducible.
template <math::Scalar T, class Normalize>
Diagnostics apply(TrackTable<T>& tracks, double t, std::span<const FrontendObservation<T>> obs, Normalize&& normalize) {
    Diagnostics d;
    std::unordered_set<std::uint64_t> seen;
    seen.reserve(obs.size());
    for (const auto& o : obs) {
        math::lie::detail::Vec<T, 2> xy;
        if (!normalize(o.camera_id, o.u, o.v, xy)) {
            ++d.rejected;
            continue;  // behind the camera / bad calibration index
        }
        tracks[o.feature_id].push_back(ObsRec<T>{t, o.camera_id, xy});
        seen.insert(o.feature_id);
        ++d.ingested;
    }
    for (const auto& [id, recs] : tracks)
        if (seen.find(id) == seen.end())
            d.ended.push_back(id);
    std::sort(d.ended.begin(), d.ended.end());
    return d;
}

/// Index of the clone taken at time `t`, or npos if it has been marginalized.
template <math::Scalar T, class Cov>
[[nodiscard]] std::size_t clone_index_at(const State<T, Cov>& s, double t) {
    for (std::size_t i = 0; i < s.clones.size(); ++i)
        if (s.clones[i].timestamp == t)
            return i;
    return static_cast<std::size_t>(-1);
}

/// The updater's view of one feature: its observations whose clone is still in
/// the window, with clone times resolved to current clone indices.
template <math::Scalar T, class Cov>
[[nodiscard]] FeatureTrack<T> assemble(const std::vector<ObsRec<T>>& recs, const State<T, Cov>& s) {
    FeatureTrack<T> track;
    track.observations.reserve(recs.size());
    for (const ObsRec<T>& rec : recs) {
        const std::size_t ci = clone_index_at(s, rec.clone_time);
        if (ci != static_cast<std::size_t>(-1))
            track.observations.push_back({ci, rec.camera_id, rec.xy});
    }
    return track;
}

}  // namespace branes::sdk::msckf::stages::s4_frontend

#endif  // BRANES_SDK_MSCKF_STAGES_S4_FRONTEND_HPP
