// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s4_frontend.hpp — stage S4_frontend as an explicit
// transformation on the backend side (issue #452,
// docs/arch/vio-pipeline-canonical.md §S4).
//
//   apply(tracks, t, frame observations, S0 normalize) → (tracks', diagnostics: ended tracks)
//   assemble(track records, state)                     → FeatureTrack for S5/S6
//
// The image-domain tracking (KLT, outlier rejection) runs in the front end
// (cv/klt.hpp via VioEstimator) and hands the backend per-frame pixel
// observations. This stage is that hand-off: each observation is normalized by
// S0_sensor_model and appended to its feature's track, keyed by the clone TIME it
// was taken at (clone indices shift on marginalization; times do not). A
// feature not observed in this frame has ended; its id is released, in feature-id
// order, to the update.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S4_FRONTEND_HPP
#define BRANES_SDK_MSCKF_STAGES_S4_FRONTEND_HPP

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

namespace branes::sdk::msckf::stages::s4_frontend {

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
