// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s9_marginalization.hpp — stage S9_marginalization as
// an explicit transformation (issue #452, docs/arch/vio-pipeline-canonical.md
// §S9).
//
//   apply(state, clone index)      → (state' without the clone, diagnostics)
//   tracks_touching(tracks, t)     → features observed at the clone about to go
//   purge(tracks, t)               → tracks' without observations at that clone
//
// Removing a clone deletes its 6 error-state rows/cols: P' is the principal
// submatrix of P, so the kept marginal is unchanged. The window policy (which
// clone, and updating with the features that touch it first so their
// information is not lost) is the backend's; the track bookkeeping it needs is
// here so the stage can be benched with it.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S9_MARGINALIZATION_HPP
#define BRANES_SDK_MSCKF_STAGES_S9_MARGINALIZATION_HPP

#include <branes/sdk/msckf/stages/s4_frontend.hpp>
#include <branes/sdk/msckf/state_helper.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <vector>

namespace branes::sdk::msckf::stages::s9_marginalization {

struct Diagnostics {
    std::size_t dim_before = 0;
    std::size_t dim_after = 0;  ///< dim_before − 6
    double removed_time = 0.0;  ///< timestamp of the clone removed
    std::size_t clones = 0;     ///< window size after removal
};

/// Remove clone `idx` from the window. Precondition: idx < s.clones.size().
template <math::Scalar T, class Cov>
Diagnostics apply(State<T, Cov>& s, std::size_t idx) {
    Diagnostics d;
    d.dim_before = s.dim();
    d.removed_time = s.clones[idx].timestamp;
    StateHelper<T>::marginalize_clone(s, idx);
    d.dim_after = s.dim();
    d.clones = s.clones.size();
    return d;
}

/// Features with an observation at clone time `t`, ascending id (deterministic
/// update order; see s4_frontend::apply).
template <math::Scalar T>
[[nodiscard]] std::vector<std::uint64_t> tracks_touching(const s4_frontend::TrackTable<T>& tracks, double t) {
    std::vector<std::uint64_t> touching;
    for (const auto& [id, recs] : tracks)
        for (const auto& rec : recs)
            if (rec.clone_time == t) {
                touching.push_back(id);
                break;
            }
    std::sort(touching.begin(), touching.end());
    return touching;
}

/// Drop every observation taken at clone time `t`; remove tracks left empty.
template <math::Scalar T>
void purge(s4_frontend::TrackTable<T>& tracks, double t) {
    for (auto it = tracks.begin(); it != tracks.end();) {
        auto& recs = it->second;
        for (auto r = recs.begin(); r != recs.end();)
            r = (r->clone_time == t) ? recs.erase(r) : r + 1;
        it = recs.empty() ? tracks.erase(it) : std::next(it);
    }
}

}  // namespace branes::sdk::msckf::stages::s9_marginalization

#endif  // BRANES_SDK_MSCKF_STAGES_S9_MARGINALIZATION_HPP
