// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/clone_window_probe.hpp — the S3 (state augmentation / stochastic
// cloning) and S9 (marginalization / clone management) probes of the VIO contract
// program (docs/arch/vio-pipeline-canonical.md).
//
// Both stages are pure covariance bookkeeping on the sliding window, and both have
// a textbook clean-room failure mode the probe checks against the SHIPPED code
// (StateHelper::augment_clone / marginalize_clone):
//
//   S3 — a clone is a DETERMINISTIC COPY of the current pose, so at augmentation
//        its marginal must equal the cloned pose's marginal AND its cross-covariance
//        with every other state must equal the pose's. "Just add a new block" (zero
//        / block-diagonal cross-covariance) is the classic inconsistency.
//   S9 — dropping a clone is principal-submatrix extraction, so the kept-state
//        marginal must be UNCHANGED. (Marginalizing a variable never alters the
//        marginal of the others.)
//
// The probe builds a genuinely-correlated covariance (augment + propagate a few
// clones), then exercises the shipped operations and measures the invariant
// residuals. Native units: dimensionless (covariance residuals), eigenvalue
// (PSD). Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_CLONE_WINDOW_PROBE_HPP
#define BRANES_SDK_EVAL_CLONE_WINDOW_PROBE_HPP

#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/state_helper.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace branes::sdk::eval {

namespace clone_detail {
// Largest |a−b| over the index lists (rows ra↔rb, cols ca↔cb) of two matrices.
template <class T, class M>
T max_block_diff(const M& A,
                 const std::vector<std::size_t>& ra,
                 const std::vector<std::size_t>& ca,
                 const M& B,
                 const std::vector<std::size_t>& rb,
                 const std::vector<std::size_t>& cb) {
    T m = 0;
    for (std::size_t i = 0; i < ra.size(); ++i)
        for (std::size_t j = 0; j < ca.size(); ++j)
            m = std::max(m, std::abs(A(ra[i], ca[j]) - B(rb[i], cb[j])));
    return m;
}

// A State with `nclones` clones and a genuinely correlated covariance: set a
// distinct pose, augment, then propagate a few IMU steps to build cross-terms.
template <math::Scalar T>
branes::sdk::msckf::State<T> correlated_state(std::size_t nclones) {
    using namespace branes::sdk::msckf;
    using Vec3 = typename State<T>::Vec3;
    State<T> s(T{3} / T{10});
    Propagator<T> prop;
    for (std::size_t i = 0; i < nclones; ++i) {
        s.p = Vec3{{T(i) / T{2}, T(i) / T{10}, T(i) / T{5}}};  // distinct clone positions
        StateHelper<T>::augment_clone(s);
        for (int k = 0; k < 10; ++k)
            prop.propagate(s,
                           Vec3{{T{1} / T{100}, T{-1} / T{200}, T{1} / T{125}}},
                           Vec3{{T{1} / T{20}, T{1} / T{50}, T{981} / T{100}}},
                           T{1} / T{100});
    }
    return s;
}
}  // namespace clone_detail

// ── S3 — state augmentation / stochastic cloning ────────────────────────────
template <math::Scalar T>
struct AugmentResult {
    T clone_marginal_err = 0;  ///< ‖P'[clone,clone] − P[pose,pose]‖_max (must be ≈ 0)
    T clone_cross_err = 0;     ///< ‖P'[clone,*] − P[pose,*]‖_max over existing states (≈ 0)
    T roundtrip_err = 0;       ///< ‖marginalize(augment(P)) − P‖_max (≈ 0)
    bool psd = false;          ///< P' positive semidefinite
    std::size_t dim_before = 0, dim_after = 0;
};

template <math::Scalar T>
[[nodiscard]] AugmentResult<T> augmentation_probe() {
    using namespace branes::sdk::msckf;
    using clone_detail::max_block_diff;
    auto s = clone_detail::correlated_state<T>(2);  // two clones already → full-rank, correlated P

    const auto P = s.covariance();
    const std::size_t d = s.dim();
    // The cloned pose is the IMU (θ, p) block: indices kTheta..+3, kPos..+3.
    const std::vector<std::size_t> pose{State<T>::kTheta + 0,
                                        State<T>::kTheta + 1,
                                        State<T>::kTheta + 2,
                                        State<T>::kPos + 0,
                                        State<T>::kPos + 1,
                                        State<T>::kPos + 2};
    std::vector<std::size_t> all(d);
    for (std::size_t i = 0; i < d; ++i)
        all[i] = i;

    StateHelper<T>::augment_clone(s);
    const auto Pp = s.covariance();
    const std::vector<std::size_t> clone{d + 0, d + 1, d + 2, d + 3, d + 4, d + 5};

    AugmentResult<T> r;
    r.dim_before = d;
    r.dim_after = s.dim();
    r.clone_marginal_err = max_block_diff<T>(Pp, clone, clone, P, pose, pose);
    r.clone_cross_err = max_block_diff<T>(Pp, clone, all, P, pose, all);
    r.psd = is_positive_semidefinite(Pp);

    // Round-trip: marginalizing the just-added (last) clone must restore P exactly.
    StateHelper<T>::marginalize_clone(s, s.clones.size() - 1);
    const auto Prt = s.covariance();
    r.roundtrip_err = max_block_diff<T>(Prt, all, all, P, all, all);
    return r;
}

// ── S9 — marginalization / clone management ─────────────────────────────────
template <math::Scalar T>
struct MarginResult {
    T kept_marginal_err = 0;  ///< ‖P'[keep] − P[keep,keep]‖_max (must be ≈ 0 for extraction)
    bool psd = false;
    std::size_t dim_before = 0, dim_after = 0;
    std::size_t clones_before = 0, clones_after = 0;
};

template <math::Scalar T>
[[nodiscard]] MarginResult<T> marginalization_probe() {
    using namespace branes::sdk::msckf;
    using clone_detail::max_block_diff;
    auto s = clone_detail::correlated_state<T>(3);  // three clones in the window

    const auto P = s.covariance();
    const std::size_t d = s.dim();
    const std::size_t idx = 0;  // drop the oldest clone
    const std::size_t off = s.clone_offset(idx);
    std::vector<std::size_t> keep;
    keep.reserve(d - State<T>::kCloneDim);
    for (std::size_t i = 0; i < d; ++i)
        if (i < off || i >= off + State<T>::kCloneDim)
            keep.push_back(i);

    MarginResult<T> r;
    r.dim_before = d;
    r.clones_before = s.clones.size();
    StateHelper<T>::marginalize_clone(s, idx);
    const auto Pm = s.covariance();
    r.dim_after = s.dim();
    r.clones_after = s.clones.size();

    std::vector<std::size_t> kept_dst(keep.size());
    for (std::size_t i = 0; i < keep.size(); ++i)
        kept_dst[i] = i;
    r.kept_marginal_err = max_block_diff<T>(Pm, kept_dst, kept_dst, P, keep, keep);
    r.psd = is_positive_semidefinite(Pm);
    return r;
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_CLONE_WINDOW_PROBE_HPP
