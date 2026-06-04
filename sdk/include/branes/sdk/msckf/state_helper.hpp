// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/state_helper.hpp — clone augmentation, marginalization,
// and the EKF measurement update for the MSCKF State. Clean-room from the
// standard MSCKF bookkeeping (Mourikis & Roumeliotis, 2007).
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STATE_HELPER_HPP
#define BRANES_SDK_MSCKF_STATE_HELPER_HPP

#include <branes/sdk/msckf/state.hpp>

#include <cassert>
#include <cstddef>
#include <span>

namespace branes::sdk::msckf {

template <math::Scalar T>
struct StateHelper {
    using SO3 = typename State<T>::SO3;
    using Vec3 = typename State<T>::Vec3;

    /// Append a clone of the current IMU pose to the window, expanding the
    /// covariance with the exact correlation P' = G P Gᵀ where
    ///   G = [[I], [J]],  J = ∂(clone error)/∂(state error)
    /// (identity onto the IMU δθ, δp blocks). The covariance policy applies
    /// the map; only the structural Jacobian is built here.
    template <class Cov>
    static void augment_clone(State<T, Cov>& s) {
        using St = State<T, Cov>;
        const std::size_t d = s.dim();
        DynMat<T> G(d + St::kCloneDim, d);
        for (std::size_t i = 0; i < d; ++i)
            G(i, i) = T{1};  // top block: identity (existing error states)
        for (std::size_t i = 0; i < 3; ++i) {
            G(d + i, St::kTheta + i) = T{1};    // clone δθ_c = δθ
            G(d + 3 + i, St::kPos + i) = T{1};  // clone δp_c = δp
        }
        s.cov.transform(G);
        // The clone's first-estimate pose (R_fej, p_fej) is its value at creation
        // — the propagated IMU pose at this image time — and is frozen here for
        // FEJ; ekf_update never touches it (#280).
        s.clones.push_back({s.R, s.p, s.timestamp, s.R, s.p});
    }

    /// Drop clone `idx`: delete its 6 error-state rows/cols (a principal
    /// submatrix of a PD matrix stays PD) and remove it from the window.
    template <class Cov>
    static void marginalize_clone(State<T, Cov>& s, std::size_t idx) {
        using St = State<T, Cov>;
        assert(idx < s.clones.size() && "marginalize_clone: index out of range");
        const std::size_t d = s.dim();
        const std::size_t off = s.clone_offset(idx);
        std::vector<std::size_t> keep;
        keep.reserve(d - St::kCloneDim);
        for (std::size_t i = 0; i < d; ++i)
            if (i < off || i >= off + St::kCloneDim)
                keep.push_back(i);
        s.cov.marginalize(keep);
        s.clones.erase(s.clones.begin() + static_cast<std::ptrdiff_t>(idx));
    }

    /// EKF update with measurement Jacobian `H` (k×dim), residual `r` (k),
    /// and diagonal measurement noise `R_diag` (k). The covariance policy
    /// performs the update and returns the error-state correction δx, which
    /// is then applied to the mean via the manifold box-plus.
    template <class Cov>
    static void ekf_update(State<T, Cov>& s, const DynMat<T>& H, std::span<const T> r, std::span<const T> R_diag) {
        const std::size_t d = s.dim();
        const std::size_t k = H.rows;
        assert(H.cols == d && "ekf_update: H must be k×dim");
        assert(r.size() == k && R_diag.size() == k && "ekf_update: r / R_diag length must be H.rows");
#ifndef NDEBUG
        for (std::size_t i = 0; i < k; ++i)
            assert(R_diag[i] > T{0} && "ekf_update: measurement noise must be positive");
#endif
        const std::vector<T> dx = s.cov.update(H, r, R_diag);
        assert(dx.size() == d && "ekf_update: covariance policy returned wrong δx length");

        // Box-plus the mean: nav state then each clone.
        s.R = s.R * SO3::exp(slice3(dx, State<T>::kTheta));
        s.p = s.p + slice3(dx, State<T>::kPos);
        s.v = s.v + slice3(dx, State<T>::kVel);
        s.bg = s.bg + slice3(dx, State<T>::kBg);
        s.ba = s.ba + slice3(dx, State<T>::kBa);
        for (std::size_t c = 0; c < s.clones.size(); ++c) {
            const std::size_t off = s.clone_offset(c);
            s.clones[c].R = s.clones[c].R * SO3::exp(slice3(dx, off));
            s.clones[c].p = s.clones[c].p + slice3(dx, off + 3);
        }
    }

private:
    static Vec3 slice3(const std::vector<T>& v, std::size_t off) {
        return Vec3{{v[off], v[off + 1], v[off + 2]}};
    }
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_STATE_HELPER_HPP
