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
    /// covariance with the exact correlation:
    ///   P' = [[P, P Jᵀ], [J P, J P Jᵀ]],  J = ∂(clone error)/∂(state error)
    /// (identity onto the IMU δθ, δp blocks).
    static void augment_clone(State<T>& s) {
        const std::size_t d = s.dim();
        DynMat<T> J(State<T>::kCloneDim, d);
        for (std::size_t i = 0; i < 3; ++i) {
            J(i, State<T>::kTheta + i) = T{1};
            J(3 + i, State<T>::kPos + i) = T{1};
        }
        const DynMat<T> Jt = transpose(J);
        const DynMat<T> PJt = mul(s.P, Jt);  // d×6
        const DynMat<T> JP = mul(J, s.P);    // 6×d
        const DynMat<T> JPJt = mul(JP, Jt);  // 6×6

        DynMat<T> Pn(d + State<T>::kCloneDim, d + State<T>::kCloneDim);
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < d; ++j)
                Pn(i, j) = s.P(i, j);
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < State<T>::kCloneDim; ++j) {
                Pn(i, d + j) = PJt(i, j);
                Pn(d + j, i) = JP(j, i);
            }
        for (std::size_t i = 0; i < State<T>::kCloneDim; ++i)
            for (std::size_t j = 0; j < State<T>::kCloneDim; ++j)
                Pn(d + i, d + j) = JPJt(i, j);
        symmetrize(Pn);
        s.P = std::move(Pn);
        s.clones.push_back({s.R, s.p, s.timestamp});
    }

    /// Drop clone `idx`: delete its 6 error-state rows/cols (a principal
    /// submatrix of a PD matrix stays PD) and remove it from the window.
    static void marginalize_clone(State<T>& s, std::size_t idx) {
        assert(idx < s.clones.size() && "marginalize_clone: index out of range");
        const std::size_t d = s.dim();
        const std::size_t off = s.clone_offset(idx);
        std::vector<std::size_t> keep;
        keep.reserve(d - State<T>::kCloneDim);
        for (std::size_t i = 0; i < d; ++i)
            if (i < off || i >= off + State<T>::kCloneDim)
                keep.push_back(i);

        DynMat<T> Pn(keep.size(), keep.size());
        for (std::size_t a = 0; a < keep.size(); ++a)
            for (std::size_t b = 0; b < keep.size(); ++b)
                Pn(a, b) = s.P(keep[a], keep[b]);
        s.P = std::move(Pn);
        s.clones.erase(s.clones.begin() + static_cast<std::ptrdiff_t>(idx));
    }

    /// EKF update with measurement Jacobian `H` (k×dim), residual `r` (k),
    /// and diagonal measurement noise `R_diag` (k). Uses the Joseph form
    /// so the covariance stays symmetric positive-definite, then applies
    /// the error-state correction via the manifold box-plus.
    static void ekf_update(State<T>& s, const DynMat<T>& H, std::span<const T> r, std::span<const T> R_diag) {
        const std::size_t d = s.dim();
        const std::size_t k = H.rows;
        assert(H.cols == d && "ekf_update: H must be k×dim");
        assert(r.size() == k && R_diag.size() == k && "ekf_update: r / R_diag length must be H.rows");
#ifndef NDEBUG
        for (std::size_t i = 0; i < k; ++i)
            assert(R_diag[i] > T{0} && "ekf_update: measurement noise must be positive");
#endif
        const DynMat<T> Ht = transpose(H);
        const DynMat<T> PHt = mul(s.P, Ht);  // d×k
        DynMat<T> S = mul(H, PHt);           // k×k
        for (std::size_t i = 0; i < k; ++i)
            S(i, i) += R_diag[i];

        const DynMat<T> Kt = spd_solve(S, transpose(PHt));  // k×d
        const DynMat<T> K = transpose(Kt);                  // d×k

        // Error-state correction δx = K r.
        std::vector<T> dx(d, T{0});
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < k; ++j)
                dx[i] += K(i, j) * r[j];

        // Joseph form: P = (I−KH) P (I−KH)ᵀ + K diag(R) Kᵀ.
        const DynMat<T> KH = mul(K, H);  // d×d
        DynMat<T> ImKH = DynMat<T>::identity(d);
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < d; ++j)
                ImKH(i, j) -= KH(i, j);
        DynMat<T> P1 = mul(mul(ImKH, s.P), transpose(ImKH));
        DynMat<T> KR = K;  // K with columns scaled by R_diag
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < k; ++j)
                KR(i, j) *= R_diag[j];
        DynMat<T> KRKt = mul(KR, Kt);
        DynMat<T> P = add(P1, KRKt);
        symmetrize(P);
        s.P = std::move(P);

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
