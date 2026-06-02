// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/camera_updater.hpp — the MSCKF camera measurement
// update. A feature tracked across a window of cloned poses is
// triangulated, its stacked reprojection residual and Jacobians are
// formed, the feature is marginalized by left-null-space projection, the
// innovation is Mahalanobis-gated, and the surviving constraint is fed to
// the EKF update. Clean-room from the MSCKF measurement model (Mourikis &
// Roumeliotis, 2007).
//
// Works in *normalized* image coordinates (x/z, y/z): camera intrinsics
// and distortion are removed by the front end, so the updater is
// intrinsics-agnostic. Mono and stereo are handled uniformly — an
// observation names the clone it was taken in and the camera (extrinsic)
// it came through, so a stereo feature simply contributes observations
// through two extrinsics that share the same clone poses.
//
// Extension hook: any sensor updater (GPS, wheel, lidar, …) follows the
// same recipe — build (H, r, R), optionally gate, then call
// StateHelper::ekf_update. This class is the camera instance of it.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_CAMERA_UPDATER_HPP
#define BRANES_SDK_MSCKF_CAMERA_UPDATER_HPP

#include <branes/sdk/features/msckf_nullspace.hpp>
#include <branes/sdk/msckf/state_helper.hpp>

#include <cstddef>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace branes::sdk::msckf {

/// Camera→IMU extrinsics: the camera pose in the IMU frame. `R_imu_cam`
/// rotates camera axes into the IMU frame; `p_imu_cam` is the camera
/// origin in the IMU frame. Identity = camera coincident with the IMU.
template <math::Scalar T>
struct CameraExtrinsics {
    using SO3 = math::lie::SO3<T>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    SO3 R_imu_cam{};
    Vec3 p_imu_cam{};
};

/// One normalized observation of a feature: which clone it was seen in,
/// which camera (index into the updater's extrinsics), and the normalized
/// image coordinates (x/z, y/z).
template <math::Scalar T>
struct CameraObservation {
    std::size_t clone_index = 0;
    std::size_t camera_index = 0;
    math::lie::detail::Vec<T, 2> xy{};
};

/// A feature track: the same landmark observed in several clones (and/or
/// cameras). MSCKF needs ≥ 2 observations so that 2·m > 3 and the feature
/// can be marginalized.
template <math::Scalar T>
struct FeatureTrack {
    std::vector<CameraObservation<T>> observations;
};

/// Per-update innovation telemetry: the Normalized Innovation Squared
/// `value = νᵀ S⁻¹ ν` and its degrees of freedom `dof` (the projected residual
/// dimension). `valid` is false when the innovation covariance was
/// ill-conditioned (no usable NIS). Kept here in `msckf` (a plain struct, no
/// `eval` dependency) so the backend can bridge it into a consistency
/// accumulator without a layering cycle. See issue #264.
template <math::Scalar T>
struct NisSample {
    T value = T{0};
    std::size_t dof = 0;
    bool valid = false;
};

template <math::Scalar T>
struct CameraUpdaterOptions {
    T normalized_sigma = T{1} / T{100};  ///< measurement σ in normalized image coords
    /// Reject the track if the gated Mahalanobis distance exceeds
    /// `chi2_per_dof · (#projected rows)`. A loose default; the daemon can
    /// substitute a proper χ² quantile per residual dimension.
    T chi2_per_dof = T{5};
    bool enable_gating = true;
    std::size_t max_triangulation_iters = 5;  ///< Gauss-Newton refinement steps
};

template <math::Scalar T>
class CameraUpdater {
public:
    using SO3 = math::lie::SO3<T>;
    using Vec2 = math::lie::detail::Vec<T, 2>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Mat3 = math::lie::detail::Mat<T, 3, 3>;
    using Options = CameraUpdaterOptions<T>;
    using Extrinsics = CameraExtrinsics<T>;

    CameraUpdater(std::vector<Extrinsics> cameras, const Options& opts = {})
        : cameras_(std::move(cameras)), opts_(opts) {
        // Reject misconfiguration at the SDK boundary: a non-positive σ
        // would feed zero/negative noise into ekf_update (which requires
        // strictly positive noise), and a non-positive gate would reject
        // or admit everything.
        if (!(opts_.normalized_sigma > T{0}))
            throw std::invalid_argument("CameraUpdater: normalized_sigma must be positive");
        if (opts_.enable_gating && !(opts_.chi2_per_dof > T{0}))
            throw std::invalid_argument("CameraUpdater: chi2_per_dof must be positive when gating");
    }

    /// Triangulate, marginalize, gate, and (if accepted) apply the EKF
    /// update for one feature track. Returns true iff the state was
    /// updated. Tracks that are too short, triangulate behind a camera, or
    /// fail the Mahalanobis gate are skipped (the filter is unchanged).
    template <class Cov>
    bool update(State<T, Cov>& s, const FeatureTrack<T>& track, NisSample<T>* nis_out = nullptr) const {
        const auto& obs = track.observations;
        const std::size_t m = obs.size();
        if (m < 2)
            return false;
        for (const auto& o : obs)
            if (o.clone_index >= s.clones.size() || o.camera_index >= cameras_.size())
                return false;

        Vec3 p_f;
        if (!triangulate(s, obs, p_f))
            return false;

        // Stacked feature Jacobian H_f (2m×3), state Jacobian H_x (2m×N),
        // and residual r (2m). H_x is zero except in each observation's
        // clone columns. Rows are in normalized image coordinates.
        const std::size_t n = s.dim();
        const std::size_t rows2 = 2 * m;
        std::vector<T> Hf(rows2 * 3, T{0});
        std::vector<T> Hx(rows2 * n, T{0});
        std::vector<T> r(rows2, T{0});

        for (std::size_t i = 0; i < m; ++i) {
            Jacobians J;
            if (!project(s, obs[i], p_f, J))
                return false;  // feature behind a camera ⇒ drop the track
            const std::size_t row = 2 * i;
            const std::size_t off = s.clone_offset(obs[i].clone_index);
            for (std::size_t a = 0; a < 2; ++a) {
                r[row + a] = obs[i].xy[a] - J.h[a];
                for (std::size_t b = 0; b < 3; ++b) {
                    Hf[(row + a) * 3 + b] = J.Hf(a, b);
                    Hx[(row + a) * n + off + b] = J.Htheta(a, b);   // δθ block
                    Hx[(row + a) * n + off + 3 + b] = -J.Hf(a, b);  // δp block = −Hf
                }
            }
        }

        // Marginalize the feature: left-null-space projection. The
        // reflectors are orthonormal, so the projected measurement noise
        // stays σ²·I on the surviving rows.
        const auto proj = features::msckf_left_nullspace_project<T>(Hf, Hx, r, rows2, n);
        if (proj.rows == 0)
            return false;

        DynMat<T> H(proj.rows, n);
        for (std::size_t i = 0; i < proj.rows; ++i)
            for (std::size_t j = 0; j < n; ++j)
                H(i, j) = proj.H_x[i * n + j];
        const T r2 = opts_.normalized_sigma * opts_.normalized_sigma;
        std::vector<T> R_diag(proj.rows, r2);

        // Innovation NIS (γ = rᵀ S⁻¹ r, dof = proj.rows), computed once (only
        // when needed) and used for both the optional consistency telemetry and
        // the Mahalanobis gate.
        T gamma = T{0};
        bool nis_valid = false;
        if (nis_out != nullptr || opts_.enable_gating) {
            nis_valid = s.cov.mahalanobis(H, std::span<const T>{proj.r}, r2, gamma);
            if (nis_out != nullptr)
                *nis_out = NisSample<T>{gamma, proj.rows, nis_valid};
        }
        if (opts_.enable_gating && (!nis_valid || gamma > opts_.chi2_per_dof * static_cast<T>(proj.rows)))
            return false;  // ill-conditioned innovation or gated out

        StateHelper<T>::ekf_update(s, H, std::span<const T>{proj.r}, std::span<const T>{R_diag});
        return true;
    }

    /// Update with a batch of tracks, returning how many were applied.
    template <class Cov>
    std::size_t update_all(State<T, Cov>& s, std::span<const FeatureTrack<T>> tracks) const {
        std::size_t applied = 0;
        for (const auto& t : tracks)
            applied += update(s, t) ? 1 : 0;
        return applied;
    }

private:
    // Per-observation projection h(p_f) and its Jacobians w.r.t. the
    // feature position (Hf) and the clone orientation error (Htheta).
    struct Jacobians {
        Vec2 h{};
        math::lie::detail::Mat<T, 2, 3> Hf{};
        math::lie::detail::Mat<T, 2, 3> Htheta{};
    };

    // Camera-frame point of `p_f` for observation `o`. `y` is the feature
    // in the clone's IMU frame (needed for the orientation Jacobian).
    template <class Cov>
    bool to_camera(const State<T, Cov>& s, const CameraObservation<T>& o, const Vec3& p_f, Vec3& p_c, Vec3& y) const {
        const auto& cl = s.clones[o.clone_index];
        const auto& ex = cameras_[o.camera_index];
        const Mat3 Rit = cl.R.inverse().matrix();
        const Mat3 Rct = ex.R_imu_cam.inverse().matrix();
        y = Rit * (p_f - cl.p);          // feature in IMU frame
        p_c = Rct * (y - ex.p_imu_cam);  // feature in camera frame
        return p_c[2] > T{0};            // must be in front of the camera
    }

    template <class Cov>
    bool project(const State<T, Cov>& s, const CameraObservation<T>& o, const Vec3& p_f, Jacobians& J) const {
        Vec3 p_c, y;
        if (!to_camera(s, o, p_f, p_c, y))
            return false;
        const T z = p_c[2];
        const T inv = T{1} / z;
        J.h = Vec2{{p_c[0] * inv, p_c[1] * inv}};

        // ∂h/∂p_c = (1/z)[[1,0,−x/z],[0,1,−y/z]].
        math::lie::detail::Mat<T, 2, 3> dh{};
        dh(0, 0) = inv;
        dh(0, 2) = -p_c[0] * inv * inv;
        dh(1, 1) = inv;
        dh(1, 2) = -p_c[1] * inv * inv;

        const auto& cl = s.clones[o.clone_index];
        const auto& ex = cameras_[o.camera_index];
        const Mat3 Rct = ex.R_imu_cam.inverse().matrix();
        const Mat3 Rit = cl.R.inverse().matrix();
        const Mat3 M = Rct * Rit;  // ∂p_c/∂p_f
        // ∂p_c/∂δθ = R_camᵀ · [y]×  (right perturbation R ← R·Exp(δθ)).
        const Mat3 dpc_dtheta = Rct * math::lie::detail::hat(y);
        J.Hf = dh * M;
        J.Htheta = dh * dpc_dtheta;
        return true;
    }

    // Linear (ray-perpendicular) triangulation, then a few Gauss-Newton
    // reprojection refinements. Solves Σ(I − d̂d̂ᵀ)·p_f = Σ(I − d̂d̂ᵀ)·C.
    template <class Cov>
    bool triangulate(const State<T, Cov>& s, const std::vector<CameraObservation<T>>& obs, Vec3& p_f) const {
        DynMat<T> A(3, 3);
        DynMat<T> b(3, 1);
        for (const auto& o : obs) {
            const auto& cl = s.clones[o.clone_index];
            const auto& ex = cameras_[o.camera_index];
            const Mat3 Rwc = (cl.R * ex.R_imu_cam).matrix();  // world ← camera
            const Vec3 center = cl.p + cl.R.matrix() * ex.p_imu_cam;
            Vec3 d = Rwc * Vec3{{o.xy[0], o.xy[1], T{1}}};
            const T dn = math::lie::detail::norm(d);
            if (!(dn > T{0}))
                return false;
            d = d * (T{1} / dn);
            // I − d̂ d̂ᵀ
            for (std::size_t a = 0; a < 3; ++a) {
                for (std::size_t bb = 0; bb < 3; ++bb) {
                    const T proj = (a == bb ? T{1} : T{0}) - d[a] * d[bb];
                    A(a, bb) += proj;
                    b(a, 0) += proj * center[bb];
                }
            }
        }
        DynMat<T> L;
        if (!cholesky(A, L))
            return false;  // degenerate geometry (parallel rays)
        const DynMat<T> x = cholesky_solve(L, b);
        p_f = Vec3{{x(0, 0), x(1, 0), x(2, 0)}};

        // Gauss-Newton refinement on the reprojection error.
        for (std::size_t it = 0; it < opts_.max_triangulation_iters; ++it) {
            Mat3 H{};
            Vec3 g{};
            for (const auto& o : obs) {
                Jacobians J;
                if (!project(s, o, p_f, J))
                    return false;
                const Vec2 res{{o.xy[0] - J.h[0], o.xy[1] - J.h[1]}};
                // Normal equations on Hf (2×3): H += HfᵀHf, g += Hfᵀ res.
                for (std::size_t a = 0; a < 3; ++a) {
                    for (std::size_t bb = 0; bb < 3; ++bb)
                        for (std::size_t k = 0; k < 2; ++k)
                            H(a, bb) += J.Hf(k, a) * J.Hf(k, bb);
                    for (std::size_t k = 0; k < 2; ++k)
                        g[a] += J.Hf(k, a) * res[k];
                }
            }
            DynMat<T> Hn(3, 3), gn(3, 1), Ln;
            for (std::size_t a = 0; a < 3; ++a) {
                for (std::size_t bb = 0; bb < 3; ++bb)
                    Hn(a, bb) = H(a, bb);
                Hn(a, a) += T(1e-9);
                gn(a, 0) = g[a];
            }
            if (!cholesky(Hn, Ln))
                break;
            const DynMat<T> dx = cholesky_solve(Ln, gn);
            p_f = p_f + Vec3{{dx(0, 0), dx(1, 0), dx(2, 0)}};
        }
        return true;
    }

    std::vector<Extrinsics> cameras_;
    Options opts_;
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_CAMERA_UPDATER_HPP
