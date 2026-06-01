// SPDX-License-Identifier: MIT
//
// branes/sdk/sfm/init_window.hpp — assemble the multi-frame SfM window that
// dynamic visual-inertial initialization needs (issue #230, epic #211). Given a
// handful of init-window camera frames (normalized feature observations keyed
// by track id) plus the IMU preintegration between them, recover an up-to-scale
// camera trajectory and turn it into `DynInitKeyframe`s for
// `ImuInitializer::try_dynamic`.
//
// The first two frames are bootstrapped with the two-view essential matrix
// (#228), fixing the points; every later frame is resectioned against those
// points by PnP (#235), so the whole window shares one scale gauge. Camera
// poses are composed with the camera→IMU extrinsics to body poses, and the
// preintegration deltas are carried through unchanged. The metric scale is left
// for try_dynamic (#229) to resolve from the IMU.
//
// Clean-room; header-only, C++20, type-generic.

#ifndef BRANES_SDK_SFM_INIT_WINDOW_HPP
#define BRANES_SDK_SFM_INIT_WINDOW_HPP

#include <branes/sdk/imu_init.hpp>              // DynInitKeyframe
#include <branes/sdk/msckf/camera_updater.hpp>  // CameraExtrinsics
#include <branes/sdk/sfm/pnp.hpp>
#include <branes/sdk/sfm/two_view.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace branes::sdk::sfm {

/// One init-window frame: normalized feature observations keyed by track id,
/// plus the IMU preintegration accumulated since the previous frame (zeroed for
/// the first frame). The same `DynInitKeyframe` delta fields, minus the vision
/// pose this builder fills in.
template <math::Scalar T>
struct InitFrame {
    using SO3 = math::lie::SO3<T>;
    double timestamp = 0.0;
    std::vector<std::uint64_t> ids;            ///< track id per observation
    std::vector<Vec2<T>> obs;                  ///< normalized (x/z, y/z) per observation
    SO3 dR{};                                  ///< preintegrated rotation since the previous frame
    Vec3<T> dv{};                              ///< preintegrated velocity
    Vec3<T> dp{};                              ///< preintegrated position
    math::lie::detail::Mat<T, 3, 3> dR_dbg{};  ///< ∂ΔR/∂(gyro bias)
    T dt = T{0};                               ///< preintegration interval
};

/// Nearest rotation as an SO3 from a 3×3 matrix (Shepperd's quaternion form).
/// `SO3` only constructs from a quaternion, and the SfM stages produce rotation
/// matrices, so this bridges them. Assumes `R` is (numerically) orthonormal.
template <math::Scalar T>
[[nodiscard]] math::lie::SO3<T> so3_from_matrix(const Mat3<T>& R) {
    const T tr = R(0, 0) + R(1, 1) + R(2, 2);
    ld::Vec<T, 4> q;  // (w, x, y, z)
    if (tr > T{0}) {
        const T s = ld::sqrt_(tr + T{1}) * T{2};  // s = 4w
        q = ld::Vec<T, 4>{{T{1} / T{4} * s, (R(2, 1) - R(1, 2)) / s, (R(0, 2) - R(2, 0)) / s, (R(1, 0) - R(0, 1)) / s}};
    } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
        const T s = ld::sqrt_(T{1} + R(0, 0) - R(1, 1) - R(2, 2)) * T{2};  // s = 4x
        q = ld::Vec<T, 4>{{(R(2, 1) - R(1, 2)) / s, T{1} / T{4} * s, (R(0, 1) + R(1, 0)) / s, (R(0, 2) + R(2, 0)) / s}};
    } else if (R(1, 1) > R(2, 2)) {
        const T s = ld::sqrt_(T{1} + R(1, 1) - R(0, 0) - R(2, 2)) * T{2};  // s = 4y
        q = ld::Vec<T, 4>{{(R(0, 2) - R(2, 0)) / s, (R(0, 1) + R(1, 0)) / s, T{1} / T{4} * s, (R(1, 2) + R(2, 1)) / s}};
    } else {
        const T s = ld::sqrt_(T{1} + R(2, 2) - R(0, 0) - R(1, 1)) * T{2};  // s = 4z
        q = ld::Vec<T, 4>{{(R(1, 0) - R(0, 1)) / s, (R(0, 2) + R(2, 0)) / s, (R(1, 2) + R(2, 1)) / s, T{1} / T{4} * s}};
    }
    const T nq = ld::norm(q);
    if (nq > T{0})
        q = q * (T{1} / nq);
    return math::lie::SO3<T>(q);
}

/// Result of the window assembly: the per-frame keyframes (one per input frame
/// successfully placed) and whether enough frames were recovered for a dynamic
/// alignment (`try_dynamic` needs ≥ 2, and observability wants more).
template <math::Scalar T>
struct InitWindowResult {
    bool success = false;
    std::vector<DynInitKeyframe<T>> keyframes;
};

namespace detail {

/// Match observations shared by two frames into ordered correspondence lists.
template <math::Scalar T>
void shared(const InitFrame<T>& a,
            const InitFrame<T>& b,
            std::vector<Vec2<T>>& xa,
            std::vector<Vec2<T>>& xb,
            std::vector<std::uint64_t>& ids) {
    for (std::size_t i = 0; i < a.ids.size(); ++i)
        for (std::size_t j = 0; j < b.ids.size(); ++j)
            if (a.ids[i] == b.ids[j]) {
                xa.push_back(a.obs[i]);
                xb.push_back(b.obs[j]);
                ids.push_back(a.ids[i]);
                break;
            }
}

}  // namespace detail

/// Build the dynamic-init keyframes from the window. Returns `success = false`
/// (and no keyframes) when the bootstrap or a resection fails or too few frames
/// are placed. `extr` is the camera→IMU pose (identity = camera at the IMU).
template <math::Scalar T>
[[nodiscard]] InitWindowResult<T> build_init_window(std::span<const InitFrame<T>> frames,
                                                    const msckf::CameraExtrinsics<T>& extr = {}) {
    using SO3 = math::lie::SO3<T>;
    InitWindowResult<T> out;
    const std::size_t n = frames.size();
    if (n < 3)
        return out;

    // 1) Two-view bootstrap on frames 0–1 fixes the points (cam-0 frame, up to
    //    scale). cam0 is the world origin; cam1's world pose is the inverse of
    //    the recovered cam0→cam1 transform.
    std::vector<Vec2<T>> x0, x1;
    std::vector<std::uint64_t> ids01;
    detail::shared<T>(frames[0], frames[1], x0, x1, ids01);
    const auto tv = estimate_relative_pose<T>(x0, x1);
    if (!tv.success)
        return out;

    // World landmarks (cam-0 frame), keyed by track id.
    std::vector<std::uint64_t> land_ids;
    std::vector<Vec3<T>> land_pts;
    for (std::size_t k = 0; k < tv.inliers.size(); ++k) {
        land_ids.push_back(ids01[tv.inliers[k]]);
        land_pts.push_back(tv.points_cam0[k]);
    }
    if (land_pts.size() < 6)
        return out;  // PnP needs ≥ 6 landmarks for the later frames

    // Camera→world poses (world = cam0). cam0 = identity.
    std::vector<SO3> R_w_cam(n);
    std::vector<Vec3<T>> p_w_cam(n);
    R_w_cam[0] = SO3{};
    p_w_cam[0] = Vec3<T>{};
    // cam1: X_c1 = R_1_0 X_c0 + t ⇒ world←cam1 = (R_1_0ᵀ, −R_1_0ᵀ t).
    const Mat3<T> R10t = math::lie::detail::transpose(tv.R_1_0);
    R_w_cam[1] = so3_from_matrix<T>(R10t);
    p_w_cam[1] = R10t * tv.t_1_0_unit * (-T{1});

    // 2) PnP each later frame against the world landmarks it observes.
    for (std::size_t f = 2; f < n; ++f) {
        std::vector<Vec3<T>> P;
        std::vector<Vec2<T>> obs;
        for (std::size_t i = 0; i < frames[f].ids.size(); ++i)
            for (std::size_t j = 0; j < land_ids.size(); ++j)
                if (frames[f].ids[i] == land_ids[j]) {
                    P.push_back(land_pts[j]);
                    obs.push_back(frames[f].obs[i]);
                    break;
                }
        const auto pose = estimate_pose<T>(P, obs);
        if (!pose.success)
            return out;
        // PnP: X_cam = R·X_world + t ⇒ world←cam = (Rᵀ, −Rᵀ t).
        const Mat3<T> Rt = math::lie::detail::transpose(pose.R);
        R_w_cam[f] = so3_from_matrix<T>(Rt);
        p_w_cam[f] = Rt * pose.t * (-T{1});
    }

    // 3) Compose camera→world with the camera→IMU extrinsics to body poses, and
    //    carry the preintegration deltas. T_w_imu = T_w_cam · T_cam_imu, with
    //    T_cam_imu = (R_imu_camᵀ, −R_imu_camᵀ p_imu_cam).
    const SO3 R_cam_imu = extr.R_imu_cam.inverse();
    const Vec3<T> p_cam_imu = R_cam_imu * extr.p_imu_cam * (-T{1});
    out.keyframes.resize(n);
    for (std::size_t f = 0; f < n; ++f) {
        DynInitKeyframe<T>& kf = out.keyframes[f];
        kf.R_world_imu = R_w_cam[f] * R_cam_imu;
        kf.p_world_imu = R_w_cam[f] * p_cam_imu + p_w_cam[f];
        kf.dR = frames[f].dR;
        kf.dv = frames[f].dv;
        kf.dp = frames[f].dp;
        kf.dR_dbg = frames[f].dR_dbg;
        kf.dt = frames[f].dt;
    }
    out.success = true;
    return out;
}

}  // namespace branes::sdk::sfm

#endif  // BRANES_SDK_SFM_INIT_WINDOW_HPP
