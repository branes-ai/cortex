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
#include <branes/sdk/sfm/bundle_adjust.hpp>
#include <branes/sdk/sfm/pnp.hpp>
#include <branes/sdk/sfm/two_view.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
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
    math::lie::detail::Mat<T, 3, 3> dv_dba{};  ///< ∂Δv/∂(accel bias), or zero (DynInitKeyframe)
    math::lie::detail::Mat<T, 3, 3> dp_dba{};  ///< ∂Δp/∂(accel bias), or zero
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

/// Where a window build stopped (`Ok` when it placed every frame).
enum class InitWindowStatus : int {
    Ok = 0,
    TooFewFrames = 1,     ///< fewer than 3 frames
    MalformedFrame = 2,   ///< a frame's ids and obs differ in length
    NoBootstrapPair = 3,  ///< no frame shares ≥ 8 features with frame 0, or none has the minimum parallax
    TwoViewFailed = 4,    ///< the essential-matrix RANSAC rejected the bootstrap pair
    TooFewLandmarks = 5,  ///< the bootstrap triangulated < 6 landmarks
    PnpFailed = 6,        ///< a later frame could not be resectioned (`failed_frame`)
};

/// How the two-view bootstrap picks its second view.
enum class InitBootstrap : int {
    MaxParallax = 0,  ///< the frame with the largest mean disparity to frame 0 (shipped, #247)
    Adjacent = 1,     ///< frame 1 (the pre-#247 choice; near-zero parallax at camera rates)
};

template <math::Scalar T>
struct InitWindowOptions {
    InitBootstrap bootstrap = InitBootstrap::MaxParallax;
    /// Minimum time between the keyframes handed to the alignment (s). 0 (the
    /// shipped behaviour) makes every frame a keyframe. Otherwise the SfM still
    /// places every frame, but consecutive frames are merged, composing their
    /// preintegration, until at least this much time has passed. The alignment
    /// reads metric scale from accelerations, i.e. second differences of the
    /// positions, so close keyframes amplify a position error by ~1/Δt² (#472).
    double min_keyframe_dt = 0.0;
    /// RANSAC gates of the two-view bootstrap and of the per-frame PnP. The
    /// defaults are fixed in normalized units; `init_window_options_for_noise`
    /// derives them from the track noise instead (#500).
    TwoViewOptions<T> two_view{};
    PnpOptions<T> pnp{};
    /// The bootstrap pair's mean disparity must reach this (normalized units),
    /// else the build stops with `NoBootstrapPair`. Below it the essential
    /// matrix fits pixel noise: at rest on EuRoC MH_05 the bootstrap accepted
    /// pairs of about 1e-4 (0.05 px) and PnP then failed (#500). 0 disables it.
    T min_bootstrap_parallax = T{0};
    /// Refine the placed window by bundle adjustment before it is handed to the
    /// alignment (#501): every track seen in at least `refine_min_views`
    /// frames becomes a landmark (the bootstrap's, plus the others
    /// triangulated from the PnP poses), and the poses and landmarks are
    /// adjusted jointly with frame 0 held. An observation whose reprojection
    /// exceeds `refine_gate` (normalized) after the first pass is dropped and
    /// the adjustment rerun. Off by default.
    bool refine = false;
    std::size_t refine_min_views = 3;
    T refine_gate = T{1} / T{200};
    BundleAdjustOptions<T> bundle_adjust{};
};

/// The init window's gates for a track noise of `sigma` per image coordinate
/// (normalized units: the pixel σ over the focal length). Each RANSAC gate is
/// the 99 % quantile of its residual under that noise: the Sampson distance
/// has one degree of freedom (√χ²₁ = 2.58 σ) and the PnP reprojection error
/// two (√χ²₂ = 3.03 σ). The bootstrap needs a mean disparity of 20 σ (#500).
template <math::Scalar T>
[[nodiscard]] InitWindowOptions<T> init_window_options_for_noise(T sigma) {
    InitWindowOptions<T> o;
    o.two_view.sampson_threshold = T(2.576) * sigma;
    o.pnp.reprojection_threshold = T(3.035) * sigma;
    o.two_view.confidence = o.pnp.confidence = 0.999;
    o.two_view.max_iterations = o.pnp.max_iterations = 2000;
    o.min_bootstrap_parallax = T{20} * sigma;
    o.refine = true;
    o.refine_gate = o.pnp.reprojection_threshold;
    o.bundle_adjust.huber = T{2} * sigma;
    return o;
}

/// Result of the window assembly: the per-frame keyframes (one per input frame
/// successfully placed) and whether enough frames were recovered for a dynamic
/// alignment (`try_dynamic` needs ≥ 2, and observability wants more), plus where
/// the build stopped and what the bootstrap and resections saw.
template <math::Scalar T>
struct InitWindowResult {
    bool success = false;
    std::vector<DynInitKeyframe<T>> keyframes;
    std::vector<std::size_t> keyframe_frames;  ///< the input frame each keyframe is
    InitWindowStatus status = InitWindowStatus::TooFewFrames;
    std::size_t bootstrap_frame = 0;            ///< the bootstrap's second view (0 = none)
    T bootstrap_parallax = T{0};                ///< its mean disparity to frame 0 (normalized units)
    T bootstrap_translation_parallax = T{0};    ///< …with the recovered rotation removed (TwoViewResult::parallax)
    std::size_t bootstrap_correspondences = 0;  ///< features it shares with frame 0
    std::size_t landmarks = 0;                  ///< points the bootstrap triangulated
    std::size_t failed_frame = 0;               ///< the frame PnP failed on (`PnpFailed`)
    std::vector<std::size_t> pnp_inliers;       ///< per frame (0 for frame 0 and the bootstrap view)
    std::vector<T> pnp_rms;                     ///< per frame: reprojection RMS over the PnP inliers
    // With InitWindowOptions::refine (#501):
    std::size_t refined_landmarks = 0;     ///< landmarks the adjustment used
    std::size_t refined_observations = 0;  ///< observations it kept after the outlier gate
    T refine_rms_before = T{0};            ///< RMS reprojection (normalized) of those, before
    T refine_rms_after = T{0};             ///< …and after
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

/// Bundle-adjust the placed window in place (`InitWindowOptions::refine`). The
/// poses are camera→world (`R_w_cam`, `p_w_cam`); `land_ids` / `land_pts` are
/// the bootstrap's landmarks.
template <math::Scalar T>
void refine_window(std::span<const InitFrame<T>> frames,
                   const std::vector<std::uint64_t>& land_ids,
                   const std::vector<Vec3<T>>& land_pts,
                   std::vector<math::lie::SO3<T>>& R_w_cam,
                   std::vector<Vec3<T>>& p_w_cam,
                   const InitWindowOptions<T>& opt,
                   InitWindowResult<T>& out) {
    const std::size_t n = frames.size();
    std::vector<Mat3<T>> R(n);
    std::vector<Vec3<T>> t(n);
    for (std::size_t f = 0; f < n; ++f) {
        R[f] = ld::transpose(R_w_cam[f].matrix());  // world→camera
        t[f] = R[f] * p_w_cam[f] * (-T{1});
    }
    // Every track's observations, in frame order.
    struct Track {
        std::uint64_t id;
        std::vector<std::pair<std::size_t, Vec2<T>>> seen;
    };
    std::vector<Track> tracks;
    for (std::size_t f = 0; f < n; ++f)
        for (std::size_t i = 0; i < frames[f].ids.size(); ++i) {
            const std::uint64_t id = frames[f].ids[i];
            Track* tr = nullptr;
            for (auto& x : tracks)
                if (x.id == id) {
                    tr = &x;
                    break;
                }
            if (tr == nullptr) {
                tracks.push_back(Track{id, {}});
                tr = &tracks.back();
            }
            tr->seen.emplace_back(f, frames[f].obs[i]);
        }
    // Landmarks: the bootstrap's, and every other track seen often enough that
    // triangulates in front of its cameras within the gate in every view.
    std::vector<Vec3<T>> pts;
    std::vector<const Track*> owner;
    for (const auto& tr : tracks) {
        std::size_t b = land_ids.size();
        for (std::size_t j = 0; j < land_ids.size(); ++j)
            if (land_ids[j] == tr.id) {
                b = j;
                break;
            }
        Vec3<T> X;
        if (b < land_ids.size()) {
            X = land_pts[b];
        } else {
            if (tr.seen.size() < opt.refine_min_views)
                continue;
            std::vector<Mat3<T>> Rv;
            std::vector<Vec3<T>> tv;
            std::vector<Vec2<T>> xv;
            for (const auto& [f, x] : tr.seen) {
                Rv.push_back(R[f]);
                tv.push_back(t[f]);
                xv.push_back(x);
            }
            if (!triangulate_views<T>(Rv, tv, xv, X))
                continue;
            bool ok = true;
            for (const auto& [f, x] : tr.seen) {
                Vec2<T> r;
                if (!reproject<T>(R[f], t[f], X, x, r) || !(ld::norm(r) <= opt.refine_gate)) {
                    ok = false;
                    break;
                }
            }
            if (!ok)
                continue;
        }
        pts.push_back(X);
        owner.push_back(&tr);
    }
    std::vector<BaObservation<T>> obs;
    for (std::size_t p = 0; p < pts.size(); ++p)
        for (const auto& [f, x] : owner[p]->seen)
            obs.push_back(BaObservation<T>{f, p, x});
    std::vector<char> fixed(n, 0);
    fixed[0] = 1;
    const auto first = bundle_adjust<T>(R, t, pts, obs, fixed, opt.bundle_adjust);
    if (!first.success)
        return;
    // Drop what the refined geometry does not explain, and adjust again.
    std::vector<BaObservation<T>> kept;
    for (const auto& o : obs) {
        Vec2<T> r;
        if (reproject<T>(R[o.frame], t[o.frame], pts[o.point], o.x, r) && ld::norm(r) <= opt.refine_gate)
            kept.push_back(o);
    }
    const auto second = bundle_adjust<T>(R, t, pts, kept, fixed, opt.bundle_adjust);
    if (!second.success)
        return;
    for (std::size_t f = 0; f < n; ++f) {
        const Mat3<T> Rt = ld::transpose(R[f]);
        R_w_cam[f] = so3_from_matrix<T>(Rt);
        p_w_cam[f] = Rt * t[f] * (-T{1});
    }
    out.refined_landmarks = pts.size();
    out.refined_observations = kept.size();
    out.refine_rms_before = first.initial_rms;
    out.refine_rms_after = second.final_rms;
}

}  // namespace detail

/// Build the dynamic-init keyframes from the window. Returns `success = false`
/// (and no keyframes) when the bootstrap or a resection fails or too few frames
/// are placed. `extr` is the camera→IMU pose (identity = camera at the IMU).
template <math::Scalar T>
[[nodiscard]] InitWindowResult<T> build_init_window(std::span<const InitFrame<T>> frames,
                                                    const msckf::CameraExtrinsics<T>& extr = {},
                                                    const InitWindowOptions<T>& opt = {}) {
    using SO3 = math::lie::SO3<T>;
    InitWindowResult<T> out;
    const std::size_t n = frames.size();
    if (n < 3)
        return out;
    // Each frame's ids and obs are parallel arrays; a mismatch would index out
    // of bounds in the correspondence loops below.
    for (const auto& f : frames)
        if (f.ids.size() != f.obs.size()) {
            out.status = InitWindowStatus::MalformedFrame;
            return out;
        }

    // 1) Two-view bootstrap fixes the points (cam-0 frame, up to scale). cam0 is
    //    the world origin; the second view's world pose is the inverse of the
    //    recovered cam0→cam_b transform. Pick the bootstrap view by the LARGEST
    //    parallax with frame 0 rather than the adjacent frame: at camera rates
    //    (e.g. 20 Hz) consecutive frames have near-zero parallax, a degenerate
    //    essential matrix, and the up-to-scale geometry — hence the window's
    //    scale gauge — collapses (#247). Select by a cheap mean-disparity scan,
    //    then run the essential-matrix RANSAC ONCE on the best pair (running it
    //    on every candidate is far too slow for a per-frame init retry).
    //    InitBootstrap::Adjacent considers frame 1 only.
    std::vector<Vec2<T>> x0, x1;
    std::vector<std::uint64_t> ids01;
    std::size_t b = 0;  // chosen second bootstrap view (0 = none found)
    T best_par = T{0};
    const std::size_t last = opt.bootstrap == InitBootstrap::Adjacent ? 2 : n;
    for (std::size_t f = 1; f < last; ++f) {
        std::vector<Vec2<T>> a0, a1;
        std::vector<std::uint64_t> ids;
        detail::shared<T>(frames[0], frames[f], a0, a1, ids);
        if (ids.size() < 8)
            continue;  // too few correspondences for the 8-point estimate
        T par = T{0};
        for (std::size_t k = 0; k < a0.size(); ++k)
            par += math::lie::detail::norm(a0[k] - a1[k]);
        par /= static_cast<T>(a0.size());  // mean feature disparity
        if (par > best_par) {
            best_par = par;
            b = f;
            x0 = std::move(a0);
            x1 = std::move(a1);
            ids01 = std::move(ids);
        }
    }
    out.status = InitWindowStatus::NoBootstrapPair;
    if (b == 0 || best_par < opt.min_bootstrap_parallax)
        return out;  // no frame shares enough features with frame 0, or none moved enough
    out.bootstrap_frame = b;
    out.bootstrap_parallax = best_par;
    out.bootstrap_correspondences = ids01.size();
    const auto tv = estimate_relative_pose<T>(x0, x1, opt.two_view);
    out.status = InitWindowStatus::TwoViewFailed;
    if (!tv.success)
        return out;
    out.bootstrap_translation_parallax = tv.parallax;
    // The same floor on the parallax translation produced: a pure rotation
    // passes the raw-disparity check above, but its baseline is noise.
    out.status = InitWindowStatus::NoBootstrapPair;
    if (tv.parallax < opt.min_bootstrap_parallax)
        return out;

    // World landmarks (cam-0 frame), keyed by track id.
    std::vector<std::uint64_t> land_ids;
    std::vector<Vec3<T>> land_pts;
    for (std::size_t k = 0; k < tv.inliers.size(); ++k) {
        land_ids.push_back(ids01[tv.inliers[k]]);
        land_pts.push_back(tv.points_cam0[k]);
    }
    out.landmarks = land_pts.size();
    out.status = InitWindowStatus::TooFewLandmarks;
    if (land_pts.size() < 6)
        return out;  // PnP needs ≥ 6 landmarks for the later frames

    // Camera→world poses (world = cam0). cam0 = identity; the bootstrap view b
    // from the two-view solve; every other frame by PnP below.
    std::vector<SO3> R_w_cam(n);
    std::vector<Vec3<T>> p_w_cam(n);
    std::vector<bool> placed(n, false);
    R_w_cam[0] = SO3{};
    p_w_cam[0] = Vec3<T>{};
    placed[0] = true;
    // cam_b: X_cb = R_b_0 X_c0 + t ⇒ world←cam_b = (R_b_0ᵀ, −R_b_0ᵀ t).
    const Mat3<T> R10t = math::lie::detail::transpose(tv.R_1_0);
    R_w_cam[b] = so3_from_matrix<T>(R10t);
    p_w_cam[b] = R10t * tv.t_1_0_unit * (-T{1});
    placed[b] = true;

    // 2) PnP every remaining frame against the world landmarks it observes.
    out.pnp_inliers.assign(n, 0);
    out.pnp_rms.assign(n, T{0});
    for (std::size_t f = 1; f < n; ++f) {
        if (placed[f])
            continue;
        std::vector<Vec3<T>> P;
        std::vector<Vec2<T>> obs;
        for (std::size_t i = 0; i < frames[f].ids.size(); ++i)
            for (std::size_t j = 0; j < land_ids.size(); ++j)
                if (frames[f].ids[i] == land_ids[j]) {
                    P.push_back(land_pts[j]);
                    obs.push_back(frames[f].obs[i]);
                    break;
                }
        const auto pose = estimate_pose<T>(P, obs, opt.pnp);
        if (!pose.success) {
            out.status = InitWindowStatus::PnpFailed;
            out.failed_frame = f;
            return out;
        }
        T ss = T{0};
        for (const std::size_t k : pose.inliers) {
            const T e = detail::reprojection_error<T>(pose.R, pose.t, P[k], obs[k]);
            ss += e * e;
        }
        out.pnp_inliers[f] = pose.inliers.size();
        out.pnp_rms[f] = ld::sqrt_(ss / static_cast<T>(pose.inliers.size()));
        // PnP: X_cam = R·X_world + t ⇒ world←cam = (Rᵀ, −Rᵀ t).
        const Mat3<T> Rt = math::lie::detail::transpose(pose.R);
        R_w_cam[f] = so3_from_matrix<T>(Rt);
        p_w_cam[f] = Rt * pose.t * (-T{1});
    }

    // 2b) Refine the placed window by bundle adjustment (#501).
    if (opt.refine)
        detail::refine_window<T>(frames, land_ids, land_pts, R_w_cam, p_w_cam, opt, out);

    // 3) Compose camera→world with the camera→IMU extrinsics to body poses, and
    //    carry the preintegration deltas. T_w_imu = T_w_cam · T_cam_imu, with
    //    T_cam_imu = (R_imu_camᵀ, −R_imu_camᵀ p_imu_cam).
    const SO3 R_cam_imu = extr.R_imu_cam.inverse();
    const Vec3<T> p_cam_imu = R_cam_imu * extr.p_imu_cam * (-T{1});
    //    With `min_keyframe_dt`, consecutive intervals are composed (i→j then
    //    j→k) until the keyframe spacing is reached:
    //      ΔR = ΔR_ij·ΔR_jk,  Δv = Δv_ij + ΔR_ij·Δv_jk,
    //      Δp = Δp_ij + Δv_ij·Δt_jk + ΔR_ij·Δp_jk,
    //      ∂ΔR/∂b_g = ΔR_jkᵀ·∂ΔR_ij/∂b_g + ∂ΔR_jk/∂b_g,  Δt = Δt_ij + Δt_jk,
    //      ∂Δv/∂b_a = ∂Δv_ij/∂b_a + ΔR_ij·∂Δv_jk/∂b_a,
    //      ∂Δp/∂b_a = ∂Δp_ij/∂b_a + ∂Δv_ij/∂b_a·Δt_jk + ΔR_ij·∂Δp_jk/∂b_a.
    //    The last frame always closes a keyframe.
    const auto keyframe = [&](std::size_t f) {
        DynInitKeyframe<T> kf;
        kf.R_world_imu = R_w_cam[f] * R_cam_imu;
        kf.p_world_imu = R_w_cam[f] * p_cam_imu + p_w_cam[f];
        return kf;
    };
    if (!(opt.min_keyframe_dt > 0.0)) {
        out.keyframes.resize(n);
        for (std::size_t f = 0; f < n; ++f) {
            DynInitKeyframe<T>& kf = out.keyframes[f];
            kf = keyframe(f);
            kf.dR = frames[f].dR;
            kf.dv = frames[f].dv;
            kf.dp = frames[f].dp;
            kf.dR_dbg = frames[f].dR_dbg;
            kf.dv_dba = frames[f].dv_dba;
            kf.dp_dba = frames[f].dp_dba;
            kf.dt = frames[f].dt;
            out.keyframe_frames.push_back(f);
        }
    } else {
        out.keyframes.push_back(keyframe(0));  // the first keyframe has no preintegration
        out.keyframe_frames.push_back(0);
        SO3 dR{};
        Vec3<T> dv{}, dp{};
        Mat3<T> J{}, Jv{}, Jp{};  // ∂ΔR/∂bg, ∂Δv/∂ba, ∂Δp/∂ba of the composed interval
        T dt = T{0};
        for (std::size_t f = 1; f < n; ++f) {
            const auto& fr = frames[f];
            dp = dp + dv * fr.dt + dR * fr.dp;
            Jp = Jp + Jv * fr.dt + dR.matrix() * fr.dp_dba;
            dv = dv + dR * fr.dv;
            Jv = Jv + dR.matrix() * fr.dv_dba;
            J = fr.dR.inverse().matrix() * J + fr.dR_dbg;
            dR = dR * fr.dR;
            dt = dt + fr.dt;
            if (static_cast<double>(dt) >= opt.min_keyframe_dt || f + 1 == n) {
                DynInitKeyframe<T> kf = keyframe(f);
                kf.dR = dR;
                kf.dv = dv;
                kf.dp = dp;
                kf.dR_dbg = J;
                kf.dv_dba = Jv;
                kf.dp_dba = Jp;
                kf.dt = dt;
                out.keyframes.push_back(kf);
                out.keyframe_frames.push_back(f);
                dR = SO3{};
                dv = Vec3<T>{};
                dp = Vec3<T>{};
                J = Mat3<T>{};
                Jv = Mat3<T>{};
                Jp = Mat3<T>{};
                dt = T{0};
            }
        }
    }
    out.success = true;
    out.status = InitWindowStatus::Ok;
    return out;
}

}  // namespace branes::sdk::sfm

#endif  // BRANES_SDK_SFM_INIT_WINDOW_HPP
