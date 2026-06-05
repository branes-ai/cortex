// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/synthetic_world.hpp — a controllable synthetic VIO world with
// EXACT ground truth, for the end-to-end noise→robustness demo
// (docs/arch/vio-pipeline-canonical.md). The whole VIO premise is additive
// noise: the filter is handed noisy camera + IMU streams and must deliver a
// robust metric estimate. To show that — and measure it — we need a source whose
// ground truth is known exactly, so noise→error is clean and attributable.
//
// The world is a smooth body trajectory (a gentle loop with yaw, starting from a
// stationary warm-up so static init fires) plus a cloud of 3D landmarks ahead of
// a forward-looking camera. It emits the two sensor streams the backend
// consumes — IMU samples (gyro+accel) and per-frame feature observations
// (pixels, with stable track ids) — and the per-frame ground-truth nav state.
// Generation is from the ANALYTIC trajectory (closed-form v, a, ω), so the IMU
// is consistent with the poses and the camera observations are exact
// projections; a separate noise injector (see tools/) corrupts the streams.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_SYNTHETIC_WORLD_HPP
#define BRANES_SDK_EVAL_SYNTHETIC_WORLD_HPP

#include <branes/math/cameras.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace branes::sdk::eval {

/// Knobs for the synthetic world. The motion amplitudes/rates double as the
/// "robot class" lever — gentle (ground robot) to aggressive (drone).
template <math::Scalar T>
struct SyntheticConfig {
    T duration_s = T{14};
    T warmup_s = T{1.5};  ///< stationary start so static init fires
    T imu_rate_hz = T{200};
    T cam_rate_hz = T{20};
    T gravity = T{981} / T{100};

    // Motion (after warm-up): a loop of radius `trans_amp` at angular rate
    // `motion_rate`, with a vertical bob and an oscillating yaw — translation
    // for parallax/scale, rotation for bias/observability.
    T trans_amp = T{0.8};    ///< m
    T motion_rate = T{1.2};  ///< rad/s of the path parameter
    T yaw_amp = T{0.5};      ///< rad of yaw oscillation
    T vbob_amp = T{0.15};    ///< m vertical bob

    // Scene: landmarks ahead of the forward-looking camera (body +x).
    std::size_t num_landmarks = 240;
    T scene_near = T{3};
    T scene_far = T{9};
    T scene_half_extent = T{3};  ///< lateral / vertical half-size of the cloud

    // True biases (constant; the filter must estimate them).
    math::lie::detail::Vec<T, 3> gyro_bias{{T{0.004}, T{-0.002}, T{0.003}}};
    math::lie::detail::Vec<T, 3> accel_bias{{T{0.02}, T{0.01}, T{-0.015}}};

    std::uint64_t seed = 0x5EED;
};

/// One ground-truth nav state.
template <math::Scalar T>
struct GtSample {
    double t = 0;
    math::lie::SO3<T> R{};             ///< world ← body
    math::lie::detail::Vec<T, 3> p{};  ///< world position
    math::lie::detail::Vec<T, 3> v{};  ///< world velocity
};

/// One camera frame: timestamp + the pixel observations of the visible
/// landmarks, plus each observation's camera-frame depth (z, metres) in the same
/// order — used to colour-code features by depth so parallax reads at a glance.
template <math::Scalar T>
struct SyntheticFrame {
    double t = 0;
    std::vector<FrontendObservation<T>> obs;
    std::vector<T> depth;  ///< camera-frame z (m) per obs, same order
};

/// The generated world: the two clean sensor streams, the per-frame ground truth,
/// and the camera the backend must be configured with.
template <math::Scalar T>
struct SyntheticData {
    std::vector<ImuMeasurement<T>> imu;     ///< clean gyro+accel
    std::vector<SyntheticFrame<T>> frames;  ///< clean pixel observations
    std::vector<GtSample<T>> gt;            ///< ground truth at each frame time
    math::cameras::PinholeRadtanCamera<T> camera{};
    math::lie::SO3<T> R_imu_cam{};             ///< camera→IMU rotation (extrinsic)
    math::lie::detail::Vec<T, 3> p_imu_cam{};  ///< camera origin in the IMU frame
    math::lie::detail::Vec<T, 3> gyro_bias{};
    math::lie::detail::Vec<T, 3> accel_bias{};
};

namespace sw_detail {
template <math::Scalar T>
using Vec3 = math::lie::detail::Vec<T, 3>;

// EuRoC cam0 intrinsics — the canonical VIO camera.
template <math::Scalar T>
[[nodiscard]] math::cameras::PinholeRadtanCamera<T> euroc_camera() {
    return {T{458654} / T{1000},
            T{457296} / T{1000},
            T{367215} / T{1000},
            T{248375} / T{1000},
            T{-28340811} / T{100000000},
            T{7395907} / T{100000000},
            T{19359} / T{100000000},
            T{176187114} / T{10000000000}};
}
constexpr double kImgW = 752, kImgH = 480;
}  // namespace sw_detail

/// Generate the world. The analytic trajectory: stationary for `warmup_s`, then
/// p(t') = amp·(1−cos, sin) loop + vertical bob, R = yaw oscillation about world z.
template <math::Scalar T>
[[nodiscard]] SyntheticData<T> generate_world(const SyntheticConfig<T>& cfg) {
    using sw_detail::euroc_camera;
    using sw_detail::kImgH;
    using sw_detail::kImgW;
    using SO3 = math::lie::SO3<T>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using std::cos;
    using std::sin;

    const Vec3 ez{{T{0}, T{0}, T{1}}};
    const Vec3 g_world{{T{0}, T{0}, -cfg.gravity}};
    const T w = cfg.motion_rate;

    // Analytic trajectory (t' = max(0, t − warmup)). Stationary before warm-up.
    auto active = [&](T t) { return t > cfg.warmup_s ? t - cfg.warmup_s : T{0}; };
    auto pos = [&](T t) -> Vec3 {
        const T s = active(t);
        return Vec3{
            {cfg.trans_amp * (T{1} - cos(w * s)), cfg.trans_amp * sin(w * s), cfg.vbob_amp * sin(T{2} * w * s)}};
    };
    auto vel = [&](T t) -> Vec3 {
        if (!(t > cfg.warmup_s))
            return Vec3{};
        const T s = active(t);
        return Vec3{{cfg.trans_amp * w * sin(w * s),
                     cfg.trans_amp * w * cos(w * s),
                     cfg.vbob_amp * T{2} * w * cos(T{2} * w * s)}};
    };
    auto acc = [&](T t) -> Vec3 {
        if (!(t > cfg.warmup_s))
            return Vec3{};
        const T s = active(t);
        return Vec3{{cfg.trans_amp * w * w * cos(w * s),
                     -cfg.trans_amp * w * w * sin(w * s),
                     -cfg.vbob_amp * T{4} * w * w * sin(T{2} * w * s)}};
    };
    auto yaw = [&](T t) { return cfg.yaw_amp * sin(w * active(t)); };
    auto yaw_rate = [&](T t) { return t > cfg.warmup_s ? cfg.yaw_amp * w * cos(w * active(t)) : T{0}; };
    auto rot = [&](T t) { return SO3::exp(ez * yaw(t)); };

    SyntheticData<T> out;
    out.camera = euroc_camera<T>();
    out.gyro_bias = cfg.gyro_bias;
    out.accel_bias = cfg.accel_bias;
    // Forward-looking camera: optical axis (cam +z) → body +x. Quaternion
    // (w,x,y,z) = (½,−½,½,−½) realizes cam→body = [[0,0,1],[−1,0,0],[0,−1,0]].
    out.R_imu_cam = SO3(math::lie::detail::Vec<T, 4>{{T{1} / T{2}, -T{1} / T{2}, T{1} / T{2}, -T{1} / T{2}}});
    out.p_imu_cam = Vec3{{T{5} / T{100}, T{0}, T{0}}};

    // Landmark cloud ahead of the camera (world +x region, since the body looks +x).
    std::mt19937_64 rng(cfg.seed);
    std::uniform_real_distribution<T> ux(cfg.scene_near, cfg.scene_far);
    std::uniform_real_distribution<T> uyz(-cfg.scene_half_extent, cfg.scene_half_extent);
    std::vector<Vec3> landmarks(cfg.num_landmarks);
    for (auto& L : landmarks)
        L = Vec3{{ux(rng), uyz(rng), uyz(rng)}};

    // IMU stream: gyro = ω_body + b_g, accel = Rᵀ(a_world − g) + b_a.
    const T imu_dt = T{1} / cfg.imu_rate_hz;
    const auto n_imu = static_cast<std::size_t>(cfg.duration_s * cfg.imu_rate_hz);
    out.imu.reserve(n_imu);
    for (std::size_t k = 0; k < n_imu; ++k) {
        const T t = static_cast<T>(k) * imu_dt;
        const SO3 R = rot(t);
        const Vec3 wb{{T{0}, T{0}, yaw_rate(t)}};
        const Vec3 sf = R.inverse() * (acc(t) - g_world);
        const Vec3 wm = wb + cfg.gyro_bias;
        const Vec3 am = sf + cfg.accel_bias;
        ImuMeasurement<T> m;
        m.timestamp_s = static_cast<double>(t);
        // ImuMeasurement carries std::array (the SDK POD boundary), not the Lie Vec.
        m.angular_velocity = {wm[0], wm[1], wm[2]};
        m.linear_acceleration = {am[0], am[1], am[2]};
        out.imu.push_back(m);
    }

    // Camera stream: project the visible landmarks to pixels each frame.
    const T cam_dt = T{1} / cfg.cam_rate_hz;
    const auto n_cam = static_cast<std::size_t>(cfg.duration_s * cfg.cam_rate_hz);
    out.frames.reserve(n_cam);
    out.gt.reserve(n_cam);
    for (std::size_t f = 0; f < n_cam; ++f) {
        const T t = static_cast<T>(f) * cam_dt;
        // Skip frames during the stationary warm-up: zero-parallax observations
        // there would triangulate degenerately and a feature spanning the
        // warm-up→motion boundary would poison the update. Static init is
        // IMU-only, so the camera stream starts once the platform is moving.
        if (t < cfg.warmup_s)
            continue;
        const SO3 R = rot(t);
        const Vec3 p = pos(t);
        out.gt.push_back(GtSample<T>{static_cast<double>(t), R, p, vel(t)});

        SyntheticFrame<T> frame;
        frame.t = static_cast<double>(t);
        // World→camera: p_cam = R_cam_imu (R_world_imuᵀ (L − p) − p_imu_cam).
        const SO3 R_cam_imu = out.R_imu_cam.inverse();
        for (std::size_t i = 0; i < landmarks.size(); ++i) {
            const Vec3 in_imu = R.inverse() * (landmarks[i] - p);
            const Vec3 in_cam = R_cam_imu * (in_imu - out.p_imu_cam);
            if (!(in_cam[2] > T{0}))
                continue;  // behind the camera
            const auto px = out.camera.project(math::cameras::Vec3<T>{in_cam[0], in_cam[1], in_cam[2]});
            if (px[0] < T{0} || px[0] >= T{kImgW} || px[1] < T{0} || px[1] >= T{kImgH})
                continue;  // outside the image
            FrontendObservation<T> o;
            o.feature_id = i;  // stable across frames → MSCKF track
            o.camera_id = 0;
            o.u = px[0];
            o.v = px[1];
            frame.obs.push_back(o);
            frame.depth.push_back(in_cam[2]);  // camera-frame depth, for colour-coding
        }
        out.frames.push_back(std::move(frame));
    }
    return out;
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_SYNTHETIC_WORLD_HPP
