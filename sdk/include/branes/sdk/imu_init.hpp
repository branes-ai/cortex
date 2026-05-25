// SPDX-License-Identifier: MIT
//
// branes/sdk/imu_init.hpp — visual-inertial initialization.
//
// Bootstraps the estimator's initial state from the first seconds of
// data, in two flavours:
//
//   • Static  — the platform is stationary. The gyroscope mean is the
//     gyro bias; the accelerometer mean points along −gravity, fixing the
//     initial roll/pitch (yaw is unobservable and left at zero).
//
//   • Dynamic — no static window is available, so the platform is moving.
//     Recover the gyro bias from the rotation mismatch between vision and
//     IMU preintegration, then the gravity direction and the per-keyframe
//     velocities from the (metric) position/velocity preintegration
//     constraints. This is the linear visual-inertial alignment.
//
// Clean-room from the papers only — Qin et al., "VINS-Mono" (2018), and
// Martinelli, "Closed-Form Solution of Visual-Inertial Structure from
// Motion" (2014). No third-party VIO source.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_IMU_INIT_HPP
#define BRANES_SDK_IMU_INIT_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/dense.hpp>  // DynMat + spd_solve for the linear alignment

#include <cstddef>
#include <initializer_list>
#include <span>
#include <utility>
#include <vector>

namespace branes::sdk {

// The dynamic alignment reuses the SDK's small dense linear-algebra
// facility (the only runtime-sized SPD solver in the SDK).
using msckf::DynMat;
using msckf::is_positive_definite;
using msckf::spd_solve;

/// Knobs for the initializer. Defaults target a consumer MEMS IMU.
template <math::Scalar T>
struct ImuInitConfig {
    T gravity_magnitude = T{981} / T{100};  ///< 9.81 m/s²
    /// Stationarity gates (static init). The accelerometer-magnitude and
    /// gyro standard deviations over the window must stay below these for
    /// the window to count as "at rest".
    T max_accel_std = T{1} / T{2};   ///< m/s²
    T max_gyro_std = T{1} / T{100};  ///< rad/s
    /// The measured gravity magnitude must match `gravity_magnitude` to
    /// within this fraction, else the window is rejected (e.g. the device
    /// was accelerating, not at rest).
    T gravity_tol = T{5} / T{100};  ///< 5 %
};

/// What initialization recovers. `R_world_imu` is gravity-aligned with
/// zero yaw; `gravity_world` is the gravity vector in the world frame
/// (≈ (0,0,−g)). For dynamic init, `velocities_world` holds one velocity
/// per input keyframe; it is empty after static init.
template <math::Scalar T>
struct ImuInitResult {
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using SO3 = math::lie::SO3<T>;

    bool success = false;
    Vec3 gyro_bias{};
    Vec3 accel_bias{};
    Vec3 gravity_world{};
    SO3 R_world_imu{};
    std::vector<Vec3> velocities_world;
};

/// One keyframe for dynamic initialization. The caller supplies the
/// vision-estimated IMU pose in the world frame (camera poses already
/// composed with the camera→IMU extrinsics, so these are *body* poses),
/// plus the IMU preintegration accumulated from the previous keyframe.
/// `dt` is the preintegration interval. `dR_dbg` is ∂ΔR/∂bg.
template <math::Scalar T>
struct DynInitKeyframe {
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Mat3 = math::lie::detail::Mat<T, 3, 3>;
    using SO3 = math::lie::SO3<T>;

    SO3 R_world_imu{};   ///< body orientation in world (from vision)
    Vec3 p_world_imu{};  ///< body position in world, metric (from vision)
    SO3 dR{};            ///< preintegrated rotation since previous keyframe
    Vec3 dv{};           ///< preintegrated velocity
    Vec3 dp{};           ///< preintegrated position
    Mat3 dR_dbg{};       ///< ∂ΔR/∂(gyro bias)
    T dt = T{0};         ///< interval covered by the preintegration
};

template <math::Scalar T>
class ImuInitializer {
public:
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Mat3 = math::lie::detail::Mat<T, 3, 3>;
    using SO3 = math::lie::SO3<T>;
    using Result = ImuInitResult<T>;
    using Config = ImuInitConfig<T>;

    explicit ImuInitializer(const Config& cfg = {}) : cfg_(cfg) {}

    /// Static initialization from a window of stationary IMU samples.
    /// Returns `success = false` (and an otherwise-default result) when
    /// the window is empty or fails the stationarity / gravity gates.
    [[nodiscard]] Result try_static(std::span<const Vec3> gyro, std::span<const Vec3> accel) const {
        Result r;
        const std::size_t n = gyro.size();
        if (n == 0 || accel.size() != n)
            return r;

        const Vec3 gyro_mean = mean(gyro);
        const Vec3 accel_mean = mean(accel);
        // Reject the window unless it is genuinely at rest and the
        // measured gravity magnitude is sane.
        if (std_dev(gyro, gyro_mean) > cfg_.max_gyro_std)
            return r;
        if (std_dev(accel, accel_mean) > cfg_.max_accel_std)
            return r;
        const T g_meas = math::lie::detail::norm(accel_mean);
        if (!(g_meas > T{0}))
            return r;
        const T rel = abs_(g_meas - cfg_.gravity_magnitude) / cfg_.gravity_magnitude;
        if (rel > cfg_.gravity_tol)
            return r;

        // Stationary ⇒ true angular rate is zero, so the gyro mean is the
        // bias. The accelerometer reads specific force a_m = Rᵀ(−g), i.e.
        // it points "up" in the body frame; align that with world +z.
        r.gyro_bias = gyro_mean;
        r.accel_bias = Vec3{};  // unobservable from one static window
        const Vec3 up_body = accel_mean * (T{1} / g_meas);
        r.R_world_imu = align_to_world_up(up_body);
        r.gravity_world = Vec3{{T{0}, T{0}, -cfg_.gravity_magnitude}};
        r.success = true;
        return r;
    }

    /// Dynamic initialization from ≥ 2 keyframes. Recovers the gyro bias
    /// (one Gauss-Newton step on the vision/IMU rotation mismatch), then
    /// the gravity vector and per-keyframe velocities by linear least
    /// squares on the preintegration constraints. `success = false` if
    /// there are too few keyframes or the linear system is singular.
    [[nodiscard]] Result try_dynamic(std::span<const DynInitKeyframe<T>> kfs) const {
        Result r;
        const std::size_t n = kfs.size();
        if (n < 2)
            return r;

        r.gyro_bias = estimate_gyro_bias(kfs);
        if (!estimate_gravity_and_velocities(kfs, r))
            return r;
        // Build the gravity-aligned attitude at the first keyframe, with
        // its yaw taken from the vision pose (which fixes the gauge).
        r.R_world_imu = kfs[0].R_world_imu;
        r.accel_bias = Vec3{};
        r.success = true;
        return r;
    }

private:
    static T abs_(const T& x) {
        return math::lie::detail::abs_(x);
    }

    static Vec3 mean(std::span<const Vec3> v) {
        Vec3 s{};
        for (const auto& x : v)
            s = s + x;
        return s * (T{1} / static_cast<T>(v.size()));
    }

    // Per-axis standard deviation, reduced to a single scalar (the max
    // over axes) so one threshold gates the whole window.
    static T std_dev(std::span<const Vec3> v, const Vec3& m) {
        Vec3 var{};
        for (const auto& x : v) {
            const Vec3 d = x - m;
            for (std::size_t i = 0; i < 3; ++i)
                var[i] += d[i] * d[i];
        }
        const T inv = T{1} / static_cast<T>(v.size());
        T worst = T{0};
        for (std::size_t i = 0; i < 3; ++i) {
            const T s = math::lie::detail::sqrt_(var[i] * inv);
            if (s > worst)
                worst = s;
        }
        return worst;
    }

    // Rotation R_world_imu such that R · up_body = world +z. up_body is a
    // unit vector. Handles the parallel / anti-parallel degeneracies.
    static SO3 align_to_world_up(const Vec3& up_body) {
        const Vec3 ez{{T{0}, T{0}, T{1}}};
        const T c = math::lie::detail::dot(up_body, ez);
        const Vec3 v = math::lie::detail::cross(up_body, ez);
        const T s = math::lie::detail::norm(v);
        if (!(s > T(1e-9))) {
            // Parallel (already up) or anti-parallel (upside down).
            if (c > T{0})
                return SO3{};  // identity
            // 180° about world x: maps +z(body) → −z.
            return SO3::exp(Vec3{{kPi(), T{0}, T{0}}});
        }
        const T angle = math::lie::detail::atan2_(s, c);
        const Vec3 axis = v * (angle / s);
        return SO3::exp(axis);
    }

    static T kPi() {
        // 4·atan(1), evaluated in T so posit/float pick their own value.
        return T{4} * math::lie::detail::atan2_(T{1}, T{1});
    }

    // One Gauss-Newton step: minimize Σ ‖Log(ΔRᵢⱼ Exp(Jᵢ δbg))ᵀ Rᵢᵀ Rⱼ‖².
    // Linearizing about δbg = 0 gives Jᵢ δbg ≈ Log(ΔRᵢⱼᵀ Rᵢᵀ Rⱼ); solve the
    // stacked 3×3 normal equations (Σ JᵀJ) δbg = Σ Jᵀ residual.
    static Vec3 estimate_gyro_bias(std::span<const DynInitKeyframe<T>> kfs) {
        Mat3 H{};  // zero-initialized
        Vec3 b{};
        for (std::size_t i = 1; i < kfs.size(); ++i) {
            const SO3 R_i = kfs[i - 1].R_world_imu;
            const SO3 R_j = kfs[i].R_world_imu;
            const Mat3 J = kfs[i].dR_dbg;
            const SO3 dR = kfs[i].dR;
            const Vec3 res = (dR.inverse() * R_i.inverse() * R_j).log();
            const Mat3 Jt = math::lie::detail::transpose(J);
            H = H + Jt * J;
            b = b + Jt * res;
        }
        // Small, dense 3×3 SPD solve (ridge keeps it invertible when the
        // motion barely excites the gyro bias).
        DynMat<T> A(3, 3);
        DynMat<T> rhs(3, 1);
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j)
                A(i, j) = H(i, j);
            A(i, i) += T(1e-9);
            rhs(i, 0) = b[i];
        }
        const DynMat<T> x = spd_solve(A, rhs);
        return Vec3{{x(0, 0), x(1, 0), x(2, 0)}};
    }

    // Linear alignment for the gravity vector and per-keyframe velocities.
    // Between consecutive keyframes (known metric poses Rᵢ, pᵢ):
    //   pⱼ − pᵢ = vᵢ Δt − ½ g Δt² + Rᵢ Δp
    //   vⱼ − vᵢ = −g Δt + Rᵢ Δv
    // Unknowns x = [v₀ … v_{n-1}, g] (3n + 3). Solve the normal equations.
    [[nodiscard]] bool estimate_gravity_and_velocities(std::span<const DynInitKeyframe<T>> kfs, Result& r) const {
        const std::size_t n = kfs.size();
        const std::size_t dim = 3 * n + 3;  // velocities + gravity
        const std::size_t g0 = 3 * n;       // gravity block offset
        DynMat<T> H(dim, dim);
        DynMat<T> b(dim, 1);

        // One vector residual row-block Σ_k Aₖ·x[colₖ] = rhs, folded into
        // the normal equations HᵀH, Hᵀr (each term a (col, 3×3) pair).
        using Term = std::pair<std::size_t, Mat3>;
        auto accumulate = [&](std::initializer_list<Term> terms, const Vec3& rhs) {
            for (const auto& [ca, A] : terms) {
                const Mat3 At = math::lie::detail::transpose(A);
                for (const auto& [cb, B] : terms) {
                    const Mat3 AtB = At * B;
                    for (std::size_t i = 0; i < 3; ++i)
                        for (std::size_t j = 0; j < 3; ++j)
                            H(ca + i, cb + j) += AtB(i, j);
                }
                const Vec3 Atr = At * rhs;
                for (std::size_t i = 0; i < 3; ++i)
                    b(ca + i, 0) += Atr[i];
            }
        };

        const Mat3 I = Mat3::identity();
        for (std::size_t i = 1; i < n; ++i) {
            const std::size_t vi = 3 * (i - 1);
            const std::size_t vj = 3 * i;
            const Mat3 R_i = kfs[i - 1].R_world_imu.matrix();
            const T dt = kfs[i].dt;

            // Position constraint: vᵢ Δt − ½ g Δt² = (pⱼ − pᵢ) − Rᵢ Δp.
            const Vec3 dp_world = kfs[i].p_world_imu - kfs[i - 1].p_world_imu;
            const Vec3 rhs_p = dp_world - R_i * kfs[i].dp;
            accumulate({{vi, I * dt}, {g0, I * (-T{0.5} * dt * dt)}}, rhs_p);

            // Velocity constraint: vⱼ − vᵢ + g Δt = Rᵢ Δv.
            const Vec3 rhs_v = R_i * kfs[i].dv;
            accumulate({{vj, I}, {vi, I * (-T{1})}, {g0, I * dt}}, rhs_v);
        }

        // Gravity is well-determined; the velocities need motion to be
        // observable. A small ridge regularizes the rest.
        for (std::size_t i = 0; i < dim; ++i)
            H(i, i) += T(1e-9);
        if (!is_positive_definite(H))
            return false;
        const DynMat<T> x = spd_solve(H, b);

        r.velocities_world.resize(n);
        for (std::size_t i = 0; i < n; ++i)
            r.velocities_world[i] = Vec3{{x(3 * i, 0), x(3 * i + 1, 0), x(3 * i + 2, 0)}};
        r.gravity_world = Vec3{{x(g0, 0), x(g0 + 1, 0), x(g0 + 2, 0)}};
        return true;
    }

    Config cfg_;
};

}  // namespace branes::sdk

#endif  // BRANES_SDK_IMU_INIT_HPP
