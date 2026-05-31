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
#include <tuple>
#include <utility>
#include <vector>

namespace branes::sdk {

// The dynamic alignment reuses the SDK's small dense linear-algebra
// facility (the only runtime-sized SPD solver in the SDK).
using msckf::cholesky;
using msckf::cholesky_solve;
using msckf::DynMat;
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
    /// Looser band for the gravity-only attitude bootstrap (try_gravity_align):
    /// over a short window even a moving platform's mean specific force is
    /// gravity-dominated, so its *direction* is usable for roll/pitch as long
    /// as its magnitude is within this fraction of g. Wider than gravity_tol
    /// because we only need a direction, not a rest condition.
    T gravity_align_tol = T{30} / T{100};  ///< 30 %
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
    /// Metric scale of the vision-estimated keyframe positions: a `p_world_imu`
    /// supplied up to scale (e.g. from a two-view SfM bootstrap) is metric when
    /// multiplied by `scale`. Resolved by `try_dynamic`; 1 on the other paths,
    /// where poses are already metric.
    T scale = T{1};
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

    /// Gravity-only attitude bootstrap for a platform that is *not* at rest
    /// (so try_static rejects it) but for which no visual-inertial keyframes
    /// are available yet (so try_dynamic cannot run). Over a short window the
    /// mean specific force is dominated by gravity, so its direction fixes
    /// roll/pitch (yaw stays at zero, unobservable). This is strictly better
    /// than a blind identity attitude, which on a tilted mount leaves gravity
    /// uncancelled in propagation and diverges the filter.
    ///
    /// `success = false` only when the window is empty/mismatched or the mean
    /// specific force is not plausibly gravity (magnitude outside
    /// `gravity_align_tol`), in which case its direction is meaningless and
    /// the caller should fall back to identity. The gyro mean is trusted as
    /// the bias only when the window is rotation-quiet (within `max_gyro_std`);
    /// on a rotating start it is real angular rate, not bias, so bias stays 0.
    [[nodiscard]] Result try_gravity_align(std::span<const Vec3> gyro, std::span<const Vec3> accel) const {
        Result r;
        const std::size_t n = accel.size();
        if (n == 0 || gyro.size() != n)
            return r;

        const Vec3 accel_mean = mean(accel);
        const T g_meas = math::lie::detail::norm(accel_mean);
        if (!(g_meas > T{0}))
            return r;
        const T rel = abs_(g_meas - cfg_.gravity_magnitude) / cfg_.gravity_magnitude;
        if (rel > cfg_.gravity_align_tol)
            return r;  // mean force isn't gravity-like — direction unusable

        const Vec3 up_body = accel_mean * (T{1} / g_meas);
        r.R_world_imu = align_to_world_up(up_body);
        r.gravity_world = Vec3{{T{0}, T{0}, -cfg_.gravity_magnitude}};
        const Vec3 gyro_mean = mean(gyro);
        r.gyro_bias = (std_dev(gyro, gyro_mean) <= cfg_.max_gyro_std) ? gyro_mean : Vec3{};
        r.accel_bias = Vec3{};
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

        // The alignment recovers gravity in the (arbitrary) vision world
        // frame. Rotate the whole result into a gravity-aligned world —
        // gravity along −z — so that R_world_imu and gravity_world are
        // mutually consistent, exactly as on the static path. The rotation
        // is about a horizontal axis, so it only corrects roll/pitch and
        // preserves the vision yaw gauge.
        const T g_norm = math::lie::detail::norm(r.gravity_world);
        if (!(g_norm > T{0}))
            return r;                                              // degenerate gravity ⇒ leave success = false
        const Vec3 up_world = r.gravity_world * (-T{1} / g_norm);  // "up" = −gravity
        const SO3 R_align = align_to_world_up(up_world);           // vision → gravity world
        for (auto& vel : r.velocities_world)
            vel = R_align * vel;
        r.gravity_world = R_align * r.gravity_world;  // now (0,0,−g_norm)
        r.R_world_imu = R_align * kfs[0].R_world_imu;
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

    // Linear alignment for the gravity vector, per-keyframe velocities, and the
    // metric scale of the vision poses. The keyframe positions arrive only up to
    // scale (a two-view SfM bootstrap cannot fix the baseline magnitude), so the
    // metric position is s·pᵢ and s is an unknown alongside gravity. Between
    // consecutive keyframes:
    //   s (pⱼ − pᵢ) = vᵢ Δt − ½ g Δt² + Rᵢ Δp
    //   vⱼ − vᵢ      = −g Δt + Rᵢ Δv
    //
    // Solved unconstrained, [v, g, s] is rank-deficient: a constant velocity
    // offset trades off against gravity magnitude and scale. So this follows
    // VINS-Mono (§V): a rough solve seeds the gravity *direction*, then gravity
    // is constrained to the known magnitude G and refined on its 2-D tangent
    // (unknowns [v, b₁, b₂, s]) over a few iterations, which pins the gauge and
    // recovers the metric scale. Velocities and gravity come out metric.
    [[nodiscard]] bool estimate_gravity_and_velocities(std::span<const DynInitKeyframe<T>> kfs, Result& r) const {
        const std::size_t n = kfs.size();
        Vec3 g_rough;
        if (!solve_alignment(kfs, /*g_hat=*/nullptr, g_rough, r))
            return false;  // rough pass: also yields the gravity direction to refine

        const T g_norm = math::lie::detail::norm(g_rough);
        if (!(g_norm > T{0}))
            return false;
        Vec3 g_hat = g_rough * (T{1} / g_norm);
        Vec3 g_final = g_rough;
        for (int iter = 0; iter < 4; ++iter) {  // refine on the |g|=G sphere tangent
            if (!solve_alignment(kfs, &g_hat, g_final, r))
                return false;
            const T gn = math::lie::detail::norm(g_final);
            if (!(gn > T{0}))
                return false;
            g_hat = g_final * (T{1} / gn);
        }
        r.gravity_world = g_final;
        if (!(r.scale > T{0}))
            return false;  // a non-positive metric scale is non-physical ⇒ reject
        (void)n;
        return true;
    }

    // One linear solve of the alignment normal equations. With `g_hat == nullptr`
    // gravity is a free 3-vector (unknowns [v, g, s], the rough pass) and `g_out`
    // returns the recovered gravity. Otherwise gravity is pinned to the known
    // magnitude G along `*g_hat` and refined on its tangent (unknowns [v, b₁, b₂,
    // s]), `g_out = G·ĝ + w₁ b₁ + w₂ b₂`. Either way `r.velocities_world` and
    // `r.scale` are filled. Returns false if the system is singular.
    [[nodiscard]] bool
    solve_alignment(std::span<const DynInitKeyframe<T>> kfs, const Vec3* g_hat, Vec3& g_out, Result& r) const {
        const std::size_t n = kfs.size();
        const bool constrained = g_hat != nullptr;
        // Column layout: velocities [0, 3n); then gravity — a free 3-vector
        // (g0..g0+2) when rough, or two tangent coeffs (g0, g0+1) when refining;
        // then the scale column.
        const std::size_t g0 = 3 * n;
        const std::size_t gw = constrained ? 2 : 3;  // gravity unknown width
        const std::size_t s0 = g0 + gw;
        const std::size_t dim = s0 + 1;

        Vec3 w1{}, w2{}, base{};
        if (constrained) {
            tangent_basis(*g_hat, w1, w2);
            base = *g_hat * cfg_.gravity_magnitude;  // fixed-magnitude gravity base
        }

        DynMat<T> H(dim, dim);
        DynMat<T> b(dim, 1);
        // Fold one 3-row vector constraint J·x = rhs (given as its nonzero
        // (row, col, value) entries) into the normal equations. A generic entry
        // list lets the scalar columns (tangent coeffs, scale) share the system
        // with the 3-vector unknowns.
        std::vector<std::tuple<std::size_t, std::size_t, T>> e;
        auto diag = [&](std::size_t col, T c) {
            for (std::size_t k = 0; k < 3; ++k)
                e.emplace_back(k, col + k, c);
        };
        auto col = [&](std::size_t c, const Vec3& v) {
            for (std::size_t k = 0; k < 3; ++k)
                e.emplace_back(k, c, v[k]);
        };
        auto flush = [&](const Vec3& rhs) {
            for (const auto& [r1, c1, v1] : e) {
                for (const auto& [r2, c2, v2] : e)
                    if (r1 == r2)
                        H(c1, c2) += v1 * v2;
                b(c1, 0) += v1 * rhs[r1];
            }
            e.clear();
        };

        for (std::size_t i = 1; i < n; ++i) {
            const std::size_t vi = 3 * (i - 1);
            const std::size_t vj = 3 * i;
            const Mat3 R_i = kfs[i - 1].R_world_imu.matrix();
            const T dt = kfs[i].dt;
            const Vec3 dp_world = kfs[i].p_world_imu - kfs[i - 1].p_world_imu;

            // Position: vᵢ Δt − ½ g Δt² − s (pⱼ − pᵢ) = −Rᵢ Δp.
            diag(vi, dt);
            col(s0, dp_world * (-T{1}));
            if (constrained) {
                col(g0, w1 * (-T{0.5} * dt * dt));
                col(g0 + 1, w2 * (-T{0.5} * dt * dt));
                flush(R_i * kfs[i].dp * (-T{1}) + base * (T{0.5} * dt * dt));
            } else {
                diag(g0, -T{0.5} * dt * dt);
                flush(R_i * kfs[i].dp * (-T{1}));
            }

            // Velocity: vⱼ − vᵢ + g Δt = Rᵢ Δv (scale-free).
            diag(vj, T{1});
            diag(vi, -T{1});
            if (constrained) {
                col(g0, w1 * dt);
                col(g0 + 1, w2 * dt);
                flush(R_i * kfs[i].dv - base * dt);
            } else {
                diag(g0, dt);
                flush(R_i * kfs[i].dv);
            }
        }

        // The velocities need motion to be observable; a small ridge
        // regularizes. Factor once and reuse the factor for the solve.
        for (std::size_t i = 0; i < dim; ++i)
            H(i, i) += T(1e-9);
        DynMat<T> L;
        if (!cholesky(H, L))
            return false;
        const DynMat<T> x = cholesky_solve(L, b);

        r.velocities_world.resize(n);
        for (std::size_t i = 0; i < n; ++i)
            r.velocities_world[i] = Vec3{{x(3 * i, 0), x(3 * i + 1, 0), x(3 * i + 2, 0)}};
        r.scale = x(s0, 0);
        g_out = constrained ? base + w1 * x(g0, 0) + w2 * x(g0 + 1, 0) : Vec3{{x(g0, 0), x(g0 + 1, 0), x(g0 + 2, 0)}};
        return true;
    }

    // Two orthonormal vectors spanning the tangent plane of the unit `g_hat`.
    static void tangent_basis(const Vec3& g_hat, Vec3& w1, Vec3& w2) {
        const Vec3 a = abs_(g_hat[0]) < T{9} / T{10} ? Vec3{{T{1}, T{0}, T{0}}} : Vec3{{T{0}, T{0}, T{1}}};
        Vec3 t = a - g_hat * math::lie::detail::dot(a, g_hat);
        const T tn = math::lie::detail::norm(t);
        w1 = tn > T{0} ? t * (T{1} / tn) : Vec3{{T{1}, T{0}, T{0}}};
        w2 = math::lie::detail::cross(g_hat, w1);
    }

    Config cfg_;
};

}  // namespace branes::sdk

#endif  // BRANES_SDK_IMU_INIT_HPP
