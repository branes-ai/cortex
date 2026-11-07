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
#include <optional>
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
    /// Low-pass for `vibrating_rest_bias`, in samples (#591). The window is cut
    /// into blocks of this many samples, and `max_gyro_std` bounds the spread
    /// of the block means instead of the samples. At rest the angular rate is
    /// constant, so the test is for a rate that changes. Rotor vibration on a
    /// grounded multirotor changes the reading far faster than the platform
    /// can turn: on EuRoC MH_03 it puts the per-sample std at 0.02–0.04 rad/s
    /// on a start that is still. Averaging over a block removes it, while a
    /// real change of rate, slow compared with the block, survives. 10 samples
    /// is 50 ms at EuRoC's 200 Hz, a 20 Hz low-pass; 1 tests the raw samples.
    std::size_t gyro_lowpass_samples = 10;
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
    /// Scale-observability gate for the dynamic alignment. The metric scale is
    /// only observable with enough translational excitation; on a near-hover
    /// window the rank-deficient solve collapses scale → 0 (the trivial
    /// IMU-only solution), which would seed a degenerate state and diverge the
    /// filter. Require the resolved metric motion across the init window
    /// (scale · max keyframe displacement) to exceed this floor, else decline
    /// dynamic init so the backend retries on a later, better-excited window or
    /// falls back. (scale > 0 alone is insufficient — 1e-8 is "positive".)
    T min_dynamic_motion = T{5} / T{100};  ///< 5 cm of resolved motion
    /// Scale-settling gate for the dynamic alignment (#592). The alignment is
    /// re-solved on the window's first ⌈2n/3⌉ keyframes (the same SfM gauge,
    /// so the two scales are comparable), and declined if that scale differs
    /// from the full window's by more than this fraction: the scale has not
    /// settled, and the window does not determine it. On the S1 bench's 0.1 px
    /// window, whose scale is 21 % off, the two differ by 16 %. It tests
    /// convergence, not accuracy: on EuRoC, settled 1 s windows are still
    /// about 14 % off (median), a bias of the short-baseline SfM track (#501).
    T max_scale_change = T{5} / T{100};  ///< 5 %
    /// Estimate the accelerometer bias in the dynamic alignment, held toward
    /// zero by a prior row of this weight per component (#501). The
    /// alignment's rows are unweighted (metres for the position rows, m/s for
    /// the velocity rows), so the weight is in those units per m/s². A bias the
    /// alignment ignores is read as acceleration and biases the scale: on
    /// EuRoC windows with ground-truth poses, estimating it brings the median
    /// scale error from 5–18 % to about 2 %. Over a 1 s window the bias is
    /// weakly observable, and estimated freely it absorbs the vision noise
    /// instead; 0.01 was best on the refined EuRoC windows. 0 does not
    /// estimate it.
    T accel_bias_weight = T{1} / T{100};
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
    /// Resolved metric motion across the window: `scale · max keyframe
    /// displacement` in vision units. Set by `try_dynamic` whenever the linear
    /// solve runs — even when the observability gate then declines — so callers
    /// can see how close the window came to the `min_dynamic_motion` floor.
    T resolved_motion = T{0};
    /// The scale re-solved on the window's leading ⌈2n/3⌉ keyframes, for the
    /// `max_scale_change` gate; 0 when it was not solved (fewer than 4
    /// keyframes, or the alignment declined earlier).
    T prefix_scale = T{0};
    /// True when the alignment was declined only because its scale had not
    /// settled (the window resolved enough motion, but the prefix disagreed):
    /// the platform is moving and a longer window may yet align.
    bool scale_unsettled = false;
};

/// One keyframe for dynamic initialization. The caller supplies the
/// vision-estimated IMU pose in the world frame (camera poses already
/// composed with the camera→IMU extrinsics, so these are *body* poses),
/// plus the IMU preintegration accumulated from the previous keyframe.
/// `dt` is the preintegration interval. `dR_dbg` is ∂ΔR/∂bg; `dv_dba` and
/// `dp_dba` are ∂Δv/∂ba and ∂Δp/∂ba, for the accelerometer-bias estimate (both
/// zero: not supplied, and the alignment takes −Δt·I and −½Δt²·I, exact for a
/// constant attitude over the interval).
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
    Mat3 dv_dba{};       ///< ∂Δv/∂(accel bias), or zero
    Mat3 dp_dba{};       ///< ∂Δp/∂(accel bias), or zero
    T dt = T{0};         ///< interval covered by the preintegration
};

/// A gyro-bias estimate from a window at rest but for vibration (#591):
/// the window's gyro mean, and how much its low-passed rate varied (smaller is
/// stiller, so a better estimate).
template <math::Scalar T>
struct RestBias {
    math::lie::detail::Vec<T, 3> gyro_bias{};
    T rate_variation = T{0};  ///< rad/s
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

    /// The gyro mean of a window that is at rest but for vibration, as a gyro-bias
    /// estimate (#591), with its rate variation; nullopt if the window is not. The gates are try_static's,
    /// except that the gyro is tested after the `gyro_lowpass_samples` low-pass.
    /// try_static keeps the per-sample test, so a start it accepts today is
    /// accepted the same way. The backend keeps the stillest such mean while it
    /// waits for a static window and hands it to `try_gravity_align` if it
    /// times out, so a vibrating start is not seeded with a zero bias. (The
    /// stillest, not the newest: a window that ends as motion starts can still
    /// pass, with a skewed mean, and the bias barely drifts over the wait.) Like
    /// try_static's gate, it cannot tell a constant slow turn from a bias: no
    /// test of how the rate varies can.
    [[nodiscard]] std::optional<RestBias<T>> vibrating_rest_bias(std::span<const Vec3> gyro,
                                                                 std::span<const Vec3> accel) const {
        const std::size_t n = gyro.size();
        if (n == 0 || accel.size() != n)
            return std::nullopt;
        const Vec3 accel_mean = mean(accel);
        const T variation = rate_variation(gyro);
        if (variation > cfg_.max_gyro_std || std_dev(accel, accel_mean) > cfg_.max_accel_std)
            return std::nullopt;
        const T g_meas = math::lie::detail::norm(accel_mean);
        if (!(g_meas > T{0}) || abs_(g_meas - cfg_.gravity_magnitude) / cfg_.gravity_magnitude > cfg_.gravity_tol)
            return std::nullopt;
        return RestBias<T>{mean(gyro), variation};
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
    /// the caller should fall back to identity. The gyro bias is, in order:
    /// `rest_bias` when given (a `vibrating_rest_bias` from earlier in the
    /// wait); the window's gyro mean when its rate does not vary, even under
    /// vibration (the low-passed test); else 0, since on a rotating start the
    /// mean is real angular rate, not bias.
    [[nodiscard]] Result try_gravity_align(std::span<const Vec3> gyro,
                                           std::span<const Vec3> accel,
                                           const std::optional<Vec3>& rest_bias = std::nullopt) const {
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
        if (rest_bias)
            r.gyro_bias = *rest_bias;
        else
            r.gyro_bias = (rate_variation(gyro) <= cfg_.max_gyro_std) ? gyro_mean : Vec3{};
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

        // Scale-observability gate: on a near-hover window the metric scale is
        // unobservable and the solve collapses it toward zero, so reject when
        // the resolved metric motion (scale · the largest keyframe displacement
        // in vision units) is below the floor. Leaves success = false so the
        // caller retries later or falls back. See #247.
        T vis_span = T{0};
        for (std::size_t i = 1; i < n; ++i) {
            const T d = math::lie::detail::norm(kfs[i].p_world_imu - kfs[0].p_world_imu);
            if (d > vis_span)
                vis_span = d;
        }
        r.resolved_motion = r.scale * vis_span;  // recorded even when we decline below
        if (r.resolved_motion < cfg_.min_dynamic_motion)
            return r;  // unobservable scale ⇒ decline (success stays false)

        // Scale-settling gate (#592): the same alignment on the leading part of
        // the window must agree on the scale. A prefix that cannot be solved
        // counts as unsettled.
        if (n >= kMinSettlingKeyframes) {
            Result prefix;
            prefix.gyro_bias = r.gyro_bias;
            const std::size_t m = (2 * n + 2) / 3;  // ⌈2n/3⌉
            if (estimate_gravity_and_velocities(kfs.first(m), prefix))
                r.prefix_scale = prefix.scale;
            const T change = r.prefix_scale > T{0} ? abs_(r.scale / r.prefix_scale - T{1}) : T{1};
            if (!(change <= cfg_.max_scale_change)) {
                r.scale_unsettled = true;
                return r;  // unsettled scale ⇒ decline (success stays false)
            }
        }

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
        r.success = true;  // accel_bias is the alignment's (body frame), or zero when not estimated
        return r;
    }

private:
    /// The settling gate needs a prefix of at least 3 keyframes (⌈2·4/3⌉ = 3);
    /// shorter windows skip it.
    static constexpr std::size_t kMinSettlingKeyframes = 4;

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

    /// How much the angular rate varies over the window, below the low-pass:
    /// the std (max over axes) of the means of consecutive blocks of
    /// `gyro_lowpass_samples` samples, the newest block ending the window. Any
    /// leftover oldest samples are not used. With fewer than two blocks, or a
    /// block of 1, it is the per-sample std.
    [[nodiscard]] T rate_variation(std::span<const Vec3> gyro) const {
        const std::size_t k = cfg_.gyro_lowpass_samples;
        const std::size_t blocks = k == 0 ? 0 : gyro.size() / k;
        if (k <= 1 || blocks < 2)
            return std_dev(gyro, mean(gyro));
        std::vector<Vec3> means;
        means.reserve(blocks);
        const std::size_t first = gyro.size() - blocks * k;
        for (std::size_t b = 0; b < blocks; ++b)
            means.push_back(mean(gyro.subspan(first + b * k, k)));
        const std::span<const Vec3> m{means};
        return std_dev(m, mean(m));
    }

    // Rotation R_world_imu such that R · up_body = world +z. up_body is a
    // unit vector. Handles the parallel / anti-parallel degeneracies.
    static SO3 align_to_world_up(const Vec3& up_body) {
        const Vec3 ez{{T{0}, T{0}, T{1}}};
        const T c = math::lie::detail::dot(up_body, ez);
        const Vec3 v = math::lie::detail::cross(up_body, ez);
        const T s = math::lie::detail::norm(v);
        // s = |up × ẑ| = sin of the angle to vertical, in [0, 1]. Below the
        // threshold the axis v/s is noise, so take the exact (anti)parallel
        // branch. This is a TOLERANCE on a unit-scale quantity, so it is floored
        // at a few ulps of T (#449): in float, s carries ~1e-7 of rounding, and a
        // fixed 1e-9 would accept that rounding as an axis.
        if (!(s > math::at_least_roundoff(T(1e-9)))) {
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
            // A REGULARIZER, a fixed 1e-9, not scaled with T (#449). It must
            // stay far below the bias modes' eigenvalues, and it is 0 in IEEE
            // half and Q16.16; see the velocity ridge below.
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
        // The accelerometer bias (body frame), when estimated (#501).
        const bool with_ba = cfg_.accel_bias_weight > T{0};
        const std::size_t b0 = s0 + 1;
        const std::size_t dim = b0 + (with_ba ? 3 : 0);

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

            // Position: vᵢ Δt + ½ g Δt² − s (pⱼ − pᵢ) = −Rᵢ Δp. The
            // ImuPreintegrator Δp is gravity-free (matches predict()), so
            // Rᵢ Δp = s·dp − vᵢ Δt − ½ g Δt² — note the +½g (not −½g): a sign
            // mismatch here inverts the recovered gravity (#247).
            diag(vi, dt);
            col(s0, dp_world * (-T{1}));
            // The preintegration ran with a zero accelerometer bias, so the true
            // Δp is Δp + (∂Δp/∂b_a)·b_a and −Rᵢ Δp_true = −Rᵢ Δp − Rᵢ (∂Δp/∂b_a) b_a.
            if (with_ba) {
                const Mat3 RJ = R_i * dp_dba(kfs[i]);
                for (std::size_t k = 0; k < 3; ++k)
                    col(b0 + k, Vec3{{RJ(0, k), RJ(1, k), RJ(2, k)}});
            }
            if (constrained) {
                col(g0, w1 * (T{0.5} * dt * dt));
                col(g0 + 1, w2 * (T{0.5} * dt * dt));
                flush(R_i * kfs[i].dp * (-T{1}) - base * (T{0.5} * dt * dt));
            } else {
                diag(g0, T{0.5} * dt * dt);
                flush(R_i * kfs[i].dp * (-T{1}));
            }

            // Velocity: vⱼ − vᵢ − g Δt = Rᵢ Δv (scale-free). Gravity-free Δv ⇒
            // Rᵢ Δv = vⱼ − vᵢ − g Δt (predict() convention).
            diag(vj, T{1});
            diag(vi, -T{1});
            // Likewise Rᵢ Δv_true = Rᵢ Δv + Rᵢ (∂Δv/∂b_a) b_a.
            if (with_ba) {
                const Mat3 RJ = R_i * dv_dba(kfs[i]);
                for (std::size_t k = 0; k < 3; ++k)
                    col(b0 + k, Vec3{{-RJ(0, k), -RJ(1, k), -RJ(2, k)}});
            }
            if (constrained) {
                col(g0, w1 * (-dt));
                col(g0 + 1, w2 * (-dt));
                flush(R_i * kfs[i].dv + base * dt);
            } else {
                diag(g0, -dt);
                flush(R_i * kfs[i].dv);
            }
        }

        // The velocities need motion to be observable; a small ridge
        // regularizes. Factor once and reuse the factor for the solve.
        //
        // NUMERICAL NOTE (#449). The ridge λ is a REGULARIZER, so it is a fixed
        // 1e-9, deliberately not scaled with ε_T. (Double, float and posit32
        // hold 1e-9 and posit16 rounds it up to 3.7e-9; IEEE half and Q16.16
        // round it to 0 and have no ridge at all.) It makes an
        // unexcited direction (an exactly zero or near-null mode) factorable. It
        // also shrinks the solution along every eigenvector of H by λ/(σᵢ + λ).
        // On the S1 known-answer window:
        //  - the refined pass's smallest eigenvalue is 5.5e-6 (cond 7e5), and
        //    its eigenvector includes the scale;
        //  - λ = 1e-9 shrinks it by 0.02 %;
        //  - λ = 16·ε_float = 1.9e-6 shrank it by 26 %, a 17 % scale error;
        //  - λ = 16 ulps of each diagonal entry gave 25–29 %.
        // So λ must sit far below the informative eigenvalues, which are set by
        // the motion, not by T. Do not floor it with at_least_roundoff.
        //
        // Precision: rounding moves the solution by about cond(H)·ε_T. That is
        // 0.55 % of scale for a float32 solve of this H (cond 7e5). H = JᵀJ (normal
        // equations) squares the conditioning of J; a narrow format (half,
        // bfloat16, posit16: cond·ε ≥ 1) cannot resolve the scale this way at
        // all. A port to such a format needs a QR/square-root solve of the
        // stacked constraints, or a rescaled problem.
        // See docs-site …/debugging/numerical-thresholds.mdx.
        for (std::size_t i = 0; i < dim; ++i)
            H(i, i) += T(1e-9);
        if (with_ba)  // the zero-bias prior rows, w·b_a = 0
            for (std::size_t k = 0; k < 3; ++k)
                H(b0 + k, b0 + k) += cfg_.accel_bias_weight * cfg_.accel_bias_weight;
        DynMat<T> L;
        if (!cholesky(H, L))
            return false;
        const DynMat<T> x = cholesky_solve(L, b);

        r.velocities_world.resize(n);
        for (std::size_t i = 0; i < n; ++i)
            r.velocities_world[i] = Vec3{{x(3 * i, 0), x(3 * i + 1, 0), x(3 * i + 2, 0)}};
        r.scale = x(s0, 0);
        r.accel_bias = with_ba ? Vec3{{x(b0, 0), x(b0 + 1, 0), x(b0 + 2, 0)}} : Vec3{};
        g_out = constrained ? base + w1 * x(g0, 0) + w2 * x(g0 + 1, 0) : Vec3{{x(g0, 0), x(g0 + 1, 0), x(g0 + 2, 0)}};
        return true;
    }

    // The keyframe's accelerometer-bias Jacobians, or the constant-attitude
    // ones (−½Δt²·I, −Δt·I) when it carries none.
    static bool has_dba(const DynInitKeyframe<T>& k) {
        for (std::size_t i = 0; i < 9; ++i)
            if (k.dv_dba.e[i] != T{0} || k.dp_dba.e[i] != T{0})
                return true;
        return false;
    }
    static Mat3 dp_dba(const DynInitKeyframe<T>& k) {
        return has_dba(k) ? k.dp_dba : Mat3::identity() * (-T{0.5} * k.dt * k.dt);
    }
    static Mat3 dv_dba(const DynInitKeyframe<T>& k) {
        return has_dba(k) ? k.dv_dba : Mat3::identity() * (-k.dt);
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
