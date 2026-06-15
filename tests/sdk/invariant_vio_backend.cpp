// R-IEKF VIO backend adapter (issue #365) — drive MsckfInvariantBackend through the
// validated body-frame track lifecycle (InvariantVioBackend). This locks the
// INTEGRATION end to end and reports the attitude split for visibility. It does NOT
// yet assert convergence: the integrated run localized a real bug — the invariant
// measurement update is HARMFUL in the assembled continuous filter (pure propagation
// drifts ~15 m here; with the update ~160 m, and the update being worse than
// no-update rules out track management / geometry / propagation). Tracked in #366.

#include <branes/sdk/eval/nav_consistency.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/invariant_vio_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <random>
#include <span>
#include <stdexcept>
#include <vector>

namespace {
namespace ev = branes::sdk::eval;
namespace ms = branes::sdk::msckf;
namespace lie = branes::math::lie;
using T = double;
using Vec3 = lie::detail::Vec<T, 3>;
using Vec2 = lie::detail::Vec<T, 2>;
using SO3 = lie::SO3<T>;
using SE3 = lie::SE3<T>;
using SE23 = lie::SE23<T>;

// Identity-extrinsic normalized observation of L from an IMU pose (FOV-gated).
bool observe(const SO3& R, const Vec3& p, const Vec3& L, Vec2& xy) {
    const Vec3 y = R.inverse() * Vec3{{L[0] - p[0], L[1] - p[1], L[2] - p[2]}};
    if (!(y[2] > T{1} / T{4}))
        return false;
    const T u = y[0] / y[2], v = y[1] / y[2];
    if (u * u + v * v > T{0.49})
        return false;
    xy = Vec2{{u, v}};
    return true;
}
}  // namespace

TEST_CASE("invariant VIO backend: end-to-end integration (continuous-update divergence KNOWN, #366)",
          "[sdk][riekf][backend][s365]") {
    ev::SyntheticConfig<T> scfg;
    scfg.seed = 0x5EEDu;
    const auto world = ev::generate_world<T>(scfg);
    REQUIRE(world.frames.size() > 50);

    std::mt19937 rng(0xA11CE);
    std::normal_distribution<T> nd(0, 1);
    // Landmarks above the trajectory (the pure-yaw motion ⇒ identity-extrinsic
    // camera looks straight up); good parallax from the lateral loop.
    std::uniform_real_distribution<T> ux(-2, 3), uy(-2, 2), uz(3, 8);
    std::vector<Vec3> landmarks(240);
    for (auto& L : landmarks)
        L = Vec3{{ux(rng), uy(rng), uz(rng)}};

    const T gyro_density = T{1} / T{2000}, accel_density = T{1} / T{200}, pix = T{1} / T{200};
    const T dt_imu = T{1} / scfg.imu_rate_hz;
    const T g_std = gyro_density / std::sqrt(dt_imu), a_std = accel_density / std::sqrt(dt_imu);

    ms::InvariantVioBackend<T>::Config cfg;
    cfg.max_clones = 11;
    cfg.backend.initial_sigma = T{1} / T{20};
    cfg.backend.normalized_sigma = pix;
    cfg.backend.noise = ms::ImuNoise<T>{gyro_density, accel_density, T{1} / T{50000}, T{1} / T{3000}};
    ms::InvariantVioBackend<T> be(cfg);

    const auto& g0 = world.gt.front();
    be.set_nav(SE23(g0.R, g0.v, g0.p), Vec3{}, Vec3{}, g0.t);
    const Vec3 true_bg = world.gyro_bias, true_ba = world.accel_bias;

    SE3 anchor;
    bool anchored = false;
    ev::ConsistencyAccumulator nees_full, nees_yaw, nees_rp;
    double sum_pos2 = 0, ndiag = 0;
    std::size_t imu_idx = 0;
    bool psd_ok = true;

    for (std::size_t f = 0; f < world.frames.size(); ++f) {
        const double t = world.frames[f].t;
        for (; imu_idx < world.imu.size() && world.imu[imu_idx].timestamp_s <= t; ++imu_idx) {
            const auto& m = world.imu[imu_idx];
            be.process_imu(Vec3{{m.angular_velocity[0] + g_std * nd(rng),
                                 m.angular_velocity[1] + g_std * nd(rng),
                                 m.angular_velocity[2] + g_std * nd(rng)}},
                           Vec3{{m.linear_acceleration[0] + a_std * nd(rng),
                                 m.linear_acceleration[1] + a_std * nd(rng),
                                 m.linear_acceleration[2] + a_std * nd(rng)}},
                           m.timestamp_s);
        }
        // Observations from the TRUE pose (real measurements come from the world).
        std::vector<ms::NormalizedObs<T>> obs;
        for (std::size_t L = 0; L < landmarks.size(); ++L) {
            Vec2 xy;
            if (observe(world.gt[f].R, world.gt[f].p, landmarks[L], xy))
                obs.push_back({static_cast<std::uint64_t>(L), Vec2{{xy[0] + pix * nd(rng), xy[1] + pix * nd(rng)}}});
        }
        be.process_camera(t, std::span<const ms::NormalizedObs<T>>{obs});

        if (!ms::is_positive_semidefinite(be.covariance()))
            psd_ok = false;

        // Sample error vs gauge-aligned truth (steady state).
        const auto& gt = world.gt[f];
        const SE3 est_pose(be.nav().X.rotation(), be.nav().X.position());
        if (!anchored) {
            anchor = ev::gauge_align<T>(est_pose, SE3(gt.R, gt.p));
            anchored = true;
        }
        if (t < g0.t + 5.0)
            continue;
        const ev::NavSample<T> truth = ev::align_truth<T>(anchor, ev::NavSample<T>{gt.R, gt.p, gt.v, true_bg, true_ba});
        const SE23 X_est = be.nav().X;
        const SE23 X_truth(truth.R, truth.v, truth.p);
        const auto xi = (X_est * X_truth.inverse()).log();  // [δθ δv δp]
        const Vec3 dp{
            {X_est.position()[0] - truth.p[0], X_est.position()[1] - truth.p[1], X_est.position()[2] - truth.p[2]}};
        sum_pos2 += dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2];
        ndiag += 1;

        std::array<T, ev::kNavErrorDim> e{};
        for (std::size_t k = 0; k < 9; ++k)
            e[k] = xi[k];
        const Vec3 dbg = be.nav().bg - truth.bg, dba = be.nav().ba - truth.ba;
        for (std::size_t k = 0; k < 3; ++k) {
            e[9 + k] = dbg[k];
            e[12 + k] = dba[k];
        }
        const ms::DynMat<T> P = ev::core_covariance<T>(be.covariance());
        try {
            nees_full.add(ev::nees<T>(std::span<const T>{e}, P), static_cast<int>(ev::kNavErrorDim));
            ms::DynMat<T> Py(1, 1);
            Py(0, 0) = P(2, 2);
            nees_yaw.add(ev::nees<T>(std::span<const T>{std::array<T, 1>{{e[2]}}}, Py), 1);
            ms::DynMat<T> Prp(2, 2);
            for (std::size_t i = 0; i < 2; ++i)
                for (std::size_t j = 0; j < 2; ++j)
                    Prp(i, j) = P(i, j);
            nees_rp.add(ev::nees<T>(std::span<const T>{std::array<T, 2>{{e[0], e[1]}}}, Prp), 2);
        } catch (const std::domain_error&) {}
    }

    REQUIRE(ndiag > 0);  // post-warmup samples accumulated (else the RMS is undefined)
    const double rms_pos = std::sqrt(sum_pos2 / ndiag);
    WARN("invariant VIO backend: RMS pos err=" << rms_pos << " m  | full NEES=" << nees_full.report().normalized
                                               << "  yaw=" << nees_yaw.report().normalized
                                               << "  roll/pitch=" << nees_rp.report().normalized
                                               << "  (KNOWN divergence — the invariant update is harmful in the"
                                                  " continuous filter; tracked in #366)");

    // This test locks the INTEGRATION (the adapter mirrors the validated body-frame
    // track lifecycle and drives the invariant backend end to end). It does NOT yet
    // assert convergence: the invariant measurement update is harmful in the
    // assembled continuous filter (pure propagation drifts ~15 m here; with the
    // update ~160 m — localized in #365, tracked in #366). When #366 is fixed,
    // turn the WARN above into `REQUIRE(rms_pos < <bound>)`.
    REQUIRE(psd_ok);                  // covariance never goes non-PSD
    REQUIRE(std::isfinite(rms_pos));  // runs to completion without NaN/Inf
    REQUIRE(nees_full.samples() > 50);
    REQUIRE(be.num_clones() <= 11);  // the adapter-owned window stays bounded
}
