// R-IEKF VIO backend adapter (issue #365) — drive MsckfInvariantBackend through the
// validated body-frame track lifecycle (InvariantVioBackend). This locks the
// INTEGRATION end to end over a sweep of noise seeds, and checks the filter's
// covariance is consistent with its error (NEES), not only that it tracks.
//
// #537: one seed passed and others diverged (2.4–120 m) because the synthetic
// world stepped the velocity by ≈1 m/s at the end of warm-up with no
// accelerometer impulse, and the adapter integrated the warm-up IMU from a state
// seeded after it. With both fixed, every seed tracks to centimetres at NEES ≈ 1.

#include <branes/sdk/eval/consistency.hpp>
#include <branes/sdk/eval/nav_consistency.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/invariant_vio_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <span>
#include <stdexcept>
#include <vector>

namespace {
namespace ms = branes::sdk::msckf;
namespace ev = branes::sdk::eval;
namespace lie = branes::math::lie;

// Identity-extrinsic normalized observation of L from an IMU pose (FOV-gated).
template <typename T>
bool observe(const lie::SO3<T>& R,
             const lie::detail::Vec<T, 3>& p,
             const lie::detail::Vec<T, 3>& L,
             lie::detail::Vec<T, 2>& xy) {
    const lie::detail::Vec<T, 3> y = R.inverse() * lie::detail::Vec<T, 3>{{L[0] - p[0], L[1] - p[1], L[2] - p[2]}};
    if (!(y[2] > T{1} / T{4}))
        return false;
    const T u = y[0] / y[2], v = y[1] / y[2];
    if (u * u + v * v > T{0.49})
        return false;
    xy = lie::detail::Vec<T, 2>{{u, v}};
    return true;
}

/// What one end-to-end run measured; the structural checks and the accuracy
/// bounds are asserted in separate test cases on the same runs.
struct EndToEnd {
    double rms_pos = 0;
    ev::ConsistencyAccumulator nees_full, nees_pos, nees_yaw, nees_rp;
    bool psd_ok = true;
    std::size_t nees_skip = 0;
    std::size_t num_clones = 0;
};

EndToEnd run_end_to_end(std::uint32_t seed) {
    using T = double;
    using SO3 = lie::SO3<T>;
    using SE3 = lie::SE3<T>;
    using SE23 = lie::SE23<T>;
    using Vec3 = lie::detail::Vec<T, 3>;
    using Vec2 = lie::detail::Vec<T, 2>;

    ev::SyntheticConfig<T> scfg;
    scfg.seed = 0x5EEDu;
    const auto world = ev::generate_world<T>(scfg);
    REQUIRE(world.frames.size() > 50);

    std::mt19937 rng(seed);
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
    cfg.backend.enable_gating = true;
    cfg.backend.chi2_per_dof = T{16};  // 4-sigma gate
    ms::InvariantVioBackend<T> be(cfg);

    const auto& g0 = world.gt.front();
    be.set_nav(SE23(g0.R, g0.v, g0.p), Vec3{}, Vec3{}, g0.t);
    const Vec3 true_bg = world.gyro_bias, true_ba = world.accel_bias;

    SE3 anchor;
    bool anchored = false;
    ev::ConsistencyAccumulator nees_full, nees_pos, nees_yaw, nees_rp;
    double sum_pos2 = 0, ndiag = 0;
    std::size_t imu_idx = 0;
    bool psd_ok = true;
    std::size_t nees_skip = 0;

    for (std::size_t f = 0; f < world.frames.size(); ++f) {
        const double t = world.frames[f].t;
        for (; imu_idx < world.imu.size() && world.imu[imu_idx].timestamp_s <= t; ++imu_idx) {
            const auto& m = world.imu[imu_idx];
            // Draw gyro then accel in named statements: two arguments of one call
            // are evaluated in an unspecified order (GCC right to left, Clang left
            // to right), which gave each compiler a different noise sequence (#437).
            const Vec3 gyro{{m.angular_velocity[0] + g_std * nd(rng),
                             m.angular_velocity[1] + g_std * nd(rng),
                             m.angular_velocity[2] + g_std * nd(rng)}};
            const Vec3 accel{{m.linear_acceleration[0] + a_std * nd(rng),
                              m.linear_acceleration[1] + a_std * nd(rng),
                              m.linear_acceleration[2] + a_std * nd(rng)}};
            be.process_imu(gyro, accel, m.timestamp_s);
        }

        std::vector<ms::NormalizedObs<T>> obs;
        for (std::size_t L = 0; L < landmarks.size(); ++L) {
            Vec2 xy;
            if (observe<T>(world.gt[f].R, world.gt[f].p, landmarks[L], xy))
                obs.push_back({L, 0, Vec2{{xy[0] + pix * nd(rng), xy[1] + pix * nd(rng)}}});
        }
        be.process_camera(t, std::span<const ms::NormalizedObs<T>>{obs});

        if (!ms::is_positive_semidefinite(be.covariance()))
            psd_ok = false;

        // Sample error vs gauge-aligned truth (steady state).
        const auto& gt = world.gt[f];
        if (!anchored) {
            anchor = SE3(gt.R, gt.p) * SE3(be.nav().X.rotation(), be.nav().X.position()).inverse();
            anchored = true;
        }
        if (t < g0.t + 5.0)
            continue;
        const ev::NavSample<T> truth = ev::align_truth<T>(anchor, ev::NavSample<T>{gt.R, gt.p, gt.v, true_bg, true_ba});
        const SE23 X_est = be.nav().X;

        // The right-invariant error the filter estimates: truth = Exp(ξ)·estimate,
        // matching its retraction R ← Exp(δθ)·R, v ← Exp(δθ)·v + δν, p ← Exp(δθ)·p + δρ.
        const SO3 R_err = truth.R * X_est.rotation().inverse();
        const Vec3 phi = R_err.log();
        const Vec3 nu = truth.v - R_err * X_est.velocity();
        const Vec3 rho = truth.p - R_err * X_est.position();

        sum_pos2 += rho[0] * rho[0] + rho[1] * rho[1] + rho[2] * rho[2];
        ndiag += 1;

        std::array<T, ev::kNavErrorDim> e{};
        // Match InvariantNavState ordering: [theta, vel, pos, bg, ba]
        for (std::size_t k = 0; k < 3; ++k) {
            e[k] = phi[k];      // attitude (0-2)
            e[3 + k] = nu[k];   // velocity (3-5)
            e[6 + k] = rho[k];  // position (6-8)
        }
        const Vec3 dbg = truth.bg - be.nav().bg, dba = truth.ba - be.nav().ba;
        for (std::size_t k = 0; k < 3; ++k) {
            e[9 + k] = dbg[k];
            e[12 + k] = dba[k];
        }

        const ms::DynMat<T> P_joint = be.covariance();
        ms::DynMat<T> P(ev::kNavErrorDim, ev::kNavErrorDim);
        for (std::size_t i = 0; i < ev::kNavErrorDim; ++i)
            for (std::size_t j = 0; j < ev::kNavErrorDim; ++j)
                P(i, j) = P_joint(i, j);

        try {
            const T full_val = ev::nees<T>(std::span<const T>{e}, P);
            ms::DynMat<T> Py(1, 1);
            Py(0, 0) = P(2, 2);
            const T yaw_val = ev::nees<T>(std::span<const T>{std::array<T, 1>{{e[2]}}}, Py);
            ms::DynMat<T> Prp(2, 2);
            for (std::size_t i = 0; i < 2; ++i)
                for (std::size_t j = 0; j < 2; ++j)
                    Prp(i, j) = P(i, j);
            const T rp_val = ev::nees<T>(std::span<const T>{std::array<T, 2>{{e[0], e[1]}}}, Prp);
            ms::DynMat<T> Pp(3, 3);
            for (std::size_t i = 0; i < 3; ++i)
                for (std::size_t j = 0; j < 3; ++j)
                    Pp(i, j) = P(6 + i, 6 + j);
            const T pos_val = ev::nees<T>(std::span<const T>{std::array<T, 3>{{e[6], e[7], e[8]}}}, Pp);

            nees_full.add(full_val, static_cast<int>(ev::kNavErrorDim));
            nees_pos.add(pos_val, 3);
            nees_yaw.add(yaw_val, 1);
            nees_rp.add(rp_val, 2);
        } catch (const std::domain_error&) {
            ++nees_skip;
        }
    }

    REQUIRE(ndiag > 0);
    EndToEnd out;
    out.rms_pos = std::sqrt(sum_pos2 / ndiag);
    out.nees_full = nees_full;
    out.nees_pos = nees_pos;
    out.nees_yaw = nees_yaw;
    out.nees_rp = nees_rp;
    out.psd_ok = psd_ok;
    out.nees_skip = nees_skip;
    out.num_clones = be.num_clones();
    return out;
}

// The CI seed (0xA11CE), the two worst of those that diverged before #537 (13 at
// 120 m, 8 at 73 m), and one that tracked. One run takes ~25 s in a debug build,
// so the full 21-seed sweep is the hidden [.sweep] case below.
constexpr std::array<std::uint32_t, 4> kSeeds{{0xA11CE, 13, 8, 3}};

std::vector<EndToEnd> run_seeds(std::span<const std::uint32_t> seeds) {
    std::vector<EndToEnd> r;
    for (const std::uint32_t seed : seeds)
        r.push_back(run_end_to_end(seed));
    return r;
}

/// Test cases in one process share the runs.
const std::vector<EndToEnd>& end_to_end() {
    static const std::vector<EndToEnd> runs = run_seeds(kSeeds);
    return runs;
}

void check_structure(const EndToEnd& e) {
    REQUIRE(e.psd_ok);
    REQUIRE(std::isfinite(e.rms_pos));
    REQUIRE(e.nees_full.samples() > 50);
    REQUIRE(e.nees_yaw.samples() > 50);
    REQUIRE(e.nees_rp.samples() > 50);
    REQUIRE(e.nees_skip == 0);
    REQUIRE(e.num_clones <= 11);
}

// Before #537 the seeds tracked to 2.4–120 m at a position NEES of ~10³. Now
// each tracks to centimetres (worst 0.08 m over 21 seeds) and its NEES is near
// its dimension (normalized 0.6–1.8 full, <= 3 roll/pitch). Yaw and global
// position are the unobservable gauge: their σ stays near the prior while the
// error stays at centimetres, so their NEES sits well below 1 (pessimistic,
// never the over-confident ~10³ of before).
void check_accuracy(std::span<const std::uint32_t> seeds, const std::vector<EndToEnd>& runs) {
    std::vector<double> rms;
    for (std::size_t k = 0; k < runs.size(); ++k) {
        const EndToEnd& e = runs[k];
        INFO("seed " << seeds[k]);
        rms.push_back(e.rms_pos);
        CHECK(e.rms_pos < 0.15);
        CHECK(e.nees_full.report().normalized > 0.25);
        CHECK(e.nees_full.report().normalized < 3.0);
        // One-sided by design: global position is gauge, so its σ is set by the
        // initial prior (it stays near it) rather than earned from data, and a
        // lower bound would test the prior, not the filter (measured 0.04–0.15).
        // The full-state band above still rejects a uniformly pessimistic P.
        CHECK(e.nees_pos.report().normalized < 4.0);
        CHECK(e.nees_rp.report().normalized < 6.0);
        CHECK(e.nees_yaw.report().normalized < 3.0);
    }
    std::sort(rms.begin(), rms.end());
    CHECK(rms[rms.size() / 2] < 0.06);  // median
}
}  // namespace

TEST_CASE("invariant VIO backend: end-to-end integration over a seed sweep", "[sdk][riekf][backend][s365]") {
    const auto& runs = end_to_end();
    for (std::size_t k = 0; k < runs.size(); ++k) {
        const EndToEnd& e = runs[k];
        INFO("seed " << kSeeds[k]);
        WARN("invariant VIO backend seed "
             << kSeeds[k] << ": RMS pos err=" << e.rms_pos << " m  | NEES full=" << e.nees_full.report().normalized
             << "  pos=" << e.nees_pos.report().normalized << "  yaw=" << e.nees_yaw.report().normalized
             << "  roll/pitch=" << e.nees_rp.report().normalized);
        check_structure(e);
    }
}

TEST_CASE("invariant VIO backend: tracks and stays consistent on every seed", "[sdk][riekf][backend][s365]") {
    check_accuracy(kSeeds, end_to_end());
}

// The full sweep the #537 bounds were set from: ~9 min in a debug build, ~1 min
// in release. Run with `invariant_vio_backend "[.sweep]"`.
TEST_CASE("invariant VIO backend: 21-seed sweep", "[sdk][riekf][backend][.sweep]") {
    std::array<std::uint32_t, 21> seeds{};
    seeds[0] = 0xA11CE;
    for (std::uint32_t k = 1; k < seeds.size(); ++k)
        seeds[k] = k;
    const auto runs = run_seeds(seeds);
    for (std::size_t k = 0; k < runs.size(); ++k) {
        INFO("seed " << seeds[k]);
        check_structure(runs[k]);
    }
    check_accuracy(seeds, runs);
}
