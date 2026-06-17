// SPDX-License-Identifier: MIT
//
// invariant_nees_verdict — a system-level NEES DIAGNOSTIC for the R-IEKF backend
// (issues #347/#348/#212). NOT a passing verdict: read the FINDING below.
//
// The R-IEKF observability fix is proven at the UNIT level (the propagation Φ and
// the camera-update H annihilate the 4-DoF gauge by construction — see the
// invariant_propagator / invariant_update / msckf_invariant_backend tests). This
// tool asks the next question: does that translate to a CONSISTENT filter end to
// end? It drives MsckfInvariantBackend over a Monte-Carlo of generate_world()
// trajectories (EuRoC camera intrinsics; real EuRoC sequences are env-gated and
// absent in CI), synthesising identity-extrinsic observations from ground truth and
// injecting Q/R-consistent IMU+pixel noise, then accumulates gauge-anchored NEES.
//
// FINDING (as measured): the assembled filter is broadly OVER-confident, and the
// over-confidence is NOT the gravity-yaw mechanism — splitting attitude shows
// roll/pitch (gravity-OBSERVABLE, leak-impossible) inflated far more than yaw. That
// points at a DRIVER over-fusion / marginal-stability problem in this hand-rolled
// loop (clone correlation, relinearisation, triangulation at drifted poses), not a
// failure of the unit-proven observability fix. So this is a harness that LOCALISES
// the remaining gap, not a confirmation. A trustworthy verdict likely needs the
// validated VioEstimator front-end / track manager rather than this loop.
//
// NEES is computed in the filter's OWN SE2(3) right-invariant convention (error
// log(X_est · X_truth^-1) against the invariant covariance) — NEES is
// parameterisation-invariant, so no change-of-coordinates is needed. Truth is
// gauge-aligned (4-DoF: global translation + yaw about gravity) once at the first
// post-init frame, as the real-data harness (tests/sdk/vio_euroc.cpp) does.
//
// Diagnostic env knobs: VERDICT_TRUE_BIAS, VERDICT_NO_UPDATE, VERDICT_NO_GATE,
// VERDICT_PER_TRIAL (per-trial final position error to stderr).
//
//   ./invariant_nees_verdict [--trials N] [--window K] [--warmup S]

#include <branes/sdk/eval/consistency.hpp>
#include <branes/sdk/eval/nav_consistency.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/msckf_invariant_backend.hpp>

#include <array>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <map>
#include <random>
#include <span>
#include <string>
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
using Backend = ms::MsckfInvariantBackend<T>;

int arg_int(int argc, char** argv, const std::string& key, int def) {
    for (int i = 1; i + 1 < argc; ++i)
        if (key == argv[i])
            return std::atoi(argv[i + 1]);
    return def;
}

// Identity-extrinsic normalized observation of world point L from an IMU pose,
// with FOV + cheirality gating. The invariant backend models clone == camera with
// identity extrinsic, so we synthesise observations straight from the IMU pose and
// the known landmark cloud (bypassing the world's 90°-rotated camera mount).
bool observe(const SO3& R, const Vec3& p, const Vec3& L, Vec2& xy) {
    const Vec3 y = R.inverse() * Vec3{{L[0] - p[0], L[1] - p[1], L[2] - p[2]}};
    if (!(y[2] > T{1} / T{4}))  // ≥ 0.25 m in front
        return false;
    const T u = y[0] / y[2], v = y[1] / y[2];
    if (u * u + v * v > T{0.49})  // ~35° half-FOV
        return false;
    xy = Vec2{{u, v}};
    return true;
}
}  // namespace

// The window is managed manually (MSCKF update-on-leave), so the backend's own cap
// is set effectively unbounded; this constant guards the manual `--window` value.
constexpr std::size_t kMaxClones = 100000;

int main(int argc, char** argv) {
    const int trials = arg_int(argc, argv, "--trials", 30);
    const int window_arg = arg_int(argc, argv, "--window", 11);
    if (window_arg < 2 || static_cast<std::size_t>(window_arg) >= kMaxClones) {
        std::cerr << "--window must be in [2, " << (kMaxClones - 1) << "] (got " << window_arg << ")\n";
        return 2;
    }
    const std::size_t window = static_cast<std::size_t>(window_arg);
    const double warmup_s = static_cast<double>(arg_int(argc, argv, "--warmup", 5));

    // IMU + camera noise. The injected white noise is consistent with the filter's
    // process/measurement model (Q, R), so a CONSISTENT filter yields NEES ≈ 1; an
    // over-confident one yields NEES ≫ 1 even though the noise matches.
    const T gyro_density = T{1} / T{2000};  // 5e-4 rad/s/√Hz
    const T accel_density = T{1} / T{200};  // 5e-3 m/s²/√Hz
    const T pix_sigma = T{1} / T{200};      // normalized image σ (≈ 1 px @ f≈460)

    Backend::Config cfg;
    cfg.max_clones = kMaxClones;  // we manage the window manually (MSCKF update-on-leave)
    cfg.initial_sigma = T{1} / T{20};
    cfg.normalized_sigma = pix_sigma;
    cfg.noise = ms::ImuNoise<T>{gyro_density, accel_density, T{1} / T{50000}, T{1} / T{3000}};
    if (std::getenv("VERDICT_NO_GATE"))
        cfg.enable_gating = false;

    ev::ConsistencyAccumulator nees_full;
    std::array<ev::ConsistencyAccumulator, ev::kNumNavBlocks> nees_block;
    ev::ConsistencyAccumulator nees_yaw, nees_rollpitch;  // split the attitude tell
    // diagnostics
    double sum_pos_err2 = 0, sum_att_err2 = 0, sum_att_sig2 = 0, n_diag = 0;
    double sum_feat = 0, n_upd = 0;

    for (int trial = 0; trial < trials; ++trial) {
        ev::SyntheticConfig<T> scfg;
        scfg.seed = 0x5EEDu + static_cast<std::uint64_t>(trial) * 0x9E37u;
        const auto world = ev::generate_world<T>(scfg);
        if (world.frames.size() < 10)
            continue;
        std::mt19937 rng(0xA11CE + static_cast<unsigned>(trial));
        std::normal_distribution<T> nd(T{0}, T{1});
        // The synthetic trajectory is pure-yaw (rotation about gravity), so the
        // identity-extrinsic camera (clone == IMU, optical axis = body +z) looks
        // straight UP. generate_world's landmarks sit in the +x region for its
        // forward-looking mount, off-axis for us — so build our own cloud ABOVE the
        // trajectory, seen with good parallax from the lateral loop motion.
        std::uniform_real_distribution<T> ux(T{-2}, T{3}), uy(T{-2}, T{2}), uz(T{3}, T{8});
        std::vector<Vec3> landmarks(240);
        for (auto& L : landmarks)
            L = Vec3{{ux(rng), uy(rng), uz(rng)}};
        const T dt_imu = T{1} / scfg.imu_rate_hz;
        const T g_std = gyro_density / std::sqrt(dt_imu);
        const T a_std = accel_density / std::sqrt(dt_imu);

        Backend b(cfg);
        // Initialise at the first ground-truth nav state (biases at zero — the
        // filter estimates them); prior covariance is cfg.initial_sigma.
        const auto& g0 = world.gt.front();
        const Vec3 true_bg = world.gyro_bias, true_ba = world.accel_bias;
        const bool true_bias_init = std::getenv("VERDICT_TRUE_BIAS") != nullptr;
        const bool no_update = std::getenv("VERDICT_NO_UPDATE") != nullptr;
        b.set_nav(SE23(g0.R, g0.v, g0.p), true_bias_init ? true_bg : Vec3{}, true_bias_init ? true_ba : Vec3{}, g0.t);

        // Gauge anchor (fixed once at the first frame) and per-clone observation log.
        SE3 anchor;
        bool anchored = false;
        std::deque<std::map<std::size_t, Vec2>> clone_obs;  // parallel to b.clones()
        std::size_t imu_idx = 0;
        T trial_pos_err = T{0};

        for (std::size_t f = 0; f < world.frames.size(); ++f) {
            const double t = world.frames[f].t;
            // Propagate (noisy IMU) up to this frame time.
            for (; imu_idx < world.imu.size() && world.imu[imu_idx].timestamp_s <= t; ++imu_idx) {
                const auto& m = world.imu[imu_idx];
                const Vec3 gyro{{m.angular_velocity[0] + g_std * nd(rng),
                                 m.angular_velocity[1] + g_std * nd(rng),
                                 m.angular_velocity[2] + g_std * nd(rng)}};
                const Vec3 acc{{m.linear_acceleration[0] + a_std * nd(rng),
                                m.linear_acceleration[1] + a_std * nd(rng),
                                m.linear_acceleration[2] + a_std * nd(rng)}};
                b.propagate(gyro, acc, dt_imu);
            }

            // Clone the current pose; record which landmarks it sees (with noise).
            // The observations are generated from the TRUE pose (real measurements
            // come from the real world, not the filter's drifted estimate) — using
            // the estimate here would make the measurements track the drift and the
            // update could never correct it.
            b.augment_clone();
            const SO3 Rc = world.gt[f].R;
            const Vec3 pc = world.gt[f].p;
            std::map<std::size_t, Vec2> seen;
            for (std::size_t L = 0; L < landmarks.size(); ++L) {
                Vec2 xy;
                if (observe(Rc, pc, landmarks[L], xy))
                    seen[L] = Vec2{{xy[0] + pix_sigma * nd(rng), xy[1] + pix_sigma * nd(rng)}};
            }
            clone_obs.push_back(std::move(seen));

            // MSCKF: once the window is full, update with the tracks the OLDEST
            // clone participates in (≥2 views), then marginalise it.
            while (clone_obs.size() > window) {
                const auto& oldest = clone_obs.front();
                for (const auto& [L, xy0] : oldest) {
                    ms::InvariantTrack<T> tr;
                    for (std::size_t c = 0; c < clone_obs.size(); ++c) {
                        const auto it = clone_obs[c].find(L);
                        if (it != clone_obs[c].end())
                            tr.push_back({c, 0, it->second});
                    }
                    if (tr.size() >= 2 && !no_update) {
                        b.update(tr);
                        sum_feat += static_cast<double>(tr.size());
                        n_upd += 1;
                    }
                }
                // Consume the oldest clone's features so they are not re-fused, then
                // drop it (after the update has extracted its information).
                const auto consumed = clone_obs.front();
                clone_obs.pop_front();
                b.marginalize_clone(0);
                for (const auto& [L, _] : consumed)
                    for (auto& cm : clone_obs)
                        cm.erase(L);
            }

            // Sample NEES against gauge-aligned truth (steady state only).
            const auto& gt = world.gt[f];
            const SE3 est_pose(b.nav().X.rotation(), b.nav().X.position());
            if (!anchored) {
                anchor = ev::gauge_align<T>(est_pose, SE3(gt.R, gt.p));
                anchored = true;
            }
            if (t < g0.t + warmup_s)
                continue;
            const ev::NavSample<T> truth =
                ev::align_truth<T>(anchor, ev::NavSample<T>{gt.R, gt.p, gt.v, true_bg, true_ba});
            const SE23 X_est = b.nav().X;
            const SE23 X_truth(truth.R, truth.v, truth.p);
            const auto xi = (X_est * X_truth.inverse()).log();  // [δθ δv δp] (invariant)
            const Vec3 dbg = b.nav().bg - truth.bg, dba = b.nav().ba - truth.ba;
            std::array<T, ev::kNavErrorDim> e{};  // backend order [δθ δv δp δbg δba]
            for (std::size_t k = 0; k < 9; ++k)
                e[k] = xi[k];
            for (std::size_t k = 0; k < 3; ++k) {
                e[9 + k] = dbg[k];
                e[12 + k] = dba[k];
            }
            ms::DynMat<T> P = ev::core_covariance<T>(b.covariance());
            T trial_last_pos_err = T{0};
            {  // diagnostics: tracking error vs reported sigma
                const Vec3 dpw{{X_est.position()[0] - truth.p[0],
                                X_est.position()[1] - truth.p[1],
                                X_est.position()[2] - truth.p[2]}};
                trial_last_pos_err = std::sqrt(dpw[0] * dpw[0] + dpw[1] * dpw[1] + dpw[2] * dpw[2]);
                trial_pos_err = trial_last_pos_err;
                sum_pos_err2 += dpw[0] * dpw[0] + dpw[1] * dpw[1] + dpw[2] * dpw[2];
                sum_att_err2 += xi[0] * xi[0] + xi[1] * xi[1] + xi[2] * xi[2];
                sum_att_sig2 += P(0, 0) + P(1, 1) + P(2, 2);
                n_diag += 1;
            }
            try {
                nees_full.add(ev::nees<T>(std::span<const T>{e}, P), static_cast<int>(ev::kNavErrorDim));
                // Split the attitude: yaw = world-z (gravity axis, the unobservable
                // gauge) vs roll/pitch = world-xy (gravity-OBSERVABLE). If only yaw
                // blows up -> residual observability leak; if roll/pitch blows up too
                // -> a broad driver over-fusion bug, not the #212 mechanism.
                {
                    ms::DynMat<T> Py(1, 1);
                    Py(0, 0) = P(2, 2);
                    const std::array<T, 1> ey{{e[2]}};
                    nees_yaw.add(ev::nees<T>(std::span<const T>{ey}, Py), 1);
                    ms::DynMat<T> Prp(2, 2);
                    std::array<T, 2> erp{{e[0], e[1]}};
                    for (std::size_t i = 0; i < 2; ++i)
                        for (std::size_t j = 0; j < 2; ++j)
                            Prp(i, j) = P(i, j);
                    nees_rollpitch.add(ev::nees<T>(std::span<const T>{erp}, Prp), 2);
                }
                // Per 3-block NEES (attitude is the #212 tell). Backend order puts
                // attitude at 0, velocity at 3, position at 6 — remap to report
                // attitude/position/velocity/biases by their backend offsets.
                static constexpr std::array<std::size_t, ev::kNumNavBlocks> off{{0, 6, 3, 9, 12}};
                for (std::size_t bk = 0; bk < ev::kNumNavBlocks; ++bk) {
                    ms::DynMat<T> Pb(3, 3);
                    std::array<T, 3> eb{};
                    for (std::size_t i = 0; i < 3; ++i) {
                        eb[i] = e[off[bk] + i];
                        for (std::size_t j = 0; j < 3; ++j)
                            Pb(i, j) = P(off[bk] + i, off[bk] + j);
                    }
                    nees_block[bk].add(ev::nees<T>(std::span<const T>{eb}, Pb), 3);
                }
            } catch (const std::domain_error&) {
                // non-PD covariance at this frame — skip
            }
        }
        if (std::getenv("VERDICT_PER_TRIAL"))
            std::cerr << "trial " << trial << " final pos err " << trial_pos_err << " m\n";
    }

    const auto rep = nees_full.report();
    std::cout << "\n=== R-IEKF invariant-backend NEES diagnostic (issue #212) ===\n";
    std::cout << "trials=" << trials << "  samples=" << rep.samples << "  (warmup " << warmup_s << " s skipped)\n";
    std::cout << "full nav NEES (15 DoF): normalized=" << rep.normalized << "  band [" << rep.lower << ", " << rep.upper
              << "]  -> "
              << (rep.consistent() ? "CONSISTENT" : (rep.overconfident ? "OVER-confident" : "UNDER-confident")) << "\n";
    std::cout << "per-block (each 3 DoF, normalized; 1.0 = consistent):\n";
    for (std::size_t bk = 0; bk < ev::kNumNavBlocks; ++bk) {
        const auto br = nees_block[bk].report();
        std::cout << "  " << ev::nav_block_name(bk) << ": " << br.normalized << "  ["
                  << (br.consistent() ? "consistent" : (br.overconfident ? "OVER" : "under")) << "]\n";
    }
    std::cout << "attitude split:  yaw(1 DoF)=" << nees_yaw.report().normalized
              << "   roll/pitch(2 DoF)=" << nees_rollpitch.report().normalized
              << "   <- roll/pitch is gravity-observable; if it is also >>1 the cause is\n"
              << "      broad over-fusion in the driver, not the gravity-yaw gauge leak.\n";
    // Guard the per-sample averages: n_diag is 0 with no steady-state frames, and
    // n_upd is 0 under VERDICT_NO_UPDATE — emit N/A rather than dividing by zero.
    std::cout << "diagnostics: ";
    if (n_diag > 0)
        std::cout << "RMS pos err=" << std::sqrt(sum_pos_err2 / n_diag)
                  << " m,  RMS att err=" << std::sqrt(sum_att_err2 / n_diag)
                  << " rad,  mean att sigma=" << std::sqrt(sum_att_sig2 / n_diag / 3) << " rad";
    else
        std::cout << "no steady-state samples (warmup skipped all frames)";
    std::cout << ",  mean feats/update=";
    if (n_upd > 0)
        std::cout << (sum_feat / n_upd);
    else
        std::cout << "N/A";
    std::cout << "\n";
    std::cout << "\nInterpretation: NEES >> 1 across ALL blocks (and roll/pitch >> yaw) is a\n"
              << "broad driver over-fusion signature, NOT the unit-proven gravity-yaw fix\n"
              << "failing. This harness localises the remaining gap; it is not a pass.\n";
    return 0;
}
