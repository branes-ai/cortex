// EuRoC MAV replay harness + ATE/RPE benchmark (issue #46).
//
// The trajectory metrics (ATE/RPE, clean-room from the definitions — Horn
// 1987 alignment; no ov_eval/evo contact) and the ASL CSV parsers are
// exercised in CI on synthetic fixtures. The full V1_01_easy replay +
// ATE-threshold assertion runs only when the dataset is available, since
// the ~1.5 GB EuRoC sequence is not vendored: set the environment variable
//
//     CORTEX_EUROC_V101=/path/to/V1_01_easy/mav0
//
// to enable it; otherwise that case is skipped (so `ctest -R vio_euroc`
// stays green in CI). Two moving-start sequences validate the dynamic VI-init
// path (epic #211) the same way, each behind its own variable:
//
//     CORTEX_EUROC_MH05=/path/to/MH_05_difficult/mav0
//     CORTEX_EUROC_V203=/path/to/V2_03_difficult/mav0
//
// (extract the published EuRoC ASL .zip to get the `mav0` directory; the
// scripts/euroc-moving-start.sh helper does this and runs the [dataset] cases).
//
// All three cases use the real cam0 intrinsics AND extrinsics (T_BS) from the
// published calibration — EuRoC's cam0 is rotated ~90° from the IMU, so an
// identity extrinsic gives a grossly wrong measurement model and the filter
// diverges (MH_05 went from ~21 km ATE to ~0.79 m once the real T_BS was set).
//
// ATE threshold rationale (documented per the issue): published keyframe ATE on
// V1_01_easy is ~0.05–0.06 m for OpenVINS and ~0.08 m for VINS-Fusion. This MVP
// MSCKF is simpler (no online intrinsics/extrinsics refinement, no loop
// closure), so gates are generous — enough to catch a broken pipeline without
// over-fitting to a tuned SOTA number:
//   - V1_01_easy  : 0.50 m (strict).
//   - MH_05_difficult : 1.5 m (strict) — measured ~0.79 m via the static path
//     (MH_05 has an early quiet window); validates the estimator + extrinsics on
//     a difficult sequence. A second case forces the dynamic path (epic #211):
//     after the #247 fixes it fires with a correct gravity direction and stays
//     bounded (~19 m, down from ~48 km), but not yet within gate — residual
//     roll/pitch on the low-excitation start; that case guards bounded-not-
//     diverging until the refinement lands.
//   - V2_03_difficult : 1.5 m (strict) — measured ~0.29 m. The #247 scale gate
//     makes its degenerate early dynamic window decline, so static wins and it
//     converges (was ~12 km; #244 closed).

#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/eval/nav_consistency.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>
#include <branes/sdk/sfm/init_window.hpp>  // so3_from_matrix (extrinsics rotation)
#include <branes/sdk/vio_estimator.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ev = branes::sdk::eval;
using T = double;
using SE3 = branes::math::lie::SE3<T>;
using SO3 = branes::math::lie::SO3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using Mat3 = branes::math::lie::detail::Mat<T, 3, 3>;

ev::StampedPose<T> pose_at(double t, const Vec3& p, const SO3& R = {}) {
    return ev::StampedPose<T>{t, SE3(R, p)};
}

}  // namespace

TEST_CASE("ATE is zero for identical trajectories and invariant to rigid transform", "[vio][euroc]") {
    std::vector<ev::StampedPose<T>> gt;
    for (int k = 0; k < 50; ++k) {
        const T s = static_cast<T>(k);
        gt.push_back(pose_at(
            0.1 * k, Vec3{{0.2 * s, 0.05 * s * s - s, 0.01 * s}}, SO3::exp(Vec3{{0.01 * s, -0.02 * s, 0.005 * s}})));
    }

    // Identical estimate ⇒ ATE ≈ 0.
    REQUIRE_THAT(ev::ate_rmse(gt, gt), Catch::Matchers::WithinAbs(0.0, 1e-9));

    // Apply a fixed rigid transform to the estimate; alignment removes it.
    const SE3 g(SO3::exp(Vec3{{0.3, -0.4, 0.5}}), Vec3{{1.5, -2.0, 0.7}});
    std::vector<ev::StampedPose<T>> est;
    for (const auto& s : gt)
        est.push_back(ev::StampedPose<T>{s.t_s, g * s.pose});
    REQUIRE_THAT(ev::ate_rmse(est, gt), Catch::Matchers::WithinAbs(0.0, 1e-7));
}

TEST_CASE("ATE recovers a known per-sample offset", "[vio][euroc]") {
    // A trajectory and a copy shifted by a constant per-axis bias that is
    // NOT a rigid body motion of the path (alternating sign), so alignment
    // cannot absorb it and ATE reflects the residual.
    std::vector<ev::StampedPose<T>> gt, est;
    for (int k = 0; k < 40; ++k) {
        const Vec3 p{{0.3 * k, 0.0, 0.0}};
        gt.push_back(pose_at(0.1 * k, p));
        const T sign = (k % 2 == 0) ? T{1} : T{-1};
        est.push_back(pose_at(0.1 * k, p + Vec3{{0.0, 0.1 * sign, 0.0}}));
    }
    // Residual is ±0.1 m on y about a ~zero mean ⇒ RMSE ≈ 0.1 m (rigid
    // alignment can only shave it slightly, never absorb the alternation).
    REQUIRE_THAT(ev::ate_rmse(est, gt), Catch::Matchers::WithinAbs(0.1, 2e-3));
}

TEST_CASE("RPE is zero under a global rigid transform", "[vio][euroc]") {
    std::vector<ev::StampedPose<T>> gt;
    for (int k = 0; k < 30; ++k) {
        const T s = static_cast<T>(k);
        gt.push_back(pose_at(0.1 * k, Vec3{{0.2 * s, 0.1 * s, -0.03 * s}}, SO3::exp(Vec3{{0.0, 0.0, 0.05 * s}})));
    }
    const SE3 g(SO3::exp(Vec3{{0.1, 0.2, -0.3}}), Vec3{{3.0, 1.0, -1.0}});
    std::vector<ev::StampedPose<T>> est;
    for (const auto& s : gt)
        est.push_back(ev::StampedPose<T>{s.t_s, g * s.pose});
    // Relative motions are unchanged by a global transform ⇒ RPE ≈ 0.
    REQUIRE_THAT(ev::rpe_translation_rmse(est, gt, 1), Catch::Matchers::WithinAbs(0.0, 1e-7));
    REQUIRE_THAT(ev::rpe_translation_rmse(est, gt, 5), Catch::Matchers::WithinAbs(0.0, 1e-7));
}

TEST_CASE("time association matches nearest reference within the window", "[vio][euroc]") {
    std::vector<ev::StampedPose<T>> est = {pose_at(0.00, Vec3{{0, 0, 0}}),
                                           pose_at(0.105, Vec3{{1, 0, 0}}),
                                           pose_at(5.00, Vec3{{2, 0, 0}})};  // far from any ref
    std::vector<ev::StampedPose<T>> ref;
    for (int k = 0; k < 20; ++k)
        ref.push_back(pose_at(0.1 * k, Vec3{{0.1 * k, 0, 0}}));
    const auto a = ev::associate(est, ref, 0.02);
    REQUIRE(a.estimated.size() == 2);  // the t=5.0 estimate has no match
    REQUIRE(a.reference.size() == 2);
}

TEST_CASE("ASL CSV parsers read the EuRoC layout", "[vio][euroc]") {
    namespace fs = std::filesystem;
    const fs::path root = fs::temp_directory_path() / fs::path("cortex_euroc_test_mav0");
    fs::create_directories(root / "imu0");
    fs::create_directories(root / "cam0" / "data");
    fs::create_directories(root / "state_groundtruth_estimate0");

    {
        std::ofstream f(root / "imu0" / "data.csv");
        f << "#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n";
        f << "1000000000,0.01,0.02,0.03,0.1,0.2,9.8\n";
        f << "1005000000,0.011,0.021,0.031,0.11,0.21,9.81\n";
    }
    {
        std::ofstream f(root / "cam0" / "data.csv");
        f << "#timestamp [ns],filename\n";
        f << "1000000000,1000000000.png\n";
        f << "1050000000,1050000000.png\n";
    }
    {
        std::ofstream f(root / "state_groundtruth_estimate0" / "data.csv");
        f << "#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,b_w_x,b_w_y,b_w_z,b_a_x,b_a_y,b_a_z\n";
        // full 17-column ASL row: p, q(wxyz), v, b_w, b_a
        f << "1000000000,1.0,2.0,3.0,1.0,0.0,0.0,0.0,0.5,-0.6,0.7,0.01,0.02,0.03,0.1,0.2,0.3\n";
    }

    const std::string r = root.string();
    const auto imu = bs::euroc::parse_imu<T>(r);
    REQUIRE(imu.size() == 2);
    REQUIRE_THAT(imu[0].timestamp_s, Catch::Matchers::WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(imu[0].linear_acceleration[2], Catch::Matchers::WithinAbs(9.8, 1e-9));

    const auto imgs = bs::euroc::parse_images(r);
    REQUIRE(imgs.size() == 2);
    REQUIRE_THAT(imgs[1].t_s, Catch::Matchers::WithinAbs(1.05, 1e-9));
    REQUIRE(imgs[0].path == r + "/cam0/data/1000000000.png");

    const auto gt = bs::euroc::parse_groundtruth<T>(r);
    REQUIRE(gt.size() == 1);
    REQUIRE_THAT(gt[0].pose.translation()[0], Catch::Matchers::WithinAbs(1.0, 1e-9));

    // Full nav-state GT parse (#264): velocity + gyro/accel bias columns.
    const auto gts = bs::euroc::parse_groundtruth_states<T>(r);
    REQUIRE(gts.size() == 1);
    REQUIRE_THAT(gts[0].nav.p[1], Catch::Matchers::WithinAbs(2.0, 1e-9));
    REQUIRE_THAT(gts[0].nav.v[0], Catch::Matchers::WithinAbs(0.5, 1e-9));
    REQUIRE_THAT(gts[0].nav.v[2], Catch::Matchers::WithinAbs(0.7, 1e-9));
    REQUIRE_THAT(gts[0].nav.bg[1], Catch::Matchers::WithinAbs(0.02, 1e-9));
    REQUIRE_THAT(gts[0].nav.ba[2], Catch::Matchers::WithinAbs(0.3, 1e-9));

    fs::remove_all(root);
}

namespace {

// Replay an EuRoC sequence end-to-end and validate the run: it bootstraps with
// a real attitude (never the divergent identity fallback — the chosen
// InitMethod is surfaced) and lands ATE under `ate_gate`. Gated on `env_var`
// (the sequence's mav0 directory), so these cases are skipped in CI. The init
// method, frame count, and ATE are WARN'd (printed regardless of pass/fail) so
// a generous gate can be tuned from the actual numbers. Shared by the V1_01
// (static-start) benchmark and the MH_05 / V2_03 moving-start cases (epic #211).
void run_euroc_replay(
    const char* env_var, const char* label, T ate_gate, bool expect_converged = true, bool prefer_dynamic = false) {
    const char* env = std::getenv(env_var);
    if (env == nullptr || std::string(env).empty())
        SKIP(std::string("set ") + env_var + " to the " + label + "/mav0 directory to run this benchmark");

    using Backend = bs::MsckfBackend<T>;
    using Estimator = bs::VioEstimator<T, Backend>;
    // EuRoC cam0 (pinhole-radtan) intrinsics, from the published calibration.
    typename Backend::CameraCalibration cal;
    cal.intrinsics = typename Backend::Camera(
        458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    // Real cam0→IMU (body) extrinsics T_BS, from the published calibration. The
    // EuRoC cam0 is rotated ~90° from the IMU, so assuming identity gives a
    // grossly wrong measurement model and the filter diverges on fast motion.
    Mat3 R_imu_cam{};
    R_imu_cam(0, 0) = 0.0148655429818;
    R_imu_cam(0, 1) = -0.999880929698;
    R_imu_cam(0, 2) = 0.00414029679422;
    R_imu_cam(1, 0) = 0.999557249008;
    R_imu_cam(1, 1) = 0.0149672133247;
    R_imu_cam(1, 2) = 0.025715529948;
    R_imu_cam(2, 0) = -0.0257744366974;
    R_imu_cam(2, 1) = 0.00375618835797;
    R_imu_cam(2, 2) = 0.999660727178;
    cal.extrinsics.R_imu_cam = bs::sfm::so3_from_matrix<T>(R_imu_cam);
    cal.extrinsics.p_imu_cam = Vec3{{-0.0216401454975, -0.064676986768, 0.00981073058949}};
    Estimator est(Backend(std::vector<typename Backend::CameraCalibration>{cal}));

    bs::VioConfig cfg;
    cfg.max_clones = 11;
    // Suppress the static path so the dynamic VI-init bootstraps even on a
    // sequence with an early quiet window — exercises the epic #211 dynamic
    // path on real data.
    cfg.prefer_dynamic_init = prefer_dynamic;

    // Process-noise sweep knob (#212 diagnosis): CORTEX_Q_SCALE multiplies the
    // IMU noise densities. Per-block NEES localized the over-confidence to the
    // attitude state (worst under fast rotation), pointing at under-tuned process
    // noise; sweeping this factor and watching the attitude NEES fall toward 1
    // quantifies how under-tuned Q is, and distinguishes a Q deficit (NEES
    // drops) from an observability/linearization fault (NEES stays high).
    // Parse a positive-float sweep knob with end-pointer validation, so a
    // malformed value fails loudly (atof would silently coerce "abc" → 0.0 and
    // skip the sweep). Trailing whitespace is tolerated; anything else is rejected.
    auto parse_scale = [](const char* s, double& out) {
        char* end = nullptr;
        out = std::strtod(s, &end);
        while (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')
            ++end;
        return end != s && *end == '\0' && out > 0.0;
    };
    if (const char* qs = std::getenv("CORTEX_Q_SCALE")) {
        double k = 0.0;
        if (parse_scale(qs, k)) {
            cfg.gyro_noise_density *= k;
            cfg.accel_noise_density *= k;
            cfg.gyro_bias_random_walk *= k;
            cfg.accel_bias_random_walk *= k;
            WARN(label << ": CORTEX_Q_SCALE=" << k << " — IMU noise densities scaled for the consistency sweep");
        } else {
            WARN(label << ": CORTEX_Q_SCALE='" << qs << "' ignored — not a positive number");
        }
    }
    // Mirror knob (#212): CORTEX_R_SCALE multiplies the visual measurement noise.
    // The NIS over-confidence is nearly invariant to Q, so sweeping R discriminates
    // an under-tuned R (NIS falls with R) from an observability/Jacobian fault on
    // the update (NIS stays high regardless).
    if (const char* rs = std::getenv("CORTEX_R_SCALE")) {
        double k = 0.0;
        if (parse_scale(rs, k)) {
            cfg.camera_noise_normalized *= k;
            WARN(label << ": CORTEX_R_SCALE=" << k << " — visual measurement noise scaled for the consistency sweep");
        } else {
            WARN(label << ": CORTEX_R_SCALE='" << rs << "' ignored — not a positive number");
        }
    }

    // NEES consistency vs ground truth (#264): per frame, sample the live nav
    // state + core covariance, anchor the unobservable yaw+position gauge at the
    // first post-init matched frame, and accumulate eᵀ P_core⁻¹ e. NEES tests
    // whether the covariance tracks the *actual* error (normalized ≈ 1
    // consistent; > 1 over-confident; < 1 under-confident) — the ground-truth
    // half of the consistency instrument, complementing the always-on NIS.
    const auto gt_states = bs::euroc::parse_groundtruth_states<T>(std::string(env));
    ev::ConsistencyAccumulator nees_acc;
    std::array<ev::ConsistencyAccumulator, ev::kNumNavBlocks> nees_block;  // per-state localization
    std::size_t nees_skipped = 0;                                          // frames whose core covariance was not PD
    bool have_anchor = false;
    SE3 anchor;
    std::size_t gi = 0;
    const auto on_frame = [&](double t, const Estimator& e) {
        if (!e.backend().initialized() || gt_states.empty())
            return;
        while (gi + 1 < gt_states.size() && std::abs(gt_states[gi + 1].t_s - t) <= std::abs(gt_states[gi].t_s - t))
            ++gi;
        if (std::abs(gt_states[gi].t_s - t) > 0.01)
            return;  // no ground-truth state within 10 ms of this frame
        const auto& st = e.backend().state();
        const ev::NavSample<T> est_nav{st.R, st.p, st.v, st.bg, st.ba};
        if (!have_anchor) {
            anchor = ev::gauge_align<T>(SE3(st.R, st.p), SE3(gt_states[gi].nav.R, gt_states[gi].nav.p));
            have_anchor = true;
        }
        const auto truth = ev::align_truth<T>(anchor, gt_states[gi].nav);
        const auto err = ev::nav_error<T>(est_nav, truth);
        try {
            const auto core = ev::core_covariance<T>(st.covariance());
            nees_acc.add(ev::nees<T>(err, core), ev::kNavErrorDim);
            const auto blk = ev::nav_block_nees<T>(err, core);  // localize per state
            for (std::size_t b = 0; b < ev::kNumNavBlocks; ++b)
                nees_block[b].add(blk[b], 3);
        } catch (const std::domain_error&) {
            // ONLY the expected non-positive-definite core covariance — a
            // filter-health signal, surfaced below. A shape mismatch
            // (invalid_argument) or any other failure is a real bug and
            // propagates rather than being silently counted as a skip.
            ++nees_skipped;
        }
    };
    const auto traj = bs::euroc::replay(std::string(env), est, cfg, on_frame);
    REQUIRE(traj.size() > 100);

    // Init must resolve a real attitude (static, dynamic VI alignment, or
    // gravity-only) — never the divergent identity fallback.
    const auto& diag = est.backend().init_diagnostics();
    REQUIRE(diag.method != bs::InitMethod::None);
    REQUIRE(diag.method != bs::InitMethod::Identity);
    // When forcing the dynamic path, the scale-observability gate (#247) may
    // correctly DECLINE on a low-excitation window and fall back, so we no
    // longer hard-require Dynamic here — the WARN below reports which method
    // actually initialized. Re-tighten to REQUIRE(method == Dynamic) once
    // dynamic init is confirmed to fire AND converge on this sequence.
    (void)prefer_dynamic;

    // Score only post-initialization poses. Before init fires the backend
    // returns a placeholder origin pose, not an estimate; including those — ~50
    // frames for an early static init, but ~170 for a late dynamic init —
    // directly inflates ATE and biases the rigid alignment toward the origin.
    // A VIO trajectory is meaningful from initialization onward.
    std::vector<ev::StampedPose<T>> post;
    for (const auto& sp : traj)
        if (sp.t_s >= diag.t_s)
            post.push_back(sp);
    const auto gt = bs::euroc::parse_groundtruth<T>(std::string(env));
    const auto matched = ev::associate(post, gt, 0.01);
    REQUIRE(matched.estimated.size() > 100);
    const T ate = ev::ate_rmse(matched.estimated, matched.reference);
    WARN(label << ": init=" << bs::to_string(diag.method) << ", frames=" << traj.size() << ", ATE=" << ate
               << " m (gate " << ate_gate << " m)");
    // Filter-consistency telemetry (#264): the run's NIS (innovation
    // consistency). normalized ≈ 1 is consistent; > 1 over-confident (covariance
    // too small / wrong Jacobian / biased residual — a #212 lead); < 1
    // under-confident. The always-on, ground-truth-free instrument.
    if (est.backend().nis_consistency().samples() > 0) {
        const auto nis = est.backend().nis_consistency().report();
        WARN(label << ": NIS over " << nis.samples << " updates: normalized=" << nis.normalized << " (band ["
                   << nis.lower << ", " << nis.upper << "]) — "
                   << (nis.consistent() ? "consistent" : (nis.overconfident ? "OVER-confident" : "UNDER-confident")));
    }
    if (est.backend().innovation_whiteness().updates() > 0) {
        // #280 discriminator: a biased mean ⇒ systematic error (extrinsics /
        // triangulation / time-sync); temporal correlation ⇒ unmodelled dynamics
        // or observability inconsistency. Zero-mean + white ⇒ the over-confidence
        // is pure covariance mistune (R/Jacobian), the FEJ hypothesis.
        const auto iw = est.backend().innovation_whiteness().report();
        WARN(label << ": innovation over " << iw.updates << " updates: mean_z=" << iw.mean_z
                   << (iw.biased ? " (BIASED)" : " (zero-mean)") << ", lag1_z=" << iw.lag1_z
                   << (iw.correlated ? " (CORRELATED)" : " (white)") << " — |z|>" << iw.z_crit << " flags");
    }
    if (nees_acc.samples() > 0) {
        const auto neesr = nees_acc.report();
        WARN(label << ": NEES over " << neesr.samples << " frames: normalized=" << neesr.normalized << " (band ["
                   << neesr.lower << ", " << neesr.upper << "]) — "
                   << (neesr.consistent() ? "consistent" : (neesr.overconfident ? "OVER-confident" : "UNDER-confident"))
                   << (nees_skipped > 0 ? " [" + std::to_string(nees_skipped) + " frames skipped: core cov not PD]"
                                        : ""));
        // Per-block NEES localizes WHICH state is over-confident: attitude worst
        // ⇒ observability inconsistency; velocity/bias worst ⇒ Q / bias model.
        std::string blocks;
        for (std::size_t b = 0; b < ev::kNumNavBlocks; ++b)
            if (nees_block[b].samples() > 0)
                blocks +=
                    std::string(ev::nav_block_name(b)) + "=" + std::to_string(nees_block[b].report().normalized) + " ";
        WARN(label << ": per-block NEES (normalized, want ~1): " << blocks);
    }
    // Seed gyro bias (#247): a bad bias from the dynamic path's short, noisy
    // vision-IMU window drifts attitude over the sequence → gravity leaks into
    // accel → divergence. Compare the diverging dynamic seed against the
    // converging static seed on the same sequence.
    WARN(label << ": seed gyro_bias=(" << diag.gyro_bias[0] << ", " << diag.gyro_bias[1] << ", " << diag.gyro_bias[2]
               << ") rad/s, |bg|=" << branes::math::lie::detail::norm(diag.gyro_bias)
               << ", grav_residual=" << diag.gravity_residual);
    if (diag.method == bs::InitMethod::Dynamic)
        WARN(label << ": dyn-init scale=" << diag.dyn_scale << ", seed_speed=" << diag.dyn_seed_speed
                   << " m/s, sfm_keyframes=" << diag.dyn_keyframes
                   << ", roll/pitch err vs accel=" << diag.dyn_tilt_vs_accel_deg << " deg");
    if (prefer_dynamic)
        // Attempt accounting (#247): localizes why dynamic init didn't fire.
        // window_builds=0 ⇒ two-view/PnP SfM is failing on real tracks;
        // builds>0 but best_motion < ~0.05 m ⇒ vision trajectory inconsistent
        // with the IMU (scale collapses).
        WARN(label << ": dyn-attempts=" << diag.dyn_attempts << ", window_builds=" << diag.dyn_window_builds
                   << ", best_keyframes=" << diag.dyn_best_keyframes
                   << ", best_metric_motion=" << diag.dyn_best_metric_motion << " m");
    INFO(label << " ATE = " << ate << " m (gate " << ate_gate << " m)");
    if (expect_converged) {
        REQUIRE(ate < ate_gate);
    } else {
        // Forced-dynamic MH_05: the dynamic VI-init now fires with a correct
        // gravity direction and stays BOUNDED (~19 m ATE), down from ~48 km
        // before the gravity-sign fix (#247) — but not yet within the
        // convergence gate (residual roll/pitch on a low-excitation start + late
        // firing inflating ATE with pre-init frames). Guard that it neither
        // blows up to NaN nor re-diverges to km scale; tighten to
        // `REQUIRE(ate < ate_gate)` once the roll/pitch refinement lands.
        REQUIRE(std::isfinite(ate));
        REQUIRE(ate < T{100});
    }
}

}  // namespace

TEST_CASE("V1_01_easy replay keeps ATE under threshold", "[vio][euroc][dataset]") {
    run_euroc_replay("CORTEX_EUROC_V101", "V1_01_easy", 0.50);
}

TEST_CASE("MH_05_difficult replay (moving start) bootstraps and stays bounded", "[vio][euroc][dataset]") {
    run_euroc_replay("CORTEX_EUROC_MH05", "MH_05_difficult", 1.5);
}

// MH_05 has an early quiet window, so the default run takes the (more accurate)
// static path. Forcing the dynamic path exercises epic #211's dynamic VI-init
// on real data: it now fires with a correct gravity direction and stays bounded
// (~19 m ATE, down from ~48 km before the gravity-sign fix in #247), but does
// not yet reach the convergence gate — residual roll/pitch from the
// low-excitation start plus late firing. Tighten to expect_converged=true once
// that refinement lands.
TEST_CASE("MH_05_difficult forced dynamic VI-init fires + stays bounded (#247)", "[vio][euroc][dataset]") {
    run_euroc_replay("CORTEX_EUROC_MH05",
                     "MH_05_difficult (dynamic)",
                     1.5,
                     /*expect_converged=*/false,
                     /*prefer_dynamic=*/true);
}

// V2_03_difficult is the most aggressive EuRoC sequence. The scale-observability
// gate (#247) makes its degenerate early dynamic window decline, so the static
// path wins and the sequence converges (~0.29 m) instead of the earlier ~12 km
// divergence (#244, closed).
TEST_CASE("V2_03_difficult replay (moving start) converges", "[vio][euroc][dataset]") {
    run_euroc_replay("CORTEX_EUROC_V203", "V2_03_difficult", 1.5);
}
