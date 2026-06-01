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
// (extract the published EuRoC ASL .zip to get the `mav0` directory).
//
// ATE threshold rationale (documented per the issue): published keyframe
// ATE on V1_01_easy is ~0.05–0.06 m for OpenVINS and ~0.08 m for
// VINS-Fusion. This MVP MSCKF is simpler (no online intrinsics/extrinsics
// refinement, no loop closure), so the gate is set to a generous 0.50 m —
// enough to catch a broken pipeline without over-fitting to a tuned SOTA
// number.

#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>
#include <branes/sdk/vio_estimator.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ev = branes::sdk::eval;
using T = double;
using SE3 = branes::math::lie::SE3<T>;
using SO3 = branes::math::lie::SO3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;

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
        f << "#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z\n";
        f << "1000000000,1.0,2.0,3.0,1.0,0.0,0.0,0.0\n";
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

    fs::remove_all(root);
}

TEST_CASE("V1_01_easy replay keeps ATE under threshold", "[vio][euroc][dataset]") {
    const char* env = std::getenv("CORTEX_EUROC_V101");
    if (env == nullptr || std::string(env).empty())
        SKIP("set CORTEX_EUROC_V101 to the V1_01_easy/mav0 directory to run this benchmark");

    using Backend = bs::MsckfBackend<T>;
    using Estimator = bs::VioEstimator<T, Backend>;

    // EuRoC cam0 (pinhole-radtan) intrinsics, from the published calibration.
    Backend::CameraCalibration cal;
    cal.intrinsics =
        Backend::Camera(458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    Estimator est(Backend(std::vector<Backend::CameraCalibration>{cal}));

    bs::VioConfig cfg;
    cfg.max_clones = 11;
    const auto traj = bs::euroc::replay(std::string(env), est, cfg);
    REQUIRE(traj.size() > 100);

    const auto gt = bs::euroc::parse_groundtruth<T>(std::string(env));
    const auto matched = ev::associate(traj, gt, 0.01);
    REQUIRE(matched.estimated.size() > 100);

    const T ate = ev::ate_rmse(matched.estimated, matched.reference);
    INFO("V1_01_easy ATE = " << ate << " m (gate 0.50 m)");
    REQUIRE(ate < 0.50);
}

namespace {

// Replay a moving-start EuRoC sequence and validate it bootstraps with a real
// attitude (the dynamic VI-init path, epic #211 — surfaced here) and lands ATE
// under a generous gate. Gated on `env_var` (the sequence's mav0 directory), so
// it is skipped in CI; the gate is generous because these are the *difficult*
// sequences run through the MVP MSCKF with identity extrinsics — tune against
// the printed ATE on first local run.
void replay_moving_start(const char* env_var, const char* label, T ate_gate) {
    const char* env = std::getenv(env_var);
    if (env == nullptr || std::string(env).empty())
        SKIP(std::string("set ") + env_var + " to the " + label + "/mav0 directory to run this benchmark");

    using Backend = bs::MsckfBackend<T>;
    using Estimator = bs::VioEstimator<T, Backend>;
    typename Backend::CameraCalibration cal;
    cal.intrinsics = typename Backend::Camera(
        458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    Estimator est(Backend(std::vector<typename Backend::CameraCalibration>{cal}));

    bs::VioConfig cfg;
    cfg.max_clones = 11;
    const auto traj = bs::euroc::replay(std::string(env), est, cfg);
    REQUIRE(traj.size() > 100);

    // The platform moves from the start; init must resolve a real attitude
    // (static, dynamic VI alignment, or gravity-only) — never the divergent
    // identity fallback. Surface which path was taken.
    const auto& diag = est.backend().init_diagnostics();
    INFO(label << " init method = " << bs::to_string(diag.method));
    REQUIRE(diag.method != bs::InitMethod::None);
    REQUIRE(diag.method != bs::InitMethod::Identity);

    const auto gt = bs::euroc::parse_groundtruth<T>(std::string(env));
    const auto matched = ev::associate(traj, gt, 0.01);
    REQUIRE(matched.estimated.size() > 100);
    const T ate = ev::ate_rmse(matched.estimated, matched.reference);
    INFO(label << " ATE = " << ate << " m (gate " << ate_gate << " m)");
    REQUIRE(ate < ate_gate);
}

}  // namespace

TEST_CASE("MH_05_difficult replay (moving start) bootstraps and stays bounded", "[vio][euroc][dataset]") {
    replay_moving_start("CORTEX_EUROC_MH05", "MH_05_difficult", 1.5);
}

TEST_CASE("V2_03_difficult replay (moving start) bootstraps and stays bounded", "[vio][euroc][dataset]") {
    replay_moving_start("CORTEX_EUROC_V203", "V2_03_difficult", 1.5);
}
