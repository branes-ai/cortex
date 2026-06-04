// Synthetic VIO world + end-to-end backend integration (the noise-demo source).
//
// Validates that the synthetic world generator (sdk/eval/synthetic_world.hpp)
// produces a self-consistent set of sensor streams + exact ground truth that the
// real MSCKF backend can track: with zero injected noise on a gentle (ground-
// robot) trajectory the filter stays initialized, tracks within a bound, keeps a
// PSD covariance, and is over-conservative (NIS < 1) on perfectly clean sensors.
// This is the foundation the vio_pipeline noise→robustness demo builds on.

#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <span>
#include <vector>

namespace {
namespace ev = branes::sdk::eval;
using T = double;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;

double norm3(const Vec3& a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}
}  // namespace

TEST_CASE("synthetic world generates a trackable VIO stream with exact ground truth", "[sdk][synthetic][pipeline]") {
    ev::SyntheticConfig<T> cfg;  // gentle "ground robot" so the baseline is tight
    cfg.trans_amp = 0.5;
    cfg.motion_rate = 0.6;
    cfg.yaw_amp = 0.3;
    const auto world = ev::generate_world<T>(cfg);

    // The world is non-degenerate: high-rate IMU, frame-rate camera, and the
    // camera actually sees a healthy number of landmarks per frame.
    REQUIRE(world.imu.size() > 1000);
    REQUIRE(world.frames.size() > 100);
    REQUIRE(world.frames.size() == world.gt.size());
    std::size_t feat = 0;
    for (const auto& f : world.frames)
        feat += f.obs.size();
    REQUIRE(feat / world.frames.size() > 20);  // mean features per frame

    using Backend = branes::sdk::MsckfBackend<T>;
    Backend::CameraCalibration cal;
    cal.intrinsics = world.camera;
    cal.extrinsics.R_imu_cam = world.R_imu_cam;
    cal.extrinsics.p_imu_cam = world.p_imu_cam;
    Backend backend(std::vector<Backend::CameraCalibration>{cal});
    branes::sdk::VioConfig vcfg;
    backend.initialize(vcfg);

    std::size_t imu_idx = 0, tracked = 0;
    double sq = 0;
    for (std::size_t f = 0; f < world.frames.size(); ++f) {
        const double t = world.frames[f].t;
        for (; imu_idx < world.imu.size() && world.imu[imu_idx].timestamp_s <= t; ++imu_idx)
            backend.process_imu(world.imu[imu_idx]);
        backend.process_camera(t, std::span<const branes::sdk::FrontendObservation<T>>{world.frames[f].obs});
        if (!backend.initialized())
            continue;
        const auto est = backend.current_state();
        const Vec3 ep = est.T_world_imu.translation();
        for (int i = 0; i < 3; ++i)
            REQUIRE(std::isfinite(ep[i]));  // no NaN/Inf
        const Vec3& gp = world.gt[f].p;
        const double e = norm3(Vec3{{ep[0] - gp[0], ep[1] - gp[1], ep[2] - gp[2]}});
        sq += e * e;
        ++tracked;
    }

    REQUIRE(backend.initialized());
    REQUIRE(tracked > world.frames.size() * 3 / 4);  // filter ran most of the sequence
    const double ate = std::sqrt(sq / static_cast<double>(tracked));
    INFO("zero-noise ATE (RMS position) = " << ate << " m");
    REQUIRE(ate < 0.5);  // gentle motion, clean sensors ⇒ tracks within half a metre
    // Covariance stays PSD, and on perfectly clean sensors the filter is
    // over-conservative (innovations smaller than the assumed noise ⇒ NIS < 1).
    REQUIRE(branes::sdk::msckf::is_positive_semidefinite(backend.state().covariance()));
    const double nis = backend.nis_consistency().report().normalized;
    REQUIRE(std::isfinite(nis));
    REQUIRE(nis < 1.0);
}
