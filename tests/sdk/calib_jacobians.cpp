// S10 online-calibration: measurement-Jacobian validation (issue #332).
//
// The camera update grows two new Jacobian blocks when the camera↔IMU
// extrinsics are estimated in-state: ∂h/∂δθ_ic and ∂h/∂δp_ic. These are the
// load-bearing math of the fix — get the sign or convention wrong and the
// filter walks the calibration the wrong way. This locks them by central
// finite differences against the analytic blocks the updater builds, on a
// generic (non-degenerate) pose so every entry is exercised. The pre-existing
// feature/clone Jacobians are differenced too, confirming the test seam returns
// exactly what update() stacks into H.

#include <branes/sdk/msckf.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstddef>

namespace {
namespace ms = branes::sdk::msckf;
using T = double;
using SO3 = ms::State<T>::SO3;
using Vec3 = ms::State<T>::Vec3;

// A generic scene: estimated extrinsic away from identity, a tilted/translated
// clone, a feature well in front of the camera.
ms::State<T> make_scene() {
    ms::State<T> s(0.1);
    ms::State<T>::CalibState c;
    c.R_imu_cam = SO3::exp(Vec3{{0.08, -0.05, 0.15}});
    c.p_imu_cam = Vec3{{0.04, -0.02, 0.03}};
    s.enable_calibration({c}, /*rot_sigma=*/0.1, /*trans_sigma=*/0.05);
    s.R = SO3::exp(Vec3{{0.2, 0.1, -0.1}});
    s.p = Vec3{{0.5, -0.3, 0.2}};
    ms::StateHelper<T>::augment_clone(s);  // clone 0 = current pose
    return s;
}
}  // namespace

TEST_CASE("S10 extrinsic Jacobians match finite differences", "[sdk][s10][update][jacobian]") {
    const ms::CameraUpdater<T> upd(std::vector<ms::CameraExtrinsics<T>>(1));
    const ms::State<T> s = make_scene();
    const ms::CameraObservation<T> obs{0, 0, {{0.0, 0.0}}};  // xy unused by the Jacobian
    const Vec3 p_f{{1.0, 0.5, 5.0}};

    const auto J = upd.projection_jacobians(s, obs, p_f);
    REQUIRE(J.valid);  // feature is in front of the camera

    const T eps = 1e-6;
    auto h_of = [&](const ms::State<T>& st) { return upd.projection_jacobians(st, obs, p_f).h; };

    for (std::size_t k = 0; k < 3; ++k) {
        Vec3 ep{{0, 0, 0}}, em{{0, 0, 0}};
        ep[k] = eps;
        em[k] = -eps;

        // ∂h/∂δθ_ic — right-perturb the extrinsic rotation.
        {
            ms::State<T> sp = s, sm = s;
            sp.calib[0].R_imu_cam = s.calib[0].R_imu_cam * SO3::exp(ep);
            sm.calib[0].R_imu_cam = s.calib[0].R_imu_cam * SO3::exp(em);
            const auto hp = h_of(sp);
            const auto hm = h_of(sm);
            for (std::size_t a = 0; a < 2; ++a)
                REQUIRE((hp[a] - hm[a]) / (2 * eps) == Catch::Approx(J.Hext_theta(a, k)).margin(1e-6));
        }
        // ∂h/∂δp_ic — perturb the extrinsic translation.
        {
            ms::State<T> sp = s, sm = s;
            sp.calib[0].p_imu_cam = Vec3{
                {s.calib[0].p_imu_cam[0] + ep[0], s.calib[0].p_imu_cam[1] + ep[1], s.calib[0].p_imu_cam[2] + ep[2]}};
            sm.calib[0].p_imu_cam = Vec3{
                {s.calib[0].p_imu_cam[0] + em[0], s.calib[0].p_imu_cam[1] + em[1], s.calib[0].p_imu_cam[2] + em[2]}};
            const auto hp = h_of(sp);
            const auto hm = h_of(sm);
            for (std::size_t a = 0; a < 2; ++a)
                REQUIRE((hp[a] - hm[a]) / (2 * eps) == Catch::Approx(J.Hext_p(a, k)).margin(1e-6));
        }
        // ∂h/∂δθ (clone orientation) — already shipped; confirm the seam agrees.
        {
            ms::State<T> sp = s, sm = s;
            sp.clones[0].R = s.clones[0].R * SO3::exp(ep);
            sm.clones[0].R = s.clones[0].R * SO3::exp(em);
            const auto hp = h_of(sp);
            const auto hm = h_of(sm);
            for (std::size_t a = 0; a < 2; ++a)
                REQUIRE((hp[a] - hm[a]) / (2 * eps) == Catch::Approx(J.Htheta(a, k)).margin(1e-6));
        }
    }

    // ∂h/∂p_f (feature) — perturb the world point directly.
    for (std::size_t k = 0; k < 3; ++k) {
        Vec3 pp = p_f, pm = p_f;
        pp[k] += eps;
        pm[k] -= eps;
        const auto hp = upd.projection_jacobians(s, obs, pp).h;
        const auto hm = upd.projection_jacobians(s, obs, pm).h;
        for (std::size_t a = 0; a < 2; ++a)
            REQUIRE((hp[a] - hm[a]) / (2 * eps) == Catch::Approx(J.Hf(a, k)).margin(1e-6));
    }
}
