// S10 online-calibration: extrinsic-recovery capability test (issue #332).
//
// The decisive end-to-end check that the mechanism *self-calibrates*: seed the
// filter with a deliberately-WRONG camera↔IMU extrinsic, feed it observations
// generated from the TRUE extrinsic across a window of poses, and confirm the
// in-state estimate is driven back toward truth — and its covariance shrinks.
//
// This exercises the whole chain at once: the calibration Jacobians (sign and
// magnitude), the H_x coupling that makes T_CI observable, the EKF update, and
// the SO3 box-plus on the extrinsic mean. A wrong sign or a missing coupling
// would move the estimate the wrong way or not at all. Clean (noise-free)
// observations make convergence deterministic.

#include <branes/sdk/msckf.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace {
namespace ms = branes::sdk::msckf;
using T = double;
using SO3 = ms::State<T>::SO3;
using Vec3 = ms::State<T>::Vec3;

T rot_angle(const SO3& a, const SO3& b) {  // geodesic angle between two rotations
    return branes::math::lie::detail::norm((a.inverse() * b).log());
}
T vnorm(const Vec3& v) {
    return branes::math::lie::detail::norm(v);
}

// Normalized observation of world point p_f from a clone pose (R_c, p_c) through
// extrinsics (R_ic, p_ic) — the noise-free measurement the filter should explain.
bool observe(const SO3& R_c, const Vec3& p_c, const SO3& R_ic, const Vec3& p_ic, const Vec3& p_f, Vec3& xy) {
    const Vec3 y = R_c.inverse() * (p_f - p_c);     // feature in IMU frame
    const Vec3 pcam = R_ic.inverse() * (y - p_ic);  // feature in camera frame
    if (pcam[2] <= T{0})
        return false;
    xy = Vec3{{pcam[0] / pcam[2], pcam[1] / pcam[2], T{0}}};
    return true;
}
}  // namespace

TEST_CASE("S10 online calibration recovers a wrong extrinsic from observations", "[sdk][s10][update][calib]") {
    // Ground-truth extrinsic: ~2.9° camera↔IMU rotation offset + 5 cm translation.
    const SO3 R_ic_true = SO3::exp(Vec3{{0.01, -0.015, 0.05}});
    const Vec3 p_ic_true{{0.05, -0.02, 0.03}};

    // A window of known poses: a translating baseline (parallax) with mild
    // orientation variation; features spread across the field of view so the
    // extrinsic rotation is observable (a bearing-dependent angular offset).
    constexpr std::size_t kClones = 6;
    std::vector<SO3> Rc;
    std::vector<Vec3> pc;
    for (std::size_t c = 0; c < kClones; ++c) {
        const T u = static_cast<T>(c);
        Rc.push_back(SO3::exp(Vec3{{0.04 * std::sin(u), 0.03 * u - 0.07, 0.05 * std::cos(u)}}));
        pc.push_back(Vec3{{0.3 * u, 0.1 * std::sin(u), 0.05 * u}});
    }
    // Features spread in bearing at moderate depth, well in front of the cameras.
    std::vector<Vec3> feats;
    for (int ix = -2; ix <= 2; ++ix)
        for (int iy = -1; iy <= 1; ++iy)
            feats.push_back(Vec3{{static_cast<T>(ix), static_cast<T>(iy), 6.0}});

    // The filter: start the extrinsic estimate at IDENTITY (wrong by ~2.9°/5 cm),
    // with a generous prior so corrections are allowed. Clones are well-known
    // (small covariance) so the updates flow information into the calibration.
    ms::State<T> s(0.02);
    s.enable_calibration({ms::State<T>::CalibState{}}, /*rot_sigma=*/0.2, /*trans_sigma=*/0.2);
    for (std::size_t c = 0; c < kClones; ++c) {
        s.R = Rc[c];
        s.p = pc[c];
        ms::StateHelper<T>::augment_clone(s);
    }

    const T rot_err0 = rot_angle(s.calib[0].R_imu_cam, R_ic_true);
    const T trans_err0 = vnorm(Vec3{{s.calib[0].p_imu_cam[0] - p_ic_true[0],
                                     s.calib[0].p_imu_cam[1] - p_ic_true[1],
                                     s.calib[0].p_imu_cam[2] - p_ic_true[2]}});
    REQUIRE(rot_err0 > 0.04);  // we really did start ~2.9° off

    // Gating off (clean data, every track is a true inlier); EKF only.
    ms::CameraUpdaterOptions<T> opts;
    opts.enable_gating = false;
    const ms::CameraUpdater<T> upd(std::vector<ms::CameraExtrinsics<T>>(1), opts);

    // A few sweeps over the features: each update re-linearizes at the improving
    // estimate (the calibration problem is mildly nonlinear).
    std::size_t applied = 0;
    for (int round = 0; round < 6; ++round) {
        for (const Vec3& pf : feats) {
            ms::FeatureTrack<T> track;
            for (std::size_t c = 0; c < kClones; ++c) {
                Vec3 xy;
                if (observe(Rc[c], pc[c], R_ic_true, p_ic_true, pf, xy))
                    track.observations.push_back({c, 0, {{xy[0], xy[1]}}});
            }
            if (track.observations.size() >= 2)
                applied += upd.update(s, track) ? 1 : 0;
        }
    }
    REQUIRE(applied > 20);  // the geometry is healthy; most tracks update

    const T rot_err1 = rot_angle(s.calib[0].R_imu_cam, R_ic_true);
    const T trans_err1 = vnorm(Vec3{{s.calib[0].p_imu_cam[0] - p_ic_true[0],
                                     s.calib[0].p_imu_cam[1] - p_ic_true[1],
                                     s.calib[0].p_imu_cam[2] - p_ic_true[2]}});

    // It moved the estimate decisively toward truth, not away from it.
    REQUIRE(rot_err1 < 0.25 * rot_err0);
    REQUIRE(trans_err1 < 0.5 * trans_err0);
    // And it gained information: the calibration covariance shrank below the prior.
    const auto P = s.covariance();
    const std::size_t c0 = s.calib_offset(0);
    REQUIRE(P(c0, c0) < 0.2 * 0.2);          // rotation variance below the prior
    REQUIRE(P(c0 + 3, c0 + 3) < 0.2 * 0.2);  // translation variance below the prior
    REQUIRE(ms::is_positive_semidefinite(P));
}
