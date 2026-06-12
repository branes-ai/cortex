// First-Estimates Jacobians (FEJ) mechanism test (issue #339).
//
// FEJ fixes the structural NEES over-confidence (#212) by evaluating the
// measurement Jacobians at each clone's FROZEN first-estimate pose while still
// computing the residual at the current estimate — keeping the window's
// linearization points consistent so the unobservable yaw null space is
// preserved (the leak the #337 observability probe measures). These tests lock
// the two halves of that contract:
//   (1) the first estimate is captured at augmentation and never moved by the
//       EKF update;
//   (2) with use_fej, the Jacobian is pinned to R_fej/p_fej (unchanged when only
//       the current pose drifts) while the residual h tracks the current pose;
//       with use_fej off, the Jacobian follows the current pose (the leak path).

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

T rot_angle(const SO3& a, const SO3& b) {
    return branes::math::lie::detail::norm((a.inverse() * b).log());
}
T vdiff(const Vec3& a, const Vec3& b) {
    return branes::math::lie::detail::norm(Vec3{{a[0] - b[0], a[1] - b[1], a[2] - b[2]}});
}
// Max-abs difference of two 2×3 Jacobian blocks.
template <class M>
T mdiff(const M& a, const M& b) {
    T m = 0;
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            m = std::max(m, std::abs(a(i, j) - b(i, j)));
    return m;
}
}  // namespace

TEST_CASE("FEJ: augmentation captures the first estimate; the EKF update never moves it", "[sdk][fej][s6]") {
    ms::State<T> s(T{1} / T{10});
    s.R = SO3::exp(Vec3{{0.1, -0.05, 0.2}});
    s.p = Vec3{{0.5, -0.3, 0.2}};
    ms::StateHelper<T>::augment_clone(s);
    // At augmentation the frozen pose equals the current pose.
    REQUIRE(rot_angle(s.clones[0].R, s.clones[0].R_fej) < 1e-15);
    REQUIRE(vdiff(s.clones[0].p, s.clones[0].p_fej) < 1e-15);

    const SO3 R_fej0 = s.clones[0].R_fej;
    const Vec3 p_fej0 = s.clones[0].p_fej;

    // A real EKF update touching the clone. It must move the current R/p but
    // leave the frozen first estimate untouched.
    ms::Propagator<T> prop;
    for (int k = 0; k < 40; ++k)
        prop.propagate(s, Vec3{{0.01, 0, 0}}, Vec3{{0, 0, 9.81}}, 0.01);
    const std::size_t d = s.dim();
    ms::DynMat<T> H(2, d);
    for (std::size_t j = 0; j < d; ++j) {
        H(0, j) = std::sin(0.4 * j + 0.1);
        H(1, j) = std::cos(0.3 * j);
    }
    const std::vector<T> r = {0.03, -0.02};
    const std::vector<T> Rd = {0.01, 0.01};
    ms::StateHelper<T>::ekf_update(s, H, std::span<const T>{r}, std::span<const T>{Rd});

    REQUIRE(rot_angle(s.clones[0].R_fej, R_fej0) < 1e-15);  // frozen
    REQUIRE(vdiff(s.clones[0].p_fej, p_fej0) < 1e-15);
}

TEST_CASE("FEJ: the Jacobian is pinned to the first estimate; the residual tracks the current pose", "[sdk][fej][s6]") {
    const Vec3 p_f{{0.5, 0.2, 5.0}};
    const ms::CameraObservation<T> obs{0, 0, {{0.0, 0.0}}};

    // One clone at a true pose, with frozen == current.
    auto make = [&] {
        ms::State<T> s(T{1} / T{10});
        s.R = SO3::exp(Vec3{{0.1, 0.05, -0.08}});
        s.p = Vec3{{0.3, -0.1, 0.2}};
        ms::StateHelper<T>::augment_clone(s);  // R_fej = R, p_fej = p
        return s;
    };

    ms::CameraUpdaterOptions<T> fej_opts;
    fej_opts.use_fej = true;
    const ms::CameraUpdater<T> upd_fej(std::vector<ms::CameraExtrinsics<T>>(1), fej_opts);
    const ms::CameraUpdater<T> upd_std(std::vector<ms::CameraExtrinsics<T>>(1));  // use_fej = false

    const ms::State<T> s0 = make();
    const auto base = upd_fej.prediction(s0, obs, p_f);
    REQUIRE(base.valid);

    // Drift the CURRENT pose only; leave the frozen first estimate where it was.
    ms::State<T> s = make();
    s.clones[0].R = s.clones[0].R * SO3::exp(Vec3{{0.06, -0.04, 0.05}});
    s.clones[0].p = Vec3{{s.clones[0].p[0] + 0.1, s.clones[0].p[1] - 0.05, s.clones[0].p[2] + 0.08}};

    const auto pf = upd_fej.prediction(s, obs, p_f);  // FEJ
    const auto ps = upd_std.prediction(s, obs, p_f);  // current-estimate linearization
    REQUIRE(pf.valid);
    REQUIRE(ps.valid);

    // FEJ: Jacobians are pinned to the frozen pose ⇒ unchanged from baseline…
    REQUIRE(mdiff(pf.Hf, base.Hf) < 1e-12);
    REQUIRE(mdiff(pf.Htheta, base.Htheta) < 1e-12);
    // …but the residual prediction h moved with the current pose.
    REQUIRE(vdiff(Vec3{{pf.h[0], pf.h[1], 0}}, Vec3{{base.h[0], base.h[1], 0}}) > 1e-3);

    // No FEJ: the Jacobian followed the drifted current pose (the leak path).
    REQUIRE(mdiff(ps.Hf, base.Hf) > 1e-3);
    REQUIRE(mdiff(ps.Htheta, base.Htheta) > 1e-3);

    // Both predict the SAME h (residual is always at the current pose).
    REQUIRE(vdiff(Vec3{{pf.h[0], pf.h[1], 0}}, Vec3{{ps.h[0], ps.h[1], 0}}) < 1e-12);
}
