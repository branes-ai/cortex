// R-IEKF MSCKF backend (issue #347, Phase B) — the assembled invariant filter:
// SE₂(3) state + invariant propagation + invariant camera update over one joint
// right-invariant covariance. Locks three properties of the wired backend:
//   (1) end-to-end recovery — an observable clone error is driven out;
//   (2) propagation + update runs and keeps the covariance PSD;
//   (3) the decisive one — the assembled, feature-marginalized joint update
//       annihilates the 4-DoF gauge across the LIVE NAV and ALL clones
//       (H_proj·N = 0), so a camera update injects no information along the
//       unobservable yaw/translation manifold. The #212 fix at the filter level.

#include <branes/sdk/msckf/msckf_invariant_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {
namespace ms = branes::sdk::msckf;
namespace det = branes::math::lie::detail;
using T = double;
using Vec3 = det::Vec<T, 3>;
using Vec2 = det::Vec<T, 2>;
using SO3 = branes::math::lie::SO3<T>;
using SE23 = branes::math::lie::SE23<T>;
using Backend = ms::MsckfInvariantBackend<T>;
using DynMat = ms::DynMat<T>;

struct Pose {
    SO3 R;
    Vec3 p;
};
std::vector<Pose> true_poses() {
    std::vector<Pose> v;
    for (int c = 0; c < 5; ++c)
        v.push_back({SO3::exp(Vec3{{0.02 * c, -0.01 * c, 0.015 * c}}), Vec3{{0.25 * c, 0.0, 0.0}}});
    return v;
}
std::vector<Vec3> features() {
    std::vector<Vec3> f;
    for (int i = 0; i < 16; ++i)
        f.push_back(Vec3{{static_cast<T>((i % 4) - 2) * T{0.4}, static_cast<T>((i / 4) - 2) * T{0.4}, 3.0 + 0.1 * i}});
    return f;
}
Vec2 project(const Pose& c, const Vec3& pf) {
    const Vec3 y = c.R.inverse().matrix() * (pf - c.p);
    return Vec2{{y[0] / y[2], y[1] / y[2]}};
}
// A track over ALL clones, observed from the TRUE poses.
ms::InvariantTrack<T> track(const std::vector<Pose>& poses, const Vec3& pf) {
    ms::InvariantTrack<T> obs;
    for (std::size_t c = 0; c < poses.size(); ++c)
        obs.push_back({c, project(poses[c], pf)});
    return obs;
}
// Gauge-invariant inconsistency of the backend's clones vs the true observations.
T inconsistency(const Backend& b, const std::vector<Pose>& truth, const std::vector<Vec3>& feats) {
    std::vector<ms::InvariantClone<T>> cl(b.clones());
    T cost = T{0};
    for (const auto& pf0 : feats) {
        const auto obs = track(truth, pf0);
        const auto tri = ms::triangulate_invariant<T>(cl, obs);
        REQUIRE(tri.ok);
        for (const auto& o : obs) {
            const Pose cp{cl[o.clone_index].R, cl[o.clone_index].p};
            const Vec2 rr = project(cp, tri.p_f);
            cost += (rr[0] - o.xy[0]) * (rr[0] - o.xy[0]) + (rr[1] - o.xy[1]) * (rr[1] - o.xy[1]);
        }
    }
    return cost;
}
}  // namespace

TEST_CASE("invariant backend: an observable clone error is driven out end-to-end", "[sdk][riekf][backend]") {
    const auto truth = true_poses();
    const auto feats = features();
    Backend::Config cfg;
    cfg.initial_sigma = 0.5;
    cfg.max_clones = 11;
    cfg.noise = ms::ImuNoise<T>{0.1, 0.2, 1e-4, 1e-4};  // enough to decorrelate clones
    Backend b(cfg);

    // Build the window: clone each true pose, but seed clone 2 from a WRONG nav so
    // the window carries a purely observable error (the others anchor the gauge).
    // Propagate a few IMU steps BEFORE fixing each pose so the nav covariance
    // inflates and successive clones are decorrelated (a window of identical copies
    // of one never-propagated nav block has certain relative geometry — nothing to
    // correct). set_nav then pins the mean to the exact pose; the inflation stays.
    for (std::size_t i = 0; i < truth.size(); ++i) {
        for (int k = 0; k < 8; ++k)
            b.propagate(Vec3{{0.01, -0.01, 0.02}}, Vec3{{0.05, 0.0, 9.81}}, 0.01);
        Pose pose = truth[i];
        if (i == 2) {
            pose.R = SO3::exp(Vec3{{0.03, -0.02, 0.025}}) * pose.R;
            pose.p = pose.p + Vec3{{0.04, -0.03, 0.02}};
        }
        b.set_nav(SE23(pose.R, Vec3{}, pose.p), Vec3{}, Vec3{});
        b.augment_clone();
    }
    const T pre = inconsistency(b, truth, feats);
    REQUIRE(pre > 1e-3);

    for (const auto& pf : feats)
        b.update(track(truth, pf));

    REQUIRE(inconsistency(b, truth, feats) < 0.02 * pre);  // observable error removed
    REQUIRE(ms::is_positive_semidefinite(b.covariance()));
}

TEST_CASE("invariant backend: propagate + augment + update runs and stays PSD", "[sdk][riekf][backend]") {
    Backend b;
    b.set_nav(SE23(SO3::exp(Vec3{{0.05, 0.0, 0.1}}), Vec3{{0.2, 0.0, 0.0}}, Vec3{}), Vec3{}, Vec3{});
    const auto truth = true_poses();
    const auto feats = features();
    for (int k = 0; k < 20; ++k)
        b.propagate(Vec3{{0.01, -0.01, 0.02}}, Vec3{{0.05, 0.0, 9.81}}, 0.01);
    b.set_nav(SE23(truth[0].R, Vec3{}, truth[0].p), Vec3{}, Vec3{});
    b.augment_clone();
    b.set_nav(SE23(truth[1].R, Vec3{}, truth[1].p), Vec3{}, Vec3{});
    b.augment_clone();
    const bool applied = b.update(track({truth[0], truth[1]}, feats[5]));
    REQUIRE(applied);
    REQUIRE(b.num_clones() == 2);
    REQUIRE(ms::is_positive_semidefinite(b.covariance()));
}

TEST_CASE("invariant backend: a degenerate (zero-baseline) track is rejected without side effects",
          "[sdk][riekf][backend]") {
    Backend b;
    // Two clones at the SAME pose ⇒ zero baseline ⇒ degenerate triangulation.
    const Pose pose = true_poses()[0];
    b.set_nav(SE23(pose.R, Vec3{}, pose.p), Vec3{}, Vec3{});
    b.augment_clone();
    b.set_nav(SE23(pose.R, Vec3{}, pose.p), Vec3{}, Vec3{});
    b.augment_clone();
    const DynMat cov_before = b.covariance();

    const Vec2 obs = project(pose, features()[6]);
    const bool applied = b.update(ms::InvariantTrack<T>{{0, obs}, {1, obs}});
    REQUIRE_FALSE(applied);  // low-parallax track rejected
    REQUIRE(b.num_clones() == 2);
    // Covariance untouched.
    const DynMat cov_after = b.covariance();
    T diff = T{0};
    for (std::size_t i = 0; i < cov_before.rows; ++i)
        for (std::size_t j = 0; j < cov_before.cols; ++j)
            diff += std::abs(cov_after(i, j) - cov_before(i, j));
    REQUIRE(diff < 1e-15);
}

TEST_CASE("invariant backend: the marginalized joint update annihilates the 4-DoF gauge",
          "[sdk][riekf][backend][observability]") {
    const auto truth = true_poses();
    const auto feats = features();
    Backend b;
    for (const auto& pose : truth) {
        b.set_nav(SE23(pose.R, Vec3{}, pose.p), Vec3{}, Vec3{});
        b.augment_clone();
    }
    const std::size_t d = b.dim();
    const std::size_t nc = b.num_clones();
    std::vector<ms::InvariantClone<T>> cl(b.clones());

    // The global gauge over the JOINT state [nav(15) ⊕ 6·#clones]:
    //   yaw  — nav δθ = a and every clone δθ_c = a (a = vertical/gravity axis);
    //   trans— nav δp = e_k and every clone δρ_c = e_k.
    const Vec3 a{{0.0, 0.0, 1.0}};
    auto gauge = [&](int g) {
        std::vector<T> N(d, T{0});
        if (g < 3) {
            N[Backend::Nav::kPos + g] = T{1};  // nav δp
            for (std::size_t c = 0; c < nc; ++c)
                N[Backend::Nav::kDim + 6 * c + 3 + g] = T{1};  // clone δρ_c
        } else {
            for (int k = 0; k < 3; ++k)
                N[Backend::Nav::kTheta + k] = a[k];  // nav δθ
            for (std::size_t c = 0; c < nc; ++c)
                for (int k = 0; k < 3; ++k)
                    N[Backend::Nav::kDim + 6 * c + k] = a[k];  // clone δθ_c
        }
        return N;
    };

    // Reconstruct the marginalized joint Jacobian exactly as update() does, then
    // confirm H_proj annihilates all four gauge directions.
    auto worst_leak = [&](const Vec3& pf) {
        const auto obs = track(truth, pf);
        const auto tri = ms::triangulate_invariant<T>(cl, obs);
        REQUIRE(tri.ok);
        const auto M = ms::build_invariant_measurement<T>(cl, obs, tri.p_f);
        REQUIRE(M.ok);
        const std::size_t nc6 = 6 * nc;
        std::vector<T> Hj(M.rows2 * d, T{0});
        for (std::size_t row = 0; row < M.rows2; ++row)
            for (std::size_t c = 0; c < nc; ++c)
                for (std::size_t k = 0; k < 6; ++k)
                    Hj[row * d + Backend::Nav::kDim + 6 * c + k] = M.H_x[row * nc6 + 6 * c + k];
        const auto proj = branes::sdk::features::msckf_left_nullspace_project<T>(M.H_f, Hj, M.r, M.rows2, d);
        REQUIRE(proj.rows > 0);
        T worst = T{0};
        for (int g = 0; g < 4; ++g) {
            const std::vector<T> N = gauge(g);
            for (std::size_t i = 0; i < proj.rows; ++i) {
                T hv = T{0};
                for (std::size_t j = 0; j < d; ++j)
                    hv += proj.H_x[i * d + j] * N[j];
                worst = std::max(worst, std::abs(hv));
            }
        }
        return worst;
    };

    REQUIRE(worst_leak(feats[0]) < 1e-10);
    REQUIRE(worst_leak(feats[8]) < 1e-10);
    REQUIRE(worst_leak(feats[15]) < 1e-10);
}
