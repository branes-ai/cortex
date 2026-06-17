// DIAGNOSTIC SCRATCH (issue #366) — reproduce the continuous-update divergence as a
// controlled unit experiment and localize the convention error in the nav↔clone
// coupling. Perfect sensors, a small initial roll error, full-window feature tracks.
// Also houses finite-difference linearization tests for Φ, G, and H.

#include <branes/math/lie/se23.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/features/msckf_nullspace.hpp>
#include <branes/sdk/msckf/invariant_propagator.hpp>
#include <branes/sdk/msckf/invariant_update.hpp>
#include <branes/sdk/msckf/invariant_vio_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <span>
#include <vector>

namespace {
namespace ms = branes::sdk::msckf;
using T = double;
using SO3 = branes::math::lie::SO3<T>;
using SE23 = branes::math::lie::SE23<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using Vec2 = branes::math::lie::detail::Vec<T, 2>;

bool visible(const SO3& R, const Vec3& p, const Vec3& pf, Vec2& xy) {
    const Vec3 y = R.inverse() * (pf - p);
    if (!(y[2] > T{0.25}))
        return false;
    const T u = y[0] / y[2], v = y[1] / y[2];
    if (u * u + v * v > T{0.49})
        return false;
    xy = Vec2{{u, v}};
    return true;
}

T deg(T r) {
    return r * T{180} / std::numbers::pi_v<T>;
}

// boxminus to the 15-dim nav error [θ;ν;ρ;bg;ba] in the filter's conventions.
std::array<T, 15>
bm_nav(const SE23& Xe, const Vec3& bge, const Vec3& bae, const SE23& Xt, const Vec3& bgt, const Vec3& bat) {
    std::array<T, 15> e{};
    const auto xi = (Xe * Xt.inverse()).log();
    for (int i = 0; i < 9; ++i)
        e[i] = xi[i];
    for (int i = 0; i < 3; ++i) {
        e[9 + i] = bge[i] - bgt[i];
        e[12 + i] = bae[i] - bat[i];
    }
    return e;
}

SE23 retract_se23(const SE23& X, const std::array<T, 15>& e) {
    SE23::Tangent xi{};
    for (int i = 0; i < 9; ++i)
        xi[i] = e[i];
    return SE23::exp(xi) * X;
}
}  // namespace

TEST_CASE("DIAG FD: nav propagation Phi under rotation", "[sdk][riekf][diag366]") {
    const Vec3 g{{0, 0, T{-981} / T{100}}};
    ms::InvariantPropagator<T> prop(ms::ImuNoise<T>{}, g);
    const Vec3 gyro{{T{1} / T{10}, T{-1} / T{20}, T{3} / T{10}}};
    const Vec3 accel{{T{4} / T{10}, T{1} / T{10}, T{98} / T{10}}};
    const T dt = T{1} / T{2000};

    ms::InvariantNavState<T> Tn;
    Tn.X = SE23(SO3::exp(Vec3{{T{2} / T{10}, T{-1} / T{10}, T{1} / T{2}}}),
                Vec3{{T{3} / T{10}, T{1} / T{10}, 0}},
                Vec3{{1, 2, T{1} / T{2}}});

    const ms::DynMat<T> Phi = prop.phi(Tn.X.rotation(), Tn.X.velocity(), Tn.X.position(), dt);
    // Φ2 = I + A*dt + ½(A*dt)²  (A*dt = Phi − I); test whether the 2nd-order term closes it.
    ms::DynMat<T> Adt(15, 15);
    for (int i = 0; i < 15; ++i)
        for (int j = 0; j < 15; ++j)
            Adt(i, j) = Phi(i, j) - (i == j ? T{1} : T{0});
    ms::DynMat<T> Phi2(15, 15);
    for (int i = 0; i < 15; ++i)
        for (int j = 0; j < 15; ++j) {
            T s = (i == j ? T{1} : T{0}) + Adt(i, j);
            for (int k2 = 0; k2 < 15; ++k2)
                s += T{1} / T{2} * Adt(i, k2) * Adt(k2, j);
            Phi2(i, j) = s;
        }

    const T eps = T{1} / T{1000000};
    std::putchar(10);
    std::puts("[diag366-Phi] dir  max_abs_FD_minus_linear / eps");
    T worst = 0;
    for (int k = 0; k < 15; ++k) {
        std::array<T, 15> e{};
        e[k] = eps;
        ms::InvariantNavState<T> est;
        est.X = retract_se23(Tn.X, e);
        est.bg = Vec3{{e[9], e[10], e[11]}};
        est.ba = Vec3{{e[12], e[13], e[14]}};
        ms::InvariantNavState<T> tp = Tn, ep = est;
        prop.propagate_mean(tp, gyro, accel, dt);
        prop.propagate_mean(ep, gyro, accel, dt);
        const auto e_after = bm_nav(ep.X, ep.bg, ep.ba, tp.X, tp.bg, tp.ba);
        T mx = 0, mx2 = 0;
        for (int i = 0; i < 15; ++i) {
            mx = std::max(mx, std::abs(e_after[i] - Phi(i, k) * eps));
            mx2 = std::max(mx2, std::abs(e_after[i] - Phi2(i, k) * eps));
        }
        worst = std::max(worst, mx / eps);
        std::printf("[diag366-Phi] %2d  Euler=%12.4e  2nd-order=%12.4e", k, mx / eps, mx2 / eps);
        std::putchar(10);
        if (k == 9 || k == 10 || k == 11) {
            std::printf("[diag366-Phi]   dir%d component breakdown (FD/eps  vs  phi col):", k);
            std::putchar(10);
            for (int i = 0; i < 15; ++i)
                if (std::abs(e_after[i] / eps - Phi(i, k)) > 1e-6) {
                    std::printf("[diag366-Phi]     i=%2d  FD=%12.4e  phi=%12.4e  diff=%12.4e",
                                i,
                                e_after[i] / eps,
                                Phi(i, k),
                                e_after[i] / eps - Phi(i, k));
                    std::putchar(10);
                }
        }
    }
    REQUIRE(worst < 1e-4);
}

TEST_CASE("DIAG FD: augmentation G under rotation", "[sdk][riekf][diag366]") {
    // clone copies nav pose; clone error (φ_c, ρ_c) vs G's claim (θ_nav, ρ_nav).
    SE23 Xt(SO3::exp(Vec3{{T{2} / T{10}, T{-1} / T{10}, T{1} / T{2}}}),
            Vec3{{T{3} / T{10}, T{1} / T{10}, 0}},
            Vec3{{1, 2, T{1} / T{2}}});
    const T eps = T{1} / T{1000000};
    std::putchar(10);
    std::puts("[diag366-G] dir  (phi_c-theta, rho_c-rho) residual / eps");
    T worst = 0;
    for (int k = 0; k < 9; ++k) {
        std::array<T, 15> e{};
        e[k] = eps;
        const SE23 Xe = retract_se23(Xt, e);
        // clone box-minus in the left-invariant SE(3) clone convention.
        const Vec3 phi_c = (Xe.rotation() * Xt.rotation().inverse()).log();
        const SO3 dR = SO3::exp(phi_c);
        const Vec3 rho_c{{Xe.position()[0] - (dR * Xt.position())[0],
                          Xe.position()[1] - (dR * Xt.position())[1],
                          Xe.position()[2] - (dR * Xt.position())[2]}};
        T mx = 0;
        for (int i = 0; i < 3; ++i) {
            mx = std::max(mx, std::abs(phi_c[i] - e[i]));      // G: φ_c = θ
            mx = std::max(mx, std::abs(rho_c[i] - e[6 + i]));  // G: ρ_c = ρ
        }
        worst = std::max(worst, mx / eps);
        std::printf("[diag366-G] %2d  %12.4e", k, mx / eps);
        std::putchar(10);
    }
    REQUIRE(worst < 1e-6);
}

TEST_CASE("DIAG FD: measurement-update linearization (proj.r vs -H*e)", "[sdk][riekf][diag366]") {
    // True clone window (rotated, with baseline) and a feature. Perturb each clone by a
    // known left-invariant error e; check the projected residual ≈ -proj.H*e.
    std::vector<ms::InvariantClone<T>> truth, est;
    for (int c = 0; c < 4; ++c) {
        SO3 R = SO3::exp(Vec3{{T(0.05) * c, T(-0.03) * c, T(0.2) * c}});
        Vec3 p{{T(0.3) * c, T(0.1) * c, T(0.02) * c}};
        truth.push_back({R, p});
    }
    const Vec3 pf{{T(0.4), T(-0.2), T(3.5)}};

    // True normalized observations.
    std::vector<ms::InvariantObs<T>> obs;
    for (std::size_t c = 0; c < truth.size(); ++c) {
        const Vec3 y =
            truth[c].R.inverse() * Vec3{{pf[0] - truth[c].p[0], pf[1] - truth[c].p[1], pf[2] - truth[c].p[2]}};
        obs.push_back({c, 0, Vec2{{y[0] / y[2], y[1] / y[2]}}});
    }

    // Known per-clone error e (6 each), applied via the clone retraction.
    const T s = T{1} / T{100000};
    std::vector<T> e(6 * truth.size());
    est = truth;
    for (std::size_t c = 0; c < est.size(); ++c) {
        for (int k = 0; k < 6; ++k)
            e[6 * c + k] = s * ((k + 1) * (c % 2 ? -1 : 1) + T(0.3) * c);
        ms::retract_invariant<T>(
            est[c], Vec3{{e[6 * c], e[6 * c + 1], e[6 * c + 2]}}, Vec3{{e[6 * c + 3], e[6 * c + 4], e[6 * c + 5]}});
    }

    const auto tri = ms::triangulate_invariant<T>(est, obs);
    REQUIRE(tri.ok);
    const auto M = ms::build_invariant_measurement<T>(est, obs, tri.p_f);
    REQUIRE(M.ok);
    const std::size_t n = 6 * est.size();
    const auto proj = branes::sdk::features::msckf_left_nullspace_project<T>(M.H_f, M.H_x, M.r, M.rows2, n);
    REQUIRE(proj.rows > 0);

    // proj.r ≈ -proj.H*e
    T worst = 0, rmag = 0;
    for (std::size_t i = 0; i < proj.rows; ++i) {
        T He = 0;
        for (std::size_t j = 0; j < n; ++j)
            He += proj.H_x[i * n + j] * e[j];
        worst = std::max(worst, std::abs(proj.r[i] + He));
        rmag = std::max(rmag, std::abs(proj.r[i]));
    }
    REQUIRE(rmag > 1e-9);
    REQUIRE(worst / rmag < 1e-4);
}

TEST_CASE("DIAG FD: measurement-update linearization with extrinsics (proj.r vs -H*e)", "[sdk][riekf][diag366]") {
    // True clone window and calibration. Perturb both clones and extrinsics by a
    // known left-invariant error e; check the projected residual ≈ -proj.H*e.
    std::vector<ms::InvariantClone<T>> truth, est;
    for (int c = 0; c < 4; ++c) {
        SO3 R = SO3::exp(Vec3{{T(0.05) * c, T(-0.03) * c, T(0.2) * c}});
        Vec3 p{{T(0.3) * c, T(0.1) * c, T(0.02) * c}};
        truth.push_back({R, p});
    }
    const Vec3 pf{{T(0.4), T(-0.2), T(3.5)}};

    const ms::InvariantCalib<T> calib_truth{SO3::exp(Vec3{{0.1, -0.2, 0.15}}), Vec3{{0.05, -0.02, 0.03}}};
    std::vector<ms::InvariantCalib<T>> calibs_truth{calib_truth};

    // True normalized observations using the true extrinsics.
    std::vector<ms::InvariantObs<T>> obs;
    for (std::size_t c = 0; c < truth.size(); ++c) {
        const auto R_cam = truth[c].R * calib_truth.R_imu_cam;
        const auto p_cam = truth[c].p + truth[c].R * calib_truth.p_imu_cam;
        const Vec3 y = R_cam.inverse() * (pf - p_cam);
        obs.push_back({c, 0, Vec2{{y[0] / y[2], y[1] / y[2]}}});
    }

    // Joint error e (6 * clones + 6 * calibs)
    const T s = T{1} / T{1000000};
    const std::size_t n = 6 * truth.size() + 6 * calibs_truth.size();
    std::vector<T> e(n);

    // Perturb clones
    est = truth;
    for (std::size_t c = 0; c < est.size(); ++c) {
        for (int k = 0; k < 6; ++k)
            e[6 * c + k] = s * ((k + 1) * (c % 2 ? -1 : 1) + T(0.3) * c);
        ms::retract_invariant<T>(
            est[c], Vec3{{e[6 * c], e[6 * c + 1], e[6 * c + 2]}}, Vec3{{e[6 * c + 3], e[6 * c + 4], e[6 * c + 5]}});
    }

    // Perturb calibration
    std::vector<ms::InvariantCalib<T>> calibs_est = calibs_truth;
    for (int k = 0; k < 6; ++k)
        e[6 * est.size() + k] = s * (k + 1) * T(0.5);

    const Vec3 dtheta{{e[6 * est.size()], e[6 * est.size() + 1], e[6 * est.size() + 2]}};
    const Vec3 dp{{e[6 * est.size() + 3], e[6 * est.size() + 4], e[6 * est.size() + 5]}};
    calibs_est[0].R_imu_cam = calibs_est[0].R_imu_cam * SO3::exp(dtheta);
    calibs_est[0].p_imu_cam = calibs_est[0].p_imu_cam + dp;

    const auto tri = ms::triangulate_invariant<T>(est, obs, calibs_est);
    REQUIRE(tri.ok);
    const auto M = ms::build_invariant_measurement<T>(est, obs, tri.p_f, calibs_est, true);
    REQUIRE(M.ok);
    const auto proj = branes::sdk::features::msckf_left_nullspace_project<T>(M.H_f, M.H_x, M.r, M.rows2, n);
    REQUIRE(proj.rows > 0);

    // proj.r ≈ -proj.H*e
    T worst = 0, rmag = 0;
    for (std::size_t i = 0; i < proj.rows; ++i) {
        T He = 0;
        for (std::size_t j = 0; j < n; ++j)
            He += proj.H_x[i * n + j] * e[j];
        worst = std::max(worst, std::abs(proj.r[i] + He));
        rmag = std::max(rmag, std::abs(proj.r[i]));
    }
    REQUIRE(rmag > 1e-9);
    REQUIRE(worst / rmag < 1e-4);
}

TEST_CASE("DIAG invariant joint step: perfect-sensor roll-error reproduction (adapter)", "[sdk][riekf][diag366]") {
    const Vec3 g{{0, 0, T{-981} / T{100}}};

    // Landmarks overhead (identity-extrinsic camera = body frame, looks up); the loop
    // is gentle enough that they stay in the FOV across the window.
    std::vector<Vec3> feats;
    for (int i = 0; i < 60; ++i)
        feats.push_back(Vec3{{T(-1) + T(0.3) * (i % 7), T(-1) + T(0.3) * (i / 7), T(3) + T(0.25) * (i % 5)}});

    ms::InvariantVioBackend<T>::Config cfg;
    cfg.max_clones = 11;
    cfg.backend.initial_sigma = T{1} / T{20};
    cfg.backend.normalized_sigma = T{1} / T{500};
    cfg.backend.noise = ms::ImuNoise<T>{T{1} / T{2000}, T{1} / T{200}, T{1} / T{50000}, T{1} / T{3000}};
    cfg.backend.enable_gating = false;
    ms::InvariantVioBackend<T> be(cfg);

    const T roll = T{1} / T{10};  // 5.7 deg initial roll error; everything else exact
    be.set_nav(SE23(SO3::exp(Vec3{{roll, 0, 0}}), Vec3{}, Vec3{}), Vec3{}, Vec3{}, 0.0);

    const T dt = T{1} / T{200};
    const int imu_per_frame = 10;
    ms::InvariantPropagator<T> truth_prop(cfg.backend.noise, g);
    ms::InvariantNavState<T> truth_nav;
    truth_nav.X = SE23(SO3{}, Vec3{}, Vec3{});
    const Vec3 gyro_in{{0, 0, T{1} / T{5}}};                  // gentle yaw
    const Vec3 accel_in{{T{2} / T{10}, 0, T{981} / T{100}}};  // gentle curved translation

    T sum_att = 0, sum_pos = 0, last_att = 0, last_pos = 0;
    int nsamp = 0;
    for (int step = 0; step < 800; ++step) {
        const double t = (step + 1) * dt;
        be.process_imu(gyro_in, accel_in, t);
        truth_prop.propagate_mean(truth_nav, gyro_in, accel_in, dt);
        if ((step + 1) % imu_per_frame != 0)
            continue;
        const SO3 Rt = truth_nav.X.rotation();
        const Vec3 pt = truth_nav.X.position();
        std::vector<ms::NormalizedObs<T>> obs;
        for (std::size_t L = 0; L < feats.size(); ++L) {
            Vec2 xy;
            if (visible(Rt, pt, feats[L], xy))
                obs.push_back({static_cast<std::uint64_t>(L), 0, xy});
        }
        be.process_camera(t, std::span<const ms::NormalizedObs<T>>{obs});

        const Vec3 att = (be.nav().X.rotation() * Rt.inverse()).log();
        last_att = deg(std::sqrt(att[0] * att[0] + att[1] * att[1] + att[2] * att[2]));
        const Vec3 ep = be.nav().X.position();
        last_pos = std::sqrt((ep[0] - pt[0]) * (ep[0] - pt[0]) + (ep[1] - pt[1]) * (ep[1] - pt[1]) +
                             (ep[2] - pt[2]) * (ep[2] - pt[2]));
        if (t > 3.0) {
            sum_att += last_att;
            sum_pos += last_pos;
            ++nsamp;
        }
        if ((step + 1) % 80 == 0) {
            const Vec3 bg = be.nav().bg, ba = be.nav().ba;
            std::printf("[diag366-bias] t=%.2f att=%.3f deg  bg=(%.4f %.4f %.4f) ba=(%.4f %.4f %.4f)",
                        t,
                        last_att,
                        bg[0],
                        bg[1],
                        bg[2],
                        ba[0],
                        ba[1],
                        ba[2]);
            std::putchar(10);
        }
    }
    REQUIRE(nsamp > 0);
    REQUIRE(sum_att / nsamp < 3.0);
    REQUIRE(sum_pos / nsamp < 2.0);
}
