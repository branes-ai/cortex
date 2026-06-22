// SPDX-License-Identifier: MIT
//
// Gate for the real-data observability inspector core (branes/tools/
// observability_inspect.hpp, #212 / #337). The full obs_inspect tool taps a live
// EuRoC run, which CI does not vendor, so here we drive the same header math on a
// constructed clone window and pin the gauge property the diagnosis relies on:
//
//   * at a CONSISTENT linearization (H and N at the same poses) the SHIPPED camera
//     Jacobian annihilates the 4-DoF gauge -> both translation and yaw leak are
//     ~machine-eps (a regression in the production projection_jacobians would
//     break this, not just a copy of it);
//   * under a PERTURBED clone window with N held at the true gauge, the TRANSLATION
//     leak stays ~0 (structurally protected by the +/-Hf cancellation) while the
//     YAW leak grows -- the over-confidence mechanism, localized to yaw.
//
// ASCII-only TEST_CASE names (the Windows MSVC ctest job rejects non-ASCII).

#include <branes/math/lie/detail.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/tools/observability_inspect.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <random>
#include <vector>

namespace bt = branes::tools;
namespace ms = branes::sdk::msckf;
namespace ld = branes::math::lie::detail;
using Catch::Matchers::WithinAbs;

namespace {
using T = double;
using Vec3 = ld::Vec<T, 3>;
using SO3 = branes::math::lie::SO3<T>;

// A small translating clone window with mild rotation diversity + one feature at
// healthy parallax — the same shape as observability_probe::make_scene.
std::vector<bt::ObsPose<T>> window(std::size_t m) {
    std::vector<bt::ObsPose<T>> cl;
    for (std::size_t c = 0; c < m; ++c) {
        const T u = static_cast<T>(c);
        cl.push_back({SO3::exp(Vec3{{0.04 * std::sin(u), 0.03 * u - 0.1, 0.05 * std::cos(u)}}),
                      Vec3{{0.3 * u, 0.1 * std::sin(u), 0.05 * u}}});
    }
    return cl;
}
}  // namespace

TEST_CASE("observability inspector: shipped Jacobian annihilates the gauge at a consistent point",
          "[tools][obs_inspect]") {
    const ms::CameraUpdater<T> updater(std::vector<ms::CameraExtrinsics<T>>{ms::CameraExtrinsics<T>{}});
    const Vec3 g{{0.0, 0.0, 1.0}};
    const Vec3 p_f{{0.2, -0.1, 6.0}};
    const auto cl = window(5);

    const auto N = bt::obs_build_N<T>(cl, p_f, g);
    const auto [tr, yaw] = bt::obs_leak<T>(bt::obs_build_H<T>(updater, cl, p_f), N);

    // H linearized at the same poses N is built from => the gauge is annihilated.
    REQUIRE_THAT(tr, WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(yaw, WithinAbs(0.0, 1e-9));
}

TEST_CASE("observability inspector: perturbing the window leaks yaw but not translation", "[tools][obs_inspect]") {
    const ms::CameraUpdater<T> updater(std::vector<ms::CameraExtrinsics<T>>{ms::CameraExtrinsics<T>{}});
    const Vec3 g{{0.0, 0.0, 1.0}};
    const Vec3 p_f{{0.2, -0.1, 6.0}};
    const auto truth = window(5);
    const auto N = bt::obs_build_N<T>(truth, p_f, g);  // gauge at the TRUE poses

    // Perturb the clone estimates (the cross-window linearization drift), keep N at
    // the true gauge — the standard-EKF inconsistency the diagnosis measures.
    std::mt19937_64 rng(0x0B5E11ull);
    auto perturbed = [&](T s) {
        auto e = truth;
        for (auto& c : e) {
            c.R =
                c.R * SO3::exp(Vec3{{s * bt::obs_urand<T>(rng), s * bt::obs_urand<T>(rng), s * bt::obs_urand<T>(rng)}});
            c.p = Vec3{{c.p[0] + s * bt::obs_urand<T>(rng),
                        c.p[1] + s * bt::obs_urand<T>(rng),
                        c.p[2] + s * bt::obs_urand<T>(rng)}};
        }
        return e;
    };

    const auto [tr_small, yaw_small] = bt::obs_leak<T>(bt::obs_build_H<T>(updater, perturbed(0.01), p_f), N);
    const auto [tr_big, yaw_big] = bt::obs_leak<T>(bt::obs_build_H<T>(updater, perturbed(0.05), p_f), N);

    // Translation is structurally protected: the clone-dp and feature-dp columns
    // are -Hf and +Hf and cancel for ANY single H, perturbed or not.
    REQUIRE_THAT(tr_small, WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(tr_big, WithinAbs(0.0, 1e-9));
    // Yaw leaks, and grows with the perturbation — the over-confidence enters here.
    REQUIRE(yaw_small > 1e-6);
    REQUIRE(yaw_big > yaw_small);
}

TEST_CASE("observability inspector: the R-IEKF parameterization does not leak yaw under perturbation",
          "[tools][obs_inspect]") {
    const ms::CameraExtrinsics<T> ext{};  // identity extrinsic
    const Vec3 g{{0.0, 0.0, 1.0}};
    const Vec3 p_f{{0.2, -0.1, 6.0}};
    const auto truth = window(5);
    const auto Ninv = bt::obs_build_N_invariant<T>(truth, p_f, g);

    // Consistent point: the shipped invariant Jacobian annihilates the invariant gauge.
    const auto [tr0, yaw0] = bt::obs_leak<T>(bt::obs_build_H_invariant<T>(truth, p_f, ext), Ninv);
    REQUIRE_THAT(tr0, WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(yaw0, WithinAbs(0.0, 1e-9));

    // Same clone-window perturbation that makes the STANDARD yaw leak (previous
    // test). The invariant gauge directions are constants (rho = e_k, phi = g), so
    // the leak must STAY ~0 — yaw observable by construction. This is the fix.
    std::mt19937_64 rng(0x0B5E11ull);
    auto e = truth;
    for (auto& c : e) {
        c.R =
            c.R *
            SO3::exp(Vec3{{0.05 * bt::obs_urand<T>(rng), 0.05 * bt::obs_urand<T>(rng), 0.05 * bt::obs_urand<T>(rng)}});
        c.p = Vec3{{c.p[0] + 0.05 * bt::obs_urand<T>(rng),
                    c.p[1] + 0.05 * bt::obs_urand<T>(rng),
                    c.p[2] + 0.05 * bt::obs_urand<T>(rng)}};
    }
    const auto [tr_p, yaw_p] = bt::obs_leak<T>(bt::obs_build_H_invariant<T>(e, p_f, ext), Ninv);
    REQUIRE_THAT(tr_p, WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(yaw_p, WithinAbs(0.0, 1e-9));  // flat — unlike the standard filter
}
