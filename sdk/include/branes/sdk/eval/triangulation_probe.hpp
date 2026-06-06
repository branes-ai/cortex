// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/triangulation_probe.hpp — the S5 (feature triangulation) probe
// of the VIO contract program (docs/arch/vio-pipeline-canonical.md).
//
// S5's pre-condition is "sufficiently-parallax" observations; its post-condition
// bounds the triangulation normal-matrix condition number. The #212-relevant
// question is whether the cortex pipeline gates low-parallax features before
// admitting them to the update at full weight — admitting a feature triangulated
// at 0.2° as if it were precise injects optimistic information, a plausible
// contributor to the aggressive-motion (V2_03) divergence where parallax is erratic.
//
// This probe drives the SHIPPED triangulator (CameraUpdater::triangulate) on a
// synthetic two-view geometry whose parallax we sweep exactly, and measures, per
// parallax level:
//   • whether the shipped triangulate() reports success (its only guard is the
//     Cholesky breakdown at near-parallel rays — a hard degeneracy, not a soft gate);
//   • the linear system's condition number (the geometric conditioner of depth);
//   • the recovered-depth error with clean observations (correctness);
//   • the recovered-depth σ under 1 px observation noise (Monte Carlo — the TRUE
//     uncertainty a low-parallax feature carries);
//   • the reprojection RMS at the solution.
//
// Native units: degrees (parallax), millimetres (depth error/σ), pixels
// (reprojection), dimensionless (condition number). Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_TRIANGULATION_PROBE_HPP
#define BRANES_SDK_EVAL_TRIANGULATION_PROBE_HPP

#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace branes::sdk::eval {

namespace triang_detail {
using std::atan;
using std::sqrt;
using std::tan;

// EuRoC cam0 focal (px) — converts the 1 px observation noise to normalized units.
template <class T>
inline constexpr T kFocal = T{458654} / T{1000};

// Symmetric 3×3 eigenvalues via cyclic Jacobi — for the condition number of the
// triangulation normal matrix. Returns the three eigenvalues (unordered).
template <class T>
std::array<T, 3> sym3_eigs(std::array<std::array<T, 3>, 3> a) {
    for (int sweep = 0; sweep < 24; ++sweep) {
        int p = 0, q = 1;
        T off = std::abs(a[0][1]);
        if (std::abs(a[0][2]) > off) {
            off = std::abs(a[0][2]);
            p = 0;
            q = 2;
        }
        if (std::abs(a[1][2]) > off) {
            off = std::abs(a[1][2]);
            p = 1;
            q = 2;
        }
        if (off < T{1e-18})
            break;
        const T phi = T{0.5} * std::atan2(T{2} * a[p][q], a[q][q] - a[p][p]);
        const T c = std::cos(phi), s = std::sin(phi);
        for (int k = 0; k < 3; ++k) {
            const T akp = a[k][p], akq = a[k][q];
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {
            const T apk = a[p][k], aqk = a[q][k];
            a[p][k] = c * apk - s * aqk;
            a[q][k] = s * apk + c * aqk;
        }
    }
    return {a[0][0], a[1][1], a[2][2]};
}
}  // namespace triang_detail

/// One parallax level's measured triangulation health.
template <math::Scalar T>
struct ParallaxPoint {
    T parallax_deg = 0;
    bool status_ok = false;       ///< shipped triangulate() returned true (clean obs)
    T condition_number = 0;       ///< cond(A) of the linear triangulation system
    T depth_error_mm = 0;         ///< |recovered − true| depth, clean observations
    T depth_sigma_mm = 0;         ///< std of recovered depth under 1 px noise (Monte Carlo)
    T reproj_rms_px = 0;          ///< reprojection RMS at the clean solution
    std::size_t mc_failures = 0;  ///< noisy trials where triangulate() failed
};

template <math::Scalar T>
struct ParallaxSweep {
    std::vector<ParallaxPoint<T>> curve;
    T depth_m = 0;
    T px_noise = 0;
    T gate_parallax_deg = 0;  ///< smallest swept parallax where depth σ ≤ 5% of depth (+∞ if none)
};

/// Sweep two-view parallax exactly and measure the shipped triangulator. The two
/// cameras look down +z, offset along x by the baseline that yields each target
/// parallax for a feature at `depth_m`. Identity extrinsics (camera == IMU).
template <math::Scalar T>
[[nodiscard]] ParallaxSweep<T> triangulation_parallax_sweep(T depth_m = T{5}, T px_noise = T{1}, std::size_t mc = 400) {
    using namespace triang_detail;
    namespace ms = branes::sdk::msckf;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using SO3 = math::lie::SO3<T>;
    using Obs = ms::CameraObservation<T>;

    const ms::CameraUpdater<T> upd(std::vector<ms::CameraExtrinsics<T>>(1));  // identity extrinsics
    const Vec3 F{{T{0}, T{0}, depth_m}};
    const T sigma_norm = px_noise / kFocal<T>;
    std::mt19937_64 rng(0xBA5EBA11ull);
    std::normal_distribution<double> N01(0.0, 1.0);

    ParallaxSweep<T> out;
    out.depth_m = depth_m;
    out.px_noise = px_noise;

    const T degs[] = {T{0.1}, T{0.25}, T{0.5}, T{1}, T{2}, T{5}, T{10}};
    for (const T deg : degs) {
        const T theta = deg * T{3.14159265358979323846} / T{180};
        const T half_b = depth_m * std::tan(theta / T{2});

        // Two clones offset along x; both world-aligned (look +z).
        ms::State<T> s(T{1});
        s.clones.push_back({SO3{}, Vec3{{-half_b, T{0}, T{0}}}, 0.0});
        s.clones.push_back({SO3{}, Vec3{{half_b, T{0}, T{0}}}, 1.0});

        auto project_norm = [&](std::size_t i, const Vec3& p) {
            const auto& cl = s.clones[i];
            const Vec3 pc = cl.R.inverse() * (p - cl.p);
            return std::array<T, 2>{pc[0] / pc[2], pc[1] / pc[2]};
        };
        const auto xy0 = project_norm(0, F), xy1 = project_norm(1, F);

        ParallaxPoint<T> pt;
        pt.parallax_deg = deg;

        // Condition number of A = Σ(I − d̂ d̂ᵀ) over the clean bearings.
        std::array<std::array<T, 3>, 3> A{};
        for (const auto& xy : {xy0, xy1}) {
            Vec3 d{{xy[0], xy[1], T{1}}};
            const T dn = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            d = d * (T{1} / dn);
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b)
                    A[a][b] += (a == b ? T{1} : T{0}) - d[a] * d[b];
        }
        auto eigs = sym3_eigs<T>(A);
        T lmax = eigs[0], lmin = eigs[0];
        for (T e : eigs) {
            lmax = std::max(lmax, e);
            lmin = std::min(lmin, e);
        }
        pt.condition_number = lmin > T{0} ? lmax / lmin : std::numeric_limits<T>::infinity();

        // Clean triangulation: correctness + reprojection.
        std::vector<Obs> clean{{0, 0, {xy0[0], xy0[1]}}, {1, 0, {xy1[0], xy1[1]}}};
        Vec3 pf{};
        pt.status_ok = upd.triangulate(s, clean, pf);
        if (pt.status_ok) {
            pt.depth_error_mm = std::abs(pf[2] - depth_m) * T{1000};
            T sse = 0;
            for (std::size_t i = 0; i < 2; ++i) {
                const auto rp = project_norm(i, pf);
                const auto& xy = i == 0 ? xy0 : xy1;
                sse += (rp[0] - xy[0]) * (rp[0] - xy[0]) + (rp[1] - xy[1]) * (rp[1] - xy[1]);
            }
            pt.reproj_rms_px = std::sqrt(sse / T{4}) * kFocal<T>;
        }

        // Monte Carlo depth σ under 1 px observation noise — the true uncertainty.
        T sum = 0, sum2 = 0;
        std::size_t ok = 0;
        for (std::size_t k = 0; k < mc; ++k) {
            std::vector<Obs> noisy{{0, 0, {xy0[0] + sigma_norm * T(N01(rng)), xy0[1] + sigma_norm * T(N01(rng))}},
                                   {1, 0, {xy1[0] + sigma_norm * T(N01(rng)), xy1[1] + sigma_norm * T(N01(rng))}}};
            Vec3 q{};
            if (upd.triangulate(s, noisy, q)) {
                sum += q[2];
                sum2 += q[2] * q[2];
                ++ok;
            }
        }
        pt.mc_failures = mc - ok;
        if (ok > 1) {
            const T mean = sum / T(ok);
            const T var = std::max(T{0}, sum2 / T(ok) - mean * mean);
            pt.depth_sigma_mm = std::sqrt(var) * T{1000};
        }
        out.curve.push_back(pt);
    }

    // Suggested gate: the smallest swept parallax whose depth σ is within 5% of
    // depth. If no swept level qualifies, leave the gate at +∞ (an honest "not
    // found within the swept range") rather than reporting the last sample as if
    // it were achieved. A point with a valid Monte-Carlo σ has mc_failures < mc.
    const T budget_mm = T{0.05} * depth_m * T{1000};
    out.gate_parallax_deg = std::numeric_limits<T>::infinity();
    for (const auto& p : out.curve)
        if (p.mc_failures < mc && p.depth_sigma_mm <= budget_mm) {
            out.gate_parallax_deg = p.parallax_deg;
            break;
        }
    return out;
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_TRIANGULATION_PROBE_HPP
