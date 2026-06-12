// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/update_probe.hpp — the S6 (MSCKF measurement update) probe of
// the VIO contract program (docs/arch/vio-pipeline-canonical.md). S6 is the
// consistency-critical stage: it is where the filter's three master invariants
// (NEES≈dim, NIS≈dof, white innovations) are won or lost. The global #212 symptom
// is NEES ≫ dim; this probe measures its *local* mirror — **NIS per update** — and
// the algebra the update rides on (feature marginalization, Joseph PSD).
//
// The headline instrument is a Monte-Carlo NIS consistency test that drives the
// SHIPPED CameraUpdater::update on a well-conditioned mono scene with a
// self-consistent (P, R): the belief clone poses are perturbed by a draw from the
// filter's own covariance P, and the observations carry pixel noise of exactly the
// σ the filter assumes. Under correct linearization and an honest (P, R) the
// projected innovation νᵀS⁻¹ν is χ²(2m−3), so mean NIS → dof. If the shipped update
// mis-scales R, mis-builds H, or breaks the null-space noise structure, the mean
// departs from dof even though the inputs are self-consistent — a genuine test of
// the update *algebra*, isolated from the rest of the pipeline.
//
// A noise-mismatch sweep then reproduces over-confidence locally: hold the filter's
// assumed σ fixed and inject noise_scale·σ. NIS/dof rises above 1 as the true
// measurement noise exceeds what the filter models — the same mechanism the S10
// calibration probe quantifies globally as the empirical "R×4", seen here at the
// innovation. Two deterministic algebra checks accompany it: the left-null-space
// projection marginalizes the feature (NᵀH_f ≈ 0) with orthonormal reflectors
// (NᵀN = I, so the projected noise stays σ²·I), and Joseph keeps P PSD.
//
// Native units: NIS/dof (dimensionless, target 1), dimensionless covariance
// residuals. Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_UPDATE_PROBE_HPP
#define BRANES_SDK_EVAL_UPDATE_PROBE_HPP

#include <branes/sdk/eval/consistency.hpp>
#include <branes/sdk/features/msckf_nullspace.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace branes::sdk::eval {

namespace upd_detail {
template <math::Scalar T>
using Vec3 = math::lie::detail::Vec<T, 3>;
template <math::Scalar T>
using SO3 = math::lie::SO3<T>;

// The isolated scene: m world-aligned clones translating along x by `baseline`
// each, looking down +z, observing one feature at `F`. The geometry is chosen for
// healthy parallax (~13° over a 1.2 m baseline at 5 m) so the test exercises the
// UPDATE, not triangulation degeneracy (that is S5's probe).
template <math::Scalar T>
struct Scene {
    T baseline = T{4} / T{10};                      // clone-to-clone translation (m)
    Vec3<T> F{{T{3} / T{10}, T{-1} / T{5}, T{5}}};  // feature in world (m)
    T sigma_theta = T{873} / T{100000};             // clone attitude σ ≈ 0.5° (rad)
    T sigma_pos = T{2} / T{100};                    // clone position σ (m)
    T sigma_meas = T{5} / T{1000};                  // assumed normalized image σ
    T sigma_imu = T{1} / T{100};                    // IMU-block σ (untouched by cam update)
};
}  // namespace upd_detail

/// One Monte-Carlo NIS run of the shipped update at a given measurement-noise
/// mismatch. `noise_scale` = 1 is the self-consistent case (NIS/dof → 1).
template <math::Scalar T>
struct NisRun {
    T noise_scale = T{1};              ///< injected σ ÷ assumed σ (1 = matched)
    T mean_nis = T{0};                 ///< mean projected NIS over the updates
    T nis_over_dof = T{0};             ///< mean_nis / dof — the headline (target 1 at matched)
    std::size_t dof = 0;               ///< projected residual dimension per update (2m−3)
    std::size_t samples = 0;           ///< updates accumulated
    T band_lo = T{0}, band_hi = T{0};  ///< χ² consistency band on nis_over_dof
    bool consistent = false;           ///< nis_over_dof within the band
    bool joseph_pd_all = true;         ///< P stayed positive-definite after every update
};

/// Drive the SHIPPED CameraUpdater::update `trials` times on the isolated scene.
/// Each trial draws a fresh clone-pose error from the filter's own covariance P
/// and fresh image noise of `noise_scale · σ`, runs one feature update with gating
/// OFF (so the NIS sample is unbiased by the gate truncating the χ² tail), and
/// accumulates the projected NIS. Returns the mean NIS/dof and whether Joseph kept
/// P PSD throughout.
template <math::Scalar T>
[[nodiscard]] NisRun<T> update_nis_run(std::size_t m = 4,
                                       std::size_t trials = 4000,
                                       T noise_scale = T{1},
                                       std::uint64_t seed = 0xC0FFEEull,
                                       T calib_rot_sigma = T{0}) {
    // MSCKF needs >= 2 observations (2m > 3); fewer underflows the size_t dof `2m-3`.
    assert(m >= 2 && "update_nis_run: m (clones/observations) must be >= 2");
    using namespace upd_detail;
    namespace ms = branes::sdk::msckf;
    using Obs = ms::CameraObservation<T>;
    const Scene<T> sc;

    ms::CameraUpdaterOptions<T> opts;
    opts.normalized_sigma = sc.sigma_meas;
    opts.enable_gating = false;  // collect the full NIS distribution, untruncated
    opts.min_parallax_deg = T{0};
    // S10 calibration-uncertainty term: when set, it inflates the modeled R, so a
    // run with injected noise_scale·σ becomes consistent again (the compensation).
    opts.calib_rot_sigma = calib_rot_sigma;
    const ms::CameraUpdater<T> upd(std::vector<ms::CameraExtrinsics<T>>(1), opts);

    std::mt19937_64 rng(seed);
    std::normal_distribution<double> N01(0.0, 1.0);
    auto g = [&] { return T(N01(rng)); };

    const std::size_t dim = ms::State<T>::kImuDim + ms::State<T>::kCloneDim * m;
    const T th2 = sc.sigma_theta * sc.sigma_theta;
    const T p2 = sc.sigma_pos * sc.sigma_pos;
    const T inj = noise_scale * sc.sigma_meas;  // actual injected image σ

    NisRun<T> out;
    out.noise_scale = noise_scale;
    out.dof = 2 * m - 3;

    ConsistencyAccumulator acc;
    std::size_t pd_fail = 0;
    for (std::size_t t = 0; t < trials; ++t) {
        ms::State<T> s(sc.sigma_imu);  // 15×15 IMU covariance to start

        // Believed covariance P: small IMU block (untouched by the camera update)
        // plus a per-clone (θ,p) block — the covariance the perturbation is drawn
        // from, so (P, R) are self-consistent by construction.
        ms::DynMat<T> P(dim, dim);
        for (std::size_t i = 0; i < ms::State<T>::kImuDim; ++i)
            P(i, i) = sc.sigma_imu * sc.sigma_imu;

        s.clones.clear();
        std::vector<Obs> obs;
        obs.reserve(m);
        for (std::size_t c = 0; c < m; ++c) {
            const std::size_t off = ms::State<T>::kImuDim + ms::State<T>::kCloneDim * c;
            for (std::size_t a = 0; a < 3; ++a) {
                P(off + a, off + a) = th2;
                P(off + 3 + a, off + 3 + a) = p2;
            }
            // True clone pose: world-aligned, on the x baseline.
            const Vec3<T> p_true{{sc.baseline * T(c), T{0}, T{0}}};
            // Belief = true ⊞ draw from P: R ← Exp(δθ), p ← p + δp.
            const Vec3<T> dth{{sc.sigma_theta * g(), sc.sigma_theta * g(), sc.sigma_theta * g()}};
            const Vec3<T> dp{{sc.sigma_pos * g(), sc.sigma_pos * g(), sc.sigma_pos * g()}};
            s.clones.push_back({SO3<T>::exp(dth),
                                Vec3<T>{{p_true[0] + dp[0], p_true[1] + dp[1], p_true[2] + dp[2]}},
                                static_cast<double>(c)});
            // True observation from the TRUE pose (identity rotation) + image noise.
            const Vec3<T> pc{{sc.F[0] - p_true[0], sc.F[1] - p_true[1], sc.F[2] - p_true[2]}};
            obs.push_back(Obs{c, 0, {{pc[0] / pc[2] + inj * g(), pc[1] / pc[2] + inj * g()}}});
        }
        s.cov.P = std::move(P);

        ms::NisSample<T> nis;
        const ms::FeatureTrack<T> track{obs};
        if (!upd.update(s, track, &nis) || !nis.valid)
            continue;  // degenerate trial (should not happen at this geometry)
        acc.add(static_cast<double>(nis.value), static_cast<int>(nis.dof));
        if (!ms::is_positive_definite(s.cov.P))
            ++pd_fail;
        ++out.samples;
    }

    if (out.samples > 0) {
        const auto rep = acc.report();
        out.nis_over_dof = static_cast<T>(rep.normalized);
        out.mean_nis = static_cast<T>(rep.sum_statistic / static_cast<double>(out.samples));
        out.band_lo = static_cast<T>(rep.lower);
        out.band_hi = static_cast<T>(rep.upper);
        out.consistent = rep.consistent();
    }
    out.joseph_pd_all = (pd_fail == 0);
    return out;
}

/// Deterministic check of the S6b/S6e algebra that does not need Monte Carlo:
/// the left-null-space projection must (1) marginalize the feature — NᵀH_f ≈ 0 —
/// and (2) use orthonormal reflectors — NᵀN = I — so the projected measurement
/// noise stays σ²·I and the surviving system has dimension 2m−3.
template <math::Scalar T>
struct NullspaceCheck {
    std::size_t rows_out = 0;       ///< projected residual dimension reported
    std::size_t rows_expected = 0;  ///< 2m − 3
    T ntHf_max = T{0};              ///< ‖NᵀH_f‖_max — feature marginalized (≈ 0)
    T orth_max = T{0};              ///< ‖NᵀN − I‖_max — noise-preserving (≈ 0)
};

template <math::Scalar T>
[[nodiscard]] NullspaceCheck<T> update_nullspace_check(std::size_t m = 4) {
    // MSCKF needs >= 2 observations (2m > 3); fewer underflows the size_t dim `2m-3`.
    assert(m >= 2 && "update_nullspace_check: m must be >= 2");
    using std::abs;
    const std::size_t rows = 2 * m;

    // A full-column-rank feature Jacobian H_f (2m×3) with distinct, non-degenerate
    // rows; the projection is geometry-agnostic, so deterministic values suffice.
    std::vector<T> Hf(rows * 3);
    for (std::size_t i = 0; i < rows; ++i) {
        Hf[i * 3 + 0] = T{1} + T(i) / T{7};
        Hf[i * 3 + 1] = T(i % 3) - T{1} + T(i) / T{11};
        Hf[i * 3 + 2] = T{1} - T(i) / T{13};
    }
    // Put the identity in the H_x slot so the projected H_x rows ARE Nᵀ (the last
    // 2m−3 rows of Qᵀ). One call then yields both invariants.
    std::vector<T> Ix(rows * rows, T{0});
    for (std::size_t i = 0; i < rows; ++i)
        Ix[i * rows + i] = T{1};
    std::vector<T> r(rows, T{0});

    const auto proj = features::msckf_left_nullspace_project<T>(Hf, Ix, r, rows, rows);

    NullspaceCheck<T> out;
    out.rows_out = proj.rows;
    out.rows_expected = rows - 3;
    if (proj.rows == 0)
        return out;

    // M = proj.H_x is (2m−3)×(2m) = Nᵀ (rows span the left null space of H_f).
    auto M = [&](std::size_t i, std::size_t j) { return proj.H_x[i * rows + j]; };

    // (1) NᵀH_f = M · H_f  →  must be ≈ 0 (feature eliminated).
    for (std::size_t i = 0; i < proj.rows; ++i)
        for (std::size_t b = 0; b < 3; ++b) {
            T s = T{0};
            for (std::size_t k = 0; k < rows; ++k)
                s += M(i, k) * Hf[k * 3 + b];
            out.ntHf_max = std::max(out.ntHf_max, abs(s));
        }
    // (2) NᵀN = M · Mᵀ  →  must be ≈ I (orthonormal ⇒ noise stays σ²·I).
    for (std::size_t i = 0; i < proj.rows; ++i)
        for (std::size_t j = 0; j < proj.rows; ++j) {
            T s = T{0};
            for (std::size_t k = 0; k < rows; ++k)
                s += M(i, k) * M(j, k);
            out.orth_max = std::max(out.orth_max, abs(s - (i == j ? T{1} : T{0})));
        }
    return out;
}

/// Aggregate S6 reading: the algebra checks, the matched-noise NIS consistency,
/// and the noise-mismatch NIS sweep (the local over-confidence lever).
template <math::Scalar T>
struct UpdateProbe {
    NullspaceCheck<T> nullspace;
    NisRun<T> matched;             ///< noise_scale = 1
    std::vector<NisRun<T>> sweep;  ///< NIS/dof vs injected-noise mismatch
};

template <math::Scalar T>
[[nodiscard]] UpdateProbe<T> update_probe(std::size_t m = 4, std::size_t trials = 4000) {
    UpdateProbe<T> out;
    out.nullspace = update_nullspace_check<T>(m);
    out.matched = update_nis_run<T>(m, trials, T{1}, 0xC0FFEEull);
    for (const T sc : {T{1} / T{2}, T{1}, T{2}, T{4}})
        out.sweep.push_back(update_nis_run<T>(m, trials, sc, 0xC0FFEEull));
    return out;
}

/// S10 reading: the calibration-uncertainty term as the *principled* cure for the
/// over-confidence the noise-mismatch sweep exposes. The filter is fed true image
/// noise of `noise_scale·σ` but models only `σ` ⇒ NIS/dof ≫ 1 (the local image of
/// the empirical "R×4"). The fix is not to rewrite the update but to put the
/// missing variance where it belongs — in `R` — via the calibration term
/// `calib_rot_sigma = σ·√(noise_scale²−1)`, which makes the modeled variance
/// `σ² + calib_rot_sigma² = (noise_scale·σ)²` match the truth. NIS/dof then returns
/// to ≈ 1. This is the innovation-level proof that **modeling the inputs** (S10),
/// not the EKF algebra (S6, already measured-consistent), removes the #212 fault.
template <math::Scalar T>
struct CalibCompensation {
    T noise_scale = T{2};      ///< true image noise ÷ assumed σ (the under-model)
    T calib_rot_sigma = T{0};  ///< the S10 term applied to match it
    NisRun<T> uncompensated;   ///< calib term off: NIS/dof ≫ 1 (over-confident)
    NisRun<T> compensated;     ///< calib term on at the matching σ: NIS/dof ≈ 1
};

template <math::Scalar T>
[[nodiscard]] CalibCompensation<T>
update_calib_compensation(std::size_t m = 4, std::size_t trials = 4000, T noise_scale = T{2}) {
    using std::sqrt;
    assert(noise_scale >= T{1} && "update_calib_compensation: noise_scale must be >= 1");
    const upd_detail::Scene<T> sc;
    CalibCompensation<T> out;
    out.noise_scale = noise_scale;
    // The exact term that lifts the modeled variance σ² to (noise_scale·σ)².
    out.calib_rot_sigma = sc.sigma_meas * sqrt(noise_scale * noise_scale - T{1});
    out.uncompensated = update_nis_run<T>(m, trials, noise_scale, 0xC0FFEEull, T{0});
    out.compensated = update_nis_run<T>(m, trials, noise_scale, 0xC0FFEEull, out.calib_rot_sigma);
    return out;
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_UPDATE_PROBE_HPP
