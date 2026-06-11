// S6 MSCKF measurement-update probe (docs/arch/vio-pipeline-canonical.md).
//
// S6 is the consistency-critical stage — where NEES≈dim / NIS≈dof / white
// innovations are won or lost. This locks the measured S6 verdict against the
// shipped CameraUpdater::update, driven in isolation on a self-consistent (P, R)
// scene (clone poses perturbed by a draw from the filter's OWN covariance, image
// noise at exactly the assumed σ):
//
//   • the left-null-space projection marginalizes the feature (NᵀH_f ≈ 0) with
//     orthonormal reflectors (NᵀN = I), so the projected noise stays σ²·I, and the
//     surviving system has dimension 2m−3;
//   • at matched (P, R) the projected innovation is χ²(2m−3), so NIS/dof ≈ 1 — the
//     update ALGEBRA is consistent (the #212 over-confidence enters via the INPUTS,
//     not the update math);
//   • Joseph keeps P positive-definite on every update;
//   • understating R drives NIS/dof above 1 — the innovation-level image of the
//     empirical "R×4".
//
// The Monte-Carlo draws use fixed seeds, so these readings are deterministic and
// gate in CI without the EuRoC dataset.

#include <branes/sdk/eval/update_probe.hpp>

#include <catch2/catch_test_macros.hpp>

namespace {
namespace ev = branes::sdk::eval;
using T = double;
constexpr std::size_t kM = 4;  // clones per feature ⇒ dof = 2m−3 = 5
constexpr std::size_t kTrials = 3000;
}  // namespace

TEST_CASE("S6 left-null-space projection marginalizes the feature with orthonormal reflectors", "[sdk][s6][update]") {
    const auto ns = ev::update_nullspace_check<T>(kM);
    // Feature eliminated: NᵀH_f ≈ 0.
    REQUIRE(ns.ntHf_max < 1e-10);
    // Orthonormal: NᵀN = I ⇒ the projected measurement noise stays σ²·I.
    REQUIRE(ns.orth_max < 1e-10);
    // Surviving system has dimension 2m−3.
    REQUIRE(ns.rows_out == ns.rows_expected);
    REQUIRE(ns.rows_out == 2 * kM - 3);
}

TEST_CASE("S6 update is consistent at matched (P, R): NIS approx dof, Joseph keeps P PSD", "[sdk][s6][update]") {
    const auto run = ev::update_nis_run<T>(kM, kTrials, /*noise_scale=*/1.0, 0xC0FFEEull);
    REQUIRE(run.samples > kTrials / 2);  // the geometry triangulates every trial
    REQUIRE(run.dof == 2 * kM - 3);
    // Fed an honest (P, R), the shipped update makes the innovation χ²(dof): NIS/dof ≈ 1.
    REQUIRE(run.nis_over_dof > 0.9);
    REQUIRE(run.nis_over_dof < 1.1);
    REQUIRE(run.consistent);  // within the χ² band on the run
    // Joseph form keeps the covariance positive-definite through every update.
    REQUIRE(run.joseph_pd_all);
}

TEST_CASE("S6 NIS is the local over-confidence lever: understating R drives NIS above dof", "[sdk][s6][update]") {
    const auto lo = ev::update_nis_run<T>(kM, kTrials, 0.5, 0xC0FFEEull);
    const auto matched = ev::update_nis_run<T>(kM, kTrials, 1.0, 0xC0FFEEull);
    const auto hi = ev::update_nis_run<T>(kM, kTrials, 2.0, 0xC0FFEEull);
    const auto hi2 = ev::update_nis_run<T>(kM, kTrials, 4.0, 0xC0FFEEull);

    // Monotone in the noise mismatch: over-modeled R (NIS↓) … matched … under-modeled R (NIS↑).
    REQUIRE(lo.nis_over_dof < matched.nis_over_dof);
    REQUIRE(matched.nis_over_dof < hi.nis_over_dof);
    REQUIRE(hi.nis_over_dof < hi2.nis_over_dof);
    // The under-modeled cases are decisively over-confident (NIS ≫ dof) — the
    // innovation-level signature of the #212 over-confidence.
    REQUIRE(hi.nis_over_dof > 1.3);
    REQUIRE(hi2.nis_over_dof > 2.0);
}
