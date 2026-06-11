// S3 (state augmentation / stochastic cloning) and S9 (marginalization / clone
// management) probes (docs/arch/vio-pipeline-canonical.md).
//
// Locks the measured contract of the shipped StateHelper bookkeeping against the
// textbook clean-room failure modes:
//
//   S3 — a clone is a DETERMINISTIC COPY of the current pose, so at augmentation
//        its marginal AND its cross-covariance with every other state must equal
//        the cloned pose's. The classic inconsistency is to "just add a new block"
//        with zero / block-diagonal cross-covariance.
//   S9 — dropping a clone is principal-submatrix extraction, so the kept-state
//        marginal must be UNCHANGED.
//
// The headline reading these assertions pin: both residuals are IDENTICALLY ZERO
// (P' = G P Gᵀ with a copy-selection G is exact algebra, and cov.marginalize is an
// exact submatrix extraction) — so neither S3 nor S9 is a source of the #212
// over-confidence. These run on the synthetic, GT-free covariance the probe builds
// (no EuRoC dataset needed), so they gate in CI.

#include <branes/sdk/eval/clone_window_probe.hpp>

#include <catch2/catch_test_macros.hpp>

namespace {
namespace ev = branes::sdk::eval;
using T = double;
// The block-equality residuals are exact algebra; allow only rounding slack.
constexpr T kEps = 1e-12;
}  // namespace

TEST_CASE("S3 augment clones the pose with exact correlation, not an independent block",
          "[sdk][s3][augmentation][clone-window]") {
    const auto r = ev::augmentation_probe<T>();

    // The clone's own block equals the cloned pose's marginal …
    REQUIRE(r.clone_marginal_err < kEps);
    // … and its cross-covariance with every existing state equals the pose's
    // (perfectly correlated — the zero-cross-covariance inconsistency would make
    // this nonzero and under-confidence the filter at clone time).
    REQUIRE(r.clone_cross_err < kEps);
    // P' is a valid covariance.
    REQUIRE(r.psd);
    // Augmentation adds exactly one 6-DoF clone.
    REQUIRE(r.dim_after == r.dim_before + 6);
}

TEST_CASE("S3 augment then marginalize the fresh clone restores P exactly (round-trip)",
          "[sdk][s3][augmentation][clone-window]") {
    const auto r = ev::augmentation_probe<T>();
    REQUIRE(r.roundtrip_err < kEps);
}

TEST_CASE("S9 marginalize is exact principal-submatrix extraction - kept marginal unchanged",
          "[sdk][s9][marginalization][clone-window]") {
    const auto r = ev::marginalization_probe<T>();

    // Dropping a clone never alters the marginal of the kept states.
    REQUIRE(r.kept_marginal_err < kEps);
    REQUIRE(r.psd);
    // The window shrinks by exactly one 6-DoF clone and stays bounded.
    REQUIRE(r.dim_after == r.dim_before - 6);
    REQUIRE(r.clones_after == r.clones_before - 1);
}
