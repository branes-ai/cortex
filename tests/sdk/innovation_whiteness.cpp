// Innovation zero-mean + whiteness tests (#280 discriminator).
//
// Verified against independent oracles — synthetic innovation sequences with a
// KNOWN structure must produce the expected verdict:
//   • i.i.d. N(0,1)        → zero-mean, white (no flag)
//   • constant offset       → biased, but NOT correlated (independent axes)
//   • zero-mean ±1 alternation → white-mean but strongly anti-correlated
// — not against the accumulator's own assumptions.
//
// The normal draws are generated explicitly (Box–Muller over the mt19937 engine's
// standardized output), not via std::normal_distribution, whose generation
// algorithm is implementation-defined and would make the verdicts platform-flaky.

#include <branes/sdk/eval/innovation_whiteness.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <random>

namespace {
namespace ev = branes::sdk::eval;
}  // namespace

TEST_CASE("empty accumulator reports nothing", "[sdk][eval][innovation]") {
    ev::InnovationWhitenessAccumulator acc;
    const auto rep = acc.report();
    REQUIRE(rep.updates == 0);
    REQUIRE(rep.dof_total == 0);
    REQUIRE_FALSE(rep.biased);
    REQUIRE_FALSE(rep.correlated);
    acc.add(1.0, 0);  // a zero-dof update is ignored
    REQUIRE(acc.updates() == 0);
}

TEST_CASE("i.i.d. N(0,1) innovations read as zero-mean and white", "[sdk][eval][innovation]") {
    ev::InnovationWhitenessAccumulator acc;
    // Deterministic standard-normal draws via Box–Muller over mt19937's
    // platform-independent output (avoids implementation-defined distributions).
    std::mt19937 gen(424242u);
    auto unit = [&gen] { return (static_cast<double>(gen()) + 0.5) / 4294967296.0; };  // (0,1)
    auto draw_normal = [&] {
        const double u1 = unit(), u2 = unit();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
    };
    for (int i = 0; i < 20000; ++i)
        acc.add(draw_normal(), 1);  // dof=1 ⇒ innov_sum is itself the standardized u
    const auto rep = acc.report(0.01);
    // Under H0 both z-scores are a single N(0,1) draw, so the verdict booleans
    // (thresholded at z≈2.58) fire ~1% of the time on any one clean sequence by
    // construction — asserting them here would be inherently flaky. The robust,
    // seed-independent statement is that the magnitudes stay O(1): P(|N(0,1)|>4.5)
    // ≈ 7e-6, far below any CI flake budget.
    REQUIRE(std::abs(rep.mean_z) < 4.5);
    REQUIRE(std::abs(rep.lag1_z) < 4.5);
}

TEST_CASE("a constant offset is flagged BIASED", "[sdk][eval][innovation]") {
    ev::InnovationWhitenessAccumulator acc;
    for (int i = 0; i < 2000; ++i)
        acc.add(0.3, 1);  // persistent positive innovation
    const auto rep = acc.report(0.01);
    REQUIRE_THAT(rep.mean, Catch::Matchers::WithinAbs(0.3, 1e-9));
    REQUIRE(rep.mean_z > rep.z_crit);  // 0.3·√2000 ≈ 13.4 ≫ 2.576
    REQUIRE(rep.biased);
    // A constant offset is biased but temporally white: mean-centering the
    // autocorrelation makes `biased` and `correlated` independent axes.
    REQUIRE_FALSE(rep.correlated);
}

TEST_CASE("a zero-mean alternating sequence is white-mean but CORRELATED", "[sdk][eval][innovation]") {
    ev::InnovationWhitenessAccumulator acc;
    for (int i = 0; i < 2000; ++i)
        acc.add((i % 2 == 0) ? 1.0 : -1.0, 1);  // +1,−1,+1,−1, … : mean 0, anti-correlated
    const auto rep = acc.report(0.01);
    REQUIRE_THAT(rep.mean, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_FALSE(rep.biased);                                       // zero mean
    REQUIRE_THAT(rep.lag1, Catch::Matchers::WithinAbs(-1.0, 0.01));  // perfect anti-correlation
    REQUIRE(rep.correlated);                                         // |lag1_z| ≈ √1999 ≫ 2.576
}

TEST_CASE("innov_sum aggregates components: N(0,dof) scaling", "[sdk][eval][innovation]") {
    // Feed each update a sum over `dof` unit-variance components. With dof=4 and a
    // per-update sum of 2.0, the standardized u = 2/√4 = 1.0; a constant u=1 is a
    // (biased) mean of 0.5 per component (sum/dof_total).
    ev::InnovationWhitenessAccumulator acc;
    for (int i = 0; i < 1000; ++i)
        acc.add(2.0, 4);
    const auto rep = acc.report(0.01);
    REQUIRE(rep.dof_total == 4000);
    REQUIRE_THAT(rep.mean, Catch::Matchers::WithinAbs(0.5, 1e-9));  // 2000 / 4000
    REQUIRE(rep.biased);
}
