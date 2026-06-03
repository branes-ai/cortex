// Innovation zero-mean + whiteness tests (#280 discriminator).
//
// Verified against independent oracles — synthetic innovation sequences with a
// KNOWN structure must produce the expected verdict:
//   • i.i.d. N(0,1)        → zero-mean, white (no flag)
//   • constant offset       → biased
//   • zero-mean ±1 alternation → white-mean but strongly anti-correlated
// — not against the accumulator's own assumptions.

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
    std::mt19937 gen(424242u);
    std::normal_distribution<double> nrm(0.0, 1.0);
    for (int i = 0; i < 20000; ++i)
        acc.add(nrm(gen), 1);  // dof=1 ⇒ innov_sum is itself the standardized u
    const auto rep = acc.report(0.01);
    // Both z-scores are ~N(0,1) under H0; a fixed seed keeps them well inside ±4.
    REQUIRE(std::abs(rep.mean_z) < 4.0);
    REQUIRE(std::abs(rep.lag1_z) < 4.0);
    REQUIRE_FALSE(rep.biased);
    REQUIRE_FALSE(rep.correlated);
}

TEST_CASE("a constant offset is flagged BIASED", "[sdk][eval][innovation]") {
    ev::InnovationWhitenessAccumulator acc;
    for (int i = 0; i < 2000; ++i)
        acc.add(0.3, 1);  // persistent positive innovation
    const auto rep = acc.report(0.01);
    REQUIRE_THAT(rep.mean, Catch::Matchers::WithinAbs(0.3, 1e-9));
    REQUIRE(rep.mean_z > rep.z_crit);  // 0.3·√2000 ≈ 13.4 ≫ 2.576
    REQUIRE(rep.biased);
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
