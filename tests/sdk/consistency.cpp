// Filter-consistency statistics: NEES / NIS + the chi-square run-consistency
// test (issue #264, the keystone diagnostic instrument).
//
// These tests verify the instrument against an oracle INDEPENDENT of the
// implementation: hand-computed quadratic forms, the known probit values, and
// the chi-square distribution of Gaussian quadratic forms itself (sampled with
// a seeded RNG). That independence is the point — the methodology post-mortem
// (docs/assessments/vio-diagnostic-methodology.md) traced the gravity-sign bug
// to synthetic tests that re-encoded the code's own assumptions, so a
// consistency instrument must be checked against probability theory, not
// against itself.

#include <branes/sdk/eval/consistency.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
using T = double;
using ev::DynMat;

DynMat<T> mat(std::size_t n, std::initializer_list<T> rowmajor) {
    DynMat<T> m(n, n);
    auto it = rowmajor.begin();
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            m(i, j) = *it++;
    return m;
}

}  // namespace

TEST_CASE("normalized_squared matches hand-computed quadratic forms", "[sdk][eval][consistency]") {
    // Identity covariance ⇒ eᵀ P⁻¹ e = ‖e‖². e=(3,4) ⇒ 25.
    {
        const std::array<T, 2> e{3.0, 4.0};
        REQUIRE_THAT(ev::normalized_squared<T>(e, mat(2, {1, 0, 0, 1})), Catch::Matchers::WithinAbs(25.0, 1e-12));
    }
    // Diagonal covariance ⇒ Σ eᵢ²/σᵢ². e=(2,3), P=diag(4,9) ⇒ 4/4 + 9/9 = 2.
    {
        const std::array<T, 2> e{2.0, 3.0};
        REQUIRE_THAT(ev::normalized_squared<T>(e, mat(2, {4, 0, 0, 9})), Catch::Matchers::WithinAbs(2.0, 1e-12));
    }
    // Full 2×2: P=[[2,1],[1,2]], P⁻¹=(1/3)[[2,-1],[-1,2]], e=(1,1) ⇒ (1/3)(2-1-1+2)=2/3.
    {
        const std::array<T, 2> e{1.0, 1.0};
        REQUIRE_THAT(ev::normalized_squared<T>(e, mat(2, {2, 1, 1, 2})), Catch::Matchers::WithinAbs(2.0 / 3.0, 1e-12));
    }
}

TEST_CASE("normalized_squared hard-refuses degenerate inputs", "[sdk][eval][consistency]") {
    const std::array<T, 2> e{1.0, 2.0};
    // Shape mismatch (vector longer than the matrix).
    const std::array<T, 3> e3{1.0, 2.0, 3.0};
    REQUIRE_THROWS_AS(ev::normalized_squared<T>(e3, mat(2, {1, 0, 0, 1})), std::invalid_argument);
    // Non-positive-definite covariance (zero on the diagonal): a hard refusal,
    // not a NaN/garbage number.
    REQUIRE_THROWS_AS(ev::normalized_squared<T>(e, mat(2, {0, 0, 0, 1})), std::domain_error);
    // Indefinite (negative eigenvalue) covariance.
    REQUIRE_THROWS_AS(ev::normalized_squared<T>(e, mat(2, {1, 2, 2, 1})), std::domain_error);
}

TEST_CASE("inverse_normal_cdf matches known probit values", "[sdk][eval][consistency]") {
    using ev::detail::inverse_normal_cdf;
    REQUIRE_THAT(inverse_normal_cdf(0.5), Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(inverse_normal_cdf(0.975), Catch::Matchers::WithinAbs(1.959963985, 1e-7));
    REQUIRE_THAT(inverse_normal_cdf(0.995), Catch::Matchers::WithinAbs(2.575829304, 1e-7));
    REQUIRE_THAT(inverse_normal_cdf(0.025), Catch::Matchers::WithinAbs(-1.959963985, 1e-7));
    REQUIRE_THROWS_AS(inverse_normal_cdf(0.0), std::invalid_argument);
    REQUIRE_THROWS_AS(inverse_normal_cdf(1.0), std::invalid_argument);
}

TEST_CASE("ConsistencyAccumulator argument + state contracts", "[sdk][eval][consistency]") {
    ev::ConsistencyAccumulator acc;
    REQUIRE_THROWS_AS(acc.report(), std::domain_error);  // no samples yet
    REQUIRE_THROWS_AS(acc.add(3.0, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(acc.add(3.0, -2), std::invalid_argument);
    acc.add(3.0, 3);  // varying dof is allowed (e.g. NIS across frames)
    acc.add(2.0, 2);
    const auto r = acc.report();
    REQUIRE(r.samples == 2);
    REQUIRE_THAT(r.total_dof, Catch::Matchers::WithinAbs(5.0, 1e-12));
    REQUIRE_THAT(r.normalized, Catch::Matchers::WithinAbs(1.0, 1e-12));  // (3+2)/(3+2)
}

// The real test of the instrument: against the chi-square distribution itself.
// Draw e ~ N(0, P) for a known P; the quadratic form eᵀ P⁻¹ e must be χ²(dim),
// so the run-average NEES must read ≈ dim and the accumulator must call it
// consistent — while a deliberately mis-tuned covariance must be FLAGGED.
TEST_CASE("NEES/NIS flag over- and under-confidence on Gaussian samples", "[sdk][eval][consistency]") {
    // A known SPD 3×3 covariance and its lower Cholesky factor L (P = L Lᵀ),
    // so e = L·z with z ~ N(0, I) has covariance exactly P.
    const DynMat<T> P = mat(3, {4, 1, 0, 1, 3, 1, 0, 1, 2});
    DynMat<T> L;
    REQUIRE(branes::sdk::msckf::cholesky(P, L));

    std::mt19937 gen(20240602u);
    std::normal_distribution<T> nrm(0.0, 1.0);
    constexpr int N = 40000;
    constexpr std::size_t dim = 3;

    ev::ConsistencyAccumulator consistent, over, under;
    // Over-confident: filter reports P/4 (too small) while the data has cov P.
    // Under-confident: filter reports 4·P (too large).
    DynMat<T> P_over = P, P_under = P;
    for (std::size_t i = 0; i < dim; ++i)
        for (std::size_t j = 0; j < dim; ++j) {
            P_over(i, j) = P(i, j) / 4.0;
            P_under(i, j) = P(i, j) * 4.0;
        }

    double sum_nees = 0.0;
    for (int s = 0; s < N; ++s) {
        std::array<T, dim> z{nrm(gen), nrm(gen), nrm(gen)};
        std::array<T, dim> e{};
        for (std::size_t i = 0; i < dim; ++i)  // e = L z
            for (std::size_t j = 0; j <= i; ++j)
                e[i] += L(i, j) * z[j];
        const T q = ev::nees<T>(e, P);
        sum_nees += q;
        consistent.add(q, dim);
        over.add(ev::nees<T>(e, P_over), dim);
        under.add(ev::nees<T>(e, P_under), dim);
    }

    // Mean NEES ≈ dim (3) — the defining property of a consistent filter.
    REQUIRE_THAT(sum_nees / N, Catch::Matchers::WithinAbs(static_cast<double>(dim), 0.06));

    // Correctly-specified covariance passes even a wide (α=0.001) band.
    REQUIRE(consistent.report(0.001).consistent());

    // P/4 ⇒ NEES inflated ~4× ⇒ flagged over-confident; 4·P ⇒ ~0.25× ⇒ under.
    const auto ro = over.report();
    REQUIRE(ro.overconfident);
    REQUIRE_FALSE(ro.consistent());
    REQUIRE_THAT(ro.normalized, Catch::Matchers::WithinAbs(4.0, 0.1));

    const auto ru = under.report();
    REQUIRE(ru.underconfident);
    REQUIRE_FALSE(ru.consistent());
    REQUIRE_THAT(ru.normalized, Catch::Matchers::WithinAbs(0.25, 0.02));
}
