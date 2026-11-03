// SPDX-License-Identifier: MIT
//
// branes/math/stats.hpp tests (#481): the chi-square quantile the MSCKF gate
// uses, against tabulated values, and the CDF it inverts.

#include <branes/math/stats.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace bm = branes::math;

TEST_CASE("chi_square_quantile matches tabulated values", "[math][stats]") {
    struct Row {
        double p;
        std::size_t k;
        double x;
    };
    // Standard chi-square table (six significant figures).
    const Row rows[] = {{0.95, 1, 3.841459},
                        {0.95, 2, 5.991465},
                        {0.95, 3, 7.814728},
                        {0.95, 9, 16.918978},
                        {0.95, 21, 32.670573},
                        {0.95, 100, 124.342113},
                        {0.99, 1, 6.634897},
                        {0.99, 3, 11.344867},
                        {0.99, 30, 50.892181},
                        {0.05, 1, 0.003932},
                        {0.05, 10, 3.940299},
                        {0.50, 4, 3.356694}};
    for (const auto& r : rows) {
        CAPTURE(r.p, r.k);
        const double x = bm::chi_square_quantile(r.p, r.k);
        REQUIRE(std::abs(x - r.x) < 1e-6 * std::max(1.0, r.x));
    }
}

TEST_CASE("chi_square_quantile inverts the chi-square CDF", "[math][stats]") {
    // Closed forms: k = 2 is exponential, P(chi2_2 <= x) = 1 - exp(-x/2).
    for (const double p : {0.01, 0.5, 0.95, 0.999})
        REQUIRE(std::abs(bm::chi_square_quantile(p, 2) + 2.0 * std::log(1.0 - p)) < 1e-9);
    for (const std::size_t k : {1u, 3u, 7u, 41u, 200u})
        for (const double p : {0.01, 0.05, 0.5, 0.95, 0.99}) {
            CAPTURE(k, p);
            const double x = bm::chi_square_quantile(p, k);
            REQUIRE(std::abs(bm::regularized_gamma_p(0.5 * static_cast<double>(k), 0.5 * x) - p) < 1e-10);
        }
}

TEST_CASE("chi_square_quantile rejects a bad level or dof", "[math][stats]") {
    REQUIRE_THROWS_AS(bm::chi_square_quantile(0.0, 3), std::invalid_argument);
    REQUIRE_THROWS_AS(bm::chi_square_quantile(1.0, 3), std::invalid_argument);
    REQUIRE_THROWS_AS(bm::chi_square_quantile(0.95, 0), std::invalid_argument);
    REQUIRE_THROWS_AS(bm::regularized_gamma_p(0.0, 1.0), std::invalid_argument);
}
