// Direct sparse factorization tests (issue #28).
//
// Builds a 2D 5-point Laplacian (symmetric positive definite) in both a
// dense layout and a CSR view, solves A x = b with branes::math::
// SparseCholesky, and requires the result to match a dense Cholesky
// reference solve (and the known solution) to working precision. Also
// checks LDLᵀ agreement and a float instantiation.

#include <branes/math/sparse_direct.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

using Index = branes::math::SparseIndex;

// 2D 5-point Laplacian on a g×g grid → N = g·g, SPD: diagonal 4,
// in-grid neighbours -1. Returned dense, row-major.
std::vector<double> laplacian_dense(std::size_t g) {
    const std::size_t N = g * g;
    std::vector<double> A(N * N, 0.0);
    auto at = [&](std::size_t r, std::size_t c) -> double& { return A[r * N + c]; };
    for (std::size_t i = 0; i < g; ++i) {
        for (std::size_t j = 0; j < g; ++j) {
            const std::size_t r = i * g + j;
            at(r, r) = 4.0;
            if (j + 1 < g) {
                at(r, r + 1) = -1.0;
                at(r + 1, r) = -1.0;
            }
            if (i + 1 < g) {
                at(r, r + g) = -1.0;
                at(r + g, r) = -1.0;
            }
        }
    }
    return A;
}

struct Csr {
    std::vector<double> values;
    std::vector<Index> col_indices;
    std::vector<Index> row_ptr;
};
Csr dense_to_csr(const std::vector<double>& A, std::size_t N) {
    Csr c;
    c.row_ptr.push_back(0);
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            const double v = A[i * N + j];
            if (v != 0.0) {
                c.values.push_back(v);
                c.col_indices.push_back(static_cast<Index>(j));
            }
        }
        c.row_ptr.push_back(static_cast<Index>(c.values.size()));
    }
    return c;
}

// Dense Cholesky reference: factor A = L Lᵀ (column-major access into a
// row-major buffer is fine since A is symmetric) and solve A x = b.
std::vector<double> dense_cholesky_solve(std::vector<double> A, std::size_t N, std::vector<double> b) {
    auto at = [&](std::size_t r, std::size_t c) -> double& { return A[r * N + c]; };
    std::vector<double> L(N * N, 0.0);
    auto Lat = [&](std::size_t r, std::size_t c) -> double& { return L[r * N + c]; };
    for (std::size_t j = 0; j < N; ++j) {
        double sum = at(j, j);
        for (std::size_t k = 0; k < j; ++k)
            sum -= Lat(j, k) * Lat(j, k);
        Lat(j, j) = std::sqrt(sum);
        for (std::size_t i = j + 1; i < N; ++i) {
            double s = at(i, j);
            for (std::size_t k = 0; k < j; ++k)
                s -= Lat(i, k) * Lat(j, k);
            Lat(i, j) = s / Lat(j, j);
        }
    }
    // Forward solve L y = b.
    std::vector<double> y(N, 0.0);
    for (std::size_t i = 0; i < N; ++i) {
        double s = b[i];
        for (std::size_t k = 0; k < i; ++k)
            s -= Lat(i, k) * y[k];
        y[i] = s / Lat(i, i);
    }
    // Back solve Lᵀ x = y.
    std::vector<double> x(N, 0.0);
    for (std::size_t ii = N; ii-- > 0;) {
        double s = y[ii];
        for (std::size_t k = ii + 1; k < N; ++k)
            s -= Lat(k, ii) * x[k];
        x[ii] = s / Lat(ii, ii);
    }
    return x;
}

}  // namespace

TEST_CASE("Sparse Cholesky matches a dense reference solve", "[math][sparse-direct]") {
    constexpr std::size_t g = 6;
    constexpr std::size_t N = g * g;
    const auto dense = laplacian_dense(g);
    auto csr = dense_to_csr(dense, N);

    // Known solution and its right-hand side.
    std::vector<double> x_true(N);
    for (std::size_t i = 0; i < N; ++i)
        x_true[i] = 0.5 + 0.3 * double(i);
    std::vector<double> b(N, 0.0);
    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j)
            b[i] += dense[i * N + j] * x_true[j];

    branes::math::CsrView<double> A(N,
                                    N,
                                    std::span<double>{csr.values},
                                    std::span<const Index>{csr.col_indices},
                                    std::span<const Index>{csr.row_ptr});
    branes::math::SparseCholesky<double> chol(A);
    REQUIRE(chol.dimension() == N);

    std::vector<double> x(N, 0.0);
    chol.solve(std::span<const double>{b}, std::span<double>{x});

    const auto x_ref = dense_cholesky_solve(dense, N, b);
    for (std::size_t i = 0; i < N; ++i) {
        REQUIRE(std::abs(x[i] - x_ref[i]) < 1e-10);  // matches dense reference
        REQUIRE(std::abs(x[i] - x_true[i]) < 1e-9);  // matches exact solution
    }
}

TEST_CASE("Sparse LDLT agrees with sparse Cholesky on an SPD system", "[math][sparse-direct]") {
    constexpr std::size_t g = 5;
    constexpr std::size_t N = g * g;
    const auto dense = laplacian_dense(g);
    auto csr = dense_to_csr(dense, N);

    std::vector<double> b(N, 1.0);
    branes::math::CsrView<double> A(N,
                                    N,
                                    std::span<double>{csr.values},
                                    std::span<const Index>{csr.col_indices},
                                    std::span<const Index>{csr.row_ptr});

    branes::math::SparseCholesky<double> chol(A);
    branes::math::SparseLdlt<double> ldlt(A);
    std::vector<double> xc(N, 0.0), xl(N, 0.0);
    chol.solve(std::span<const double>{b}, std::span<double>{xc});
    ldlt.solve(std::span<const double>{b}, std::span<double>{xl});
    for (std::size_t i = 0; i < N; ++i)
        REQUIRE(std::abs(xc[i] - xl[i]) < 1e-10);
}

TEST_CASE("Sparse Cholesky instantiates for float", "[math][sparse-direct]") {
    constexpr std::size_t g = 4;
    constexpr std::size_t N = g * g;
    const auto dense = laplacian_dense(g);
    auto csr_d = dense_to_csr(dense, N);
    // Narrow the CSR values to float.
    std::vector<float> values(csr_d.values.begin(), csr_d.values.end());

    std::vector<double> x_true(N, 1.0);
    std::vector<float> b(N, 0.0f);
    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j)
            b[i] += static_cast<float>(dense[i * N + j] * x_true[j]);

    branes::math::CsrView<float> A(N,
                                   N,
                                   std::span<float>{values},
                                   std::span<const Index>{csr_d.col_indices},
                                   std::span<const Index>{csr_d.row_ptr});
    branes::math::SparseCholesky<float> chol(A);
    std::vector<float> x(N, 0.0f);
    chol.solve(std::span<const float>{b}, std::span<float>{x});
    for (std::size_t i = 0; i < N; ++i)
        REQUIRE(std::abs(x[i] - 1.0f) < 1e-3f);
}

TEST_CASE("solve rejects mismatched buffer sizes", "[math][sparse-direct]") {
    constexpr std::size_t g = 3;
    constexpr std::size_t N = g * g;
    const auto dense = laplacian_dense(g);
    auto csr = dense_to_csr(dense, N);
    branes::math::CsrView<double> A(N,
                                    N,
                                    std::span<double>{csr.values},
                                    std::span<const Index>{csr.col_indices},
                                    std::span<const Index>{csr.row_ptr});
    branes::math::SparseCholesky<double> chol(A);

    std::vector<double> b(N, 1.0);
    std::vector<double> x_short(N - 1, 0.0);  // output too small
    REQUIRE_THROWS_AS(chol.solve(std::span<const double>{b}, std::span<double>{x_short}), std::invalid_argument);

    std::vector<double> b_short(N - 1, 1.0);  // rhs wrong length
    std::vector<double> x(N, 0.0);
    REQUIRE_THROWS_AS(chol.solve(std::span<const double>{b_short}, std::span<double>{x}), std::invalid_argument);
}

TEST_CASE("LDLT solve rejects mismatched buffer sizes", "[math][sparse-direct]") {
    constexpr std::size_t g = 3;
    constexpr std::size_t N = g * g;
    const auto dense = laplacian_dense(g);
    auto csr = dense_to_csr(dense, N);
    branes::math::CsrView<double> A(N,
                                    N,
                                    std::span<double>{csr.values},
                                    std::span<const Index>{csr.col_indices},
                                    std::span<const Index>{csr.row_ptr});
    branes::math::SparseLdlt<double> ldlt(A);

    std::vector<double> b(N, 1.0);
    std::vector<double> x_short(N - 1, 0.0);  // output too small
    REQUIRE_THROWS_AS(ldlt.solve(std::span<const double>{b}, std::span<double>{x_short}), std::invalid_argument);

    std::vector<double> b_short(N - 1, 1.0);  // rhs wrong length
    std::vector<double> x(N, 0.0);
    REQUIRE_THROWS_AS(ldlt.solve(std::span<const double>{b_short}, std::span<double>{x}), std::invalid_argument);
}
