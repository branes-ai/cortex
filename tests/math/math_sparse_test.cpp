// Sparse-view element-access + conversion tests (issue #26).
//
// Builds a ~50x50 reference matrix in dense form and in CSR/CSC/COO
// views over caller-owned buffers, then asserts that every element read
// through each sparse view matches the dense reference. Also exercises
// the allocation-free conversions (CSR→CSC, CSC→CSR, COO→CSR) and
// confirms the converted views reproduce the same dense matrix.

#include <branes/math/sparse.hpp>

#include <catch2/catch_test_macros.hpp>
#include <universal/number/posit/posit.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace {

using Index = branes::math::SparseIndex;
constexpr std::size_t N = 50;

// A deterministic, moderately sparse 50x50 pattern: nonzero where
// (3*i + 7*j) % 11 == 0, value = i*100 + j + 1 (never zero, so the
// pattern is unambiguous). Returns the dense reference (row-major).
std::vector<double> make_dense_reference() {
    std::vector<double> d(N * N, 0.0);
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            if ((3 * i + 7 * j) % 11 == 0) {
                d[i * N + j] = static_cast<double>(i * 100 + j + 1);
            }
        }
    }
    return d;
}

// Assemble CSR arrays (row-major scan) from the dense reference.
struct CsrArrays {
    std::vector<double> values;
    std::vector<Index> col_indices;
    std::vector<Index> row_ptr;
};
CsrArrays dense_to_csr(const std::vector<double>& d) {
    CsrArrays a;
    a.row_ptr.push_back(0);
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            const double v = d[i * N + j];
            if (v != 0.0) {
                a.values.push_back(v);
                a.col_indices.push_back(static_cast<Index>(j));
            }
        }
        a.row_ptr.push_back(static_cast<Index>(a.values.size()));
    }
    return a;
}

}  // namespace

TEST_CASE("CSR view matches dense reference element-by-element", "[math][sparse]") {
    const auto dense = make_dense_reference();
    auto a = dense_to_csr(dense);
    branes::math::CsrView<double> csr(
        N, N, std::span<double>{a.values}, std::span<const Index>{a.col_indices}, std::span<const Index>{a.row_ptr});

    REQUIRE(csr.num_rows() == N);
    REQUIRE(csr.num_cols() == N);
    REQUIRE(csr.nnz() == a.values.size());

    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            REQUIRE(csr(i, j) == dense[i * N + j]);
        }
    }
}

TEST_CASE("COO view matches dense reference (with duplicate accumulation)", "[math][sparse]") {
    const auto dense = make_dense_reference();
    // Build COO in scrambled order, and split one entry into two halves
    // to exercise additive-duplicate element access.
    std::vector<double> vals;
    std::vector<Index> rows;
    std::vector<Index> cols;
    bool split_done = false;
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t jj = 0; jj < N; ++jj) {
            const std::size_t j = (N - 1) - jj;  // reversed column order
            const double v = dense[i * N + j];
            if (v == 0.0)
                continue;
            if (!split_done) {
                vals.push_back(v / 2.0);
                rows.push_back(static_cast<Index>(i));
                cols.push_back(static_cast<Index>(j));
                vals.push_back(v / 2.0);  // duplicate coordinate
                rows.push_back(static_cast<Index>(i));
                cols.push_back(static_cast<Index>(j));
                split_done = true;
            } else {
                vals.push_back(v);
                rows.push_back(static_cast<Index>(i));
                cols.push_back(static_cast<Index>(j));
            }
        }
    }
    branes::math::CooView<double> coo(
        N, N, std::span<double>{vals}, std::span<const Index>{rows}, std::span<const Index>{cols});

    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            REQUIRE(coo(i, j) == dense[i * N + j]);
        }
    }
}

TEST_CASE("CSR -> CSC conversion reproduces the dense matrix", "[math][sparse]") {
    const auto dense = make_dense_reference();
    auto a = dense_to_csr(dense);
    branes::math::CsrView<double> csr(
        N, N, std::span<double>{a.values}, std::span<const Index>{a.col_indices}, std::span<const Index>{a.row_ptr});

    const std::size_t nnz = csr.nnz();
    std::vector<double> cval(nnz);
    std::vector<Index> crow(nnz);
    std::vector<Index> cptr(N + 1);
    auto csc = branes::math::csr_to_csc(csr, std::span<double>{cval}, std::span<Index>{crow}, std::span<Index>{cptr});

    REQUIRE(csc.nnz() == nnz);
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            REQUIRE(csc(i, j) == dense[i * N + j]);
        }
    }

    // Round-trip CSC -> CSR and compare again.
    std::vector<double> rval(nnz);
    std::vector<Index> rcol(nnz);
    std::vector<Index> rptr(N + 1);
    auto csr2 = branes::math::csc_to_csr(csc, std::span<double>{rval}, std::span<Index>{rcol}, std::span<Index>{rptr});
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            REQUIRE(csr2(i, j) == dense[i * N + j]);
        }
    }
    // Round-trip preserves the original CSR structure exactly.
    REQUIRE(std::equal(rptr.begin(), rptr.end(), a.row_ptr.begin()));
    REQUIRE(std::equal(rcol.begin(), rcol.end(), a.col_indices.begin()));
}

TEST_CASE("COO -> CSR conversion reproduces the dense matrix", "[math][sparse]") {
    const auto dense = make_dense_reference();
    // COO in column-reversed order so the sort must reorder entries.
    std::vector<double> vals;
    std::vector<Index> rows;
    std::vector<Index> cols;
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t jj = 0; jj < N; ++jj) {
            const std::size_t j = (N - 1) - jj;
            const double v = dense[i * N + j];
            if (v == 0.0)
                continue;
            vals.push_back(v);
            rows.push_back(static_cast<Index>(i));
            cols.push_back(static_cast<Index>(j));
        }
    }
    branes::math::CooView<double> coo(
        N, N, std::span<double>{vals}, std::span<const Index>{rows}, std::span<const Index>{cols});
    const std::size_t nnz = coo.nnz();
    std::vector<double> rval(nnz);
    std::vector<Index> rcol(nnz);
    std::vector<Index> rptr(N + 1);
    auto csr = branes::math::coo_to_csr(coo, std::span<double>{rval}, std::span<Index>{rcol}, std::span<Index>{rptr});

    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = 0; j < N; ++j) {
            REQUIRE(csr(i, j) == dense[i * N + j]);
        }
    }
}

TEST_CASE("for_each visits exactly the stored nonzeros", "[math][sparse]") {
    const auto dense = make_dense_reference();
    auto a = dense_to_csr(dense);
    branes::math::CsrView<double> csr(
        N, N, std::span<double>{a.values}, std::span<const Index>{a.col_indices}, std::span<const Index>{a.row_ptr});
    std::size_t count = 0;
    double sum = 0.0;
    csr.for_each([&](std::size_t i, std::size_t j, double v) {
        REQUIRE(dense[i * N + j] == v);
        ++count;
        sum += v;
    });
    REQUIRE(count == csr.nnz());
    double dense_sum = 0.0;
    for (double v : dense)
        dense_sum += v;
    REQUIRE(sum == dense_sum);
}

TEST_CASE("sparse view works over a Universal posit value type", "[math][sparse]") {
    using posit32 = sw::universal::posit<32, 2>;
    // 2x2 identity-ish: [[2,0],[0,3]].
    std::array<posit32, 2> vals{posit32{2}, posit32{3}};
    std::array<Index, 2> cols{0, 1};
    std::array<Index, 3> rptr{0, 1, 2};
    branes::math::CsrView<posit32> csr(
        2, 2, std::span<posit32>{vals}, std::span<const Index>{cols}, std::span<const Index>{rptr});
    REQUIRE(double(csr(0, 0)) == 2.0);
    REQUIRE(double(csr(0, 1)) == 0.0);  // implicit zero
    REQUIRE(double(csr(1, 1)) == 3.0);
}
