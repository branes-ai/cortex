// SPDX-License-Identifier: MIT
//
// branes/math/sparse_direct.hpp — direct sparse factorizations for SPD
// systems (Cholesky LLᵀ and LDLᵀ).
//
// Wraps MTL5's *native* sparse direct solvers (Davis up-looking Cholesky
// with elimination tree + fill-reducing ordering; no external SuiteSparse
// dependency, so it builds for every scalar type, on host and on the
// cross target). The math layer's contribution is a small, uniform
// front end that consumes the layer's own CsrView (the storage view the
// NLS normal equations are assembled into) and exposes a factor-once /
// solve-many interface.
//
// Cholesky requires a symmetric *positive-definite* matrix; LDLᵀ also
// handles symmetric indefinite systems. Both take the matrix in CSR.
//
// Header-only, C++20.

#ifndef BRANES_MATH_SPARSE_DIRECT_HPP
#define BRANES_MATH_SPARSE_DIRECT_HPP

#include <branes/math/sparse.hpp>

#include <mtl/mat/compressed2D.hpp>
#include <mtl/mat/inserter.hpp>
#include <mtl/sparse/factorization/sparse_cholesky.hpp>
#include <mtl/sparse/factorization/sparse_ldlt.hpp>
#include <mtl/vec/dense_vector.hpp>

#include <cstddef>
#include <span>

namespace branes::math {

namespace detail {

/// Build an MTL5 compressed2D (CSR-backed) from a CsrView.
template <Scalar T, std::integral IndexT>
mtl::mat::compressed2D<T> to_compressed2D(const CsrView<T, IndexT>& A) {
    mtl::mat::compressed2D<T> M(A.num_rows(), A.num_cols());
    {
        mtl::mat::inserter<mtl::mat::compressed2D<T>> ins(M);
        A.for_each([&](std::size_t i, std::size_t j, const T& v) { ins[i][j] << v; });
    }
    return M;
}

/// Copy a contiguous buffer into an MTL5 dense_vector.
template <Scalar T>
mtl::vec::dense_vector<T> to_dense_vector(std::span<const T> b) {
    mtl::vec::dense_vector<T> v(b.size());
    for (std::size_t i = 0; i < b.size(); ++i)
        v(i) = b[i];
    return v;
}

}  // namespace detail

/// Direct Cholesky (LLᵀ) solver for a symmetric positive-definite system.
/// Factors once at construction; `solve` reuses the factor for many
/// right-hand sides.
template <Scalar T>
class SparseCholesky {
public:
    /// Factorize an SPD matrix supplied in CSR.
    template <std::integral IndexT>
    explicit SparseCholesky(const CsrView<T, IndexT>& A) : num_(factor(A)) {}

    /// Solve A x = b. `b` and `x` are length-`dimension()` buffers.
    void solve(std::span<const T> b, std::span<T> x) const {
        const std::size_t n = num_.num_rows();
        auto bv = detail::to_dense_vector(b);
        mtl::vec::dense_vector<T> xv(n);
        num_.solve(xv, bv);
        for (std::size_t i = 0; i < n; ++i)
            x[i] = xv(i);
    }

    [[nodiscard]] std::size_t dimension() const {
        return num_.num_rows();
    }
    /// Nonzeros in the computed Cholesky factor L (fill included).
    [[nodiscard]] std::size_t factor_nnz() const {
        return num_.symbolic.nnz_L;
    }

private:
    template <std::integral IndexT>
    static mtl::sparse::factorization::cholesky_numeric<T> factor(const CsrView<T, IndexT>& A) {
        const auto M = detail::to_compressed2D(A);
        const auto sym = mtl::sparse::factorization::sparse_cholesky_symbolic(M);
        return mtl::sparse::factorization::sparse_cholesky_numeric<T>(M, sym);
    }

    mtl::sparse::factorization::cholesky_numeric<T> num_;
};

/// Direct LDLᵀ solver for a symmetric (possibly indefinite) system.
/// Same factor-once / solve-many interface as SparseCholesky.
template <Scalar T>
class SparseLdlt {
public:
    template <std::integral IndexT>
    explicit SparseLdlt(const CsrView<T, IndexT>& A) : num_(factor(A)) {}

    void solve(std::span<const T> b, std::span<T> x) const {
        const std::size_t n = num_.num_rows();
        auto bv = detail::to_dense_vector(b);
        mtl::vec::dense_vector<T> xv(n);
        num_.solve(xv, bv);
        for (std::size_t i = 0; i < n; ++i)
            x[i] = xv(i);
    }

    [[nodiscard]] std::size_t dimension() const {
        return num_.num_rows();
    }
    [[nodiscard]] std::size_t factor_nnz() const {
        return num_.symbolic.nnz_L;
    }

private:
    template <std::integral IndexT>
    static mtl::sparse::factorization::ldlt_numeric<T> factor(const CsrView<T, IndexT>& A) {
        const auto M = detail::to_compressed2D(A);
        const auto sym = mtl::sparse::factorization::sparse_ldlt_symbolic(M);
        return mtl::sparse::factorization::sparse_ldlt_numeric<T>(M, sym);
    }

    mtl::sparse::factorization::ldlt_numeric<T> num_;
};

}  // namespace branes::math

#endif  // BRANES_MATH_SPARSE_DIRECT_HPP
