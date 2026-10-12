// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/dense.hpp — a minimal dynamic dense matrix for the
// MSCKF covariance bookkeeping (the IMU+clone error-state covariance and
// the EKF update grow/shrink at runtime, so the fixed-size math-layer
// matrices don't fit). Row-major, type-generic, header-only. Not a
// general BLAS — just what the State/Propagator/StateHelper need.
//
// The arithmetic runs on MTL5 (#448): `DynMat` keeps its storage and its
// interface, and its operations are MTL5 kernels over zero-copy
// `mtl::mat::dense2D` views of that storage (`mtl_view`). Each ported
// operation reproduces the hand-rolled one it replaced bit for bit at
// T = double (the same terms, summed in the same order), which the stage
// benches and the EuRoC trajectories check (tools/src/bench_snapshot.cpp).

#ifndef BRANES_SDK_MSCKF_DENSE_HPP
#define BRANES_SDK_MSCKF_DENSE_HPP

#include <branes/math/arithmetic.hpp>  // math::Scalar

#include <mtl/mat/compressed2D.hpp>
#include <mtl/mat/dense2D.hpp>
#include <mtl/mat/operators.hpp>
#include <mtl/operation/cholesky.hpp>
#include <mtl/operation/mult.hpp>
#include <mtl/operation/trans.hpp>
#include <mtl/operation/trsm.hpp>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

namespace branes::sdk::msckf {

template <math::Scalar T>
struct DynMat {
    std::size_t rows = 0;
    std::size_t cols = 0;
    std::vector<T> d;

    DynMat() = default;
    DynMat(std::size_t r, std::size_t c) : rows(r), cols(c), d(r * c, T{0}) {}

    [[nodiscard]] T& operator()(std::size_t i, std::size_t j) {
        return d[i * cols + j];
    }
    [[nodiscard]] const T& operator()(std::size_t i, std::size_t j) const {
        return d[i * cols + j];
    }

    [[nodiscard]] static DynMat identity(std::size_t n) {
        DynMat m(n, n);
        for (std::size_t i = 0; i < n; ++i)
            m(i, i) = T{1};
        return m;
    }
};

/// Zero-copy MTL5 views of a `DynMat`'s row-major storage.
namespace mtl_view {
template <math::Scalar T>
using Dense = mtl::mat::dense2D<T>;  // row-major, the DynMat layout

template <math::Scalar T>
[[nodiscard]] Dense<T> of(DynMat<T>& m) {
    return Dense<T>(m.rows, m.cols, m.d.data());
}
/// Read-only use: MTL5 has no const external view, and nothing is written
/// through this one.
template <math::Scalar T>
[[nodiscard]] Dense<T> of(const DynMat<T>& m) {
    return Dense<T>(m.rows, m.cols, const_cast<T*>(m.d.data()));
}
}  // namespace mtl_view

/// A·B. The filter's left operands are often mostly zero: the transition F
/// is the identity on every clone row, G = [I; J] and I − KH are sparse. A
/// left operand under half dense is multiplied in compressed (CRS) form, so its
/// zeros cost nothing. Both kernels build every entry as zero + A(i,0)·B(0,j) +
/// A(i,1)·B(1,j) + … in ascending k (the CRS one over A's nonzeros only, whose
/// omitted terms are exact zeros), so they agree with each other and with the
/// hand-rolled product this replaced, bit for bit, as long as B is finite. A
/// zero A(i,k) times a non-finite B(k,j) is NaN, which the dense kernel adds
/// and the zero-skipping product never forms, so when A has zeros and B a
/// non-finite entry the CRS kernel runs whatever A's density.
template <math::Scalar T>
[[nodiscard]] DynMat<T> mul(const DynMat<T>& a, const DynMat<T>& b) {
    assert(a.cols == b.rows && "DynMat::mul: inner dimensions must match");
    DynMat<T> r(a.rows, b.cols);
    if (r.d.empty() || a.cols == 0)
        return r;
    auto R = mtl_view::of(r);
    std::size_t nnz = 0;
    for (const T& x : a.d)
        nnz += x == T{0} ? 0 : 1;
    // Classified, not computed (no x − x, which traps on NaR or FE_INVALID).
    // Unqualified, so a Universal type's own isfinite is found by ADL.
    const auto all_finite = [](const DynMat<T>& m) {
        using std::isfinite;
        for (const T& x : m.d)
            if (!isfinite(x))
                return false;
        return true;
    };
    if (2 * nnz >= a.d.size() && (nnz == a.d.size() || all_finite(b))) {
        mtl::mult(mtl_view::of(a), mtl_view::of(b), R);
        return r;
    }
    using Csr = mtl::mat::compressed2D<T>;
    using Index = typename Csr::size_type;
    std::vector<Index> starts(a.rows + 1, 0), cols;
    std::vector<T> vals;
    cols.reserve(nnz);
    vals.reserve(nnz);
    for (std::size_t i = 0; i < a.rows; ++i) {
        for (std::size_t k = 0; k < a.cols; ++k)
            if (a(i, k) != T{0}) {
                cols.push_back(static_cast<Index>(k));
                vals.push_back(a(i, k));
            }
        starts[i + 1] = static_cast<Index>(cols.size());
    }
    mtl::mult(Csr(a.rows, a.cols, nnz, starts.data(), cols.data(), vals.data()), mtl_view::of(b), R);
    return r;
}

template <math::Scalar T>
[[nodiscard]] DynMat<T> transpose(const DynMat<T>& a) {
    DynMat<T> r(a.cols, a.rows);
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t j = 0; j < a.cols; ++j)
            r(j, i) = a(i, j);
    return r;
}

template <math::Scalar T>
[[nodiscard]] DynMat<T> add(const DynMat<T>& a, const DynMat<T>& b) {
    assert(a.rows == b.rows && a.cols == b.cols && "DynMat::add: shape mismatch");
    DynMat<T> r(a.rows, a.cols);
    if (r.d.empty())
        return r;
    auto R = mtl_view::of(r);
    R = mtl_view::of(a) + mtl_view::of(b);
    return r;
}

/// Symmetrize in place: A ← (A + Aᵀ)/2. Counters drift that would
/// otherwise creep into the covariance over many updates.
template <math::Scalar T>
void symmetrize(DynMat<T>& a) {
    assert(a.rows == a.cols && "DynMat::symmetrize: matrix must be square");
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t j = i + 1; j < a.cols; ++j) {
            const T avg = (a(i, j) + a(j, i)) / T{2};
            a(i, j) = avg;
            a(j, i) = avg;
        }
}

/// Cholesky factor (lower L with A = L Lᵀ, zeros above the diagonal).
/// Returns false if A is not positive-definite to working precision,
/// including a NaN pivot. MTL5's `cholesky_factor` in its term-by-term order
/// (stillwater-sc/mtl5#556): each entry starts from a(i,j) and subtracts
/// L(i,k)·L(j,k) in ascending k, as the hand-rolled factor did, so the bits
/// are the same. The default order, a − (Σ products), rounds differently,
/// and on EuRoC V2_03 that flips 2 of ~16k update decisions.
template <math::Scalar T>
[[nodiscard]] bool cholesky(const DynMat<T>& a, DynMat<T>& L) {
    assert(a.rows == a.cols && "DynMat::cholesky: matrix must be square");
    const std::size_t n = a.rows;
    L = a;
    if (n == 0)
        return true;
    auto Lv = mtl_view::of(L);
    if (mtl::cholesky_factor(Lv, mtl::tag::subtract_term_by_term{}) != 0)
        return false;
    for (std::size_t j = 1; j < n; ++j)
        for (std::size_t i = 0; i < j; ++i)
            L(i, j) = T{0};  // MTL5 leaves the upper triangle as it was
    return true;
}

/// True iff `a` is symmetric positive-definite to working precision.
template <math::Scalar T>
[[nodiscard]] bool is_positive_definite(const DynMat<T>& a) {
    DynMat<T> L;
    return cholesky(a, L);
}

/// True iff `a` is symmetric positive-*semi*-definite (no negative
/// eigenvalues) to within a small ridge. MSCKF clone augmentation makes
/// the covariance singular at that instant (the clone is an exact copy of
/// the IMU pose), so PSD — not strict PD — is the invariant there. The ridge on
/// each diagonal entry is floored at a few ulps of that entry, so a coarse type
/// still has one (#449).
template <math::Scalar T>
[[nodiscard]] bool is_positive_semidefinite(const DynMat<T>& a, T ridge = T(1e-9)) {
    DynMat<T> b = a;
    for (std::size_t i = 0; i < b.rows; ++i)
        b(i, i) += math::at_least_roundoff(ridge, a(i, i));
    DynMat<T> L;
    return cholesky(b, L);
}

/// Solve A X = B given a precomputed lower Cholesky factor L (A = L Lᵀ).
/// Lets a caller that already factored A (e.g. to test definiteness) reuse
/// the factor instead of re-factoring inside `spd_solve`. Two MTL5 triangular
/// solves, forward (L Y = B) then back (Lᵀ X = Y), per RHS column, each entry
/// as b − L(i,0)·x(0) − L(i,1)·x(1) − … in ascending order: the hand-rolled
/// substitution this replaced, bit for bit.
template <math::Scalar T>
[[nodiscard]] DynMat<T> cholesky_solve(const DynMat<T>& L, const DynMat<T>& b) {
    assert(L.rows == L.cols && L.rows == b.rows && "cholesky_solve: shape mismatch");
    DynMat<T> x = b;
    if (x.d.empty())
        return x;
    auto X = mtl_view::of(x);
    const auto Lv = mtl_view::of(L);
    mtl::trsm(T{1}, Lv, X, /*upper=*/false);
    mtl::trsm(T{1}, mtl::trans(Lv), X, /*upper=*/true);
    return x;
}

/// Solve the SPD system A X = B (A n×n, B n×k) via the Cholesky factor.
/// Precondition: A is positive-definite.
template <math::Scalar T>
[[nodiscard]] DynMat<T> spd_solve(const DynMat<T>& a, const DynMat<T>& b) {
    assert(a.rows == a.cols && a.rows == b.rows && "spd_solve: shape mismatch");
    DynMat<T> L;
    if (!cholesky(a, L)) {
        // Precondition violated (A not PD). Fail loudly in debug; in
        // release return zeros so a degenerate update is a no-op, not UB.
        assert(false && "spd_solve: A is not positive-definite");
        return DynMat<T>(a.rows, b.cols);
    }
    return cholesky_solve(L, b);
}

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_DENSE_HPP
