// SPDX-License-Identifier: MIT
//
// branes/sdk/features/msckf_nullspace.hpp — MSCKF left-null-space
// projection. Clean-room from the MSCKF measurement model (Mourikis &
// Roumeliotis, 2007): to remove a feature from a stacked measurement
//
//     r = H_x · δx + H_f · δf + n,   H_f ∈ R^{m×3}, m > 3,
//
// left-multiply by Nᵀ, where the columns of N span the left null space of
// H_f (NᵀH_f = 0). The compressed system Nᵀr = NᵀH_x·δx + Nᵀn no longer
// depends on the feature. Implemented via Householder QR of H_f applied
// to the augmented [H_x | r]; the bottom m−3 rows are the projection.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_FEATURES_MSCKF_NULLSPACE_HPP
#define BRANES_SDK_FEATURES_MSCKF_NULLSPACE_HPP

#include <branes/math/arithmetic.hpp>  // math::Scalar

#include <mtl/mat/dense2D.hpp>
#include <mtl/operation/householder.hpp>
#include <mtl/tag/reflector_form.hpp>
#include <mtl/vec/dense_vector.hpp>

#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

namespace branes::sdk::features {

/// Result of marginalizing the feature: the compressed measurement
/// Jacobian `H_x` ((m−3)×n, row-major) and residual `r` (m−3).
template <math::Scalar T>
struct NullspaceProjection {
    std::vector<T> H_x;
    std::vector<T> r;
    std::size_t rows = 0;
    std::size_t cols = 0;
};

/// Project a stacked measurement onto the left null space of the feature
/// Jacobian. `H_f` is row-major m×3, `H_x` is row-major m×n, `r` is m.
/// Requires m > 3; otherwise the result is empty (nothing to constrain).
template <math::Scalar T>
[[nodiscard]] NullspaceProjection<T> msckf_left_nullspace_project(
    std::span<const T> H_f, std::span<const T> H_x, std::span<const T> r, std::size_t m, std::size_t n) {
    NullspaceProjection<T> out;
    if (m <= 3)
        return out;
    // The spans must hold the declared shapes before we index them.
    if (H_f.size() < m * 3 || H_x.size() < m * n || r.size() < m) {
        throw std::invalid_argument("msckf_left_nullspace_project: span smaller than m*3 / m*n / m");
    }

    // Augmented matrix M = [H_f | H_x | r], row-major, m × (3 + n + 1).
    const std::size_t cols = 3 + n + 1;
    std::vector<T> mtx(m * cols, T{0});
    for (std::size_t i = 0; i < m; ++i) {
        for (std::size_t j = 0; j < 3; ++j)
            mtx[i * cols + j] = H_f[i * 3 + j];
        for (std::size_t j = 0; j < n; ++j)
            mtx[i * cols + 3 + j] = H_x[i * n + j];
        mtx[i * cols + 3 + n] = r[i];
    }

    // Householder QR: zero the sub-diagonal of the first 3 (H_f) columns. MTL5's
    // textbook reflector (stillwater-sc/mtl5#559): unscaled v = x − α·e₁,
    // α = −sign(x_c)·‖x‖, each column updated as A(:,j) −= ((2·vᵀA(:,j))/vᵀv)·v.
    // Columns c+1.. get each reflector: the later H_f columns, then [H_x | r].
    // Column c and those before it are not read again.
    mtl::mat::dense2D<T> M(m, cols, mtx.data());
    for (std::size_t c = 0; c < 3; ++c) {
        mtl::vec::dense_vector<T> x(m - c);
        for (std::size_t i = c; i < m; ++i)
            x(i - c) = mtx[i * cols + c];
        const auto h = mtl::householder(x, mtl::tag::textbook_reflector{});
        mtl::apply_householder_left(M, h.v, h.vtv, c, c + 1, mtl::tag::textbook_reflector{});
    }

    // Rows 3..m-1 of the [H_x | r] block are the feature-free system.
    out.rows = m - 3;
    out.cols = n;
    out.H_x.resize(out.rows * n);
    out.r.resize(out.rows);
    for (std::size_t i = 0; i < out.rows; ++i) {
        const std::size_t src = (i + 3) * cols;
        for (std::size_t j = 0; j < n; ++j)
            out.H_x[i * n + j] = mtx[src + 3 + j];
        out.r[i] = mtx[src + 3 + n];
    }
    return out;
}

}  // namespace branes::sdk::features

#endif  // BRANES_SDK_FEATURES_MSCKF_NULLSPACE_HPP
