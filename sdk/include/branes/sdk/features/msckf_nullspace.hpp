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

#include <branes/math/lie/detail.hpp>  // scalar sqrt_/abs_

#include <cstddef>
#include <span>
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
    using math::lie::detail::sqrt_;

    NullspaceProjection<T> out;
    if (m <= 3)
        return out;

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

    // Householder QR: zero the sub-diagonal of the first 3 (H_f) columns,
    // applying each reflector across the whole augmented row.
    std::vector<T> v(m);
    for (std::size_t c = 0; c < 3; ++c) {
        T norm2 = T{0};
        for (std::size_t i = c; i < m; ++i) {
            const T x = mtx[i * cols + c];
            norm2 = norm2 + x * x;
        }
        const T norm = sqrt_(norm2);
        if (norm <= T{0})
            continue;
        const T diag = mtx[c * cols + c];
        const T alpha = (diag >= T{0}) ? -norm : norm;  // avoid cancellation
        for (std::size_t i = c; i < m; ++i)
            v[i] = mtx[i * cols + c];
        v[c] -= alpha;
        T vtv = T{0};
        for (std::size_t i = c; i < m; ++i)
            vtv = vtv + v[i] * v[i];
        if (vtv <= T{0})
            continue;
        for (std::size_t j = 0; j < cols; ++j) {
            T s = T{0};
            for (std::size_t i = c; i < m; ++i)
                s = s + v[i] * mtx[i * cols + j];
            const T f = (T{2} * s) / vtv;
            for (std::size_t i = c; i < m; ++i)
                mtx[i * cols + j] -= f * v[i];
        }
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
