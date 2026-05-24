// SPDX-License-Identifier: MIT
//
// branes/math/sparse.hpp — non-owning sparse-matrix views.
//
// CSR, CSC, and COO views over caller-owned index/value buffers, in the
// same spirit as TensorView: the math layer never owns sparse storage,
// it windows onto buffers supplied by the caller (and, ultimately, by
// the Rust Resource Manager across the cxx bridge). Indices and values
// are plain spans; the view adds shape, element lookup, and traversal.
//
// Conversions (CSR↔CSC, COO→CSR) are allocation-free: the caller sizes
// and supplies the destination spans, and the helper fills them and
// returns a view onto them. This keeps the whole layer heap-free on the
// hot path — sizing is a pure function of nnz / rows / cols.
//
// Header-only, C++20.

#ifndef BRANES_MATH_SPARSE_HPP
#define BRANES_MATH_SPARSE_HPP

#include <branes/math/arithmetic.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <type_traits>
#include <utility>

namespace branes::math {

/// Default index type for sparse structure arrays. Signed 32-bit
/// matches the common SuiteSparse / SciPy convention and is ample for
/// the on-device problem sizes; override via the IndexT template arg.
using SparseIndex = std::int32_t;

// ── CSR ─────────────────────────────────────────────────────────────

/// Compressed Sparse Row view. Structure:
///   - `values`      : nnz element values, row-major by row then stored order
///   - `col_indices` : nnz column indices, parallel to `values`
///   - `row_ptr`     : rows+1 offsets; row i occupies
///                     [row_ptr[i], row_ptr[i+1]) of values/col_indices
///
/// Invariant: `row_ptr.size() == rows+1`, `row_ptr[0] == 0`,
/// `row_ptr[rows] == values.size() == col_indices.size() == nnz`.
template <Scalar T, std::integral IndexT = SparseIndex>
class CsrView {
public:
    using value_type = std::remove_cv_t<T>;
    using index_type = IndexT;

    constexpr CsrView() noexcept = default;

    constexpr CsrView(std::size_t rows,
                      std::size_t cols,
                      std::span<T> values,
                      std::span<const IndexT> col_indices,
                      std::span<const IndexT> row_ptr) noexcept
        : values_(values), col_indices_(col_indices), row_ptr_(row_ptr), rows_(rows), cols_(cols) {}

    [[nodiscard]] constexpr std::size_t num_rows() const noexcept {
        return rows_;
    }
    [[nodiscard]] constexpr std::size_t num_cols() const noexcept {
        return cols_;
    }
    [[nodiscard]] constexpr std::size_t nnz() const noexcept {
        return values_.size();
    }
    [[nodiscard]] constexpr std::span<T> values() const noexcept {
        return values_;
    }
    [[nodiscard]] constexpr std::span<const IndexT> col_indices() const noexcept {
        return col_indices_;
    }
    [[nodiscard]] constexpr std::span<const IndexT> row_ptr() const noexcept {
        return row_ptr_;
    }

    /// Half-open [begin, end) offset range of row `i` within values().
    [[nodiscard]] constexpr std::pair<std::size_t, std::size_t> row_range(std::size_t i) const noexcept {
        return {static_cast<std::size_t>(row_ptr_[i]), static_cast<std::size_t>(row_ptr_[i + 1])};
    }

    /// Element (i, j); returns a stored value or `T{0}` if absent.
    [[nodiscard]] constexpr value_type operator()(std::size_t i, std::size_t j) const noexcept {
        const auto [b, e] = row_range(i);
        for (std::size_t k = b; k < e; ++k) {
            if (static_cast<std::size_t>(col_indices_[k]) == j) {
                return values_[k];
            }
        }
        return value_type{0};
    }

    /// Visit every stored entry in row-major order: f(row, col, value).
    template <class F>
    constexpr void for_each(F&& f) const {
        for (std::size_t i = 0; i < rows_; ++i) {
            const auto [b, e] = row_range(i);
            for (std::size_t k = b; k < e; ++k) {
                f(i, static_cast<std::size_t>(col_indices_[k]), values_[k]);
            }
        }
    }

private:
    std::span<T> values_{};
    std::span<const IndexT> col_indices_{};
    std::span<const IndexT> row_ptr_{};
    std::size_t rows_ = 0;
    std::size_t cols_ = 0;
};

// ── CSC ─────────────────────────────────────────────────────────────

/// Compressed Sparse Column view. Mirror of CsrView with column-major
/// structure: `row_indices` parallel to `values`, `col_ptr` of size
/// cols+1 delimiting each column's slice.
template <Scalar T, std::integral IndexT = SparseIndex>
class CscView {
public:
    using value_type = std::remove_cv_t<T>;
    using index_type = IndexT;

    constexpr CscView() noexcept = default;

    constexpr CscView(std::size_t rows,
                      std::size_t cols,
                      std::span<T> values,
                      std::span<const IndexT> row_indices,
                      std::span<const IndexT> col_ptr) noexcept
        : values_(values), row_indices_(row_indices), col_ptr_(col_ptr), rows_(rows), cols_(cols) {}

    [[nodiscard]] constexpr std::size_t num_rows() const noexcept {
        return rows_;
    }
    [[nodiscard]] constexpr std::size_t num_cols() const noexcept {
        return cols_;
    }
    [[nodiscard]] constexpr std::size_t nnz() const noexcept {
        return values_.size();
    }
    [[nodiscard]] constexpr std::span<T> values() const noexcept {
        return values_;
    }
    [[nodiscard]] constexpr std::span<const IndexT> row_indices() const noexcept {
        return row_indices_;
    }
    [[nodiscard]] constexpr std::span<const IndexT> col_ptr() const noexcept {
        return col_ptr_;
    }

    /// Half-open [begin, end) offset range of column `j`.
    [[nodiscard]] constexpr std::pair<std::size_t, std::size_t> col_range(std::size_t j) const noexcept {
        return {static_cast<std::size_t>(col_ptr_[j]), static_cast<std::size_t>(col_ptr_[j + 1])};
    }

    [[nodiscard]] constexpr value_type operator()(std::size_t i, std::size_t j) const noexcept {
        const auto [b, e] = col_range(j);
        for (std::size_t k = b; k < e; ++k) {
            if (static_cast<std::size_t>(row_indices_[k]) == i) {
                return values_[k];
            }
        }
        return value_type{0};
    }

    /// Visit every stored entry in column-major order: f(row, col, value).
    template <class F>
    constexpr void for_each(F&& f) const {
        for (std::size_t j = 0; j < cols_; ++j) {
            const auto [b, e] = col_range(j);
            for (std::size_t k = b; k < e; ++k) {
                f(static_cast<std::size_t>(row_indices_[k]), j, values_[k]);
            }
        }
    }

private:
    std::span<T> values_{};
    std::span<const IndexT> row_indices_{};
    std::span<const IndexT> col_ptr_{};
    std::size_t rows_ = 0;
    std::size_t cols_ = 0;
};

// ── COO ─────────────────────────────────────────────────────────────

/// Coordinate-list view: three parallel nnz-length arrays (row index,
/// column index, value), in arbitrary order. Duplicate coordinates are
/// permitted and treated additively (matching the assembly convention
/// where overlapping contributions accumulate).
template <Scalar T, std::integral IndexT = SparseIndex>
class CooView {
public:
    using value_type = std::remove_cv_t<T>;
    using index_type = IndexT;

    constexpr CooView() noexcept = default;

    constexpr CooView(std::size_t rows,
                      std::size_t cols,
                      std::span<T> values,
                      std::span<const IndexT> row_indices,
                      std::span<const IndexT> col_indices) noexcept
        : values_(values), row_indices_(row_indices), col_indices_(col_indices), rows_(rows), cols_(cols) {}

    [[nodiscard]] constexpr std::size_t num_rows() const noexcept {
        return rows_;
    }
    [[nodiscard]] constexpr std::size_t num_cols() const noexcept {
        return cols_;
    }
    [[nodiscard]] constexpr std::size_t nnz() const noexcept {
        return values_.size();
    }
    [[nodiscard]] constexpr std::span<T> values() const noexcept {
        return values_;
    }
    [[nodiscard]] constexpr std::span<const IndexT> row_indices() const noexcept {
        return row_indices_;
    }
    [[nodiscard]] constexpr std::span<const IndexT> col_indices() const noexcept {
        return col_indices_;
    }

    /// Element (i, j): sum of all stored entries at that coordinate.
    [[nodiscard]] constexpr value_type operator()(std::size_t i, std::size_t j) const noexcept {
        value_type acc{0};
        for (std::size_t k = 0; k < values_.size(); ++k) {
            if (static_cast<std::size_t>(row_indices_[k]) == i && static_cast<std::size_t>(col_indices_[k]) == j) {
                acc = acc + values_[k];
            }
        }
        return acc;
    }

    /// Visit every stored entry in stored order: f(row, col, value).
    template <class F>
    constexpr void for_each(F&& f) const {
        for (std::size_t k = 0; k < values_.size(); ++k) {
            f(static_cast<std::size_t>(row_indices_[k]), static_cast<std::size_t>(col_indices_[k]), values_[k]);
        }
    }

private:
    std::span<T> values_{};
    std::span<const IndexT> row_indices_{};
    std::span<const IndexT> col_indices_{};
    std::size_t rows_ = 0;
    std::size_t cols_ = 0;
};

// ── Conversions (allocation-free; fill caller-provided destinations) ──
//
// All three use the classic in-place counting sort (cf. SciPy's
// csr_tocsc): the destination pointer array doubles as the per-bucket
// write cursor during the scatter, then is shifted back into a valid
// prefix-offset array. No auxiliary allocation, so the whole layer stays
// heap-free.

namespace detail {

/// Build exclusive prefix offsets in `out_ptr` (size nbuckets+1) from
/// the per-entry bucket map `bucket_of`, leaving `out_ptr[b]` = start
/// offset of bucket b and `out_ptr[nbuckets]` = nnz.
template <std::integral IndexT, class BucketFn>
constexpr void build_offsets(std::span<IndexT> out_ptr, std::size_t nnz, std::size_t nbuckets, BucketFn bucket_of) {
    std::fill(out_ptr.begin(), out_ptr.end(), IndexT{0});
    for (std::size_t k = 0; k < nnz; ++k) {
        ++out_ptr[bucket_of(k)];
    }
    IndexT sum = 0;
    for (std::size_t b = 0; b < nbuckets; ++b) {
        const IndexT cnt = out_ptr[b];
        out_ptr[b] = sum;
        sum += cnt;
    }
    out_ptr[nbuckets] = sum;
}

/// Undo the cursor advancement left by a scatter that incremented each
/// `out_ptr[b]` once per emitted entry, restoring prefix offsets.
template <std::integral IndexT>
constexpr void unshift_cursors(std::span<IndexT> out_ptr, std::size_t nbuckets) {
    IndexT last = 0;
    for (std::size_t b = 0; b <= nbuckets; ++b) {
        const IndexT tmp = out_ptr[b];
        out_ptr[b] = last;
        last = tmp;
    }
}

}  // namespace detail

/// Convert CSR → CSC. Destination spans must be sized: `out_values` and
/// `out_row_indices` to `csr.nnz()`, `out_col_ptr` to `cols+1`. Returns
/// a CscView over the destinations. Within each output column, entries
/// appear in ascending row order (the scatter visits rows in order).
template <Scalar T, std::integral IndexT>
[[nodiscard]] CscView<T, IndexT> csr_to_csc(const CsrView<T, IndexT>& csr,
                                            std::span<T> out_values,
                                            std::span<IndexT> out_row_indices,
                                            std::span<IndexT> out_col_ptr) {
    const std::size_t rows = csr.num_rows();
    const std::size_t cols = csr.num_cols();
    const auto col_idx = csr.col_indices();
    const auto vals = csr.values();

    detail::build_offsets<IndexT>(
        out_col_ptr, csr.nnz(), cols, [&](std::size_t k) { return static_cast<std::size_t>(col_idx[k]); });
    for (std::size_t i = 0; i < rows; ++i) {
        const auto [b, e] = csr.row_range(i);
        for (std::size_t k = b; k < e; ++k) {
            const std::size_t c = static_cast<std::size_t>(col_idx[k]);
            const std::size_t dst = static_cast<std::size_t>(out_col_ptr[c]++);
            out_values[dst] = vals[k];
            out_row_indices[dst] = static_cast<IndexT>(i);
        }
    }
    detail::unshift_cursors<IndexT>(out_col_ptr, cols);
    return CscView<T, IndexT>(rows, cols, out_values, out_row_indices, out_col_ptr);
}

/// Convert CSC → CSR. Symmetric to csr_to_csc: `out_values` /
/// `out_col_indices` sized to nnz, `out_row_ptr` to rows+1. Within each
/// output row, entries appear in ascending column order.
template <Scalar T, std::integral IndexT>
[[nodiscard]] CsrView<T, IndexT> csc_to_csr(const CscView<T, IndexT>& csc,
                                            std::span<T> out_values,
                                            std::span<IndexT> out_col_indices,
                                            std::span<IndexT> out_row_ptr) {
    const std::size_t rows = csc.num_rows();
    const std::size_t cols = csc.num_cols();
    const auto row_idx = csc.row_indices();
    const auto vals = csc.values();

    detail::build_offsets<IndexT>(
        out_row_ptr, csc.nnz(), rows, [&](std::size_t k) { return static_cast<std::size_t>(row_idx[k]); });
    for (std::size_t j = 0; j < cols; ++j) {
        const auto [b, e] = csc.col_range(j);
        for (std::size_t k = b; k < e; ++k) {
            const std::size_t r = static_cast<std::size_t>(row_idx[k]);
            const std::size_t dst = static_cast<std::size_t>(out_row_ptr[r]++);
            out_values[dst] = vals[k];
            out_col_indices[dst] = static_cast<IndexT>(j);
        }
    }
    detail::unshift_cursors<IndexT>(out_row_ptr, rows);
    return CsrView<T, IndexT>(rows, cols, out_values, out_col_indices, out_row_ptr);
}

/// Convert COO → CSR via a stable counting sort on row index.
/// `out_values` / `out_col_indices` sized to `coo.nnz()`, `out_row_ptr`
/// to rows+1. Duplicate coordinates are preserved as separate entries;
/// CsrView lookup returns the first match, so for additive-duplicate
/// semantics compact the COO before converting. Column order within a
/// row follows the COO's stored order (not sorted).
template <Scalar T, std::integral IndexT>
[[nodiscard]] CsrView<T, IndexT> coo_to_csr(const CooView<T, IndexT>& coo,
                                            std::span<T> out_values,
                                            std::span<IndexT> out_col_indices,
                                            std::span<IndexT> out_row_ptr) {
    const std::size_t rows = coo.num_rows();
    const std::size_t cols = coo.num_cols();
    const auto row_idx = coo.row_indices();
    const auto col_idx = coo.col_indices();
    const auto vals = coo.values();

    detail::build_offsets<IndexT>(
        out_row_ptr, coo.nnz(), rows, [&](std::size_t k) { return static_cast<std::size_t>(row_idx[k]); });
    for (std::size_t k = 0; k < coo.nnz(); ++k) {
        const std::size_t r = static_cast<std::size_t>(row_idx[k]);
        const std::size_t dst = static_cast<std::size_t>(out_row_ptr[r]++);
        out_values[dst] = vals[k];
        out_col_indices[dst] = col_idx[k];
    }
    detail::unshift_cursors<IndexT>(out_row_ptr, rows);
    return CsrView<T, IndexT>(rows, cols, out_values, out_col_indices, out_row_ptr);
}

}  // namespace branes::math

#endif  // BRANES_MATH_SPARSE_HPP
