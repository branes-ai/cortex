// SPDX-License-Identifier: MIT
//
// branes/math/tensor.hpp — non-owning tensor views over std::span.
//
// A TensorView is a lightweight, non-owning window onto a caller-owned
// buffer plus a DLPack-style shape/stride descriptor. It carries no
// ownership, performs no allocation, and never copies the underlying
// data — it is the math layer's universal hand-off type for the cxx
// bridge tensors coming from the Rust Resource Manager (raw pointer +
// shape; see docs/arch/cortex-repo.md).
//
// Layout follows DLPack semantics: strides are expressed in *elements*
// (not bytes), and any strided layout addressable from a single base
// pointer is representable. Row-major (C) and column-major (Fortran)
// contiguous layouts get convenience constructors.
//
// Header-only, C++20, type-generic: any trivially-copyable scalar
// (float, double, Universal posit/cfloat/fixpnt, ...) is a valid
// element type. The richer arithmetic concept plumbing lands in #25.

#ifndef BRANES_MATH_TENSOR_HPP
#define BRANES_MATH_TENSOR_HPP

#include <array>
#include <cstddef>
#include <span>
#include <type_traits>

namespace branes::math {

/// Element types a TensorView may address. Views only store and
/// relocate elements, so the sole requirement is trivial copyability —
/// satisfied by the built-in floats and by the Universal number types.
/// Arithmetic-operation concepts are layered on top in the solvers
/// (see issue #25), not baked into the storage view.
template <class T>
concept TensorScalar = std::is_trivially_copyable_v<std::remove_const_t<T>>;

/// Memory layout for the contiguous-construction convenience path.
enum class Layout {
    RowMajor,  ///< C order: last axis varies fastest (stride 1).
    ColMajor,  ///< Fortran order: first axis varies fastest (stride 1).
};

/// Extent of each axis (number of elements along the axis).
template <std::size_t Rank>
using Extents = std::array<std::size_t, Rank>;

/// Per-axis stride in *elements* (DLPack convention). Signed so that
/// reversed / negative-stride views remain representable.
template <std::size_t Rank>
using Strides = std::array<std::ptrdiff_t, Rank>;

/// Compute contiguous strides (in elements) for the given extents and
/// layout. RowMajor makes the last axis unit-stride; ColMajor the first.
template <std::size_t Rank>
[[nodiscard]] constexpr Strides<Rank> contiguous_strides(const Extents<Rank>& shape, Layout layout) noexcept {
    Strides<Rank> s{};
    if constexpr (Rank == 0) {
        return s;
    } else if (layout == Layout::RowMajor) {
        s[Rank - 1] = 1;
        for (std::size_t k = Rank - 1; k > 0; --k) {
            s[k - 1] = s[k] * static_cast<std::ptrdiff_t>(shape[k]);
        }
    } else {
        s[0] = 1;
        for (std::size_t k = 1; k < Rank; ++k) {
            s[k] = s[k - 1] * static_cast<std::ptrdiff_t>(shape[k - 1]);
        }
    }
    return s;
}

/// Number of elements addressed by `shape` (product of extents).
/// Rank-0 (scalar) views hold exactly one element.
template <std::size_t Rank>
[[nodiscard]] constexpr std::size_t extent_product(const Extents<Rank>& shape) noexcept {
    std::size_t n = 1;
    for (std::size_t k = 0; k < Rank; ++k) {
        n *= shape[k];
    }
    return n;
}

/// Smallest backing-span length (in elements) required to address every
/// element of `shape` under `strides` from a single base pointer. Used
/// as a bounds precondition by the TensorView constructors.
template <std::size_t Rank>
[[nodiscard]] constexpr std::size_t required_span_extent(const Extents<Rank>& shape,
                                                         const Strides<Rank>& strides) noexcept {
    for (std::size_t k = 0; k < Rank; ++k) {
        if (shape[k] == 0) {
            return 0;  // empty along some axis ⇒ addresses nothing.
        }
    }
    // Highest reachable offset is the sum over axes of the last valid
    // index times its (assumed non-negative, contiguous-case) stride.
    std::ptrdiff_t last = 0;
    for (std::size_t k = 0; k < Rank; ++k) {
        last += static_cast<std::ptrdiff_t>(shape[k] - 1) * strides[k];
    }
    return static_cast<std::size_t>(last) + 1;
}

/// Non-owning, multi-dimensional view over a caller-owned buffer.
///
/// @tparam T     Element type (TensorScalar). Use `const T` for a
///               read-only view; const-ness of the *view object* does
///               not propagate to elements — it behaves like a pointer.
/// @tparam Rank  Number of axes, fixed at compile time. Extents and
///               strides are runtime values.
///
/// Class invariant: `data_.size() >= required_span_extent(shape_, strides_)`
/// for any non-empty view, so every in-bounds index resolves to a valid
/// element of the backing span.
template <TensorScalar T, std::size_t Rank>
class TensorView {
public:
    using element_type = T;
    using value_type = std::remove_cv_t<T>;
    using index_type = std::size_t;
    using stride_type = std::ptrdiff_t;

    static constexpr std::size_t rank_v = Rank;

    /// Empty view (no backing storage, zero extents).
    constexpr TensorView() noexcept = default;

    /// Construct from a span and an explicit shape + stride descriptor.
    /// Precondition: `data.size() >= required_span_extent(shape, strides)`.
    constexpr TensorView(std::span<T> data, Extents<Rank> shape, Strides<Rank> strides) noexcept
        : data_(data), shape_(shape), strides_(strides) {}

    /// Construct a contiguous view from a span, shape, and layout. The
    /// strides are derived from the layout; the span must hold at least
    /// `extent_product(shape)` elements.
    constexpr TensorView(std::span<T> data, Extents<Rank> shape, Layout layout = Layout::RowMajor) noexcept
        : TensorView(data, shape, contiguous_strides(shape, layout)) {}

    /// @name Descriptor accessors
    /// @{
    [[nodiscard]] static constexpr std::size_t rank() noexcept {
        return Rank;
    }
    [[nodiscard]] constexpr const Extents<Rank>& shape() const noexcept {
        return shape_;
    }
    [[nodiscard]] constexpr const Strides<Rank>& strides() const noexcept {
        return strides_;
    }
    [[nodiscard]] constexpr std::size_t extent(std::size_t axis) const noexcept {
        return shape_[axis];
    }
    [[nodiscard]] constexpr std::ptrdiff_t stride(std::size_t axis) const noexcept {
        return strides_[axis];
    }
    /// Total number of addressable elements (product of extents).
    [[nodiscard]] constexpr std::size_t size() const noexcept {
        return extent_product(shape_);
    }
    [[nodiscard]] constexpr bool empty() const noexcept {
        return size() == 0;
    }
    /// @}

    /// @name Storage accessors
    /// @{
    [[nodiscard]] constexpr std::span<T> span() const noexcept {
        return data_;
    }
    [[nodiscard]] constexpr T* data() const noexcept {
        return data_.data();
    }
    /// @}

    /// True iff the view is densely packed in `layout` (strides equal
    /// the contiguous strides derived from the current shape).
    [[nodiscard]] constexpr bool is_contiguous(Layout layout = Layout::RowMajor) const noexcept {
        return strides_ == contiguous_strides(shape_, layout);
    }

    /// Linear offset (in elements, from the span base) of a multi-index.
    /// No bounds checking — callers index within `shape()`.
    template <class... Idx>
        requires(sizeof...(Idx) == Rank) && (std::convertible_to<Idx, std::size_t> && ...)
    [[nodiscard]] constexpr std::ptrdiff_t offset_of(Idx... idx) const noexcept {
        const std::array<std::size_t, Rank> ind{static_cast<std::size_t>(idx)...};
        std::ptrdiff_t off = 0;
        for (std::size_t k = 0; k < Rank; ++k) {
            off += static_cast<std::ptrdiff_t>(ind[k]) * strides_[k];
        }
        return off;
    }

    /// Element access by multi-index. Returns a reference into the
    /// backing buffer; mutation propagates to the caller's storage
    /// unless `T` is const-qualified.
    template <class... Idx>
        requires(sizeof...(Idx) == Rank) && (std::convertible_to<Idx, std::size_t> && ...)
    [[nodiscard]] constexpr T& operator()(Idx... idx) const noexcept {
        return data_[static_cast<std::size_t>(offset_of(idx...))];
    }

private:
    std::span<T> data_{};
    Extents<Rank> shape_{};
    Strides<Rank> strides_{};
};

/// @name Convenience aliases for the common low ranks.
/// @{
template <TensorScalar T>
using VectorView = TensorView<T, 1>;
template <TensorScalar T>
using MatrixView = TensorView<T, 2>;
/// @}

}  // namespace branes::math

#endif  // BRANES_MATH_TENSOR_HPP
