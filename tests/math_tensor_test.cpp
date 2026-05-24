// Tensor-view round-trip + stride-math tests (issue #24).
//
// Verifies that TensorView resolves multi-indices to the correct
// element of a caller-owned buffer for row-major and column-major
// layouts, for custom (non-contiguous) strides, and across a couple of
// element types including a Universal posit.

#include <branes/math/tensor.hpp>

#include <catch2/catch_test_macros.hpp>
#include <universal/number/posit/posit.hpp>

#include <array>
#include <cstddef>
#include <numeric>
#include <span>
#include <vector>

using branes::math::contiguous_strides;
using branes::math::Layout;
using branes::math::MatrixView;
using branes::math::required_span_extent;
using branes::math::TensorView;

TEST_CASE("contiguous_strides match DLPack layout rules", "[math][tensor]") {
    const branes::math::Extents<3> shape{2, 3, 4};

    SECTION("row-major: last axis is unit-stride") {
        const auto s = contiguous_strides(shape, Layout::RowMajor);
        REQUIRE(s == branes::math::Strides<3>{12, 4, 1});
    }
    SECTION("column-major: first axis is unit-stride") {
        const auto s = contiguous_strides(shape, Layout::ColMajor);
        REQUIRE(s == branes::math::Strides<3>{1, 2, 6});
    }
}

TEST_CASE("row-major matrix view indexes correctly", "[math][tensor]") {
    // 3x4 matrix laid out row-major: buf[r*4 + c] == r*10 + c.
    std::array<int, 12> buf{};
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            buf[r * 4 + c] = static_cast<int>(r * 10 + c);
        }
    }
    MatrixView<int> m(std::span<int>{buf}, {3, 4}, Layout::RowMajor);

    REQUIRE(m.rank() == 2);
    REQUIRE(m.size() == 12);
    REQUIRE(m.extent(0) == 3);
    REQUIRE(m.extent(1) == 4);
    REQUIRE(m.is_contiguous(Layout::RowMajor));
    REQUIRE_FALSE(m.is_contiguous(Layout::ColMajor));

    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            REQUIRE(m(r, c) == static_cast<int>(r * 10 + c));
        }
    }
}

TEST_CASE("column-major matrix view indexes correctly", "[math][tensor]") {
    // Same logical 3x4 matrix, but stored column-major: buf[c*3 + r].
    std::array<int, 12> buf{};
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            buf[c * 3 + r] = static_cast<int>(r * 10 + c);
        }
    }
    MatrixView<int> m(std::span<int>{buf}, {3, 4}, Layout::ColMajor);

    REQUIRE(m.is_contiguous(Layout::ColMajor));
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 4; ++c) {
            REQUIRE(m(r, c) == static_cast<int>(r * 10 + c));
        }
    }
}

TEST_CASE("mutation through a view writes back to caller storage", "[math][tensor]") {
    std::array<double, 6> buf{};
    MatrixView<double> m(std::span<double>{buf}, {2, 3});
    m(1, 2) = 3.5;
    m(0, 0) = -1.0;
    REQUIRE(buf[1 * 3 + 2] == 3.5);  // row-major offset
    REQUIRE(buf[0] == -1.0);
}

TEST_CASE("const view yields read-only elements", "[math][tensor]") {
    std::array<int, 4> buf{7, 8, 9, 10};
    TensorView<const int, 1> v(std::span<const int>{buf}, {4});
    REQUIRE(v(0) == 7);
    REQUIRE(v(3) == 10);
    REQUIRE(std::is_same_v<decltype(v(0)), const int&>);
}

TEST_CASE("non-contiguous strided view (matrix row) indexes correctly", "[math][tensor]") {
    // View the middle row of a row-major 3x4 matrix as a length-4 vector
    // with unit stride, offset by pointing the span at the row start.
    std::array<int, 12> buf{};
    std::iota(buf.begin(), buf.end(), 0);  // 0..11
    // A column view: stride 4, length 3, starting at column index 2.
    std::span<int> col_span{buf.data() + 2, 9};  // covers rows 0..2 of col 2
    TensorView<int, 1> col(col_span, {3}, branes::math::Strides<1>{4});
    REQUIRE(col(0) == 2);
    REQUIRE(col(1) == 6);
    REQUIRE(col(2) == 10);
    REQUIRE_FALSE(col.is_contiguous());
}

TEST_CASE("required_span_extent computes addressable footprint", "[math][tensor]") {
    REQUIRE(required_span_extent<2>({3, 4}, contiguous_strides<2>({3, 4}, Layout::RowMajor)) == 12);
    // Empty along an axis addresses nothing.
    REQUIRE(required_span_extent<2>({0, 4}, contiguous_strides<2>({0, 4}, Layout::RowMajor)) == 0);
    // Strided column footprint: 3 elements, stride 4 ⇒ 1 + 2*4 = 9.
    REQUIRE(required_span_extent<1>({3}, branes::math::Strides<1>{4}) == 9);
    // Negative (reversed-axis) strides must report the same footprint as
    // their positive mirror, not wrap to a huge value (regression).
    REQUIRE(required_span_extent<1>({4}, branes::math::Strides<1>{-1}) == 4);
    REQUIRE(required_span_extent<1>({3}, branes::math::Strides<1>{-4}) == 9);
    // Mixed signs: rank-2 with one reversed axis spans both directions.
    REQUIRE(required_span_extent<2>({3, 4}, branes::math::Strides<2>{4, -1}) == 12);
}

TEST_CASE("view works over a Universal posit buffer", "[math][tensor]") {
    using posit32 = sw::universal::posit<32, 2>;
    std::vector<posit32> buf(6);
    MatrixView<posit32> m(std::span<posit32>{buf}, {2, 3});
    m(0, 0) = posit32{1.5};
    m(1, 2) = posit32{-2.25};
    REQUIRE(double(buf[0]) == 1.5);
    REQUIRE(double(buf[5]) == -2.25);
    REQUIRE(double(m(1, 2)) == -2.25);
}

TEST_CASE("compile-time stride math is constexpr", "[math][tensor]") {
    static constexpr auto s = contiguous_strides<2>({2, 5}, Layout::RowMajor);
    static_assert(s[0] == 5 && s[1] == 1);
    static_assert(branes::math::extent_product<2>({2, 5}) == 10);
    REQUIRE(true);
}
