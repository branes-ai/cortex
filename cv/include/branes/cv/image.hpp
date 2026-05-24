// SPDX-License-Identifier: MIT
//
// branes/cv/image.hpp — image containers for the CV front end.
//
// Two complementary types, mirroring the view/owner split used across
// the math layer:
//   - Image<T>      : a non-owning view over a caller-owned pixel buffer
//                     (row-major, with an explicit row stride so it can
//                     window into a larger image or a padded allocation).
//   - OwnedImage<T> : an owning companion backed by std::vector<T>.
//
// Pixel types are the grayscale depths the VIO pipeline needs —
// uint8_t (8-bit), uint16_t (16-bit), and float (normalized / pyramid
// working type). The containers are single-channel (grayscale); the
// EuRoC cameras are mono. Header-only, C++20.

#ifndef BRANES_CV_IMAGE_HPP
#define BRANES_CV_IMAGE_HPP

#include <cassert>
#include <concepts>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <vector>

namespace branes::cv {

/// Pixel scalar: a trivially-copyable integral or floating type. The
/// pipeline exercises uint8_t / uint16_t / float; the concept admits the
/// family rather than an explicit list so derived working types fit.
template <class T>
concept PixelType = std::is_trivially_copyable_v<T> && (std::integral<T> || std::floating_point<T>);

/// Non-owning, row-major, single-channel image view. `stride` is the
/// distance between consecutive rows *in elements* (>= width), allowing
/// a view into a sub-rectangle or a padded buffer. Indexing is
/// (row, col) = (y, x).
template <PixelType T>
class Image {
public:
    using value_type = T;

    constexpr Image() noexcept = default;

    constexpr Image(T* data, std::size_t width, std::size_t height, std::size_t stride) noexcept
        : data_(data), width_(width), height_(height), stride_(stride) {
        // Rows are addressed as row*stride + col, so stride < width would
        // overlap rows and produce out-of-bounds offsets. Debug-only so
        // the view stays zero-overhead in release.
        assert(stride >= width && "Image stride must be >= width");
    }

    /// Contiguous view (stride == width).
    constexpr Image(T* data, std::size_t width, std::size_t height) noexcept : Image(data, width, height, width) {}

    /// Implicit mutable→const view conversion, like std::span<U> →
    /// std::span<const U>. Enabled only when T is `const U`.
    template <class U>
        requires(std::same_as<T, const U>)
    constexpr Image(const Image<U>& other) noexcept
        : data_(other.data()), width_(other.width()), height_(other.height()), stride_(other.stride()) {}

    [[nodiscard]] constexpr std::size_t width() const noexcept {
        return width_;
    }
    [[nodiscard]] constexpr std::size_t height() const noexcept {
        return height_;
    }
    [[nodiscard]] constexpr std::size_t stride() const noexcept {
        return stride_;
    }
    [[nodiscard]] constexpr T* data() const noexcept {
        return data_;
    }
    [[nodiscard]] constexpr bool empty() const noexcept {
        return width_ == 0 || height_ == 0;
    }
    /// True when rows are back-to-back (no padding).
    [[nodiscard]] constexpr bool is_contiguous() const noexcept {
        return stride_ == width_;
    }

    /// Pixel access by (row, col). No bounds checking.
    [[nodiscard]] constexpr T& operator()(std::size_t row, std::size_t col) const noexcept {
        return data_[row * stride_ + col];
    }

    /// A span over row `r` (width elements).
    [[nodiscard]] constexpr std::span<T> row(std::size_t r) const noexcept {
        return std::span<T>{data_ + r * stride_, width_};
    }

private:
    T* data_ = nullptr;
    std::size_t width_ = 0;
    std::size_t height_ = 0;
    std::size_t stride_ = 0;
};

/// Owning, contiguous, single-channel image. Hands out Image<T> /
/// Image<const T> views over its storage.
template <PixelType T>
class OwnedImage {
public:
    using value_type = T;

    OwnedImage() = default;

    OwnedImage(std::size_t width, std::size_t height, T fill = T{})
        : buf_(checked_area(width, height), fill), width_(width), height_(height) {}

    [[nodiscard]] std::size_t width() const noexcept {
        return width_;
    }
    [[nodiscard]] std::size_t height() const noexcept {
        return height_;
    }
    [[nodiscard]] std::size_t size() const noexcept {
        return buf_.size();
    }
    [[nodiscard]] T* data() noexcept {
        return buf_.data();
    }
    [[nodiscard]] const T* data() const noexcept {
        return buf_.data();
    }

    /// Pixel access by (row, col). No bounds checking (mirrors Image).
    [[nodiscard]] T& operator()(std::size_t row, std::size_t col) noexcept {
        return buf_[row * width_ + col];
    }
    [[nodiscard]] const T& operator()(std::size_t row, std::size_t col) const noexcept {
        return buf_[row * width_ + col];
    }

    [[nodiscard]] Image<T> view() noexcept {
        return Image<T>(buf_.data(), width_, height_);
    }
    [[nodiscard]] Image<const T> view() const noexcept {
        return Image<const T>(buf_.data(), width_, height_);
    }

private:
    // Guard against size_t overflow in width*height before it silently
    // undersizes the allocation.
    static std::size_t checked_area(std::size_t width, std::size_t height) {
        if (width != 0 && height > std::numeric_limits<std::size_t>::max() / width) {
            throw std::length_error("OwnedImage: width*height overflows size_t");
        }
        return width * height;
    }

    std::vector<T> buf_;
    std::size_t width_ = 0;
    std::size_t height_ = 0;
};

}  // namespace branes::cv

#endif  // BRANES_CV_IMAGE_HPP
