// SPDX-License-Identifier: MIT
//
// branes/cv/pyramid.hpp — Gaussian image pyramid for the VIO front end.
//
// Builds a multi-resolution pyramid by repeated anti-aliased downsampling:
// each level is the previous level blurred with a separable Gaussian
// (sized to the scale factor) and resampled at the smaller grid. The
// scale factor is configurable (default 2.0 = a classic octave pyramid).
// Levels are owned internally and exposed as Image<const T> views.
//
// Clean-room: the construction is the standard separable-Gaussian +
// resample pipeline; no third-party CV source was consulted. Header-only,
// C++20, type-generic over the pixel type.

#ifndef BRANES_CV_PYRAMID_HPP
#define BRANES_CV_PYRAMID_HPP

#include <branes/cv/image.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace branes::cv {

namespace detail {

/// Convert a working-precision value back to the pixel type: round and
/// clamp for integral pixels, pass through for floating ones.
template <PixelType T>
[[nodiscard]] inline T to_pixel(double v) {
    if constexpr (std::integral<T>) {
        double r = std::round(v);
        const double lo = static_cast<double>(std::numeric_limits<T>::min());
        const double hi = static_cast<double>(std::numeric_limits<T>::max());
        if (r < lo)
            r = lo;
        if (r > hi)
            r = hi;
        return static_cast<T>(r);
    } else {
        return static_cast<T>(v);
    }
}

/// Normalized 1-D Gaussian kernel with radius ceil(3σ).
[[nodiscard]] inline std::vector<double> gaussian_kernel(double sigma) {
    if (sigma < 1e-6)
        return {1.0};
    const int radius = static_cast<int>(std::ceil(3.0 * sigma));
    std::vector<double> k(static_cast<std::size_t>(2 * radius + 1));
    const double inv2s2 = 1.0 / (2.0 * sigma * sigma);
    double sum = 0.0;
    for (int i = -radius; i <= radius; ++i) {
        const double w = std::exp(-static_cast<double>(i * i) * inv2s2);
        k[static_cast<std::size_t>(i + radius)] = w;
        sum += w;
    }
    for (double& w : k)
        w /= sum;
    return k;
}

[[nodiscard]] inline std::size_t clamp_idx(long i, std::size_t n) {
    if (i < 0)
        return 0;
    if (i >= static_cast<long>(n))
        return n - 1;
    return static_cast<std::size_t>(i);
}

/// Separable Gaussian blur of `src` into a working-precision buffer
/// (row-major, src.width()*src.height()), with replicated borders.
template <PixelType T>
inline std::vector<double> separable_blur(const Image<const T>& src, const std::vector<double>& kernel) {
    const std::size_t w = src.width();
    const std::size_t h = src.height();
    const long r = static_cast<long>(kernel.size() / 2);
    std::vector<double> horiz(w * h, 0.0);
    // Horizontal pass.
    for (std::size_t y = 0; y < h; ++y) {
        for (std::size_t x = 0; x < w; ++x) {
            double acc = 0.0;
            for (long t = -r; t <= r; ++t) {
                const std::size_t sx = clamp_idx(static_cast<long>(x) + t, w);
                acc += kernel[static_cast<std::size_t>(t + r)] * static_cast<double>(src(y, sx));
            }
            horiz[y * w + x] = acc;
        }
    }
    // Vertical pass.
    std::vector<double> out(w * h, 0.0);
    for (std::size_t y = 0; y < h; ++y) {
        for (std::size_t x = 0; x < w; ++x) {
            double acc = 0.0;
            for (long t = -r; t <= r; ++t) {
                const std::size_t sy = clamp_idx(static_cast<long>(y) + t, h);
                acc += kernel[static_cast<std::size_t>(t + r)] * horiz[sy * w + x];
            }
            out[y * w + x] = acc;
        }
    }
    return out;
}

/// Bilinear sample of a working-precision buffer at fractional (fx, fy).
[[nodiscard]] inline double
bilinear(const std::vector<double>& buf, std::size_t w, std::size_t h, double fx, double fy) {
    if (fx < 0.0)
        fx = 0.0;
    if (fy < 0.0)
        fy = 0.0;
    const double maxx = static_cast<double>(w - 1);
    const double maxy = static_cast<double>(h - 1);
    if (fx > maxx)
        fx = maxx;
    if (fy > maxy)
        fy = maxy;
    const std::size_t x0 = static_cast<std::size_t>(fx);
    const std::size_t y0 = static_cast<std::size_t>(fy);
    const std::size_t x1 = (x0 + 1 < w) ? x0 + 1 : x0;
    const std::size_t y1 = (y0 + 1 < h) ? y0 + 1 : y0;
    const double ax = fx - static_cast<double>(x0);
    const double ay = fy - static_cast<double>(y0);
    const double top = buf[y0 * w + x0] * (1 - ax) + buf[y0 * w + x1] * ax;
    const double bot = buf[y1 * w + x0] * (1 - ax) + buf[y1 * w + x1] * ax;
    return top * (1 - ay) + bot * ay;
}

}  // namespace detail

/// Multi-resolution Gaussian pyramid over an owned pixel store.
template <PixelType T>
class Pyramid {
public:
    Pyramid() = default;

    /// Build `num_levels` levels from `base`, reducing each dimension by
    /// `scale_factor` per level (>= 1.0; default 2.0). Level 0 is a copy
    /// of `base`; level i+1 is level i blurred (separable Gaussian,
    /// σ = scale_factor/2) and resampled to round(size / scale_factor).
    /// Construction stops early if a level would collapse below 1 pixel.
    Pyramid(Image<const T> base, int num_levels, double scale_factor = 2.0) : scale_(scale_factor) {
        if (num_levels < 1)
            num_levels = 1;
        levels_.emplace_back(base.width(), base.height());
        for (std::size_t y = 0; y < base.height(); ++y)
            for (std::size_t x = 0; x < base.width(); ++x)
                levels_[0](y, x) = base(y, x);

        const auto kernel = detail::gaussian_kernel(scale_factor * 0.5);
        for (int lvl = 1; lvl < num_levels; ++lvl) {
            const auto& prev = levels_.back();
            const std::size_t pw = prev.width();
            const std::size_t ph = prev.height();
            const auto ow = static_cast<std::size_t>(std::llround(static_cast<double>(pw) / scale_factor));
            const auto oh = static_cast<std::size_t>(std::llround(static_cast<double>(ph) / scale_factor));
            if (ow < 1 || oh < 1)
                break;

            const auto blurred = detail::separable_blur<T>(prev.view(), kernel);
            OwnedImage<T> next(ow, oh);
            const double rx = static_cast<double>(pw) / static_cast<double>(ow);
            const double ry = static_cast<double>(ph) / static_cast<double>(oh);
            for (std::size_t oy = 0; oy < oh; ++oy) {
                const double sy = (static_cast<double>(oy) + 0.5) * ry - 0.5;
                for (std::size_t ox = 0; ox < ow; ++ox) {
                    const double sx = (static_cast<double>(ox) + 0.5) * rx - 0.5;
                    next(oy, ox) = detail::to_pixel<T>(detail::bilinear(blurred, pw, ph, sx, sy));
                }
            }
            levels_.push_back(std::move(next));
        }
    }

    [[nodiscard]] int num_levels() const noexcept {
        return static_cast<int>(levels_.size());
    }
    [[nodiscard]] double scale_factor() const noexcept {
        return scale_;
    }

    [[nodiscard]] Image<const T> level(int i) const noexcept {
        return levels_[static_cast<std::size_t>(i)].view();
    }
    [[nodiscard]] std::size_t level_width(int i) const noexcept {
        return levels_[static_cast<std::size_t>(i)].width();
    }
    [[nodiscard]] std::size_t level_height(int i) const noexcept {
        return levels_[static_cast<std::size_t>(i)].height();
    }

private:
    std::vector<OwnedImage<T>> levels_;
    double scale_ = 2.0;
};

}  // namespace branes::cv

#endif  // BRANES_CV_PYRAMID_HPP
