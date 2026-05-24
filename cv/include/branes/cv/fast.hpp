// SPDX-License-Identifier: MIT
//
// branes/cv/fast.hpp — FAST-9 corner detector with non-maximum
// suppression. Clean-room from the FAST description (Rosten & Drummond,
// "Machine learning for high-speed corner detection", 2006): a pixel is
// a corner if, on the Bresenham radius-3 circle of 16 pixels, there are
// 9 contiguous pixels all brighter than center+t or all darker than
// center−t. Header-only, type-generic over the pixel type.

#ifndef BRANES_CV_FAST_HPP
#define BRANES_CV_FAST_HPP

#include <branes/cv/image.hpp>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

namespace branes::cv {

/// A detected corner: subpixel-capable position (here integer-valued) and
/// the FAST score (largest threshold for which it remains a corner).
struct KeyPoint {
    float x = 0.0f;
    float y = 0.0f;
    float response = 0.0f;
};

namespace detail {

// Bresenham circle of radius 3, 16 pixels, clockwise from the top.
inline constexpr int kFastCircleX[16] = {0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1};
inline constexpr int kFastCircleY[16] = {-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3};

}  // namespace detail

/// Detect FAST-9 corners in `img` with a brightness `threshold`, then
/// keep only local maxima of the corner score within `nms_radius`.
/// The score of a pixel is the largest t for which 9 contiguous circle
/// pixels stay all-brighter or all-darker — a corner is reported when
/// that score exceeds `threshold`.
template <PixelType T>
[[nodiscard]] std::vector<KeyPoint> detect_fast(const Image<const T>& img, double threshold, int nms_radius = 3) {
    const int w = static_cast<int>(img.width());
    const int h = static_cast<int>(img.height());
    constexpr int border = 3;  // circle radius
    std::vector<double> score(static_cast<std::size_t>(w) * h, 0.0);

    for (int y = border; y < h - border; ++y) {
        for (int x = border; x < w - border; ++x) {
            const double ip = static_cast<double>(img(y, x));
            double c[16];
            for (int k = 0; k < 16; ++k) {
                c[k] = static_cast<double>(img(y + detail::kFastCircleY[k], x + detail::kFastCircleX[k]));
            }
            // bright/dark score = max over the 16 contiguous 9-windows of
            // the min margin within the window.
            double bright = -std::numeric_limits<double>::infinity();
            double dark = -std::numeric_limits<double>::infinity();
            for (int s = 0; s < 16; ++s) {
                double min_b = std::numeric_limits<double>::infinity();
                double min_d = std::numeric_limits<double>::infinity();
                for (int win = 0; win < 9; ++win) {
                    const double v = c[(s + win) & 15];
                    min_b = std::min(min_b, v - ip);
                    min_d = std::min(min_d, ip - v);
                }
                bright = std::max(bright, min_b);
                dark = std::max(dark, min_d);
            }
            const double sc = std::max(bright, dark);
            if (sc > threshold)
                score[static_cast<std::size_t>(y) * w + x] = sc;
        }
    }

    // Non-maximum suppression: keep a corner if its score is the strict
    // local maximum in the (2r+1)² window; ties broken by raster order so
    // exactly one survivor remains on a flat plateau.
    std::vector<KeyPoint> corners;
    const int r = std::max(0, nms_radius);  // negative radius ⇒ no suppression
    for (int y = border; y < h - border; ++y) {
        for (int x = border; x < w - border; ++x) {
            const double s = score[static_cast<std::size_t>(y) * w + x];
            if (s <= 0.0)
                continue;
            bool keep = true;
            for (int ny = std::max(0, y - r); ny <= std::min(h - 1, y + r) && keep; ++ny) {
                for (int nx = std::max(0, x - r); nx <= std::min(w - 1, x + r); ++nx) {
                    if (nx == x && ny == y)
                        continue;
                    const double sn = score[static_cast<std::size_t>(ny) * w + nx];
                    if (sn > s ||
                        (sn == s && (static_cast<std::size_t>(ny) * w + nx < static_cast<std::size_t>(y) * w + x))) {
                        keep = false;
                        break;
                    }
                }
            }
            if (keep) {
                corners.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(s)});
            }
        }
    }
    return corners;
}

}  // namespace branes::cv

#endif  // BRANES_CV_FAST_HPP
