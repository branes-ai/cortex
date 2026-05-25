// SPDX-License-Identifier: MIT
//
// branes/cv/klt.hpp — sub-pixel pyramidal Lucas–Kanade feature tracker
// (inverse-compositional formulation). Clean-room from Baker & Matthews,
// "Lucas-Kanade 20 Years On" (2004), and the pyramidal LK of Bouguet.
//
// For each input point the template patch and its gradient/Hessian are
// computed once on the *previous* image (the inverse-compositional trick),
// and a translational warp is refined coarse-to-fine across the pyramid
// with bilinear sub-pixel sampling. Header-only, type-generic.

#ifndef BRANES_CV_KLT_HPP
#define BRANES_CV_KLT_HPP

#include <branes/cv/fast.hpp>  // KeyPoint
#include <branes/cv/image.hpp>
#include <branes/cv/pyramid.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace branes::cv {

enum class TrackStatus { Tracked, Lost, OutOfBounds };

/// Result of tracking one point: its position in the next image and why
/// tracking ended.
struct KltResult {
    float x = 0.0f;
    float y = 0.0f;
    TrackStatus status = TrackStatus::Lost;
};

/// Tracker tuning.
struct KltParams {
    int window_half = 5;           ///< half window; full size 2*hw+1
    int max_iters = 30;            ///< LK iterations per pyramid level
    double residual_eps = 0.01;    ///< stop when |update| < this (px)
    double min_eigenvalue = 1e-3;  ///< min per-pixel eigenvalue of H to track
};

namespace detail {

/// Bilinear sample of a level (clamped to the border).
template <PixelType T>
[[nodiscard]] double sample_bilinear(const Image<const T>& img, double fx, double fy) {
    const int w = static_cast<int>(img.width());
    const int h = static_cast<int>(img.height());
    if (fx < 0)
        fx = 0;
    if (fy < 0)
        fy = 0;
    if (fx > w - 1)
        fx = w - 1;
    if (fy > h - 1)
        fy = h - 1;
    const int x0 = static_cast<int>(fx);
    const int y0 = static_cast<int>(fy);
    const int x1 = (x0 + 1 < w) ? x0 + 1 : x0;
    const int y1 = (y0 + 1 < h) ? y0 + 1 : y0;
    const double ax = fx - x0;
    const double ay = fy - y0;
    const double v00 = static_cast<double>(img(y0, x0));
    const double v01 = static_cast<double>(img(y0, x1));
    const double v10 = static_cast<double>(img(y1, x0));
    const double v11 = static_cast<double>(img(y1, x1));
    return (v00 * (1 - ax) + v01 * ax) * (1 - ay) + (v10 * (1 - ax) + v11 * ax) * ay;
}

/// True if a window of half-size `margin` centered at (cx, cy) lies fully
/// inside a `w`×`h` level (so sampling never falls back on clamped border).
[[nodiscard]] inline bool window_in_bounds(int w, int h, double cx, double cy, double margin) {
    return cx - margin >= 0.0 && cy - margin >= 0.0 && cx + margin <= w - 1.0 && cy + margin <= h - 1.0;
}

}  // namespace detail

/// Track `points` from the `prev` pyramid into the `next` pyramid.
/// Returns one KltResult per input point, in order.
template <PixelType T>
[[nodiscard]] std::vector<KltResult> track_klt_pyramidal(const Pyramid<T>& prev,
                                                         const Pyramid<T>& next,
                                                         const std::vector<KeyPoint>& points,
                                                         const KltParams& params = {}) {
    const int levels = std::min(prev.num_levels(), next.num_levels());
    const double sf = prev.scale_factor();
    const int hw = params.window_half;
    const int win = (2 * hw + 1) * (2 * hw + 1);

    std::vector<KltResult> results;
    results.reserve(points.size());

    for (const auto& pt : points) {
        // Guard degenerate (empty) pyramids: nothing to track against.
        if (levels < 1) {
            results.push_back({pt.x, pt.y, TrackStatus::Lost});
            continue;
        }

        double flow_x = 0.0, flow_y = 0.0;  // displacement in finest pixels
        TrackStatus status = TrackStatus::Tracked;

        for (int l = levels - 1; l >= 0 && status == TrackStatus::Tracked; --l) {
            const double scale = std::pow(sf, l);
            const auto tmpl = prev.level(l);
            const auto img = next.level(l);
            const int tw = static_cast<int>(tmpl.width());
            const int th = static_cast<int>(tmpl.height());
            const double cx = pt.x / scale;  // template center at this level
            const double cy = pt.y / scale;
            double gx = flow_x / scale;  // displacement guess at this level
            double gy = flow_y / scale;

            // The template-gradient pass samples sx±1 / sy±1, so the
            // template needs an hw+1 margin; if it doesn't fit, the point
            // is too close to the edge to track here.
            if (!detail::window_in_bounds(tw, th, cx, cy, hw + 1.0)) {
                status = TrackStatus::OutOfBounds;
                break;
            }

            // Precompute template intensities, gradients, and Hessian
            // (inverse-compositional: constant across iterations).
            std::vector<double> tval(win), tgx(win), tgy(win);
            double hxx = 0, hxy = 0, hyy = 0;
            int idx = 0;
            for (int dy = -hw; dy <= hw; ++dy) {
                for (int dx = -hw; dx <= hw; ++dx, ++idx) {
                    const double sx = cx + dx, sy = cy + dy;
                    tval[idx] = detail::sample_bilinear(tmpl, sx, sy);
                    const double ix =
                        0.5 * (detail::sample_bilinear(tmpl, sx + 1, sy) - detail::sample_bilinear(tmpl, sx - 1, sy));
                    const double iy =
                        0.5 * (detail::sample_bilinear(tmpl, sx, sy + 1) - detail::sample_bilinear(tmpl, sx, sy - 1));
                    tgx[idx] = ix;
                    tgy[idx] = iy;
                    hxx += ix * ix;
                    hxy += ix * iy;
                    hyy += iy * iy;
                }
            }
            const double det = hxx * hyy - hxy * hxy;
            const double trace = hxx + hyy;
            const double min_eig = 0.5 * (trace - std::sqrt(std::max(0.0, trace * trace - 4.0 * det)));
            if (min_eig / win < params.min_eigenvalue || det <= 0.0) {
                status = TrackStatus::Lost;
                break;
            }

            for (int iter = 0; iter < params.max_iters; ++iter) {
                double bx = 0, by = 0;
                idx = 0;
                for (int dy = -hw; dy <= hw; ++dy) {
                    for (int dx = -hw; dx <= hw; ++dx, ++idx) {
                        const double e = detail::sample_bilinear(img, cx + gx + dx, cy + gy + dy) - tval[idx];
                        bx += tgx[idx] * e;
                        by += tgy[idx] * e;
                    }
                }
                // Solve H d = b, then inverse-compositional update g -= d.
                const double dxk = (hyy * bx - hxy * by) / det;
                const double dyk = (-hxy * bx + hxx * by) / det;
                gx -= dxk;
                gy -= dyk;
                if (std::sqrt(dxk * dxk + dyk * dyk) < params.residual_eps)
                    break;
            }

            // If the support window left this level, the solve ran against
            // clamped border pixels — report OutOfBounds rather than a
            // bogus result.
            if (!detail::window_in_bounds(
                    static_cast<int>(img.width()), static_cast<int>(img.height()), cx + gx, cy + gy, hw)) {
                status = TrackStatus::OutOfBounds;
                break;
            }

            flow_x = gx * scale;
            flow_y = gy * scale;
        }

        results.push_back({static_cast<float>(pt.x + flow_x), static_cast<float>(pt.y + flow_y), status});
    }
    return results;
}

}  // namespace branes::cv

#endif  // BRANES_CV_KLT_HPP
