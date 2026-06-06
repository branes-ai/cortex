// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/frontend_probe.hpp — the S4 (visual frontend) probe of the VIO
// contract program (docs/arch/vio-pipeline-canonical.md).
//
// Drives the SHIPPED frontend primitives — the FAST detector (cv/fast.hpp) and the
// pyramidal KLT tracker (cv/klt.hpp) — on synthetic richly-textured images warped
// by a KNOWN translation, and measures track quality against the S4 contract:
//
//   • forward-backward residual — track A→B then B→A; a self-consistent track
//     returns to its origin. The frontend ships NO forward-backward check and NO
//     RANSAC (only KLT's own Lost/OutOfBounds status filter), so this probe both
//     measures the FB residual AND quantifies the outliers that would be gated.
//   • endpoint error vs the known warp — the tracker's true accuracy.
//   • grid coverage — are detections spread across the frame, or clustered?
//   • track-length — over a multi-frame translating sequence, how long tracks live.
//   • pixel-noise sweep — add σ px of image noise and measure the recovered-track
//     endpoint error: the TRUE measurement noise the backend should assume (vs its
//     optimistic assumed σ — the #212 link).
//
// Native units: pixels (residual / endpoint error), % (coverage / survival),
// frames (track length). Header-only, C++20, type-generic on the metric scalar.

#ifndef BRANES_SDK_EVAL_FRONTEND_PROBE_HPP
#define BRANES_SDK_EVAL_FRONTEND_PROBE_HPP

#include <branes/cv/fast.hpp>
#include <branes/cv/image.hpp>
#include <branes/cv/klt.hpp>
#include <branes/cv/pyramid.hpp>
#include <branes/math/arithmetic.hpp>  // math::Scalar

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace branes::sdk::eval {

namespace frontend_detail {
using Img = branes::cv::OwnedImage<std::uint8_t>;

// A richly-textured, well-conditioned pattern (sum of sinusoids) — the kind KLT
// can actually track. Rendered with a backward map so the whole image translates
// by (dx, dy); optional per-pixel Gaussian noise of σ = noise_sigma (grey levels).
template <class RNG>
inline Img render(int w, int h, double dx, double dy, double noise_sigma, RNG& rng) {
    auto pattern = [](double x, double y) {
        // Textured background (KLT can lock onto the gradients everywhere) plus a
        // grid of sharp bright squares (the corners FAST detects). Both live in the
        // pattern's own coordinates, so the whole scene translates with the warp.
        double v = 100.0 + 40.0 * std::sin(0.21 * x + 0.10 * y) + 35.0 * std::cos(0.13 * x - 0.17 * y) +
                   25.0 * std::sin(0.07 * x) * std::cos(0.09 * y);
        const double mx = x - 32.0 * std::floor(x / 32.0);
        const double my = y - 32.0 * std::floor(y / 32.0);
        if (mx < 6.0 && my < 6.0)
            v = 245.0;  // bright square marker → a FAST corner
        return v;
    };
    std::normal_distribution<double> N01(0.0, 1.0);
    Img img(static_cast<std::size_t>(w), static_cast<std::size_t>(h));
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            double v = pattern(x - dx, y - dy);
            if (noise_sigma > 0)
                v += noise_sigma * N01(rng);
            img(static_cast<std::size_t>(y), static_cast<std::size_t>(x)) =
                static_cast<std::uint8_t>(std::clamp(v, 0.0, 255.0));
        }
    return img;
}

template <class T>
T median(std::vector<T> v) {
    if (v.empty())
        return T{0};
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}
}  // namespace frontend_detail

template <math::Scalar T>
struct FrontendResult {
    // ── clean forward-backward tracking ──
    std::size_t detected = 0, tracked = 0, fb_inliers = 0;
    T fb_residual_median = 0, fb_residual_max = 0;
    T endpoint_rms_all_px = 0;       ///< over EVERY backward-tracked correspondence (what the un-gated frontend feeds)
    T endpoint_rms_gated_px = 0;     ///< over the fb<1px subset (what a forward-backward gate WOULD keep)
    bool ransac_applied = false;     ///< the shipped frontend ships no RANSAC — the contract metric is "not applied"
    std::vector<T> fb_residuals;     ///< per surviving track
    std::vector<T> endpoint_errors;  ///< per surviving track (vs known warp)
    // ── spatial coverage ──
    int grid_cols = 0, grid_rows = 0;
    T grid_occupancy_pct = 0;
    // ── track length over a multi-frame sequence ──
    std::vector<std::size_t> track_len_hist;  ///< index = length (frames), value = count
    T mean_track_len = 0;
    // ── pixel-noise → track-noise calibration ──
    struct NoisePoint {
        T sigma_px = 0, endpoint_rms = 0, fb_residual_median = 0, survival_pct = 0;
    };
    std::vector<NoisePoint> noise_curve;
};

/// Run the full S4 frontend probe on synthetic textured imagery.
template <math::Scalar T>
[[nodiscard]] FrontendResult<T> frontend_probe() {
    using namespace branes::cv;
    using namespace frontend_detail;
    namespace bc = branes::cv;

    const int W = 320, H = 240, levels = 3;
    const double thresh = 20.0;
    const int nms = 5;
    const double warp_dx = 3.0, warp_dy = -2.0;  // known inter-frame translation (px)
    const KltParams klt{};
    std::mt19937_64 rng(0xF00DCAFEull);

    auto dist = [](float ax, float ay, float bx, float by) {
        return std::sqrt(static_cast<T>((ax - bx) * (ax - bx) + (ay - by) * (ay - by)));
    };

    FrontendResult<T> r;

    // ── clean A, detect, forward A→B, backward B→A ──────────────────────────
    const Img A = render(W, H, 0.0, 0.0, 0.0, rng);
    const Img B = render(W, H, warp_dx, warp_dy, 0.0, rng);
    const Pyramid<std::uint8_t> pa(A.view(), levels);
    const Pyramid<std::uint8_t> pb(B.view(), levels);
    const auto kps = detect_fast<std::uint8_t>(A.view(), thresh, nms);
    r.detected = kps.size();

    const auto fwd = track_klt_pyramidal<std::uint8_t>(pa, pb, kps, klt);
    std::vector<KeyPoint> fwd_kps(fwd.size());
    for (std::size_t i = 0; i < fwd.size(); ++i)
        fwd_kps[i] = KeyPoint{fwd[i].x, fwd[i].y, 0.0f};
    const auto bwd = track_klt_pyramidal<std::uint8_t>(pb, pa, fwd_kps, klt);

    T sq_all = 0, sq_gated = 0;
    std::size_t n_all = 0;
    for (std::size_t i = 0; i < kps.size(); ++i) {
        if (fwd[i].status != TrackStatus::Tracked)
            continue;
        ++r.tracked;
        if (bwd[i].status == TrackStatus::Tracked) {
            const T fb = dist(bwd[i].x, bwd[i].y, kps[i].x, kps[i].y);
            r.fb_residuals.push_back(fb);
            const T ep = dist(
                fwd[i].x, fwd[i].y, kps[i].x + static_cast<float>(warp_dx), kps[i].y + static_cast<float>(warp_dy));
            r.endpoint_errors.push_back(ep);
            r.fb_residual_max = std::max(r.fb_residual_max, fb);
            sq_all += ep * ep;  // over EVERY backward-tracked correspondence (the un-gated reality)
            ++n_all;
            if (fb < T{1}) {  // a 1 px FB gate — the check the frontend does NOT apply
                ++r.fb_inliers;
                sq_gated += ep * ep;
            }
        }
    }
    r.fb_residual_median = median(r.fb_residuals);
    r.endpoint_rms_all_px = n_all ? std::sqrt(sq_all / static_cast<T>(n_all)) : T{0};
    r.endpoint_rms_gated_px = r.fb_inliers ? std::sqrt(sq_gated / static_cast<T>(r.fb_inliers)) : T{0};

    // ── grid coverage of the detections ─────────────────────────────────────
    r.grid_cols = 8;
    r.grid_rows = 6;
    std::vector<int> cell(static_cast<std::size_t>(r.grid_cols * r.grid_rows), 0);
    for (const auto& k : kps) {
        const int cx = std::min(r.grid_cols - 1, static_cast<int>(k.x) * r.grid_cols / W);
        const int cy = std::min(r.grid_rows - 1, static_cast<int>(k.y) * r.grid_rows / H);
        ++cell[static_cast<std::size_t>(cy * r.grid_cols + cx)];
    }
    int occupied = 0;
    for (int c : cell)
        occupied += (c > 0);
    r.grid_occupancy_pct = T{100} * static_cast<T>(occupied) / static_cast<T>(cell.size());

    // ── track length over a multi-frame translating sequence ────────────────
    {
        const int nframes = 12;
        const double sx = 2.0, sy = -1.5;  // per-frame translation
        std::vector<Img> seq;
        seq.reserve(static_cast<std::size_t>(nframes));
        for (int f = 0; f < nframes; ++f)
            seq.push_back(render(W, H, sx * f, sy * f, 0.0, rng));
        auto pts = detect_fast<std::uint8_t>(seq[0].view(), thresh, nms);
        std::vector<std::size_t> life(pts.size(), 1);  // frames survived (incl. frame 0)
        std::vector<bool> alive(pts.size(), true);
        for (int f = 0; f + 1 < nframes; ++f) {
            const Pyramid<std::uint8_t> p0(seq[f].view(), levels), p1(seq[f + 1].view(), levels);
            const auto tr = track_klt_pyramidal<std::uint8_t>(p0, p1, pts, klt);
            for (std::size_t i = 0; i < pts.size(); ++i) {
                if (!alive[i])
                    continue;
                if (tr[i].status == TrackStatus::Tracked) {
                    pts[i] = KeyPoint{tr[i].x, tr[i].y, 0.0f};
                    ++life[i];
                } else {
                    alive[i] = false;
                }
            }
        }
        r.track_len_hist.assign(static_cast<std::size_t>(nframes) + 1, 0);
        T sum = 0;
        for (std::size_t l : life) {
            ++r.track_len_hist[l];
            sum += static_cast<T>(l);
        }
        r.mean_track_len = life.empty() ? T{0} : sum / static_cast<T>(life.size());
    }

    // ── pixel-noise → track-noise calibration sweep ─────────────────────────
    for (const double sigma : {0.0, 0.5, 1.0, 2.0, 4.0}) {
        const Img Bn = render(W, H, warp_dx, warp_dy, sigma, rng);
        const Pyramid<std::uint8_t> pbn(Bn.view(), levels);
        const auto f2 = track_klt_pyramidal<std::uint8_t>(pa, pbn, kps, klt);
        std::vector<KeyPoint> f2k(f2.size());
        for (std::size_t i = 0; i < f2.size(); ++i)
            f2k[i] = KeyPoint{f2[i].x, f2[i].y, 0.0f};
        const auto b2 = track_klt_pyramidal<std::uint8_t>(pbn, pa, f2k, klt);
        T sq = 0;
        std::size_t in = 0;
        std::vector<T> fbs;
        for (std::size_t i = 0; i < kps.size(); ++i) {
            if (f2[i].status != TrackStatus::Tracked || b2[i].status != TrackStatus::Tracked)
                continue;
            const T fb = dist(b2[i].x, b2[i].y, kps[i].x, kps[i].y);
            fbs.push_back(fb);
            if (fb < T{1}) {
                const T ep = dist(
                    f2[i].x, f2[i].y, kps[i].x + static_cast<float>(warp_dx), kps[i].y + static_cast<float>(warp_dy));
                sq += ep * ep;
                ++in;
            }
        }
        typename FrontendResult<T>::NoisePoint np;
        np.sigma_px = static_cast<T>(sigma);
        np.endpoint_rms = in ? std::sqrt(sq / static_cast<T>(in)) : T{0};
        np.fb_residual_median = median(fbs);
        np.survival_pct = r.detected ? T{100} * static_cast<T>(in) / static_cast<T>(r.detected) : T{0};
        r.noise_curve.push_back(np);
    }

    return r;
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_FRONTEND_PROBE_HPP
