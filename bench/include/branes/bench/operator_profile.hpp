// SPDX-License-Identifier: MIT
//
// branes/bench/operator_profile.hpp — first-order FLOP/byte model of the
// VIO pipeline stages, parameterized by the runtime counts the benchmark
// observes (image size, tracked features, KLT window/iters, state
// dimension, #updates). This is the per-operator profile the Phase-2
// `graphs` energy backend consumes (one SubgraphDescriptor per stage), and
// it feeds the roofline view in the report.
//
// The coefficients are deliberately first-order analytic estimates (ops
// per pixel / per feature / per matrix element), not measured FLOP counts;
// they are good enough to drive energy *modeling* and to be refined later
// with hardware performance counters. Each is a named constant so the
// assumptions are explicit.
//
// Header-only, C++20.

#ifndef BRANES_BENCH_OPERATOR_PROFILE_HPP
#define BRANES_BENCH_OPERATOR_PROFILE_HPP

#include <cstddef>
#include <string>
#include <vector>

namespace branes::bench {

/// One pipeline stage's analytic cost (per the run, not per frame).
struct StageProfile {
    std::string name;
    double flops = 0.0;
    double bytes = 0.0;  ///< bytes moved (in + out + intermediate)
};

/// Runtime counts observed by the benchmark over a sequence.
struct PipelineCounts {
    std::size_t frames = 0;
    std::size_t width = 0;
    std::size_t height = 0;
    int pyramid_levels = 3;
    double avg_tracked_features = 0.0;  ///< mean features carried per frame
    int klt_window_half = 5;            ///< full window = (2·hw+1)²
    int klt_iters = 30;
    double avg_state_dim = 15.0;     ///< mean 15 + 6·#clones
    std::size_t imu_samples = 0;     ///< propagation steps
    std::size_t camera_updates = 0;  ///< EKF measurement updates applied
    double avg_update_rows = 0.0;    ///< mean residual rows per update
};

// ── First-order coefficients (ops/elements), documented assumptions ──────
namespace coeff {
inline constexpr double kPyramidFlopsPerPx = 12.0;  ///< separable Gaussian blur+resample taps
inline constexpr double kFastFlopsPerPx = 16.0;     ///< FAST-9 ring comparisons + score
inline constexpr double kKltFlopsPerWinPx = 8.0;    ///< grad + Hessian/residual accumulate per window px
inline constexpr double kBytesPerPx = 1.0;          ///< 8-bit grayscale
inline constexpr double kMatElemBytes = 8.0;        ///< double covariance element
}  // namespace coeff

/// Image-pyramid build: each level i is ~ (W·H)/scale^(2i) pixels; with
/// scale 2 the geometric series ≈ 4/3·W·H total pixels touched.
[[nodiscard]] inline StageProfile pyramid_profile(const PipelineCounts& c) {
    const double px = static_cast<double>(c.width) * static_cast<double>(c.height);
    double level_px = 0.0, s = 1.0;
    for (int i = 0; i < c.pyramid_levels; ++i) {
        level_px += px / s;
        s *= 4.0;  // scale^2 per level
    }
    const double total = level_px * static_cast<double>(c.frames);
    return {"pyramid", total * coeff::kPyramidFlopsPerPx, total * coeff::kBytesPerPx * 2.0};
}

/// FAST detection on the full-resolution level, once per frame.
[[nodiscard]] inline StageProfile fast_profile(const PipelineCounts& c) {
    const double px = static_cast<double>(c.width) * static_cast<double>(c.height) * static_cast<double>(c.frames);
    return {"fast", px * coeff::kFastFlopsPerPx, px * coeff::kBytesPerPx};
}

/// KLT tracking: features × levels × iters × window-pixels.
[[nodiscard]] inline StageProfile klt_profile(const PipelineCounts& c) {
    const double win = static_cast<double>((2 * c.klt_window_half + 1) * (2 * c.klt_window_half + 1));
    const double work = c.avg_tracked_features * static_cast<double>(c.pyramid_levels) *
                        static_cast<double>(c.klt_iters) * win * static_cast<double>(c.frames);
    return {"klt", work * coeff::kKltFlopsPerWinPx, work * coeff::kBytesPerPx};
}

/// MSCKF covariance propagation: ~2·D³ per IMU sample (F·P·Fᵀ on the D×D
/// error-state covariance).
[[nodiscard]] inline StageProfile msckf_propagate_profile(const PipelineCounts& c) {
    const double d = c.avg_state_dim;
    const double flops = 2.0 * d * d * d * static_cast<double>(c.imu_samples);
    const double bytes = d * d * coeff::kMatElemBytes * static_cast<double>(c.imu_samples);
    return {"msckf_propagate", flops, bytes};
}

/// MSCKF measurement update: ~ rows·D² (H·P·Hᵀ, K·H) + D³ (Joseph form),
/// per applied camera update.
[[nodiscard]] inline StageProfile msckf_update_profile(const PipelineCounts& c) {
    const double d = c.avg_state_dim;
    const double rows = c.avg_update_rows;
    const double flops = (rows * d * d + 2.0 * d * d * d) * static_cast<double>(c.camera_updates);
    const double bytes = (d * d + rows * d) * coeff::kMatElemBytes * static_cast<double>(c.camera_updates);
    return {"msckf_update", flops, bytes};
}

/// The full per-stage profile for the run.
[[nodiscard]] inline std::vector<StageProfile> build_pipeline_profile(const PipelineCounts& c) {
    return {pyramid_profile(c), fast_profile(c), klt_profile(c), msckf_propagate_profile(c), msckf_update_profile(c)};
}

}  // namespace branes::bench

#endif  // BRANES_BENCH_OPERATOR_PROFILE_HPP
