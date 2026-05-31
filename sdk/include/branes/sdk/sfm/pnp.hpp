// SPDX-License-Identifier: MIT
//
// branes/sdk/sfm/pnp.hpp — perspective-n-point resectioning for the dynamic
// visual-inertial init window (issue #235, epic #211). Given 3D landmarks and
// their normalized image observations in a new frame, recover that frame's
// camera pose in the landmarks' coordinate frame. This is what extends the
// two-view bootstrap (#228) into a multi-frame SfM window with a *consistent*
// scale: the first pair fixes the points, every later frame is resectioned
// against them.
//
// Pipeline: linear DLT (the 3×4 projection matrix as a 12-vector null space) →
// orthonormalize the rotation block (SVD / nearest-rotation) → recover scale +
// cheirality sign → RANSAC over the reprojection error → refit on the
// consensus set. Reuses the generic Jacobi eigensolver and 3×3 SVD from
// two_view.hpp.
//
// Clean-room from Hartley & Zisserman, "Multiple View Geometry" §7 (DLT camera
// resectioning) — no third-party SfM source. Header-only, C++20, type-generic.

#ifndef BRANES_SDK_SFM_PNP_HPP
#define BRANES_SDK_SFM_PNP_HPP

#include <branes/sdk/sfm/two_view.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

namespace branes::sdk::sfm {

/// Tuning for the RANSAC DLT-PnP estimate.
template <math::Scalar T>
struct PnpOptions {
    T reprojection_threshold = T{1} / T{200};  ///< inlier gate on the normalized reprojection error
    std::size_t max_iterations = 256;          ///< RANSAC hypotheses
    std::size_t min_inliers = 8;               ///< reject the solution below this many inliers
    T min_inlier_ratio = T{1} / T{2};          ///< …and below this fraction of correspondences
};

/// Recovered camera pose: the new frame sees a landmark X (in the landmark
/// frame) at camera-frame point R·X + t, projecting to (·)/z. `inliers` indexes
/// the input correspondences whose reprojection passed the gate.
template <math::Scalar T>
struct PnpResult {
    bool success = false;
    Mat3<T> R{};
    Vec3<T> t{};
    std::vector<std::size_t> inliers;
};

namespace detail {

/// Linear DLT pose from ≥6 correspondences (indices into pts3d / obs2d).
/// Recovers the 3×4 projection P as the null vector of the 2n×12 system, splits
/// P = [M | p₄], fixes the overall scale α = ‖M row‖ and its sign by cheirality,
/// and orthonormalizes M/α to the nearest rotation. Returns false if degenerate.
template <math::Scalar T>
[[nodiscard]] bool pnp_dlt(std::span<const Vec3<T>> pts3d,
                           std::span<const Vec2<T>> obs2d,
                           std::span<const std::size_t> idx,
                           Mat3<T>& R,
                           Vec3<T>& t) {
    const std::size_t n = idx.size();
    if (n < 6)
        return false;

    // AᵀA (12×12) from the two DLT rows per correspondence. Unknown is
    // vec(P) = [p₁(4) p₂(4) p₃(4)] with pₖ the k-th row of P.
    ld::Mat<T, 12, 12> AtA{};
    auto fold = [&](const std::array<T, 12>& row) {
        for (std::size_t i = 0; i < 12; ++i)
            for (std::size_t j = 0; j < 12; ++j)
                AtA(i, j) += row[i] * row[j];
    };
    for (std::size_t k : idx) {
        const Vec3<T>& X = pts3d[k];
        const T u = obs2d[k][0], v = obs2d[k][1];
        const std::array<T, 4> Xh{X[0], X[1], X[2], T{1}};
        std::array<T, 12> rA{}, rB{};
        for (std::size_t c = 0; c < 4; ++c) {
            // Row A: v·(p₃·X) − (p₂·X) = 0.
            rA[4 + c] = -Xh[c];
            rA[8 + c] = v * Xh[c];
            // Row B: (p₁·X) − u·(p₃·X) = 0.
            rB[0 + c] = Xh[c];
            rB[8 + c] = -u * Xh[c];
        }
        fold(rA);
        fold(rB);
    }
    ld::Mat<T, 12, 12> V;
    ld::Vec<T, 12> w;
    jacobi_eigh<T, 12>(AtA, V, w);
    std::size_t jmin = 0;
    for (std::size_t j = 1; j < 12; ++j)
        if (w[j] < w[jmin])
            jmin = j;

    Mat3<T> M{};
    Vec3<T> p4{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t c = 0; c < 3; ++c)
            M(i, c) = V(i * 4 + c, jmin);
        p4[i] = V(i * 4 + 3, jmin);
    }

    // P is determined up to scale α (and sign). |M row| = |α| (rotation rows are
    // unit), so α = ‖M row 2‖. The cheirality fixes the sign: the projected
    // depth of a point in front is positive, and depth ∝ α·(P row 2 · Xₕ).
    const Vec3<T> m2{{M(2, 0), M(2, 1), M(2, 2)}};
    const T alpha_mag = ld::norm(m2);
    if (!(alpha_mag > T{0}))
        return false;
    T depth_sum = T{0};
    for (std::size_t k : idx)
        depth_sum += M(2, 0) * pts3d[k][0] + M(2, 1) * pts3d[k][1] + M(2, 2) * pts3d[k][2] + p4[2];
    const T alpha = (depth_sum >= T{0} ? T{1} : T{-1}) * alpha_mag;

    // R = nearest rotation to M/α; t = p₄/α.
    Mat3<T> Mn = M * (T{1} / alpha);
    Svd3<T> s = svd3<T>(Mn);
    R = s.U * ld::transpose(s.V);
    const auto det3 = [](const Mat3<T>& m) {
        return m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) - m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
               m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
    };
    if (det3(R) < T{0}) {
        Mat3<T> U = s.U;
        for (std::size_t i = 0; i < 3; ++i)
            U(i, 2) = -U(i, 2);
        R = U * ld::transpose(s.V);
    }
    t = p4 * (T{1} / alpha);
    return true;
}

/// Normalized reprojection error ‖π(R·X + t) − x‖ of a correspondence; large
/// (rejecting) when the point falls behind the camera.
template <math::Scalar T>
[[nodiscard]] T reprojection_error(const Mat3<T>& R, const Vec3<T>& t, const Vec3<T>& X, const Vec2<T>& x) {
    const Vec3<T> Xc = R * X + t;
    if (!(Xc[2] > T{0}))
        return std::numeric_limits<T>::max();
    const Vec2<T> proj{{Xc[0] / Xc[2], Xc[1] / Xc[2]}};
    return ld::norm(Vec2<T>{{proj[0] - x[0], proj[1] - x[1]}});
}

}  // namespace detail

/// Estimate the new frame's pose from 3D landmarks `pts3d` and their matched
/// normalized observations `obs2d` (`obs2d[k]` is where `pts3d[k]` is seen).
/// RANSAC-robust; returns the cheirality-consistent (R, t) and the inlier set.
template <math::Scalar T>
[[nodiscard]] PnpResult<T>
estimate_pose(std::span<const Vec3<T>> pts3d, std::span<const Vec2<T>> obs2d, const PnpOptions<T>& opt = {}) {
    PnpResult<T> out;
    const std::size_t n = pts3d.size();
    if (n != obs2d.size() || n < 6)
        return out;

    std::uint64_t rng = 0x9E3779B97F4A7C15ULL ^ static_cast<std::uint64_t>(n);
    auto next = [&rng](std::size_t m) {
        rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<std::size_t>((rng >> 33) % m);
    };

    std::vector<std::size_t> best_inliers;
    std::array<std::size_t, 6> sample{};
    for (std::size_t it = 0; it < opt.max_iterations; ++it) {
        for (std::size_t s = 0; s < 6; ++s) {
            bool dup = true;
            while (dup) {
                sample[s] = next(n);
                dup = false;
                for (std::size_t q = 0; q < s; ++q)
                    if (sample[q] == sample[s])
                        dup = true;
            }
        }
        Mat3<T> R;
        Vec3<T> t;
        if (!detail::pnp_dlt<T>(pts3d, obs2d, std::span<const std::size_t>{sample}, R, t))
            continue;
        std::vector<std::size_t> inliers;
        for (std::size_t k = 0; k < n; ++k)
            if (detail::reprojection_error<T>(R, t, pts3d[k], obs2d[k]) < opt.reprojection_threshold)
                inliers.push_back(k);
        if (inliers.size() > best_inliers.size())
            best_inliers = std::move(inliers);
    }

    if (best_inliers.size() < opt.min_inliers ||
        static_cast<T>(best_inliers.size()) < opt.min_inlier_ratio * static_cast<T>(n))
        return out;

    // Refit on the consensus set, then keep only the still-consistent points.
    Mat3<T> R;
    Vec3<T> t;
    if (!detail::pnp_dlt<T>(pts3d, obs2d, std::span<const std::size_t>{best_inliers}, R, t))
        return out;
    std::vector<std::size_t> kept;
    for (std::size_t k = 0; k < n; ++k)
        if (detail::reprojection_error<T>(R, t, pts3d[k], obs2d[k]) < opt.reprojection_threshold)
            kept.push_back(k);
    if (kept.size() < opt.min_inliers || static_cast<T>(kept.size()) < opt.min_inlier_ratio * static_cast<T>(n))
        return out;

    out.success = true;
    out.R = R;
    out.t = t;
    out.inliers = std::move(kept);
    return out;
}

}  // namespace branes::sdk::sfm

#endif  // BRANES_SDK_SFM_PNP_HPP
