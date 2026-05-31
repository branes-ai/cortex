// SPDX-License-Identifier: MIT
//
// branes/sdk/sfm/two_view.hpp — two-view structure-from-motion bootstrap for
// dynamic visual-inertial initialization (issue #228, epic #211). Given matched
// normalized image correspondences from two frames, recover the relative camera
// pose (rotation + translation direction, the latter only up to scale — metric
// scale is resolved later by the IMU) and triangulate the inlier points.
//
// Pipeline: Hartley-normalized 8-point essential-matrix estimate → rank-2
// projection → decomposition into the four (R, ±t) candidates → cheirality
// vote (most points in front of both cameras) → RANSAC over the Sampson
// distance for outlier rejection → refit on the consensus set.
//
// Clean-room from Hartley & Zisserman, "Multiple View Geometry in Computer
// Vision" (2nd ed.) §9-§11 and Longuet-Higgins (1981) — no third-party SfM
// source. Header-only, C++20, type-generic.

#ifndef BRANES_SDK_SFM_TWO_VIEW_HPP
#define BRANES_SDK_SFM_TWO_VIEW_HPP

#include <branes/math/lie/detail.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace branes::sdk::sfm {

namespace ld = math::lie::detail;

template <math::Scalar T>
using Vec2 = ld::Vec<T, 2>;
template <math::Scalar T>
using Vec3 = ld::Vec<T, 3>;
template <math::Scalar T>
using Mat3 = ld::Mat<T, 3, 3>;

/// Symmetric eigendecomposition A = V·diag(w)·Vᵀ by cyclic Jacobi rotations.
/// `A` is read by value; on return the columns of `V` are eigenvectors and
/// `w` the eigenvalues (unsorted). Generic over the dimension N — used here at
/// N=9 (the 8-point null space) and N=3 (3×3 SVD via the normal matrices).
/// Clean-room generalization of the 4×4 Horn-alignment solver.
template <math::Scalar T, std::size_t N>
void jacobi_eigh(ld::Mat<T, N, N> A, ld::Mat<T, N, N>& V, ld::Vec<T, N>& w) {
    V = ld::Mat<T, N, N>::identity();
    for (int sweep = 0; sweep < 100; ++sweep) {
        T off = T{0};
        for (std::size_t p = 0; p < N; ++p)
            for (std::size_t q = p + 1; q < N; ++q)
                off += A(p, q) * A(p, q);
        if (!(off > T{0}))
            break;  // already diagonal to working precision
        for (std::size_t p = 0; p < N; ++p) {
            for (std::size_t q = p + 1; q < N; ++q) {
                const T apq = A(p, q);
                if (apq == T{0})
                    continue;
                const T theta = (A(q, q) - A(p, p)) / (T{2} * apq);
                const T t = (theta >= T{0} ? T{1} : T{-1}) / (ld::abs_(theta) + ld::sqrt_(theta * theta + T{1}));
                const T c = T{1} / ld::sqrt_(t * t + T{1});
                const T s = t * c;
                // Apply the Givens rotation G(p,q,θ) on both sides: A ← GᵀAG.
                for (std::size_t k = 0; k < N; ++k) {
                    const T akp = A(k, p), akq = A(k, q);
                    A(k, p) = c * akp - s * akq;
                    A(k, q) = s * akp + c * akq;
                }
                for (std::size_t k = 0; k < N; ++k) {
                    const T apk = A(p, k), aqk = A(q, k);
                    A(p, k) = c * apk - s * aqk;
                    A(q, k) = s * apk + c * aqk;
                }
                for (std::size_t k = 0; k < N; ++k) {
                    const T vkp = V(k, p), vkq = V(k, q);
                    V(k, p) = c * vkp - s * vkq;
                    V(k, q) = s * vkp + c * vkq;
                }
            }
        }
    }
    for (std::size_t i = 0; i < N; ++i)
        w[i] = A(i, i);
}

/// Thin SVD of a 3×3 matrix M = U·diag(S)·Vᵀ with the singular values in S
/// sorted descending. Built from the symmetric eigendecompositions of MᵀM
/// (gives V and S²) — U is then formed column-by-column as M·vᵢ/σᵢ, which
/// fixes the sign coupling between U and V; the deficient column is closed
/// with U₂ = U₀×U₁ so U is a proper orthonormal frame.
template <math::Scalar T>
struct Svd3 {
    Mat3<T> U{};
    Vec3<T> S{};
    Mat3<T> V{};
};

template <math::Scalar T>
[[nodiscard]] Svd3<T> svd3(const Mat3<T>& M) {
    const Mat3<T> MtM = ld::transpose(M) * M;
    Mat3<T> V;
    Vec3<T> w;
    jacobi_eigh<T, 3>(MtM, V, w);

    // Sort eigenpairs by eigenvalue descending.
    std::array<std::size_t, 3> idx{0, 1, 2};
    for (std::size_t a = 0; a < 3; ++a)
        for (std::size_t b = a + 1; b < 3; ++b)
            if (w[idx[b]] > w[idx[a]]) {
                const std::size_t tmp = idx[a];
                idx[a] = idx[b];
                idx[b] = tmp;
            }

    Svd3<T> r;
    for (std::size_t c = 0; c < 3; ++c) {
        const std::size_t j = idx[c];
        const T sv = ld::sqrt_(w[j] > T{0} ? w[j] : T{0});
        r.S[c] = sv;
        for (std::size_t i = 0; i < 3; ++i)
            r.V(i, c) = V(i, j);
    }
    // U columns: uᵢ = M·vᵢ / σᵢ for the well-conditioned singular values.
    for (std::size_t c = 0; c < 2; ++c) {
        Vec3<T> v{{r.V(0, c), r.V(1, c), r.V(2, c)}};
        Vec3<T> u = M * v;
        const T n = ld::norm(u);
        if (n > T{0})
            u = u * (T{1} / n);
        for (std::size_t i = 0; i < 3; ++i)
            r.U(i, c) = u[i];
    }
    const Vec3<T> u0{{r.U(0, 0), r.U(1, 0), r.U(2, 0)}};
    const Vec3<T> u1{{r.U(0, 1), r.U(1, 1), r.U(2, 1)}};
    const Vec3<T> u2 = ld::cross(u0, u1);
    for (std::size_t i = 0; i < 3; ++i)
        r.U(i, 2) = u2[i];
    return r;
}

/// Tuning for the RANSAC essential-matrix estimate.
template <math::Scalar T>
struct TwoViewOptions {
    T sampson_threshold = T{1} / T{500};  ///< inlier gate on the Sampson distance (normalized coords)
    std::size_t max_iterations = 256;     ///< RANSAC hypotheses
    std::size_t min_inliers = 12;         ///< reject the solution below this many inliers
    T min_inlier_ratio = T{1} / T{2};     ///< …and below this fraction of correspondences
};

/// Recovered two-view geometry. `R_1_0` and `t_1_0_unit` give the second
/// camera's pose relative to the first (x₁ = R_1_0·x₀ + s·t for some s>0); the
/// translation is a unit direction only — metric scale is unobservable from
/// vision alone. `inliers` and `points_cam0` are aligned **1:1**: for each kept
/// correspondence `inliers[j]` (an index into the input), `points_cam0[j]` is
/// its triangulated landmark in camera-0 coordinates (up to the same scale).
/// Only correspondences that pass both the Sampson gate and positive-cheirality
/// triangulation are retained.
template <math::Scalar T>
struct TwoViewResult {
    bool success = false;
    Mat3<T> R_1_0{};
    Vec3<T> t_1_0_unit{};
    std::vector<Vec3<T>> points_cam0;
    std::vector<std::size_t> inliers;
};

namespace detail {

/// Hartley isotropic normalization: translate to centroid, scale so the mean
/// distance to the origin is √2. Returns the 3×3 similarity T such that the
/// conditioned point is T·[x; y; 1].
template <math::Scalar T>
[[nodiscard]] Mat3<T> hartley_transform(std::span<const Vec2<T>> pts) {
    Vec2<T> mean{{T{0}, T{0}}};
    for (const auto& p : pts)
        mean = mean + p;
    if (!pts.empty())
        mean = mean * (T{1} / static_cast<T>(pts.size()));
    T mean_dist = T{0};
    for (const auto& p : pts)
        mean_dist += ld::norm(Vec2<T>{{p[0] - mean[0], p[1] - mean[1]}});
    if (!pts.empty())
        mean_dist /= static_cast<T>(pts.size());
    const T s = mean_dist > T{0} ? ld::sqrt_(T{2}) / mean_dist : T{1};
    Mat3<T> Tn = Mat3<T>::identity();
    Tn(0, 0) = s;
    Tn(1, 1) = s;
    Tn(0, 2) = -s * mean[0];
    Tn(1, 2) = -s * mean[1];
    return Tn;
}

/// Normalized 8-point essential matrix from ≥8 correspondences (indices into
/// x0/x1). Returns false on a degenerate configuration.
template <math::Scalar T>
[[nodiscard]] bool essential_8point(std::span<const Vec2<T>> x0,
                                    std::span<const Vec2<T>> x1,
                                    std::span<const std::size_t> idx,
                                    Mat3<T>& E) {
    const std::size_t n = idx.size();
    if (n < 8)
        return false;
    std::vector<Vec2<T>> p0, p1;
    p0.reserve(n);
    p1.reserve(n);
    for (std::size_t k : idx) {
        p0.push_back(x0[k]);
        p1.push_back(x1[k]);
    }
    const Mat3<T> T0 = hartley_transform<T>(p0);
    const Mat3<T> T1 = hartley_transform<T>(p1);

    // Build the n×9 epipolar constraint and its 9×9 normal matrix AᵀA.
    ld::Mat<T, 9, 9> AtA{};
    for (std::size_t k = 0; k < n; ++k) {
        const Vec3<T> a = T0 * Vec3<T>{{p0[k][0], p0[k][1], T{1}}};
        const Vec3<T> b = T1 * Vec3<T>{{p1[k][0], p1[k][1], T{1}}};
        const std::array<T, 9> row{b[0] * a[0],
                                   b[0] * a[1],
                                   b[0] * a[2],
                                   b[1] * a[0],
                                   b[1] * a[1],
                                   b[1] * a[2],
                                   b[2] * a[0],
                                   b[2] * a[1],
                                   b[2] * a[2]};
        for (std::size_t i = 0; i < 9; ++i)
            for (std::size_t j = 0; j < 9; ++j)
                AtA(i, j) += row[i] * row[j];
    }
    ld::Mat<T, 9, 9> V;
    ld::Vec<T, 9> w;
    jacobi_eigh<T, 9>(AtA, V, w);
    // Null space = eigenvector of the smallest eigenvalue.
    std::size_t jmin = 0;
    for (std::size_t j = 1; j < 9; ++j)
        if (w[j] < w[jmin])
            jmin = j;
    Mat3<T> Eh{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            Eh(i, j) = V(i * 3 + j, jmin);

    // De-normalize FIRST (Ê is in Hartley-conditioned coordinates; T0/T1 are
    // similarities, used only to condition the linear solve): E_raw = T1ᵀ·Ê·T0.
    const Mat3<T> E_raw = ld::transpose(T1) * Eh * T0;
    // THEN project onto the essential manifold in the calibrated space — rank 2
    // with equal nonzero singular values. Enforcing this on the *normalized* Ê
    // instead would be undone by the non-orthogonal de-normalization, leaving E
    // not-quite-essential and the recovered rotation biased. Scale is arbitrary.
    Svd3<T> s = svd3<T>(E_raw);
    Mat3<T> D{};
    D(0, 0) = T{1};
    D(1, 1) = T{1};
    E = s.U * D * ld::transpose(s.V);
    return true;
}

/// Triangulate one correspondence under P0 = [I|0], P1 = [R|t] by the linear
/// DLT; returns the point in camera-0 coordinates and whether it is in front
/// of both cameras (positive depth).
template <math::Scalar T>
[[nodiscard]] bool triangulate(const Vec2<T>& x0, const Vec2<T>& x1, const Mat3<T>& R, const Vec3<T>& t, Vec3<T>& X) {
    // Rows of the 4×4 DLT system A·[X;1] = 0; solve via the smallest right
    // singular vector of AᵀA (4×4).
    std::array<std::array<T, 4>, 4> rows{};
    auto setrow = [](std::array<T, 4>& r, T a, T b, T c, T d) {
        r[0] = a;
        r[1] = b;
        r[2] = c;
        r[3] = d;
    };
    // Camera 0: P0 = [I|0]. x·P2 - P0, y·P2 - P1.
    setrow(rows[0], -T{1}, T{0}, x0[0], T{0});
    setrow(rows[1], T{0}, -T{1}, x0[1], T{0});
    // Camera 1: P1 = [R|t].
    for (std::size_t c = 0; c < 3; ++c) {
        rows[2][c] = x1[0] * R(2, c) - R(0, c);
        rows[3][c] = x1[1] * R(2, c) - R(1, c);
    }
    rows[2][3] = x1[0] * t[2] - t[0];
    rows[3][3] = x1[1] * t[2] - t[1];

    ld::Mat<T, 4, 4> AtA{};
    for (std::size_t k = 0; k < 4; ++k)
        for (std::size_t i = 0; i < 4; ++i)
            for (std::size_t j = 0; j < 4; ++j)
                AtA(i, j) += rows[k][i] * rows[k][j];
    ld::Mat<T, 4, 4> V;
    ld::Vec<T, 4> w;
    jacobi_eigh<T, 4>(AtA, V, w);
    std::size_t jmin = 0;
    for (std::size_t j = 1; j < 4; ++j)
        if (w[j] < w[jmin])
            jmin = j;
    const T hw = V(3, jmin);
    if (hw == T{0})
        return false;
    X = Vec3<T>{{V(0, jmin) / hw, V(1, jmin) / hw, V(2, jmin) / hw}};
    const Vec3<T> Xc1 = R * X + t;  // same point in camera-1 frame
    return X[2] > T{0} && Xc1[2] > T{0};
}

/// *Squared* Sampson distance — the first-order geometric reprojection error of
/// a correspondence to E. Returned squared to avoid a sqrt on the RANSAC hot
/// path; callers gate against threshold².
template <math::Scalar T>
[[nodiscard]] T sampson_squared(const Mat3<T>& E, const Vec2<T>& a, const Vec2<T>& b) {
    const Vec3<T> x0{{a[0], a[1], T{1}}};
    const Vec3<T> x1{{b[0], b[1], T{1}}};
    const Vec3<T> Ex0 = E * x0;
    const Vec3<T> Etx1 = ld::transpose(E) * x1;
    const T num = ld::dot(x1, Ex0);
    const T den = Ex0[0] * Ex0[0] + Ex0[1] * Ex0[1] + Etx1[0] * Etx1[0] + Etx1[1] * Etx1[1];
    if (!(den > T{0}))
        return T{0};
    return (num * num) / den;
}

}  // namespace detail

/// Decompose an essential matrix into its two rotation candidates and the
/// translation direction (the four poses are {R0,R1} × {±t}).
template <math::Scalar T>
void decompose_essential(const Mat3<T>& E, Mat3<T>& R0, Mat3<T>& R1, Vec3<T>& t) {
    Svd3<T> s = svd3<T>(E);
    Mat3<T> U = s.U, V = s.V;
    // Ensure proper rotations (det(U)=det(V)=+1) before composing.
    auto det3 = [](const Mat3<T>& m) {
        return m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) - m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
               m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
    };
    if (det3(U) < T{0})
        for (std::size_t i = 0; i < 3; ++i)
            U(i, 2) = -U(i, 2);
    if (det3(V) < T{0})
        for (std::size_t i = 0; i < 3; ++i)
            V(i, 2) = -V(i, 2);
    Mat3<T> W{};
    W(0, 1) = -T{1};
    W(1, 0) = T{1};
    W(2, 2) = T{1};
    const Mat3<T> Vt = ld::transpose(V);
    R0 = U * W * Vt;
    R1 = U * ld::transpose(W) * Vt;
    t = Vec3<T>{{U(0, 2), U(1, 2), U(2, 2)}};
}

/// Estimate the relative pose from matched normalized correspondences x0[k]↔x1[k].
/// Returns the cheirality-consistent (R, t-direction) plus triangulated inliers.
template <math::Scalar T>
[[nodiscard]] TwoViewResult<T>
estimate_relative_pose(std::span<const Vec2<T>> x0, std::span<const Vec2<T>> x1, const TwoViewOptions<T>& opt = {}) {
    TwoViewResult<T> out;
    const std::size_t n = x0.size();
    if (n != x1.size() || n < 8)
        return out;

    // Deterministic LCG so the RANSAC is reproducible across runs/platforms.
    std::uint64_t rng = 0x2545F4914F6CDD1DULL ^ static_cast<std::uint64_t>(n);
    auto next = [&rng](std::size_t m) {
        rng = rng * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<std::size_t>((rng >> 33) % m);
    };

    std::vector<std::size_t> best_inliers;
    Mat3<T> best_E{};
    std::array<std::size_t, 8> sample{};
    for (std::size_t it = 0; it < opt.max_iterations; ++it) {
        // Draw 8 distinct indices.
        for (std::size_t s = 0; s < 8; ++s) {
            bool dup = true;
            while (dup) {
                sample[s] = next(n);
                dup = false;
                for (std::size_t q = 0; q < s; ++q)
                    if (sample[q] == sample[s])
                        dup = true;
            }
        }
        Mat3<T> E;
        if (!detail::essential_8point<T>(x0, x1, std::span<const std::size_t>{sample}, E))
            continue;
        std::vector<std::size_t> inliers;
        for (std::size_t k = 0; k < n; ++k)
            if (detail::sampson_squared<T>(E, x0[k], x1[k]) < opt.sampson_threshold * opt.sampson_threshold)
                inliers.push_back(k);
        if (inliers.size() > best_inliers.size()) {
            best_inliers = std::move(inliers);
            best_E = E;
        }
    }

    if (best_inliers.size() < opt.min_inliers ||
        static_cast<T>(best_inliers.size()) < opt.min_inlier_ratio * static_cast<T>(n))
        return out;

    // Refit on the full consensus set for a stable essential matrix; keep the
    // minimal-sample estimate if the (non-degenerate) refit somehow fails.
    Mat3<T> E = best_E;
    Mat3<T> refit;
    if (detail::essential_8point<T>(x0, x1, std::span<const std::size_t>{best_inliers}, refit))
        E = refit;

    // Pick the (R, ±t) with the most points triangulating in front of both
    // cameras. For the winner, `inliers` and `points_cam0` are kept strictly
    // 1:1 — only the Sampson inliers that also triangulate with positive
    // cheirality are retained, so points_cam0[j] is the landmark for inliers[j].
    Mat3<T> R0, R1;
    Vec3<T> t;
    decompose_essential<T>(E, R0, R1, t);
    const Mat3<T> Rs[2] = {R0, R1};
    std::size_t best_count = 0;
    for (const Mat3<T>& R : Rs) {
        for (int sign = -1; sign <= 1; sign += 2) {
            const Vec3<T> tc = t * static_cast<T>(sign);
            std::vector<Vec3<T>> pts;
            std::vector<std::size_t> kept;
            for (std::size_t k : best_inliers) {
                Vec3<T> X;
                if (detail::triangulate<T>(x0[k], x1[k], R, tc, X)) {
                    pts.push_back(X);
                    kept.push_back(k);
                }
            }
            if (pts.size() > best_count) {
                best_count = pts.size();
                out.R_1_0 = R;
                out.t_1_0_unit = tc;
                out.points_cam0 = std::move(pts);
                out.inliers = std::move(kept);
            }
        }
    }
    out.success = best_count >= opt.min_inliers;
    return out;
}

}  // namespace branes::sdk::sfm

#endif  // BRANES_SDK_SFM_TWO_VIEW_HPP
