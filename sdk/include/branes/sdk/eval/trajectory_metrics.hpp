// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/trajectory_metrics.hpp — trajectory error metrics for
// VIO evaluation: Absolute Trajectory Error (ATE) and Relative Pose Error
// (RPE).
//
// ATE rigidly aligns the estimated trajectory to the ground truth (the
// gauge freedom of a VIO solution) and reports the translational RMSE.
// The alignment is Horn's closed-form absolute-orientation solution
// (B.K.P. Horn, "Closed-form solution of absolute orientation using unit
// quaternions", JOSA-A 1987): the optimal rotation is the eigenvector of
// the largest eigenvalue of a 4×4 profile matrix, found here by a cyclic
// Jacobi eigensolver (the top two eigenvalues are routinely near-
// degenerate, which makes power iteration far too slow). RPE compares
// relative motions over a fixed step and reports
// the translational RMSE. Both are clean-room from the definitions — no
// ov_eval / evo source contact.
//
// Two more for comparing VIO estimators (docs/benchmarks/euroc-baseline.mdx):
// a position + yaw (4-DoF) alignment, since a VIO observes roll and pitch
// but not global yaw or position, and RPE over sub-trajectories of a fixed
// travelled distance rather than a fixed index step, so the score does not
// depend on the estimator's output rate.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_TRAJECTORY_METRICS_HPP
#define BRANES_SDK_EVAL_TRAJECTORY_METRICS_HPP

#include <branes/math/lie/se3.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace branes::sdk::eval {

/// A timestamped pose (T_world_body), seconds.
template <math::Scalar T>
struct StampedPose {
    double t_s = 0.0;
    math::lie::SE3<T> pose{};
};

namespace detail {

template <math::Scalar T>
using Vec3 = math::lie::detail::Vec<T, 3>;
template <math::Scalar T>
using Vec4 = math::lie::detail::Vec<T, 4>;
template <math::Scalar T>
using Mat4 = math::lie::detail::Mat<T, 4, 4>;

/// Symmetric eigendecomposition of a 4×4 matrix by cyclic Jacobi
/// rotations (textbook method): on return `A` is diagonalized in place,
/// `eig` holds the eigenvalues, and the columns of `V` are the
/// eigenvectors. Used for Horn's 4×4 profile matrix, whose top two
/// eigenvalues are routinely near-degenerate (power iteration is far too
/// slow there).
template <math::Scalar T>
void jacobi_eigh4(Mat4<T>& A, Mat4<T>& V, Vec4<T>& eig) {
    V = Mat4<T>::identity();
    for (int sweep = 0; sweep < 50; ++sweep) {
        T off = T{0};
        for (int p = 0; p < 4; ++p)
            for (int q = p + 1; q < 4; ++q)
                off += A(p, q) * A(p, q);
        if (!(off > T(1e-30)))
            break;
        for (int p = 0; p < 4; ++p)
            for (int q = p + 1; q < 4; ++q) {
                if (!(std::abs(A(p, q)) > T(1e-30)))
                    continue;
                const T theta = (A(q, q) - A(p, p)) / (T{2} * A(p, q));
                const T sign = theta >= T{0} ? T{1} : T{-1};
                const T t = sign / (std::abs(theta) + std::sqrt(theta * theta + T{1}));
                const T c = T{1} / std::sqrt(t * t + T{1});
                const T s = t * c;
                for (int k = 0; k < 4; ++k) {  // columns p,q
                    const T akp = A(k, p), akq = A(k, q);
                    A(k, p) = c * akp - s * akq;
                    A(k, q) = s * akp + c * akq;
                }
                for (int k = 0; k < 4; ++k) {  // rows p,q
                    const T apk = A(p, k), aqk = A(q, k);
                    A(p, k) = c * apk - s * aqk;
                    A(q, k) = s * apk + c * aqk;
                }
                for (int k = 0; k < 4; ++k) {  // accumulate eigenvectors
                    const T vkp = V(k, p), vkq = V(k, q);
                    V(k, p) = c * vkp - s * vkq;
                    V(k, q) = s * vkp + c * vkq;
                }
            }
    }
    for (int i = 0; i < 4; ++i)
        eig[i] = A(i, i);
}

/// Optimal rigid rotation (as a unit quaternion w,x,y,z) aligning `src`
/// onto `dst` after centering — Horn's profile-matrix method. `src`/`dst`
/// are centered point sets of equal length (≥ 1).
template <math::Scalar T>
[[nodiscard]] math::lie::SO3<T> horn_rotation(const std::vector<Vec3<T>>& src, const std::vector<Vec3<T>>& dst) {
    // Sum of products S_ab = Σ src_a · dst_b.
    T s[3][3] = {{T{0}, T{0}, T{0}}, {T{0}, T{0}, T{0}}, {T{0}, T{0}, T{0}}};
    for (std::size_t i = 0; i < src.size(); ++i)
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b)
                s[a][b] += src[i][static_cast<std::size_t>(a)] * dst[i][static_cast<std::size_t>(b)];

    // Horn's symmetric 4×4 profile matrix N (quaternion order w,x,y,z).
    Mat4<T> N{};
    N(0, 0) = s[0][0] + s[1][1] + s[2][2];
    N(0, 1) = s[1][2] - s[2][1];
    N(0, 2) = s[2][0] - s[0][2];
    N(0, 3) = s[0][1] - s[1][0];
    N(1, 1) = s[0][0] - s[1][1] - s[2][2];
    N(1, 2) = s[0][1] + s[1][0];
    N(1, 3) = s[2][0] + s[0][2];
    N(2, 2) = -s[0][0] + s[1][1] - s[2][2];
    N(2, 3) = s[1][2] + s[2][1];
    N(3, 3) = -s[0][0] - s[1][1] + s[2][2];
    N(1, 0) = N(0, 1);
    N(2, 0) = N(0, 2);
    N(2, 1) = N(1, 2);
    N(3, 0) = N(0, 3);
    N(3, 1) = N(1, 3);
    N(3, 2) = N(2, 3);

    // The optimal quaternion is the eigenvector of the largest eigenvalue.
    Mat4<T> V;
    Vec4<T> eig;
    jacobi_eigh4<T>(N, V, eig);
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (eig[i] > eig[best])
            best = i;
    const Vec4<T> q{{V(0, best), V(1, best), V(2, best), V(3, best)}};
    return math::lie::SO3<T>(q);  // SO3 normalizes
}

}  // namespace detail

/// Rigid (SE3) alignment of `estimated` onto `reference` positions,
/// returned as (R, t) with reference ≈ R·estimated + t. Both vectors must
/// be the same non-empty length.
template <math::Scalar T>
struct Alignment {
    math::lie::SO3<T> R{};
    detail::Vec3<T> t{};
};

template <math::Scalar T>
[[nodiscard]] Alignment<T> align_rigid(const std::vector<detail::Vec3<T>>& estimated,
                                       const std::vector<detail::Vec3<T>>& reference) {
    const std::size_t n = estimated.size();
    if (n == 0 || reference.size() != n)
        throw std::invalid_argument("align_rigid: empty or mismatched trajectories");

    detail::Vec3<T> mean_e{}, mean_r{};
    for (std::size_t i = 0; i < n; ++i) {
        mean_e = mean_e + estimated[i];
        mean_r = mean_r + reference[i];
    }
    const T inv = T{1} / static_cast<T>(n);
    mean_e = mean_e * inv;
    mean_r = mean_r * inv;

    std::vector<detail::Vec3<T>> ce(n), cr(n);
    for (std::size_t i = 0; i < n; ++i) {
        ce[i] = estimated[i] - mean_e;
        cr[i] = reference[i] - mean_r;
    }

    Alignment<T> a;
    a.R = detail::horn_rotation<T>(ce, cr);
    a.t = mean_r - a.R * mean_e;
    return a;
}

/// Matched (estimated, reference) pose sublists, element-wise aligned.
template <math::Scalar T>
struct Associated {
    std::vector<StampedPose<T>> estimated;
    std::vector<StampedPose<T>> reference;
};

/// Match each estimated pose to the nearest-in-time reference pose within
/// `max_dt` seconds (both lists ascending by time). Estimated poses with
/// no reference inside the window are dropped.
template <math::Scalar T>
[[nodiscard]] Associated<T> associate(const std::vector<StampedPose<T>>& estimated,
                                      const std::vector<StampedPose<T>>& reference,
                                      double max_dt = 0.02) {
    Associated<T> out;
    if (reference.empty())
        return out;
    std::size_t j = 0;
    for (const auto& e : estimated) {
        // Advance j to the reference pose nearest e.t_s.
        while (j + 1 < reference.size() && std::abs(reference[j + 1].t_s - e.t_s) <= std::abs(reference[j].t_s - e.t_s))
            ++j;
        if (std::abs(reference[j].t_s - e.t_s) <= max_dt) {
            out.estimated.push_back(e);
            out.reference.push_back(reference[j]);
        }
    }
    return out;
}

/// Position + yaw (4-DoF) alignment of `estimated` onto `reference`: the
/// rotation about world z and the translation minimizing
/// Σ‖reference − (R·estimated + t)‖². After centering, the cost depends on
/// the yaw θ only through −2(c·cos θ + s·sin θ) with
/// c = Σ(aₓbₓ + a_y b_y) and s = Σ(aₓb_y − a_y bₓ), so θ = atan2(s, c).
/// Same preconditions as `align_rigid`.
template <math::Scalar T>
[[nodiscard]] Alignment<T> align_yaw(const std::vector<detail::Vec3<T>>& estimated,
                                     const std::vector<detail::Vec3<T>>& reference) {
    const std::size_t n = estimated.size();
    if (n == 0 || reference.size() != n)
        throw std::invalid_argument("align_yaw: empty or mismatched trajectories");

    detail::Vec3<T> mean_e{}, mean_r{};
    for (std::size_t i = 0; i < n; ++i) {
        mean_e = mean_e + estimated[i];
        mean_r = mean_r + reference[i];
    }
    const T inv = T{1} / static_cast<T>(n);
    mean_e = mean_e * inv;
    mean_r = mean_r * inv;

    T c = T{0}, s = T{0};
    for (std::size_t i = 0; i < n; ++i) {
        const detail::Vec3<T> a = estimated[i] - mean_e;
        const detail::Vec3<T> b = reference[i] - mean_r;
        c += a[0] * b[0] + a[1] * b[1];
        s += a[0] * b[1] - a[1] * b[0];
    }
    Alignment<T> al;
    al.R = math::lie::SO3<T>::exp(detail::Vec3<T>{{T{0}, T{0}, std::atan2(s, c)}});
    al.t = mean_r - al.R * mean_e;
    return al;
}

namespace detail {

/// RMSE of reference − (R·estimated + t) over matched positions.
template <math::Scalar T>
[[nodiscard]] T
aligned_rmse(const std::vector<Vec3<T>>& estimated, const std::vector<Vec3<T>>& reference, const Alignment<T>& a) {
    T sum = T{0};
    for (std::size_t i = 0; i < estimated.size(); ++i) {
        const Vec3<T> e = reference[i] - (a.R * estimated[i] + a.t);
        sum += math::lie::detail::dot(e, e);
    }
    return std::sqrt(sum / static_cast<T>(estimated.size()));
}

template <math::Scalar T>
[[nodiscard]] std::vector<Vec3<T>> translations(const std::vector<StampedPose<T>>& poses) {
    std::vector<Vec3<T>> p;
    p.reserve(poses.size());
    for (const auto& s : poses)
        p.push_back(s.pose.translation());
    return p;
}

}  // namespace detail

/// Absolute Trajectory Error: translational RMSE after rigid alignment.
/// `estimated` and `reference` are matched element-wise (same length).
template <math::Scalar T>
[[nodiscard]] T ate_rmse(const std::vector<detail::Vec3<T>>& estimated, const std::vector<detail::Vec3<T>>& reference) {
    return detail::aligned_rmse<T>(estimated, reference, align_rigid(estimated, reference));
}

/// ATE after a position + yaw (4-DoF) alignment (`align_yaw`). Never below
/// `ate_rmse` on the same input, since the rigid alignment is the wider search.
template <math::Scalar T>
[[nodiscard]] T ate_rmse_4dof(const std::vector<detail::Vec3<T>>& estimated,
                              const std::vector<detail::Vec3<T>>& reference) {
    return detail::aligned_rmse<T>(estimated, reference, align_yaw(estimated, reference));
}

/// Convenience overload: 4-DoF ATE on matched pose lists (uses translations).
template <math::Scalar T>
[[nodiscard]] T ate_rmse_4dof(const std::vector<StampedPose<T>>& estimated,
                              const std::vector<StampedPose<T>>& reference) {
    return ate_rmse_4dof<T>(detail::translations(estimated), detail::translations(reference));
}

/// Convenience overload: ATE on matched pose lists (uses translations).
template <math::Scalar T>
[[nodiscard]] T ate_rmse(const std::vector<StampedPose<T>>& estimated, const std::vector<StampedPose<T>>& reference) {
    return ate_rmse<T>(detail::translations(estimated), detail::translations(reference));
}

/// Relative Pose Error: translational RMSE of the relative motion over a
/// fixed index step `delta`. For each i, compares
/// (Pᵢ⁻¹·Pᵢ₊δ) between estimated and reference and takes the norm of the
/// translational part of their difference pose. Matched lists, same length.
template <math::Scalar T>
[[nodiscard]] T rpe_translation_rmse(const std::vector<StampedPose<T>>& estimated,
                                     const std::vector<StampedPose<T>>& reference,
                                     std::size_t delta = 1) {
    const std::size_t n = estimated.size();
    if (n == 0 || reference.size() != n)
        throw std::invalid_argument("rpe: empty or mismatched trajectories");
    if (delta == 0 || delta >= n)
        throw std::invalid_argument("rpe: delta must be in [1, n)");

    T sum = T{0};
    std::size_t count = 0;
    for (std::size_t i = 0; i + delta < n; ++i) {
        const auto rel_e = estimated[i].pose.inverse() * estimated[i + delta].pose;
        const auto rel_r = reference[i].pose.inverse() * reference[i + delta].pose;
        const auto err = rel_r.inverse() * rel_e;
        const auto te = err.translation();
        sum += math::lie::detail::dot(te, te);
        ++count;
    }
    return std::sqrt(sum / static_cast<T>(count));
}

/// KITTI-style relative drift over a fixed index step `delta`: the
/// translational error as a percentage of the ground-truth segment length,
/// and the rotational error in degrees per metre, averaged over all pairs.
/// Segments with ~zero ground-truth motion are skipped.
template <math::Scalar T>
struct RpeDrift {
    T translation_pct = T{0};     ///< mean ‖trans error‖ / segment length, ×100
    T rotation_deg_per_m = T{0};  ///< mean rotation error (deg) / segment length
};

template <math::Scalar T>
[[nodiscard]] RpeDrift<T> rpe_drift(const std::vector<StampedPose<T>>& estimated,
                                    const std::vector<StampedPose<T>>& reference,
                                    std::size_t delta = 1) {
    const std::size_t n = estimated.size();
    if (n == 0 || reference.size() != n)
        throw std::invalid_argument("rpe_drift: empty or mismatched trajectories");
    if (delta == 0 || delta >= n)
        throw std::invalid_argument("rpe_drift: delta must be in [1, n)");

    const T rad2deg = static_cast<T>(180) / std::numbers::pi_v<T>;
    T trans_frac_sum = T{0};
    T rot_per_m_sum = T{0};
    std::size_t count = 0;
    for (std::size_t i = 0; i + delta < n; ++i) {
        const auto rel_e = estimated[i].pose.inverse() * estimated[i + delta].pose;
        const auto rel_r = reference[i].pose.inverse() * reference[i + delta].pose;
        const auto rt = rel_r.translation();
        const T seg = std::sqrt(math::lie::detail::dot(rt, rt));
        if (!(seg > T(1e-6)))
            continue;  // no ground-truth motion over this segment
        const auto err = rel_r.inverse() * rel_e;
        const auto te = err.translation();
        const T trans_err = std::sqrt(math::lie::detail::dot(te, te));
        const auto axis = err.rotation().log();
        const T rot_err_deg = std::sqrt(math::lie::detail::dot(axis, axis)) * rad2deg;
        trans_frac_sum += trans_err / seg;
        rot_per_m_sum += rot_err_deg / seg;
        ++count;
    }
    RpeDrift<T> d;
    if (count > 0) {
        d.translation_pct = (trans_frac_sum / static_cast<T>(count)) * static_cast<T>(100);
        d.rotation_deg_per_m = rot_per_m_sum / static_cast<T>(count);
    }
    return d;
}

/// RPE over sub-trajectories of a fixed travelled distance, for one length.
template <math::Scalar T>
struct RpeSegment {
    double length_m = 0.0;   ///< ground-truth distance travelled over the segment
    T translation_m = T{0};  ///< mean ‖translation error‖ of the relative motion
    T rotation_deg = T{0};   ///< mean rotation error of the relative motion
    std::size_t pairs = 0;   ///< segments averaged
};

/// Relative Pose Error by travelled distance. For each start i and length L,
/// j is the first pose at least L metres further along the reference path;
/// the error of the pair is E = (Rᵢ⁻¹·Rⱼ)⁻¹·(Eᵢ⁻¹·Eⱼ), with R the reference and
/// E the estimate, scored by ‖t(E)‖ and the angle of its rotation. A length
/// longer than the whole path yields no entry. Matched lists, same length;
/// invariant to a global rigid transform of either list.
template <math::Scalar T>
[[nodiscard]] std::vector<RpeSegment<T>> rpe_by_distance(const std::vector<StampedPose<T>>& estimated,
                                                         const std::vector<StampedPose<T>>& reference,
                                                         const std::vector<double>& lengths_m) {
    const std::size_t n = estimated.size();
    if (n == 0 || reference.size() != n)
        throw std::invalid_argument("rpe_by_distance: empty or mismatched trajectories");

    std::vector<double> dist(n, 0.0);  // distance along the reference path
    for (std::size_t i = 1; i < n; ++i) {
        const auto d = reference[i].pose.translation() - reference[i - 1].pose.translation();
        dist[i] = dist[i - 1] + static_cast<double>(std::sqrt(math::lie::detail::dot(d, d)));
    }

    const T rad2deg = static_cast<T>(180) / std::numbers::pi_v<T>;
    std::vector<RpeSegment<T>> out;
    for (const double L : lengths_m) {
        if (!(L > 0.0))
            throw std::invalid_argument("rpe_by_distance: segment lengths must be positive");
        RpeSegment<T> seg;
        seg.length_m = L;
        T trans_sum = T{0}, rot_sum = T{0};
        std::size_t j = 0;
        for (std::size_t i = 0; i < n; ++i) {
            j = std::max(j, i);
            while (j < n && dist[j] < dist[i] + L)
                ++j;
            if (j >= n)
                break;  // dist is non-decreasing: no later start reaches L either
            const auto rel_e = estimated[i].pose.inverse() * estimated[j].pose;
            const auto rel_r = reference[i].pose.inverse() * reference[j].pose;
            const auto err = rel_r.inverse() * rel_e;
            const auto te = err.translation();
            const auto phi = err.rotation().log();
            trans_sum += std::sqrt(math::lie::detail::dot(te, te));
            rot_sum += std::sqrt(math::lie::detail::dot(phi, phi)) * rad2deg;
            ++seg.pairs;
        }
        if (seg.pairs == 0)
            continue;
        seg.translation_m = trans_sum / static_cast<T>(seg.pairs);
        seg.rotation_deg = rot_sum / static_cast<T>(seg.pairs);
        out.push_back(seg);
    }
    return out;
}

}  // namespace branes::sdk::eval

#endif  // BRANES_SDK_EVAL_TRAJECTORY_METRICS_HPP
