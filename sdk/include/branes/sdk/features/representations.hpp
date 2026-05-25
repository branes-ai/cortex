// SPDX-License-Identifier: MIT
//
// branes/sdk/features/representations.hpp — landmark parameterizations
// for the VIO/SLAM estimators and conversions between them.
//
//   XyzFeature                   — Euclidean world point (x, y, z).
//   AnchoredInverseDepthFeature  — anchor camera pose + bearing (α, β)
//                                  + inverse depth ρ; well-conditioned
//                                  for distant / low-parallax features.
//   MsckfNullspaceFeature        — the XYZ point an MSCKF tracks and then
//                                  marginalizes via the left-null-space
//                                  projection of its measurement Jacobian.
//
// Header-only, C++20, type-generic. Reuses the math-layer Lie groups.

#ifndef BRANES_SDK_FEATURES_REPRESENTATIONS_HPP
#define BRANES_SDK_FEATURES_REPRESENTATIONS_HPP

#include <branes/math/lie/se3.hpp>

namespace branes::sdk::features {

template <math::Scalar T>
using Vec3 = math::lie::detail::Vec<T, 3>;
template <math::Scalar T>
using SE3 = math::lie::SE3<T>;

/// A landmark as a Euclidean point in the world frame.
template <math::Scalar T>
struct XyzFeature {
    Vec3<T> position{};
    [[nodiscard]] Vec3<T> world_point() const {
        return position;
    }
};

/// A landmark anchored to a camera pose `anchor` (world←anchor-camera),
/// parameterized by its bearing (α, β) = (x/z, y/z) in the anchor frame
/// and its inverse depth ρ = 1/z. The point in the anchor frame is
/// (α, β, 1)/ρ; in the world it is `anchor · that`.
template <math::Scalar T>
struct AnchoredInverseDepthFeature {
    SE3<T> anchor{};
    T alpha{0};
    T beta{0};
    T rho{1};

    [[nodiscard]] Vec3<T> world_point() const {
        const T inv = T{1} / rho;
        return anchor * Vec3<T>{{alpha * inv, beta * inv, inv}};
    }

    /// Build from a world point and an anchor pose.
    [[nodiscard]] static AnchoredInverseDepthFeature from_world(const SE3<T>& anchor, const Vec3<T>& p_world) {
        const Vec3<T> p_a = anchor.inverse() * p_world;
        return {anchor, p_a[0] / p_a[2], p_a[1] / p_a[2], T{1} / p_a[2]};
    }
};

/// The MSCKF working representation: a triangulated world point that the
/// filter does not keep in its state vector but marginalizes per update
/// via the left-null-space projection (see msckf_nullspace.hpp).
template <math::Scalar T>
struct MsckfNullspaceFeature {
    Vec3<T> position{};
    [[nodiscard]] Vec3<T> world_point() const {
        return position;
    }
};

// ── Conversions ─────────────────────────────────────────────────────

template <math::Scalar T>
[[nodiscard]] XyzFeature<T> to_xyz(const AnchoredInverseDepthFeature<T>& f) {
    return {f.world_point()};
}
template <math::Scalar T>
[[nodiscard]] XyzFeature<T> to_xyz(const MsckfNullspaceFeature<T>& f) {
    return {f.position};
}

template <math::Scalar T>
[[nodiscard]] AnchoredInverseDepthFeature<T> to_inverse_depth(const SE3<T>& anchor, const XyzFeature<T>& f) {
    return AnchoredInverseDepthFeature<T>::from_world(anchor, f.position);
}

template <math::Scalar T>
[[nodiscard]] MsckfNullspaceFeature<T> to_msckf(const XyzFeature<T>& f) {
    return {f.position};
}

}  // namespace branes::sdk::features

#endif  // BRANES_SDK_FEATURES_REPRESENTATIONS_HPP
