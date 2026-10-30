// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/rebind.hpp — move the MSCKF state and the stage operators
// from one arithmetic type to another (issue #450, epic #444 §C).
//
// A backend whose stages compute in different types (stage_types.hpp) carries
// its state in one storage type and, around a stage that computes in another,
// rebinds the state into that type, runs the stage, and rebinds the result back.
// The operators the stages consume (the propagator, the camera updater, the
// camera model) are rebound once, when the backend is configured.
//
// Every value is converted by rounding to the nearest value of the target type.
// Rotations are renormalized in the target type: a unit quaternion of one type
// is unit only to that type's precision, so the target type's own SO3
// construction normalizes it. Timestamps stay double in every type.
//
// Header-only, C++20.

#ifndef BRANES_SDK_MSCKF_REBIND_HPP
#define BRANES_SDK_MSCKF_REBIND_HPP

#include <branes/math/arithmetic.hpp>
#include <branes/math/cameras/pinhole_radtan.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/covariance.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/sqrt_covariance.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <cstddef>
#include <type_traits>
#include <vector>

namespace branes::sdk::msckf {

/// `x` rounded to U. Between two non-IEEE types (posit to posit) the value goes
/// through long double, which holds every value of the types the benches use.
template <math::Scalar U, math::Scalar T>
[[nodiscard]] U scalar_cast(const T& x) {
    if constexpr (std::is_same_v<U, T>)
        return x;
    else if constexpr (std::is_arithmetic_v<T> || std::is_arithmetic_v<U>)
        return static_cast<U>(x);
    else
        return static_cast<U>(static_cast<long double>(x));
}

/// A covariance representation over U in place of T: FullCovariance<T> →
/// FullCovariance<U>, SqrtCovariance<T> → SqrtCovariance<U>.
template <class Cov, math::Scalar U>
struct rebind_cov;
template <template <class> class C, math::Scalar T, math::Scalar U>
struct rebind_cov<C<T>, U> {
    using type = C<U>;
};
template <class Cov, math::Scalar U>
using rebind_cov_t = typename rebind_cov<Cov, U>::type;

template <math::Scalar U, math::Scalar T, std::size_t N>
[[nodiscard]] math::lie::detail::Vec<U, N> rebind(const math::lie::detail::Vec<T, N>& v) {
    math::lie::detail::Vec<U, N> out;
    for (std::size_t i = 0; i < N; ++i)
        out[i] = scalar_cast<U>(v[i]);
    return out;
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] math::lie::SO3<U> rebind(const math::lie::SO3<T>& r) {
    if constexpr (std::is_same_v<U, T>)
        return r;
    else
        return math::lie::SO3<U>(rebind<U>(r.quaternion()));  // normalizes in U
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] DynMat<U> rebind(const DynMat<T>& m) {
    DynMat<U> out(m.rows, m.cols);
    for (std::size_t i = 0; i < m.d.size(); ++i)
        out.d[i] = scalar_cast<U>(m.d[i]);
    return out;
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] FullCovariance<U> rebind(const FullCovariance<T>& c) {
    FullCovariance<U> out;
    out.P = rebind<U>(c.P);
    return out;
}

/// The factor itself is rebound (not P = S·Sᵀ), so a square-root filter stays
/// one in every type.
template <math::Scalar U, math::Scalar T>
[[nodiscard]] SqrtCovariance<U> rebind(const SqrtCovariance<T>& c) {
    SqrtCovariance<U> out;
    out.S = rebind<U>(c.S);
    return out;
}

template <math::Scalar U, math::Scalar T, class Cov>
[[nodiscard]] State<U, rebind_cov_t<Cov, U>> rebind(const State<T, Cov>& s) {
    State<U, rebind_cov_t<Cov, U>> out;
    out.R = rebind<U>(s.R);
    out.p = rebind<U>(s.p);
    out.v = rebind<U>(s.v);
    out.bg = rebind<U>(s.bg);
    out.ba = rebind<U>(s.ba);
    out.timestamp = s.timestamp;
    out.calib.reserve(s.calib.size());
    for (const auto& c : s.calib)
        out.calib.push_back({rebind<U>(c.R_imu_cam), rebind<U>(c.p_imu_cam)});
    out.clones.reserve(s.clones.size());
    for (const auto& c : s.clones)
        out.clones.push_back({rebind<U>(c.R), rebind<U>(c.p), c.timestamp});
    out.cov = rebind<U>(s.cov);
    return out;
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] CameraExtrinsics<U> rebind(const CameraExtrinsics<T>& e) {
    return {rebind<U>(e.R_imu_cam), rebind<U>(e.p_imu_cam)};
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] CameraUpdaterOptions<U> rebind(const CameraUpdaterOptions<T>& o) {
    CameraUpdaterOptions<U> out;
    out.normalized_sigma = scalar_cast<U>(o.normalized_sigma);
    out.chi2_per_dof = scalar_cast<U>(o.chi2_per_dof);
    out.enable_gating = o.enable_gating;
    out.max_triangulation_iters = o.max_triangulation_iters;
    out.min_parallax_deg = scalar_cast<U>(o.min_parallax_deg);
    out.calib_rot_sigma = scalar_cast<U>(o.calib_rot_sigma);
    return out;
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] CameraUpdater<U> rebind(const CameraUpdater<T>& u) {
    std::vector<CameraExtrinsics<U>> cams;
    cams.reserve(u.cameras().size());
    for (const auto& e : u.cameras())
        cams.push_back(rebind<U>(e));
    return CameraUpdater<U>(std::move(cams), rebind<U>(u.options()));
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] Propagator<U> rebind(const Propagator<T>& p) {
    const auto& n = p.noise();
    const ImuNoise<U> noise{
        scalar_cast<U>(n.gyro), scalar_cast<U>(n.accel), scalar_cast<U>(n.gyro_bias), scalar_cast<U>(n.accel_bias)};
    return Propagator<U>(noise, rebind<U>(p.gravity()));
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] math::cameras::PinholeRadtanCamera<U> rebind(const math::cameras::PinholeRadtanCamera<T>& c) {
    const auto& d = c.radtan();
    return math::cameras::PinholeRadtanCamera<U>(scalar_cast<U>(c.fx()),
                                                 scalar_cast<U>(c.fy()),
                                                 scalar_cast<U>(c.cx()),
                                                 scalar_cast<U>(c.cy()),
                                                 scalar_cast<U>(d.k1),
                                                 scalar_cast<U>(d.k2),
                                                 scalar_cast<U>(d.p1),
                                                 scalar_cast<U>(d.p2),
                                                 scalar_cast<U>(d.k3));
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] FeatureTrack<U> rebind(const FeatureTrack<T>& t) {
    FeatureTrack<U> out;
    out.observations.reserve(t.observations.size());
    for (const auto& o : t.observations)
        out.observations.push_back({o.clone_index, o.camera_index, rebind<U>(o.xy)});
    return out;
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] ProjectedMeasurement<U> rebind(const ProjectedMeasurement<T>& m) {
    ProjectedMeasurement<U> out;
    out.H = rebind<U>(m.H);
    out.r.reserve(m.r.size());
    for (const auto& x : m.r)
        out.r.push_back(scalar_cast<U>(x));
    return out;
}

template <math::Scalar U, math::Scalar T>
[[nodiscard]] NisSample<U> rebind(const NisSample<T>& n) {
    return {scalar_cast<U>(n.value), n.dof, n.valid, scalar_cast<U>(n.innov_sum)};
}

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_REBIND_HPP
