// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s6_scene.hpp — the shared input of the S6 sub-step benches
// (issue #457, epic #444): a filter state with a genuinely propagated covariance,
// the feature tracks it observes, the unobservable directions at its
// linearization point, and the codecs for the S6 boundary types.
//
// The scene runs the synthetic world through S2_propagation and S3_augmentation
// from frame 1's true state, cloning at frames f0 … f0+m−1, and keeps the whole
// state: the IMU block and the m clones, with the covariance those stages
// produced. Features are the landmarks seen in all m frames. With `true_poses`
// the mean is replaced by the ground truth (nav state and clones) and the
// observations are exact projections; otherwise the mean is the estimate and the
// observations are the world's pixels normalized through S0_sensor_model.
//
// Each S6 sub-step bench takes its own boundary as input — S6a a track and a
// feature position, S6b a stacked measurement system, S6c a projected system,
// S6d and S6e a state and a projected system — so the scene runs the upstream
// sub-steps (S5, S6a, S6b) in double to produce each one.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S6_SCENE_HPP
#define BRANES_TOOLS_BENCH_S6_SCENE_HPP

#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/stages/s0_sensor_model.hpp>
#include <branes/sdk/msckf/stages/s2_propagation.hpp>
#include <branes/sdk/msckf/stages/s3_augmentation.hpp>
#include <branes/sdk/msckf/stages/s6_msckf_update.hpp>
#include <branes/tools/bench/bench.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace branes::tools::bench::s6 {

template <class T>
using State = sdk::msckf::State<T>;
template <class T>
using Vec3 = math::lie::detail::Vec<T, 3>;
template <class T>
using Track = sdk::msckf::FeatureTrack<T>;
template <class T>
using System = sdk::msckf::MeasurementSystem<T>;
template <class T>
using Projected = sdk::msckf::ProjectedMeasurement<T>;
template <class T>
using Options = sdk::msckf::CameraUpdaterOptions<T>;
using Extrinsics = sdk::msckf::CameraExtrinsics<double>;
template <class T>
using Mat = sdk::msckf::DynMat<T>;

// ── Codecs ──────────────────────────────────────────────────────────────────

template <class T>
[[nodiscard]] json encode_track(const Track<T>& t) {
    json obs = json::array();
    for (const auto& o : t.observations)
        obs.push_back(json::array({o.clone_index, o.camera_index, pack(o.xy[0]), pack(o.xy[1])}));
    return obs;
}
/// The benches supply one camera; any other index is an input error.
template <class T>
[[nodiscard]] Track<T> decode_track(const json& j, std::size_t clones) {
    Track<T> t;
    for (const auto& o : j) {
        const auto ci = o.at(0).get<std::size_t>();
        if (ci >= clones)
            throw std::invalid_argument("s6 bench: observation clone index out of range");
        if (o.at(1).get<std::size_t>() != 0)
            throw std::invalid_argument("s6 bench: observation camera index must be 0 (one camera)");
        t.observations.push_back({ci, 0, {{unpack_scalar<T>(o.at(2)), unpack_scalar<T>(o.at(3))}}});
    }
    return t;
}

template <class T>
[[nodiscard]] json encode_options(const Options<T>& o) {
    return json{{"normalized_sigma", pack(o.normalized_sigma)},
                {"chi2_per_dof", pack(o.chi2_per_dof)},
                {"enable_gating", o.enable_gating},
                {"max_triangulation_iters", o.max_triangulation_iters},
                {"min_parallax_deg", pack(o.min_parallax_deg)},
                {"calib_rot_sigma", pack(o.calib_rot_sigma)}};
}
template <class T>
[[nodiscard]] Options<T> decode_options(const json& j) {
    Options<T> o;
    o.normalized_sigma = unpack_scalar<T>(j.at("normalized_sigma"));
    o.chi2_per_dof = unpack_scalar<T>(j.at("chi2_per_dof"));
    o.enable_gating = j.at("enable_gating").get<bool>();
    o.max_triangulation_iters = j.at("max_triangulation_iters").get<std::size_t>();
    o.min_parallax_deg = unpack_scalar<T>(j.at("min_parallax_deg"));
    o.calib_rot_sigma = unpack_scalar<T>(j.at("calib_rot_sigma"));
    return o;
}

template <class T>
[[nodiscard]] json encode_system(const System<T>& s) {
    return json{{"rows", s.rows},
                {"cols", s.cols},
                {"Hf", pack_vec<T>(s.Hf)},
                {"Hx", pack_vec<T>(s.Hx)},
                {"r", pack_vec<T>(s.r)}};
}
template <class T>
[[nodiscard]] System<T> decode_system(const json& j) {
    System<T> s;
    s.rows = j.at("rows").get<std::size_t>();
    s.cols = j.at("cols").get<std::size_t>();
    s.Hf = unpack_vec<T>(j.at("Hf"));
    s.Hx = unpack_vec<T>(j.at("Hx"));
    s.r = unpack_vec<T>(j.at("r"));
    if (s.Hf.size() != s.rows * 3 || s.Hx.size() != s.rows * s.cols || s.r.size() != s.rows)
        throw std::invalid_argument("s6 bench: measurement system shapes disagree with rows/cols");
    return s;
}

template <class T>
[[nodiscard]] json encode_projected(const Projected<T>& p) {
    return json{{"H", pack(p.H)}, {"r", pack_vec<T>(p.r)}};
}
template <class T>
[[nodiscard]] Projected<T> decode_projected(const json& j) {
    Projected<T> p;
    p.H = unpack_mat<T>(j.at("H"));
    p.r = unpack_vec<T>(j.at("r"));
    if (p.r.size() != p.H.rows)
        throw std::invalid_argument("s6 bench: projected residual length disagrees with H");
    return p;
}

// ── The unobservable directions ─────────────────────────────────────────────

/// The four unobservable directions of a VIO state at its linearization point
/// (global position, yaw about gravity), as columns of an n×4 matrix in the
/// State's error-state layout (right perturbation R ← R·Exp(δθ)):
///   translation t:  δp = t, δp_c = t (clones), everything else 0;
///   yaw ψ about ẑ:  δθ = Rᵀẑ, δp = ẑ×p, δv = ẑ×v; per clone δθ_c = R_cᵀẑ,
///                   δp_c = ẑ×p_c.
/// A camera update evaluated at one consistent point satisfies H·N = 0.
template <class T>
[[nodiscard]] Mat<T> unobservable_basis(const State<T>& s) {
    using St = State<T>;
    Mat<T> n(s.dim(), 4);
    const Vec3<T> z{{T{0}, T{0}, T{1}}};
    auto zx = [](const Vec3<T>& a) { return Vec3<T>{{-a[1], a[0], T{0}}}; };  // ẑ × a
    auto put = [&](std::size_t row, std::size_t col, const Vec3<T>& v) {
        for (std::size_t i = 0; i < 3; ++i)
            n(row + i, col) = v[i];
    };
    for (std::size_t i = 0; i < 3; ++i)
        n(St::kPos + i, i) = T{1};
    put(St::kTheta, 3, s.R.inverse() * z);
    put(St::kPos, 3, zx(s.p));
    put(St::kVel, 3, zx(s.v));
    for (std::size_t c = 0; c < s.clones.size(); ++c) {
        const std::size_t off = s.clone_offset(c);
        for (std::size_t i = 0; i < 3; ++i)
            n(off + 3 + i, i) = T{1};
        put(off, 3, s.clones[c].R.inverse() * z);
        put(off + 3, 3, zx(s.clones[c].p));
    }
    return n;
}

// ── The scene ───────────────────────────────────────────────────────────────

struct Scene {
    State<double> state{0.1};
    Extrinsics extrinsics{};
    std::vector<Track<double>> tracks;
    std::vector<Vec3<double>> landmarks;  ///< the true position of each track's feature
};

/// Build the scene (see the header comment). `m` clones from frame `f0`;
/// at most `max_features` tracks. `init`, if set, runs on the fresh state before
/// any propagation or clone (S10_online_calibration must run there).
[[nodiscard]] inline Scene build_scene(bool true_poses,
                                       std::size_t m = 6,
                                       std::size_t max_features = 16,
                                       const std::function<void(State<double>&)>& init = {}) {
    namespace st = sdk::msckf::stages;
    const auto world = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
    const std::size_t f0 = 4;
    if (m < 2 || f0 + m > world.frames.size())
        throw std::invalid_argument("s6 scene: clone window out of the world's frames");

    Scene sc;
    sc.extrinsics = {world.R_imu_cam, world.p_imu_cam};
    auto& s = sc.state;
    const sdk::msckf::Propagator<double> prop({}, {{0.0, 0.0, -9.81}});
    // Start at frame 1's true state at its time: frame 0 precedes the world's
    // velocity step at the end of its warm-up (#475).
    s.R = world.gt[1].R;
    s.p = world.gt[1].p;
    s.v = world.gt[1].v;
    s.timestamp = world.gt[1].t;
    if (init)
        init(s);
    std::size_t k = 0;
    while (k < world.imu.size() && world.imu[k].timestamp_s <= s.timestamp)
        ++k;
    for (std::size_t fr = 1; fr < f0 + m; ++fr) {
        for (; k < world.imu.size() && world.imu[k].timestamp_s <= world.frames[fr].t; ++k) {
            const auto& im = world.imu[k];
            st::s2_propagation::apply(
                s,
                prop,
                {{im.angular_velocity[0], im.angular_velocity[1], im.angular_velocity[2]}},
                {{im.linear_acceleration[0], im.linear_acceleration[1], im.linear_acceleration[2]}},
                im.timestamp_s - s.timestamp);
            s.timestamp = im.timestamp_s;
        }
        if (fr >= f0)
            st::s3_augmentation::apply(s, world.frames[fr].t);
    }
    if (true_poses) {
        const auto& g = world.gt[f0 + m - 1];
        s.R = g.R;
        s.p = g.p;
        s.v = g.v;
        for (std::size_t c = 0; c < m; ++c) {
            s.clones[c].R = world.gt[f0 + c].R;
            s.clones[c].p = world.gt[f0 + c].p;
        }
    }

    // Features seen in every frame of the window, in frame f0's order.
    for (const auto& o : world.frames[f0].obs) {
        if (sc.tracks.size() >= max_features)
            break;
        std::vector<double> xs, ys;
        for (std::size_t c = 0; c < m; ++c) {
            bool seen = false;
            for (const auto& q : world.frames[f0 + c].obs)
                if (q.feature_id == o.feature_id) {
                    const auto nrm = st::s0_sensor_model::apply(world.camera, q.u, q.v);
                    xs.push_back(nrm.xy[0]);
                    ys.push_back(nrm.xy[1]);
                    seen = true;
                    break;
                }
            if (!seen)
                break;
        }
        if (xs.size() != m)
            continue;
        const auto& lm = world.landmarks[o.feature_id];
        Track<double> t;
        for (std::size_t c = 0; c < m; ++c) {
            if (true_poses) {
                const auto& g = world.gt[f0 + c];
                const auto y = g.R.inverse() * (lm - g.p);
                const auto pc = world.R_imu_cam.inverse() * (y - world.p_imu_cam);
                t.observations.push_back({c, 0, {{pc[0] / pc[2], pc[1] / pc[2]}}});
            } else {
                t.observations.push_back({c, 0, {{xs[c], ys[c]}}});
            }
        }
        sc.tracks.push_back(t);
        sc.landmarks.push_back(lm);
    }
    if (sc.tracks.empty())
        throw std::logic_error("s6 scene: no feature is seen in every frame of the window");
    return sc;
}

/// The shipped updater for a scene.
[[nodiscard]] inline sdk::msckf::CameraUpdater<double> updater(const Scene& sc, const Options<double>& opts = {}) {
    return sdk::msckf::CameraUpdater<double>(std::vector<Extrinsics>{sc.extrinsics}, opts);
}

/// The boundaries of one track through the shipped S5 → S6b in double.
struct Boundaries {
    Vec3<double> p_f{};
    System<double> system;
    Projected<double> projected;
};
[[nodiscard]] inline Boundaries boundaries(const Scene& sc, std::size_t track, const Options<double>& opts = {}) {
    namespace st = sdk::msckf::stages;
    const auto upd = updater(sc, opts);
    Boundaries b;
    const auto tri = st::s5_triangulation::apply(sc.state, upd, sc.tracks.at(track));
    if (!tri.ok)
        throw std::logic_error("s6 scene: the track does not triangulate");
    b.p_f = tri.p_f;
    const auto jac = st::s6a_jacobians::apply(sc.state, upd, sc.tracks[track], b.p_f);
    if (!jac.ok)
        throw std::logic_error("s6 scene: the feature is behind a camera");
    b.system = jac.system;
    auto proj = st::s6b_nullspace_projection::apply(b.system);
    if (!proj.ok)
        throw std::logic_error("s6 scene: no rows survive the projection");
    b.projected = std::move(proj.projected);
    return b;
}

/// A portable standard-normal draw from an integer index (a hash and
/// Box–Muller, not <random>, so sweeps reproduce on every standard library).
[[nodiscard]] inline double pseudo_normal(std::uint32_t i) {
    auto u = [](std::uint32_t k) {
        k ^= k >> 16;
        k *= 0x7feb352du;
        k ^= k >> 15;
        k *= 0x846ca68bu;
        k ^= k >> 16;
        return (static_cast<double>(k) + 0.5) / 4294967296.0;
    };
    const double u1 = u(2 * i + 1), u2 = u(2 * i + 2);
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
}

/// Relative Frobenius distance ‖a − b‖ / max(‖b‖, 1), in double.
template <class T>
[[nodiscard]] double rel_diff(const Mat<T>& a, const Mat<T>& b) {
    if (a.rows != b.rows || a.cols != b.cols)
        return inv::detail::kInf;
    double d = 0.0, nb = 0.0;
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t j = 0; j < a.cols; ++j) {
            const double x = static_cast<double>(a(i, j)), y = static_cast<double>(b(i, j));
            if (!std::isfinite(x) || !std::isfinite(y))
                return inv::detail::kInf;
            d += (x - y) * (x - y);
            nb += y * y;
        }
    return std::sqrt(d) / std::max(std::sqrt(nb), 1.0);
}

/// The innovation covariance S = H·P·Hᵀ + var·I.
template <class T>
[[nodiscard]] Mat<T> innovation(const Mat<T>& P, const Mat<T>& H, T var) {
    Mat<T> s = sdk::msckf::mul(sdk::msckf::mul(H, P), sdk::msckf::transpose(H));
    for (std::size_t i = 0; i < s.rows; ++i)
        s(i, i) += var;
    return s;
}

/// A row-major span as an r×c matrix.
template <class T>
[[nodiscard]] Mat<T> as_mat(const std::vector<T>& v, std::size_t r, std::size_t c) {
    Mat<T> m(r, c);
    for (std::size_t i = 0; i < r; ++i)
        for (std::size_t j = 0; j < c; ++j)
            m(i, j) = v[i * c + j];
    return m;
}
template <class T>
[[nodiscard]] Mat<T> as_col(const std::vector<T>& v) {
    return as_mat(v, v.size(), 1);
}

}  // namespace branes::tools::bench::s6

#endif  // BRANES_TOOLS_BENCH_S6_SCENE_HPP
