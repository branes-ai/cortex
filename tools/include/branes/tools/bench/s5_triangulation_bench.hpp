// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s5_triangulation_bench.hpp — the S5_triangulation stage
// bench (issue #456, epic #444).
//
// Stage under test: stages::s5_triangulation::apply(state, updater, track) — the
// linear ray-perpendicular solve, the optional parallax gate, Gauss-Newton
// reprojection refinement. Contract (#445 §B, S5):
//   • the track resolves (except where a parallax gate legitimately rejects it);
//   • the feature has positive depth in every observing camera;
//   • the reprojection residual is bounded (a few σ of the measurement noise);
//   • reported (the #265 scope): the maximum inter-view parallax and the
//     condition number of the normal matrix Σ(I − d̂d̂ᵀ).
//
// Fixtures:
//   known_answer   four clones on a wide baseline, a noiseless feature: the
//                  expected position is the feature itself
//   ground_truth   a synthetic-world landmark seen from the TRUE poses of the
//                  frames that observe it, exact observations; truth = the landmark
//   captured       the same landmark from the S2/S3 stage sequence's estimated
//                  clone poses, observations through S0, recorded
//
// Variants: "shipped" (linear + 5 Gauss-Newton steps, no gate), "linear_only"
// (no refinement), "parallax_gate_2deg" (reject below 2° of parallax), and the
// alternative methods of stages::s5_triangulation::Method — "midpoint_two_view"
// (best-conditioned pair, common-perpendicular midpoint), "dlt_linear" (inhomogeneous
// DLT), "inverse_depth_gn" (anchored inverse depth, Gauss-Newton).
//
// Sweep: parallax × pixel noise → depth error per method, normal-matrix κ, and
// the share the 2° gate rejects (the S5 probe's finding: 0.1–5° features
// admitted at full weight).
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S5_TRIANGULATION_BENCH_HPP
#define BRANES_TOOLS_BENCH_S5_TRIANGULATION_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/stages/s0_sensor_model.hpp>
#include <branes/sdk/msckf/stages/s2_propagation.hpp>
#include <branes/sdk/msckf/stages/s3_augmentation.hpp>
#include <branes/sdk/msckf/stages/s5_triangulation.hpp>
#include <branes/tools/bench/bench.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S5TriangulationBench {
    static constexpr std::string_view kStage = "S5_triangulation";
    static constexpr inv::Stage kInvStage = inv::Stage::S5_triangulation;

    template <class T>
    using State = sdk::msckf::State<T>;
    template <class T>
    using Vec3 = math::lie::detail::Vec<T, 3>;

    template <class T>
    struct Input {
        State<T> state{T{1}};
        sdk::msckf::CameraExtrinsics<T> extrinsics{};
        sdk::msckf::CameraUpdaterOptions<T> options{};
        sdk::msckf::FeatureTrack<T> track{};
    };
    template <class T>
    struct Output {
        int ok = 0;
        Vec3<T> p_f{};
        double gate_deg = 0.0;  ///< the parallax gate the run used (0 = none)
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "linear ray-perpendicular solve + 5 Gauss-Newton steps, no parallax gate"},
                {"linear_only", "the linear solve without refinement"},
                {"parallax_gate_2deg", "reject tracks with less than 2 deg of parallax"},
                {"midpoint_two_view", "midpoint of the common perpendicular of the best-conditioned pair of rays"},
                {"dlt_linear", "inhomogeneous DLT: algebraic least squares over every view"},
                {"inverse_depth_gn", "anchored inverse depth seeded by the DLT, Gauss-Newton on reprojection"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        json obs = json::array();
        for (const auto& o : in.track.observations)
            obs.push_back(json::array({o.clone_index, o.camera_index, pack(o.xy[0]), pack(o.xy[1])}));
        const auto& op = in.options;
        return json{{"state", pack(in.state)},
                    {"extrinsics",
                     {{"R_imu_cam", pack(in.extrinsics.R_imu_cam)}, {"p_imu_cam", pack(in.extrinsics.p_imu_cam)}}},
                    {"options",
                     {{"normalized_sigma", pack(op.normalized_sigma)},
                      {"min_parallax_deg", pack(op.min_parallax_deg)},
                      {"max_triangulation_iters", op.max_triangulation_iters}}},
                    {"observations", obs}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        Input<T> in;
        in.state = unpack_state<T>(j.at("state"));
        in.extrinsics.R_imu_cam = unpack_so3<T>(j.at("extrinsics").at("R_imu_cam"));
        in.extrinsics.p_imu_cam = unpack_fixed<T, 3>(j.at("extrinsics").at("p_imu_cam"));
        const auto& op = j.at("options");
        in.options.normalized_sigma = unpack_scalar<T>(op.at("normalized_sigma"));
        in.options.min_parallax_deg = unpack_scalar<T>(op.at("min_parallax_deg"));
        in.options.max_triangulation_iters = op.at("max_triangulation_iters").get<std::size_t>();
        for (const auto& o : j.at("observations")) {
            const auto ci = o.at(0).get<std::size_t>();
            if (ci >= in.state.clones.size())
                throw std::invalid_argument("s5 bench: observation clone index out of range");
            // The bench supplies one camera; another index would fail S5's track
            // validation and could pass as a gate rejection.
            if (o.at(1).get<std::size_t>() != 0)
                throw std::invalid_argument("s5 bench: observation camera index must be 0 (one camera)");
            in.track.observations.push_back(
                {ci, o.at(1).get<std::size_t>(), {{unpack_scalar<T>(o.at(2)), unpack_scalar<T>(o.at(3))}}});
        }
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"ok", out.ok}, {"p_f", pack(out.p_f)}, {"gate_deg", out.gate_deg}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        return Output<T>{j.at("ok").get<int>(), unpack_fixed<T, 3>(j.at("p_f")), j.value("gate_deg", 0.0)};
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        auto opts = in.options;
        if (variant == "linear_only")
            opts.max_triangulation_iters = 0;
        else if (variant == "parallax_gate_2deg")
            opts.min_parallax_deg = T{2};
        const sdk::msckf::CameraUpdater<T> upd(std::vector<sdk::msckf::CameraExtrinsics<T>>{in.extrinsics}, opts);
        const auto r = sdk::msckf::stages::s5_triangulation::apply(in.state, upd, in.track, method(variant));
        return Output<T>{r.ok ? 1 : 0, r.ok ? r.p_f : Vec3<T>{}, static_cast<double>(opts.min_parallax_deg)};
    }

    /// The triangulation method a variant runs.
    [[nodiscard]] static sdk::msckf::stages::s5_triangulation::Method method(std::string_view variant) {
        using M = sdk::msckf::stages::s5_triangulation::Method;
        if (variant == "midpoint_two_view")
            return M::Midpoint;
        if (variant == "dlt_linear")
            return M::Dlt;
        if (variant == "inverse_depth_gn")
            return M::InverseDepth;
        return M::Shipped;
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        return {static_cast<double>(out.ok),
                static_cast<double>(out.p_f[0]),
                static_cast<double>(out.p_f[1]),
                static_cast<double>(out.p_f[2]),
                out.gate_deg};
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        const auto geo = geometry(in, out.p_f);
        // Parallax and conditioning: the context every other number is read against (#265).
        r.push_back(inv::check_scalar(
            geo.max_parallax_deg, 0.0, inv::Bound::Report, kInvStage, "parallax.max_inter_view", "deg"));
        r.push_back(inv::report_condition_number(geo.normal, kInvStage, "normal_matrix.condition_number"));

        // A gate exists to reject: its "no" is a result, not a failure — but only
        // when the measured parallax is actually below the gate. A rejection above
        // it is a triangulation failure and stays one.
        const bool gated = out.gate_deg > 0.0 && geo.max_parallax_deg < out.gate_deg;
        auto resolved = inv::check_scalar(out.ok, 1.0, inv::Bound::Lower, kInvStage, "triangulation.resolved", "flag");
        if (gated) {
            resolved.bound = inv::Bound::Report;
            resolved.pass = true;
        }
        r.push_back(resolved);
        if (!out.ok)
            return r;

        r.push_back(inv::check_at_least<double>(geo.depths, 1e-6, kInvStage, "depth.positive_in_every_camera", "m"));
        // Reprojection: the point explains its observations within 5σ of the
        // filter's assumed measurement noise (floored at the type's precision).
        const double sigma = static_cast<double>(in.options.normalized_sigma);
        const double bound = 5.0 * std::max(sigma, tolerance_vs_double<T>(8, 1.0, 64.0));
        r.push_back(
            inv::check_bounded<double>(geo.reproj, bound, kInvStage, "reprojection.max_residual", "normalized"));

        if (f.kind == FixtureKind::GroundTruth) {
            const auto p = unpack_fixed<double, 3>(f.truth.at("p_f"));
            double worst = 0.0, scale = 1.0;
            for (std::size_t i = 0; i < 3; ++i) {
                const double d = std::abs(static_cast<double>(out.p_f[i]) - p[i]);
                worst = std::isfinite(d) ? std::max(worst, d) : inv::detail::kInf;
                scale = std::max(scale, std::abs(p[i]));
            }
            // Exact observations: the error is arithmetic, amplified by the
            // conditioning of the normal matrix.
            // First-order bound for a backward-stable solve: κ(A)·ε_T·|p|.
            const auto kappa = inv::report_condition_number(geo.normal, kInvStage).value;
            r.push_back(inv::check_scalar(worst,
                                          tolerance_vs_double<T>(3, scale * std::max(1.0, kappa), 4.0),
                                          inv::Bound::Upper,
                                          kInvStage,
                                          "truth.feature_position",
                                          "m"));
        }
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_wide_baseline", known_answer()},
                {"ground_truth_synthetic_landmark", ground_truth()},
                {"captured_estimated_poses", captured()}};
    }

    /// Four clones on a 3 m baseline looking down +z, a noiseless feature at
    /// 3 m: the triangulated position is the feature.
    [[nodiscard]] static Fixture known_answer() {
        Input<double> in;
        in.state = State<double>(0.1);
        const Vec3<double> F{{0.2, -0.1, 3.0}};
        for (int c = 0; c < 4; ++c) {
            const Vec3<double> p{{-1.5 + 1.0 * c, 0.0, 0.0}};
            in.state.clones.push_back({{}, p, double(c)});
            in.track.observations.push_back(
                {static_cast<std::size_t>(c), 0, {{(F[0] - p[0]) / (F[2] - p[2]), (F[1] - p[1]) / (F[2] - p[2])}}});
        }
        in.state.cov.P = sdk::msckf::DynMat<double>::identity(in.state.dim());
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.variant = "shipped";  // the gate variant records its gate in the output
        f.source = "s5_triangulation_bench: 4 clones on a 3 m baseline, noiseless feature at 3 m";
        f.description = "expected: the feature position";
        f.input = encode_input(in);
        f.expected = encode_output(Output<double>{1, F, 0.0});
        return f;
    }

    /// A landmark the synthetic world observes in frames `f0`…`f0+m−1`, seen
    /// from the TRUE poses with exact (camera-frame) observations.
    [[nodiscard]] static Fixture ground_truth() {
        const auto lm = landmark_window(/*true_poses=*/true);
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config), landmark " + std::to_string(lm.id) + ", true poses, exact obs";
        f.seed = sdk::eval::SyntheticConfig<double>{}.seed;
        f.description = "truth: the landmark position";
        f.input = encode_input(lm.input);
        f.truth = json{{"p_f", pack(lm.truth)}};
        return f;
    }

    /// The same landmark from the S2/S3 stage sequence's ESTIMATED clone poses,
    /// with the world's pixels normalized through S0 — recorded.
    [[nodiscard]] static Fixture captured() {
        const auto lm = landmark_window(/*true_poses=*/false);
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config), landmark " + std::to_string(lm.id) +
                           ", S2/S3 estimated poses, S0-normalized pixels",
                       encode_input(lm.input),
                       encode_output(run<double>(lm.input, kShipped)));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("parallax_deg", {0.25, 1.0, 2.0, 5.0, 15.0}).axis("px_noise", {0.0, 0.5, 1.0, 2.0});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"depth_err_m",
                "kappa",
                "gate_2deg_rejected",
                "depth_err_midpoint_m",
                "depth_err_dlt_m",
                "depth_err_inverse_depth_m"};
    }
    /// Four clones whose outermost rays subtend `parallax_deg` at a feature 5 m
    /// away, pixel noise `px_noise` (EuRoC fx); the mean depth error over 16
    /// deterministic noise draws (shipped, then each alternative method; ∞ when
    /// a method fails any draw), κ, and the share the 2° gate rejects.
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double par = p.at("parallax_deg"), px = p.at("px_noise");
        if (!std::isfinite(par) || par <= 0.0 || par > 90.0)
            throw std::invalid_argument("s5 bench sweep: parallax_deg must be in (0, 90]");
        if (!std::isfinite(px) || px < 0.0 || px > 50.0)
            throw std::invalid_argument("s5 bench sweep: px_noise must be in [0, 50]");
        const double depth = 5.0, half = depth * std::tan(par * std::numbers::pi / 360.0);
        const double sigma = px / 458.654;
        const Vec3<double> F{{0.0, 0.0, depth}};
        double err = 0.0, kappa = 0.0, rejected = 0.0;
        const char* alts[] = {"midpoint_two_view", "dlt_linear", "inverse_depth_gn"};
        double alt_err[3] = {0.0, 0.0, 0.0};
        const int draws = 16;
        for (int d = 0; d < draws; ++d) {
            Input<double> in;
            in.state = State<double>(0.1);
            for (int c = 0; c < 4; ++c) {
                const Vec3<double> pc{{-half + 2.0 * half * c / 3.0, 0.0, 0.0}};
                in.state.clones.push_back({{}, pc, double(c)});
                const double nx = sigma * pseudo_normal(d * 8 + c * 2), ny = sigma * pseudo_normal(d * 8 + c * 2 + 1);
                in.track.observations.push_back(
                    {static_cast<std::size_t>(c), 0, {{(F[0] - pc[0]) / F[2] + nx, (F[1] - pc[1]) / F[2] + ny}}});
            }
            in.state.cov.P = sdk::msckf::DynMat<double>::identity(in.state.dim());
            const auto inT = decode_input<T>(encode_input(in));
            const auto out = run<T>(inT, kShipped);
            const auto gated = run<T>(inT, "parallax_gate_2deg");
            err += out.ok ? std::abs(static_cast<double>(out.p_f[2]) - depth) : inv::detail::kInf;
            rejected += gated.ok ? 0.0 : 1.0;
            for (std::size_t a = 0; a < 3; ++a) {
                const auto o = run<T>(inT, alts[a]);
                alt_err[a] += o.ok ? std::abs(static_cast<double>(o.p_f[2]) - depth) : inv::detail::kInf;
            }
            if (d == 0)
                kappa = inv::report_condition_number(geometry(inT, out.p_f).normal, kInvStage).value;
        }
        return {err / draws, kappa, rejected / draws, alt_err[0] / draws, alt_err[1] / draws, alt_err[2] / draws};
    }

private:
    template <class T>
    struct Geometry {
        std::vector<double> depths, reproj;
        double max_parallax_deg = 0.0;
        sdk::msckf::DynMat<T> normal{3, 3};  ///< Σ (I − d̂ d̂ᵀ) over the world-frame rays
    };

    /// Per-observation depth and reprojection residual at p_f, the ray-set
    /// parallax, and the normal matrix — the same camera model as the updater.
    template <class T>
    [[nodiscard]] static Geometry<T> geometry(const Input<T>& in, const Vec3<T>& p_f) {
        Geometry<T> g;
        const auto& ex = in.extrinsics;
        std::vector<Vec3<double>> rays;
        for (const auto& o : in.track.observations) {
            const auto& cl = in.state.clones[o.clone_index];
            const Vec3<T> y = cl.R.inverse() * (p_f - cl.p);
            const Vec3<T> pc = ex.R_imu_cam.inverse() * (y - ex.p_imu_cam);
            g.depths.push_back(static_cast<double>(pc[2]));
            if (pc[2] > T{0}) {
                g.reproj.push_back(std::abs(static_cast<double>(pc[0] / pc[2] - o.xy[0])));
                g.reproj.push_back(std::abs(static_cast<double>(pc[1] / pc[2] - o.xy[1])));
            }
            const Vec3<T> d = (cl.R * ex.R_imu_cam) * Vec3<T>{{o.xy[0], o.xy[1], T{1}}};
            const double n = std::sqrt(static_cast<double>(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
            rays.push_back(
                {{static_cast<double>(d[0]) / n, static_cast<double>(d[1]) / n, static_cast<double>(d[2]) / n}});
            for (std::size_t a = 0; a < 3; ++a)
                for (std::size_t b = 0; b < 3; ++b)
                    g.normal(a, b) += T((a == b ? 1.0 : 0.0) - rays.back()[a] * rays.back()[b]);
        }
        double min_dot = 1.0;
        for (std::size_t i = 0; i < rays.size(); ++i)
            for (std::size_t j = i + 1; j < rays.size(); ++j)
                min_dot =
                    std::min(min_dot, rays[i][0] * rays[j][0] + rays[i][1] * rays[j][1] + rays[i][2] * rays[j][2]);
        g.max_parallax_deg = std::acos(std::clamp(min_dot, -1.0, 1.0)) * 180.0 / std::numbers::pi;
        return g;
    }

    /// A portable standard-normal draw from an integer index (Box–Muller over a
    /// hash, not <random>, so sweeps reproduce on every standard library).
    [[nodiscard]] static double pseudo_normal(int i) {
        auto u = [](std::uint32_t k) {
            k ^= k >> 16;
            k *= 0x7feb352du;
            k ^= k >> 15;
            k *= 0x846ca68bu;
            k ^= k >> 16;
            return (static_cast<double>(k) + 0.5) / 4294967296.0;
        };
        const double u1 = u(static_cast<std::uint32_t>(2 * i + 1)), u2 = u(static_cast<std::uint32_t>(2 * i + 2));
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
    }

    struct Landmark {
        Input<double> input;
        Vec3<double> truth{};
        std::size_t id = 0;
    };

    /// Pick the landmark observed in the most consecutive frames from frame 4,
    /// and build its track over (up to) 6 of those frames: clones at the true
    /// poses with exact observations, or at the S2/S3 stage sequence's estimated
    /// poses with the world's pixels normalized through S0.
    [[nodiscard]] static Landmark landmark_window(bool true_poses) {
        namespace st = sdk::msckf::stages;
        const auto world = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
        const std::size_t f0 = 4, m = 6;
        // The landmark seen in every one of frames f0 … f0+m−1.
        std::size_t id = 0;
        for (const auto& o : world.frames[f0].obs) {
            bool all = true;
            for (std::size_t k = f0 + 1; k < f0 + m && all; ++k) {
                bool seen = false;
                for (const auto& q : world.frames[k].obs)
                    seen = seen || q.feature_id == o.feature_id;
                all = seen;
            }
            if (all) {
                id = o.feature_id;
                break;
            }
        }
        Landmark lm;
        lm.id = id;
        lm.truth = world.landmarks[id];
        auto& in = lm.input;
        in.extrinsics = {world.R_imu_cam, world.p_imu_cam};

        State<double> s(0.1);
        if (true_poses) {
            for (std::size_t k = f0; k < f0 + m; ++k)
                s.clones.push_back({world.gt[k].R, world.gt[k].p, world.frames[k].t});
        } else {
            const sdk::msckf::Propagator<double> prop({}, {{0.0, 0.0, -9.81}});
            // Start at frame 1's true state AT ITS TIME, so the estimated poses
            // carry only IMU-propagation drift (zero bias estimates against the
            // world's true biases). Not frame 0: the synthetic trajectory steps
            // from rest to ~1 m/s at the end of the warm-up, which no IMU stream
            // can follow.
            s.R = world.gt[1].R;
            s.p = world.gt[1].p;
            s.v = world.gt[1].v;
            s.timestamp = world.gt[1].t;
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
            // Keep only the clone window over the observing frames (pose state
            // and covariance are irrelevant to S5; it reads the clone poses).
            std::vector<typename State<double>::Clone> keep(s.clones.end() - static_cast<std::ptrdiff_t>(m),
                                                            s.clones.end());
            s = State<double>(0.1);
            s.clones = keep;
        }
        s.cov.P = sdk::msckf::DynMat<double>::identity(s.dim());
        in.state = s;
        const auto cam = world.camera;
        for (std::size_t k = 0; k < m; ++k) {
            if (true_poses) {
                const auto& g = world.gt[f0 + k];
                const auto y = g.R.inverse() * (lm.truth - g.p);
                const auto pc = world.R_imu_cam.inverse() * (y - world.p_imu_cam);
                in.track.observations.push_back({k, 0, {{pc[0] / pc[2], pc[1] / pc[2]}}});
            } else {
                for (const auto& q : world.frames[f0 + k].obs)
                    if (q.feature_id == id) {
                        const auto n = st::s0_sensor_model::apply(cam, q.u, q.v);
                        in.track.observations.push_back({k, 0, {{n.xy[0], n.xy[1]}}});
                    }
            }
        }
        return lm;
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S5_TRIANGULATION_BENCH_HPP
