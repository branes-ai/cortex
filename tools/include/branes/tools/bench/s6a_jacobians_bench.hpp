// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s6a_jacobians_bench.hpp — the S6a_jacobians stage bench
// (issue #457, epic #444).
//
// Stage under test: stages::s6a_jacobians::apply(state, updater, track, p_f) —
// the stacked feature Jacobian H_f (2m×3), state Jacobian H_x (2m×n) and
// residual r = z − h(x̂, p_f). Contract (#445 §B, S6a):
//   • H_f and H_x agree with central finite differences of h, in p_f and in the
//     state's error coordinates (box-plus: R ← R·Exp(δθ), the rest additive);
//   • r is finite, and zero at the true state with exact observations;
//   • reported: the residual RMS in normalized image units.
//
// Fixtures:
//   known_answer   three clones 1 m apart (a 2 m baseline), identity attitudes and
//                  extrinsics, a feature at 4 m: H_f, H_x in closed form, r = 0
//   ground_truth   a synthetic-world track at the true poses with exact
//                  observations and the true landmark: r = 0
//   captured       the same track at the S2/S3 estimated poses, observations
//                  through S0, p_f from the shipped S5 — recorded
//
// Variants: "shipped". First-estimates Jacobians (FEJ) are a planned variant.
//
// Sweep: observations per track × feature depth → the finite-difference
// agreement of H_f and H_x and the size of H_f (conditioning of the feature
// direction).
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S6A_JACOBIANS_BENCH_HPP
#define BRANES_TOOLS_BENCH_S6A_JACOBIANS_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/tools/bench/s6_scene.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S6aJacobiansBench {
    static constexpr std::string_view kStage = "S6a_jacobians";
    static constexpr inv::Stage kInvStage = inv::Stage::S6a_jacobians;

    template <class T>
    struct Input {
        s6::State<T> state{T{1}};
        sdk::msckf::CameraExtrinsics<T> extrinsics{};
        s6::Options<T> options{};
        s6::Track<T> track{};
        s6::Vec3<T> p_f{};
    };
    template <class T>
    struct Output {
        int ok = 0;
        s6::System<T> system{};
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "the analytic Jacobians at the current estimate"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        return json{{"state", pack(in.state)},
                    {"extrinsics",
                     {{"R_imu_cam", pack(in.extrinsics.R_imu_cam)}, {"p_imu_cam", pack(in.extrinsics.p_imu_cam)}}},
                    {"options", s6::encode_options(in.options)},
                    {"observations", s6::encode_track(in.track)},
                    {"p_f", pack(in.p_f)}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        Input<T> in;
        in.state = unpack_state<T>(j.at("state"));
        in.extrinsics.R_imu_cam = unpack_so3<T>(j.at("extrinsics").at("R_imu_cam"));
        in.extrinsics.p_imu_cam = unpack_fixed<T, 3>(j.at("extrinsics").at("p_imu_cam"));
        in.options = s6::decode_options<T>(j.at("options"));
        in.track = s6::decode_track<T>(j.at("observations"), in.state.clones.size());
        in.p_f = unpack_fixed<T, 3>(j.at("p_f"));
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"ok", out.ok}, {"system", s6::encode_system(out.system)}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        return Output<T>{j.at("ok").get<int>(), s6::decode_system<T>(j.at("system"))};
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static sdk::msckf::CameraUpdater<T> updater(const Input<T>& in) {
        return sdk::msckf::CameraUpdater<T>(std::vector<sdk::msckf::CameraExtrinsics<T>>{in.extrinsics}, in.options);
    }

    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view /*variant*/) {
        const auto r = sdk::msckf::stages::s6a_jacobians::apply(in.state, updater(in), in.track, in.p_f);
        return Output<T>{r.ok ? 1 : 0, r.system};
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        std::vector<double> f{static_cast<double>(out.ok)};
        for (const auto* v : {&out.system.Hf, &out.system.Hx, &out.system.r})
            for (const T& x : *v)
                f.push_back(static_cast<double>(x));
        return f;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        r.push_back(inv::check_scalar(out.ok, 1.0, inv::Bound::Lower, kInvStage, "jacobians.resolved", "flag"));
        if (!out.ok)
            return r;
        const auto upd = updater(in);
        const auto& sys = out.system;
        r.push_back(inv::check_finite<T>(std::span<const T>(sys.r), kInvStage, "residual.finite"));

        // h = z − r, as a function of the feature position and of the state's
        // error coordinates.
        std::vector<T> z;
        for (const auto& o : in.track.observations) {
            z.push_back(o.xy[0]);
            z.push_back(o.xy[1]);
        }
        auto h_of = [&](const s6::State<T>& s, const s6::Vec3<T>& p) {
            s6::System<T> q;
            std::vector<T> h(z.size(), std::numeric_limits<T>::quiet_NaN());
            if (upd.measurement_system(s, in.track.observations, p, q))
                for (std::size_t i = 0; i < z.size(); ++i)
                    h[i] = z[i] - q.r[i];
            return h;
        };
        const std::vector<T> p0{in.p_f[0], in.p_f[1], in.p_f[2]};
        r.push_back(inv::check_jacobian_fd<T>(
            [&](std::span<const T> p) { return h_of(in.state, s6::Vec3<T>{{p[0], p[1], p[2]}}); },
            std::span<const T>(p0),
            std::span<const T>(sys.Hf),
            kInvStage,
            "jacobian.feature_vs_fd"));
        const std::vector<T> dx0(sys.cols, T{0});
        r.push_back(
            inv::check_jacobian_fd<T>([&](std::span<const T> dx) { return h_of(boxplus(in.state, dx), in.p_f); },
                                      std::span<const T>(dx0),
                                      std::span<const T>(sys.Hx),
                                      kInvStage,
                                      "jacobian.state_vs_fd"));

        double ss = 0.0;
        for (const T& v : sys.r)
            ss += static_cast<double>(v) * static_cast<double>(v);
        const double rms = sys.r.empty() ? 0.0 : std::sqrt(ss / static_cast<double>(sys.r.size()));
        if (f.kind == FixtureKind::GroundTruth)
            // Exact observations of the true landmark from the true poses.
            r.push_back(inv::check_scalar(rms,
                                          tolerance_vs_double<T>(sys.rows, 1.0),
                                          inv::Bound::Upper,
                                          kInvStage,
                                          "residual.zero_at_truth",
                                          "normalized"));
        else
            r.push_back(inv::check_scalar(rms, 0.0, inv::Bound::Report, kInvStage, "residual.rms", "normalized"));
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_three_clones", known_answer()},
                {"ground_truth_synthetic_track", ground_truth()},
                {"captured_estimated_poses", captured()}};
    }

    /// Three clones at x = −1, 0, 1 m with identity attitude and extrinsics, a
    /// feature at (0.3, −0.2, 4): h = (x/z, y/z) of p_c = p_f − p_k, so
    ///   H_f = dh = (1/z)[[1, 0, −x/z], [0, 1, −y/z]],
    ///   H_θ = dh·[p_c]×,   H_p = −dh,
    /// at each clone's block, and r = 0.
    [[nodiscard]] static Fixture known_answer() {
        Input<double> in;
        in.state = s6::State<double>(0.1);
        const s6::Vec3<double> F{{0.3, -0.2, 4.0}};
        for (int c = 0; c < 3; ++c) {
            sdk::msckf::StateHelper<double>::augment_clone(in.state);
            in.state.clones.back().p = {{-1.0 + c, 0.0, 0.0}};
        }
        Output<double> want;
        want.ok = 1;
        auto& sys = want.system;
        sys.rows = 6;
        sys.cols = in.state.dim();
        sys.Hf.assign(sys.rows * 3, 0.0);
        sys.Hx.assign(sys.rows * sys.cols, 0.0);
        sys.r.assign(sys.rows, 0.0);
        for (std::size_t c = 0; c < 3; ++c) {
            const auto& pk = in.state.clones[c].p;
            const s6::Vec3<double> pc{{F[0] - pk[0], F[1] - pk[1], F[2] - pk[2]}};
            in.track.observations.push_back({c, 0, {{pc[0] / pc[2], pc[1] / pc[2]}}});
            const double iz = 1.0 / pc[2];
            const double dh[2][3] = {{iz, 0.0, -pc[0] * iz * iz}, {0.0, iz, -pc[1] * iz * iz}};
            // [p_c]× = [[0, −z, y], [z, 0, −x], [−y, x, 0]]
            const double hat[3][3] = {{0.0, -pc[2], pc[1]}, {pc[2], 0.0, -pc[0]}, {-pc[1], pc[0], 0.0}};
            const std::size_t off = in.state.clone_offset(c);
            for (std::size_t a = 0; a < 2; ++a)
                for (std::size_t b = 0; b < 3; ++b) {
                    const std::size_t row = 2 * c + a;
                    sys.Hf[row * 3 + b] = dh[a][b];
                    double th = 0.0;
                    for (std::size_t k = 0; k < 3; ++k)
                        th += dh[a][k] * hat[k][b];
                    sys.Hx[row * sys.cols + off + b] = th;
                    sys.Hx[row * sys.cols + off + 3 + b] = -dh[a][b];
                }
        }
        in.p_f = F;
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.source = "s6a_jacobians_bench: 3 clones on a 2 m baseline, identity attitude, feature at 4 m";
        f.description = "expected: closed-form H_f, H_x; r = 0";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        return f;
    }

    /// The first synthetic-world track at the true poses: exact observations of
    /// the true landmark.
    [[nodiscard]] static Fixture ground_truth() {
        const auto sc = s6::build_scene(/*true_poses=*/true);
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config), true poses, exact observations, true landmark";
        f.seed = sdk::eval::SyntheticConfig<double>{}.seed;
        f.description = "truth: r = 0";
        f.input = encode_input(input_of(sc, sc.landmarks.front()));
        f.truth = json{{"p_f", pack(sc.landmarks.front())}, {"residual", 0.0}};
        return f;
    }

    /// The same track at the S2/S3 estimated poses, with the shipped S5's p_f —
    /// recorded.
    [[nodiscard]] static Fixture captured() {
        const auto sc = s6::build_scene(/*true_poses=*/false);
        const auto in = input_of(sc, s6::boundaries(sc, 0).p_f);
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config), S2/S3 estimated poses, S0-normalized pixels, shipped S5 p_f",
                       encode_input(in),
                       encode_output(run<double>(in, kShipped)));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("observations", {2.0, 3.0, 5.0, 8.0}).axis("depth_m", {1.0, 4.0, 16.0, 64.0});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"fd_err_feature", "fd_err_state", "max_abs_Hf"};
    }
    /// `observations` clones on a 0.2 m-spaced baseline, a feature `depth_m`
    /// ahead: the finite-difference error of H_f and H_x, and max |H_f| (∝ 1/depth).
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double m = p.at("observations"), depth = p.at("depth_m");
        if (!(m >= 2.0 && m <= 64.0) || !(depth > 0.0 && depth <= 1e4))
            throw std::invalid_argument("s6a bench sweep: observations in [2, 64], depth_m in (0, 1e4]");
        Input<double> in;
        in.state = s6::State<double>(0.1);
        const s6::Vec3<double> F{{0.1, -0.05, depth}};
        for (std::size_t c = 0; c < static_cast<std::size_t>(m); ++c) {
            sdk::msckf::StateHelper<double>::augment_clone(in.state);
            in.state.clones.back().p = {{0.2 * static_cast<double>(c), 0.0, 0.0}};
            const auto& pk = in.state.clones.back().p;
            in.track.observations.push_back({c, 0, {{(F[0] - pk[0]) / F[2], (F[1] - pk[1]) / F[2]}}});
        }
        in.p_f = F;
        const auto inT = decode_input<T>(encode_input(in));
        const auto out = run<T>(inT, kShipped);
        const auto rep = invariants<T>(inT, out, Fixture{});
        double fd_f = inv::detail::kInf, fd_x = inv::detail::kInf, hmax = 0.0;
        for (const auto& x : rep) {
            if (x.name == "jacobian.feature_vs_fd")
                fd_f = x.value;
            if (x.name == "jacobian.state_vs_fd")
                fd_x = x.value;
        }
        for (const T& v : out.system.Hf)
            hmax = std::max(hmax, std::abs(static_cast<double>(v)));
        return {fd_f, fd_x, hmax};
    }

    /// Error-state box-plus: R ← R·Exp(δθ), p, v, biases additive; each clone
    /// R_c ← R_c·Exp(δθ_c), p_c additive.
    template <class T>
    [[nodiscard]] static s6::State<T> boxplus(const s6::State<T>& s, std::span<const T> dx) {
        using St = s6::State<T>;
        using SO3 = typename St::SO3;
        auto v3 = [&](std::size_t o) { return s6::Vec3<T>{{dx[o], dx[o + 1], dx[o + 2]}}; };
        St o = s;
        o.R = s.R * SO3::exp(v3(St::kTheta));
        o.p = s.p + v3(St::kPos);
        o.v = s.v + v3(St::kVel);
        o.bg = s.bg + v3(St::kBg);
        o.ba = s.ba + v3(St::kBa);
        for (std::size_t c = 0; c < s.clones.size(); ++c) {
            const std::size_t off = s.clone_offset(c);
            o.clones[c].R = s.clones[c].R * SO3::exp(v3(off));
            o.clones[c].p = s.clones[c].p + v3(off + 3);
        }
        return o;
    }

private:
    [[nodiscard]] static Input<double> input_of(const s6::Scene& sc, const s6::Vec3<double>& p_f) {
        Input<double> in;
        in.state = sc.state;
        in.extrinsics = sc.extrinsics;
        in.track = sc.tracks.front();
        in.p_f = p_f;
        return in;
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S6A_JACOBIANS_BENCH_HPP
