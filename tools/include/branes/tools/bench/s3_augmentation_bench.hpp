// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s3_augmentation_bench.hpp — the S3_augmentation stage
// bench (issue #455, epic #444).
//
// Stage under test: stages::s3_augmentation::apply(state, t) → state with a
// stochastic clone of the IMU pose. Contract (#445 §B, S3):
//   • the dimension grows by exactly 6;
//   • P_aug = [[P, P Jᵀ], [J P, J P Jᵀ]] with J the clone Jacobian (identity onto
//     the IMU δθ and δp blocks) — the clone carries the exact cross-covariance;
//   • P_aug stays ⪰ 0 (it is singular by construction: the clone is a copy);
//   • the clone is the current pose, stamped at t; the mean is untouched.
//
// Fixtures:
//   known_answer   a correlated state with clones: the augmented P built block
//                  by block, independently of the stage
//   ground_truth   the synthetic world with the mean injected to the true pose
//                  (#266); truth = the clone must be exactly that pose
//   captured       an S3 boundary of the synthetic world's S2/S3 stage sequence
//
// Variants: "shipped" (full covariance, P ← G P Gᵀ) and "sqrt_covariance" (the
// square-root factor re-triangularized, reported as S Sᵀ).
//
// Sweep: clones already in the window × conditioning → block residual,
// λ_min/‖P‖ after augmentation, and run time.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S3_AUGMENTATION_BENCH_HPP
#define BRANES_TOOLS_BENCH_S3_AUGMENTATION_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/sqrt_covariance.hpp>
#include <branes/sdk/msckf/stages/s2_propagation.hpp>
#include <branes/sdk/msckf/stages/s3_augmentation.hpp>
#include <branes/tools/bench/bench.hpp>
#include <branes/tools/bench/s2_propagation_bench.hpp>  // correlated_spd

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S3AugmentationBench {
    static constexpr std::string_view kStage = "S3_augmentation";
    static constexpr inv::Stage kInvStage = inv::Stage::S3_augmentation;
    /// One dense (d+6)×d product per run; in software arithmetic one timed run is enough.
    static constexpr int kTimingReps = 1;

    template <class T>
    using State = sdk::msckf::State<T>;
    template <class T>
    using DynMat = sdk::msckf::DynMat<T>;

    template <class T>
    struct Input {
        State<T> state{T{1}};
        double t = 0.0;  ///< the clone's timestamp (the frame time)
    };
    template <class T>
    struct Output {
        State<T> state{T{1}};
        sdk::msckf::stages::s3_augmentation::Diagnostics diag{};
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "stages::s3_augmentation::apply (full covariance, P <- G P G^T)"},
                {"sqrt_covariance", "the square-root factor re-triangularized, reported as S S^T"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        return json{{"state", pack(in.state)}, {"t", pack_num(in.t)}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        return Input<T>{unpack_state<T>(j.at("state")), unpack_num(j.at("t"))};
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"state", pack(out.state)},
                    {"dim_before", out.diag.dim_before},
                    {"dim_after", out.diag.dim_after},
                    {"clones", out.diag.clones}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        Output<T> out{unpack_state<T>(j.at("state")), {}};
        out.diag = {j.at("dim_before").get<std::size_t>(),
                    j.at("dim_after").get<std::size_t>(),
                    j.at("clones").get<std::size_t>()};
        return out;
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        Output<T> out{in.state, {}};
        if (variant != "sqrt_covariance") {
            out.diag = sdk::msckf::stages::s3_augmentation::apply(out.state, in.t);
            return out;
        }
        // Square-root form: S = chol(P), augment the factor, report S Sᵀ.
        sdk::msckf::State<T, sdk::msckf::SqrtCovariance<T>> sq(T{1});
        sq.R = in.state.R;
        sq.p = in.state.p;
        sq.v = in.state.v;
        sq.bg = in.state.bg;
        sq.ba = in.state.ba;
        sq.timestamp = in.state.timestamp;
        for (const auto& c : in.state.calib)
            sq.calib.push_back({c.R_imu_cam, c.p_imu_cam});
        for (const auto& c : in.state.clones)
            sq.clones.push_back({c.R, c.p, c.timestamp});
        DynMat<T> L;
        if (!psd_factor(in.state.cov.P, L)) {
            out.diag = sdk::msckf::stages::s3_augmentation::apply(out.state, in.t);
            for (auto& x : out.state.cov.P.d)
                x = T(std::numeric_limits<double>::quiet_NaN());  // not PSD in T: no square-root form
            return out;
        }
        sq.cov.S = L;
        out.diag = sdk::msckf::stages::s3_augmentation::apply(sq, in.t);
        out.state.timestamp = sq.timestamp;
        out.state.clones.clear();
        for (const auto& c : sq.clones)
            out.state.clones.push_back({c.R, c.p, c.timestamp});
        out.state.cov.P = sq.covariance();
        return out;
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        std::vector<double> v;
        const auto& s = out.state;
        for (std::size_t i = 0; i < 4; ++i)
            v.push_back(static_cast<double>(s.R.quaternion()[i]));
        for (const auto* x : {&s.p, &s.v, &s.bg, &s.ba})
            for (std::size_t i = 0; i < 3; ++i)
                v.push_back(static_cast<double>((*x)[i]));
        v.push_back(s.timestamp);
        for (const auto& c : s.clones) {
            for (std::size_t i = 0; i < 4; ++i)
                v.push_back(static_cast<double>(c.R.quaternion()[i]));
            for (std::size_t i = 0; i < 3; ++i)
                v.push_back(static_cast<double>(c.p[i]));
            v.push_back(c.timestamp);
        }
        for (const T& x : s.cov.P.d)
            v.push_back(static_cast<double>(x));
        v.insert(v.end(),
                 {static_cast<double>(out.diag.dim_before),
                  static_cast<double>(out.diag.dim_after),
                  static_cast<double>(out.diag.clones)});
        return v;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        const auto& before = in.state;
        const auto& after = out.state;
        for (auto& x : inv::check_augmentation(before.cov.P, clone_jacobian<T>(before), after.cov.P, kInvStage))
            r.push_back(x);
        r.push_back(inv::check_symmetric(after.cov.P, kInvStage, "covariance.symmetric"));
        r.push_back(inv::check_psd(after.cov.P, kInvStage, "covariance.psd"));
        r.push_back(
            inv::check_dimension(after.clones.size(), before.clones.size() + 1, kInvStage, "window.clones_plus_1"));

        // The clone is the current pose, stamped at t; nothing else in the mean moves.
        std::size_t changed = 0;
        auto cmp = [&](double a, double b) { changed += detail::same_bits(a, b) ? 0 : 1; };
        if (!after.clones.empty()) {
            const auto& c = after.clones.back();
            for (std::size_t i = 0; i < 4; ++i)
                cmp(static_cast<double>(c.R.quaternion()[i]), static_cast<double>(before.R.quaternion()[i]));
            for (std::size_t i = 0; i < 3; ++i)
                cmp(static_cast<double>(c.p[i]), static_cast<double>(before.p[i]));
            cmp(c.timestamp, in.t);
        } else {
            changed = 1;
        }
        r.push_back(inv::check_scalar(
            static_cast<double>(changed), 0.0, inv::Bound::Upper, kInvStage, "clone.equals_current_pose", "count"));
        r.push_back(inv::check_scalar(
            std::abs(after.timestamp - in.t), 0.0, inv::Bound::Upper, kInvStage, "state.stamped_at_t", "s"));
        if (f.kind == FixtureKind::GroundTruth)
            r.push_back(truth_check<T>(after, f.truth));
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_two_clones", known_answer()},
                {"ground_truth_synthetic_pose", ground_truth()},
                {"captured_stage_sequence", captured()}};
    }

    /// A correlated state with two clones; the augmented covariance is
    /// assembled block by block here, independently of the stage.
    [[nodiscard]] static Fixture known_answer() {
        Input<double> in;
        in.state = State<double>(0.1);
        in.state.R = math::lie::SO3<double>::exp({{0.2, -0.1, 0.4}});
        in.state.p = {{0.5, 1.5, -0.3}};
        in.state.v = {{0.1, 0.0, 0.2}};
        for (int c = 0; c < 2; ++c)
            in.state.clones.push_back(
                {math::lie::SO3<double>::exp({{0.05 * c, 0.0, 0.1}}), {{0.2 * c, 0.0, 0.0}}, 1.0 + 0.05 * c});
        in.state.cov.P = S2PropagationBench::correlated_spd(in.state.dim());
        in.state.timestamp = 1.2;
        in.t = 1.25;

        const std::size_t d = in.state.dim();
        Output<double> want{in.state, {d, d + 6, 3}};
        auto& P = want.state.cov.P;
        P = DynMat<double>(d + 6, d + 6);
        const auto& P0 = in.state.cov.P;
        for (std::size_t i = 0; i < d; ++i)
            for (std::size_t j = 0; j < d; ++j)
                P(i, j) = P0(i, j);
        // The clone block copies the IMU (δθ, δp) rows/cols: J selects indices
        // 0–2 (θ) and 3–5 (p), so P_aug[clone, ·] = P[0..5, ·].
        for (std::size_t a = 0; a < 6; ++a) {
            for (std::size_t j = 0; j < d; ++j) {
                P(d + a, j) = P0(a, j);
                P(j, d + a) = P0(j, a);
            }
            for (std::size_t b = 0; b < 6; ++b)
                P(d + a, d + b) = P0(a, b);
        }
        want.state.timestamp = in.t;
        want.state.clones.push_back({in.state.R, in.state.p, in.t});
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.source = "s3_augmentation_bench: correlated 27x27 P, two clones, augment at t = 1.25";
        f.description = "augmented covariance assembled block by block; valid for every variant";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        return f;
    }

    /// The synthetic world's frame 3, with the mean injected to the true pose:
    /// the new clone must be exactly that pose.
    [[nodiscard]] static Fixture ground_truth() {
        const auto seq = stage_sequence(/*inject_truth=*/true, /*frame=*/3);
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config), S2/S3 with truth injected, augment at frame 3";
        f.seed = sdk::eval::SyntheticConfig<double>{}.seed;
        f.description = "truth: the clone is the true pose of the frame";
        f.input = encode_input(seq.input);
        f.truth = json{{"R", pack(seq.truth_R)}, {"p", pack(seq.truth_p)}};
        return f;
    }

    /// The S3 boundary of frame 3 of the S2/S3 stage sequence, recorded.
    [[nodiscard]] static Fixture captured() {
        const auto seq = stage_sequence(/*inject_truth=*/false, /*frame=*/3);
        Output<double> got{seq.input.state, {}};
        got.diag = sdk::msckf::stages::s3_augmentation::apply(got.state, seq.input.t);
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config), S2/S3 stage sequence, frame 3",
                       encode_input(seq.input),
                       encode_output(got));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("clones_before", {0, 2, 5, 10}).axis("log10_cond", {0, 4, 8});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"dim_after", "block_residual", "lambda_min_rel", "run_us"};
    }
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double c = p.at("clones_before"), lc = p.at("log10_cond");
        if (!std::isfinite(c) || c < 0.0 || c > 64.0 || c != std::floor(c))
            throw std::invalid_argument("s3 bench sweep: clones_before must be an integer in [0, 64]");
        if (!std::isfinite(lc) || lc < 0.0 || lc > 15.0)
            throw std::invalid_argument("s3 bench sweep: log10_cond must be in [0, 15]");
        Input<double> in0;
        in0.state = State<double>(0.1);
        for (std::size_t k = 0; k < static_cast<std::size_t>(c); ++k)
            in0.state.clones.push_back({{}, {{0.3 * double(k), 0.0, 0.0}}, double(k)});
        in0.state.cov.P = S2PropagationBench::correlated_spd(in0.state.dim(), 1e-2 * std::pow(10.0, -lc));
        in0.t = 100.0;
        const auto in = decode_input<T>(encode_input(in0));
        Output<T> out;
        const double us = detail::best_time_us([&] { out = run<T>(in, kShipped); }, 1);
        const auto blocks = inv::check_augmentation(in.state.cov.P, clone_jacobian<T>(in.state), out.state.cov.P);
        const auto ev = inv::la::symmetric_eigenvalues(out.state.cov.P);
        const double scale = std::max(1e-300, static_cast<double>(inv::la::max_abs(out.state.cov.P)));
        return {static_cast<double>(out.state.dim()), blocks.back().value, static_cast<double>(ev.front()) / scale, us};
    }

private:
    /// J: the 6×n Jacobian of the new clone (δθ_c, δp_c) w.r.t. the error
    /// state — identity onto the IMU δθ and δp blocks.
    template <class T>
    [[nodiscard]] static DynMat<T> clone_jacobian(const State<T>& s) {
        DynMat<T> J(6, s.dim());
        for (std::size_t i = 0; i < 3; ++i) {
            J(i, State<T>::kTheta + i) = T{1};
            J(3 + i, State<T>::kPos + i) = T{1};
        }
        return J;
    }

    template <class T>
    [[nodiscard]] static inv::InvariantResult truth_check(const State<T>& after, const json& truth) {
        if (after.clones.empty())
            return inv::check_scalar(inv::detail::kInf, 0.0, inv::Bound::Upper, kInvStage, "truth.clone_pose", "m");
        const auto p = unpack_fixed<double, 3>(truth.at("p"));
        const auto q = unpack_fixed<double, 4>(truth.at("R"));
        double worst = 0.0;
        const auto& c = after.clones.back();
        for (std::size_t i = 0; i < 3; ++i) {
            const double d = std::abs(static_cast<double>(c.p[i]) - p[i]);
            worst = std::isfinite(d) ? std::max(worst, d) : inv::detail::kInf;
        }
        for (std::size_t i = 0; i < 4; ++i) {
            const double d = std::abs(static_cast<double>(c.R.quaternion()[i]) - q[i]);
            worst = std::isfinite(d) ? std::max(worst, d) : inv::detail::kInf;
        }
        // The pose went through T once (the fixture's decode): it can differ
        // from the double truth by T's rounding of an O(1..10) value.
        return inv::check_scalar(worst,
                                 tolerance_vs_double<T>(1, 10.0, 4.0),
                                 inv::Bound::Upper,
                                 kInvStage,
                                 "truth.clone_pose",
                                 "m | quaternion");
    }

    struct Sequence {
        Input<double> input;
        math::lie::SO3<double> truth_R;
        math::lie::detail::Vec<double, 3> truth_p{};
    };

    /// S2 over the synthetic IMU stream and S3 at each frame, up to frame
    /// `frame`, whose S3 input is returned. With `inject_truth`, the mean is
    /// reset to the true pose before every clone.
    [[nodiscard]] static Sequence stage_sequence(bool inject_truth, std::size_t frame) {
        namespace st = sdk::msckf::stages;
        const auto world = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
        const sdk::msckf::Propagator<double> prop({}, {{0.0, 0.0, -9.81}});
        State<double> s(0.1);
        s.R = world.gt.front().R;
        s.p = world.gt.front().p;
        s.v = world.gt.front().v;
        s.timestamp = world.imu.front().timestamp_s;
        std::size_t k = 0;
        for (std::size_t f = 0; f <= frame; ++f) {
            const double t = world.frames[f].t;
            for (; k < world.imu.size() && world.imu[k].timestamp_s <= t; ++k) {
                const auto& m = world.imu[k];
                st::s2_propagation::apply(
                    s,
                    prop,
                    {{m.angular_velocity[0], m.angular_velocity[1], m.angular_velocity[2]}},
                    {{m.linear_acceleration[0], m.linear_acceleration[1], m.linear_acceleration[2]}},
                    m.timestamp_s - s.timestamp);
                s.timestamp = m.timestamp_s;
            }
            if (inject_truth) {
                s.R = world.gt[f].R;
                s.p = world.gt[f].p;
                s.v = world.gt[f].v;
            }
            if (f == frame)
                return Sequence{{s, t}, world.gt[f].R, world.gt[f].p};
            st::s3_augmentation::apply(s, t);
        }
        throw std::logic_error("s3 bench: frame beyond the synthetic run");
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S3_AUGMENTATION_BENCH_HPP
