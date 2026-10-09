// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s9_marginalization_bench.hpp — the S9_marginalization
// stage bench (issue #453's worked example; the full S9–S10 benches are #458).
//
// Stage under test: stages::s9_marginalization::apply(state, clone index) →
// (state without that clone, diagnostics). Its contract (#445 §B, S9):
//   • P after removal is exactly the principal submatrix of P before;
//   • dimension drops by 6, the clone window by 1, the right clone goes;
//   • P stays symmetric and ⪰ 0;
//   • the mean is untouched — marginalization moves no estimate.
//
// Fixtures (all three kinds, generated from real stage runs):
//   known_answer   a correlated state with an analytic expected output (the
//                  gathered submatrix, computed independently of the stage)
//   ground_truth   clones placed at the synthetic world's TRUE poses (truth
//                  injected upstream, #266); the kept clones must stay exactly
//                  on the truth
//   captured       the S9 boundary recorded while running the S2/S3/S9 stage
//                  sequence on the synthetic world's IMU stream
//
// Variants: "shipped" (the stage) and "direct_gather" (an index gather written
// out by hand), to exercise the variant slot on identical fixtures.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S9_MARGINALIZATION_BENCH_HPP
#define BRANES_TOOLS_BENCH_S9_MARGINALIZATION_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/stages/s2_propagation.hpp>
#include <branes/sdk/msckf/stages/s3_augmentation.hpp>
#include <branes/sdk/msckf/stages/s9_marginalization.hpp>
#include <branes/tools/bench/bench.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S9MarginalizationBench {
    static constexpr std::string_view kStage = "S9_marginalization";
    static constexpr inv::Stage kInvStage = inv::Stage::S9_marginalization;

    template <class T>
    using State = sdk::msckf::State<T>;

    template <class T>
    struct Input {
        State<T> state;
        std::size_t clone_index = 0;
    };
    template <class T>
    struct Output {
        State<T> state;
        sdk::msckf::stages::s9_marginalization::Diagnostics diag;
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "stages::s9_marginalization::apply"},
                {"direct_gather", "hand-written principal-submatrix gather, for comparison"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        return json{{"state", pack(in.state)}, {"clone_index", in.clone_index}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        return Input<T>{unpack_state<T>(j.at("state")), j.at("clone_index").get<std::size_t>()};
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"state", pack(out.state)},
                    {"dim_before", out.diag.dim_before},
                    {"dim_after", out.diag.dim_after},
                    {"removed_time", pack_num(out.diag.removed_time)},
                    {"clones", out.diag.clones}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        Output<T> out{unpack_state<T>(j.at("state")), {}};
        out.diag.dim_before = j.at("dim_before").get<std::size_t>();
        out.diag.dim_after = j.at("dim_after").get<std::size_t>();
        out.diag.removed_time = unpack_num(j.at("removed_time"));
        out.diag.clones = j.at("clones").get<std::size_t>();
        return out;
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        Output<T> out{in.state, {}};
        if (variant == "direct_gather") {
            auto& s = out.state;
            const std::size_t d = s.dim(), off = s.clone_offset(in.clone_index);
            out.diag.dim_before = d;
            out.diag.removed_time = s.clones[in.clone_index].timestamp;
            sdk::msckf::DynMat<T> P(d - 6, d - 6);
            for (std::size_t i = 0, a = 0; i < d; ++i) {
                if (i >= off && i < off + 6)
                    continue;
                for (std::size_t j = 0, b = 0; j < d; ++j) {
                    if (j >= off && j < off + 6)
                        continue;
                    P(a, b++) = s.cov.P(i, j);
                }
                ++a;
            }
            s.cov.P = std::move(P);
            s.clones.erase(s.clones.begin() + static_cast<std::ptrdiff_t>(in.clone_index));
            out.diag.dim_after = s.dim();
            out.diag.clones = s.clones.size();
            return out;
        }
        out.diag = sdk::msckf::stages::s9_marginalization::apply(out.state, in.clone_index);
        return out;
    }

    /// Every number of the output, for the cross-type and replay comparisons.
    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        std::vector<double> v;
        const auto& s = out.state;
        auto put3 = [&](const auto& x) {
            for (std::size_t i = 0; i < 3; ++i)
                v.push_back(static_cast<double>(x[i]));
        };
        auto put_rot = [&](const auto& r) {
            for (std::size_t i = 0; i < 4; ++i)
                v.push_back(static_cast<double>(r.quaternion()[i]));
        };
        put_rot(s.R);
        put3(s.p);
        put3(s.v);
        put3(s.bg);
        put3(s.ba);
        v.push_back(s.timestamp);
        for (const auto& c : s.calib) {
            put_rot(c.R_imu_cam);
            put3(c.p_imu_cam);
        }
        for (const auto& c : s.clones) {
            put_rot(c.R);
            put3(c.p);
            v.push_back(c.timestamp);
        }
        for (const T& x : s.cov.P.d)
            v.push_back(static_cast<double>(x));
        // Every diagnostic encode_output writes, so replay and known-answer
        // comparisons cover the whole recorded output.
        v.push_back(static_cast<double>(out.diag.dim_before));
        v.push_back(static_cast<double>(out.diag.dim_after));
        v.push_back(out.diag.removed_time);
        v.push_back(static_cast<double>(out.diag.clones));
        return v;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        const auto& before = in.state;
        const auto& after = out.state;
        const std::size_t off = before.clone_offset(in.clone_index);
        std::vector<std::size_t> keep;
        for (std::size_t i = 0; i < before.dim(); ++i)
            if (i < off || i >= off + 6)
                keep.push_back(i);
        for (auto& x : inv::check_principal_submatrix<T>(before.cov.P, keep, after.cov.P, kInvStage))
            r.push_back(x);
        r.push_back(inv::check_symmetric(after.cov.P, kInvStage, "covariance.symmetric"));
        r.push_back(inv::check_psd(after.cov.P, kInvStage, "covariance.psd"));
        r.push_back(
            inv::check_dimension(after.clones.size() + 1, before.clones.size(), kInvStage, "window.clones_minus_1"));
        r.push_back(inv::check_scalar(std::abs(out.diag.removed_time - before.clones[in.clone_index].timestamp),
                                      0.0,
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "window.removed_the_requested_clone",
                                      "s"));
        r.push_back(inv::check_scalar(static_cast<double>(mean_changes(before, after, in.clone_index)),
                                      0.0,
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "mean.untouched",
                                      "count"));
        if (f.kind == FixtureKind::GroundTruth)
            for (auto& x : truth_check<T>(after, f.truth))
                r.push_back(x);
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_three_clones", known_answer()},
                {"ground_truth_synthetic_world", ground_truth()},
                {"captured_stage_sequence", captured()}};
    }

    /// A known-answer fixture: 3 clones, a dense correlated P, remove the middle
    /// clone. The expected output is built independently of the stage.
    [[nodiscard]] static Fixture known_answer() {
        State<double> s(0.1);
        for (std::size_t c = 0; c < 3; ++c)
            s.clones.push_back({math::lie::SO3<double>::exp({{0.1 * double(c), -0.05, 0.02}}),
                                {{0.4 * double(c), 0.1, 0.0}},
                                10.0 + 0.05 * double(c)});
        s.cov.P = correlated_spd(s.dim(), 0.0);
        const Input<double> in{s, 1};

        Output<double> want{s, {}};
        const std::size_t d = s.dim(), off = s.clone_offset(1);
        sdk::msckf::DynMat<double> P(d - 6, d - 6);
        for (std::size_t i = 0, a = 0; i < d; ++i) {
            if (i >= off && i < off + 6)
                continue;
            for (std::size_t j = 0, b = 0; j < d; ++j)
                if (j < off || j >= off + 6)
                    P(a, b++) = s.cov.P(i, j);
            ++a;
        }
        want.state.cov.P = P;
        want.state.clones.erase(want.state.clones.begin() + 1);
        want.diag = {d, d - 6, s.clones[1].timestamp, 2};

        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.source = "s9_marginalization_bench: correlated_spd(33), remove clone 1";
        f.description = "principal-submatrix gather, expected output computed analytically";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        return f;
    }

    /// A ground-truth-injected fixture: the S2/S3 stage sequence runs on the
    /// synthetic world with the mean reset to the TRUE pose before every clone,
    /// so the window holds the true poses; S9 removes the oldest.
    [[nodiscard]] static Fixture ground_truth() {
        const auto seq = stage_sequence(/*inject_truth=*/true, /*max_clones=*/4, /*capture_at_frame=*/6);
        json truth = json::array();
        for (std::size_t c = 1; c < seq.input.state.clones.size(); ++c)
            truth.push_back({{"R", pack(seq.input.state.clones[c].R)}, {"p", pack(seq.input.state.clones[c].p)}});
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config), S2/S3 with truth injected, frame 6";
        f.seed = sdk::eval::SyntheticConfig<double>{}.seed;
        f.description = "kept clones must stay exactly on the injected true poses";
        f.input = encode_input(seq.input);
        f.truth = truth;
        return f;
    }

    /// A captured fixture: the S9 boundary of the S2/S3/S9 stage sequence on the
    /// synthetic IMU stream (no truth injection), recorded with the output the
    /// stage produced in that run.
    [[nodiscard]] static Fixture captured() {
        const auto seq = stage_sequence(/*inject_truth=*/false, /*max_clones=*/4, /*capture_at_frame=*/6);
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config), S2/S3/S9 stage sequence, frame 6",
                       encode_input(seq.input),
                       encode_output(seq.output));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("clones", {2, 4, 8, 11}).axis("log10_cond", {0, 4, 8});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"dim", "submatrix_residual", "lambda_min_after", "run_us"};
    }
    /// Remove the oldest of `clones` clones from a state whose P has condition
    /// number ~10^log10_cond: the residual must stay 0 (a pure gather) and the
    /// smallest eigenvalue is reported in the type under test.
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double clones = p.at("clones");
        const double log10_cond = p.at("log10_cond");
        if (!std::isfinite(clones) || clones < 1.0 || clones > 64.0 || clones != std::floor(clones))
            throw std::invalid_argument("s9 bench sweep: clones must be an integer in [1, 64]");
        if (!std::isfinite(log10_cond) || log10_cond < 0.0 || log10_cond > 15.0)
            throw std::invalid_argument("s9 bench sweep: log10_cond must be in [0, 15]");
        const auto n = static_cast<std::size_t>(clones);
        State<double> s(0.1);
        for (std::size_t c = 0; c < n; ++c)
            s.clones.push_back({{}, {{0.3 * double(c), 0.0, 0.0}}, double(c)});
        s.cov.P = correlated_spd(s.dim(), log10_cond);
        const auto in = decode_input<T>(encode_input(Input<double>{s, 0}));
        Output<T> out;
        const double us = detail::best_time_us([&] { out = run<T>(in, kShipped); }, 3);
        const auto inv_sub = inv::check_principal_submatrix<T>(in.state.cov.P, keep_of(in.state, 0), out.state.cov.P);
        const auto ev = inv::la::symmetric_eigenvalues(out.state.cov.P);
        return {static_cast<double>(s.dim()), inv_sub.back().value, static_cast<double>(ev.front()), us};
    }

private:
    template <class T>
    [[nodiscard]] static std::vector<std::size_t> keep_of(const State<T>& s, std::size_t idx) {
        std::vector<std::size_t> keep;
        const std::size_t off = s.clone_offset(idx);
        for (std::size_t i = 0; i < s.dim(); ++i)
            if (i < off || i >= off + 6)
                keep.push_back(i);
        return keep;
    }

    /// A dense SPD matrix B·Bᵀ + 10^(−log10_cond)·I with B (n × n/2) fixed and
    /// rank-deficient, so the condition number is ~10^log10_cond.
    [[nodiscard]] static sdk::msckf::DynMat<double> correlated_spd(std::size_t n, double log10_cond) {
        const std::size_t k = n / 2 + 1;
        sdk::msckf::DynMat<double> B(n, k);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < k; ++j)
                B(i, j) = 0.1 * std::sin(0.7 * double(i + 1) + 1.3 * double(j + 1));
        auto P = sdk::msckf::mul(B, sdk::msckf::transpose(B));
        const double ridge = std::pow(10.0, -log10_cond) * 0.01;
        for (std::size_t i = 0; i < n; ++i)
            P(i, i) += ridge;
        sdk::msckf::symmetrize(P);
        return P;
    }

    /// Count of mean components that changed (nav + kept clones), bitwise.
    template <class T>
    [[nodiscard]] static std::size_t mean_changes(const State<T>& a, const State<T>& b, std::size_t removed) {
        std::size_t n = 0;
        auto cmp3 = [&](const auto& x, const auto& y) {
            for (std::size_t i = 0; i < 3; ++i)
                n += detail::same_bits(static_cast<double>(x[i]), static_cast<double>(y[i])) ? 0 : 1;
        };
        auto cmp_rot = [&](const auto& x, const auto& y) {
            for (std::size_t i = 0; i < 4; ++i)
                n += detail::same_bits(static_cast<double>(x.quaternion()[i]), static_cast<double>(y.quaternion()[i]))
                         ? 0
                         : 1;
        };
        cmp_rot(a.R, b.R);
        cmp3(a.p, b.p);
        cmp3(a.v, b.v);
        cmp3(a.bg, b.bg);
        cmp3(a.ba, b.ba);
        for (std::size_t c = 0, k = 0; c < a.clones.size(); ++c) {
            if (c == removed)
                continue;
            if (k >= b.clones.size())
                return n + 1;
            cmp_rot(a.clones[c].R, b.clones[k].R);
            cmp3(a.clones[c].p, b.clones[k].p);
            ++k;
        }
        return n;
    }

    /// Ground truth: the kept clones' positions (m) and rotations (rad) against
    /// the injected truth. The rotation error is the angle of q_trueᶜ ⊗ q,
    /// 2·atan2(‖v‖, |w|), which stays accurate near zero (unlike acos of a dot).
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult> truth_check(const State<T>& after, const json& truth) {
        const bool same_size = after.clones.size() == truth.size();
        double worst_p = same_size ? 0.0 : inv::detail::kInf;
        double worst_r = same_size ? 0.0 : inv::detail::kInf;
        for (std::size_t c = 0; c < std::min(after.clones.size(), truth.size()); ++c) {
            const auto p = unpack_fixed<double, 3>(truth[c].at("p"));
            for (std::size_t i = 0; i < 3; ++i) {
                // A non-finite position (estimate or truth) never matches: std::max drops a NaN.
                const double d = std::abs(static_cast<double>(after.clones[c].p[i]) - p[i]);
                worst_p = std::isfinite(d) ? std::max(worst_p, d) : inv::detail::kInf;
            }
            const auto qt = unpack_fixed<double, 4>(truth[c].at("R"));
            const auto& qa = after.clones[c].R.quaternion();
            const double a0 = static_cast<double>(qa[0]), a1 = static_cast<double>(qa[1]);
            const double a2 = static_cast<double>(qa[2]), a3 = static_cast<double>(qa[3]);
            // q_rel = conj(qt) ⊗ qa (Hamilton product).
            const double w = qt[0] * a0 + qt[1] * a1 + qt[2] * a2 + qt[3] * a3;
            const double x = qt[0] * a1 - qt[1] * a0 - qt[2] * a3 + qt[3] * a2;
            const double y = qt[0] * a2 + qt[1] * a3 - qt[2] * a0 - qt[3] * a1;
            const double z = qt[0] * a3 - qt[1] * a2 + qt[2] * a1 - qt[3] * a0;
            const double angle = 2.0 * std::atan2(std::sqrt(x * x + y * y + z * z), std::abs(w));
            worst_r = std::max(worst_r, std::isfinite(angle) ? angle : inv::detail::kInf);
        }
        return {inv::check_scalar(worst_p,
                                  inv::arithmetic_tolerance<T>(3, 10.0),
                                  inv::Bound::Upper,
                                  kInvStage,
                                  "truth.kept_clone_position",
                                  "m"),
                inv::check_scalar(worst_r,
                                  inv::arithmetic_tolerance<T>(4, 1.0),
                                  inv::Bound::Upper,
                                  kInvStage,
                                  "truth.kept_clone_rotation",
                                  "rad")};
    }

    struct Sequence {
        Input<double> input;
        Output<double> output;
    };

    /// Run S2 (every IMU sample) and S3 (every camera frame) on the synthetic
    /// world, S9-ing the oldest clone whenever the window is full, and capture
    /// the S9 boundary of camera frame `capture_at_frame`. With `inject_truth`,
    /// the mean is reset to the true pose before each clone (#266).
    [[nodiscard]] static Sequence
    stage_sequence(bool inject_truth, std::size_t max_clones, std::size_t capture_at_frame) {
        namespace st = sdk::msckf::stages;
        const auto world = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
        const sdk::msckf::Propagator<double> prop({}, {{0.0, 0.0, -9.81}});
        State<double> s(0.1);
        s.R = world.gt.front().R;
        s.p = world.gt.front().p;
        s.v = world.gt.front().v;
        s.timestamp = world.imu.front().timestamp_s;
        std::size_t k = 0;
        Sequence seq;
        for (std::size_t f = 0; f < world.frames.size(); ++f) {
            const double t = world.frames[f].t;
            for (; k < world.imu.size() && world.imu[k].timestamp_s <= t; ++k) {
                const double dt = world.imu[k].timestamp_s - s.timestamp;
                const auto& m = world.imu[k];
                st::s2_propagation::apply(
                    s,
                    prop,
                    {{m.angular_velocity[0], m.angular_velocity[1], m.angular_velocity[2]}},
                    {{m.linear_acceleration[0], m.linear_acceleration[1], m.linear_acceleration[2]}},
                    dt);
                s.timestamp = world.imu[k].timestamp_s;
            }
            if (s.clones.size() >= max_clones) {
                if (f == capture_at_frame) {
                    seq.input = {s, 0};
                    seq.output = {s, {}};
                    seq.output.diag = st::s9_marginalization::apply(seq.output.state, 0);
                    return seq;
                }
                st::s9_marginalization::apply(s, 0);
            }
            if (inject_truth) {
                s.R = world.gt[f].R;
                s.p = world.gt[f].p;
                s.v = world.gt[f].v;
            }
            st::s3_augmentation::apply(s, t);
        }
        throw std::logic_error("s9 bench: capture frame beyond the synthetic run");
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S9_MARGINALIZATION_BENCH_HPP
