// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s2_propagation_bench.hpp — the S2_propagation stage bench
// (issue #455, epic #444).
//
// Stage under test: stages::s2_propagation::apply(state, propagator, ω̃, ã, Δt),
// run over a segment of IMU samples. Contract (#445 §B, S2), checked at every
// step against the exact Φ and Q_d the filter uses (Propagator::transition):
//   • R stays in SO(3), its quaternion unit-norm;
//   • Q_d ⪰ 0;
//   • P' = Φ P Φᵀ + Q_d, and P' stays symmetric and ⪰ 0;
//   • Φ preserves the global-translation directions of the unobservable
//     subspace exactly; the yaw direction's leak (first-order Φ, no FEJ) is
//     reported, the body-frame filter's known O(Δt²) leak;
//   • the timestamp advances by ΣΔt.
//
// Fixtures:
//   known_answer   level and stationary (specific force cancels gravity): the
//                  mean is unchanged and P follows a hand-built Φ in closed form
//   ground_truth   an analytic tumbling, accelerating trajectory with biases:
//                  truth = the final pose and velocity
//   captured       an IMU segment of the synthetic world's S2/S3 stage sequence
//
// Variants: "shipped" (first-order Φ, the diagonal Q_d), "canonical_qd" (same Φ,
// the canonical B·Σ·Bᵀ Q_d with the position and v–p terms the diagonal drops),
// "sqrt_covariance" (the square-root covariance form of the same propagation).
//
// Sweep: Δt × trajectory dynamics × Q scale → position error vs the analytic
// truth, yaw leak, the diagonal-vs-canonical position-σ gap, and λ_min/‖P‖.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S2_PROPAGATION_BENCH_HPP
#define BRANES_TOOLS_BENCH_S2_PROPAGATION_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/eval/propagation_probe.hpp>  // pp_detail::qd_canonical
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/sqrt_covariance.hpp>
#include <branes/sdk/msckf/stages/s2_propagation.hpp>
#include <branes/sdk/msckf/stages/s3_augmentation.hpp>
#include <branes/tools/bench/bench.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S2PropagationBench {
    static constexpr std::string_view kStage = "S2_propagation";
    static constexpr inv::Stage kInvStage = inv::Stage::S2_propagation;
    /// A segment is dozens of dense steps; in software arithmetic one timed run is enough.
    static constexpr int kTimingReps = 1;

    template <class T>
    using State = sdk::msckf::State<T>;
    template <class T>
    using Vec3 = math::lie::detail::Vec<T, 3>;
    template <class T>
    using DynMat = sdk::msckf::DynMat<T>;

    template <class T>
    struct Sample {
        T dt{0};
        Vec3<T> gyro{}, accel{};
    };
    template <class T>
    struct Input {
        State<T> state{T{1}};
        std::vector<Sample<T>> samples;
        sdk::msckf::ImuNoise<T> noise{};
        Vec3<T> gravity{{T{0}, T{0}, T{-981} / T{100}}};
    };
    /// What one step looked like, for the per-step invariants (not serialized).
    template <class T>
    struct Step {
        DynMat<T> P_before, F, Q, P_after;
        DynMat<T> N_before, N_after;  ///< unobservable-subspace bases at both ends
    };
    template <class T>
    struct Output {
        State<T> state{T{1}};
        std::vector<Step<T>> steps;
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "stages::s2_propagation::apply (first-order Phi, diagonal Q_d)"},
                {"canonical_qd", "same Phi, canonical B*Sigma*B^T Q_d (position and v-p terms)"},
                {"sqrt_covariance", "the square-root covariance form of the same propagation"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        json s = json::array();
        for (const auto& x : in.samples)
            s.push_back({{"dt", pack(x.dt)}, {"gyro", pack(x.gyro)}, {"accel", pack(x.accel)}});
        return json{{"state", pack(in.state)},
                    {"samples", s},
                    {"noise",
                     {{"gyro", pack(in.noise.gyro)},
                      {"accel", pack(in.noise.accel)},
                      {"gyro_bias", pack(in.noise.gyro_bias)},
                      {"accel_bias", pack(in.noise.accel_bias)}}},
                    {"gravity", pack(in.gravity)}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        Input<T> in;
        in.state = unpack_state<T>(j.at("state"));
        for (const auto& x : j.at("samples"))
            in.samples.push_back(
                {unpack_scalar<T>(x.at("dt")), unpack_fixed<T, 3>(x.at("gyro")), unpack_fixed<T, 3>(x.at("accel"))});
        const auto& n = j.at("noise");
        in.noise = {unpack_scalar<T>(n.at("gyro")),
                    unpack_scalar<T>(n.at("accel")),
                    unpack_scalar<T>(n.at("gyro_bias")),
                    unpack_scalar<T>(n.at("accel_bias"))};
        in.gravity = unpack_fixed<T, 3>(j.at("gravity"));
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"state", pack(out.state)}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        return Output<T>{unpack_state<T>(j.at("state")), {}};
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        const sdk::msckf::Propagator<T> prop(in.noise, in.gravity);
        Output<T> out{in.state, {}};
        auto& s = out.state;
        if (variant == "sqrt_covariance")
            return run_sqrt(in, prop);
        for (const auto& x : in.samples) {
            if (!(x.dt > T{0}))
                continue;
            Step<T> st;
            st.P_before = s.cov.P;
            st.N_before = unobservable_basis(s);
            const auto tr = prop.transition(s, x.gyro, x.accel, x.dt);
            st.F = tr.F;
            st.Q = dense_q(s.dim(), tr.q);
            if (variant == "canonical_qd") {
                State<T> mean = s;
                prop.propagate(mean, x.gyro, x.accel, x.dt);  // the mean is the shipped one
                st.Q = canonical_q(s.dim(), in.noise, x.dt);
                DynMat<T> P =
                    sdk::msckf::add(sdk::msckf::mul(sdk::msckf::mul(tr.F, s.cov.P), sdk::msckf::transpose(tr.F)), st.Q);
                sdk::msckf::symmetrize(P);
                mean.cov.P = std::move(P);
                s = std::move(mean);
            } else {
                sdk::msckf::stages::s2_propagation::apply(s, prop, x.gyro, x.accel, x.dt);
            }
            st.P_after = s.cov.P;
            st.N_after = unobservable_basis(s);
            out.steps.push_back(std::move(st));
        }
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
        return v;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        const auto& s = out.state;
        // The rotation entered T from a double-held fixture: never hold it
        // tighter than double's ε (matters for long double).
        const double safety = safety_vs_double<T>();
        for (auto& x : inv::check_so3<T>(s.R, kInvStage, "attitude.orthogonality", "attitude.determinant", safety))
            r.push_back(x);
        r.push_back(inv::check_unit_quaternion<T>(
            std::span<const T, 4>(s.R.quaternion().e), kInvStage, "attitude.unit_quaternion", safety));

        // Definiteness needs an eigensolve, which is expensive in software
        // arithmetic, so it is checked where it matters: Q_d at the first step
        // (its structure is the same every step) and P at the end of the
        // segment. The product-based identities are checked at every step, each
        // reported at its tightest step.
        if (!out.steps.empty()) {
            r.push_back(inv::check_psd(out.steps.front().Q, kInvStage, "Qd.psd"));
            r.push_back(inv::check_symmetric(out.steps.back().P_after, kInvStage, "covariance.symmetric"));
            r.push_back(inv::check_psd(out.steps.back().P_after, kInvStage, "covariance.psd"));
        }
        Worst ident, trans, yaw;
        for (const auto& st : out.steps) {
            ident.take(inv::check_covariance_propagation(
                st.P_before, st.F, st.Q, st.P_after, kInvStage, "covariance.propagation_identity"));
            trans.take(inv::check_subspace_preserved(st.F,
                                                     columns(st.N_before, 0, 3),
                                                     columns(st.N_after, 0, 3),
                                                     kInvStage,
                                                     "observability.translation_preserved"));
            auto y = inv::check_subspace_preserved(st.F, st.N_before, st.N_after, kInvStage, "observability.yaw_leak");
            y.bound = inv::Bound::Report;  // the body-frame filter's known first-order leak: reported, not gated
            y.pass = std::isfinite(y.value);
            yaw.take(y);
        }
        for (auto* w : {&ident, &trans, &yaw})
            if (w->set)
                r.push_back(w->result);

        double dt_sum = 0.0;
        for (const auto& x : in.samples)
            if (x.dt > T{0})
                dt_sum += static_cast<double>(x.dt);
        r.push_back(
            inv::check_scalar(std::abs(s.timestamp - (in.state.timestamp + dt_sum)),
                              inv::arithmetic_tolerance<double>(in.samples.size() + 1, 1.0 + std::abs(s.timestamp)),
                              inv::Bound::Upper,
                              kInvStage,
                              "timestamp.advanced_by_dt",
                              "s"));
        r.push_back(inv::check_dimension(s.dim(), in.state.dim(), kInvStage, "state.dimension_unchanged"));
        if (f.kind == FixtureKind::GroundTruth)
            for (auto& x : truth_checks<T>(s, f.truth, in.samples.size()))
                r.push_back(x);
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_stationary_level", known_answer()},
                {"ground_truth_tumbling_accel", ground_truth()},
                {"captured_synthetic_segment", captured()}};
    }

    /// Level and stationary (the accelerometer reads +g, gyro zero, no biases),
    /// with one clone: the mean is exactly unchanged and P follows a Φ built by
    /// hand from the error-state model, independently of the propagator.
    [[nodiscard]] static Fixture known_answer() {
        Input<double> in;
        in.state = State<double>(0.1);
        in.state.p = {{1.0, 2.0, 3.0}};
        in.state.timestamp = 10.0;
        sdk::msckf::stages::s3_augmentation::apply(in.state, 10.0);
        in.state.cov.P = correlated_spd(in.state.dim());
        for (int k = 0; k < 10; ++k)
            in.samples.push_back({0.005, {{0.0, 0.0, 0.0}}, {{0.0, 0.0, 9.81}}});

        // Hand-built Φ for ω = 0, a = (0,0,g), R = I: F[p,v] = I·dt,
        // F[v,θ] = −[a]× dt, F[θ,bg] = −I·dt, F[v,ba] = −I·dt; Q diagonal.
        const auto& n = in.noise;
        Output<double> want{in.state, {}};
        auto& P = want.state.cov.P;
        const std::size_t d = P.rows;
        for (const auto& x : in.samples) {
            DynMat<double> F = DynMat<double>::identity(d);
            const double dt = x.dt, g = 9.81;
            for (std::size_t i = 0; i < 3; ++i) {
                F(3 + i, 6 + i) += dt;    // p ← v
                F(0 + i, 9 + i) += -dt;   // θ ← bg
                F(6 + i, 12 + i) += -dt;  // v ← ba
            }
            // −[a]× dt with a = (0,0,g): [a]× = [[0,−g,0],[g,0,0],[0,0,0]].
            F(6, 1) += g * dt;
            F(7, 0) += -g * dt;
            DynMat<double> Pn = sdk::msckf::mul(sdk::msckf::mul(F, P), sdk::msckf::transpose(F));
            for (std::size_t i = 0; i < 3; ++i) {
                Pn(0 + i, 0 + i) += n.gyro * n.gyro * dt;
                Pn(6 + i, 6 + i) += n.accel * n.accel * dt;
                Pn(9 + i, 9 + i) += n.gyro_bias * n.gyro_bias * dt;
                Pn(12 + i, 12 + i) += n.accel_bias * n.accel_bias * dt;
            }
            sdk::msckf::symmetrize(Pn);
            P = Pn;
            want.state.timestamp += dt;
        }
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.variant = "shipped";  // the canonical-Q and sqrt variants propagate a different P
        f.source = "s2_propagation_bench: level, stationary, 10 x 5 ms, one clone";
        f.description = "mean unchanged; covariance by a hand-built Phi and the diagonal Q_d";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        return f;
    }

    /// An analytic trajectory: constant body rate ω, constant world acceleration
    /// a_w, true biases in the IMU. Initial state = truth; truth = the final pose.
    [[nodiscard]] static Fixture ground_truth() {
        const auto traj = trajectory(/*dynamics=*/1, 0.005, 30);
        json truth{{"R", pack(traj.R_end)}, {"p", pack(traj.p_end)}, {"v", pack(traj.v_end)}};
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "s2_propagation_bench: analytic tumbling + accelerating trajectory, 30 x 5 ms, biased IMU";
        f.description = "truth: final attitude, position, velocity";
        f.input = encode_input(traj.input);
        f.truth = truth;
        return f;
    }

    /// The IMU segment between camera frames 3 and 4 of the synthetic world's
    /// S2/S3 stage sequence, recorded with the state the stage produced.
    [[nodiscard]] static Fixture captured() {
        namespace st = sdk::msckf::stages;
        const auto world = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
        const sdk::msckf::Propagator<double> prop({}, {{0.0, 0.0, -9.81}});
        State<double> s(0.1);
        s.R = world.gt.front().R;
        s.p = world.gt.front().p;
        s.v = world.gt.front().v;
        s.timestamp = world.imu.front().timestamp_s;
        std::size_t k = 0;
        auto imu_to = [&](double t, Input<double>* record) {
            for (; k < world.imu.size() && world.imu[k].timestamp_s <= t; ++k) {
                const auto& m = world.imu[k];
                const Sample<double> x{
                    m.timestamp_s - s.timestamp,
                    {{m.angular_velocity[0], m.angular_velocity[1], m.angular_velocity[2]}},
                    {{m.linear_acceleration[0], m.linear_acceleration[1], m.linear_acceleration[2]}}};
                if (record)
                    record->samples.push_back(x);
                st::s2_propagation::apply(s, prop, x.gyro, x.accel, x.dt);
                s.timestamp = m.timestamp_s;
            }
        };
        for (std::size_t fr = 0; fr < 3; ++fr) {
            imu_to(world.frames[fr].t, nullptr);
            st::s3_augmentation::apply(s, world.frames[fr].t);
        }
        Input<double> in;
        in.state = s;
        in.gravity = {{0.0, 0.0, -9.81}};
        State<double> replay = s;
        imu_to(world.frames[3].t, &in);
        // The recorded output is what the stage produced on exactly these samples.
        Output<double> got{replay, {}};
        for (const auto& x : in.samples)
            st::s2_propagation::apply(got.state, prop, x.gyro, x.accel, x.dt);
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config), S2/S3 stage sequence, IMU between frames 3 and 4",
                       encode_input(in),
                       encode_output(got));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("dt", {0.0025, 0.005, 0.01, 0.02}).axis("dynamics", {0, 1, 2}).axis("q_scale", {0.5, 1.0, 2.0});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"pos_err_m", "yaw_leak", "pos_sigma_gap_pct", "lambda_min_rel"};
    }
    /// 0.5 s of the analytic trajectory at `dynamics` (0 static, 1 tumbling, 2
    /// aggressive) sampled every `dt`, with the IMU noise densities × q_scale.
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double dt = p.at("dt"), dyn = p.at("dynamics"), qs = p.at("q_scale");
        // Lower bound: each step keeps six matrices for the per-step checks, so
        // 0.5 s at dt = 1e-4 (5000 steps) is the most a sweep point may allocate.
        if (!std::isfinite(dt) || dt < 1e-4 || dt > 0.1)
            throw std::invalid_argument("s2 bench sweep: dt must be in [1e-4, 0.1]");
        if (dyn != 0.0 && dyn != 1.0 && dyn != 2.0)
            throw std::invalid_argument("s2 bench sweep: dynamics must be 0, 1 or 2");
        if (!std::isfinite(qs) || qs <= 0.0 || qs > 100.0)
            throw std::invalid_argument("s2 bench sweep: q_scale must be in (0, 100]");
        auto traj = trajectory(static_cast<int>(dyn), dt, static_cast<std::size_t>(std::lround(0.5 / dt)));
        auto& n = traj.input.noise;
        n = {n.gyro * qs, n.accel * qs, n.gyro_bias * qs, n.accel_bias * qs};
        const auto in = decode_input<T>(encode_input(traj.input));
        const auto shipped = run<T>(in, kShipped);
        const auto canon = run<T>(in, "canonical_qd");
        double pos_err = 0.0, leak = 0.0;
        for (std::size_t i = 0; i < 3; ++i)
            pos_err = std::max(pos_err, std::abs(static_cast<double>(shipped.state.p[i]) - traj.p_end[i]));
        for (const auto& st : shipped.steps)
            leak = std::max(leak, inv::check_subspace_preserved(st.F, st.N_before, st.N_after).value);
        auto pos_sigma = [](const DynMat<T>& P) {
            double tr = 0.0;
            for (std::size_t i = 3; i < 6; ++i)
                tr += static_cast<double>(P(i, i));
            return std::sqrt(tr);
        };
        const double sd = pos_sigma(shipped.state.cov.P), sc = pos_sigma(canon.state.cov.P);
        const auto ev = inv::la::symmetric_eigenvalues(shipped.state.cov.P);
        return {pos_err,
                leak,
                sc > 0.0 ? (sc - sd) / sc * 100.0 : 0.0,
                static_cast<double>(ev.front()) /
                    std::max(1e-300, static_cast<double>(inv::la::max_abs(shipped.state.cov.P)))};
    }

    // ── Helpers (also used by the S3 bench) ─────────────────────────────────
    /// A dense SPD matrix B·Bᵀ + ridge·I with a fixed, rank-deficient B.
    [[nodiscard]] static DynMat<double> correlated_spd(std::size_t n, double ridge = 1e-4) {
        const std::size_t k = n / 2 + 1;
        DynMat<double> B(n, k);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < k; ++j)
                B(i, j) = 0.05 * std::sin(0.7 * double(i + 1) + 1.3 * double(j + 1));
        auto P = sdk::msckf::mul(B, sdk::msckf::transpose(B));
        for (std::size_t i = 0; i < n; ++i)
            P(i, i) += ridge;
        sdk::msckf::symmetrize(P);
        return P;
    }

    /// The 4-D unobservable subspace of the body-frame error state at `s`:
    /// columns 0–2 global translation (δp = I on the IMU and every clone),
    /// column 3 global yaw about world z (δθ = Rᵀẑ, δp = ẑ×p, δv = ẑ×v; per clone
    /// δθ_c = R_cᵀẑ, δp_c = ẑ×p_c). Calibration and bias blocks are zero.
    template <class T>
    [[nodiscard]] static DynMat<T> unobservable_basis(const State<T>& s) {
        DynMat<T> N(s.dim(), 4);
        const Vec3<T> z{{T{0}, T{0}, T{1}}};
        auto zx = [&](const Vec3<T>& a) { return math::lie::detail::cross(z, a); };
        auto put = [&](std::size_t row, std::size_t col, const Vec3<T>& v) {
            for (std::size_t i = 0; i < 3; ++i)
                N(row + i, col) = v[i];
        };
        for (std::size_t i = 0; i < 3; ++i)
            N(State<T>::kPos + i, i) = T{1};
        put(State<T>::kTheta, 3, s.R.inverse() * z);
        put(State<T>::kPos, 3, zx(s.p));
        put(State<T>::kVel, 3, zx(s.v));
        for (std::size_t c = 0; c < s.clones.size(); ++c) {
            const std::size_t off = s.clone_offset(c);
            for (std::size_t i = 0; i < 3; ++i)
                N(off + 3 + i, i) = T{1};
            put(off, 3, s.clones[c].R.inverse() * z);
            put(off + 3, 3, zx(s.clones[c].p));
        }
        return N;
    }

private:
    /// Keeps the result closest to (or furthest past) its threshold.
    struct Worst {
        inv::InvariantResult result{};
        bool set = false;
        void take(const inv::InvariantResult& r) {
            const bool worse = !r.pass && result.pass;
            if (!set || worse || (r.pass == result.pass && r.margin() > result.margin()))
                result = r;
            set = true;
        }
    };

    template <class T>
    [[nodiscard]] static DynMat<T> columns(const DynMat<T>& m, std::size_t c0, std::size_t c1) {
        DynMat<T> out(m.rows, c1 - c0);
        for (std::size_t i = 0; i < m.rows; ++i)
            for (std::size_t j = c0; j < c1; ++j)
                out(i, j - c0) = m(i, j);
        return out;
    }

    template <class T>
    [[nodiscard]] static DynMat<T> dense_q(std::size_t d, const std::array<sdk::msckf::NoiseTerm<T>, 12>& q) {
        DynMat<T> Q(d, d);
        for (const auto& [i, var] : q)
            Q(i, i) += var;
        return Q;
    }

    template <class T>
    [[nodiscard]] static DynMat<T> canonical_q(std::size_t d, const sdk::msckf::ImuNoise<T>& n, T dt) {
        const DynMat<T> Qi = sdk::eval::pp_detail::qd_canonical<T>(n, dt);
        DynMat<T> Q(d, d);
        for (std::size_t i = 0; i < Qi.rows; ++i)
            for (std::size_t j = 0; j < Qi.cols; ++j)
                Q(i, j) = Qi(i, j);
        return Q;
    }

    /// The same propagation in square-root form: S = chol(P), propagate the
    /// factor through the stage, report P = S Sᵀ.
    template <class T>
    [[nodiscard]] static Output<T> run_sqrt(const Input<T>& in, const sdk::msckf::Propagator<T>& prop) {
        Output<T> out{in.state, {}};
        sdk::msckf::State<T, sdk::msckf::SqrtCovariance<T>> sq(T{1});
        sq.R = in.state.R;
        sq.p = in.state.p;
        sq.v = in.state.v;
        sq.bg = in.state.bg;
        sq.ba = in.state.ba;
        sq.timestamp = in.state.timestamp;
        sq.calib = {};
        for (const auto& c : in.state.calib)
            sq.calib.push_back({c.R_imu_cam, c.p_imu_cam});
        for (const auto& c : in.state.clones)
            sq.clones.push_back({c.R, c.p, c.timestamp});
        DynMat<T> L;
        if (!psd_factor(in.state.cov.P, L)) {
            for (auto& x : out.state.cov.P.d)
                x = T(std::numeric_limits<double>::quiet_NaN());  // not PSD in T: no square-root form
            return out;
        }
        sq.cov.S = L;
        for (const auto& x : in.samples) {
            if (!(x.dt > T{0}))
                continue;
            Step<T> st;
            st.P_before = sq.covariance();
            st.N_before = unobservable_basis(out.state);
            const auto tr = prop.transition(sq, x.gyro, x.accel, x.dt);
            st.F = tr.F;
            st.Q = dense_q(sq.dim(), tr.q);
            sdk::msckf::stages::s2_propagation::apply(sq, prop, x.gyro, x.accel, x.dt);
            out.state.R = sq.R;
            out.state.p = sq.p;
            out.state.v = sq.v;
            out.state.timestamp = sq.timestamp;
            st.P_after = sq.covariance();
            st.N_after = unobservable_basis(out.state);
            out.steps.push_back(std::move(st));
        }
        out.state.cov.P = sq.covariance();
        return out;
    }

    struct Trajectory {
        Input<double> input;
        math::lie::SO3<double> R_end;
        Vec3<double> p_end, v_end;
    };

    /// Constant body rate ω and constant world acceleration a_w from a start
    /// pose, with true biases in the IMU (gyro = ω + b_g, accel = Rᵀ(a_w − g) +
    /// b_a, sampled at the start of each interval). dynamics: 0 static,
    /// 1 tumbling (moderate ω and a), 2 aggressive.
    [[nodiscard]] static Trajectory trajectory(int dynamics, double dt, std::size_t n) {
        using SO3 = math::lie::SO3<double>;
        const Vec3<double> w = dynamics == 0   ? Vec3<double>{}
                               : dynamics == 1 ? Vec3<double>{{0.6, -0.3, 1.0}}
                                               : Vec3<double>{{2.5, -1.5, 3.0}};
        const Vec3<double> a_w = dynamics == 0   ? Vec3<double>{}
                                 : dynamics == 1 ? Vec3<double>{{0.8, -0.4, 0.3}}
                                                 : Vec3<double>{{4.0, -3.0, 2.0}};
        const Vec3<double> g{{0.0, 0.0, -9.81}}, bg{{0.004, -0.002, 0.003}}, ba{{0.02, 0.01, -0.015}};
        const SO3 R0 = SO3::exp({{0.1, -0.2, 0.3}});
        const Vec3<double> p0{{1.0, -2.0, 0.5}}, v0{{0.3, 0.1, -0.2}};
        Trajectory tr;
        auto& in = tr.input;
        in.state = State<double>(0.1);
        in.state.R = R0;
        in.state.p = p0;
        in.state.v = v0;
        in.state.bg = bg;
        in.state.ba = ba;
        in.state.timestamp = 0.0;
        in.state.cov.P = correlated_spd(in.state.dim());
        in.gravity = g;
        for (std::size_t k = 0; k < n; ++k) {
            const SO3 R = R0 * SO3::exp(w * (dt * double(k)));
            in.samples.push_back({dt, w + bg, R.inverse() * (a_w - g) + ba});
        }
        const double T_end = dt * double(n);
        tr.R_end = R0 * SO3::exp(w * T_end);
        tr.p_end = p0 + v0 * T_end + a_w * (0.5 * T_end * T_end);
        tr.v_end = v0 + a_w * T_end;
        return tr;
    }

    /// Final pose and velocity against the analytic truth. On this trajectory
    /// the propagation is exact in exact arithmetic: ω is constant, so
    /// R ← R·Exp(ωΔt) is exact, and each accel sample is Rₖᵀ(a_w − g) + b_a, so
    /// Rₖ(ã − b_a) + g = a_w exactly at every step. Any residual is arithmetic,
    /// held to the type's tolerance accumulated over the `steps` steps.
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    truth_checks(const State<T>& s, const json& truth, std::size_t steps) {
        const auto R = unpack_so3<double>(truth.at("R"));
        const auto p = unpack_fixed<double, 3>(truth.at("p"));
        const auto v = unpack_fixed<double, 3>(truth.at("v"));
        double ep = 0.0, ev = 0.0;
        for (std::size_t i = 0; i < 3; ++i) {
            const double dp = std::abs(static_cast<double>(s.p[i]) - p[i]);
            const double dv = std::abs(static_cast<double>(s.v[i]) - v[i]);
            ep = std::isfinite(dp) ? std::max(ep, dp) : inv::detail::kInf;
            ev = std::isfinite(dv) ? std::max(ev, dv) : inv::detail::kInf;
        }
        // Attitude error angle of Rᵀ_true R, via 2·atan2(‖v‖, |w|) of the quaternion.
        const auto& qt = R.quaternion();
        const auto& qa = s.R.quaternion();
        const double a0 = static_cast<double>(qa[0]), a1 = static_cast<double>(qa[1]);
        const double a2 = static_cast<double>(qa[2]), a3 = static_cast<double>(qa[3]);
        const double qw = qt[0] * a0 + qt[1] * a1 + qt[2] * a2 + qt[3] * a3;
        const double qx = qt[0] * a1 - qt[1] * a0 - qt[2] * a3 + qt[3] * a2;
        const double qy = qt[0] * a2 + qt[1] * a3 - qt[2] * a0 - qt[3] * a1;
        const double qz = qt[0] * a3 - qt[1] * a2 + qt[2] * a1 - qt[3] * a0;
        double ang = 2.0 * std::atan2(std::sqrt(qx * qx + qy * qy + qz * qz), std::abs(qw));
        if (!std::isfinite(ang))
            ang = inv::detail::kInf;
        double p_scale = 1.0, v_scale = 1.0;
        for (std::size_t i = 0; i < 3; ++i) {
            p_scale = std::max(p_scale, std::abs(p[i]));
            v_scale = std::max(v_scale, std::abs(v[i]));
        }
        const std::size_t n = std::max<std::size_t>(steps, 1);
        return {
            inv::check_scalar(
                ep, tolerance_vs_double<T>(n, p_scale), inv::Bound::Upper, kInvStage, "truth.position_error", "m"),
            inv::check_scalar(
                ev, tolerance_vs_double<T>(n, v_scale), inv::Bound::Upper, kInvStage, "truth.velocity_error", "m/s"),
            inv::check_scalar(
                ang, tolerance_vs_double<T>(n, 1.0), inv::Bound::Upper, kInvStage, "truth.attitude_error", "rad")};
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S2_PROPAGATION_BENCH_HPP
