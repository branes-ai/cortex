// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s2_propagation_bench.hpp — the S2_propagation stage bench
// (issue #455, epic #444).
//
// Stage under test: stages::s2_propagation::apply(state, propagator, interval),
// run over a segment of IMU intervals, each with the samples at both ends (#616). Contract (#445 §B, S2), checked at
// every step against the exact Φ and Q_d the filter uses (Propagator::transition):
//   • R stays in SO(3), its quaternion unit-norm;
//   • Q_d ⪰ 0;
//   • P' = Φ P Φᵀ + Q_d, and P' stays symmetric and ⪰ 0;
//   • Φ preserves the unobservable subspace, global translation and the yaw
//     about gravity, to roundoff: since #616 it is the exact Jacobian of the
//     RK4 mean, whose preintegration form commutes with both. (The pre-#616
//     first-order Φ leaked yaw at O(Δt²); the sweep still reports that leak.)
//   • the timestamp advances by ΣΔt.
//
// Fixtures:
//   known_answer   level and stationary (specific force cancels gravity): the
//                  mean is unchanged and P follows a hand-built Φ in closed form
//   ground_truth   an analytic tumbling, accelerating trajectory with biases:
//                  truth = the final pose and velocity (attitude to roundoff;
//                  position and velocity to what the camera resolves, #616)
//   captured       an IMU segment of the synthetic world's S2/S3 stage sequence
//
// Variants: "shipped" (the RK4 mean's Jacobian Φ, the diagonal Q_d), "canonical_qd" (same Φ,
// the canonical B·Σ·Bᵀ Q_d with the position and v–p terms the diagonal drops),
// "sqrt_covariance" (the square-root covariance form of the same propagation),
// "higher_order_phi" (the pre-#616 Φ expanded, I + AΔt + ½(AΔt)², on the shipped
// mean, #471's comparison) and "invariant"
// (the R-IEKF's InvariantPropagator: SE₂(3) mean, its own Φ, on the invariant
// error ξ, #471). The invariant variant's per-step checks run in ξ, against the
// invariant unobservable basis; its output P is mapped back to the body-frame
// error at the final mean, so it compares with the other variants'.
//
// Sweep: Δt × trajectory dynamics × Q scale → position error vs the analytic
// truth, yaw leak (the shipped Φ, the pre-#616 first-order, second-order and
// exp(AΔt) Φ, and the invariant
// filter's), the diagonal-vs-canonical position-σ gap, and λ_min/‖P‖.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S2_PROPAGATION_BENCH_HPP
#define BRANES_TOOLS_BENCH_S2_PROPAGATION_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/eval/propagation_probe.hpp>  // pp_detail::qd_canonical
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/invariant_propagator.hpp>
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

    /// What the camera resolves over a clone window (the s2_closure bound of
    /// docs/methodology/specs/euroc-consistency-gate.yaml): a tenth of the
    /// 0.5 px track noise at cam0's fx = 458.654 is 1.09e-4 rad, and that angle
    /// at the nearest landmark depth, 2 m, is 0.22 mm; a velocity error held
    /// over the 0.55 s window must stay under the same 0.22 mm.
    static constexpr double kResolvedRotation = 0.1 * 0.5 / 458.654;
    static constexpr double kResolvedPosition = 2.0 * kResolvedRotation;
    static constexpr double kResolvedVelocity = kResolvedPosition / 0.55;

    template <class T>
    using State = sdk::msckf::State<T>;
    template <class T>
    using Vec3 = math::lie::detail::Vec<T, 3>;
    template <class T>
    using DynMat = sdk::msckf::DynMat<T>;

    /// One IMU interval (#616): the sample at its start, and, unless the input
    /// is held constant over it, the sample at its end. A fixture written
    /// before #616 has no end samples and decodes as held intervals.
    template <class T>
    struct Sample {
        T dt{0};
        Vec3<T> gyro{}, accel{};    ///< the sample at the start of the interval
        Vec3<T> gyro1{}, accel1{};  ///< the sample at its end (when !held)
        bool held = true;
        [[nodiscard]] sdk::msckf::ImuInterval<T> interval() const {
            return held ? sdk::msckf::ImuInterval<T>::held(gyro, accel, dt)
                        : sdk::msckf::ImuInterval<T>{gyro, accel, gyro1, accel1, dt};
        }
        [[nodiscard]] static Sample of(const sdk::msckf::ImuInterval<T>& in) {
            bool same = true;
            for (std::size_t i = 0; i < 3; ++i)
                same = same && in.gyro0[i] == in.gyro1[i] && in.accel0[i] == in.accel1[i];
            return Sample{in.dt, in.gyro0, in.accel0, in.gyro1, in.accel1, same};
        }
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
        DynMat<T> F1;                 ///< the pre-#616 first-order Φ at the same state and mean input, for comparison
    };
    template <class T>
    struct Output {
        State<T> state{T{1}};
        std::vector<Step<T>> steps;
        /// The steps are in the invariant error ξ, whose Φ preserves the whole
        /// unobservable subspace by construction (not serialized).
        bool invariant_error = false;
        /// The steps use a Φ that is not the Jacobian of the mean they propagate
        /// (the higher_order_phi comparison), so the yaw leak is reported, not
        /// gated (not serialized).
        bool phi_leaks_yaw = false;
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {
            {"shipped", "stages::s2_propagation::apply (the RK4 mean's Jacobian Phi, diagonal Q_d)"},
            {"canonical_qd", "same Phi, canonical B*Sigma*B^T Q_d (position and v-p terms)"},
            {"sqrt_covariance", "the square-root covariance form of the same propagation"},
            {"higher_order_phi", "the pre-#616 Phi expanded, I + A dt + (A dt)^2 / 2, diagonal Q_d, the shipped mean"},
            {"invariant", "InvariantPropagator (R-IEKF): SE2(3) mean, its Phi, on the invariant error"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        json s = json::array();
        for (const auto& x : in.samples) {
            json e{{"dt", pack(x.dt)}, {"gyro", pack(x.gyro)}, {"accel", pack(x.accel)}};
            if (!x.held) {
                e["gyro1"] = pack(x.gyro1);
                e["accel1"] = pack(x.accel1);
            }
            s.push_back(std::move(e));
        }
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
        for (const auto& x : j.at("samples")) {
            Sample<T> smp{
                unpack_scalar<T>(x.at("dt")), unpack_fixed<T, 3>(x.at("gyro")), unpack_fixed<T, 3>(x.at("accel"))};
            if (x.contains("gyro1")) {
                smp.gyro1 = unpack_fixed<T, 3>(x.at("gyro1"));
                smp.accel1 = unpack_fixed<T, 3>(x.at("accel1"));
                smp.held = false;
            }
            in.samples.push_back(smp);
        }
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
        return Output<T>{unpack_state<T>(j.at("state")), {}, false};
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        const sdk::msckf::Propagator<T> prop(in.noise, in.gravity);
        Output<T> out{in.state, {}};
        out.phi_leaks_yaw = variant == "higher_order_phi";
        auto& s = out.state;
        if (variant == "sqrt_covariance")
            return run_sqrt(in, prop);
        if (variant == "invariant")
            return run_invariant(in);
        for (const auto& x : in.samples) {
            if (!(x.dt > T{0}))
                continue;
            Step<T> st;
            st.P_before = s.cov.P;
            st.N_before = unobservable_basis(s);
            const auto tr = prop.transition(s, x.interval());
            st.F = tr.F;
            st.Q = dense_q(s.dim(), tr.q);
            const auto iv = x.interval();
            st.F1 = prop.first_order_transition(s, iv.mean_gyro(), iv.mean_accel(), iv.dt).F;
            if (variant == "canonical_qd" || variant == "higher_order_phi") {
                State<T> mean = s;
                prop.propagate(mean, x.interval());  // the mean is the shipped one
                if (variant == "canonical_qd")
                    st.Q = canonical_q(s.dim(), in.noise, x.dt);
                else
                    st.F = second_order(st.F1);  // #471's comparison: the first-order Φ, expanded
                DynMat<T> P =
                    sdk::msckf::add(sdk::msckf::mul(sdk::msckf::mul(st.F, s.cov.P), sdk::msckf::transpose(st.F)), st.Q);
                sdk::msckf::symmetrize(P);
                mean.cov.P = std::move(P);
                s = std::move(mean);
            } else {
                sdk::msckf::stages::s2_propagation::apply(s, prop, x.interval());
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
            if (out.phi_leaks_yaw) {
                y.bound = inv::Bound::Report;  // a comparison Φ off the mean's Jacobian: reported, not gated
                y.pass = std::isfinite(y.value);
            }  // the shipped Φ (the RK4 mean's Jacobian, #616) and the invariant one: held to roundoff
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

        // Hand-built Φ of one RK4 step at rest, ω = 0, a = (0,0,g), R = I (#616):
        // ΔR = I, Δv = a·dt, Δp = a·dt²/2, and the bias columns of the
        // increments in closed form, so
        //   F[θ,bg] = −I·dt,     F[v,θ] = −[a]× dt,     F[v,bg] = [a]× dt²/2, F[v,ba] = −I·dt,
        //   F[p,v]  = I·dt,      F[p,θ] = −[a]× dt²/2,  F[p,bg] = [a]× dt³/6, F[p,ba] = −I·dt²/2;
        // Q diagonal.
        const auto& n = in.noise;
        Output<double> want{in.state, {}};
        auto& P = want.state.cov.P;
        const std::size_t d = P.rows;
        for (const auto& x : in.samples) {
            DynMat<double> F = DynMat<double>::identity(d);
            const double dt = x.dt, g = 9.81;
            for (std::size_t i = 0; i < 3; ++i) {
                F(3 + i, 6 + i) += dt;               // p ← v
                F(0 + i, 9 + i) += -dt;              // θ ← bg
                F(6 + i, 12 + i) += -dt;             // v ← ba
                F(3 + i, 12 + i) += -0.5 * dt * dt;  // p ← ba
            }
            // c·[a]× at (r0, c0), with [a]× = [[0,−g,0],[g,0,0],[0,0,0]] for a = (0,0,g).
            const auto skew = [&](std::size_t r0, std::size_t c0, double c) {
                F(r0 + 0, c0 + 1) += -g * c;
                F(r0 + 1, c0 + 0) += g * c;
            };
            skew(6, 0, -dt);                 // v ← θ
            skew(6, 9, 0.5 * dt * dt);       // v ← bg
            skew(3, 0, -0.5 * dt * dt);      // p ← θ
            skew(3, 9, dt * dt * dt / 6.0);  // p ← bg
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
        f.description = "mean unchanged; covariance by a hand-built RK4-step Phi and the diagonal Q_d";
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
        const auto gyro = [&](std::size_t i) {
            const auto& w = world.imu[i].angular_velocity;
            return Vec3<double>{{w[0], w[1], w[2]}};
        };
        const auto accel = [&](std::size_t i) {
            const auto& a = world.imu[i].linear_acceleration;
            return Vec3<double>{{a[0], a[1], a[2]}};
        };
        // Each interval runs from the previous sample to this one (#616), as the
        // backends integrate it; the first sample only starts the clock.
        auto imu_to = [&](double t, Input<double>* record) {
            for (; k < world.imu.size() && world.imu[k].timestamp_s <= t; ++k) {
                const std::size_t j = k > 0 ? k - 1 : 0;
                const auto x =
                    Sample<double>::of({gyro(j), accel(j), gyro(k), accel(k), world.imu[k].timestamp_s - s.timestamp});
                if (record)
                    record->samples.push_back(x);
                st::s2_propagation::apply(s, prop, x.interval());
                s.timestamp = world.imu[k].timestamp_s;
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
            st::s2_propagation::apply(got.state, prop, x.interval());
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
        return {"pos_err_m",
                "yaw_leak",
                "yaw_leak_1st_order",
                "yaw_leak_2nd_order",
                "yaw_leak_exp",
                "yaw_leak_invariant",
                "pos_err_invariant_m",
                "pos_sigma_gap_pct",
                "lambda_min_rel"};
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
        const auto invar = run<T>(in, "invariant");
        double pos_err = 0.0, pos_err_inv = 0.0, leak = 0.0, leak1 = 0.0, leak2 = 0.0, leak_exp = 0.0, leak_inv = 0.0;
        for (std::size_t i = 0; i < 3; ++i) {
            pos_err = std::max(pos_err, std::abs(static_cast<double>(shipped.state.p[i]) - traj.p_end[i]));
            pos_err_inv = std::max(pos_err_inv, std::abs(static_cast<double>(invar.state.p[i]) - traj.p_end[i]));
        }
        // The same body-frame steps under the shipped Φ (the exact Jacobian of
        // the RK4 mean) and under each expansion of the pre-#616 Φ = exp(AΔt):
        // the mean, and so the basis at both ends, is the shipped one throughout.
        for (const auto& st : shipped.steps) {
            leak = std::max(leak, inv::check_subspace_preserved(st.F, st.N_before, st.N_after).value);
            leak1 = std::max(leak1, inv::check_subspace_preserved(st.F1, st.N_before, st.N_after).value);
            leak2 = std::max(leak2, inv::check_subspace_preserved(second_order(st.F1), st.N_before, st.N_after).value);
            leak_exp =
                std::max(leak_exp, inv::check_subspace_preserved(exponential(st.F1), st.N_before, st.N_after).value);
        }
        for (const auto& st : invar.steps)
            leak_inv = std::max(leak_inv, inv::check_subspace_preserved(st.F, st.N_before, st.N_after).value);
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
                leak1,
                leak2,
                leak_exp,
                leak_inv,
                pos_err_inv,
                sc > 0.0 ? (sc - sd) / sc * 100.0 : 0.0,
                static_cast<double>(ev.front()) /
                    std::max(1e-300, static_cast<double>(inv::la::max_abs(shipped.state.cov.P)))};
    }

    // ── Precision characterization (#450) ───────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<OutputGroup> precision_groups(const Output<T>& out) {
        return state_groups(out.state);
    }
    [[nodiscard]] static std::vector<std::string> conditioning_columns() {
        return {"steps", "dt", "max_gyro_rad_s", "log10_cond_P0"};
    }
    [[nodiscard]] static std::vector<double> conditioning(const Input<double>& in) {
        double dt = 0.0, w = 0.0;
        for (const auto& s : in.samples) {
            dt = std::max(dt, s.dt);
            w = std::max(w, std::sqrt(s.gyro[0] * s.gyro[0] + s.gyro[1] * s.gyro[1] + s.gyro[2] * s.gyro[2]));
        }
        return {static_cast<double>(in.samples.size()), dt, w, log10_cond_sym(in.state.cov.P)};
    }
    /// 0.5 s of the static, tumbling and aggressive trajectories at 400 Hz and
    /// 50 Hz: propagation accumulates rounding over its steps, so the error
    /// grows with the step count as well as with the dynamics.
    [[nodiscard]] static std::vector<NamedFixture> precision_fixtures() {
        std::vector<NamedFixture> out;
        for (const int dyn : {0, 1, 2})
            for (const double dt : {0.0025, 0.02}) {
                const auto traj = trajectory(dyn, dt, static_cast<std::size_t>(std::lround(0.5 / dt)));
                out.push_back(ladder_fixture(kStage,
                                             "ladder_dynamics_" + std::to_string(dyn) + "_dt_" +
                                                 std::to_string(static_cast<int>(std::lround(dt * 1e4))) + "e-4",
                                             encode_input(traj.input),
                                             "0.5 s of trajectory " + std::to_string(dyn)));
            }
        return out;
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

    /// The unobservable subspace in the invariant error ξ (R = Exp(ξθ)·R̂,
    /// v = v̂ + ξθ×v̂ + ξv, p = p̂ + ξθ×p̂ + ξp; per clone the same in θ, p),
    /// laid out at the body-frame indices. It does not depend on the state:
    /// translation is ξp = eᵢ (IMU and every clone), and global yaw is ξθ = ẑ
    /// with the other blocks zero. `to_invariant` maps `unobservable_basis(s)`
    /// onto it.
    template <class T>
    [[nodiscard]] static DynMat<T> invariant_basis(const State<T>& s) {
        DynMat<T> N(s.dim(), 4);
        for (std::size_t i = 0; i < 3; ++i)
            N(State<T>::kPos + i, i) = T{1};
        N(State<T>::kTheta + 2, 3) = T{1};
        for (std::size_t c = 0; c < s.clones.size(); ++c) {
            const std::size_t off = s.clone_offset(c);
            for (std::size_t i = 0; i < 3; ++i)
                N(off + 3 + i, i) = T{1};
            N(off + 2, 3) = T{1};
        }
        return N;
    }

    /// J with ξ = J·δ, from the body-frame error δ (R = R̂·Exp(δθ), p and v
    /// additive) to the invariant error ξ, at the mean `s`: ξθ = R̂δθ,
    /// ξv = δv + [v̂]×R̂δθ, ξp = δp + [p̂]×R̂δθ, per clone ξθ = R̂c·δθc and
    /// ξp = δpc + [p̂c]×R̂c·δθc; biases and calibration unchanged. With
    /// `inverse`, J⁻¹: δθ = R̂ᵀξθ, δv = ξv − [v̂]×ξθ, δp = ξp − [p̂]×ξθ.
    template <class T>
    [[nodiscard]] static DynMat<T> to_invariant(const State<T>& s, bool inverse = false) {
        using math::lie::detail::hat;
        DynMat<T> J = DynMat<T>::identity(s.dim());
        auto put = [&](std::size_t r0, std::size_t c0, const auto& B) {
            for (std::size_t i = 0; i < 3; ++i)
                for (std::size_t j = 0; j < 3; ++j)
                    J(r0 + i, c0 + j) = B(i, j);
        };
        auto block =
            [&](std::size_t th, std::size_t pos, std::size_t vel, const auto& R, const Vec3<T>& p, const Vec3<T>* v) {
                const auto Rm = R.matrix();
                if (inverse) {
                    put(th, th, R.inverse().matrix());
                    put(pos, th, hat(p) * T{-1});
                    if (v)
                        put(vel, th, hat(*v) * T{-1});
                } else {
                    put(th, th, Rm);
                    put(pos, th, hat(p) * Rm);
                    if (v)
                        put(vel, th, hat(*v) * Rm);
                }
            };
        block(State<T>::kTheta, State<T>::kPos, State<T>::kVel, s.R, s.p, &s.v);
        for (std::size_t c = 0; c < s.clones.size(); ++c) {
            const std::size_t off = s.clone_offset(c);
            block(off, off + 3, 0, s.clones[c].R, s.clones[c].p, nullptr);
        }
        return J;
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
            const auto tr = prop.transition(sq, x.interval());
            st.F = tr.F;
            st.Q = dense_q(sq.dim(), tr.q);
            sdk::msckf::stages::s2_propagation::apply(sq, prop, x.interval());
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

    /// Φ₂ = I + M + ½M² from the first-order F = I + M (M = AΔt).
    template <class T>
    [[nodiscard]] static DynMat<T> second_order(const DynMat<T>& F) {
        DynMat<T> M = F;
        for (std::size_t i = 0; i < M.rows; ++i)
            M(i, i) -= T{1};
        const DynMat<T> M2 = sdk::msckf::mul(M, M);
        DynMat<T> out = F;
        for (std::size_t i = 0; i < out.d.size(); ++i)
            out.d[i] += M2.d[i] / T{2};
        return out;
    }

    /// exp(M) from the first-order F = I + M, by its Taylor series to the
    /// working precision (M = AΔt is small, so it converges in a few terms).
    template <class T>
    [[nodiscard]] static DynMat<T> exponential(const DynMat<T>& F) {
        DynMat<T> M = F;
        for (std::size_t i = 0; i < M.rows; ++i)
            M(i, i) -= T{1};
        DynMat<T> out = F, term = M;
        for (int k = 2; k < 30; ++k) {
            term = sdk::msckf::mul(term, M);
            for (auto& x : term.d)
                x /= T(k);
            double big = 0.0;
            for (std::size_t i = 0; i < out.d.size(); ++i) {
                out.d[i] += term.d[i];
                big = std::max(big, std::abs(static_cast<double>(term.d[i])));
            }
            if (!(big > 1e-300) || big < 1e-18)
                break;
        }
        return out;
    }

    /// The R-IEKF propagation (#471): the mean on SE₂(3) by
    /// InvariantPropagator::propagate_mean, P on the invariant error ξ by its Φ
    /// (I + AΔt + ½(AΔt)², a state-independent nav block plus the bias lever
    /// arms) and its diagonal Q. P enters through J (`to_invariant`) and leaves
    /// through J⁻¹ at the final mean. Φ is the identity on the clones and the
    /// calibration, as in the body-frame filter.
    template <class T>
    [[nodiscard]] static Output<T> run_invariant(const Input<T>& in) {
        namespace ms = sdk::msckf;
        using Nav = ms::InvariantNavState<T>;
        const ms::InvariantPropagator<T> prop(in.noise, in.gravity);
        Output<T> out{in.state, {}, /*invariant_error=*/true};
        auto& s = out.state;
        const std::size_t d = s.dim();
        // Invariant order (θ, v, p, bg, ba) → the body-frame indices (θ, p, v, bg, ba).
        std::array<std::size_t, Nav::kDim> at{};
        for (std::size_t i = 0; i < 3; ++i) {
            at[Nav::kTheta + i] = State<T>::kTheta + i;
            at[Nav::kVel + i] = State<T>::kVel + i;
            at[Nav::kPos + i] = State<T>::kPos + i;
            at[Nav::kBg + i] = State<T>::kBg + i;
            at[Nav::kBa + i] = State<T>::kBa + i;
        }
        const DynMat<T> J = to_invariant(s);
        DynMat<T> P = ms::mul(ms::mul(J, s.cov.P), ms::transpose(J));
        ms::symmetrize(P);
        const DynMat<T> N = invariant_basis(s);
        Nav x;
        x.X = typename Nav::SE23(s.R, s.v, s.p);
        x.bg = s.bg;
        x.ba = s.ba;
        for (const auto& smp : in.samples) {
            if (!(smp.dt > T{0}))
                continue;
            Step<T> st;
            st.P_before = P;
            st.N_before = N;
            const DynMat<T> phi = prop.phi(x.X.rotation(), x.X.velocity(), x.X.position(), smp.dt);
            st.F = DynMat<T>::identity(d);
            for (std::size_t i = 0; i < Nav::kDim; ++i)
                for (std::size_t j = 0; j < Nav::kDim; ++j)
                    st.F(at[i], at[j]) = phi(i, j);
            // InvariantPropagator::propagate's Q: diagonal, so the reordering is all it needs.
            st.Q = DynMat<T>(d, d);
            for (std::size_t i = 0; i < 3; ++i) {
                st.Q(State<T>::kTheta + i, State<T>::kTheta + i) = in.noise.gyro * in.noise.gyro * smp.dt;
                st.Q(State<T>::kVel + i, State<T>::kVel + i) = in.noise.accel * in.noise.accel * smp.dt;
                st.Q(State<T>::kBg + i, State<T>::kBg + i) = in.noise.gyro_bias * in.noise.gyro_bias * smp.dt;
                st.Q(State<T>::kBa + i, State<T>::kBa + i) = in.noise.accel_bias * in.noise.accel_bias * smp.dt;
            }
            P = ms::add(ms::mul(ms::mul(st.F, P), ms::transpose(st.F)), st.Q);
            ms::symmetrize(P);
            prop.propagate_mean(x, smp.interval());
            s.timestamp += static_cast<double>(smp.dt);
            st.P_after = P;
            st.N_after = N;
            out.steps.push_back(std::move(st));
        }
        s.R = x.X.rotation();
        s.v = x.X.velocity();
        s.p = x.X.position();
        const DynMat<T> Ji = to_invariant(s, /*inverse=*/true);
        s.cov.P = ms::mul(ms::mul(Ji, P), ms::transpose(Ji));
        ms::symmetrize(s.cov.P);
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
        // Interval k runs from the sample at kΔt to the one at (k+1)Δt (#616).
        const auto sample_accel = [&](std::size_t k) {
            return (R0 * SO3::exp(w * (dt * double(k)))).inverse() * (a_w - g) + ba;
        };
        for (std::size_t k = 0; k < n; ++k)
            in.samples.push_back(Sample<double>::of({w + bg, sample_accel(k), w + bg, sample_accel(k + 1), dt}));
        const double T_end = dt * double(n);
        tr.R_end = R0 * SO3::exp(w * T_end);
        tr.p_end = p0 + v0 * T_end + a_w * (0.5 * T_end * T_end);
        tr.v_end = v0 + a_w * T_end;
        return tr;
    }

    /// Final pose and velocity against the analytic truth. ω is constant, so
    /// the attitude is exact in exact arithmetic (R ← R·Exp(ωΔt), no coning)
    /// and held to the type's tolerance accumulated over the `steps` steps.
    /// The body-frame specific force Rᵀ(a_w − g) is not linear between samples,
    /// so the interpolated input (#616) leaves an O(Δt²) error in velocity and
    /// position (5e-6 m/s here); those are held to what the camera resolves,
    /// not to what RK4 achieves, and the old end-sample hold misses them.
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
        return {inv::check_scalar(ep,
                                  std::max(kResolvedPosition, tolerance_vs_double<T>(n, p_scale)),
                                  inv::Bound::Upper,
                                  kInvStage,
                                  "truth.position_error",
                                  "m"),
                inv::check_scalar(ev,
                                  std::max(kResolvedVelocity, tolerance_vs_double<T>(n, v_scale)),
                                  inv::Bound::Upper,
                                  kInvStage,
                                  "truth.velocity_error",
                                  "m/s"),
                inv::check_scalar(
                    ang, tolerance_vs_double<T>(n, 1.0), inv::Bound::Upper, kInvStage, "truth.attitude_error", "rad")};
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S2_PROPAGATION_BENCH_HPP
