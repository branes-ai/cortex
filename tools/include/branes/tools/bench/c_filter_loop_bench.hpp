// SPDX-License-Identifier: MIT
//
// branes/tools/bench/c_filter_loop_bench.hpp — the composition bench for the
// MSCKF loop (issue #447, epic #444 §C).
//
// The stage benches localize a fault to one transformation. Some failures only
// emerge from the stages running together — over-confidence is a property of
// the cycle. This bench runs the loop itself: a TAPE of stage operations, in the
// order the backend ran them,
//
//   S2 (one IMU step) … S2 → [S6 per track touching the oldest clone → S9] → S3 → S6 per ended track
//
// on a starting state, through the shipped stage functions. At every boundary
// the operation's own stage contract is checked — the stage bench's invariants
// evaluated on the LOOP's input and output for that operation — so a fault is
// attributed to the stage that introduced it. Loop-level invariants cover what
// no single stage owns: the covariance at the end, the information the loop
// gained along the four unobservable directions, NIS, and (ground truth) NEES.
//
// Fixtures:
//   known_answer   constant velocity, exact observations, P₀ = 0 and Q = 0: the
//                  mean follows the motion exactly, P stays 0, NIS is 0
//   ground_truth   a synthetic run's tape over three frames (post-init 16–18), its starting mean
//                  (nav, biases, clones) reset to truth: NEES at the end
//   captured       the same tape from the run's estimated state — the run's
//                  final state and per-update NIS, replayed bit for bit
//
// Variants: "shipped"; "fault_s3_clone_jacobian" (S3's clone cross-covariance
// is perturbed); "fault_s6a_jacobian" (a clone-rotation block of H_x is scaled
// by 1.5). The faults demonstrate attribution: the first failing invariant is
// the injected stage's.
//
// Sweep: cycles → final nav NEES (truth-injected), the unobservable
// information ratio and the mean NIS/dof, from one to sixteen cycles.
//
// Capture from any run: `LoopTape<T>` is a StageTap that records the tape.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_C_FILTER_LOOP_BENCH_HPP
#define BRANES_TOOLS_BENCH_C_FILTER_LOOP_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/eval/nav_consistency.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/stage_tap.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/tools/bench/s2_propagation_bench.hpp>
#include <branes/tools/bench/s3_augmentation_bench.hpp>
#include <branes/tools/bench/s5_triangulation_bench.hpp>
#include <branes/tools/bench/s6a_jacobians_bench.hpp>
#include <branes/tools/bench/s6b_nullspace_projection_bench.hpp>
#include <branes/tools/bench/s6c_compression_bench.hpp>
#include <branes/tools/bench/s6d_gating_bench.hpp>
#include <branes/tools/bench/s6e_ekf_update_bench.hpp>
#include <branes/tools/bench/s9_marginalization_bench.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace branes::tools::bench {

struct CFilterLoopBench {
    static constexpr std::string_view kStage = "C_filter_loop";
    static constexpr inv::Stage kInvStage = inv::Stage::EndToEnd;
    static constexpr int kTimingReps = 1;  // every operation is a full stage run

    enum class OpKind { S2, S3, S6, S9 };

    /// One stage operation of the loop.
    template <class T>
    struct Op {
        OpKind kind = OpKind::S2;
        std::uint64_t frame = 0;
        s6::Vec3<T> gyro{}, accel{};  ///< S2
        T dt{0};                      ///< S2
        double t = 0.0;               ///< S3: the clone's time
        std::uint64_t feature = 0;    ///< S6
        s6::Track<T> track{};         ///< S6 (clone indices into the window at that point)
        std::size_t index = 0;        ///< S9: the clone removed
    };

    template <class T>
    struct Input {
        s6::State<T> state{T{1}};  ///< before the first operation
        sdk::msckf::ImuNoise<T> noise{};
        s6::Vec3<T> gravity{{T{0}, T{0}, T{-981} / T{100}}};
        sdk::msckf::CameraExtrinsics<T> extrinsics{};
        s6::Options<T> options{};
        std::vector<Op<T>> ops;
    };

    /// One S6 update's result.
    struct Update {
        int outcome = 0;  ///< s6_msckf_update::Outcome
        double nis = 0.0;
        std::size_t dof = 0;
    };

    /// The contract check of one operation (evaluated lazily by `invariants`).
    struct Check {
        std::size_t op = 0;
        std::function<std::vector<inv::InvariantResult>()> eval;
    };

    template <class T>
    struct Output {
        s6::State<T> state{T{1}};
        std::vector<Update> updates;
        std::vector<Check> checks;              ///< not serialized
        double leak_max = 0.0, leak_sum = 0.0;  ///< unobservable_leak over applied updates (not serialized)
        std::size_t leak_count = 0;
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "the shipped stage functions in the backend's order"},
                {"fault_s3_clone_jacobian", "S3 with its clone cross-covariance perturbed (an injected fault)"},
                {"fault_s6a_jacobian", "S6a with a clone-rotation block of H_x scaled by 1.5 (an injected fault)"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        json ops = json::array();
        for (const auto& o : in.ops) {
            switch (o.kind) {
            case OpKind::S2:
                ops.push_back({{"op", "S2"},
                               {"frame", o.frame},
                               {"gyro", pack(o.gyro)},
                               {"accel", pack(o.accel)},
                               {"dt", pack(o.dt)}});
                break;
            case OpKind::S3:
                ops.push_back({{"op", "S3"}, {"frame", o.frame}, {"t", pack_num(o.t)}});
                break;
            case OpKind::S6:
                ops.push_back(
                    {{"op", "S6"}, {"frame", o.frame}, {"feature", o.feature}, {"track", s6::encode_track(o.track)}});
                break;
            case OpKind::S9:
                ops.push_back({{"op", "S9"}, {"frame", o.frame}, {"index", o.index}});
                break;
            }
        }
        return json{{"state", pack(in.state)},
                    {"noise",
                     {{"gyro", pack(in.noise.gyro)},
                      {"accel", pack(in.noise.accel)},
                      {"gyro_bias", pack(in.noise.gyro_bias)},
                      {"accel_bias", pack(in.noise.accel_bias)}}},
                    {"gravity", pack(in.gravity)},
                    {"extrinsics",
                     {{"R_imu_cam", pack(in.extrinsics.R_imu_cam)}, {"p_imu_cam", pack(in.extrinsics.p_imu_cam)}}},
                    {"options", s6::encode_options(in.options)},
                    {"ops", ops}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        Input<T> in;
        in.state = unpack_state<T>(j.at("state"));
        const auto& n = j.at("noise");
        in.noise = {unpack_scalar<T>(n.at("gyro")),
                    unpack_scalar<T>(n.at("accel")),
                    unpack_scalar<T>(n.at("gyro_bias")),
                    unpack_scalar<T>(n.at("accel_bias"))};
        in.gravity = unpack_fixed<T, 3>(j.at("gravity"));
        in.extrinsics = {unpack_so3<T>(j.at("extrinsics").at("R_imu_cam")),
                         unpack_fixed<T, 3>(j.at("extrinsics").at("p_imu_cam"))};
        in.options = s6::decode_options<T>(j.at("options"));
        for (const auto& o : j.at("ops")) {
            Op<T> op;
            op.frame = o.at("frame").get<std::uint64_t>();
            const auto kind = o.at("op").get<std::string>();
            if (kind == "S2") {
                op.kind = OpKind::S2;
                op.gyro = unpack_fixed<T, 3>(o.at("gyro"));
                op.accel = unpack_fixed<T, 3>(o.at("accel"));
                op.dt = unpack_scalar<T>(o.at("dt"));
                if (!(op.dt > T{0}))
                    throw std::invalid_argument("c_filter_loop bench: an S2 step needs dt > 0");
            } else if (kind == "S3") {
                op.kind = OpKind::S3;
                op.t = unpack_num(o.at("t"));
            } else if (kind == "S6") {
                op.kind = OpKind::S6;
                op.feature = o.at("feature").get<std::uint64_t>();
                // Clone indices are checked against the window when the op runs.
                op.track = s6::decode_track<T>(o.at("track"), std::numeric_limits<std::size_t>::max());
            } else if (kind == "S9") {
                op.kind = OpKind::S9;
                op.index = o.at("index").get<std::size_t>();
            } else {
                throw std::invalid_argument("c_filter_loop bench: unknown op '" + kind + "'");
            }
            in.ops.push_back(std::move(op));
        }
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        json u = json::array();
        for (const auto& x : out.updates)
            u.push_back(json::array({x.outcome, pack_num(x.nis), x.dof}));
        return json{{"state", pack(out.state)}, {"updates", u}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        Output<T> out;
        out.state = unpack_state<T>(j.at("state"));
        for (const auto& x : j.at("updates"))
            out.updates.push_back({x.at(0).get<int>(), unpack_num(x.at(1)), x.at(2).get<std::size_t>()});
        return out;
    }

    // ── The loop ────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        namespace st = sdk::msckf::stages;
        using Outcome = st::s6_msckf_update::Outcome;
        const sdk::msckf::Propagator<T> prop(in.noise, in.gravity);
        const sdk::msckf::CameraUpdater<T> upd(std::vector<sdk::msckf::CameraExtrinsics<T>>{in.extrinsics}, in.options);
        const bool fault_s3 = variant == "fault_s3_clone_jacobian";
        const bool fault_s6a = variant == "fault_s6a_jacobian";

        Output<T> out;
        auto& s = out.state;
        s = in.state;
        for (std::size_t k = 0; k < in.ops.size(); ++k) {
            const auto& op = in.ops[k];
            switch (op.kind) {
            case OpKind::S2: {
                const auto before = s;
                st::s2_propagation::apply(s, prop, op.gyro, op.accel, op.dt);
                out.checks.push_back({k, [before, after = s, op, noise = in.noise, gravity = in.gravity] {
                                          using B = S2PropagationBench;
                                          typename B::template Input<T> bi;
                                          bi.state = before;
                                          bi.samples.push_back({op.dt, op.gyro, op.accel});
                                          bi.noise = noise;
                                          bi.gravity = gravity;
                                          // The step's Φ and Q_d from the input; the loop's own P after.
                                          auto bo = B::template run<T>(bi, kShipped);
                                          bo.state = after;
                                          if (!bo.steps.empty()) {
                                              bo.steps.front().P_after = after.cov.P;
                                              bo.steps.front().N_after = B::unobservable_basis(after);
                                          }
                                          return B::template invariants<T>(bi, bo, Fixture{});
                                      }});
                break;
            }
            case OpKind::S3: {
                const auto before = s;
                const auto diag = st::s3_augmentation::apply(s, op.t);
                if (fault_s3) {
                    // A wrong clone Jacobian: the new clone's cross-covariance with
                    // the IMU pose is off by 1e-3 (kept symmetric).
                    const std::size_t c = s.clone_offset(s.clones.size() - 1);
                    for (std::size_t i = 0; i < 6; ++i)
                        for (std::size_t j = 0; j < 6; ++j) {
                            s.cov.P(c + i, j) += T(1e-3);
                            s.cov.P(j, c + i) += T(1e-3);
                        }
                }
                out.checks.push_back({k, [before, after = s, diag, t = op.t] {
                                          using B = S3AugmentationBench;
                                          const typename B::template Input<T> bi{before, t};
                                          return B::template invariants<T>(
                                              bi, typename B::template Output<T>{after, diag}, Fixture{});
                                      }});
                break;
            }
            case OpKind::S9: {
                if (op.index >= s.clones.size())
                    throw std::invalid_argument("c_filter_loop bench: S9 index outside the window");
                const auto before = s;
                const auto diag = st::s9_marginalization::apply(s, op.index);
                out.checks.push_back({k, [before, after = s, diag, idx = op.index] {
                                          using B = S9MarginalizationBench;
                                          const typename B::template Input<T> bi{before, idx};
                                          return B::template invariants<T>(
                                              bi, typename B::template Output<T>{after, diag, idx}, Fixture{});
                                      }});
                break;
            }
            case OpKind::S6: {
                for (const auto& o : op.track.observations)
                    if (o.clone_index >= s.clones.size())
                        throw std::invalid_argument("c_filter_loop bench: S6 observation outside the window");
                Update u;
                const auto before = s;
                const auto& opts = in.options;
                const auto tri = st::s5_triangulation::apply(before, upd, op.track);
                out.checks.push_back(
                    {k, [before, op, ext = in.extrinsics, opts, tri] {
                         using B = S5TriangulationBench;
                         const typename B::template Input<T> bi{before, ext, opts, op.track};
                         return B::template invariants<T>(
                             bi,
                             typename B::template Output<T>{tri.ok ? 1 : 0,
                                                            tri.ok ? tri.p_f : s6::Vec3<T>{},
                                                            static_cast<double>(opts.min_parallax_deg)},
                             Fixture{});
                     }});
                u.outcome = static_cast<int>(Outcome::Rejected);
                if (tri.ok) {
                    auto jac = st::s6a_jacobians::apply(before, upd, op.track, tri.p_f);
                    if (fault_s6a && jac.ok) {
                        const std::size_t c = before.clone_offset(op.track.observations.front().clone_index);
                        for (std::size_t i = 0; i < jac.system.rows; ++i)
                            for (std::size_t j = c; j < c + 3; ++j)
                                jac.system.Hx[i * jac.system.cols + j] *= T(3) / T(2);
                    }
                    out.checks.push_back(
                        {k, [before, op, ext = in.extrinsics, opts, p_f = tri.p_f, jac] {
                             using B = S6aJacobiansBench;
                             const typename B::template Input<T> bi{before, ext, opts, op.track, p_f};
                             return B::template invariants<T>(
                                 bi, typename B::template Output<T>{jac.ok ? 1 : 0, jac.system}, Fixture{});
                         }});
                    u.outcome = static_cast<int>(Outcome::BehindCamera);
                    if (jac.ok) {
                        const auto basis = s6::unobservable_basis<T>(before);
                        auto proj = st::s6b_nullspace_projection::apply(jac.system);
                        out.checks.push_back({k, [system = jac.system, basis, proj] {
                                                  using B = S6bNullspaceProjectionBench;
                                                  const typename B::template Input<T> bi{system, basis};
                                                  auto bo = B::template run<T>(bi, kShipped);  // for Nᵀ
                                                  bo.ok = proj.ok ? 1 : 0;
                                                  bo.projected = proj.projected;
                                                  return B::template invariants<T>(bi, bo, Fixture{});
                                              }});
                        u.outcome = static_cast<int>(Outcome::NoNullspace);
                        if (proj.ok) {
                            const auto projected = proj.projected;
                            const auto comp = st::s6c_compression::apply(std::move(proj.projected));
                            out.checks.push_back(
                                {k, [projected, comp] {
                                     using B = S6cCompressionBench;
                                     const typename B::template Input<T> bi{projected};
                                     return B::template invariants<T>(
                                         bi,
                                         typename B::template Output<T>{comp.compressed,
                                                                        comp.rows_in,
                                                                        comp.rows_out,
                                                                        comp.compressed_applied ? 1 : 0},
                                         Fixture{});
                                 }});
                            const auto gate = st::s6d_gating::apply(before, upd, comp.compressed);
                            const std::size_t rows = comp.compressed.H.rows;
                            const double thr = opts.enable_gating
                                                   ? static_cast<double>(opts.chi2_per_dof) * static_cast<double>(rows)
                                                   : 0.0;
                            out.checks.push_back({k, [before, opts, compressed = comp.compressed, gate, thr] {
                                                      using B = S6dGatingBench;
                                                      const typename B::template Input<T> bi{before, opts, compressed};
                                                      return B::template invariants<T>(
                                                          bi,
                                                          typename B::template Output<T>{gate.accepted ? 1 : 0,
                                                                                         gate.nis.value,
                                                                                         gate.nis.dof,
                                                                                         gate.nis.valid ? 1 : 0,
                                                                                         thr},
                                                          Fixture{});
                                                  }});
                            u.nis = static_cast<double>(gate.nis.value);
                            u.dof = gate.nis.dof;
                            u.outcome = static_cast<int>(Outcome::Gated);
                            if (gate.accepted) {
                                const double leak = unobservable_leak(before, comp.compressed);
                                out.leak_max = std::max(out.leak_max, leak);
                                out.leak_sum += leak;
                                ++out.leak_count;
                                const auto dx = st::s6e_ekf_update::apply(s, upd, comp.compressed).dx;
                                out.checks.push_back(
                                    {k,
                                     [before,
                                      opts,
                                      compressed = comp.compressed,
                                      basis,
                                      dx,
                                      p_plus = s.cov.covariance()] {
                                         using B = S6eEkfUpdateBench;
                                         const typename B::template Input<T> bi{before, opts, compressed, basis, {}};
                                         return B::template invariants<T>(
                                             bi, typename B::template Output<T>{dx, p_plus}, Fixture{});
                                     }});
                                u.outcome = static_cast<int>(Outcome::Applied);
                            }
                        }
                    }
                }
                out.updates.push_back(u);
                break;
            }
            }
        }
        return out;
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        const auto& s = out.state;
        std::vector<double> f;
        auto put = [&](const auto& v) {
            for (const auto& x : v)
                f.push_back(static_cast<double>(x));
        };
        put(s.R.quaternion().e);
        put(s.p.e);
        put(s.v.e);
        put(s.bg.e);
        put(s.ba.e);
        for (const auto& c : s.clones) {
            put(c.R.quaternion().e);
            put(c.p.e);
            f.push_back(c.timestamp);
        }
        put(s.cov.P.d);
        for (const auto& u : out.updates) {
            f.push_back(static_cast<double>(u.outcome));
            f.push_back(u.nis);
            f.push_back(static_cast<double>(u.dof));
        }
        return f;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& f) {
        // Every operation's own stage contract, on the loop's input and output for
        // it. Reported once per (stage, invariant): the first failure, else the
        // tightest margin; the first failing operation is reported on its own.
        std::vector<inv::InvariantResult> r;
        std::map<std::pair<int, std::string_view>, std::size_t> slot;
        double first_bad = -1.0;
        std::optional<std::pair<int, std::string_view>> first_key;
        for (const auto& c : out.checks) {
            for (auto x : c.eval()) {
                x.name = qualified(x.stage, x.name);
                const auto key = std::make_pair(static_cast<int>(x.stage), x.name);
                const auto it = slot.find(key);
                if (!x.pass && first_bad < 0.0) {
                    first_bad = static_cast<double>(c.op);
                    first_key = key;
                }
                if (it == slot.end()) {
                    slot.emplace(key, r.size());
                    r.push_back(x);
                } else {
                    auto& kept = r[it->second];
                    if (kept.pass && (!x.pass || x.margin() > kept.margin()))
                        kept = x;
                }
            }
        }
        // The first violation leads the report, so `first_failure()` names the
        // stage that broke first, not the first invariant to break at all.
        if (first_key) {
            const auto at = r.begin() + static_cast<std::ptrdiff_t>(slot.at(*first_key));
            std::rotate(r.begin(), at, at + 1);
        }
        r.push_back(
            inv::check_scalar(first_bad, 0.0, inv::Bound::Report, kInvStage, "loop.first_violating_op", "op index"));

        // The loop: the covariance it ends with, and what it learned where it must not.
        const auto& P = out.state.cov.P;
        double pmax = 0.0;
        for (const T& x : P.d)
            pmax = std::max(pmax, std::abs(static_cast<double>(x)));
        r.push_back(inv::check_symmetric(P, kInvStage, "loop.covariance_symmetric", safety_at<T>(pmax)));
        r.push_back(inv::check_psd(P, kInvStage, "loop.covariance_psd", safety_vs_double<T>()));
        r.push_back(inv::check_scalar(
            out.leak_max, 0.0, inv::Bound::Report, kInvStage, "loop.unobservable_leak_max", "fraction"));
        r.push_back(inv::check_scalar(out.leak_count ? out.leak_sum / static_cast<double>(out.leak_count) : 0.0,
                                      0.0,
                                      inv::Bound::Report,
                                      kInvStage,
                                      "loop.unobservable_leak_mean",
                                      "fraction"));
        double nis = 0.0, dof = 0.0;
        for (const auto& u : out.updates)
            if (u.dof > 0) {
                nis += u.nis;
                dof += static_cast<double>(u.dof);
            }
        r.push_back(inv::check_scalar(
            dof > 0.0 ? nis / dof : 0.0, 0.0, inv::Bound::Report, kInvStage, "loop.nis_per_dof", "per dof"));
        r.push_back(inv::check_scalar(
            static_cast<double>(in.ops.size()), 0.0, inv::Bound::Report, kInvStage, "loop.operations", "count"));

        if (f.kind == FixtureKind::GroundTruth) {
            // NEES of the final nav state (15 dof) against its truth.
            const auto& t = f.truth;
            const sdk::eval::NavSample<double> truth{unpack_so3<double>(t.at("R")),
                                                     unpack_fixed<double, 3>(t.at("p")),
                                                     unpack_fixed<double, 3>(t.at("v")),
                                                     unpack_fixed<double, 3>(t.at("bg")),
                                                     unpack_fixed<double, 3>(t.at("ba"))};
            const auto sd = to_double(out.state);
            const sdk::eval::NavSample<double> est{sd.R, sd.p, sd.v, sd.bg, sd.ba};
            const auto e = sdk::eval::nav_error<double>(est, truth);
            double nees = inv::detail::kInf;
            try {
                nees = static_cast<double>(sdk::eval::nees<double>(e, sdk::eval::core_covariance<double>(sd.cov.P)));
            } catch (const std::domain_error&) {}
            r.push_back(
                inv::check_scalar(nees, kChi2_999_15, inv::Bound::Upper, kInvStage, "truth.nav_nees", "chi2 (15 dof)"));
        }
        return r;
    }

    /// "S6a_jacobians.jacobian.state_vs_fd": an operation's invariant under its
    /// stage's name (the report holds string views, so the names are interned).
    [[nodiscard]] static std::string_view qualified(inv::Stage stage, std::string_view name) {
        static std::set<std::string, std::less<>> names;
        std::string q = std::string(inv::to_string(stage)) + "." + std::string(name);
        return *names.insert(std::move(q)).first;
    }

    /// χ²₀.₉₉₉ with 15 degrees of freedom.
    static constexpr double kChi2_999_15 = 37.697;

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_constant_velocity", known_answer()},
                {"ground_truth_synthetic_three_frames", ground_truth()},
                {"captured_synthetic_three_frames", captured()}};
    }

    /// Constant velocity along x, R = I, identity extrinsics, exact normalized
    /// observations of landmarks 4–6 m ahead (+z), P₀ = 0 and Q = 0: the mean
    /// follows the motion exactly, P stays 0, every residual (so every NIS) is 0.
    [[nodiscard]] static Fixture known_answer() {
        constexpr std::size_t kFrames = 6, kSteps = 10, kWindow = 4;
        const double dt = 0.005, v = 1.0;
        Input<double> in;
        in.noise = {0.0, 0.0, 0.0, 0.0};
        in.gravity = {{0.0, 0.0, -9.81}};
        in.state = s6::State<double>(0.0);
        in.state.v = {{v, 0.0, 0.0}};
        const std::vector<s6::Vec3<double>> landmarks{
            {{0.5, 0.3, 4.0}}, {{-0.4, 0.2, 5.0}}, {{0.2, -0.5, 6.0}}, {{-0.3, -0.2, 4.5}}, {{0.8, 0.1, 5.5}}};
        double t = 0.0;
        std::vector<double> clone_x;  // clone positions along x, window order
        for (std::size_t fr = 0; fr < kFrames; ++fr) {
            if (fr > 0)
                for (std::size_t k = 0; k < kSteps; ++k) {
                    Op<double> op;
                    op.kind = OpKind::S2;
                    op.frame = fr;
                    op.accel = {{0.0, 0.0, 9.81}};
                    op.dt = dt;
                    in.ops.push_back(op);
                }
            t = static_cast<double>(fr) * kSteps * dt;
            if (clone_x.size() == kWindow) {
                for (std::size_t l = 0; l < landmarks.size(); ++l) {
                    Op<double> op;
                    op.kind = OpKind::S6;
                    op.frame = fr;
                    op.feature = l;
                    for (std::size_t c = 0; c < kWindow; ++c) {
                        const auto& L = landmarks[l];
                        op.track.observations.push_back({c, 0, {{(L[0] - clone_x[c]) / L[2], L[1] / L[2]}}});
                    }
                    in.ops.push_back(op);
                }
                Op<double> m;
                m.kind = OpKind::S9;
                m.frame = fr;
                in.ops.push_back(m);
                clone_x.erase(clone_x.begin());
            }
            Op<double> a;
            a.kind = OpKind::S3;
            a.frame = fr;
            a.t = t;
            in.ops.push_back(a);
            clone_x.push_back(v * t);
        }
        // The expected output, from the motion alone.
        Output<double> want;
        auto& s = want.state;
        s = s6::State<double>(0.0);
        s.v = {{v, 0.0, 0.0}};
        s.p = {{v * t, 0.0, 0.0}};
        s.timestamp = t;
        for (const double x : clone_x)
            s.clones.push_back({{}, {{x, 0.0, 0.0}}, x / v});
        s.cov.P = s6::Mat<double>(s.dim(), s.dim());
        for (std::size_t fr = kWindow; fr < kFrames; ++fr)
            for (std::size_t l = 0; l < landmarks.size(); ++l)
                want.updates.push_back(
                    {static_cast<int>(sdk::msckf::stages::s6_msckf_update::Outcome::Applied), 0.0, 2 * kWindow - 3});
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.variant = "shipped";
        f.source = "c_filter_loop_bench: constant velocity, exact observations, P0 = 0, Q = 0";
        f.description = "expected: the mean follows the motion, P stays 0, every NIS is 0";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        return f;
    }

    [[nodiscard]] static Fixture ground_truth() {
        return ground_truth_over(kFirstFrame, kFirstFrame + 2);
    }

    [[nodiscard]] static Fixture captured() {
        const auto& tape = synthetic_tape(kFirstFrame, kFirstFrame + 2);
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config, 0.5 px noise), MSCKF run, post-init frames 16-18",
                       encode_input(tape.input),
                       encode_output(tape.output));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("cycles", {1.0, 2.0, 4.0, 8.0, 16.0});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"nav_nees", "unobservable_leak_max", "nis_per_dof", "operations"};
    }
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double c = p.at("cycles");
        if (!(c >= 1.0 && c <= 24.0) || c != std::floor(c))
            throw std::invalid_argument("c_filter_loop bench sweep: cycles is an integer in [1, 24]");
        const auto f = ground_truth_over(kFirstFrame, kFirstFrame + static_cast<std::uint64_t>(c) - 1);
        const auto in = decode_input<T>(f.input);
        const auto out = run<T>(in, kShipped);
        std::vector<double> row(4, inv::detail::kInf);
        for (const auto& x : invariants<T>(in, out, f)) {
            if (x.name == "truth.nav_nees")
                row[0] = x.value;
            if (x.name == "loop.unobservable_leak_max")
                row[1] = x.value;
            if (x.name == "loop.nis_per_dof")
                row[2] = x.value;
            if (x.name == "loop.operations")
                row[3] = x.value;
        }
        return row;
    }

    // ── Helpers ─────────────────────────────────────────────────────────────

    /// The fraction of an update's information that lies along the four
    /// unobservable directions: ‖H₀ N̂‖²_F / ‖H₀‖²_F, with N̂ the orthonormalized
    /// basis at the update's linearization point. The update adds H₀ᵀR⁻¹H₀ to
    /// the information, so this is the share it puts where nothing is
    /// observable. Zero for a consistent update (H₀ N = 0); in [0, 1].
    template <class T>
    [[nodiscard]] static double unobservable_leak(const s6::State<T>& s, const s6::Projected<T>& h0) {
        const auto sd = to_double(s);
        auto N = s6::unobservable_basis<double>(sd);
        // Modified Gram–Schmidt on the four columns.
        for (std::size_t j = 0; j < N.cols; ++j) {
            for (std::size_t k = 0; k < j; ++k) {
                double d = 0.0;
                for (std::size_t i = 0; i < N.rows; ++i)
                    d += N(i, k) * N(i, j);
                for (std::size_t i = 0; i < N.rows; ++i)
                    N(i, j) -= d * N(i, k);
            }
            double n2 = 0.0;
            for (std::size_t i = 0; i < N.rows; ++i)
                n2 += N(i, j) * N(i, j);
            const double inv = n2 > 0.0 ? 1.0 / std::sqrt(n2) : 0.0;
            for (std::size_t i = 0; i < N.rows; ++i)
                N(i, j) *= inv;
        }
        double along = 0.0, total = 0.0;
        for (std::size_t r = 0; r < h0.H.rows; ++r) {
            for (std::size_t c = 0; c < h0.H.cols; ++c) {
                const double h = static_cast<double>(h0.H(r, c));
                total += h * h;
            }
            for (std::size_t j = 0; j < N.cols; ++j) {
                double hn = 0.0;
                for (std::size_t c = 0; c < h0.H.cols; ++c)
                    hn += static_cast<double>(h0.H(r, c)) * N(c, j);
                along += hn * hn;
            }
        }
        return total > 0.0 ? along / total : 0.0;
    }

    /// The state in double, field by field. (Not through the fixture codec: it
    /// holds a rotation to double's unit norm, which a float state's is not.)
    template <class T>
    [[nodiscard]] static s6::State<double> to_double(const s6::State<T>& s) {
        auto d3 = [](const s6::Vec3<T>& v) {
            return s6::Vec3<double>{{static_cast<double>(v[0]), static_cast<double>(v[1]), static_cast<double>(v[2])}};
        };
        auto rot = [](const math::lie::SO3<T>& r) {
            const auto q = r.quaternion();
            math::lie::detail::Vec<double, 4> qd;
            for (std::size_t i = 0; i < 4; ++i)
                qd[i] = static_cast<double>(q[i]);
            return math::lie::SO3<double>::from_unit_quaternion(qd);
        };
        s6::State<double> d(1.0);
        d.R = rot(s.R);
        d.p = d3(s.p);
        d.v = d3(s.v);
        d.bg = d3(s.bg);
        d.ba = d3(s.ba);
        d.timestamp = s.timestamp;
        for (const auto& c : s.calib)
            d.calib.push_back({rot(c.R_imu_cam), d3(c.p_imu_cam)});
        for (const auto& c : s.clones)
            d.clones.push_back({rot(c.R), d3(c.p), c.timestamp});
        d.cov.P = s6::Mat<double>(s.cov.P.rows, s.cov.P.cols);
        for (std::size_t i = 0; i < s.cov.P.d.size(); ++i)
            d.cov.P.d[i] = static_cast<double>(s.cov.P.d[i]);
        return d;
    }

    /// The first post-init frame of the built-in tapes: the window is full and
    /// S9 runs every frame. Frames 13–15 (and 20–22) hold a two-view track whose
    /// rays diverge, which S5's contract still counts as a failure (#487).
    static constexpr std::uint64_t kFirstFrame = 16;

    struct Tape {
        Input<double> input;
        Output<double> output;  ///< the run's final state and updates
    };
    [[nodiscard]] static const Tape& synthetic_tape(std::uint64_t first, std::uint64_t last);
    [[nodiscard]] static Fixture ground_truth_over(std::uint64_t first, std::uint64_t last);
};

/// A StageTap that records the loop over a post-init frame range: the state
/// before its first operation, every S2/S3/S6/S9 operation in order, and the
/// run's state and updates at the end. Set it on an MsckfBackend<T> with
/// `set_stage_tap`. The S6 operations need the single-camera updater.
template <math::Scalar T>
class LoopTape final : public sdk::msckf::StageTap<T, sdk::msckf::FullCovariance<T>> {
public:
    using Base = sdk::msckf::StageTap<T, sdk::msckf::FullCovariance<T>>;
    using St = typename Base::St;
    using Vec3 = typename Base::Vec3;
    using B = CFilterLoopBench;

    LoopTape(std::uint64_t first_frame, std::uint64_t last_frame) : first_(first_frame), last_(last_frame) {
        if (first_frame > last_frame)
            throw std::invalid_argument("loop tape: first_frame > last_frame");
    }

    [[nodiscard]] bool started() const noexcept {
        return started_;
    }
    /// Post-init frames the backend has started (the tape is complete once this passes last_frame + 1).
    [[nodiscard]] std::uint64_t frames_seen() const noexcept {
        return next_frame_;
    }
    [[nodiscard]] const typename B::template Input<T>& input() const noexcept {
        return in_;
    }
    [[nodiscard]] const typename B::template Output<T>& output() const noexcept {
        return out_;
    }
    /// The captured fixture (call once the run has passed `last_frame`).
    [[nodiscard]] Fixture fixture(std::string source) const {
        if (!started_)
            throw std::logic_error("loop tape: no operation was recorded");
        return capture(std::string(B::kStage),
                       std::string(type_name<T>()),
                       std::move(source) + ", post-init frames " + std::to_string(first_) + "-" + std::to_string(last_),
                       B::encode_input(in_),
                       B::encode_output(out_));
    }

    void on_frame(double /*t*/) override {
        frame_ = next_frame_++;
    }
    [[nodiscard]] bool wants(sdk::msckf::TapStage stage) const override {
        using TS = sdk::msckf::TapStage;
        const std::uint64_t f = stage == TS::S2_propagation ? next_frame_ : frame_;
        return stage != TS::S0_sensor_model && f >= first_ && f <= last_;
    }
    void on_s2(const St& before,
               const sdk::msckf::Propagator<T>& prop,
               const Vec3& g,
               const Vec3& a,
               T dt,
               const St& after) override {
        begin(before);
        in_.noise = prop.noise();
        in_.gravity = prop.gravity();
        typename B::template Op<T> op;
        op.kind = B::OpKind::S2;
        op.frame = next_frame_;
        op.gyro = g;
        op.accel = a;
        op.dt = dt;
        in_.ops.push_back(op);
        end(after);
    }
    void on_s3(const St& before,
               double t,
               const sdk::msckf::stages::s3_augmentation::Diagnostics&,
               const St& after) override {
        begin(before);
        typename B::template Op<T> op;
        op.kind = B::OpKind::S3;
        op.frame = frame_;
        op.t = t;
        in_.ops.push_back(op);
        end(after);
    }
    void on_s9(const St& before,
               std::size_t index,
               const sdk::msckf::stages::s9_marginalization::Diagnostics&,
               const St& after) override {
        begin(before);
        typename B::template Op<T> op;
        op.kind = B::OpKind::S9;
        op.frame = frame_;
        op.index = index;
        in_.ops.push_back(op);
        end(after);
    }
    void on_s6(const St& before,
               const sdk::msckf::CameraUpdater<T>& upd,
               std::uint64_t feature,
               const sdk::msckf::FeatureTrack<T>& track,
               const sdk::msckf::stages::s6_msckf_update::Diagnostics<T>& diag,
               const St& after) override {
        if (upd.cameras().size() != 1)
            throw std::logic_error("loop tape: the composition bench models one camera");
        begin(before);
        in_.extrinsics = upd.cameras().front();
        in_.options = upd.options();
        typename B::template Op<T> op;
        op.kind = B::OpKind::S6;
        op.frame = frame_;
        op.feature = feature;
        op.track = track;
        in_.ops.push_back(op);
        out_.updates.push_back({static_cast<int>(diag.outcome), static_cast<double>(diag.nis.value), diag.nis.dof});
        end(after);
    }

private:
    // The state before the first operation. (Between S2 steps the backend sets
    // the nav time to the sample's stamp; the loop's S2 adds dt instead. Only
    // that field differs, and the bench does not compare it.)
    void begin(const St& before) {
        if (!started_) {
            in_.state = before;
            started_ = true;
        }
    }
    void end(const St& after) {
        out_.state = after;
    }

    std::uint64_t first_, last_;
    std::uint64_t next_frame_ = 0, frame_ = 0;
    bool started_ = false;
    typename B::template Input<T> in_;
    typename B::template Output<T> out_;
};

/// The loop of a synthetic MSCKF run (default world, 0.5 px pixel noise from a
/// portable draw) over post-init frames [first, last]. Cached per range.
inline const CFilterLoopBench::Tape& CFilterLoopBench::synthetic_tape(std::uint64_t first, std::uint64_t last) {
    static std::map<std::pair<std::uint64_t, std::uint64_t>, Tape> cache;
    if (const auto it = cache.find({first, last}); it != cache.end())
        return it->second;
    const auto w = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
    using Backend = sdk::MsckfBackend<double>;
    Backend::CameraCalibration cal;
    cal.intrinsics = w.camera;
    cal.extrinsics.R_imu_cam = w.R_imu_cam;
    cal.extrinsics.p_imu_cam = w.p_imu_cam;
    Backend be(std::vector<Backend::CameraCalibration>{cal});
    be.initialize(sdk::VioConfig{});
    LoopTape<double> tape(first, last);
    be.set_stage_tap(&tape);
    std::size_t k = 0;
    std::uint32_t draw = 0;
    for (std::size_t f = 0; f < w.frames.size() && tape.frames_seen() <= last + 1; ++f) {
        for (; k < w.imu.size() && w.imu[k].timestamp_s <= w.frames[f].t; ++k)
            be.process_imu(w.imu[k]);
        auto obs = w.frames[f].obs;
        for (auto& o : obs) {
            o.u += 0.5 * s6::pseudo_normal(draw++);
            o.v += 0.5 * s6::pseudo_normal(draw++);
        }
        be.process_camera(w.frames[f].t, std::span<const sdk::FrontendObservation<double>>(obs));
    }
    if (!tape.started() || tape.frames_seen() <= last)
        throw std::logic_error("c_filter_loop bench: the synthetic run did not reach the tape's frames");
    return cache.emplace(std::make_pair(first, last), Tape{tape.input(), tape.output()}).first->second;
}

/// The synthetic tape over [first, last] with its starting mean (nav, biases,
/// clones) reset to truth; truth = the nav state at the tape's end.
inline Fixture CFilterLoopBench::ground_truth_over(std::uint64_t first, std::uint64_t last) {
    const auto& tape = synthetic_tape(first, last);
    const sdk::eval::SyntheticConfig<double> cfg{};
    const auto w = sdk::eval::generate_world<double>(cfg);
    auto gt_at = [&](double t) -> const sdk::eval::GtSample<double>& {
        for (const auto& g : w.gt)
            if (std::abs(g.t - t) < 1e-9)
                return g;
        throw std::logic_error("c_filter_loop bench: no ground truth at t = " + std::to_string(t));
    };
    auto in = tape.input;
    auto& s = in.state;
    const auto& g0 = gt_at(s.timestamp);
    s.R = g0.R;
    s.p = g0.p;
    s.v = g0.v;
    s.bg = cfg.gyro_bias;
    s.ba = cfg.accel_bias;
    for (auto& c : s.clones) {
        const auto& g = gt_at(c.timestamp);
        c.R = g.R;
        c.p = g.p;
    }
    const auto& g1 = gt_at(tape.output.state.timestamp);
    Fixture f;
    f.stage = std::string(kStage);
    f.kind = FixtureKind::GroundTruth;
    f.source = "synthetic_world (default config, 0.5 px noise), MSCKF run, post-init frames " + std::to_string(first) +
               "-" + std::to_string(last) + ", starting mean reset to truth";
    f.seed = cfg.seed;
    f.description = "truth: the nav state at the end of the tape";
    f.input = encode_input(in);
    f.truth = json{{"R", pack(g1.R)},
                   {"p", pack(g1.p)},
                   {"v", pack(g1.v)},
                   {"bg", pack(cfg.gyro_bias)},
                   {"ba", pack(cfg.accel_bias)}};
    return f;
}

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_C_FILTER_LOOP_BENCH_HPP
