// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s6b_nullspace_projection_bench.hpp — the
// S6b_nullspace_projection stage bench (issue #457, epic #444).
//
// Stage under test: stages::s6b_nullspace_projection::apply(system) — the left
// null space of H_f applied to [H_f | H_x | r], marginalizing the feature:
// H₀ = Nᵀ H_x, r₀ = Nᵀ r with N (2m × 2m−3) spanning the left null space of H_f.
// The stage never forms N; the bench recovers Nᵀ by running the same method on
// [H_f | I | 0]. Contract (#445 §B, S6b):
//   • NᵀN = I (orthonormal rows), Nᵀ H_f = 0, rank Nᵀ = 2m − 3;
//   • H₀ = Nᵀ H_x and r₀ = Nᵀ r (the projection is the one Nᵀ describes);
//   • H₀ annihilates the unobservable directions (global position, yaw) when
//     they come with the input: a camera update at one consistent linearization
//     point carries no information along them.
//
// Fixtures:
//   known_answer   a 3-observation system with H_f = [I₃; 0₃] stacked: the
//                  null space is the last three rows, so H₀ and r₀ are H_x's and
//                  r's last three rows (Householder signs: the identity's
//                  reflectors only negate the first three rows)
//   ground_truth   the synthetic-world track's system at the true state, with
//                  the unobservable basis
//   captured       the same at the S2/S3 estimated state — recorded
//
// Variants: "shipped" (Householder reflectors), "givens" (plane rotations).
//
// Sweep: observations per track × feature depth → orthonormality and
// annihilation error for each method.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S6B_NULLSPACE_PROJECTION_BENCH_HPP
#define BRANES_TOOLS_BENCH_S6B_NULLSPACE_PROJECTION_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/tools/bench/s6_scene.hpp>

#include <algorithm>
#include <cmath>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S6bNullspaceProjectionBench {
    static constexpr std::string_view kStage = "S6b_nullspace_projection";
    static constexpr inv::Stage kInvStage = inv::Stage::S6b_nullspace_projection;

    template <class T>
    struct Input {
        s6::System<T> system{};
        s6::Mat<T> unobservable{};  ///< n×4 unobservable basis, or empty
    };
    template <class T>
    struct Output {
        int ok = 0;
        s6::Projected<T> projected{};
        s6::Mat<T> nt{};  ///< Nᵀ, recovered by projecting [H_f | I | 0]
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "Householder reflectors on the three H_f columns"},
                {"givens", "Givens plane rotations, column by column, bottom-up"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        return json{{"system", s6::encode_system(in.system)}, {"unobservable", pack(in.unobservable)}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        Input<T> in;
        in.system = s6::decode_system<T>(j.at("system"));
        in.unobservable = unpack_mat<T>(j.at("unobservable"));
        if (in.unobservable.rows != 0 && in.unobservable.rows != in.system.cols)
            throw std::invalid_argument("s6b bench: unobservable basis rows != state dimension");
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"ok", out.ok}, {"projected", s6::encode_projected(out.projected)}, {"nt", pack(out.nt)}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        return Output<T>{j.at("ok").get<int>(), s6::decode_projected<T>(j.at("projected")), unpack_mat<T>(j.at("nt"))};
    }

    // ── The stage ───────────────────────────────────────────────────────────
    [[nodiscard]] static sdk::msckf::stages::s6b_nullspace_projection::Method method(std::string_view variant) {
        using M = sdk::msckf::stages::s6b_nullspace_projection::Method;
        return variant == "givens" ? M::Givens : M::Householder;
    }

    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        namespace s6b = sdk::msckf::stages::s6b_nullspace_projection;
        Output<T> out;
        auto r = s6b::apply(in.system, method(variant));
        out.ok = r.ok ? 1 : 0;
        if (!r.ok)
            return out;
        out.projected = std::move(r.projected);
        // Nᵀ: the same method on [H_f | I | 0].
        s6::System<T> id;
        id.rows = in.system.rows;
        id.cols = in.system.rows;
        id.Hf = in.system.Hf;
        id.Hx.assign(id.rows * id.rows, T{0});
        for (std::size_t i = 0; i < id.rows; ++i)
            id.Hx[i * id.rows + i] = T{1};
        id.r.assign(id.rows, T{0});
        const auto n = s6b::apply(id, method(variant));
        if (n.ok)
            out.nt = n.projected.H;
        return out;
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        std::vector<double> f{static_cast<double>(out.ok)};
        const auto& h = out.projected.H;
        for (std::size_t i = 0; i < h.rows; ++i)
            for (std::size_t j = 0; j < h.cols; ++j)
                f.push_back(static_cast<double>(h(i, j)));
        for (const T& v : out.projected.r)
            f.push_back(static_cast<double>(v));
        return f;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& /*f*/) {
        std::vector<inv::InvariantResult> r;
        const auto& sys = in.system;
        r.push_back(inv::check_scalar(out.ok, 1.0, inv::Bound::Lower, kInvStage, "projection.resolved", "flag"));
        if (!out.ok)
            return r;
        const std::size_t k = sys.rows - 3;
        r.push_back(inv::check_scalar(std::abs(static_cast<double>(out.projected.H.rows) - static_cast<double>(k)),
                                      0.0,
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "projection.rows_2m_minus_3",
                                      "rows"));
        const double safety = safety_vs_double<T>();
        r.push_back(inv::check_orthonormal_rows(out.nt, kInvStage, "nullspace.orthonormal_rows", safety));
        r.push_back(inv::check_annihilates(
            out.nt, s6::as_mat(sys.Hf, sys.rows, 3), kInvStage, "nullspace.annihilates_feature", safety));
        r.push_back(inv::check_rank(out.nt, k, kInvStage, "nullspace.rank", safety));

        // H₀ = Nᵀ H_x, r₀ = Nᵀ r: relative, in units of the system's size.
        const auto nth = sdk::msckf::mul(out.nt, s6::as_mat(sys.Hx, sys.rows, sys.cols));
        const auto ntr = sdk::msckf::mul(out.nt, s6::as_col(sys.r));
        const double tol = tolerance_vs_double<T>(sys.rows, 1.0);
        r.push_back(inv::check_scalar(s6::rel_diff(out.projected.H, nth),
                                      tol,
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "projection.matches_nt_hx",
                                      "dimensionless"));
        // r₀ against Nᵀr, absolute (rel_diff floors its denominator at 1 and r
        // is small), held to the arithmetic error of a size-‖r‖ quantity at T's
        // precision for r's magnitude: r is zero at the true state, where a
        // tapered type has fewer fraction bits.
        double rn = 0.0;
        for (const T& v : sys.r)
            rn += static_cast<double>(v) * static_cast<double>(v);
        r.push_back(inv::check_scalar(
            s6::rel_diff(s6::as_col(out.projected.r), ntr),
            tolerance_vs_double<T>(
                sys.rows, std::max(std::sqrt(rn), 1e-30), safety_at<T>(std::sqrt(rn / static_cast<double>(sys.rows)))),
            inv::Bound::Upper,
            kInvStage,
            "projection.matches_nt_r",
            "normalized"));
        if (in.unobservable.rows != 0)
            r.push_back(inv::check_annihilates(
                out.projected.H, in.unobservable, kInvStage, "projected.annihilates_unobservable", safety));
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_identity_feature_block", known_answer()},
                {"ground_truth_synthetic_track", ground_truth()},
                {"captured_estimated_poses", captured()}};
    }

    /// H_f = [I₃; 0₃] (6×3), H_x = 6×2 with distinct entries, r = (1 … 6). The
    /// left null space of H_f is the span of the last three unit rows. The
    /// Householder reflectors for H_f's identity columns are e_c ↦ −e_c on rows
    /// 0–2 only, so rows 3–5 of [H_x | r] pass through unchanged.
    [[nodiscard]] static Fixture known_answer() {
        Input<double> in;
        auto& s = in.system;
        s.rows = 6;
        s.cols = 2;
        s.Hf.assign(18, 0.0);
        for (std::size_t i = 0; i < 3; ++i)
            s.Hf[i * 3 + i] = 1.0;
        s.Hx = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
        s.r = {1, 2, 3, 4, 5, 6};
        Output<double> want;
        want.ok = 1;
        want.projected.H = s6::Mat<double>(3, 2);
        want.projected.r = {4, 5, 6};
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 2; ++j)
                want.projected.H(i, j) = s.Hx[(i + 3) * 2 + j];
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.variant = "shipped";  // the Givens rows are another basis of the same space
        f.source = "s6b_nullspace_projection_bench: H_f = [I3; 0], 6x2 H_x";
        f.description = "expected: H0, r0 = the last three rows of H_x, r";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        return f;
    }

    [[nodiscard]] static Fixture ground_truth() {
        const auto sc = s6::build_scene(/*true_poses=*/true);
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config), true state, exact observations; unobservable basis";
        f.seed = sdk::eval::SyntheticConfig<double>{}.seed;
        f.description = "truth: the projection annihilates the unobservable directions";
        f.input = encode_input(input_of(sc));
        f.truth = json{{"projected_times_unobservable", 0.0}};
        return f;
    }

    [[nodiscard]] static Fixture captured() {
        const auto sc = s6::build_scene(/*true_poses=*/false);
        const auto in = input_of(sc);
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config), S2/S3 estimated state, shipped S5/S6a",
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
        return {"orthonormal_householder",
                "orthonormal_givens",
                "annihilates_feature_householder",
                "annihilates_feature_givens",
                "unobservable_leak_householder",
                "unobservable_leak_givens"};
    }
    /// `observations` clones on a 0.2 m-spaced baseline (yawing 2° per clone), a
    /// feature `depth_m` ahead; each method's NᵀN − I, Nᵀ H_f and H₀·N errors.
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double m = p.at("observations"), depth = p.at("depth_m");
        if (!(m >= 2.0 && m <= 64.0) || !(depth > 0.0 && depth <= 1e4))
            throw std::invalid_argument("s6b bench sweep: observations in [2, 64], depth_m in (0, 1e4]");
        s6::State<double> st(0.1);
        const s6::Vec3<double> F{{0.1, -0.05, depth}};
        s6::Track<double> tr;
        for (std::size_t c = 0; c < static_cast<std::size_t>(m); ++c) {
            sdk::msckf::StateHelper<double>::augment_clone(st);
            auto& cl = st.clones.back();
            cl.R = s6::State<double>::SO3::exp({{0.0, 0.0, 0.035 * static_cast<double>(c)}});
            cl.p = {{0.2 * static_cast<double>(c), 0.0, 0.0}};
            const auto pc = cl.R.inverse() * (F - cl.p);
            tr.observations.push_back({c, 0, {{pc[0] / pc[2], pc[1] / pc[2]}}});
        }
        const sdk::msckf::CameraUpdater<double> upd({sdk::msckf::CameraExtrinsics<double>{}});
        Input<double> in;
        if (!upd.measurement_system(st, tr.observations, F, in.system))
            throw std::logic_error("s6b bench sweep: feature behind a camera");
        in.unobservable = s6::unobservable_basis(st);
        const auto inT = decode_input<T>(encode_input(in));
        std::vector<double> cols(6, inv::detail::kInf);
        for (std::size_t v = 0; v < 2; ++v) {
            const auto rep = invariants<T>(inT, run<T>(inT, v == 0 ? "shipped" : "givens"), Fixture{});
            for (const auto& x : rep) {
                if (x.name == "nullspace.orthonormal_rows")
                    cols[v] = x.value;
                if (x.name == "nullspace.annihilates_feature")
                    cols[2 + v] = x.value;
                if (x.name == "projected.annihilates_unobservable")
                    cols[4 + v] = x.value;
            }
        }
        return cols;
    }

private:
    [[nodiscard]] static Input<double> input_of(const s6::Scene& sc) {
        Input<double> in;
        in.system = s6::boundaries(sc, 0).system;
        in.unobservable = s6::unobservable_basis(sc.state);
        return in;
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S6B_NULLSPACE_PROJECTION_BENCH_HPP
