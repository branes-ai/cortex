// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s6c_compression_bench.hpp — the S6c_compression stage
// bench (issue #457, epic #444).
//
// Stage under test: stages::s6c_compression::apply(projected, method). The
// shipped update runs one feature at a time, whose projected system (2m − 3
// rows) is shorter than the state, so it compresses nothing (the identity).
// Stacking several features' projected systems makes a tall system; the "qr"
// variant compresses it to at most n + 1 rows. Contract (#445 §B, S6c):
//   • the normal equations are preserved: H_cᵀH_c = HᵀH, H_cᵀr_c = Hᵀr;
//   • the residual does not grow: ‖r_c‖ ≤ ‖r‖;
//   • a compression keeps rows_out ≤ min(rows_in, n + 1) (reported for the
//     identity).
//
// Fixtures:
//   known_answer   a 4×2 system: the identity returns it unchanged (bound to
//                  "shipped"; the QR rows are another basis)
//   ground_truth   the stacked projected systems of the first eight synthetic-world tracks (72 rows > n + 1 = 52)
//                  at the true state
//   captured       the same at the S2/S3 estimated state — recorded
//
// Variants: "shipped" (identity), "qr" (Householder QR of [H | r]).
//
// Sweep: features stacked → rows in, rows out and the normal-matrix error of the
// QR compression.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S6C_COMPRESSION_BENCH_HPP
#define BRANES_TOOLS_BENCH_S6C_COMPRESSION_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/tools/bench/s6_scene.hpp>

#include <algorithm>
#include <cmath>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S6cCompressionBench {
    static constexpr std::string_view kStage = "S6c_compression";
    static constexpr inv::Stage kInvStage = inv::Stage::S6c_compression;

    template <class T>
    struct Input {
        s6::Projected<T> stacked{};
    };
    template <class T>
    struct Output {
        s6::Projected<T> compressed{};
        std::size_t rows_in = 0;
        std::size_t rows_out = 0;
        int applied = 0;
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "no compression (the shipped update is one feature at a time)"},
                {"qr", "Householder QR of [H | r], keeping at most n + 1 rows"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        return json{{"stacked", s6::encode_projected(in.stacked)}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        return Input<T>{s6::decode_projected<T>(j.at("stacked"))};
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"compressed", s6::encode_projected(out.compressed)},
                    {"rows_in", out.rows_in},
                    {"rows_out", out.rows_out},
                    {"applied", out.applied}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        return Output<T>{s6::decode_projected<T>(j.at("compressed")),
                         j.at("rows_in").get<std::size_t>(),
                         j.at("rows_out").get<std::size_t>(),
                         j.at("applied").get<int>()};
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        namespace s6c = sdk::msckf::stages::s6c_compression;
        const auto method = variant == "qr" ? s6c::Method::Qr : s6c::Method::Identity;
        auto r = s6c::apply(in.stacked, method);
        return Output<T>{std::move(r.compressed), r.rows_in, r.rows_out, r.compressed_applied ? 1 : 0};
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        std::vector<double> f{static_cast<double>(out.rows_in), static_cast<double>(out.rows_out)};
        const auto& h = out.compressed.H;
        for (std::size_t i = 0; i < h.rows; ++i)
            for (std::size_t j = 0; j < h.cols; ++j)
                f.push_back(static_cast<double>(h(i, j)));
        for (const T& v : out.compressed.r)
            f.push_back(static_cast<double>(v));
        return f;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& /*f*/) {
        std::vector<inv::InvariantResult> r;
        const auto& h = in.stacked.H;
        // Held to T's precision at the residual's magnitude (the normal right-hand
        // side Hᵀr carries it): zero at the true state.
        double rr = 0.0;
        for (const T& v : in.stacked.r)
            rr += static_cast<double>(v) * static_cast<double>(v);
        const double r_rms = in.stacked.r.empty() ? 1.0 : std::sqrt(rr / static_cast<double>(in.stacked.r.size()));
        for (auto&& x : inv::check_compression(h,
                                               std::span<const T>(in.stacked.r),
                                               out.compressed.H,
                                               std::span<const T>(out.compressed.r),
                                               kInvStage,
                                               safety_at<T>(r_rms)))
            r.push_back(std::move(x));
        // A compression keeps at most n + 1 rows (the residual column included);
        // the identity keeps them all, which is its result, not a violation.
        const double cap = static_cast<double>(std::min(h.rows, h.cols + 1));
        r.push_back(inv::check_scalar(static_cast<double>(out.rows_out),
                                      out.applied ? cap : 0.0,
                                      out.applied ? inv::Bound::Upper : inv::Bound::Report,
                                      kInvStage,
                                      "compression.rows_out",
                                      "rows"));
        r.push_back(inv::check_scalar(
            static_cast<double>(out.rows_in), 0.0, inv::Bound::Report, kInvStage, "compression.rows_in", "rows"));
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_small_system", known_answer()},
                {"ground_truth_stacked_tracks", ground_truth()},
                {"captured_estimated_poses", captured()}};
    }

    [[nodiscard]] static Fixture known_answer() {
        Input<double> in;
        in.stacked.H = s6::Mat<double>(4, 2);
        const double hv[8] = {1, 2, 0, 1, 3, -1, 2, 2};
        for (std::size_t i = 0; i < 4; ++i)
            for (std::size_t j = 0; j < 2; ++j)
                in.stacked.H(i, j) = hv[i * 2 + j];
        in.stacked.r = {0.5, -1.0, 2.0, 0.25};
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.variant = "shipped";
        f.source = "s6c_compression_bench: a 4x2 system";
        f.description = "expected: the identity returns the input";
        f.input = encode_input(in);
        f.expected = encode_output(Output<double>{in.stacked, 4, 4, 0});
        return f;
    }

    [[nodiscard]] static Fixture ground_truth() {
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config), true state, every track's projected system stacked";
        f.seed = sdk::eval::SyntheticConfig<double>{}.seed;
        f.description = "truth: the uncompressed normal equations";
        f.input = encode_input(stack(s6::build_scene(/*true_poses=*/true), 8));
        f.truth = json{{"normal_equations", "preserved"}, {"residual", 0.0}};
        return f;
    }

    [[nodiscard]] static Fixture captured() {
        const auto in = stack(s6::build_scene(/*true_poses=*/false), 8);
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config), S2/S3 estimated state, every track's projected system",
                       encode_input(in),
                       encode_output(run<double>(in, kShipped)));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("features", {1.0, 2.0, 4.0, 8.0, 16.0});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"rows_in", "rows_out_qr", "qr_normal_matrix_err", "qr_normal_rhs_err"};
    }
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double k = p.at("features");
        if (!(k >= 1.0 && k <= 64.0))
            throw std::invalid_argument("s6c bench sweep: features in [1, 64]");
        const auto inT =
            decode_input<T>(encode_input(stack(s6::build_scene(false, 6, 64), static_cast<std::size_t>(k))));
        const auto out = run<T>(inT, "qr");
        double nm = inv::detail::kInf, nr = inv::detail::kInf;
        for (const auto& x : invariants<T>(inT, out, Fixture{})) {
            if (x.name == "compression.normal_matrix")
                nm = x.value;
            if (x.name == "compression.normal_rhs")
                nr = x.value;
        }
        return {static_cast<double>(out.rows_in), static_cast<double>(out.rows_out), nm, nr};
    }

    /// The projected systems of the scene's first `k` tracks, stacked.
    [[nodiscard]] static Input<double> stack(const s6::Scene& sc, std::size_t k) {
        std::vector<s6::Projected<double>> parts;
        std::size_t rows = 0;
        for (std::size_t t = 0; t < std::min(k, sc.tracks.size()); ++t) {
            parts.push_back(s6::boundaries(sc, t).projected);
            rows += parts.back().H.rows;
        }
        Input<double> in;
        const std::size_t n = sc.state.dim();
        in.stacked.H = s6::Mat<double>(rows, n);
        std::size_t at = 0;
        for (const auto& pm : parts) {
            for (std::size_t i = 0; i < pm.H.rows; ++i, ++at) {
                for (std::size_t j = 0; j < n; ++j)
                    in.stacked.H(at, j) = pm.H(i, j);
                in.stacked.r.push_back(pm.r[i]);
            }
        }
        return in;
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S6C_COMPRESSION_BENCH_HPP
