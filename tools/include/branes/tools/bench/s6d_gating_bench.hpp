// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s6d_gating_bench.hpp — the S6d_gating stage bench (issue
// #457, epic #444).
//
// Stage under test: stages::s6d_gating::apply(state, updater, projected) — the
// innovation covariance S = H P Hᵀ + σ²I, the NIS γ = rᵀS⁻¹r with dof = rows,
// and the Mahalanobis gate. Contract (#445 §B, S6d):
//   • S is symmetric positive-definite (its condition number reported);
//   • γ is valid and equals rᵀS⁻¹r computed directly;
//   • the decision is the gate's: accept ⇔ gating off or γ ≤ threshold;
//   • over many draws r ~ N(0, S), γ/dof averages 1 inside the χ² band (the
//     sweep, and its test).
//
// Fixtures:
//   known_answer   P = 0.01·I₁₅, H selecting δp_x and δp_y, r = (0.2, 0.2),
//                  σ = 0.01: S = 0.0101·I₂, γ = 0.08/0.0101 ≈ 7.92 on 2 dof,
//                  accepted by the shipped gate (5 per dof = 10)
//   ground_truth   the synthetic-world track at the true state, exact
//                  observations: γ ≈ 0
//   captured       the same at the S2/S3 estimated state — recorded
//
// Variants: "shipped" (γ ≤ 5·dof), "chi2_95" (γ ≤ the 95% χ²(dof) quantile,
// Wilson–Hilferty), "gate_off".
//
// Sweep: dof × outlier size → the mean γ/dof of inliers and the acceptance rates
// of inliers and outliers under the shipped and the χ²₉₅ gates.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S6D_GATING_BENCH_HPP
#define BRANES_TOOLS_BENCH_S6D_GATING_BENCH_HPP

#include <branes/sdk/eval/consistency.hpp>
#include <branes/sdk/eval/invariants.hpp>
#include <branes/tools/bench/s6_scene.hpp>

#include <algorithm>
#include <cmath>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S6dGatingBench {
    static constexpr std::string_view kStage = "S6d_gating";
    static constexpr inv::Stage kInvStage = inv::Stage::S6d_gating;

    template <class T>
    struct Input {
        s6::State<T> state{T{1}};
        s6::Options<T> options{};
        s6::Projected<T> projected{};
    };
    template <class T>
    struct Output {
        int accepted = 0;
        T nis{};
        std::size_t dof = 0;
        int valid = 0;
        double threshold = 0.0;  ///< the gate the run used (0 = gating off)
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "accept if NIS <= 5 per dof"},
                {"chi2_95", "accept if NIS <= the 95% chi-square(dof) quantile (Wilson-Hilferty)"},
                {"gate_off", "accept every measurement (NIS still computed)"}};
    }

    /// The 95% χ²(k) quantile, Wilson–Hilferty: k·(1 − 2/(9k) + z₉₅·√(2/(9k)))³.
    [[nodiscard]] static double chi2_95(std::size_t k) {
        const double kk = static_cast<double>(k), a = 2.0 / (9.0 * kk);
        const double c = 1.0 - a + 1.6448536269514722 * std::sqrt(a);
        return kk * c * c * c;
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        return json{{"state", pack(in.state)},
                    {"options", s6::encode_options(in.options)},
                    {"projected", s6::encode_projected(in.projected)}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        Input<T> in;
        in.state = unpack_state<T>(j.at("state"));
        in.options = s6::decode_options<T>(j.at("options"));
        in.projected = s6::decode_projected<T>(j.at("projected"));
        if (in.projected.H.cols != in.state.dim())
            throw std::invalid_argument("s6d bench: H columns != state dimension");
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"accepted", out.accepted},
                    {"nis", pack(out.nis)},
                    {"dof", out.dof},
                    {"valid", out.valid},
                    {"threshold", pack_num(out.threshold)}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        return Output<T>{j.at("accepted").get<int>(),
                         unpack_scalar<T>(j.at("nis")),
                         j.at("dof").get<std::size_t>(),
                         j.at("valid").get<int>(),
                         unpack_num(j.at("threshold"))};
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        auto opts = in.options;
        const std::size_t k = in.projected.H.rows;
        if (variant == "gate_off")
            opts.enable_gating = false;
        else if (variant == "chi2_95" && k > 0)
            opts.chi2_per_dof = T(chi2_95(k) / static_cast<double>(k));
        const sdk::msckf::CameraUpdater<T> upd(std::vector<sdk::msckf::CameraExtrinsics<T>>{{}}, opts);
        const auto g = sdk::msckf::stages::s6d_gating::apply(in.state, upd, in.projected);
        const double thr = opts.enable_gating ? static_cast<double>(opts.chi2_per_dof) * static_cast<double>(k) : 0.0;
        return Output<T>{g.accepted ? 1 : 0, g.nis.value, g.nis.dof, g.nis.valid ? 1 : 0, thr};
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        return {static_cast<double>(out.accepted),
                static_cast<double>(out.nis),
                static_cast<double>(out.dof),
                static_cast<double>(out.valid),
                out.threshold};
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        const auto& pm = in.projected;
        const T var = in.options.normalized_sigma * in.options.normalized_sigma +
                      in.options.calib_rot_sigma * in.options.calib_rot_sigma;
        const auto S = s6::innovation(in.state.cov.P, pm.H, var);
        // S's entries sit near σ², far from 1: hold a tapered type's symmetry to
        // its precision there. (Not the SPD margin: a lower bound gets stricter
        // with a larger safety factor.)
        double smin = inv::detail::kInf;
        for (std::size_t i = 0; i < S.rows; ++i)
            smin = std::min(smin, std::abs(static_cast<double>(S(i, i))));
        const double safety = safety_at<T>(smin);
        // H·P·Hᵀ cancels: its rounding scales with |H|·|P|·|H|ᵀ, which can far
        // exceed |S| (a captured EuRoC-like update: 3.8e-18 asymmetry against a
        // max|S|-scaled bound). Hold symmetry to the products' magnitude.
        double s_max = 0.0;
        for (const T& x : S.d)
            s_max = std::max(s_max, std::abs(static_cast<double>(x)));
        const double products = magnitude_of_products(pm.H, in.state.cov.P) + static_cast<double>(var);
        const double sym_safety = s_max > 0.0 ? safety * std::max(1.0, products / s_max) : safety;
        r.push_back(inv::check_symmetric(S, kInvStage, "innovation.symmetric", sym_safety));
        r.push_back(inv::check_spd(S, kInvStage, "innovation.spd", safety_vs_double<T>()));
        const auto kappa = inv::report_condition_number(S, kInvStage, "innovation.condition_number");
        r.push_back(kappa);
        r.push_back(inv::check_scalar(out.valid, 1.0, inv::Bound::Lower, kInvStage, "nis.valid", "flag"));

        double direct = inv::detail::kInf;
        try {
            direct = static_cast<double>(sdk::eval::normalized_squared<T>(std::span<const T>(pm.r), S));
        } catch (const std::exception&) {}
        const double nis = static_cast<double>(out.nis);
        // Both are rᵀS⁻¹r through a Cholesky of S: their difference is the
        // solves' error, ∝ κ(S)·γ.
        r.push_back(
            inv::check_scalar(std::abs(nis - direct),
                              tolerance_vs_double<T>(pm.H.rows, std::max(1.0, kappa.value) * std::max(nis, 1e-30)),
                              inv::Bound::Upper,
                              kInvStage,
                              "nis.matches_direct",
                              "dimensionless"));
        const bool want = out.threshold == 0.0 || (out.valid && nis <= out.threshold);
        r.push_back(inv::check_scalar(want == (out.accepted != 0) ? 0.0 : 1.0,
                                      0.0,
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "gate.decision_consistent",
                                      "count"));
        const double per_dof = out.dof ? nis / static_cast<double>(out.dof) : inv::detail::kInf;
        if (f.kind == FixtureKind::GroundTruth)
            // Exact observations at the true state: no innovation at all.
            r.push_back(inv::check_scalar(per_dof,
                                          tolerance_vs_double<T>(pm.H.rows, 1.0),
                                          inv::Bound::Upper,
                                          kInvStage,
                                          "nis.zero_at_truth",
                                          "per dof"));
        else
            r.push_back(inv::check_scalar(per_dof, 0.0, inv::Bound::Report, kInvStage, "nis.per_dof", "per dof"));
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_two_rows", known_answer()},
                {"ground_truth_synthetic_track", ground_truth()},
                {"captured_estimated_poses", captured()}};
    }

    [[nodiscard]] static Fixture known_answer() {
        Input<double> in;
        in.state = s6::State<double>(0.1);
        in.projected.H = s6::Mat<double>(2, in.state.dim());
        in.projected.H(0, s6::State<double>::kPos) = 1.0;
        in.projected.H(1, s6::State<double>::kPos + 1) = 1.0;
        in.projected.r = {0.2, 0.2};
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.variant = "shipped";
        f.source = "s6d_gating_bench: P = 0.01 I, H selects dp_x, dp_y, r = (0.2, 0.2), sigma = 0.01";
        f.description = "expected: NIS = 0.08 / 0.0101 on 2 dof, accepted by 5 per dof";
        f.input = encode_input(in);
        f.expected = encode_output(Output<double>{1, 0.08 / 0.0101, 2, 1, 10.0});
        return f;
    }

    [[nodiscard]] static Fixture ground_truth() {
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config), true state, exact observations";
        f.seed = sdk::eval::SyntheticConfig<double>{}.seed;
        f.description = "truth: NIS = 0";
        f.input = encode_input(input_of(s6::build_scene(/*true_poses=*/true)));
        f.truth = json{{"nis", 0.0}};
        return f;
    }

    [[nodiscard]] static Fixture captured() {
        const auto in = input_of(s6::build_scene(/*true_poses=*/false));
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config), S2/S3 estimated state, shipped S5/S6a/S6b",
                       encode_input(in),
                       encode_output(run<double>(in, kShipped)));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("dof", {1.0, 3.0, 9.0, 21.0}).axis("outlier_sigma", {0.0, 2.0, 4.0, 8.0});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"nis_per_dof_mean", "accept_rate_shipped", "accept_rate_chi2_95", "chi2_band_lower", "chi2_band_upper"};
    }
    /// 200 draws r = L·z (S = L·Lᵀ, z standard normal) plus `outlier_sigma`·√S_ii
    /// on every component, against a random 15-state H with P = 0.01·I.
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double kd = p.at("dof"), out_s = p.at("outlier_sigma");
        if (!(kd >= 1.0 && kd <= 200.0) || !(out_s >= 0.0 && out_s <= 1e3))
            throw std::invalid_argument("s6d bench sweep: dof in [1, 200], outlier_sigma in [0, 1e3]");
        const auto k = static_cast<std::size_t>(kd);
        Input<double> in;
        in.state = s6::State<double>(0.1);
        const std::size_t n = in.state.dim();
        in.projected.H = s6::Mat<double>(k, n);
        for (std::size_t i = 0; i < k; ++i)
            for (std::size_t j = 0; j < n; ++j)
                in.projected.H(i, j) = 0.5 * s6::pseudo_normal(static_cast<std::uint32_t>(1000 + i * n + j));
        const double var = 1e-4;
        const auto S = s6::innovation(in.state.cov.P, in.projected.H, var);
        s6::Mat<double> L;
        if (!sdk::msckf::cholesky(S, L))
            throw std::logic_error("s6d bench sweep: S is not SPD");
        const int draws = 200;
        double nis_sum = 0.0, acc_ship = 0.0, acc_chi = 0.0;
        sdk::eval::ConsistencyAccumulator acc;
        for (int d = 0; d < draws; ++d) {
            in.projected.r.assign(k, 0.0);
            for (std::size_t i = 0; i < k; ++i) {
                double v = 0.0;
                for (std::size_t c = 0; c <= i; ++c)
                    v += L(i, c) * s6::pseudo_normal(static_cast<std::uint32_t>(d * 4096 + c));
                in.projected.r[i] = v + out_s * std::sqrt(S(i, i));
            }
            const auto inT = decode_input<T>(encode_input(in));
            const auto ship = run<T>(inT, "shipped");
            const auto chi = run<T>(inT, "chi2_95");
            nis_sum += static_cast<double>(ship.nis) / static_cast<double>(k);
            acc.add(static_cast<double>(ship.nis), static_cast<int>(k));
            acc_ship += ship.accepted;
            acc_chi += chi.accepted;
        }
        const auto rep = acc.report(0.05);
        return {nis_sum / draws, acc_ship / draws, acc_chi / draws, rep.lower, rep.upper};
    }

    /// max over (i, j) of (|H|·|P|·|H|ᵀ)ᵢⱼ, in double: the magnitude the terms
    /// of H·P·Hᵀ reach before they cancel.
    template <class T>
    [[nodiscard]] static double magnitude_of_products(const s6::Mat<T>& H, const s6::Mat<T>& P) {
        s6::Mat<double> h(H.rows, H.cols), p(P.rows, P.cols);
        for (std::size_t i = 0; i < H.d.size(); ++i)
            h.d[i] = std::abs(static_cast<double>(H.d[i]));
        for (std::size_t i = 0; i < P.d.size(); ++i)
            p.d[i] = std::abs(static_cast<double>(P.d[i]));
        const auto m = sdk::msckf::mul(sdk::msckf::mul(h, p), sdk::msckf::transpose(h));
        double worst = 0.0;
        for (const double x : m.d)
            worst = std::max(worst, x);
        return worst;
    }

private:
    [[nodiscard]] static Input<double> input_of(const s6::Scene& sc) {
        Input<double> in;
        in.state = sc.state;
        in.projected = s6::boundaries(sc, 0).projected;
        return in;
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S6D_GATING_BENCH_HPP
