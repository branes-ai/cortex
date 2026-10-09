// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s6e_ekf_update_bench.hpp — the S6e_ekf_update stage bench
// (issue #457, epic #444).
//
// Stage under test: stages::s6e_ekf_update::apply(state, updater, projected) —
// the Kalman gain, the covariance update (Joseph form on the dense covariance,
// or the QR array form on the square-root factor) and the box-plus of δx onto
// the mean. Contract (#445 §B, S6e):
//   • δx is finite and equals K·r with K = P Hᵀ S⁻¹ computed directly;
//   • P⁺ is symmetric, ⪰ 0, ⪯ P⁻, and equals P⁻ − K S Kᵀ;
//   • the update gains no information along the unobservable directions N
//     (global position, yaw): the information-form correction P⁻¹δx = Hᵀ S⁻¹ r
//     has no component along N. A camera update at one consistent linearization
//     point satisfies this exactly; the FEJ question is whether the filter's
//     linearization points stay consistent across stages (S2, not S6).
//   • reported: tr(P⁺)/tr(P⁻) and ‖δx‖.
//
// Fixtures:
//   known_answer   P = 0.01·I₁₅, H selecting δp_x and δp_y, r = (0.1, 0.2),
//                  σ = 0.01: δp = r·0.01/0.0101, P⁺ on those two = 1e-6/0.0101
//   ground_truth   the synthetic-world track at the true state (δx ≈ 0), with
//                  the unobservable basis
//   captured       the same at the S2/S3 estimated state — recorded
//
// Variants: "shipped" (Joseph form, dense P), "sqrt_array" (the square-root
// covariance's QR array update).
//
// Sweep: pixel noise × calibration rotation uncertainty (the S10 coupling) →
// covariance reduction, information along N, ‖δx‖ and NIS per dof.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S6E_EKF_UPDATE_BENCH_HPP
#define BRANES_TOOLS_BENCH_S6E_EKF_UPDATE_BENCH_HPP

#include <branes/sdk/eval/consistency.hpp>
#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/msckf/sqrt_covariance.hpp>
#include <branes/tools/bench/s6_scene.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S6eEkfUpdateBench {
    static constexpr std::string_view kStage = "S6e_ekf_update";
    static constexpr inv::Stage kInvStage = inv::Stage::S6e_ekf_update;

    template <class T>
    struct Input {
        s6::State<T> state{T{1}};
        s6::Options<T> options{};
        s6::Projected<T> projected{};
        s6::Mat<T> unobservable{};  ///< n×4 unobservable basis, or empty
    };
    template <class T>
    struct Output {
        std::vector<T> dx;
        s6::Mat<T> p_plus{};
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "Joseph-form update of the dense covariance"},
                {"sqrt_array", "QR array update of the square-root covariance factor"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        return json{{"state", pack(in.state)},
                    {"options", s6::encode_options(in.options)},
                    {"projected", s6::encode_projected(in.projected)},
                    {"unobservable", pack(in.unobservable)}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        Input<T> in;
        in.state = unpack_state<T>(j.at("state"));
        in.options = s6::decode_options<T>(j.at("options"));
        in.projected = s6::decode_projected<T>(j.at("projected"));
        in.unobservable = unpack_mat<T>(j.at("unobservable"));
        if (in.projected.H.cols != in.state.dim())
            throw std::invalid_argument("s6e bench: H columns != state dimension");
        if (in.unobservable.rows != 0 && in.unobservable.rows != in.state.dim())
            throw std::invalid_argument("s6e bench: unobservable basis rows != state dimension");
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"dx", pack_vec<T>(out.dx)}, {"p_plus", pack(out.p_plus)}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        return Output<T>{unpack_vec<T>(j.at("dx")), unpack_mat<T>(j.at("p_plus"))};
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        namespace s6e = sdk::msckf::stages::s6e_ekf_update;
        const sdk::msckf::CameraUpdater<T> upd(std::vector<sdk::msckf::CameraExtrinsics<T>>{{}}, in.options);
        Output<T> out;
        if (variant != "sqrt_array") {
            auto s = in.state;
            out.dx = s6e::apply(s, upd, in.projected).dx;
            out.p_plus = s.cov.covariance();
            return out;
        }
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
        s6::Mat<T> L;
        if (!psd_factor(in.state.cov.P, L)) {
            // Not PSD in T: no square-root form exists.
            out.dx.assign(in.state.dim(), T(std::numeric_limits<double>::quiet_NaN()));
            out.p_plus = in.state.cov.P;
            return out;
        }
        sq.cov.S = L;
        out.dx = s6e::apply(sq, upd, in.projected).dx;
        out.p_plus = sq.covariance();
        return out;
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        std::vector<double> f;
        for (const T& v : out.dx)
            f.push_back(static_cast<double>(v));
        for (std::size_t i = 0; i < out.p_plus.rows; ++i)
            for (std::size_t j = 0; j < out.p_plus.cols; ++j)
                f.push_back(static_cast<double>(out.p_plus(i, j)));
        return f;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& /*f*/) {
        namespace m = sdk::msckf;
        std::vector<inv::InvariantResult> r;
        const auto& P = in.state.cov.P;
        const auto& H = in.projected.H;
        const std::size_t n = P.rows;
        const double safety = safety_vs_double<T>();
        r.push_back(inv::check_finite<T>(std::span<const T>(out.dx), kInvStage, "dx.finite"));

        const T var = in.options.normalized_sigma * in.options.normalized_sigma +
                      in.options.calib_rot_sigma * in.options.calib_rot_sigma;
        const auto S = s6::innovation(P, H, var);
        const double kappa = std::max(1.0, inv::report_condition_number(S, kInvStage).value);
        s6::Mat<T> L;
        const bool spd = m::cholesky(S, L);
        r.push_back(inv::check_scalar(spd ? 1.0 : 0.0, 1.0, inv::Bound::Lower, kInvStage, "innovation.spd", "flag"));
        if (!spd)
            return r;
        const auto PHt = m::mul(P, m::transpose(H));
        const auto y = m::cholesky_solve(L, s6::as_col(in.projected.r));  // S⁻¹ r
        const auto dx_ref = m::mul(PHt, y);                               // K r

        // δx against K·r: their difference is the solves' error, ∝ κ(S)·|K r|.
        double worst = out.dx.size() == n ? 0.0 : inv::detail::kInf, scale = 0.0;
        for (std::size_t i = 0; i < std::min(n, out.dx.size()); ++i) {
            const double a = static_cast<double>(out.dx[i]), b = static_cast<double>(dx_ref(i, 0));
            worst = std::isfinite(a) ? std::max(worst, std::abs(a - b)) : inv::detail::kInf;
            scale = std::max(scale, std::abs(b));
        }
        r.push_back(inv::check_scalar(worst,
                                      tolerance_vs_double<T>(n, kappa * std::max(scale, 1e-300), safety_at<T>(scale)),
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "dx.equals_gain_times_residual",
                                      "state units"));

        // P⁺ = P − PHᵀ S⁻¹ HP, symmetric, PSD, and no larger than P.
        const auto& Pp = out.p_plus;
        r.push_back(inv::check_symmetric(Pp, kInvStage, "covariance.symmetric", safety));
        r.push_back(inv::check_psd(Pp, kInvStage, "covariance.psd", safety));
        r.push_back(inv::check_loewner_le(Pp, P, kInvStage, "covariance.no_larger_than_prior", safety));
        const auto p_ref = la_sub(P, m::mul(PHt, m::cholesky_solve(L, m::transpose(PHt))));
        r.push_back(inv::check_scalar(s6::rel_diff(Pp, p_ref),
                                      tolerance_vs_double<T>(n, kappa),
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "covariance.equals_p_minus_k_s_kt",
                                      "dimensionless"));

        if (in.unobservable.rows != 0) {
            // The information-form correction P⁻¹δx = Hᵀ S⁻¹ r.
            const auto info = m::mul(m::transpose(H), y);
            std::vector<T> iv(n);
            for (std::size_t i = 0; i < n; ++i)
                iv[i] = info(i, 0);
            double yn = 0.0;
            for (const T& v : iv)
                yn += static_cast<double>(v) * static_cast<double>(v);
            r.push_back(inv::check_no_update_along<T>(std::span<const T>(iv),
                                                      in.unobservable,
                                                      kInvStage,
                                                      "update.info_along_unobservable",
                                                      safety_at<T>(std::sqrt(yn / static_cast<double>(n)))));
        }

        double tp = 0.0, tq = 0.0, dn = 0.0;
        for (std::size_t i = 0; i < n && i < Pp.rows; ++i) {
            tp += static_cast<double>(P(i, i));
            tq += static_cast<double>(Pp(i, i));
        }
        for (const T& v : out.dx)
            dn += static_cast<double>(v) * static_cast<double>(v);
        r.push_back(inv::check_scalar(tp > 0.0 ? tq / tp : inv::detail::kInf,
                                      0.0,
                                      inv::Bound::Report,
                                      kInvStage,
                                      "covariance.trace_ratio",
                                      "ratio"));
        r.push_back(inv::check_scalar(std::sqrt(dn), 0.0, inv::Bound::Report, kInvStage, "dx.norm", "state units"));
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
        const std::size_t n = in.state.dim(), px = s6::State<double>::kPos;
        in.projected.H = s6::Mat<double>(2, n);
        in.projected.H(0, px) = 1.0;
        in.projected.H(1, px + 1) = 1.0;
        in.projected.r = {0.1, 0.2};
        Output<double> want;
        want.dx.assign(n, 0.0);
        const double g = 0.01 / 0.0101;
        want.dx[px] = 0.1 * g;
        want.dx[px + 1] = 0.2 * g;
        want.p_plus = s6::Mat<double>(n, n);
        for (std::size_t i = 0; i < n; ++i)
            want.p_plus(i, i) = 0.01;
        want.p_plus(px, px) = want.p_plus(px + 1, px + 1) = 0.01 * 1e-4 / 0.0101;
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.source = "s6e_ekf_update_bench: P = 0.01 I, H selects dp_x, dp_y, r = (0.1, 0.2), sigma = 0.01";
        f.description = "expected: closed-form scalar Kalman update on two coordinates";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        return f;
    }

    [[nodiscard]] static Fixture ground_truth() {
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config), true state, exact observations; unobservable basis";
        f.seed = sdk::eval::SyntheticConfig<double>{}.seed;
        f.description = "truth: no information along the unobservable directions";
        f.input = encode_input(input_of(s6::build_scene(/*true_poses=*/true)));
        f.truth = json{{"info_along_unobservable", 0.0}};
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
        sw.axis("sigma_px", {0.25, 1.0, 4.0}).axis("calib_rot_sigma", {0.0, 1e-3, 1e-2});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"trace_ratio", "info_along_unobservable", "dx_norm", "nis_per_dof"};
    }
    /// The captured track's update at pixel noise `sigma_px` (EuRoC fx) plus an
    /// isotropic calibration-rotation term `calib_rot_sigma` (rad).
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double sp = p.at("sigma_px"), cr = p.at("calib_rot_sigma");
        if (!(sp > 0.0 && sp <= 100.0) || !(cr >= 0.0 && cr <= 0.5))
            throw std::invalid_argument("s6e bench sweep: sigma_px in (0, 100], calib_rot_sigma in [0, 0.5]");
        auto in = input_of(s6::build_scene(false));
        in.options.normalized_sigma = sp / 458.654;
        in.options.calib_rot_sigma = cr;
        const auto inT = decode_input<T>(encode_input(in));
        const auto out = run<T>(inT, kShipped);
        double tr = inv::detail::kInf, info = inv::detail::kInf, dn = inv::detail::kInf;
        for (const auto& x : invariants<T>(inT, out, Fixture{})) {
            if (x.name == "covariance.trace_ratio")
                tr = x.value;
            if (x.name == "update.info_along_unobservable")
                info = x.value;
            if (x.name == "dx.norm")
                dn = x.value;
        }
        const double var = in.options.normalized_sigma * in.options.normalized_sigma + cr * cr;
        const auto S = s6::innovation(in.state.cov.P, in.projected.H, var);
        const double nis = sdk::eval::normalized_squared<double>(std::span<const double>(in.projected.r), S);
        return {tr, info, dn, nis / static_cast<double>(in.projected.H.rows)};
    }

private:
    template <class T>
    [[nodiscard]] static s6::Mat<T> la_sub(const s6::Mat<T>& a, const s6::Mat<T>& b) {
        s6::Mat<T> c = a;
        for (std::size_t i = 0; i < a.rows; ++i)
            for (std::size_t j = 0; j < a.cols; ++j)
                c(i, j) = a(i, j) - b(i, j);
        return c;
    }

    [[nodiscard]] static Input<double> input_of(const s6::Scene& sc) {
        Input<double> in;
        in.state = sc.state;
        in.projected = s6::boundaries(sc, 0).projected;
        in.unobservable = s6::unobservable_basis(sc.state);
        return in;
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S6E_EKF_UPDATE_BENCH_HPP
