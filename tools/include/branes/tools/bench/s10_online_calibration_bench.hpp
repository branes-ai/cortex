// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s10_online_calibration_bench.hpp — the
// S10_online_calibration stage bench (issue #458, epic #444).
//
// Stage under test: stages::s10_online_calibration::apply(state, extrinsics,
// σ_rot, σ_trans) — the camera↔IMU extrinsic enters the error state with its
// prior — and what it is for: the extrinsic estimated by the camera updates
// that follow (S6a contributes its Jacobian columns, S6e its correction). The
// bench runs S10 on a fresh state (its own contract), then the S6 update over
// a window of tracks with the calibration treated three ways. Contract (#445
// §B, S10):
//   • S10 adds 6 error states per camera, with the prior σ_rot², σ_trans² on
//     the diagonal of an uncorrelated block;
//   • the estimated extrinsic stays in SO(3); P ⪰ 0 and symmetric after the
//     updates; the calibration covariance is ⪰ 0 and ⪯ its prior;
//   • on ground truth (the true extrinsic known), the estimated calibration's
//     error is consistent with its covariance: NEES ≤ χ²₀.₉₉₉(6);
//   • reported: the extrinsic rotation error before and after, NIS per dof.
//
// The scene (s6_scene.hpp) applies S10 to the fresh state with a believed
// extrinsic perturbed from the world's true one, then runs S2/S3; the world's
// observations come from the true extrinsic. Time offset t_d is not modeled
// by the filter, so it has no bench axis.
//
// Fixtures:
//   known_answer   a fresh state, S10 with σ = (0.03 rad, 0.02 m), no tracks:
//                  P = blockdiag(0.01·I₁₅, prior), the extrinsic unchanged
//   ground_truth   true poses and exact observations, the believed extrinsic
//                  1° and 1 cm off the true one (the truth)
//   captured       the same perturbation at the S2/S3 estimated state — recorded
//
// Variants: "shipped" (the extrinsic estimated as state), "fixed" (the
// calibration block dropped, the believed extrinsic trusted), "folded_into_r"
// (fixed, with the prior's σ_rot added to the measurement noise — the
// `calib_ext_rot_sigma_deg` option).
//
// Sweep: extrinsic rotation error × rotation prior → the estimated extrinsic's
// remaining error, and NIS per dof for each treatment.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S10_ONLINE_CALIBRATION_BENCH_HPP
#define BRANES_TOOLS_BENCH_S10_ONLINE_CALIBRATION_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/msckf/stages/s10_online_calibration.hpp>
#include <branes/tools/bench/s6_scene.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S10OnlineCalibrationBench {
    static constexpr std::string_view kStage = "S10_online_calibration";
    static constexpr inv::Stage kInvStage = inv::Stage::S10_online_calibration;
    static constexpr double kDeg = std::numbers::pi / 180.0;

    template <class T>
    struct Input {
        s6::State<T> state{T{1}};  ///< after S10 (calibration block present) and S2/S3
        T rot_sigma{};             ///< the S10 prior (rad)
        T trans_sigma{};           ///< the S10 prior (m)
        s6::Options<T> options{};
        std::vector<s6::Track<T>> tracks;
    };
    template <class T>
    struct Output {
        math::lie::SO3<T> R_ic{};  ///< the extrinsic after the window (estimated or trusted)
        s6::Vec3<T> p_ic{};
        s6::Mat<T> calib_cov{};  ///< 6×6 calibration block (empty when not estimated)
        s6::Mat<T> p_after{};
        double nis_sum = 0.0;
        double dof_sum = 0.0;
        std::size_t applied = 0;        ///< tracks the update accepted
        std::size_t s10_dim_after = 0;  ///< S10 on a fresh state: the dimension it produced
        std::vector<T> s10_block;       ///< … and the diagonal of the block it added
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "the extrinsic estimated as error state (S10 + S6)"},
                {"fixed", "the calibration block dropped; the believed extrinsic trusted"},
                {"folded_into_r", "fixed, with the prior rotation sigma added to the measurement noise"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        json tracks = json::array();
        for (const auto& t : in.tracks)
            tracks.push_back(s6::encode_track(t));
        return json{{"state", pack(in.state)},
                    {"rot_sigma", pack(in.rot_sigma)},
                    {"trans_sigma", pack(in.trans_sigma)},
                    {"options", s6::encode_options(in.options)},
                    {"tracks", tracks}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        Input<T> in;
        in.state = unpack_state<T>(j.at("state"));
        if (in.state.calib.size() != 1)
            throw std::invalid_argument("s10 bench: the state must carry exactly one calibration block");
        in.rot_sigma = unpack_scalar<T>(j.at("rot_sigma"));
        in.trans_sigma = unpack_scalar<T>(j.at("trans_sigma"));
        if (!(in.rot_sigma > T{0}) || !(in.trans_sigma > T{0}))
            throw std::invalid_argument("s10 bench: the calibration prior must be positive");
        in.options = s6::decode_options<T>(j.at("options"));
        for (const auto& t : j.at("tracks"))
            in.tracks.push_back(s6::decode_track<T>(t, in.state.clones.size()));
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"R_ic", pack(out.R_ic)},
                    {"p_ic", pack(out.p_ic)},
                    {"calib_cov", pack(out.calib_cov)},
                    {"p_after", pack(out.p_after)},
                    {"nis_sum", pack_num(out.nis_sum)},
                    {"dof_sum", pack_num(out.dof_sum)},
                    {"applied", out.applied},
                    {"s10_dim_after", out.s10_dim_after},
                    {"s10_block", pack_vec<T>(out.s10_block)}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        Output<T> out;
        out.R_ic = unpack_so3<T>(j.at("R_ic"));
        out.p_ic = unpack_fixed<T, 3>(j.at("p_ic"));
        out.calib_cov = unpack_mat<T>(j.at("calib_cov"));
        out.p_after = unpack_mat<T>(j.at("p_after"));
        out.nis_sum = unpack_num(j.at("nis_sum"));
        out.dof_sum = unpack_num(j.at("dof_sum"));
        out.applied = j.at("applied").get<std::size_t>();
        out.s10_dim_after = j.at("s10_dim_after").get<std::size_t>();
        out.s10_block = unpack_vec<T>(j.at("s10_block"));
        return out;
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        namespace st = sdk::msckf::stages;
        using St = s6::State<T>;
        Output<T> out;
        const auto believed = in.state.calib.front();

        // S10 itself, on a fresh state with this IMU block: what it adds.
        {
            St fresh(T{1});
            fresh.R = in.state.R;
            fresh.p = in.state.p;
            fresh.v = in.state.v;
            for (std::size_t i = 0; i < St::kImuDim; ++i)
                for (std::size_t j = 0; j < St::kImuDim; ++j)
                    fresh.cov.P(i, j) = in.state.cov.P(i, j);
            const auto d = st::s10_online_calibration::apply(fresh, {believed}, in.rot_sigma, in.trans_sigma);
            out.s10_dim_after = d.dim_after;
            for (std::size_t i = 0; i < 6 && St::kImuDim + i < fresh.dim(); ++i)
                out.s10_block.push_back(fresh.cov.P(St::kImuDim + i, St::kImuDim + i));
        }

        St s = variant == "shipped" ? in.state : drop_calibration(in.state);
        auto opts = in.options;
        if (variant == "folded_into_r")
            opts.calib_rot_sigma = in.rot_sigma;
        const sdk::msckf::CameraUpdater<T> upd(
            std::vector<sdk::msckf::CameraExtrinsics<T>>{{believed.R_imu_cam, believed.p_imu_cam}}, opts);
        for (const auto& t : in.tracks) {
            const auto d = st::s6_msckf_update::apply(s, upd, t);
            if (d.nis.valid) {
                out.nis_sum += static_cast<double>(d.nis.value);
                out.dof_sum += static_cast<double>(d.nis.dof);
            }
            out.applied += d.accepted() ? 1 : 0;
        }
        if (variant == "shipped") {
            out.R_ic = s.calib.front().R_imu_cam;
            out.p_ic = s.calib.front().p_imu_cam;
            out.calib_cov = s6::Mat<T>(6, 6);
            const std::size_t off = s.calib_offset(0);
            for (std::size_t i = 0; i < 6; ++i)
                for (std::size_t j = 0; j < 6; ++j)
                    out.calib_cov(i, j) = s.cov.P(off + i, off + j);
        } else {
            out.R_ic = believed.R_imu_cam;
            out.p_ic = believed.p_imu_cam;
        }
        out.p_after = s.cov.P;
        return out;
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        std::vector<double> f;
        for (std::size_t i = 0; i < 4; ++i)
            f.push_back(static_cast<double>(out.R_ic.quaternion()[i]));
        for (std::size_t i = 0; i < 3; ++i)
            f.push_back(static_cast<double>(out.p_ic[i]));
        for (const auto* m : {&out.calib_cov, &out.p_after})
            for (const T& x : m->d)
                f.push_back(static_cast<double>(x));
        f.push_back(out.nis_sum);
        f.push_back(out.dof_sum);
        f.push_back(static_cast<double>(out.applied));
        f.push_back(static_cast<double>(out.s10_dim_after));
        for (const T& x : out.s10_block)
            f.push_back(static_cast<double>(x));
        return f;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        using St = s6::State<T>;
        // S10's own contract: 6 states per camera, prior on the diagonal.
        r.push_back(inv::check_dimension(out.s10_dim_after, St::kImuDim + 6, kInvStage, "s10.dimension"));
        const double sr = static_cast<double>(in.rot_sigma), stt = static_cast<double>(in.trans_sigma);
        double prior_err = out.s10_block.size() == 6 ? 0.0 : inv::detail::kInf;
        for (std::size_t i = 0; i < std::min<std::size_t>(6, out.s10_block.size()); ++i) {
            const double want = i < 3 ? sr * sr : stt * stt;
            prior_err = std::max(prior_err, std::abs(static_cast<double>(out.s10_block[i]) - want) / want);
        }
        r.push_back(inv::check_scalar(
            prior_err, tolerance_vs_double<T>(2, 1.0), inv::Bound::Upper, kInvStage, "s10.prior_block", "relative"));

        for (auto&& x : inv::check_so3(out.R_ic,
                                       kInvStage,
                                       "extrinsic.so3_orthogonality",
                                       "extrinsic.so3_determinant",
                                       safety_vs_double<T>()))
            r.push_back(std::move(x));
        double pmin = inv::detail::kInf;
        for (std::size_t i = 0; i < out.p_after.rows; ++i)
            pmin = std::min(pmin, std::abs(static_cast<double>(out.p_after(i, i))));
        const double safety = safety_at<T>(pmin);
        r.push_back(inv::check_symmetric(out.p_after, kInvStage, "covariance.symmetric", safety));
        r.push_back(inv::check_psd(out.p_after, kInvStage, "covariance.psd", safety_vs_double<T>()));
        if (out.calib_cov.rows == 6) {
            s6::Mat<T> prior(6, 6);
            for (std::size_t i = 0; i < 6; ++i)
                prior(i, i) = i < 3 ? in.rot_sigma * in.rot_sigma : in.trans_sigma * in.trans_sigma;
            r.push_back(inv::check_psd(out.calib_cov, kInvStage, "calibration.covariance_psd", safety_vs_double<T>()));
            r.push_back(inv::check_loewner_le(
                out.calib_cov, prior, kInvStage, "calibration.covariance_within_prior", safety_at<T>(stt * stt)));
        }
        r.push_back(inv::check_scalar(out.dof_sum > 0 ? out.nis_sum / out.dof_sum : 0.0,
                                      0.0,
                                      inv::Bound::Report,
                                      kInvStage,
                                      "nis.per_dof",
                                      "per dof"));
        r.push_back(inv::check_scalar(
            static_cast<double>(out.applied), 0.0, inv::Bound::Report, kInvStage, "update.tracks_applied", "count"));

        if (f.kind == FixtureKind::GroundTruth) {
            const auto Rt = unpack_so3<double>(f.truth.at("R_imu_cam"));
            const auto pt = unpack_fixed<double, 3>(f.truth.at("p_imu_cam"));
            const auto err = calib_error(Rt, pt, out.R_ic, out.p_ic);
            const auto before = calib_error(Rt, pt, in.state.calib.front().R_imu_cam, in.state.calib.front().p_imu_cam);
            r.push_back(inv::check_scalar(
                rot_deg(before), 0.0, inv::Bound::Report, kInvStage, "truth.rotation_error_before", "deg"));
            r.push_back(inv::check_scalar(
                rot_deg(err), 0.0, inv::Bound::Report, kInvStage, "truth.rotation_error_after", "deg"));
            if (out.calib_cov.rows == 6) {
                // The estimate's error is consistent with its covariance.
                double nees = inv::detail::kInf;
                s6::Mat<double> C(6, 6), L;
                for (std::size_t i = 0; i < 6; ++i)
                    for (std::size_t j = 0; j < 6; ++j)
                        C(i, j) = static_cast<double>(out.calib_cov(i, j));
                if (sdk::msckf::cholesky(C, L)) {
                    const auto y = sdk::msckf::cholesky_solve(L, s6::as_col(err));
                    nees = 0.0;
                    for (std::size_t i = 0; i < 6; ++i)
                        nees += err[i] * y(i, 0);
                }
                r.push_back(inv::check_scalar(
                    nees, 22.458, inv::Bound::Upper, kInvStage, "calibration.nees_chi2_999", "chi2(6)"));
            }
        }
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_fresh_state", known_answer()},
                {"ground_truth_perturbed_extrinsic", ground_truth()},
                {"captured_estimated_poses", captured()}};
    }

    /// A fresh state, S10 with σ = (0.03 rad, 0.02 m), no tracks: the update
    /// changes nothing, P = blockdiag(0.01·I₁₅, prior).
    [[nodiscard]] static Fixture known_answer() {
        Input<double> in;
        in.state = s6::State<double>(0.1);
        in.rot_sigma = 0.03;
        in.trans_sigma = 0.02;
        const typename s6::State<double>::CalibState ex{s6::State<double>::SO3::exp({{0.01, -0.02, 0.03}}),
                                                        {{0.05, -0.01, 0.02}}};
        sdk::msckf::stages::s10_online_calibration::apply(in.state, {ex}, in.rot_sigma, in.trans_sigma);
        Output<double> want;
        want.R_ic = ex.R_imu_cam;
        want.p_ic = ex.p_imu_cam;
        want.calib_cov = s6::Mat<double>(6, 6);
        want.p_after = s6::Mat<double>(21, 21);
        for (std::size_t i = 0; i < 21; ++i)
            want.p_after(i, i) = i < 15 ? 0.01 : (i < 18 ? 0.03 * 0.03 : 0.02 * 0.02);
        for (std::size_t i = 0; i < 6; ++i)
            want.calib_cov(i, i) = want.p_after(15 + i, 15 + i);
        want.s10_dim_after = 21;
        for (std::size_t i = 0; i < 6; ++i)
            want.s10_block.push_back(want.p_after(15 + i, 15 + i));
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.variant = "shipped";
        f.source = "s10_online_calibration_bench: fresh state, prior (0.03 rad, 0.02 m), no tracks";
        f.description = "expected: P = blockdiag(0.01 I15, prior), extrinsic unchanged";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        return f;
    }

    [[nodiscard]] static Fixture ground_truth() {
        const auto [in, truth] = scenario(/*true_poses=*/true, 1.0, 2.0);
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source =
            "synthetic_world (default config), true poses, exact observations; believed extrinsic 1 deg, 1 cm off";
        f.seed = sdk::eval::SyntheticConfig<double>{}.seed;
        f.description = "truth: the world's camera-IMU extrinsic";
        f.input = encode_input(in);
        f.truth = truth;
        return f;
    }

    [[nodiscard]] static Fixture captured() {
        const auto in = scenario(/*true_poses=*/false, 1.0, 2.0).first;
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config), S10 then S2/S3 estimated state; extrinsic 1 deg, 1 cm off",
                       encode_input(in),
                       encode_output(run<double>(in, kShipped)));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("rot_error_deg", {0.0, 0.5, 1.0, 2.0, 5.0}).axis("rot_prior_deg", {0.5, 2.0, 5.0});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"rot_err_after_estimated_deg", "nis_per_dof_estimated", "nis_per_dof_fixed", "nis_per_dof_folded"};
    }
    /// The true-pose scene with the believed extrinsic `rot_error_deg` off (and
    /// 1 cm), S10 prior `rot_prior_deg` (and 2 cm).
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double e = p.at("rot_error_deg"), pr = p.at("rot_prior_deg");
        if (!(e >= 0.0 && e <= 30.0) || !(pr > 0.0 && pr <= 30.0))
            throw std::invalid_argument("s10 bench sweep: rot_error_deg in [0, 30], rot_prior_deg in (0, 30]");
        const auto [in, truth] = scenario(true, e, pr);
        const auto inT = decode_input<T>(encode_input(in));
        std::vector<double> cols;
        const auto est = run<T>(inT, kShipped);
        const auto err = calib_error(unpack_so3<double>(truth.at("R_imu_cam")),
                                     unpack_fixed<double, 3>(truth.at("p_imu_cam")),
                                     est.R_ic,
                                     est.p_ic);
        cols.push_back(rot_deg(err));
        for (const char* v : {"shipped", "fixed", "folded_into_r"}) {
            const auto o = run<T>(inT, v);
            cols.push_back(o.dof_sum > 0 ? o.nis_sum / o.dof_sum : inv::detail::kInf);
        }
        return cols;
    }

    /// The scene with S10 applied to the fresh state: the believed extrinsic is
    /// the true one rotated by `rot_error_deg` about a fixed axis and shifted
    /// 1 cm; the prior is (`rot_prior_deg`, 2 cm). Returns the input and the
    /// truth (the world's extrinsic).
    [[nodiscard]] static std::pair<Input<double>, json>
    scenario(bool true_poses, double rot_error_deg, double rot_prior_deg) {
        namespace st = sdk::msckf::stages;
        using St = s6::State<double>;
        const auto world_ex = s6::build_scene(true_poses, 2, 1).extrinsics;  // the world's (true) extrinsic
        const double ax[3] = {0.6, -0.48, 0.64};                             // a unit axis
        const double a = rot_error_deg * kDeg;
        const typename St::CalibState believed{world_ex.R_imu_cam * St::SO3::exp({{a * ax[0], a * ax[1], a * ax[2]}}),
                                               world_ex.p_imu_cam + s6::Vec3<double>{{0.01, 0.0, 0.0}}};
        Input<double> in;
        in.rot_sigma = rot_prior_deg * kDeg;
        in.trans_sigma = 0.02;
        const auto sc = s6::build_scene(true_poses, 6, 16, [&](St& s) {
            st::s10_online_calibration::apply(s, {believed}, in.rot_sigma, in.trans_sigma);
        });
        in.state = sc.state;
        in.tracks = sc.tracks;
        json truth{{"R_imu_cam", pack(world_ex.R_imu_cam)}, {"p_imu_cam", pack(world_ex.p_imu_cam)}};
        return {in, truth};
    }

private:
    /// The state without its calibration block: the block is dropped from P
    /// (it is uncorrelated until an update) and the calibration list cleared.
    template <class T>
    [[nodiscard]] static s6::State<T> drop_calibration(const s6::State<T>& s) {
        using St = s6::State<T>;
        St o = s;
        const std::size_t off = St::kImuDim, cd = s.calib_dim(), d = s.dim();
        s6::Mat<T> P(d - cd, d - cd);
        for (std::size_t i = 0, a = 0; i < d; ++i) {
            if (i >= off && i < off + cd)
                continue;
            for (std::size_t j = 0, b = 0; j < d; ++j) {
                if (j >= off && j < off + cd)
                    continue;
                P(a, b++) = s.cov.P(i, j);
            }
            ++a;
        }
        o.calib.clear();
        o.cov.P = std::move(P);
        return o;
    }

    /// The error of an estimated extrinsic against the truth, in the error
    /// state's convention: δθ = Log(R_trueᵀ·R), δp = p − p_true.
    template <class T>
    [[nodiscard]] static std::vector<double> calib_error(const math::lie::SO3<double>& Rt,
                                                         const s6::Vec3<double>& pt,
                                                         const math::lie::SO3<T>& R,
                                                         const s6::Vec3<T>& p) {
        const auto q = R.quaternion();
        const math::lie::SO3<double> Rd(typename math::lie::SO3<double>::Quaternion{{static_cast<double>(q[0]),
                                                                                     static_cast<double>(q[1]),
                                                                                     static_cast<double>(q[2]),
                                                                                     static_cast<double>(q[3])}});
        const auto th = (Rt.inverse() * Rd).log();
        return {th[0],
                th[1],
                th[2],
                static_cast<double>(p[0]) - pt[0],
                static_cast<double>(p[1]) - pt[1],
                static_cast<double>(p[2]) - pt[2]};
    }
    [[nodiscard]] static double rot_deg(const std::vector<double>& e) {
        return std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]) / kDeg;
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S10_ONLINE_CALIBRATION_BENCH_HPP
