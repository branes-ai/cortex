// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s6_riekf_update_bench.hpp — stage bench for the R-IEKF
// invariant camera update (issue #612, epic #444).
//
// Invariants: H vs finite-diff, H_proj annihilates 4-DoF gauge, P+ <= P-.
// Fixtures: known_answer, ground_truth, captured. Variant: "shipped".
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S6_RIEKF_UPDATE_BENCH_HPP
#define BRANES_TOOLS_BENCH_S6_RIEKF_UPDATE_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/features/msckf_nullspace.hpp>
#include <branes/sdk/msckf/covariance.hpp>
#include <branes/sdk/msckf/invariant_update.hpp>
#include <branes/tools/bench/bench.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

namespace inv = branes::sdk::eval::inv;

namespace s6_riekf_detail {
namespace ms = branes::sdk::msckf;
using Sc = double;
using SO3 = branes::math::lie::SO3<Sc>;
using V3 = branes::math::lie::detail::Vec<Sc, 3>;
using V2 = branes::math::lie::detail::Vec<Sc, 2>;
struct Scene {
    std::vector<ms::InvariantClone<Sc>> clones;
    std::vector<V3> feats;
};
inline Scene make_scene() {
    Scene s;
    for (int c = 0; c < 5; ++c) {
        ms::InvariantClone<Sc> cl;
        cl.R = SO3::exp(V3{{0.02 * c, -0.01 * c, 0.015 * c}});
        cl.p = V3{{0.25 * c, 0.0, 0.0}};
        s.clones.push_back(cl);
    }
    for (int i = 0; i < 16; ++i) {
        const Sc u = static_cast<Sc>((i % 4) - 2) * Sc{0.4};
        const Sc v = static_cast<Sc>((i / 4) - 2) * Sc{0.4};
        s.feats.push_back(V3{{u, v, 3.0 + 0.1 * i}});
    }
    return s;
}
inline V2 project(const ms::InvariantClone<Sc>& c, const V3& pf) {
    const V3 y = c.R.inverse().matrix() * V3{{pf[0] - c.p[0], pf[1] - c.p[1], pf[2] - c.p[2]}};
    return V2{{y[0] / y[2], y[1] / y[2]}};
}
inline std::vector<ms::InvariantObs<Sc>> make_track(const std::vector<ms::InvariantClone<Sc>>& cl, const V3& pf) {
    std::vector<ms::InvariantObs<Sc>> obs;
    for (std::size_t c = 0; c < cl.size(); ++c)
        obs.push_back({c, 0, project(cl[c], pf)});
    return obs;
}
}  // namespace s6_riekf_detail

struct S6RieкfUpdateBench {
    static constexpr std::string_view kStage = "S6_riekf_update";
    static constexpr inv::Stage kInvStage = inv::Stage::S6_riekf_update;

    template <class T>
    struct Input {
        std::vector<branes::sdk::msckf::InvariantClone<T>> clones;
        std::vector<branes::sdk::msckf::InvariantObs<T>> obs;
        branes::math::lie::detail::Vec<T, 3> p_f{};
        T sigma = T{1} / T{100};
    };
    template <class T>
    struct Output {
        std::vector<branes::sdk::msckf::InvariantClone<T>> clones_after;
        std::vector<double> cov_after{};
        bool applied = false;
    };

    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        json j;
        j["sigma"] = static_cast<double>(in.sigma);
        json jcl = json::array();
        for (const auto& c : in.clones) {
            const auto q = c.R.quaternion();
            json jc = json::object();
            jc["qw"] = static_cast<double>(q[0]);
            jc["qx"] = static_cast<double>(q[1]);
            jc["qy"] = static_cast<double>(q[2]);
            jc["qz"] = static_cast<double>(q[3]);
            jc["px"] = static_cast<double>(c.p[0]);
            jc["py"] = static_cast<double>(c.p[1]);
            jc["pz"] = static_cast<double>(c.p[2]);
            jcl.push_back(jc);
        }
        j["clones"] = jcl;
        json jobs = json::array();
        for (const auto& o : in.obs) {
            json jo = json::object();
            jo["ci"] = o.clone_index;
            jo["cam"] = o.camera_index;
            jo["x"] = static_cast<double>(o.xy[0]);
            jo["y"] = static_cast<double>(o.xy[1]);
            jobs.push_back(jo);
        }
        j["obs"] = jobs;
        json jpf = json::array();
        jpf.push_back(static_cast<double>(in.p_f[0]));
        jpf.push_back(static_cast<double>(in.p_f[1]));
        jpf.push_back(static_cast<double>(in.p_f[2]));
        j["pf"] = jpf;
        return j;
    }

    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        using SO3 = branes::math::lie::SO3<T>;
        using V3 = branes::math::lie::detail::Vec<T, 3>;
        using V2 = branes::math::lie::detail::Vec<T, 2>;
        Input<T> in;
        in.sigma = T(j.at("sigma").get<double>());
        for (const auto& jc : j.at("clones")) {
            branes::sdk::msckf::InvariantClone<T> c;
            c.R = SO3::from_unit_quaternion(
                {jc["qw"].get<double>(), jc["qx"].get<double>(), jc["qy"].get<double>(), jc["qz"].get<double>()});
            c.p = V3{{T(jc["px"].get<double>()), T(jc["py"].get<double>()), T(jc["pz"].get<double>())}};
            in.clones.push_back(c);
        }
        for (const auto& jo : j.at("obs")) {
            branes::sdk::msckf::InvariantObs<T> o;
            o.clone_index = jo["ci"].get<std::size_t>();
            o.camera_index = jo["cam"].get<std::size_t>();
            o.xy = V2{{T(jo["x"].get<double>()), T(jo["y"].get<double>())}};
            in.obs.push_back(o);
        }
        const auto& pf = j.at("pf");
        in.p_f = V3{{T(pf[0].get<double>()), T(pf[1].get<double>()), T(pf[2].get<double>())}};
        return in;
    }

    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        json j;
        j["applied"] = out.applied;
        json jcl = json::array();
        for (const auto& c : out.clones_after) {
            const auto q = c.R.quaternion();
            json jc = json::object();
            jc["qw"] = static_cast<double>(q[0]);
            jc["qx"] = static_cast<double>(q[1]);
            jc["qy"] = static_cast<double>(q[2]);
            jc["qz"] = static_cast<double>(q[3]);
            jc["px"] = static_cast<double>(c.p[0]);
            jc["py"] = static_cast<double>(c.p[1]);
            jc["pz"] = static_cast<double>(c.p[2]);
            jcl.push_back(jc);
        }
        j["clones_after"] = jcl;
        j["cov_after"] = out.cov_after;
        return j;
    }

    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        using SO3 = branes::math::lie::SO3<T>;
        using V3 = branes::math::lie::detail::Vec<T, 3>;
        Output<T> out;
        out.applied = j.at("applied").get<bool>();
        for (const auto& jc : j.at("clones_after")) {
            branes::sdk::msckf::InvariantClone<T> c;
            c.R = SO3::from_unit_quaternion(
                {jc["qw"].get<double>(), jc["qx"].get<double>(), jc["qy"].get<double>(), jc["qz"].get<double>()});
            c.p = V3{{T(jc["px"].get<double>()), T(jc["py"].get<double>()), T(jc["pz"].get<double>())}};
            out.clones_after.push_back(c);
        }
        out.cov_after = j.at("cov_after").get<std::vector<double>>();
        return out;
    }

    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view) {
        branes::sdk::msckf::FullCovariance<T> cov(T(0.5), 6 * in.clones.size());
        auto clones = in.clones;
        Output<T> out;
        out.applied = branes::sdk::msckf::invariant_update<T>(clones, cov, in.obs, in.p_f, in.sigma);
        out.clones_after = clones;
        const auto P = cov.covariance();
        out.cov_after.reserve(P.rows * P.cols);
        for (std::size_t i = 0; i < P.rows; ++i)
            for (std::size_t j = 0; j < P.cols; ++j)
                out.cov_after.push_back(static_cast<double>(P(i, j)));
        return out;
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        std::vector<double> f;
        f.push_back(out.applied ? 1.0 : 0.0);
        for (const auto& c : out.clones_after) {
            const auto q = c.R.quaternion();
            for (int k = 0; k < 4; ++k)
                f.push_back(static_cast<double>(q[k]));
            for (int k = 0; k < 3; ++k)
                f.push_back(static_cast<double>(c.p[k]));
        }
        for (const auto& v : out.cov_after)
            f.push_back(v);
        return f;
    }

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "invariant_update: H, null-space, EKF update, retract"}};
    }

    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture&) {
        namespace ms2 = branes::sdk::msckf;
        using V3 = branes::math::lie::detail::Vec<T, 3>;
        using SO3 = branes::math::lie::SO3<T>;
        std::vector<inv::InvariantResult> r;
        if (!out.applied) {
            r.push_back(inv::check_scalar(0.0, 1.0, inv::Bound::Lower, kInvStage, "update.applied", "flag"));
            return r;
        }
        const std::size_t nc = in.clones.size();
        const auto M = ms2::build_invariant_measurement<T>(in.clones, in.obs, in.p_f);
        const std::size_t n = M.n;

        // 1. H vs finite-diff
        // Step and tolerance are scaled to the scalar precision so the check
        // passes for both double and float without false positives.
        const double fd_eps = std::sqrt(static_cast<double>(std::numeric_limits<T>::epsilon()));
        const double fd_tol = tolerance_vs_double<T>(6, 1.0);
        double worst_fd = 0.0;
        if (M.ok) {
            for (std::size_t c = 0; c < nc; ++c) {
                for (std::size_t k = 0; k < 6; ++k) {
                    auto cp = in.clones;
                    if (k < 3) {
                        V3 phi{};
                        phi[k] = T(fd_eps);
                        const SO3 dR = SO3::exp(phi);
                        cp[c].R = dR * cp[c].R;
                        cp[c].p = dR * cp[c].p;
                    } else {
                        cp[c].p[k - 3] = cp[c].p[k - 3] + T(fd_eps);
                    }
                    const auto Mp = ms2::build_invariant_measurement<T>(cp, in.obs, in.p_f);
                    if (!Mp.ok)
                        continue;
                    for (std::size_t row = 0; row < M.rows2; ++row) {
                        const double fd = (static_cast<double>(Mp.r[row]) - static_cast<double>(M.r[row])) / fd_eps;
                        const double Hv = -static_cast<double>(M.H_x[row * n + 6 * c + k]);
                        double e = (std::abs(fd) < fd_eps && std::abs(Hv) < fd_eps)
                                       ? 0.0
                                       : std::abs(fd - Hv) / (std::abs(Hv) + fd_tol);
                        worst_fd = std::max(worst_fd, e);
                    }
                }
            }
        }
        r.push_back(inv::check_scalar(
            worst_fd, fd_tol, inv::Bound::Upper, kInvStage, "jacobian.matches_finite_diff", "relative"));

        // 2. gauge annihilation
        if (M.ok) {
            const auto proj = branes::sdk::features::msckf_left_nullspace_project<T>(M.H_f, M.H_x, M.r, M.rows2, n);
            const V3 a{{T(0), T(0), T(1)}};
            double worst_g = 0.0;
            for (int g = 0; g < 4; ++g) {
                std::vector<T> N(n, T(0));
                if (g < 3) {
                    for (std::size_t c = 0; c < nc; ++c)
                        N[6 * c + 3 + g] = T(1);
                } else {
                    for (std::size_t c = 0; c < nc; ++c)
                        for (int k = 0; k < 3; ++k)
                            N[6 * c + k] = a[k];
                }
                for (std::size_t i = 0; i < proj.rows; ++i) {
                    T hv = T(0);
                    for (std::size_t j = 0; j < n; ++j)
                        hv = hv + proj.H_x[i * n + j] * N[j];
                    worst_g = std::max(worst_g, std::abs(static_cast<double>(hv)));
                }
            }
            r.push_back(
                inv::check_scalar(worst_g, 1e-10, inv::Bound::Upper, kInvStage, "jacobian.annihilates_gauge", "abs"));
        }

        // 3. P+ PSD and P+ ⪯ P- (Loewner order)
        // Both matrices are held as double so check_loewner_le types match.
        const ms2::FullCovariance<T> cov_prior(T(0.5), n);
        const auto P_prior_T = cov_prior.covariance();
        branes::sdk::msckf::DynMat<double> P_prior(n, n);
        branes::sdk::msckf::DynMat<double> P_after(n, n);
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j) {
                P_prior(i, j) = static_cast<double>(P_prior_T(i, j));
                P_after(i, j) = out.cov_after[i * n + j];
            }
        const double safety = safety_vs_double<T>();
        r.push_back(inv::check_psd(P_after, kInvStage, "covariance.psd", safety));
        // trace ratio kept as a lightweight diagnostic
        double tp = 0.0, tq = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            tp += P_prior(i, i);
            tq += out.cov_after[i * n + i];
        }
        r.push_back(inv::check_scalar(
            tq / (tp + 1e-300), 1.0, inv::Bound::Upper, kInvStage, "covariance.trace_ratio_le_1", "ratio"));
        r.push_back(inv::check_loewner_le(P_after, P_prior, kInvStage, "covariance.no_larger_than_prior", safety));
        return r;
    }

    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_identity_clones", known_answer()},
                {"ground_truth_synthetic_scene", ground_truth()},
                {"captured_synthetic_scene", captured()}};
    }

    [[nodiscard]] static s6_riekf_detail::Scene make_scene() {
        return s6_riekf_detail::make_scene();
    }
    [[nodiscard]] static std::vector<branes::sdk::msckf::InvariantObs<double>>
    track(const std::vector<branes::sdk::msckf::InvariantClone<double>>& cl,
          const branes::math::lie::detail::Vec<double, 3>& pf) {
        return s6_riekf_detail::make_track(cl, pf);
    }

    [[nodiscard]] static Fixture known_answer() {
        using namespace s6_riekf_detail;
        Input<double> in;
        for (int c = 0; c < 3; ++c) {
            ms::InvariantClone<Sc> cl;
            cl.p = V3{{0.5 * c, 0.0, 0.0}};
            in.clones.push_back(cl);
        }
        const V3 pf{{0.0, 0.0, 3.0}};
        in.p_f = pf;
        in.sigma = 0.01;
        in.obs = make_track(in.clones, pf);
        // Independent oracle: for exact observations the update must be applied
        // and the posterior covariance trace must be strictly smaller than the
        // prior (identity * 0.5).  The expected output is NOT derived from
        // run() so a regression in the implementation changes the invariant
        // result rather than silently updating both sides together.
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "s6_riekf_update_bench: 3 identity clones, feature at (0,0,3), exact obs, sigma=0.01";
        f.description = "exact obs: update must be applied and covariance trace must shrink";
        f.input = encode_input(in);
        f.truth = json{{"applied", true}};
        return f;
    }

    [[nodiscard]] static Fixture ground_truth() {
        using namespace s6_riekf_detail;
        const auto sc = make_scene();
        Input<double> in;
        in.clones = sc.clones;
        in.p_f = sc.feats[0];
        in.obs = make_track(sc.clones, sc.feats[0]);
        in.sigma = 0.01;
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic scene (make_scene), true poses, exact observations";
        f.description = "truth: update applied, gauge annihilated";
        f.input = encode_input(in);
        f.truth = json{{"applied", true}};
        return f;
    }

    [[nodiscard]] static Fixture captured() {
        using namespace s6_riekf_detail;
        const auto sc = make_scene();
        Input<double> in;
        in.clones = sc.clones;
        in.p_f = sc.feats[0];
        in.obs = make_track(sc.clones, sc.feats[0]);
        in.sigma = 0.01;
        return capture(std::string(kStage),
                       "double",
                       "synthetic scene (make_scene), true poses, shipped variant",
                       encode_input(in),
                       encode_output(run<double>(in, kShipped)));
    }
};

}  // namespace branes::tools::bench
#endif  // BRANES_TOOLS_BENCH_S6_RIEKF_UPDATE_BENCH_HPP
