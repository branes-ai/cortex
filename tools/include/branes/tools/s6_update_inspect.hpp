// SPDX-License-Identifier: MIT
//
// branes/tools/s6_update_inspect.hpp — the S6 (MSCKF measurement update)
// inspector (epic #371, issue #380). The real-data companion to the synthetic S6
// probe (sdk/eval/update_probe.hpp); establishes the filter-internal renderer
// tier, built on the S4 inspector template (issue #374).
//
// S6 is the consistency-critical stage — where the filter's covariance is won or
// lost, and where the global #212 symptom (NEES ≫ dim) shows its LOCAL mirror:
// NIS per update. The real operator is the shipped `CameraUpdater::update`
// (triangulate → per-obs Jacobians → left-null-space projection → innovation S →
// χ² gate → Joseph covariance update). It returns only a NisSample; this
// inspector turns one observed update into a full record — NIS/dof, the χ²-gate
// decision, the per-observation reprojection residual, the covariance trace
// before/after (is the filter shrinking its uncertainty, and is it staying PSD?).
//
// Two entry points, one record builder:
//   • record(state_after, track, nis, accepted, cov_before) — the backend-observer
//     path: MsckfBackend::set_update_observer feeds REAL updates from a live run.
//   • run(S6Input) — the self-contained path: build a state from (clones, P, track),
//     run the REAL update, and record it. Unit-testable on the probe's
//     self-consistent (P, R) scene without the ~1.5 GB EuRoC dataset.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_S6_UPDATE_INSPECT_HPP
#define BRANES_TOOLS_S6_UPDATE_INSPECT_HPP

#include <branes/math/lie/se3.hpp>
#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace branes::tools {

/// One observation's reprojection residual at the triangulated landmark (px).
struct S6ObsResidual {
    std::uint32_t clone_index = 0;
    double ru_px = 0.0, rv_px = 0.0;  ///< residual components (observed − reprojected)
    double err_px = 0.0;              ///< ‖residual‖
};

/// Everything one MSCKF update produced — the inspector's record.
struct S6UpdateRecord {
    std::uint32_t n_obs = 0;        ///< observations in the track
    std::uint32_t dof = 0;          ///< projected residual dimension (2m−3)
    double nis = 0.0;               ///< νᵀS⁻¹ν
    double nis_over_dof = 0.0;      ///< the headline (target 1; >1 ⇒ over-confident)
    double innov_sum = 0.0;         ///< Σ rₖ/σ — innovation zero-mean test
    bool valid = false;             ///< NIS well-conditioned (S invertible)
    bool accepted = false;          ///< update applied (covariance changed)
    bool gated = false;             ///< valid NIS but rejected ⇒ χ²-gated out
    double chi2_threshold = 0.0;    ///< chi2_per_dof · dof — the gate line
    double cov_trace_before = 0.0;  ///< tr(P) before the update
    double cov_trace_after = 0.0;   ///< tr(P) after (== before if rejected)
    double cov_trace_ratio = 1.0;   ///< after / before (<1 ⇒ uncertainty shrank)
    bool psd_after = true;          ///< P stayed positive-definite (Joseph guarantee)
    double residual_rms_px = 0.0;   ///< RMS reprojection residual at the landmark
    std::vector<S6ObsResidual> residuals;
};

[[nodiscard]] inline nlohmann::json to_json(const S6UpdateRecord& r) {
    using nlohmann::json;
    json res = json::array();
    for (const auto& o : r.residuals)
        res.push_back(json{{"clone", o.clone_index}, {"ru", o.ru_px}, {"rv", o.rv_px}, {"err", o.err_px}});
    return json{{"n_obs", r.n_obs},
                {"dof", r.dof},
                {"nis", r.nis},
                {"nis_over_dof", r.nis_over_dof},
                {"innov_sum", r.innov_sum},
                {"valid", r.valid},
                {"accepted", r.accepted},
                {"gated", r.gated},
                {"chi2_threshold", r.chi2_threshold},
                {"cov_trace_before", r.cov_trace_before},
                {"cov_trace_after", r.cov_trace_after},
                {"cov_trace_ratio", r.cov_trace_ratio},
                {"psd_after", r.psd_after},
                {"residual_rms_px", r.residual_rms_px},
                {"residuals", std::move(res)}};
}

/// Runs the shipped MSCKF update and characterizes it. Constructed with the same
/// extrinsics + options the live backend uses, so the residual triangulation and
/// the gate threshold match the real path exactly.
class S6UpdateInspector {
public:
    using Extrinsics = branes::sdk::msckf::CameraExtrinsics<double>;
    using Options = branes::sdk::msckf::CameraUpdaterOptions<double>;
    using State = branes::sdk::msckf::State<double>;
    using FeatureTrack = branes::sdk::msckf::FeatureTrack<double>;
    using NisSample = branes::sdk::msckf::NisSample<double>;
    using DynMat = branes::sdk::msckf::DynMat<double>;
    using SE3 = branes::math::lie::SE3<double>;

    explicit S6UpdateInspector(std::vector<Extrinsics> cams = std::vector<Extrinsics>(1),
                               Options opts = {},
                               double focal_px = 458.654)
        : upd_(std::move(cams), opts), opts_(opts), focal_px_(focal_px) {}

    /// Decoupled S6 input: a clone-pose window, a believed covariance P, and the
    /// feature track to update against. (Extrinsics + options live in the inspector.)
    struct S6Input {
        std::vector<SE3> clone_poses;
        DynMat P;
        FeatureTrack track;
    };

    /// Self-contained path: build a state from `in`, run the REAL update, record it.
    [[nodiscard]] S6UpdateRecord run(const S6Input& in) const {
        State s(1.0);
        s.clones.clear();
        for (const auto& cp : in.clone_poses)
            s.clones.push_back({cp.rotation(), cp.translation(), 0.0});
        s.cov.P = in.P;

        DynMat cov_before = s.covariance();
        NisSample nis;
        const bool accepted = upd_.update(s, in.track, &nis);
        return record(s, in.track, nis, accepted, cov_before);
    }

    /// Backend-observer path: turn one observed update into a record.
    [[nodiscard]] S6UpdateRecord record(const State& state_after,
                                        const FeatureTrack& track,
                                        const NisSample& nis,
                                        bool accepted,
                                        const DynMat& cov_before) const {
        namespace ms = branes::sdk::msckf;
        S6UpdateRecord r;
        r.n_obs = static_cast<std::uint32_t>(track.observations.size());
        r.dof = static_cast<std::uint32_t>(nis.dof);
        r.nis = static_cast<double>(nis.value);
        r.nis_over_dof = nis.dof > 0 ? r.nis / static_cast<double>(nis.dof) : 0.0;
        r.innov_sum = static_cast<double>(nis.innov_sum);
        r.valid = nis.valid;
        r.accepted = accepted;
        r.gated = nis.valid && !accepted;  // well-conditioned but rejected ⇒ χ²-gated
        r.chi2_threshold = opts_.chi2_per_dof * static_cast<double>(nis.dof);

        r.cov_trace_before = trace(cov_before);
        const DynMat cov_after = state_after.covariance();
        r.cov_trace_after = trace(cov_after);
        r.cov_trace_ratio = r.cov_trace_before > 0.0 ? r.cov_trace_after / r.cov_trace_before : 1.0;
        r.psd_after = ms::is_positive_definite(cov_after);

        // Per-observation reprojection residual at the triangulated landmark.
        branes::math::lie::detail::Vec<double, 3> p_f{};
        if (upd_.triangulate(state_after, track.observations, p_f)) {
            double sse = 0.0;
            std::size_t n = 0;
            for (const auto& o : track.observations) {
                const auto J = upd_.projection_jacobians(state_after, o, p_f);
                if (!J.valid)
                    continue;
                const double ru = (o.xy[0] - J.h[0]) * focal_px_;
                const double rv = (o.xy[1] - J.h[1]) * focal_px_;
                const double err = std::sqrt(ru * ru + rv * rv);
                r.residuals.push_back(S6ObsResidual{static_cast<std::uint32_t>(o.clone_index), ru, rv, err});
                sse += ru * ru + rv * rv;
                ++n;
            }
            if (n > 0)
                r.residual_rms_px = std::sqrt(sse / static_cast<double>(2 * n));
        }
        return r;
    }

private:
    [[nodiscard]] static double trace(const branes::sdk::msckf::DynMat<double>& m) {
        double t = 0.0;
        const std::size_t n = m.rows < m.cols ? m.rows : m.cols;
        for (std::size_t i = 0; i < n; ++i)
            t += m(i, i);
        return t;
    }

    branes::sdk::msckf::CameraUpdater<double> upd_;
    Options opts_;
    double focal_px_;
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_S6_UPDATE_INSPECT_HPP
