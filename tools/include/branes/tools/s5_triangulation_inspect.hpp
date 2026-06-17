// SPDX-License-Identifier: MIT
//
// branes/tools/s5_triangulation_inspect.hpp — the S5 (feature triangulation)
// inspector (epic #371, issue #379). The real-data companion to the synthetic
// S5 contract probe (sdk/eval/triangulation_probe.hpp); establishes the 3-D
// renderer tier, built on the S4 inspector template (issue #374).
//
// An inspector re-runs the REAL stage operator on real data and exposes the
// quantities the production path hides, for study. For S5 the operator is the
// shipped `CameraUpdater::triangulate` (multi-view linear DLT + Gauss-Newton
// reprojection refinement). The hidden quantities are the landmark POSITION
// COVARIANCE, the inter-view PARALLAX, the system CONDITION NUMBER, and the
// per-observation REPROJECTION RESIDUALS — none of which the update path returns.
//
// Triangulation is decoupled behind the `trace::S5Input` / `trace::S5Output`
// I/O struct: the inspector takes the clone-pose window + feature tracks and
// produces the landmark cloud with uncertainty. The uncertainty is computed
// analytically, reusing the operator's OWN reprojection Jacobian: the
// Gauss-Newton information matrix is H = Σ Hfᵀ Hf, so the world-position
// covariance is σ_norm² · H⁻¹ with σ_norm the per-observation pixel noise in
// normalized units. This is the same linearization the filter would inherit,
// made explicit.
//
// The computation lives in this header (not the .cpp driver) so it is
// unit-testable on synthetic two-view geometry without the ~1.5 GB EuRoC dataset.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_S5_TRIANGULATION_INSPECT_HPP
#define BRANES_TOOLS_S5_TRIANGULATION_INSPECT_HPP

#include <branes/sdk/msckf/camera_updater.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/tools/vio_trace.hpp>  // trace::S5Input / S5Output (the decoupled S5 I/O struct)

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace branes::tools {

namespace s5_detail {

using Vec3 = branes::math::lie::detail::Vec<double, 3>;

/// Symmetric 3×3 eigenvalues via cyclic Jacobi (mirrors triangulation_probe's
/// sym3_eigs) — for the condition number of the triangulation information matrix.
inline std::array<double, 3> sym3_eigs(std::array<std::array<double, 3>, 3> a) {
    for (int sweep = 0; sweep < 24; ++sweep) {
        int p = 0, q = 1;
        double off = std::abs(a[0][1]);
        if (std::abs(a[0][2]) > off) {
            off = std::abs(a[0][2]);
            p = 0;
            q = 2;
        }
        if (std::abs(a[1][2]) > off) {
            off = std::abs(a[1][2]);
            p = 1;
            q = 2;
        }
        if (off < 1e-18)
            break;
        const double phi = 0.5 * std::atan2(2.0 * a[p][q], a[q][q] - a[p][p]);
        const double c = std::cos(phi), s = std::sin(phi);
        for (int k = 0; k < 3; ++k) {
            const double akp = a[k][p], akq = a[k][q];
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {
            const double apk = a[p][k], aqk = a[q][k];
            a[p][k] = c * apk - s * aqk;
            a[q][k] = s * apk + c * aqk;
        }
    }
    return {a[0][0], a[1][1], a[2][2]};
}

/// Inverse of a symmetric positive-definite 3×3 (row-major) via cofactors.
/// Returns false if (near-)singular. Used to turn the Gauss-Newton information
/// matrix H into the position covariance H⁻¹.
inline bool inv3_sym(const std::array<double, 9>& m, std::array<double, 9>& out) {
    const double a = m[0], b = m[1], c = m[2], d = m[4], e = m[5], f = m[8];
    // [[a b c],[b d e],[c e f]]
    const double A = d * f - e * e;
    const double B = c * e - b * f;
    const double C = b * e - c * d;
    const double det = a * A + b * B + c * C;
    if (!(std::abs(det) > 0.0))
        return false;
    const double inv = 1.0 / det;
    const double D = a * f - c * c;
    const double E = b * c - a * e;
    const double F = a * d - b * b;
    out[0] = A * inv;
    out[1] = B * inv;
    out[2] = C * inv;
    out[3] = B * inv;
    out[4] = D * inv;
    out[5] = E * inv;
    out[6] = C * inv;
    out[7] = E * inv;
    out[8] = F * inv;
    return true;
}

}  // namespace s5_detail

/// Build the MSCKF state + updater a triangulation needs from the decoupled S5
/// input, then drive the SHIPPED operator and characterize each landmark.
class S5TriangulationInspector {
public:
    /// `focal_px` converts the `px_noise` (px) observation noise into the
    /// normalized units the covariance scaling needs; `min_obs` skips tracks too
    /// short to triangulate.
    explicit S5TriangulationInspector(double focal_px = 458.654, double px_noise = 1.0, std::uint32_t min_obs = 2)
        : focal_px_(focal_px), px_noise_(px_noise), min_obs_(min_obs) {}

    /// Triangulate every track in `in`, returning the landmark cloud with
    /// per-landmark position covariance, parallax, condition number and
    /// reprojection RMS. Tracks with fewer than `min_obs` observations, or where
    /// the shipped triangulate() reports failure, are emitted with success=false.
    [[nodiscard]] trace::S5Output<double> run(const trace::S5Input<double>& in) const {
        namespace ms = branes::sdk::msckf;
        using SO3 = branes::math::lie::SO3<double>;

        // Extrinsics for the updater (camera_id → T_imu_cam).
        std::vector<ms::CameraExtrinsics<double>> cams;
        cams.reserve(in.extrinsics.empty() ? 1 : in.extrinsics.size());
        if (in.extrinsics.empty()) {
            cams.emplace_back();  // identity (camera == IMU)
        } else {
            for (const auto& e : in.extrinsics)
                cams.push_back(ms::CameraExtrinsics<double>{e.rotation(), e.translation()});
        }
        const ms::CameraUpdater<double> upd(cams);

        // One shared state holding the whole clone window.
        ms::State<double> s(1.0);
        for (const auto& cp : in.clone_poses)
            s.clones.push_back({cp.rotation(), cp.translation(), 0.0});

        trace::S5Output<double> out;
        out.landmarks.reserve(in.tracks.size());
        for (const auto& tr : in.tracks)
            out.landmarks.push_back(triangulate_one(upd, s, cams, tr));
        return out;
    }

private:
    using Obs = branes::sdk::msckf::CameraObservation<double>;
    using Extrinsics = branes::sdk::msckf::CameraExtrinsics<double>;

    [[nodiscard]] trace::S5Landmark<double> triangulate_one(const branes::sdk::msckf::CameraUpdater<double>& upd,
                                                            const branes::sdk::msckf::State<double>& s,
                                                            const std::vector<Extrinsics>& cams,
                                                            const trace::S5Track<double>& tr) const {
        trace::S5Landmark<double> lm;
        lm.feature_id = tr.feature_id;
        lm.n_obs = static_cast<std::uint32_t>(tr.obs.size());
        if (tr.obs.size() < min_obs_)
            return lm;

        std::vector<Obs> obs;
        obs.reserve(tr.obs.size());
        for (const auto& o : tr.obs)
            obs.push_back(Obs{o.clone_index, o.camera_id, {o.xy[0], o.xy[1]}});

        s5_detail::Vec3 p_f{};
        lm.success = upd.triangulate(s, obs, p_f);
        if (!lm.success)
            return lm;
        lm.p_world = p_f;

        // Reprojection residuals + the Gauss-Newton information matrix H = Σ Hfᵀ Hf,
        // reusing the operator's own projection Jacobian (project∘extrinsics).
        std::array<std::array<double, 3>, 3> H{};
        double sse = 0.0;
        std::size_t nres = 0;
        for (const auto& o : obs) {
            const auto J = upd.projection_jacobians(s, o, p_f);
            if (!J.valid)
                continue;
            const double rx = o.xy[0] - J.h[0], ry = o.xy[1] - J.h[1];
            sse += rx * rx + ry * ry;
            ++nres;
            for (std::size_t a = 0; a < 3; ++a)
                for (std::size_t b = 0; b < 3; ++b)
                    H[a][b] += J.Hf(0, a) * J.Hf(0, b) + J.Hf(1, a) * J.Hf(1, b);
        }
        if (nres > 0)
            lm.reproj_rms_px = std::sqrt(sse / static_cast<double>(2 * nres)) * focal_px_;

        // Covariance = σ_norm² · H⁻¹  (σ_norm = px noise in normalized units).
        const std::array<double, 9> Hflat{
            H[0][0], H[0][1], H[0][2], H[1][0], H[1][1], H[1][2], H[2][0], H[2][1], H[2][2]};
        std::array<double, 9> Hinv{};
        const double sigma_norm = px_noise_ / focal_px_;
        if (s5_detail::inv3_sym(Hflat, Hinv)) {
            for (std::size_t i = 0; i < 9; ++i)
                lm.cov[i] = sigma_norm * sigma_norm * Hinv[i];
        }
        const auto eigs = s5_detail::sym3_eigs(H);
        double lmax = eigs[0], lmin = eigs[0];
        for (double e : eigs) {
            lmax = std::max(lmax, e);
            lmin = std::min(lmin, e);
        }
        lm.condition_number = lmin > 0.0 ? lmax / lmin : std::numeric_limits<double>::infinity();
        lm.parallax_deg = max_parallax_deg(s, cams, obs);
        return lm;
    }

    /// Max inter-view parallax angle (deg) of the world-frame bearings — the
    /// depth-observability metric the synthetic probe sweeps. The observation ray
    /// is in the CAMERA frame, so it must be rotated through both the extrinsic
    /// (R_imu_cam) and the clone (R_world_imu) to reach world: R_wi · R_ic · ray.
    /// Omitting R_imu_cam skews the angle for a non-trivial camera-IMU alignment
    /// (EuRoC's is ~90°).
    [[nodiscard]] static double max_parallax_deg(const branes::sdk::msckf::State<double>& s,
                                                 const std::vector<Extrinsics>& cams,
                                                 const std::vector<Obs>& obs) {
        std::vector<s5_detail::Vec3> dirs;
        dirs.reserve(obs.size());
        for (const auto& o : obs) {
            const auto& cl = s.clones[o.clone_index];
            const auto& ex = cams[o.camera_index < cams.size() ? o.camera_index : 0];
            const s5_detail::Vec3 ray_imu = ex.R_imu_cam.matrix() * s5_detail::Vec3{{o.xy[0], o.xy[1], 1.0}};
            s5_detail::Vec3 d = cl.R.matrix() * ray_imu;
            const double dn = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            if (!(dn > 0.0))
                continue;
            dirs.push_back(d * (1.0 / dn));
        }
        double min_dot = 1.0;
        for (std::size_t i = 0; i < dirs.size(); ++i)
            for (std::size_t j = i + 1; j < dirs.size(); ++j) {
                const double dot = dirs[i][0] * dirs[j][0] + dirs[i][1] * dirs[j][1] + dirs[i][2] * dirs[j][2];
                min_dot = std::min(min_dot, dot);
            }
        const double clamped = std::max(-1.0, std::min(1.0, min_dot));
        return std::acos(clamped) * (180.0 / 3.14159265358979323846);
    }

    double focal_px_;
    double px_noise_;
    std::uint32_t min_obs_;
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_S5_TRIANGULATION_INSPECT_HPP
