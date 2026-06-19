// SPDX-License-Identifier: MIT
//
// branes/tools/s10_calibration_inspect.hpp — the S10 (online calibration)
// inspector (epic #371, issue #384). The real-data companion to the synthetic S10
// probe (sdk/eval/calibration_probe.hpp); renders on the filter-internal tier,
// built on the S4 inspector template (issue #374).
//
// When online extrinsic estimation is on (`estimate_extrinsics`), the filter
// carries the camera↔IMU extrinsic T_CI as state (a per-camera CalibState block)
// and refines it with every measurement update. Seeded from a deliberately WRONG
// extrinsic, the estimate should CONVERGE toward the dataset's true calibration as
// the filter observes — and its covariance should shrink. The real operator is the
// shipped camera update on the calibration block; this inspector samples, per
// frame, the in-state estimate's error against a reference extrinsic and the
// estimate's σ, producing the convergence curve.
//
// `sample(state, ref)` is a pure read of the calibration block + its covariance,
// so it is unit-testable on a constructed state without the ~1.5 GB EuRoC dataset.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_S10_CALIBRATION_INSPECT_HPP
#define BRANES_TOOLS_S10_CALIBRATION_INSPECT_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace branes::tools {

/// One frame's reading of the online-calibration estimate vs a reference extrinsic.
struct S10CalibSample {
    double t = 0.0;
    double rot_err_deg = 0.0;     ///< geodesic angle between the estimated R_imu_cam and the reference
    double trans_err_mm = 0.0;    ///< ‖estimated p_imu_cam − reference‖ (mm)
    double rot_sigma_deg = 0.0;   ///< √trace of the extrinsic-rotation covariance block (deg)
    double trans_sigma_mm = 0.0;  ///< √trace of the extrinsic-translation covariance block (mm)
};

/// The whole convergence run.
struct S10CalibRecord {
    bool has_calib = false;          ///< the state actually carried a calibration block (online calib on)
    double ref_rot_deg_init = 0.0;   ///< the seeded extrinsic-rotation error at the start (perturbation)
    double ref_trans_mm_init = 0.0;  ///< the seeded extrinsic-translation error at the start
    std::vector<S10CalibSample> curve;
};

[[nodiscard]] inline nlohmann::json to_json(const S10CalibRecord& r) {
    using nlohmann::json;
    json curve = json::array();
    for (const auto& s : r.curve)
        curve.push_back(json{{"t", s.t},
                             {"rot_err_deg", s.rot_err_deg},
                             {"trans_err_mm", s.trans_err_mm},
                             {"rot_sigma_deg", s.rot_sigma_deg},
                             {"trans_sigma_mm", s.trans_sigma_mm}});
    return json{{"has_calib", r.has_calib},
                {"ref_rot_deg_init", r.ref_rot_deg_init},
                {"ref_trans_mm_init", r.ref_trans_mm_init},
                {"curve", std::move(curve)}};
}

/// Reads the online-calibration estimate + its covariance against a reference.
class S10CalibrationInspector {
public:
    using State = branes::sdk::msckf::State<double>;
    using SO3 = branes::math::lie::SO3<double>;
    using Vec3 = branes::math::lie::detail::Vec<double, 3>;

    /// Sample camera `cam`'s calibration estimate at time `t` against the reference
    /// extrinsic (R_ic, p_ic). `t` is recorded; the read is pure. If the state has
    /// no calibration block, returns a zero sample (the caller checks via `has_calib`
    /// on the record).
    [[nodiscard]] S10CalibSample
    sample(const State& s, const SO3& ref_R_ic, const Vec3& ref_p_ic, double t = 0.0, std::size_t cam = 0) const {
        S10CalibSample out;
        out.t = t;
        if (cam >= s.calib.size())
            return out;
        const auto& c = s.calib[cam];
        out.rot_err_deg = norm3((c.R_imu_cam.inverse() * ref_R_ic).log()) * kRad2Deg;
        out.trans_err_mm =
            norm3(Vec3{{c.p_imu_cam[0] - ref_p_ic[0], c.p_imu_cam[1] - ref_p_ic[1], c.p_imu_cam[2] - ref_p_ic[2]}}) *
            1000.0;
        const auto P = s.covariance();
        const std::size_t c0 = s.calib_offset(cam);  // [δθ_ic(3), δp_ic(3)]
        out.rot_sigma_deg = block_sigma(P, c0) * kRad2Deg;
        out.trans_sigma_mm = block_sigma(P, c0 + 3) * 1000.0;
        return out;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kRad2Deg = 180.0 / kPi;

    static double norm3(const Vec3& v) {
        return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
    [[nodiscard]] static double block_sigma(const branes::sdk::msckf::DynMat<double>& P, std::size_t o) {
        double tr = 0.0;
        for (std::size_t i = 0; i < 3 && o + i < P.rows; ++i)
            tr += P(o + i, o + i);
        return std::sqrt(tr < 0.0 ? 0.0 : tr);
    }
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_S10_CALIBRATION_INSPECT_HPP
