// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/invariants/core.hpp — the result type, the report, and the
// arithmetic-scaled tolerance policy shared by every VIO invariant check
// (issue #445, epic #444 §B).
//
// Every check returns an InvariantResult: the MEASURED value in the invariant's
// native units (rad, m², px, s, dimensionless …), the threshold it is held to,
// which side of the threshold passes, and the verdict. The check computes in the
// arithmetic type T under test; the result is reported in `double`, which holds
// every value of float, posit<32,2> and double exactly, so one report can collect
// stages that ran in different number systems.
//
// ── Tolerance policy ─────────────────────────────────────────────────────────
// A threshold that is an artifact of floating-point arithmetic is never a
// hardcoded `1e-9`. It is
//
//     threshold = safety · n · ε_T · scale
//
// where ε_T = std::numeric_limits<T>::epsilon() (the unit roundoff at 1, which
// Universal specializes for posits as well as the IEEE types), n is the size of
// the computation (the γ_n = n·u growth of the standard backward-error bounds),
// and `scale` is the norm of the inputs the residual is measured against (so
// the threshold carries native units). Where a residual is a backward error
// (‖KS − PHᵀ‖ / (‖K‖‖S‖ + ‖PHᵀ‖)) the conditioning of the inputs cancels out of
// the bound; where it is not (rank, definiteness margins) the scale is the
// matrix norm. The same check therefore means the same thing in float, posit16,
// posit<32,2> and double.
//
// Posit note: ε_T is the spacing at magnitude 1; a posit loses fraction bits as
// |x| moves away from 1 (tapered precision). The relative residuals here are
// formed from O(1)-normalized quantities, but a fixture whose entries sit at
// 1e±6 in posit16 will see a coarser effective ε than ε_T.
//
// Where a threshold is physical rather than arithmetic (sensor ranges, Δt bounds,
// the static ‖a‖ ≈ g window, pixel bounds), it is an explicit parameter of the
// check, never a default that pretends to be universal.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_INVARIANTS_CORE_HPP
#define BRANES_SDK_EVAL_INVARIANTS_CORE_HPP

#include <branes/math/arithmetic.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string_view>
#include <vector>

namespace branes::sdk::eval::inv {

/// The pipeline stage an invariant belongs to. Each enumerator names the
/// TRANSFORMATION the stage performs, not just its position, so a stage, its
/// bench and its invariants read in isolation from the rest of the pipeline.
/// The names match the stage tools (tools/src/s<N>_<transformation>.cpp) and
/// docs/arch/vio-pipeline-canonical.md; S6 is split into S6a–S6e (#444/#452).
/// `Inputs` and `EndToEnd` are not transformations: they bracket the pipeline.
enum class Stage {
    Inputs,                    ///< sensor streams and calibration as delivered
    S0_sensor_model,           ///< project / unproject, distort / undistort: the camera and IMU models
    S1_initialization,         ///< bootstrap attitude, gravity, biases, P0
    S2_propagation,            ///< IMU mean and covariance propagation, P <- Phi P Phi^T + Q_d
    S3_augmentation,           ///< clone the IMU pose into the window, P grows by 6
    S4_frontend,               ///< track generation (KLT), pixels -> tracks
    S5_triangulation,          ///< tracks + clone poses -> 3D feature
    S6a_jacobians,             ///< measurement Jacobians H_x, H_f and residual r
    S6b_nullspace_projection,  ///< left null-space projection that marginalizes the feature
    S6c_compression,           ///< QR compression of the stacked measurement
    S6d_gating,                ///< chi-square (Mahalanobis) gate
    S6e_ekf_update,            ///< gain, correction delta-x, Joseph covariance update
    S9_marginalization,        ///< drop clones: P -> principal submatrix
    S10_online_calibration,    ///< refine extrinsics / intrinsics / time offset
    EndToEnd,                  ///< whole-run statistics (NEES/NIS windows, ATE/RPE)
};

[[nodiscard]] constexpr std::string_view to_string(Stage s) noexcept {
    switch (s) {
    case Stage::Inputs:
        return "inputs";
    case Stage::S0_sensor_model:
        return "S0_sensor_model";
    case Stage::S1_initialization:
        return "S1_initialization";
    case Stage::S2_propagation:
        return "S2_propagation";
    case Stage::S3_augmentation:
        return "S3_augmentation";
    case Stage::S4_frontend:
        return "S4_frontend";
    case Stage::S5_triangulation:
        return "S5_triangulation";
    case Stage::S6a_jacobians:
        return "S6a_jacobians";
    case Stage::S6b_nullspace_projection:
        return "S6b_nullspace_projection";
    case Stage::S6c_compression:
        return "S6c_compression";
    case Stage::S6d_gating:
        return "S6d_gating";
    case Stage::S6e_ekf_update:
        return "S6e_ekf_update";
    case Stage::S9_marginalization:
        return "S9_marginalization";
    case Stage::S10_online_calibration:
        return "S10_online_calibration";
    case Stage::EndToEnd:
        return "end_to_end";
    }
    return "?";
}

/// Which side of the threshold passes.
enum class Bound {
    Upper,   ///< pass iff value ≤ threshold (a residual)
    Lower,   ///< pass iff value ≥ threshold (a margin, e.g. λ_min of an SPD matrix)
    Band,    ///< pass iff threshold ≤ value ≤ threshold_hi (e.g. NIS/dof)
    Report,  ///< reported, never fails (e.g. a condition number to watch)
};

/// One measured invariant. `name` and `unit` are string literals owned by the
/// check, so a result is cheap to copy into a report.
struct InvariantResult {
    std::string_view name;  ///< stable id, e.g. "covariance.symmetric"
    std::string_view unit;  ///< native unit of `value` and the thresholds
    Stage stage = Stage::EndToEnd;
    Bound bound = Bound::Upper;
    double value = 0.0;         ///< the measured residual / statistic
    double threshold = 0.0;     ///< the bound (lower edge for Band)
    double threshold_hi = 0.0;  ///< upper edge, Band only
    bool pass = false;

    /// value ÷ threshold for an Upper bound — how close to failing (≤ 1 passes).
    /// Lets a summary rank the tightest invariants across stages and units.
    [[nodiscard]] double margin() const noexcept {
        if (bound != Bound::Upper || !(threshold > 0.0))
            return pass ? 0.0 : std::numeric_limits<double>::infinity();
        return value / threshold;
    }
};

namespace detail {

inline constexpr double kInf = std::numeric_limits<double>::infinity();

[[nodiscard]] inline InvariantResult make(std::string_view name,
                                          std::string_view unit,
                                          Stage stage,
                                          Bound bound,
                                          double value,
                                          double threshold,
                                          double threshold_hi = 0.0) {
    InvariantResult r;
    r.name = name;
    r.unit = unit;
    r.stage = stage;
    r.bound = bound;
    r.value = value;
    r.threshold = threshold;
    r.threshold_hi = threshold_hi;
    // NaN fails every bound: a non-finite residual is never a pass.
    switch (bound) {
    case Bound::Upper:
        r.pass = value <= threshold;
        break;
    case Bound::Lower:
        r.pass = value >= threshold;
        break;
    case Bound::Band:
        r.pass = value >= threshold && value <= threshold_hi;
        break;
    case Bound::Report:
        r.pass = true;
        break;
    }
    return r;
}

template <math::Scalar T>
[[nodiscard]] double to_double(const T& x) {
    return static_cast<double>(x);
}

}  // namespace detail

/// ε_T: the unit roundoff of the arithmetic type at magnitude 1.
template <math::Scalar T>
[[nodiscard]] T epsilon() {
    return std::numeric_limits<T>::epsilon();
}

/// Default safety factor on the γ_n·ε_T bound. Generous enough that a correct
/// computation never trips it in any validated type, tight enough that a
/// structural violation (a dropped term, a wrong sign, an unsymmetrized update)
/// is orders of magnitude above it in double and posit<32,2>, and clearly above it
/// in float.
inline constexpr double kDefaultSafety = 16.0;

/// The arithmetic tolerance `safety · n · ε_T · scale`, in double for reporting.
template <math::Scalar T>
[[nodiscard]] double arithmetic_tolerance(std::size_t n, double scale = 1.0, double safety = kDefaultSafety) {
    const double nn = n == 0 ? 1.0 : static_cast<double>(n);
    return safety * nn * detail::to_double(epsilon<T>()) * scale;
}

/// Ordered collection of results for one bench run, fixture, or stage boundary.
class InvariantReport {
public:
    void add(const InvariantResult& r) {
        results_.push_back(r);
    }
    void add(const std::vector<InvariantResult>& rs) {
        results_.insert(results_.end(), rs.begin(), rs.end());
    }

    [[nodiscard]] const std::vector<InvariantResult>& results() const noexcept {
        return results_;
    }
    [[nodiscard]] bool all_pass() const noexcept {
        for (const auto& r : results_)
            if (!r.pass)
                return false;
        return true;
    }
    [[nodiscard]] std::size_t failures() const noexcept {
        std::size_t n = 0;
        for (const auto& r : results_)
            n += r.pass ? 0 : 1;
        return n;
    }
    /// The first violated invariant in insertion order — the live-assertion mode
    /// (#447) adds results in pipeline order, so this is the first faulty stage.
    [[nodiscard]] std::optional<InvariantResult> first_failure() const {
        for (const auto& r : results_)
            if (!r.pass)
                return r;
        return std::nullopt;
    }
    /// The Upper-bound result closest to (or furthest past) its threshold.
    [[nodiscard]] std::optional<InvariantResult> tightest() const {
        std::optional<InvariantResult> best;
        for (const auto& r : results_)
            if (r.bound == Bound::Upper && (!best || r.margin() > best->margin()))
                best = r;
        return best;
    }
    /// Results of one stage, for a per-stage summary of a run.
    [[nodiscard]] std::vector<InvariantResult> for_stage(Stage s) const {
        std::vector<InvariantResult> out;
        for (const auto& r : results_)
            if (r.stage == s)
                out.push_back(r);
        return out;
    }

private:
    std::vector<InvariantResult> results_;
};

}  // namespace branes::sdk::eval::inv

#endif  // BRANES_SDK_EVAL_INVARIANTS_CORE_HPP
