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

/// The pipeline stage an invariant belongs to (docs/arch/vio-pipeline-canonical.md,
/// with S6 split into S6a–S6e as in #444/#452).
enum class Stage {
    Inputs,
    S0,
    S1,
    S2,
    S3,
    S4,
    S5,
    S6a,
    S6b,
    S6c,
    S6d,
    S6e,
    S9,
    S10,
    EndToEnd,
};

[[nodiscard]] constexpr std::string_view to_string(Stage s) noexcept {
    switch (s) {
    case Stage::Inputs:
        return "inputs";
    case Stage::S0:
        return "S0";
    case Stage::S1:
        return "S1";
    case Stage::S2:
        return "S2";
    case Stage::S3:
        return "S3";
    case Stage::S4:
        return "S4";
    case Stage::S5:
        return "S5";
    case Stage::S6a:
        return "S6a";
    case Stage::S6b:
        return "S6b";
    case Stage::S6c:
        return "S6c";
    case Stage::S6d:
        return "S6d";
    case Stage::S6e:
        return "S6e";
    case Stage::S9:
        return "S9";
    case Stage::S10:
        return "S10";
    case Stage::EndToEnd:
        return "end-to-end";
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
