// SPDX-License-Identifier: MIT
//
// branes/tools/bench/bench.hpp — the stage test-bench runner and CLI (issue
// #453, epic #444 §A). A stage bench runs ONE pipeline transformation
// (msckf/stages/) on fixtures, in several arithmetic types and implementation
// variants, and reports the stage's invariants (#445) in native units. It is the
// focused debug / test / characterization / research environment for that
// stage: one small target, a seconds-long edit-compile-run loop.
//
// ── Writing a bench ─────────────────────────────────────────────────────────
// A bench is a struct `B` describing one stage (see
// tools/include/branes/tools/bench/s9_marginalization_bench.hpp):
//
//   static constexpr std::string_view kStage;          "S9_marginalization"
//   static constexpr inv::Stage kInvStage;             the #445 stage tag
//   static std::vector<Variant> variants();            "shipped" first
//   template <class T> struct Input; struct Output;    the stage's I/O in T
//   template <class T> static Input<T>  decode_input(const json&);
//   template <class T> static Output<T> decode_output(const json&);
//   template <class T> static json      encode_output(const Output<T>&);
//   template <class T> static Output<T> run(const Input<T>&, std::string_view variant);
//   template <class T> static std::vector<double> flatten(const Output<T>&);
//   template <class T> static std::vector<inv::InvariantResult>
//                          invariants(const Input<T>&, const Output<T>&, const Fixture&);
//   static std::vector<NamedFixture> builtin_fixtures();
//
// and optionally a characterization sweep:
//
//   static Sweep default_sweep();
//   static std::vector<std::string> sweep_columns();
//   template <class T> static std::vector<double> sweep_point(const Point&);
//
// The runner adds, per fixture and type: the replay check (a captured fixture
// must reproduce its recorded output bit-for-bit in the arithmetic it was
// captured in), the known-answer check (within the type's arithmetic
// tolerance), the deviation from the `double` run, and the run time.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_BENCH_HPP
#define BRANES_TOOLS_BENCH_BENCH_HPP

#include <branes/sdk/eval/invariants.hpp>
#include <branes/tools/bench/fixture.hpp>
#include <branes/tools/bench/sweep.hpp>
#include <branes/tools/bench/types.hpp>
#include <branes/tools/bench/variant.hpp>
#include <branes/tools/stage_probe.hpp>
#include <branes/tools/vio_stage_contracts.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace branes::tools::bench {

namespace inv = branes::sdk::eval::inv;

/// A fixture with a display name (its file stem, or a built-in label).
struct NamedFixture {
    std::string name;
    Fixture fixture;
};

/// One stage run in one arithmetic type.
struct TypeRun {
    std::string type;
    inv::InvariantReport report;
    double max_diff_vs_double = 0.0;  ///< max |output − double output| over the flattened output
    double run_us = 0.0;              ///< best-of-N wall time of the stage call, microseconds
};

/// One fixture × variant, across the selected types.
struct FixtureRun {
    std::string fixture;
    FixtureKind kind = FixtureKind::KnownAnswer;
    std::string variant;
    std::vector<TypeRun> types;
    [[nodiscard]] bool pass() const {
        for (const auto& t : types)
            if (!t.report.all_pass())
                return false;
        return true;
    }
};

/// Tolerance for comparing a stage output in T against a reference held in
/// `double` (a known answer, a ground truth): the arithmetic tolerance of T,
/// but never tighter than double's, since the reference itself carries double
/// rounding. Matters for types wider than double (long double).
template <class T>
[[nodiscard]] double tolerance_vs_double(std::size_t n, double scale = 1.0, double safety = inv::kDefaultSafety) {
    return std::max(inv::arithmetic_tolerance<T>(n, scale, safety),
                    inv::arithmetic_tolerance<double>(n, scale, safety));
}

namespace detail {

/// Bitwise equality of two doubles (NaN == NaN when the bits agree; −0 ≠ +0).
[[nodiscard]] inline bool same_bits(double a, double b) {
    std::uint64_t x = 0, y = 0;
    std::memcpy(&x, &a, sizeof x);
    std::memcpy(&y, &b, sizeof y);
    return x == y;
}

template <class F>
[[nodiscard]] double best_time_us(F&& f, int reps) {
    double best = 1e300;
    for (int r = 0; r < reps; ++r) {
        const auto t0 = std::chrono::steady_clock::now();
        f();
        const auto t1 = std::chrono::steady_clock::now();
        best = std::min(best, std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    return best;
}

}  // namespace detail

/// Whether a fixture's expected output binds `variant`: every variant when the
/// fixture names none, else each variant in its comma-separated list.
[[nodiscard]] inline bool binds_variant(std::string_view list, std::string_view variant) {
    if (list.empty())
        return true;
    while (!list.empty()) {
        const auto comma = list.find(',');
        if (list.substr(0, comma) == variant)
            return true;
        if (comma == std::string_view::npos)
            break;
        list.remove_prefix(comma + 1);
    }
    return false;
}

/// Run bench `B` on one fixture in arithmetic type T, with `reference` the
/// flattened `double` output to compare against (empty: no comparison).
template <class B, class T>
[[nodiscard]] TypeRun
run_type(const Fixture& f, std::string_view variant, const std::vector<double>& reference, int timing_reps = 5) {
    TypeRun tr;
    tr.type = std::string(type_name<T>());
    const auto input = B::template decode_input<T>(f.input);
    auto output = B::template run<T>(input, variant);
    tr.run_us = detail::best_time_us([&] { output = B::template run<T>(input, variant); }, timing_reps);
    tr.report.add(B::template invariants<T>(input, output, f));

    const std::vector<double> flat = B::template flatten<T>(output);
    if (reference.size() == flat.size())
        for (std::size_t i = 0; i < flat.size(); ++i)
            tr.max_diff_vs_double = std::max(tr.max_diff_vs_double, std::abs(flat[i] - reference[i]));

    // The expected output binds only the variant it was produced by (or every
    // variant, when the fixture leaves `variant` empty).
    if (!f.expected.is_null() && binds_variant(f.variant, variant)) {
        const std::vector<double> want = B::template flatten<T>(B::template decode_output<T>(f.expected));
        if (f.kind == FixtureKind::Captured && f.arithmetic == tr.type &&
            (f.platform.empty() || f.platform == current_platform())) {
            // A captured fixture must replay to exactly the output it recorded —
            // on the platform it was recorded on.
            std::size_t mismatched = want.size() == flat.size() ? 0 : std::max(want.size(), flat.size());
            if (want.size() == flat.size())
                for (std::size_t i = 0; i < flat.size(); ++i)
                    mismatched += detail::same_bits(flat[i], want[i]) ? 0 : 1;
            tr.report.add(inv::check_scalar(static_cast<double>(mismatched),
                                            0.0,
                                            inv::Bound::Upper,
                                            B::kInvStage,
                                            "replay.bit_identical",
                                            "count"));
        } else if (f.kind == FixtureKind::Captured && f.arithmetic == tr.type) {
            // Recorded on another platform: its math library rounds sin, cos,
            // exp … differently, so bit-identity is not the contract there. The
            // invariants still run; the replay difference is reported.
            double worst = 0.0;
            std::size_t broken = want.size() == flat.size() ? 0 : std::max(want.size(), flat.size());
            if (want.size() == flat.size())
                for (std::size_t i = 0; i < flat.size(); ++i) {
                    const double d = std::abs(flat[i] - want[i]);
                    if (std::isfinite(d))
                        worst = std::max(worst, d);
                    else
                        broken += std::isfinite(flat[i]) == std::isfinite(want[i]) ? 0 : 1;
                }
            // A shape change, or a value finite on one platform and not the
            // other, is a failure anywhere.
            tr.report.add(inv::check_scalar(static_cast<double>(broken),
                                            0.0,
                                            inv::Bound::Upper,
                                            B::kInvStage,
                                            "replay.same_shape_and_finiteness",
                                            "count"));
            tr.report.add(inv::check_scalar(
                worst, 0.0, inv::Bound::Report, B::kInvStage, "replay.cross_platform_max_diff", "output units"));
        } else if (f.kind == FixtureKind::KnownAnswer) {
            double worst = want.size() == flat.size() ? 0.0 : inv::detail::kInf, scale = 1.0;
            if (want.size() == flat.size())
                for (std::size_t i = 0; i < flat.size(); ++i) {
                    // A non-finite value never matches a known answer (std::max would drop a NaN).
                    if (!std::isfinite(flat[i]) || !std::isfinite(want[i])) {
                        worst = inv::detail::kInf;
                        break;
                    }
                    worst = std::max(worst, std::abs(flat[i] - want[i]));
                    scale = std::max(scale, std::abs(want[i]));
                }
            tr.report.add(inv::check_scalar(worst,
                                            tolerance_vs_double<T>(flat.size(), scale),
                                            inv::Bound::Upper,
                                            B::kInvStage,
                                            "known_answer.residual",
                                            "output units"));
        }
    }
    return tr;
}

/// Safety factor for a check on data that entered T through a double-held
/// fixture (a rotation, a state): scaled so the threshold is never tighter than
/// double's, for types wider than double. Equal to `safety` otherwise.
template <class T>
[[nodiscard]] double safety_vs_double(double safety = inv::kDefaultSafety) {
    const double et = static_cast<double>(std::numeric_limits<T>::epsilon());
    return safety * std::max(1.0, std::numeric_limits<double>::epsilon() / et);
}

/// The relative precision of T at `magnitude`. An IEEE type's relative spacing
/// is ε_T for every normal number. A tapered type (posit) is most precise near 1
/// and loses fraction bits as |x| moves away: its relative gap to the next value
/// above |magnitude| — never less than ε_T.
template <class T>
[[nodiscard]] double eps_at(double magnitude) {
    const double eps = static_cast<double>(std::numeric_limits<T>::epsilon());
    if constexpr (std::numeric_limits<T>::is_iec559) {
        return eps;
    } else {
        const double m = std::abs(magnitude);
        if (!(m > 0.0) || !std::isfinite(m))
            return eps;
        const T a(m);
        T b = a;
        ++b;  // the next representable value
        const double da = static_cast<double>(a), db = static_cast<double>(b);
        return da > 0.0 && std::isfinite(db) ? std::max(eps, (db - da) / da) : eps;
    }
}

/// `safety_vs_double` scaled to T's precision at the data's `magnitude`: a check
/// on quantities of that size holds T to the precision it has there. Identical
/// to `safety_vs_double` for IEEE types.
template <class T>
[[nodiscard]] double safety_at(double magnitude, double safety = inv::kDefaultSafety) {
    const double eps = static_cast<double>(std::numeric_limits<T>::epsilon());
    return safety_vs_double<T>(safety) * (eps_at<T>(magnitude) / eps);
}

/// A lower-triangular factor L with L Lᵀ = P for a symmetric positive
/// SEMI-definite P (Cholesky that turns roundoff-level pivots into zero columns
/// instead of failing). Clone augmentation makes P exactly singular, so square-
/// root variants need this to start from a real filter state. Returns false only
/// if a pivot is clearly negative (P is not PSD in T).
template <class T>
[[nodiscard]] bool psd_factor(const sdk::msckf::DynMat<T>& P, sdk::msckf::DynMat<T>& L) {
    using std::abs;
    using std::sqrt;
    const std::size_t n = P.rows;
    L = sdk::msckf::DynMat<T>(n, n);
    T scale{0};
    for (const T& x : P.d)
        scale = std::max(scale, abs(x));
    const T tol = T(inv::kDefaultSafety * static_cast<double>(n)) * std::numeric_limits<T>::epsilon() * scale;
    for (std::size_t j = 0; j < n; ++j) {
        T d = P(j, j);
        for (std::size_t k = 0; k < j; ++k)
            d -= L(j, k) * L(j, k);
        if (d < -tol)
            return false;
        if (!(d > tol))
            continue;  // zero pivot: this direction carries no variance
        const T ljj = sqrt(d);
        L(j, j) = ljj;
        for (std::size_t i = j + 1; i < n; ++i) {
            T s = P(i, j);
            for (std::size_t k = 0; k < j; ++k)
                s -= L(i, k) * L(j, k);
            L(i, j) = s / ljj;
        }
    }
    return true;
}

/// The arithmetic types a bench's stage can run in: `B::SupportedTypes` if
/// the bench declares it (a stage that is not generic in T, e.g. the KLT front
/// end), else every type the framework knows.
template <class B>
[[nodiscard]] constexpr auto bench_types() {
    if constexpr (requires { typename B::SupportedTypes; })
        return typename B::SupportedTypes{};
    else
        return AllTypes{};
}

/// Timing repetitions per run: `B::kTimingReps` if the bench sets it (stages
/// that are slow in software arithmetic), else 5.
template <class B>
[[nodiscard]] constexpr int timing_reps() {
    if constexpr (requires { B::kTimingReps; })
        return B::kTimingReps;
    else
        return 5;
}

/// The types a variant runs in when no `--types` is given: the default set,
/// minus any the bench leaves out for that variant through
/// `B::runs_by_default(variant, type)`. For a variant too costly in emulated
/// arithmetic (a Universal posit runs ~10⁴× slower than native floating point),
/// such as a RANSAC front end. Naming the type in `--types` still runs it.
template <class B>
[[nodiscard]] std::vector<std::string> default_types_for(std::string_view variant) {
    auto names = type_names(DefaultTypes{});
    if constexpr (requires { B::runs_by_default(variant, std::string_view{}); })
        std::erase_if(names, [&](const std::string& t) { return !B::runs_by_default(variant, t); });
    return names;
}

/// Run bench `B` on one fixture for every type in `Types` whose name is in
/// `selected` (empty: all), always computing the `double` run as the reference.
template <class B, class... Ts>
[[nodiscard]] FixtureRun run_fixture(const NamedFixture& nf,
                                     std::string_view variant,
                                     TypeList<Ts...> types,
                                     const std::vector<std::string>& selected = {}) {
    FixtureRun fr;
    fr.fixture = nf.name;
    fr.kind = nf.fixture.kind;
    fr.variant = std::string(variant);
    const auto ref_input = B::template decode_input<double>(nf.fixture.input);
    const std::vector<double> reference = B::template flatten<double>(B::template run<double>(ref_input, variant));
    for_each_type(types, [&]<class T>() {
        const std::string name(type_name<T>());
        if (!selected.empty() && std::find(selected.begin(), selected.end(), name) == selected.end())
            return;
        fr.types.push_back(run_type<B, T>(nf.fixture, variant, reference, timing_reps<B>()));
    });
    return fr;
}

// ── Reporting ───────────────────────────────────────────────────────────────

[[nodiscard]] inline std::string fmt(double v) {
    std::ostringstream s;
    s.precision(4);
    s << v;
    return s.str();
}

inline void print_run(const FixtureRun& fr) {
    for (const auto& t : fr.types) {
        std::cout << "\n  " << fr.fixture << " [" << to_string(fr.kind) << "]  variant=" << fr.variant
                  << "  type=" << t.type << '\n';
        rule('-');
        std::cout << "  " << std::left << std::setw(40) << "invariant" << std::setw(13) << "value" << std::setw(13)
                  << "threshold" << std::setw(16) << "unit" << "verdict\n";
        rule('-');
        for (const auto& r : t.report.results())
            std::cout << "  " << std::left << std::setw(40) << r.name << std::setw(13) << fmt(r.value) << std::setw(13)
                      << (r.bound == inv::Bound::Report ? std::string("-") : fmt(r.threshold)) << std::setw(16)
                      << r.unit << (r.pass ? "PASS" : "FAIL") << '\n';
        rule('-');
        std::cout << "  max |output - double output| = " << fmt(t.max_diff_vs_double)
                  << "    stage run time (best of reps) = " << fmt(t.run_us) << " us\n";
    }
}

/// Long-form CSV of every invariant result: one row per fixture × variant ×
/// type × invariant.
inline void write_report_csv(const std::vector<FixtureRun>& runs, const std::filesystem::path& path) {
    if (path.has_parent_path())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    out << "fixture,kind,variant,type,invariant,unit,value,threshold,threshold_hi,pass,run_us,max_diff_vs_double\n";
    out.precision(17);
    for (const auto& fr : runs)
        for (const auto& t : fr.types)
            for (const auto& r : t.report.results())
                out << csv_field(fr.fixture) << ',' << to_string(fr.kind) << ',' << csv_field(fr.variant) << ','
                    << t.type << ',' << csv_field(r.name) << ',' << csv_field(r.unit) << ',' << r.value << ','
                    << r.threshold << ',' << r.threshold_hi << ',' << (r.pass ? 1 : 0) << ',' << t.run_us << ','
                    << t.max_diff_vs_double << '\n';
}

// ── Precision characterization (#450) ───────────────────────────────────────
//
// `--precision` runs the stage once per arithmetic type on each fixture and on
// the bench's conditioning ladder, and measures how much of each type's
// precision survives the stage. Per output group, the normwise relative error
// against the `double` run,
//
//     rel_err = ‖y_T − y_double‖∞ / ‖y_double‖∞,
//
// is reported in units of the type's unit roundoff (the error amplification,
// an empirical condition number of the stage at that input) and as bits lost,
// log₂ of the amplification. A backward-stable stage on a well-conditioned
// input loses a few bits; amplification that tracks the input's conditioning
// says the stage's sensitivity is the problem's, and amplification well above
// it says the algorithm is. The invariants run in every type, so a type that
// breaks the stage's contract is reported too.
//
// That error has two sources: the input rounded into T (which any algorithm
// pays, times the problem's condition number) and the stage's own arithmetic.
// `stage_err` separates the second: the T run against a double run on the
// input as T holds it (decoded in T, re-encoded, decoded in double).
//
// The reference is the `double` run, so a type wider than double measures the
// double run's own error: its rows use ε_double, and their amplification is
// double's. A posit's precision tapers away from 1; the rows use its ε at 1.
//
// Optional bench hooks:
//
//   template <class T> static std::vector<OutputGroup> precision_groups(const Output<T>&);
//       the output split into groups of like quantities ("P", "mean", "dx"),
//       so a group's error is not drowned by a larger one; default: one group,
//       the flattened output
//   static std::vector<std::string> conditioning_columns();
//   static std::vector<double> conditioning(const Input<double>&);
//       the conditioning of an input (κ(P), parallax, depth …)
//   static std::vector<NamedFixture> precision_fixtures();
//       inputs along the stage's conditioning axis (the ladder)

/// A named slice of a stage output, compared across types as one vector.
/// `floor` is the magnitude below which the group's values are noise for the
/// stage (half a pixel for a residual, 1 for an NIS): the error is relative to
/// max(‖y_double‖∞, floor), so a quantity that is ~0 in double — a residual at
/// the true poses — is not reported as having lost every bit.
struct OutputGroup {
    std::string name;
    std::vector<double> values;
    double floor = 0.0;
};

/// The normalized-image-coordinate floor of a residual: half a pixel at the
/// EuRoC focal length.
inline constexpr double kResidualFloor = 0.5 / 458.654;

/// One fixture, in one arithmetic type, against the `double` run.
struct PrecisionRow {
    std::string fixture;
    FixtureKind kind = FixtureKind::KnownAnswer;
    std::string variant;
    std::string type;
    std::vector<double> conditioning;
    double eps = 0.0;                                     ///< the unit the error is counted in
    std::vector<std::pair<std::string, double>> rel_err;  ///< per output group, vs double on the input
    std::vector<double> abs_err;                          ///< per output group, ‖y_T − y_double‖∞
    std::vector<double> stage_err;                        ///< per output group, vs double on the input rounded to T
    std::size_t invariants = 0, failed = 0;
    double worst_margin = 0.0;  ///< the tightest Upper-bound invariant, value ÷ threshold
    std::string first_failed;   ///< the first invariant that failed, if any
    std::string status = "ok";  ///< "ok", or why the run produced no output

    [[nodiscard]] static double amplification(double rel, double eps) {
        return eps > 0.0 ? rel / eps : rel;
    }
    /// log₂ of the amplification, capped at the type's precision −log₂ ε: an
    /// error as large as the value itself leaves no correct bit, however large.
    [[nodiscard]] static double bits_lost(double rel, double eps) {
        const double cap = eps > 0.0 ? -std::log2(eps) : 0.0;
        const double a = amplification(rel, eps);
        if (std::isnan(a))
            return cap;
        return std::min(cap, std::log2(std::max(a, 1.0)));
    }
};

/// ‖y − ref‖∞, the normwise difference. A value non-finite in y but not in
/// ref, or a shape change, is ∞; one non-finite in both is not a difference.
[[nodiscard]] inline double normwise_abs_err(const std::vector<double>& y, const std::vector<double>& ref) {
    if (y.size() != ref.size())
        return inv::detail::kInf;
    double num = 0.0;
    for (std::size_t i = 0; i < y.size(); ++i) {
        const bool fy = std::isfinite(y[i]), fr = std::isfinite(ref[i]);
        if (!fy && !fr)
            continue;
        if (fy != fr)
            return inv::detail::kInf;
        num = std::max(num, std::abs(y[i] - ref[i]));
    }
    return num;
}

/// The normwise relative difference ‖y − ref‖∞ / max(‖ref‖∞, floor) (absolute
/// when both are zero), with `normwise_abs_err`'s non-finite rules.
[[nodiscard]] inline double
normwise_rel_err(const std::vector<double>& y, const std::vector<double>& ref, double floor = 0.0) {
    if (y.size() != ref.size())
        return inv::detail::kInf;
    double num = 0.0, den = floor;
    for (std::size_t i = 0; i < y.size(); ++i) {
        const bool fy = std::isfinite(y[i]), fr = std::isfinite(ref[i]);
        if (!fy && !fr)
            continue;
        if (fy != fr)
            return inv::detail::kInf;
        num = std::max(num, std::abs(y[i] - ref[i]));
        den = std::max(den, std::abs(ref[i]));
    }
    return den > 0.0 ? num / den : num;
}

/// The groups of a filter state: every attitude (quaternions: nav, calibration,
/// clones), every position (m), the velocity and biases, and the covariance.
/// Timestamps are double in every type by design and are left out.
template <class T>
[[nodiscard]] std::vector<OutputGroup> state_groups(const sdk::msckf::State<T>& s) {
    OutputGroup rot{"rot", {}}, pos{"pos", {}}, vb{"vel_bias", {}}, P{"P", {}};
    auto put = [](OutputGroup& g, const auto& x, std::size_t n) {
        for (std::size_t i = 0; i < n; ++i)
            g.values.push_back(static_cast<double>(x[i]));
    };
    put(rot, s.R.quaternion(), 4);
    put(pos, s.p, 3);
    put(vb, s.v, 3);
    put(vb, s.bg, 3);
    put(vb, s.ba, 3);
    for (const auto& c : s.calib) {
        put(rot, c.R_imu_cam.quaternion(), 4);
        put(pos, c.p_imu_cam, 3);
    }
    for (const auto& c : s.clones) {
        put(rot, c.R.quaternion(), 4);
        put(pos, c.p, 3);
    }
    for (const T& x : s.cov.P.d)
        P.values.push_back(static_cast<double>(x));
    return {rot, pos, vb, P};
}

/// log₁₀ of the 2-norm condition number of a symmetric matrix, λ_max / λ_min,
/// with λ_min floored at ε_double·λ_max: 16 means singular in double (P after
/// clone augmentation is exactly singular). NaN for a non-finite or zero matrix.
[[nodiscard]] inline double log10_cond_sym(const sdk::msckf::DynMat<double>& a) {
    if (a.rows == 0)
        return std::numeric_limits<double>::quiet_NaN();
    for (const double x : a.d)
        if (!std::isfinite(x))
            return std::numeric_limits<double>::quiet_NaN();
    const auto ev = inv::la::symmetric_eigenvalues(a);
    double lo = inv::detail::kInf, hi = 0.0;
    for (const double e : ev) {
        lo = std::min(lo, std::abs(e));
        hi = std::max(hi, std::abs(e));
    }
    if (!(hi > 0.0))
        return std::numeric_limits<double>::quiet_NaN();
    return std::log10(hi / std::max(lo, std::numeric_limits<double>::epsilon() * hi));
}

/// log₁₀ of λ_max / λ_min over the nonzero spectrum of a symmetric PSD matrix:
/// eigenvalues at or below n·ε_double·λ_max are its null space and do not
/// count. For a Gram matrix HᵀH this is κ(H)² over H's rank, so a Jacobian with
/// zero columns (states a measurement does not touch) is not reported singular.
/// NaN for a non-finite or zero matrix.
[[nodiscard]] inline double log10_cond_range(const sdk::msckf::DynMat<double>& a) {
    if (a.rows == 0)
        return std::numeric_limits<double>::quiet_NaN();
    for (const double x : a.d)
        if (!std::isfinite(x))
            return std::numeric_limits<double>::quiet_NaN();
    const auto ev = inv::la::symmetric_eigenvalues(a);
    double hi = 0.0;
    for (const double e : ev)
        hi = std::max(hi, std::abs(e));
    if (!(hi > 0.0))
        return std::numeric_limits<double>::quiet_NaN();
    const double tol = static_cast<double>(a.rows) * std::numeric_limits<double>::epsilon() * hi;
    double lo = hi;
    for (const double e : ev)
        if (std::abs(e) > tol)
            lo = std::min(lo, std::abs(e));
    return std::log10(hi / lo);
}

/// A dense n × n SPD matrix of condition number exactly 10^log10_cond (up to
/// rounding): (1 − r)·G/λ_max(G) + r·I with r = 10^−log10_cond and G = B·Bᵀ for
/// a fixed, rank-deficient B, so λ_max = 1 and λ_min = r. log10_cond = 0 is the
/// identity. The conditioning ladders build P with it.
[[nodiscard]] inline sdk::msckf::DynMat<double> spd_of_condition(std::size_t n, double log10_cond) {
    const std::size_t k = n / 2 + 1;  // rank k < n: G has a null space for every n ≥ 3
    sdk::msckf::DynMat<double> G(n, n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j <= i; ++j) {
            double s = 0.0;
            for (std::size_t c = 0; c < k; ++c)
                s += std::sin(0.7 * double(i + 1) + 1.3 * double(c + 1)) *
                     std::sin(0.7 * double(j + 1) + 1.3 * double(c + 1));
            G(i, j) = G(j, i) = s;
        }
    double hi = 0.0;
    for (const double e : inv::la::symmetric_eigenvalues(G))
        hi = std::max(hi, std::abs(e));
    const double r = std::pow(10.0, -log10_cond);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            G(i, j) = (1.0 - r) * G(i, j) / hi + (i == j ? r : 0.0);
    return G;
}

/// A point on a bench's conditioning ladder: an input with no expected output
/// (the precision mode compares types against double, not against an answer).
[[nodiscard]] inline NamedFixture
ladder_fixture(std::string_view stage, std::string name, json input, std::string description) {
    Fixture f;
    f.stage = std::string(stage);
    f.kind = FixtureKind::KnownAnswer;
    f.source = "precision ladder (#450)";
    f.description = std::move(description);
    f.input = std::move(input);
    return {std::move(name), std::move(f)};
}

template <class B, class T>
[[nodiscard]] std::vector<OutputGroup> precision_groups_of(const typename B::template Output<T>& out) {
    if constexpr (requires { B::template precision_groups<T>(out); })
        return B::template precision_groups<T>(out);
    else
        return {{"output", B::template flatten<T>(out)}};
}

template <class B>
[[nodiscard]] std::vector<std::string> conditioning_columns_of() {
    if constexpr (requires { B::conditioning_columns(); })
        return B::conditioning_columns();
    else
        return {};
}

/// Run bench `B` on `nf` in every type of `selected`, against the `double` run.
template <class B, class... Ts>
[[nodiscard]] std::vector<PrecisionRow> precision_rows(const NamedFixture& nf,
                                                       std::string_view variant,
                                                       TypeList<Ts...> types,
                                                       const std::vector<std::string>& selected) {
    const auto ref_in = B::template decode_input<double>(nf.fixture.input);
    std::vector<double> cond;
    if constexpr (requires { B::conditioning(ref_in); })
        cond = B::conditioning(ref_in);
    const auto ref = precision_groups_of<B, double>(B::template run<double>(ref_in, variant));

    std::vector<PrecisionRow> rows;
    for_each_type(types, [&]<class T>() {
        const std::string name(type_name<T>());
        if (std::find(selected.begin(), selected.end(), name) == selected.end())
            return;
        PrecisionRow row;
        row.fixture = nf.name;
        row.kind = nf.fixture.kind;
        row.variant = std::string(variant);
        row.type = name;
        row.conditioning = cond;
        row.eps =
            std::max(static_cast<double>(std::numeric_limits<T>::epsilon()), std::numeric_limits<double>::epsilon());
        if constexpr (std::is_same_v<T, double>)
            row.eps = std::numeric_limits<double>::epsilon();
        try {
            const auto in = B::template decode_input<T>(nf.fixture.input);
            const auto out = B::template run<T>(in, variant);
            const auto groups = precision_groups_of<B, T>(out);
            const auto ref_rounded = [&] {
                const DecodeFromRounded scope(row.eps);
                return precision_groups_of<B, double>(
                    B::template run<double>(B::template decode_input<double>(B::encode_input(in)), variant));
            }();
            for (std::size_t g = 0; g < ref.size(); ++g) {
                const bool have = g < groups.size() && g < ref_rounded.size();
                const double floor = ref[g].floor;
                row.rel_err.emplace_back(
                    ref[g].name, have ? normwise_rel_err(groups[g].values, ref[g].values, floor) : inv::detail::kInf);
                row.abs_err.push_back(have ? normwise_abs_err(groups[g].values, ref[g].values) : inv::detail::kInf);
                row.stage_err.push_back(have ? normwise_rel_err(groups[g].values, ref_rounded[g].values, floor)
                                             : inv::detail::kInf);
            }
            for (const auto& r : B::template invariants<T>(in, out, nf.fixture)) {
                ++row.invariants;
                if (!r.pass && row.failed++ == 0)
                    row.first_failed = std::string(r.name);
                if (r.bound == inv::Bound::Upper)
                    row.worst_margin = std::max(row.worst_margin, r.margin());
            }
        } catch (const std::exception& e) {
            // A type that cannot run the stage (a posit NaR, an exhausted
            // iteration) is a result, not a bench failure.
            row.status = std::string("threw: ") + e.what();
            row.rel_err.clear();
            row.abs_err.clear();
            row.stage_err.clear();
            for (const auto& g : ref) {
                row.rel_err.emplace_back(g.name, inv::detail::kInf);
                row.abs_err.push_back(inv::detail::kInf);
                row.stage_err.push_back(inv::detail::kInf);
            }
        }
        rows.push_back(std::move(row));
    });
    return rows;
}

/// The types `--precision` runs when no `--types` is given: the issue's set
/// (#450), minus any the bench leaves out for the variant (`runs_by_default`).
template <class B>
[[nodiscard]] std::vector<std::string> precision_types_for(std::string_view variant) {
    std::vector<std::string> names{"double", "float", "posit32", "posit16", "long_double"};
    if constexpr (requires { B::runs_by_default(variant, std::string_view{}); })
        std::erase_if(names, [&](const std::string& t) { return !B::runs_by_default(variant, t); });
    return names;
}

inline void print_precision(const std::vector<PrecisionRow>& rows, const std::vector<std::string>& cond_cols) {
    std::cout << "\n  precision vs double, per output group:  <group> <rel err> <bits lost>/<stage bits lost>\n"
              << "    rel err    ‖y_T − y_double‖∞ / max(‖y_double‖∞, the group's noise floor)\n"
              << "    bits lost  log₂(rel err / ε_T), at most T's precision — for long_double, of the double run's\n"
              << "               own error (ε_double)\n"
              << "    stage      the same, against double on the input as T holds it: the stage's own arithmetic\n";
    std::string last;
    for (const auto& r : rows) {
        if (r.fixture != last) {
            std::cout << "\n  " << r.fixture << " [" << to_string(r.kind) << "]  variant=" << r.variant;
            for (std::size_t i = 0; i < cond_cols.size() && i < r.conditioning.size(); ++i)
                std::cout << "  " << cond_cols[i] << '=' << fmt(r.conditioning[i]);
            std::cout << '\n';
            last = r.fixture;
        }
        std::cout << "    " << std::left << std::setw(13) << r.type;
        if (r.status != "ok") {
            std::cout << r.status << '\n';
            continue;
        }
        for (std::size_t g = 0; g < r.rel_err.size(); ++g) {
            const double e = r.rel_err[g].second;
            std::ostringstream cell;
            cell << std::setprecision(2) << r.rel_err[g].first << ' ' << e << ' ' << std::fixed << std::setprecision(1)
                 << PrecisionRow::bits_lost(e, r.eps) << '/' << PrecisionRow::bits_lost(r.stage_err[g], r.eps);
            std::cout << std::left << std::setw(26) << cell.str();
        }
        std::cout << "invariants " << (r.invariants - r.failed) << '/' << r.invariants;
        if (r.failed)
            std::cout << "  first FAIL " << r.first_failed;
        std::cout << '\n';
    }
}

/// One row per fixture × type × output group.
inline void write_precision_csv(const std::vector<PrecisionRow>& rows,
                                const std::vector<std::string>& cond_cols,
                                const std::filesystem::path& path) {
    if (path.has_parent_path())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    out << "fixture,kind,variant,type";
    for (const auto& c : cond_cols)
        out << ',' << csv_field(c);
    out << ",group,eps,abs_err,rel_err,amplification,bits_lost,stage_rel_err,stage_bits_lost,invariants,failed,"
           "worst_margin,"
           "first_failed,status\n";
    out.precision(17);
    for (const auto& r : rows)
        for (std::size_t gi = 0; gi < r.rel_err.size(); ++gi) {
            const auto& [g, e] = r.rel_err[gi];
            const double se = r.stage_err[gi];
            out << csv_field(r.fixture) << ',' << to_string(r.kind) << ',' << csv_field(r.variant) << ',' << r.type;
            for (std::size_t i = 0; i < cond_cols.size(); ++i)
                out << ',' << (i < r.conditioning.size() ? r.conditioning[i] : inv::detail::kInf);
            out << ',' << csv_field(g) << ',' << r.eps << ',' << r.abs_err[gi] << ',' << e << ','
                << PrecisionRow::amplification(e, r.eps) << ',' << PrecisionRow::bits_lost(e, r.eps) << ',' << se << ','
                << PrecisionRow::bits_lost(se, r.eps) << ',' << r.invariants << ',' << r.failed << ',' << r.worst_margin
                << ',' << csv_field(r.first_failed) << ',' << csv_field(r.status) << '\n';
        }
}

// ── CLI ─────────────────────────────────────────────────────────────────────

struct BenchArgs {
    std::vector<std::string> fixtures;  ///< --fixture <file> (repeatable); none ⇒ the built-ins
    std::vector<std::string> types;     ///< --types double,float,posit32 (empty ⇒ the default set)
    std::string variant;                ///< --variant <name>|all ("" ⇒ shipped)
    std::string csv;                    ///< --csv <dir>: report.csv (+ sweep CSVs)
    std::vector<std::string> sweeps;    ///< --sweep name=v1,v2,… (repeatable); presence runs the sweep
    bool sweep = false;                 ///< --sweep given (with or without overrides)
    std::string capture;                ///< --capture <dir>: write the built-in fixtures as files
    bool precision = false;             ///< --precision: the per-type precision characterization (#450)
    bool help = false;
};

[[nodiscard]] inline std::vector<std::string> split_csv(std::string_view s) {
    std::vector<std::string> out;
    std::stringstream ss{std::string(s)};
    std::string item;
    while (std::getline(ss, item, ','))
        if (!item.empty())
            out.push_back(item);
    return out;
}

[[nodiscard]] inline BenchArgs parse_bench_args(int argc, char** argv) {
    BenchArgs a;
    for (int i = 1; i < argc; ++i) {
        const std::string_view v = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc)
                throw std::invalid_argument("bench: " + std::string(v) + " needs a value");
            return argv[++i];
        };
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--fixture")
            a.fixtures.push_back(next());
        else if (v == "--types")
            a.types = split_csv(next());
        else if (v == "--variant")
            a.variant = next();
        else if (v == "--csv")
            a.csv = next();
        else if (v == "--capture")
            a.capture = next();
        else if (v == "--precision")
            a.precision = true;
        else if (v == "--sweep") {
            a.sweep = true;
            if (i + 1 < argc && std::string_view(argv[i + 1]).find('=') != std::string_view::npos)
                a.sweeps.push_back(argv[++i]);
        } else
            throw std::invalid_argument("bench: unknown flag " + std::string(v));
    }
    return a;
}

/// The stage's contract from the registry, by its S<N> prefix.
[[nodiscard]] inline const StageInfo* find_contract(std::string_view stage) {
    static const std::vector<StageInfo> all = pipeline();
    std::string_view id = stage.substr(0, stage.find('_'));
    for (const auto& s : all)
        if (s.id == id)
            return &s;
    // A sub-step (S6a … S6e) shares its stage's contract (S6).
    while (!id.empty() && id.back() >= 'a' && id.back() <= 'z')
        id.remove_suffix(1);
    for (const auto& s : all)
        if (s.id == id)
            return &s;
    return nullptr;
}

template <class B>
concept HasSweep = requires { B::default_sweep(); };

/// Run the characterization sweep of `B` in every selected type: one CSV per
/// type, columns = the sweep axes followed by `B::sweep_columns()`.
template <class B, class... Ts>
void run_sweep(const BenchArgs& a, TypeList<Ts...> types) {
    Sweep sw = B::default_sweep();
    for (const auto& spec : a.sweeps)
        sw.axis_spec(spec);
    const std::vector<std::string> selected = a.types.empty() ? type_names(DefaultTypes{}) : a.types;
    for_each_type(types, [&]<class T>() {
        const std::string name(type_name<T>());
        if (std::find(selected.begin(), selected.end(), name) == selected.end())
            return;
        std::vector<std::string> header;
        for (const auto& ax : sw.axes())
            header.push_back(ax.first);
        for (const auto& c : B::sweep_columns())
            header.push_back(c);
        CsvTable table(header);
        sw.run([&](const Point& p) {
            std::vector<double> row;
            for (const auto& ax : sw.axes())
                row.push_back(p.at(ax.first));
            for (const double m : B::template sweep_point<T>(p))
                row.push_back(m);
            table.row(row);
        });
        const std::filesystem::path dir =
            a.csv.empty() ? std::filesystem::path("build/stage_benches") / B::kStage : std::filesystem::path(a.csv);
        const auto file = dir / (std::string(B::kStage) + "_sweep_" + name + ".csv");
        table.write(file);
        std::cout << "  sweep: " << table.rows() << " points × " << name << " → " << file.string() << '\n';
    });
}

/// `--precision`: the fixtures (the built-ins, or `--fixture`'s) and, with the
/// built-ins, the bench's conditioning ladder, in every selected type. Writes
/// `<stage>_precision.csv`. A characterization, so a non-double type breaking
/// the contract is reported, not failed; returns non-zero only if the double
/// run itself breaks it, or nothing ran.
template <class B>
int precision_main(const BenchArgs& a, std::vector<NamedFixture> fixtures) {
    if constexpr (requires { B::precision_fixtures(); })
        if (a.fixtures.empty())
            for (auto& nf : B::precision_fixtures())
                fixtures.push_back(std::move(nf));
    const auto cond_cols = conditioning_columns_of<B>();
    std::vector<PrecisionRow> rows;
    bool ok = true;
    for (const auto& v : select_variants(B::variants(), a.variant))
        for (const auto& nf : fixtures) {
            if (nf.fixture.stage != B::kStage)
                continue;
            try {
                for (auto& r : precision_rows<B>(
                         nf, v.name, bench_types<B>(), a.types.empty() ? precision_types_for<B>(v.name) : a.types)) {
                    if (r.type == "double" && (r.status != "ok" || r.failed))
                        ok = false;
                    rows.push_back(std::move(r));
                }
            } catch (const std::exception& e) {  // the double reference could not run
                std::cerr << "  " << nf.name << ": " << e.what() << '\n';
                ok = false;
            }
        }
    if (rows.empty()) {
        std::cerr << "  " << B::kStage << ": no precision run — check --fixture / --types\n";
        return 1;
    }
    print_precision(rows, cond_cols);
    const std::filesystem::path dir =
        a.csv.empty() ? std::filesystem::path("build/stage_benches") / B::kStage : std::filesystem::path(a.csv);
    const auto file = dir / (std::string(B::kStage) + "_precision.csv");
    write_precision_csv(rows, cond_cols, file);
    std::cout << "\n  precision: " << rows.size() << " runs → " << file.string() << '\n';
    std::cout << "\n  " << B::kStage << ": " << (ok ? "double reference PASS" : "DOUBLE REFERENCE FAILURES") << '\n';
    return ok ? 0 : 1;
}

/// The `main` of a bench executable. Returns 0 iff every invariant passed.
template <class B>
int bench_main(int argc, char** argv) {
    BenchArgs a;
    try {
        a = parse_bench_args(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 2;
    }
    if (const StageInfo* c = find_contract(B::kStage))
        print_contract(*c);
    if (a.help) {
        std::cout << "\n  usage: " << B::kStage
                  << "_bench [--fixture F.json]... [--types double,float,posit32,posit16]\n"
                  << "         [--variant NAME|all] [--csv DIR] [--sweep [name=v1,v2,...]]... [--capture DIR]\n"
                  << "         [--precision]\n"
                  << "  variants:";
        for (const auto& v : B::variants())
            std::cout << "  " << v.name;
        std::cout << '\n';
        return 0;
    }

    std::vector<NamedFixture> fixtures;
    if (a.fixtures.empty())
        fixtures = B::builtin_fixtures();
    for (const auto& path : a.fixtures)
        fixtures.push_back({std::filesystem::path(path).stem().string(), load(path)});

    if (!a.capture.empty()) {
        for (const auto& nf : fixtures)
            save(nf.fixture, std::filesystem::path(a.capture) / (nf.name + ".json"));
        std::cout << "  wrote " << fixtures.size() << " fixture(s) to " << a.capture << '\n';
    }

    if (a.precision)
        return precision_main<B>(a, fixtures);

    std::vector<FixtureRun> runs;
    bool ok = true;
    std::size_t type_runs = 0;
    for (const auto& v : select_variants(B::variants(), a.variant))
        for (const auto& nf : fixtures) {
            if (nf.fixture.stage != B::kStage) {
                std::cerr << "  skipping " << nf.name << ": fixture is for " << nf.fixture.stage << '\n';
                continue;
            }
            runs.push_back(
                run_fixture<B>(nf, v.name, bench_types<B>(), a.types.empty() ? default_types_for<B>(v.name) : a.types));
            print_run(runs.back());
            ok = ok && runs.back().pass();
            type_runs += runs.back().types.size();
        }
    // Nothing ran (no fixture for this stage, or --types named no known type):
    // that is a failure, never a vacuous "all invariants PASS".
    if (type_runs == 0) {
        std::cerr << "  " << B::kStage << ": no stage run — check --fixture / --types (known types:";
        for (const auto& t : type_names(bench_types<B>()))
            std::cerr << ' ' << t;
        std::cerr << ")\n";
        ok = false;
    }
    if (!a.csv.empty())
        write_report_csv(runs, std::filesystem::path(a.csv) / (std::string(B::kStage) + "_report.csv"));
    if constexpr (HasSweep<B>) {
        if (a.sweep) {
            try {
                run_sweep<B>(a, bench_types<B>());
            } catch (const std::exception& e) {  // a bad --sweep axis, reported like a bad flag
                std::cerr << "  " << B::kStage << ": " << e.what() << '\n';
                return 2;
            }
        }
    }
    std::cout << "\n  " << B::kStage << ": " << (ok ? "all invariants PASS" : "INVARIANT FAILURES") << '\n';
    return ok ? 0 : 1;
}

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_BENCH_HPP
