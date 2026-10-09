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
    if (!f.expected.is_null() && (f.variant.empty() || f.variant == variant)) {
        const std::vector<double> want = B::template flatten<T>(B::template decode_output<T>(f.expected));
        if (f.kind == FixtureKind::Captured && f.arithmetic == tr.type) {
            // A captured fixture must replay to exactly the output it recorded.
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

// ── CLI ─────────────────────────────────────────────────────────────────────

struct BenchArgs {
    std::vector<std::string> fixtures;  ///< --fixture <file> (repeatable); none ⇒ the built-ins
    std::vector<std::string> types;     ///< --types double,float,posit32 (empty ⇒ the default set)
    std::string variant;                ///< --variant <name>|all ("" ⇒ shipped)
    std::string csv;                    ///< --csv <dir>: report.csv (+ sweep CSVs)
    std::vector<std::string> sweeps;    ///< --sweep name=v1,v2,… (repeatable); presence runs the sweep
    bool sweep = false;                 ///< --sweep given (with or without overrides)
    std::string capture;                ///< --capture <dir>: write the built-in fixtures as files
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
    const std::string_view id = stage.substr(0, stage.find('_'));
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
                run_fixture<B>(nf, v.name, bench_types<B>(), a.types.empty() ? type_names(DefaultTypes{}) : a.types));
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
