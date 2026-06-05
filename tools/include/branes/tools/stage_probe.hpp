// SPDX-License-Identifier: MIT
//
// branes/tools/stage_probe.hpp — the shared harness for the VIO stage-probe
// utilities (tools/src/sN_*.cpp).
//
// The VIO pipeline is deconstructed into stages S0–S10 (docs/arch/
// vio-pipeline-canonical.md). Each stage is a CONTRACT: a type signature, the
// pre-conditions its inputs must satisfy, the post-conditions/invariants its
// outputs must satisfy, and the native-unit assessments that measure whether
// those hold. This harness turns a contract into a self-documenting executable:
// run `s0_sensor_model --help` and it PRINTS the contract; run it for real and
// it measures the assessments and writes CSV artifacts the figure generator
// renders.
//
// These are study utilities, NOT tests and NOT daemons — no Catch2, no Zenoh.
// They let a developer focus on one stage in isolation: perturb an input, watch
// a native-unit metric move, and build intuition for where the contract bends.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_STAGE_PROBE_HPP
#define BRANES_TOOLS_STAGE_PROBE_HPP

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace branes::tools {

/// One native-unit assessment a stage emits: what it measures, in what unit,
/// and what reading means "contract holds".
struct Assessment {
    std::string name;
    std::string unit;     ///< px, px/ms, px/deg, px/mm, m/s², mm, deg, … — NATIVE units
    std::string meaning;  ///< what the reading tells you (the post-condition it tests)
};

/// The full contract for one pipeline stage.
struct StageInfo {
    std::string id;                       ///< "S0"
    std::string title;                    ///< short human title
    std::string signature;                ///< input types → output types
    std::vector<std::string> pre;         ///< pre-conditions on the inputs
    std::vector<std::string> post;        ///< post-conditions / invariants on the outputs
    std::vector<Assessment> assessments;  ///< the native-unit probes
    std::vector<std::string> artifacts;   ///< CSV files written (rendered by the figure gen)
    std::string cortex_file;              ///< the code this stage exercises
    std::string status;                   ///< "implemented" | "scaffold"
};

// ── CLI ──────────────────────────────────────────────────────────────────

struct ProbeArgs {
    std::string out;  ///< output dir for CSV artifacts ("" = don't write)
    bool help = false;
    bool list = false;
    std::vector<std::string> rest;  ///< stage-specific flags, untouched
};

/// Parse the common flags. `--out <dir>` selects the artifact directory (default
/// build/stage_probes/<id>); `--help` prints the contract; `--list` prints the
/// whole pipeline. Unrecognized flags are left in `rest` for the stage to read.
[[nodiscard]] inline ProbeArgs parse_args(int argc, char** argv, const StageInfo& s) {
    ProbeArgs a;
    a.out = "build/stage_probes/" + s.id;
    for (int i = 1; i < argc; ++i) {
        std::string_view v = argv[i];
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--list")
            a.list = true;
        else if (v == "--out" && i + 1 < argc && std::string_view(argv[i + 1]).substr(0, 2) != "--")
            a.out = argv[++i];
        else if (v == "--no-out")
            a.out.clear();
        else
            a.rest.emplace_back(v);
    }
    return a;
}

// ── Pretty printing ────────────────────────────────────────────────────────

inline void rule(char c = '-', int n = 78) {
    std::cout << std::string(n < 0 ? std::size_t{0} : static_cast<std::size_t>(n), c) << '\n';
}

inline void print_contract(const StageInfo& s) {
    rule('=');
    std::cout << "  " << s.id << " — " << s.title;
    if (s.status == "scaffold")
        std::cout << "   [SCAFFOLD: contract defined, probe not yet wired]";
    std::cout << '\n';
    rule('=');
    std::cout << "  signature : " << s.signature << '\n';
    std::cout << "  exercises : " << s.cortex_file << "\n\n";

    std::cout << "  PRE-CONDITIONS (must hold on the inputs):\n";
    for (const auto& p : s.pre)
        std::cout << "    - " << p << '\n';
    std::cout << "\n  POST-CONDITIONS / INVARIANTS (must hold on the outputs):\n";
    for (const auto& p : s.post)
        std::cout << "    + " << p << '\n';

    std::cout << "\n  ASSESSMENTS (native-unit probes):\n";
    for (const auto& m : s.assessments) {
        std::cout << "    • " << std::left << std::setw(34) << m.name << " [" << m.unit << "]\n";
        std::cout << "        " << m.meaning << '\n';
    }
    if (!s.artifacts.empty()) {
        std::cout << "\n  ARTIFACTS (CSV, rendered by docs-site/scripts/gen-sensor-model-figures.mjs):\n";
        for (const auto& f : s.artifacts)
            std::cout << "    " << f << '\n';
    }
    rule('-');
}

/// One row of a native-unit results table.
struct ResultRow {
    std::string metric;
    std::string value;  ///< formatted value
    std::string unit;
    std::string verdict;  ///< e.g. "PASS", "see figure", a short note
};

inline void print_results(const std::string& heading, const std::vector<ResultRow>& rows) {
    std::cout << "\n  " << heading << '\n';
    rule('-');
    std::cout << "  " << std::left << std::setw(46) << "metric" << std::setw(14) << "value" << std::setw(9) << "unit"
              << "note\n";
    rule('-');
    for (const auto& r : rows)
        std::cout << "  " << std::left << std::setw(46) << r.metric << std::setw(14) << r.value << std::setw(9)
                  << r.unit << r.verdict << '\n';
    rule('-');
}

/// Open a CSV artifact in the args' output dir, creating the directory if needed
/// (otherwise ofstream::open fails silently and nothing is written). Returns a
/// (possibly closed) stream — when `--no-out` was given the stream is not open.
[[nodiscard]] inline std::ofstream open_artifact(const ProbeArgs& a, const std::string& name) {
    std::ofstream f;
    if (!a.out.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(a.out, ec);
        f.open(a.out + "/" + name);
        if (!f)
            std::cerr << "stage-probe: cannot write " << a.out << "/" << name << "\n";
    }
    return f;
}

inline void note_artifacts(const ProbeArgs& a, const StageInfo& s) {
    if (a.out.empty()) {
        std::cout << "\n  (--no-out: artifacts not written)\n";
        return;
    }
    std::cout << "\n  artifacts written to: " << a.out << "/\n";
    std::cout << "  render figures:  node docs-site/scripts/gen-sensor-model-figures.mjs " << a.out
              << " docs/assessments/figures/" << s.id << "\n";
}

/// The scaffold entry point for stages whose probe is not yet wired: print the
/// contract and the planned assessments so the stage is studyable and the
/// fill-in point is explicit. Returns 0.
inline int run_scaffold(int argc, char** argv, const StageInfo& s, const std::vector<StageInfo>* pipeline = nullptr) {
    ProbeArgs a = parse_args(argc, argv, s);
    if (a.list && pipeline) {
        std::cout << "VIO pipeline stages (docs/arch/vio-pipeline-canonical.md):\n";
        for (const auto& st : *pipeline)
            std::cout << "  " << std::left << std::setw(5) << st.id << std::setw(56) << st.title << '[' << st.status
                      << "]\n";
        return 0;
    }
    print_contract(s);
    std::cout << "\n  This stage's probe is not yet wired. Its contract and native-unit\n"
                 "  assessments are defined above; implement the probe in eval/ and drive\n"
                 "  it from tools/src/"
              << s.id << "_*.cpp following tools/src/s0_sensor_model.cpp.\n";
    return 0;
}

}  // namespace branes::tools

#endif  // BRANES_TOOLS_STAGE_PROBE_HPP
