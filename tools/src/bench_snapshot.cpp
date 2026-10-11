// SPDX-License-Identifier: MIT
//
// bench_snapshot — the equivalence gate for porting the VIO linear algebra to
// MTL5 (issue #448, epic #444 §C).
//
// Runs every stage bench (S0 … S10, the S6 sub-steps and the composition bench)
// on its built-in fixtures, under every variant, in double, and records each
// run's flattened output. Take a snapshot before a port, then compare after it:
// the port must reproduce every output to a relative tolerance (the issue's gate
// is 1e-12 at T = double). The tolerance is normwise per output, max|a − b| /
// max|reference|, so an entry that is rounding noise around zero does not fail
// on its own.
//
//   ./bench_snapshot --write before.json
//   ./bench_snapshot --compare before.json [--rel 1e-12]
//   ./bench_snapshot --compare-trajectory ref.csv new.csv [--rel 1e-9]
//
// The last form is the end-to-end gate: two vio_trajectory_report
// NAME_trajectory.csv files, column by column, normwise relative.
//
// Every output must be accounted for: a fixture that throws fails --write and
// --compare, an output missing from either side fails, and a non-finite value
// must be the same kind (NaN, or an infinity of the same sign) on both sides.
//
// Exit status: 0 when every output matches (or after --write), 1 otherwise,
// 2 on a usage error.

#include <branes/tools/bench/c_filter_loop_bench.hpp>
#include <branes/tools/bench/s0_sensor_model_bench.hpp>
#include <branes/tools/bench/s10_online_calibration_bench.hpp>
#include <branes/tools/bench/s1_initialization_bench.hpp>
#include <branes/tools/bench/s2_propagation_bench.hpp>
#include <branes/tools/bench/s3_augmentation_bench.hpp>
#include <branes/tools/bench/s4_frontend_bench.hpp>
#include <branes/tools/bench/s5_triangulation_bench.hpp>
#include <branes/tools/bench/s6a_jacobians_bench.hpp>
#include <branes/tools/bench/s6b_nullspace_projection_bench.hpp>
#include <branes/tools/bench/s6c_compression_bench.hpp>
#include <branes/tools/bench/s6d_gating_bench.hpp>
#include <branes/tools/bench/s6e_ekf_update_bench.hpp>
#include <branes/tools/bench/s9_marginalization_bench.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
using Snapshot = std::map<std::string, std::vector<double>>;

/// The outputs, and the runs that threw instead of producing one.
struct Taken {
    Snapshot outputs;
    std::vector<std::string> threw;
};

template <class B>
void take(Taken& t) {
    for (const auto& nf : B::builtin_fixtures())
        for (const auto& v : B::variants()) {
            const auto key = std::string(B::kStage) + "/" + nf.name + "/" + v.name;
            try {
                const auto in = B::template decode_input<double>(nf.fixture.input);
                t.outputs[key] = B::template flatten<double>(B::template run<double>(in, v.name));
            } catch (const std::exception& e) {
                t.threw.push_back(key + ": " + e.what());
            }
        }
}

Taken take_all() {
    Taken s;
    take<bn::S0SensorModelBench>(s);
    take<bn::S1InitializationBench>(s);
    take<bn::S2PropagationBench>(s);
    take<bn::S3AugmentationBench>(s);
    take<bn::S4FrontendBench>(s);
    take<bn::S5TriangulationBench>(s);
    take<bn::S6aJacobiansBench>(s);
    take<bn::S6bNullspaceProjectionBench>(s);
    take<bn::S6cCompressionBench>(s);
    take<bn::S6dGatingBench>(s);
    take<bn::S6eEkfUpdateBench>(s);
    take<bn::S9MarginalizationBench>(s);
    take<bn::S10OnlineCalibrationBench>(s);
    take<bn::CFilterLoopBench>(s);
    return s;
}

/// Normwise relative difference of `a` from the reference `r`:
/// max|a − r| / max|r| over the finite entries. `kind_differs` reports a
/// non-finite entry that is not the same on both sides (NaN vs NaN, or an
/// infinity of the same sign).
double compare_values(const std::vector<double>& r, const std::vector<double>& a, bool& kind_differs) {
    double scale = 0.0, diff = 0.0;
    kind_differs = false;
    for (std::size_t i = 0; i < r.size(); ++i) {
        if (std::isnan(r[i]) || std::isnan(a[i])) {
            kind_differs = kind_differs || !(std::isnan(r[i]) && std::isnan(a[i]));
            continue;
        }
        if (!std::isfinite(r[i]) || !std::isfinite(a[i])) {
            kind_differs = kind_differs || r[i] != a[i];
            continue;
        }
        scale = std::max(scale, std::abs(r[i]));
        diff = std::max(diff, std::abs(a[i] - r[i]));
    }
    return scale > 0.0 ? diff / scale : diff;
}

/// A numeric CSV with one header line.
bool read_csv(const std::string& path, std::string& header, std::vector<std::vector<double>>& rows) {
    std::ifstream in(path);
    if (!in || !std::getline(in, header))
        return false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty())
            continue;
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            // The whole cell must be the number ("1.5abc" is not 1.5); a trailing
            // CR from a CRLF file is whitespace, not data.
            while (!cell.empty() && std::isspace(static_cast<unsigned char>(cell.back())))
                cell.pop_back();
            std::size_t used = 0;
            const double v = std::stod(cell, &used);
            if (used != cell.size())
                throw std::invalid_argument("'" + cell + "' is not a number");
            row.push_back(v);
        }
        rows.push_back(std::move(row));
    }
    return true;
}

/// Two trajectory CSVs, column by column: 0 when every column is within `rel`.
int compare_trajectory(const std::string& ref_path, const std::string& new_path, double rel) {
    std::string h1, h2;
    std::vector<std::vector<double>> r, a;
    try {
        if (!read_csv(ref_path, h1, r) || !read_csv(new_path, h2, a)) {
            std::cerr << "bench_snapshot: cannot read " << ref_path << " or " << new_path << "\n";
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "bench_snapshot: a trajectory cell is not a number (" << e.what() << ")\n";
        return 1;
    }
    if (h1 != h2 || r.size() != a.size()) {
        std::cout << "bench_snapshot: trajectories differ in shape (" << r.size() << " vs " << a.size() << " rows"
                  << (h1 != h2 ? ", different columns" : "") << ")\n";
        return 1;
    }
    const std::size_t ncols = r.empty() ? 0 : r.front().size();
    std::size_t bad = 0;
    double worst = 0.0;
    for (std::size_t c = 0; c < ncols; ++c) {
        std::vector<double> rc, ac;
        for (std::size_t i = 0; i < r.size(); ++i) {
            if (r[i].size() != ncols || a[i].size() != ncols) {
                std::cout << "bench_snapshot: row " << i << " has the wrong number of cells\n";
                return 1;
            }
            rc.push_back(r[i][c]);
            ac.push_back(a[i][c]);
        }
        bool kind = false;
        const double d = compare_values(rc, ac, kind);
        worst = std::max(worst, d);
        if (kind || d > rel) {
            std::cout << "  DIFFERS  column " << c << "  relative " << d << (kind ? " (non-finite kind changed)" : "")
                      << "\n";
            ++bad;
        }
    }
    std::cout << "bench_snapshot: " << r.size() << " poses x " << ncols << " columns, " << bad << " columns outside "
              << rel << "; worst relative " << worst << "\n";
    return bad == 0 ? 0 : 1;
}

/// A JSON number, or the strings bench fixtures use for non-finite values.
nlohmann::json num(double x) {
    return bn::pack_num(x);
}

}  // namespace

int main(int argc, char** argv) {
    std::string write, compare, traj_ref, traj_new;
    std::optional<double> rel_arg;
    const auto usage = [] {
        std::cerr << "usage: bench_snapshot --write FILE | --compare FILE [--rel 1e-12]\n"
                     "       bench_snapshot --compare-trajectory REF.csv NEW.csv [--rel 1e-9]\n";
        return 2;
    };
    for (int i = 1; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--write" && i + 1 < argc)
            write = argv[++i];
        else if (a == "--compare" && i + 1 < argc)
            compare = argv[++i];
        else if (a == "--compare-trajectory" && i + 2 < argc) {
            traj_ref = argv[++i];
            traj_new = argv[++i];
        } else if (a == "--rel" && i + 1 < argc) {
            try {
                rel_arg = std::stod(argv[++i]);
            } catch (const std::exception&) {
                return usage();
            }
        } else
            return usage();
    }
    if ((write.empty() ? 0 : 1) + (compare.empty() ? 0 : 1) + (traj_ref.empty() ? 0 : 1) != 1) {
        std::cerr << "bench_snapshot: give exactly one of --write / --compare / --compare-trajectory\n";
        return 2;
    }
    if (rel_arg && !(std::isfinite(*rel_arg) && *rel_arg >= 0.0)) {
        std::cerr << "bench_snapshot: --rel must be a finite number >= 0\n";
        return 2;
    }
    if (!traj_ref.empty())
        return compare_trajectory(traj_ref, traj_new, rel_arg.value_or(1e-9));
    const double rel = rel_arg.value_or(1e-12);

    const Taken taken = take_all();
    const Snapshot& now = taken.outputs;
    for (const auto& t : taken.threw)
        std::cout << "  THREW    " << t << "\n";
    if (!write.empty()) {
        if (!taken.threw.empty()) {
            std::cerr << "bench_snapshot: " << taken.threw.size() << " runs threw; no snapshot written\n";
            return 1;
        }
        nlohmann::json j = nlohmann::json::object();
        for (const auto& [k, v] : now) {
            auto& a = j[k] = nlohmann::json::array();
            for (const double x : v)
                a.push_back(num(x));
        }
        std::ofstream out(write);
        out << j.dump() << "\n";
        if (!out) {
            std::cerr << "bench_snapshot: cannot write " << write << "\n";
            return 1;
        }
        std::cout << "bench_snapshot: " << now.size() << " outputs -> " << write << "\n";
        return 0;
    }

    std::ifstream in(compare);
    if (!in) {
        std::cerr << "bench_snapshot: cannot read " << compare << "\n";
        return 1;
    }
    // The reference, decoded up front: a malformed file is an error (exit 1),
    // not an abort.
    Snapshot ref;
    try {
        const auto j = nlohmann::json::parse(in);
        if (!j.is_object())
            throw std::invalid_argument("not a JSON object");
        for (const auto& [k, jv] : j.items()) {
            if (!jv.is_array())
                throw std::invalid_argument("'" + k + "' is not an array");
            auto& v = ref[k];
            for (const auto& x : jv)
                v.push_back(bn::unpack_num(x));
        }
    } catch (const std::exception& e) {
        std::cerr << "bench_snapshot: " << compare << " is not a snapshot (" << e.what() << ")\n";
        return 1;
    }
    std::size_t bad = taken.threw.size(), checked = 0;
    double worst = 0.0;
    std::string worst_key;
    for (const auto& [k, r] : ref) {
        const auto it = now.find(k);
        if (it == now.end()) {
            std::cout << "  MISSING  " << k << "\n";
            ++bad;
            continue;
        }
        const auto& a = it->second;
        ++checked;
        if (a.size() != r.size()) {
            std::cout << "  SHAPE    " << k << "  " << r.size() << " -> " << a.size() << "\n";
            ++bad;
            continue;
        }
        bool kind_differs = false;
        const double d = compare_values(r, a, kind_differs);
        const bool finiteness = !kind_differs;
        if (d > worst) {
            worst = d;
            worst_key = k;
        }
        if (!finiteness || d > rel) {
            std::cout << "  DIFFERS  " << k << "  relative " << d << (finiteness ? "" : " (non-finite kind changed)")
                      << "\n";
            ++bad;
        }
    }
    for (const auto& [k, v] : now)
        if (ref.count(k) == 0) {
            std::cout << "  NEW      " << k << "  (not in the reference: take a new one)\n";
            ++bad;
        }
    std::cout << "bench_snapshot: " << checked << " outputs compared, " << bad << " outside " << rel
              << "; worst relative " << worst << (worst_key.empty() ? "" : " (" + worst_key + ")") << "\n";
    return bad == 0 ? 0 : 1;
}
