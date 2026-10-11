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
//
// Exit status: 0 when every output matches (or after --write), 1 otherwise.

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
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
using Snapshot = std::map<std::string, std::vector<double>>;

template <class B>
void take(Snapshot& snap) {
    for (const auto& nf : B::builtin_fixtures())
        for (const auto& v : B::variants()) {
            const auto key = std::string(B::kStage) + "/" + nf.name + "/" + v.name;
            try {
                const auto in = B::template decode_input<double>(nf.fixture.input);
                snap[key] = B::template flatten<double>(B::template run<double>(in, v.name));
            } catch (const std::exception& e) {
                std::cerr << "  " << key << ": " << e.what() << "\n";
                snap[key] = {};  // recorded as "threw"; a compare requires the same
            }
        }
}

Snapshot take_all() {
    Snapshot s;
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

/// A JSON number, or the strings bench fixtures use for non-finite values.
nlohmann::json num(double x) {
    return bn::pack_num(x);
}

}  // namespace

int main(int argc, char** argv) {
    std::string write, compare;
    double rel = 1e-12;
    for (int i = 1; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--write" && i + 1 < argc)
            write = argv[++i];
        else if (a == "--compare" && i + 1 < argc)
            compare = argv[++i];
        else if (a == "--rel" && i + 1 < argc)
            rel = std::stod(argv[++i]);
        else {
            std::cerr << "usage: bench_snapshot --write FILE | --compare FILE [--rel 1e-12]\n";
            return 2;
        }
    }
    if (write.empty() == compare.empty()) {
        std::cerr << "bench_snapshot: give exactly one of --write / --compare\n";
        return 2;
    }

    const Snapshot now = take_all();
    if (!write.empty()) {
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
    const auto ref = nlohmann::json::parse(in);
    std::size_t bad = 0, checked = 0;
    double worst = 0.0;
    std::string worst_key;
    for (const auto& [k, jv] : ref.items()) {
        const auto it = now.find(k);
        if (it == now.end()) {
            std::cout << "  MISSING  " << k << "\n";
            ++bad;
            continue;
        }
        std::vector<double> r;
        for (const auto& x : jv)
            r.push_back(bn::unpack_num(x));
        const auto& a = it->second;
        ++checked;
        if (a.size() != r.size()) {
            std::cout << "  SHAPE    " << k << "  " << r.size() << " -> " << a.size() << "\n";
            ++bad;
            continue;
        }
        double scale = 0.0, diff = 0.0;
        bool finiteness = true;
        for (std::size_t i = 0; i < r.size(); ++i) {
            if (std::isfinite(r[i]) != std::isfinite(a[i]) || std::isnan(r[i]) != std::isnan(a[i])) {
                finiteness = false;
                continue;
            }
            if (!std::isfinite(r[i]))
                continue;
            scale = std::max(scale, std::abs(r[i]));
            diff = std::max(diff, std::abs(a[i] - r[i]));
        }
        const double d = scale > 0.0 ? diff / scale : diff;
        if (d > worst) {
            worst = d;
            worst_key = k;
        }
        if (!finiteness || d > rel) {
            std::cout << "  DIFFERS  " << k << "  relative " << d << (finiteness ? "" : " (finiteness changed)")
                      << "\n";
            ++bad;
        }
    }
    for (const auto& [k, v] : now)
        if (!ref.contains(k))
            std::cout << "  NEW      " << k << "\n";
    std::cout << "bench_snapshot: " << checked << " outputs compared, " << bad << " outside " << rel
              << "; worst relative " << worst << (worst_key.empty() ? "" : " (" + worst_key + ")") << "\n";
    return bad == 0 ? 0 : 1;
}
