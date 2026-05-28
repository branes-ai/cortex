// SPDX-License-Identifier: MIT
//
// branes/bench/report.hpp — the vio_bench result record and its JSON +
// Markdown serializers. JSON is hand-rolled (the bench layer pulls in no
// JSON dependency) and is the contract the Phase-2 `graphs` energy model
// consumes; Markdown is the human-readable summary.
//
// Header-only, C++20.

#ifndef BRANES_BENCH_REPORT_HPP
#define BRANES_BENCH_REPORT_HPP

#include <branes/bench/operator_profile.hpp>

#include <ostream>
#include <string>
#include <vector>

namespace branes::bench {

struct BenchReport {
    std::string sequence;
    PipelineCounts counts;
    std::vector<StageProfile> profile;

    // Accuracy
    double ate_m = 0.0;
    double rpe_rmse_m = 0.0;
    double rpe_translation_pct = 0.0;
    double rpe_rotation_deg_per_m = 0.0;
    double ate_gate_m = 0.0;

    // Performance
    double sequence_duration_s = 0.0;
    double processing_s = 0.0;
    double real_time_factor = 0.0;  ///< sequence_duration / processing
    double latency_p50_ms = 0.0;
    double latency_p99_ms = 0.0;
    double fps = 0.0;

    // Energy (empirical, RAPL)
    bool rapl_available = false;
    double energy_j = 0.0;
    double energy_per_frame_mj = 0.0;
    double avg_power_w = 0.0;

    // Efficiency — only meaningful when accuracy is within the gate.
    bool accuracy_within_gate = false;
    double fps_per_watt = 0.0;  ///< 0 when power unavailable or accuracy out of gate
};

namespace detail {
inline void json_kv(std::ostream& os, const char* k, double v, bool comma = true) {
    os << "    \"" << k << "\": " << v << (comma ? ",\n" : "\n");
}
}  // namespace detail

inline void to_json(std::ostream& os, const BenchReport& r) {
    os << "{\n";
    os << "  \"sequence\": \"" << r.sequence << "\",\n";

    os << "  \"accuracy\": {\n";
    detail::json_kv(os, "ate_m", r.ate_m);
    detail::json_kv(os, "ate_gate_m", r.ate_gate_m);
    detail::json_kv(os, "rpe_rmse_m", r.rpe_rmse_m);
    detail::json_kv(os, "rpe_translation_pct", r.rpe_translation_pct);
    detail::json_kv(os, "rpe_rotation_deg_per_m", r.rpe_rotation_deg_per_m, false);
    os << "  },\n";

    os << "  \"performance\": {\n";
    detail::json_kv(os, "frames", static_cast<double>(r.counts.frames));
    detail::json_kv(os, "sequence_duration_s", r.sequence_duration_s);
    detail::json_kv(os, "processing_s", r.processing_s);
    detail::json_kv(os, "real_time_factor", r.real_time_factor);
    detail::json_kv(os, "fps", r.fps);
    detail::json_kv(os, "latency_p50_ms", r.latency_p50_ms);
    detail::json_kv(os, "latency_p99_ms", r.latency_p99_ms, false);
    os << "  },\n";

    os << "  \"energy\": {\n";
    os << "    \"rapl_available\": " << (r.rapl_available ? "true" : "false") << ",\n";
    detail::json_kv(os, "energy_j", r.energy_j);
    detail::json_kv(os, "energy_per_frame_mj", r.energy_per_frame_mj);
    detail::json_kv(os, "avg_power_w", r.avg_power_w, false);
    os << "  },\n";

    os << "  \"efficiency\": {\n";
    os << "    \"accuracy_within_gate\": " << (r.accuracy_within_gate ? "true" : "false") << ",\n";
    detail::json_kv(os, "fps_per_watt", r.fps_per_watt, false);
    os << "  },\n";

    // Per-operator profile — the Phase-2 graphs energy-model input.
    os << "  \"operator_profile\": [\n";
    for (std::size_t i = 0; i < r.profile.size(); ++i) {
        const auto& s = r.profile[i];
        os << "    { \"name\": \"" << s.name << "\", \"flops\": " << s.flops << ", \"bytes\": " << s.bytes << " }"
           << (i + 1 < r.profile.size() ? ",\n" : "\n");
    }
    os << "  ],\n";

    os << "  \"counts\": {\n";
    detail::json_kv(os, "width", static_cast<double>(r.counts.width));
    detail::json_kv(os, "height", static_cast<double>(r.counts.height));
    detail::json_kv(os, "pyramid_levels", static_cast<double>(r.counts.pyramid_levels));
    detail::json_kv(os, "avg_tracked_features", r.counts.avg_tracked_features);
    detail::json_kv(os, "avg_state_dim", r.counts.avg_state_dim);
    detail::json_kv(os, "imu_samples", static_cast<double>(r.counts.imu_samples));
    detail::json_kv(os, "camera_updates", static_cast<double>(r.counts.camera_updates));
    detail::json_kv(os, "avg_update_rows", r.counts.avg_update_rows, false);
    os << "  }\n";
    os << "}\n";
}

inline void to_markdown(std::ostream& os, const BenchReport& r) {
    os << "# VIO benchmark — " << r.sequence << "\n\n";
    os << "## Accuracy\n\n";
    os << "| Metric | Value |\n|---|---|\n";
    os << "| ATE (m) | " << r.ate_m << " (gate " << r.ate_gate_m << ") |\n";
    os << "| RPE RMSE (m) | " << r.rpe_rmse_m << " |\n";
    os << "| RPE translation (%) | " << r.rpe_translation_pct << " |\n";
    os << "| RPE rotation (deg/m) | " << r.rpe_rotation_deg_per_m << " |\n\n";

    os << "## Performance\n\n";
    os << "| Metric | Value |\n|---|---|\n";
    os << "| Frames | " << r.counts.frames << " |\n";
    os << "| Real-time factor | " << r.real_time_factor << "× |\n";
    os << "| Throughput (fps) | " << r.fps << " |\n";
    os << "| Latency p50 / p99 (ms) | " << r.latency_p50_ms << " / " << r.latency_p99_ms << " |\n\n";

    os << "## Energy (empirical, RAPL)\n\n";
    if (r.rapl_available) {
        os << "| Metric | Value |\n|---|---|\n";
        os << "| Energy (J) | " << r.energy_j << " |\n";
        os << "| Energy / frame (mJ) | " << r.energy_per_frame_mj << " |\n";
        os << "| Average power (W) | " << r.avg_power_w << " |\n";
        if (r.accuracy_within_gate)
            os << "| **Intelligence/Watt** (fps/W @ ATE≤gate) | **" << r.fps_per_watt << "** |\n";
        else
            os << "| Intelligence/Watt | n/a (ATE above gate) |\n";
    } else {
        os << "_RAPL unavailable (no powercap zones, or `energy_uj` not readable — "
              "often root-only since CVE-2020-8694). Run with energy-read access for J/W._\n";
    }
    os << "\n";

    os << "## Operator profile (first-order; Phase-2 graphs energy input)\n\n";
    os << "| Stage | GFLOPs | MB moved |\n|---|---|---|\n";
    for (const auto& s : r.profile)
        os << "| " << s.name << " | " << (s.flops / 1e9) << " | " << (s.bytes / 1e6) << " |\n";
    os << "\n";
}

}  // namespace branes::bench

#endif  // BRANES_BENCH_REPORT_HPP
