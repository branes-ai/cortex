// SPDX-License-Identifier: MIT
//
// branes/bench/rapl.hpp — empirical CPU energy via Linux RAPL / powercap.
//
// Reads cumulative energy counters from /sys/class/powercap/intel-rapl:*
// (Intel) or the equivalent AMD powercap zones, around a workload, to get
// energy (J) and average power (W). The counters are monotonically
// increasing micro-joule registers that wrap at `max_energy_range_uj`; the
// wrap-correcting delta math is factored out as a pure function so it can
// be unit-tested without the sysfs tree.
//
// Availability is best-effort: since CVE-2020-8694, `energy_uj` is often
// root-readable only, so a non-privileged run reports "unavailable" rather
// than failing the benchmark.
//
// Header-only, C++20.

#ifndef BRANES_BENCH_RAPL_HPP
#define BRANES_BENCH_RAPL_HPP

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace branes::bench::rapl {

/// One powercap zone reading: a counter value and its wrap range.
struct ZoneReading {
    std::string name;                ///< e.g. "package-0", "dram"
    std::uint64_t energy_uj = 0;     ///< current counter (micro-joules)
    std::uint64_t max_range_uj = 0;  ///< wrap point; 0 = unknown (assume no wrap)
};

/// Wrap-correcting energy delta (micro-joules) between an earlier reading
/// `a` and a later reading `b` of the SAME counter. If b < a the counter
/// wrapped once: delta = (max - a) + b. With an unknown range (0) and a
/// decrease, the best we can do is treat it as no progress (0).
[[nodiscard]] inline std::uint64_t delta_uj(const ZoneReading& a, const ZoneReading& b) {
    if (b.energy_uj >= a.energy_uj)
        return b.energy_uj - a.energy_uj;
    if (b.max_range_uj > a.energy_uj)
        return (b.max_range_uj - a.energy_uj) + b.energy_uj;
    return 0;  // wrapped with unknown range — can't recover
}

/// Sum of wrap-corrected deltas across all matching zones (by name), in
/// micro-joules. Zones present in one snapshot but not the other are
/// ignored. `before`/`after` are parallel zone lists.
[[nodiscard]] inline std::uint64_t total_delta_uj(const std::vector<ZoneReading>& before,
                                                  const std::vector<ZoneReading>& after) {
    std::uint64_t sum = 0;
    for (const auto& a : before)
        for (const auto& b : after)
            if (a.name == b.name) {
                sum += delta_uj(a, b);
                break;
            }
    return sum;
}

namespace detail {
[[nodiscard]] inline bool read_u64(const std::filesystem::path& p, std::uint64_t& out) {
    std::ifstream in(p);
    return static_cast<bool>(in >> out);
}
[[nodiscard]] inline std::string read_line(const std::filesystem::path& p) {
    std::ifstream in(p);
    std::string s;
    std::getline(in, s);
    return s;
}
}  // namespace detail

/// Snapshot every readable top-level powercap zone (package/psys/dram). An
/// empty result means RAPL is unavailable (no zones, or `energy_uj` not
/// readable by this user).
[[nodiscard]] inline std::vector<ZoneReading> snapshot() {
    namespace fs = std::filesystem;
    std::vector<ZoneReading> zones;
    const fs::path root{"/sys/class/powercap"};
    std::error_code ec;
    if (!fs::exists(root, ec))
        return zones;
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        const auto fname = entry.path().filename().string();
        // Top-level domains are named intel-rapl:N (or similar); skip the
        // :N:M sub-zones to avoid double-counting energy already in the parent.
        if (fname.rfind("intel-rapl:", 0) != 0 && fname.rfind("intel-rapl-mmio:", 0) != 0 &&
            fname.rfind("amd-rapl:", 0) != 0)
            continue;
        if (fname.find(':') != fname.rfind(':'))
            continue;  // a sub-zone (two colons) — skip
        ZoneReading z;
        if (!detail::read_u64(entry.path() / "energy_uj", z.energy_uj))
            continue;  // unreadable (permissions) — RAPL effectively unavailable here
        z.name = detail::read_line(entry.path() / "name");
        if (z.name.empty())
            z.name = fname;
        std::uint64_t maxr = 0;
        if (detail::read_u64(entry.path() / "max_energy_range_uj", maxr))
            z.max_range_uj = maxr;
        zones.push_back(std::move(z));
    }
    return zones;
}

/// Measures energy across a scope. Construct to snapshot the start, call
/// `joules()` after the workload. `available()` is false when RAPL can't
/// be read (no zones captured at construction).
class EnergyMeter {
public:
    EnergyMeter() : before_(snapshot()) {}

    [[nodiscard]] bool available() const noexcept {
        return !before_.empty();
    }

    /// Energy consumed (joules) since construction. Returns 0 when RAPL is
    /// unavailable.
    [[nodiscard]] double joules() const {
        if (before_.empty())
            return 0.0;
        return static_cast<double>(total_delta_uj(before_, snapshot())) * 1e-6;
    }

private:
    std::vector<ZoneReading> before_;
};

}  // namespace branes::bench::rapl

#endif  // BRANES_BENCH_RAPL_HPP
