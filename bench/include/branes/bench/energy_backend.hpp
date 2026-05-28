// SPDX-License-Identifier: MIT
//
// branes/bench/energy_backend.hpp — pluggable empirical energy measurement.
//
// vio_bench measures energy around a run through an EnergyBackend so the
// same benchmark works on different hardware: RAPL on an x86 host, Jetson
// `tegrastats` on an edge board (see tegrastats.hpp), or a generic external
// meter exposing a cumulative-energy counter (INA/Joulescope exporter, a
// board rail in sysfs). All backends share counter semantics: construct to
// start, call joules() once after the workload.
//
// Header-only, C++20.

#ifndef BRANES_BENCH_ENERGY_BACKEND_HPP
#define BRANES_BENCH_ENERGY_BACKEND_HPP

#include <branes/bench/rapl.hpp>

#include <cstdint>
#include <string>

namespace branes::bench {

/// A measured-energy source. `available()` reflects whether the backend can
/// read energy on this host; `joules()` returns the energy consumed since
/// construction (0 when unavailable).
class EnergyBackend {
public:
    virtual ~EnergyBackend() = default;
    [[nodiscard]] virtual bool available() const = 0;
    [[nodiscard]] virtual double joules() const = 0;
    [[nodiscard]] virtual const char* name() const = 0;
};

/// Linux RAPL / powercap (x86 host). Thin wrapper over rapl::EnergyMeter.
class RaplBackend final : public EnergyBackend {
public:
    [[nodiscard]] bool available() const override {
        return meter_.available();
    }
    [[nodiscard]] double joules() const override {
        return meter_.joules();
    }
    [[nodiscard]] const char* name() const override {
        return "rapl";
    }

private:
    rapl::EnergyMeter meter_;
};

/// Generic external meter exposing a monotonic **cumulative energy counter
/// in micro-joules** at a file path (e.g. an INA/Joulescope exporter, or a
/// board power-rail sysfs node). Energy is the delta between construction
/// and joules(). No wrap handling — a 64-bit µJ counter doesn't wrap in any
/// realistic benchmark window.
class CounterFileBackend final : public EnergyBackend {
public:
    explicit CounterFileBackend(std::string path) : path_(std::move(path)) {
        ok_ = rapl::detail::read_u64(path_, start_uj_);
    }
    [[nodiscard]] bool available() const override {
        return ok_;
    }
    [[nodiscard]] double joules() const override {
        std::uint64_t now = 0;
        if (!ok_ || !rapl::detail::read_u64(path_, now))
            return 0.0;
        return now >= start_uj_ ? static_cast<double>(now - start_uj_) * 1e-6 : 0.0;
    }
    [[nodiscard]] const char* name() const override {
        return "external";
    }

private:
    std::string path_;
    std::uint64_t start_uj_ = 0;
    bool ok_ = false;
};

/// Selectable backend kinds (parsed from `--energy`).
enum class EnergyKind { Rapl, Tegrastats, External };

/// Parse a backend name; returns false on an unknown name.
[[nodiscard]] inline bool energy_kind_from_string(const std::string& s, EnergyKind& out) {
    if (s == "rapl") {
        out = EnergyKind::Rapl;
        return true;
    }
    if (s == "tegrastats") {
        out = EnergyKind::Tegrastats;
        return true;
    }
    if (s == "external") {
        out = EnergyKind::External;
        return true;
    }
    return false;
}

}  // namespace branes::bench

#endif  // BRANES_BENCH_ENERGY_BACKEND_HPP
