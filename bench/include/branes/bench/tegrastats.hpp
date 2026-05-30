// SPDX-License-Identifier: MIT
//
// branes/bench/tegrastats.hpp — Jetson on-device energy via `tegrastats`.
//
// tegrastats streams instantaneous power-rail readings; energy is the time
// integral of total board power over the run. This header provides the two
// pure, unit-tested pieces — a power-line parser and a trapezoidal
// power→energy integrator — and a POSIX backend that spawns tegrastats,
// samples its output on a reader thread, and integrates. On a host without
// tegrastats the backend simply reports unavailable.
//
// The spawning backend needs POSIX process/pipe primitives, so on Windows
// (which has no tegrastats anyway) it degrades to an always-unavailable stub.
// The two pure helpers are portable and compile everywhere.
//
// Header-only, C++20 (POSIX backend; the Jetson target is Linux).

#ifndef BRANES_BENCH_TEGRASTATS_HPP
#define BRANES_BENCH_TEGRASTATS_HPP

#include <branes/bench/energy_backend.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if !defined(_WIN32)
#include <sys/wait.h>

#include <csignal>

#include <unistd.h>
#endif

namespace branes::bench::tegrastats {

/// Total board/SoC input rails, in preference order. If tegrastats reports
/// one of these, it already represents total power and must be used instead
/// of summing the component rails (which would double-count).
inline const std::vector<std::string>& total_rail_names() {
    static const std::vector<std::string> names = {"VDD_IN", "VIN_SYS_5V0", "POM_5V_IN", "VDD_SYS_5V0"};
    return names;
}

/// Parse the total instantaneous power (milliwatts) from one tegrastats
/// line. Handles both the modern `NAME <mW>mW/<mW>mW` and the older
/// `POM_* <mW>/<mW>` formats. Prefers a total input rail; otherwise sums the
/// VDD_*/POM_* component rails. Returns 0 if no power token is found.
[[nodiscard]] inline double parse_power_mw(const std::string& line) {
    std::vector<std::pair<std::string, double>> rails;
    // Modern: VDD_GPU_SOC 1198mW/1198mW
    static const std::regex re_new(R"(([A-Za-z0-9_]+)\s+([0-9]+)mW/[0-9]+mW)");
    // Legacy: POM_5V_IN 4148/4148
    static const std::regex re_old(R"((POM_[A-Za-z0-9_]+)\s+([0-9]+)/[0-9]+)");
    for (const auto* re : {&re_new, &re_old}) {
        for (auto it = std::sregex_iterator(line.begin(), line.end(), *re); it != std::sregex_iterator(); ++it) {
            rails.emplace_back((*it)[1].str(), std::stod((*it)[2].str()));
        }
    }
    if (rails.empty())
        return 0.0;
    for (const auto& total : total_rail_names())
        for (const auto& [name, mw] : rails)
            if (name == total)
                return mw;
    double sum = 0.0;
    for (const auto& [name, mw] : rails)
        if (name.rfind("VDD_", 0) == 0 || name.rfind("POM_", 0) == 0)
            sum += mw;
    return sum;
}

/// Trapezoidal integral of (time_s, power_w) samples → energy in joules.
/// Fewer than two samples means no measurable interval ⇒ 0.
[[nodiscard]] inline double integrate_energy_j(const std::vector<std::pair<double, double>>& samples) {
    double energy = 0.0;
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const double dt = samples[i].first - samples[i - 1].first;
        if (dt > 0.0)
            energy += 0.5 * (samples[i].second + samples[i - 1].second) * dt;
    }
    return energy;
}

#if !defined(_WIN32)

/// Spawns `tegrastats --interval <ms>`, samples total power on a reader
/// thread, and integrates to joules over the scope. Unavailable (joules 0)
/// when tegrastats can't be launched (e.g. not on a Jetson).
class TegrastatsBackend final : public EnergyBackend {
public:
    explicit TegrastatsBackend(int interval_ms = 100) {
        int fds[2];
        if (pipe(fds) != 0)
            return;
        pid_ = fork();
        if (pid_ < 0) {
            close(fds[0]);
            close(fds[1]);
            return;
        }
        if (pid_ == 0) {  // child: tegrastats → pipe
            dup2(fds[1], STDOUT_FILENO);
            close(fds[0]);
            close(fds[1]);
            const std::string iv = std::to_string(interval_ms);
            execlp("tegrastats", "tegrastats", "--interval", iv.c_str(), static_cast<char*>(nullptr));
            _exit(127);  // exec failed (no tegrastats)
        }
        close(fds[1]);
        read_fd_ = fds[0];
        start_ = clock_now();
        reader_ = std::thread([this] { read_loop(); });
        started_ = true;
    }

    ~TegrastatsBackend() override {
        finish();
    }

    [[nodiscard]] bool available() const override {
        std::scoped_lock lk(mu_);
        return started_ && samples_.size() >= 2;
    }

    [[nodiscard]] double joules() const override {
        const_cast<TegrastatsBackend*>(this)->finish();
        std::scoped_lock lk(mu_);
        return integrate_energy_j(samples_);
    }

    [[nodiscard]] const char* name() const override {
        return "tegrastats";
    }

private:
    static double clock_now() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void read_loop() {
        std::FILE* f = fdopen(read_fd_, "r");
        if (f == nullptr)
            return;
        char* line = nullptr;
        std::size_t cap = 0;
        while (!stop_.load() && getline(&line, &cap, f) != -1) {
            const double w = parse_power_mw(line) / 1000.0;
            std::scoped_lock lk(mu_);
            samples_.emplace_back(clock_now() - start_, w);
        }
        std::free(line);
        std::fclose(f);  // also closes read_fd_
        read_fd_ = -1;
    }

    void finish() {
        if (!started_ || finished_)
            return;
        finished_ = true;
        stop_.store(true);
        if (pid_ > 0) {
            kill(pid_, SIGTERM);
            int status = 0;
            waitpid(pid_, &status, 0);
        }
        if (reader_.joinable())
            reader_.join();
    }

    pid_t pid_ = -1;
    int read_fd_ = -1;
    double start_ = 0.0;
    std::thread reader_;
    std::atomic<bool> stop_{false};
    mutable std::mutex mu_;
    std::vector<std::pair<double, double>> samples_;
    bool started_ = false;
    bool finished_ = false;
};

#else  // _WIN32

/// Windows stub: tegrastats is a Jetson/Linux tool with no Windows equivalent,
/// and the spawning backend relies on POSIX process/pipe primitives. It is
/// always unavailable (joules 0), so selecting `--energy tegrastats` on Windows
/// just reports no energy while keeping the bench buildable.
class TegrastatsBackend final : public EnergyBackend {
public:
    explicit TegrastatsBackend(int /*interval_ms*/ = 100) {}

    [[nodiscard]] bool available() const override {
        return false;
    }
    [[nodiscard]] double joules() const override {
        return 0.0;
    }
    [[nodiscard]] const char* name() const override {
        return "tegrastats";
    }
};

#endif  // _WIN32

}  // namespace branes::bench::tegrastats

#endif  // BRANES_BENCH_TEGRASTATS_HPP
