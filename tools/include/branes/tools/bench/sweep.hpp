// SPDX-License-Identifier: MIT
//
// branes/tools/bench/sweep.hpp — declarative characterization sweeps for the
// stage benches (issue #453). A sweep is a set of named axes (noise level, Δt,
// parallax, conditioning, window size, …); the bench is run at every point of
// their cartesian product and each point becomes one CSV row, the input format
// of the docs-site figure pipeline.
//
//   Sweep sw;
//   sw.axis("sigma", {0.5, 1, 2, 4}).axis("clones", {4, 8, 11});
//   CsvTable t({"sigma", "clones", "residual"});
//   sw.run([&](const Point& p) { t.row({p.at("sigma"), p.at("clones"), measure(p)}); });
//   t.write(dir / "s9_sweep.csv");
//
// On the command line, `--sweep name=v1,v2,v3` (repeatable) overrides or adds an
// axis, so a bench's built-in sweep can be re-pointed without recompiling.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_SWEEP_HPP
#define BRANES_TOOLS_BENCH_SWEEP_HPP

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace branes::tools::bench {

/// One point of a sweep: axis name → value.
using Point = std::map<std::string, double>;

class Sweep {
public:
    /// Add an axis, or replace the values of an existing one (axis order is
    /// the order of first insertion; the last axis varies fastest).
    Sweep& axis(std::string name, std::vector<double> values) {
        for (auto& a : axes_)
            if (a.first == name) {
                a.second = std::move(values);
                return *this;
            }
        axes_.emplace_back(std::move(name), std::move(values));
        return *this;
    }

    /// Parse a `name=v1,v2,…` override and apply it.
    Sweep& axis_spec(std::string_view spec) {
        const auto eq = spec.find('=');
        if (eq == std::string_view::npos || eq == 0)
            throw std::invalid_argument("sweep: expected name=v1,v2,... got '" + std::string(spec) + "'");
        std::vector<double> values;
        std::stringstream ss{std::string(spec.substr(eq + 1))};
        std::string item;
        while (std::getline(ss, item, ',')) {
            std::size_t used = 0;
            values.push_back(std::stod(item, &used));
            if (used != item.size())
                throw std::invalid_argument("sweep: bad value '" + item + "'");
        }
        if (values.empty())
            throw std::invalid_argument("sweep: axis '" + std::string(spec.substr(0, eq)) + "' has no values");
        return axis(std::string(spec.substr(0, eq)), std::move(values));
    }

    [[nodiscard]] std::size_t size() const noexcept {
        if (axes_.empty())
            return 0;
        std::size_t n = 1;
        for (const auto& a : axes_)
            n *= a.second.size();
        return n;
    }

    [[nodiscard]] const std::vector<std::pair<std::string, std::vector<double>>>& axes() const noexcept {
        return axes_;
    }

    /// Call `f(point)` for every point of the cartesian product.
    template <class F>
    void run(F&& f) const {
        const std::size_t n = size();
        for (std::size_t k = 0; k < n; ++k) {
            Point p;
            std::size_t rem = k;
            for (std::size_t a = axes_.size(); a-- > 0;) {
                const auto& [name, values] = axes_[a];
                p[name] = values[rem % values.size()];
                rem /= values.size();
            }
            f(static_cast<const Point&>(p));
        }
    }

private:
    std::vector<std::pair<std::string, std::vector<double>>> axes_;
};

/// A CSV table with a fixed header; numbers are written in round-trip precision.
class CsvTable {
public:
    explicit CsvTable(std::vector<std::string> header) : header_(std::move(header)) {}

    void row(const std::vector<double>& values) {
        if (values.size() != header_.size())
            throw std::invalid_argument("csv: row width does not match the header");
        rows_.push_back(values);
    }

    [[nodiscard]] std::size_t rows() const noexcept {
        return rows_.size();
    }

    void write(std::ostream& out) const {
        for (std::size_t i = 0; i < header_.size(); ++i)
            out << (i ? "," : "") << header_[i];
        out << '\n' << std::setprecision(std::numeric_limits<double>::max_digits10);
        for (const auto& r : rows_) {
            for (std::size_t i = 0; i < r.size(); ++i)
                out << (i ? "," : "") << r[i];
            out << '\n';
        }
    }

    void write(const std::filesystem::path& path) const {
        if (path.has_parent_path())
            std::filesystem::create_directories(path.parent_path());
        std::ofstream out(path);
        if (!out)
            throw std::runtime_error("csv: cannot write " + path.string());
        write(out);
    }

private:
    std::vector<std::string> header_;
    std::vector<std::vector<double>> rows_;
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_SWEEP_HPP
