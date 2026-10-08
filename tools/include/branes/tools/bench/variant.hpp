// SPDX-License-Identifier: MIT
//
// branes/tools/bench/variant.hpp — the variant slot of a stage bench (issue
// #453): named alternative implementations of one stage, run on identical
// fixtures and reported side by side. The first variant of every bench is
// "shipped", the implementation the pipeline runs; research variants (a
// different factorization, a compression, a gating rule, …) are added next to
// it and dispatched by name in the bench's `run`.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_VARIANT_HPP
#define BRANES_TOOLS_BENCH_VARIANT_HPP

#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

inline constexpr std::string_view kShipped = "shipped";

struct Variant {
    std::string name;
    std::string description;
};

/// Resolve a `--variant` selection against a bench's variants: "" → the
/// shipped one, "all" → every variant, otherwise a comma-separated list of names.
[[nodiscard]] inline std::vector<Variant> select_variants(const std::vector<Variant>& all, std::string_view spec) {
    if (spec.empty())
        return {all.front()};
    if (spec == "all")
        return all;
    std::vector<Variant> out;
    std::size_t start = 0;
    while (start <= spec.size()) {
        const std::size_t comma = spec.find(',', start);
        const std::string_view name = spec.substr(start, comma == std::string_view::npos ? spec.npos : comma - start);
        bool found = false;
        for (const auto& v : all)
            if (v.name == name) {
                out.push_back(v);
                found = true;
            }
        if (!found)
            throw std::invalid_argument("bench: unknown variant '" + std::string(name) + "'");
        if (comma == std::string_view::npos)
            break;
        start = comma + 1;
    }
    return out;
}

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_VARIANT_HPP
