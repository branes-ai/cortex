// SPDX-License-Identifier: MIT
//
// branes/tools/bench/types.hpp — the arithmetic types a stage bench
// instantiates its stage in (issue #453, epic #444 §A). A bench runs the same
// transformation in several number systems side by side and reports each
// against the `double` reference, so a stage whose output moves materially with
// the arithmetic is flagged as numerically fragile.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_TYPES_HPP
#define BRANES_TOOLS_BENCH_TYPES_HPP

#include <universal/number/posit/posit.hpp>

#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

using Posit32 = sw::universal::posit<32, 2>;
using Posit16 = sw::universal::posit<16, 1>;

/// A compile-time list of arithmetic types.
template <class... Ts>
struct TypeList {};

/// The default set: the reference first, then the types under study.
using DefaultTypes = TypeList<double, float, Posit32>;

/// Every type a bench knows how to name (for `--types` selection). `long double`
/// is the wider-than-double reference (80-bit extended on x86; equal to double
/// on MSVC, where it still runs but adds no precision).
using AllTypes = TypeList<double, float, Posit32, Posit16, long double>;

/// Stable, CLI-friendly name of an arithmetic type ("double", "posit32", …).
template <class T>
[[nodiscard]] constexpr std::string_view type_name();
template <>
[[nodiscard]] constexpr std::string_view type_name<double>() {
    return "double";
}
template <>
[[nodiscard]] constexpr std::string_view type_name<float>() {
    return "float";
}
template <>
[[nodiscard]] constexpr std::string_view type_name<Posit32>() {
    return "posit32";
}
template <>
[[nodiscard]] constexpr std::string_view type_name<Posit16>() {
    return "posit16";
}
template <>
[[nodiscard]] constexpr std::string_view type_name<long double>() {
    return "long_double";
}

/// Call `f.template operator()<T>()` for each T in the list, in order.
template <class... Ts, class F>
void for_each_type(TypeList<Ts...>, F&& f) {
    (f.template operator()<Ts>(), ...);
}

/// Names of the types in a list.
template <class... Ts>
[[nodiscard]] std::vector<std::string> type_names(TypeList<Ts...>) {
    return {std::string(type_name<Ts>())...};
}

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_TYPES_HPP
