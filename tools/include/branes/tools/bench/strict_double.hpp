// SPDX-License-Identifier: MIT
//
// branes/tools/bench/strict_double.hpp — a strict-conversion arithmetic type for
// the genericity audit (issue #449, epic #444 §C).
//
// `StrictDouble` is an IEEE double that will not silently turn back into one:
// constructing it from any arithmetic type is implicit (as for a posit), but
// converting it to double, float or an integer takes an explicit cast. So a
// stage instantiated with it fails to compile at every place where the
// estimation path narrows T to double without saying so: a `double x = t;`, a
// `std::sqrt(t)` that picks the double overload, a T passed where a double is
// taken.
//
// Every operation is the double operation, so a stage that is generic computes
// exactly what it computes in double. The benches hold it to that: in
// `StrictDouble` every fixture's output must match the double run bit for bit.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_STRICT_DOUBLE_HPP
#define BRANES_TOOLS_BENCH_STRICT_DOUBLE_HPP

#include <cmath>
#include <compare>
#include <concepts>
#include <limits>
#include <ostream>
#include <type_traits>

namespace branes::tools::bench {

namespace strict {

struct StrictDouble {
    double v;

    StrictDouble() = default;  // trivially constructible, like double

    // One non-template overload per arithmetic type, in and out: a constrained
    // template conversion crashes MSVC's optimizer (C1001), and a single double
    // constructor would make T{n} from a non-constant integer a narrowing error.
#define BRANES_STRICT_CONVERSIONS(A)                                                                                   \
    constexpr StrictDouble(A x) : v(static_cast<double>(x)) {}                                                         \
    constexpr explicit operator A() const {                                                                            \
        return static_cast<A>(v);                                                                                      \
    }
    BRANES_STRICT_CONVERSIONS(double)
    BRANES_STRICT_CONVERSIONS(float)
    BRANES_STRICT_CONVERSIONS(long double)
    BRANES_STRICT_CONVERSIONS(int)
    BRANES_STRICT_CONVERSIONS(unsigned)
    BRANES_STRICT_CONVERSIONS(long)
    BRANES_STRICT_CONVERSIONS(unsigned long)
    BRANES_STRICT_CONVERSIONS(long long)
    BRANES_STRICT_CONVERSIONS(unsigned long long)
#undef BRANES_STRICT_CONVERSIONS

    constexpr StrictDouble operator-() const {
        return StrictDouble(-v);
    }
    constexpr StrictDouble operator+() const {
        return *this;
    }
    constexpr StrictDouble& operator+=(StrictDouble o) {
        v += o.v;
        return *this;
    }
    constexpr StrictDouble& operator-=(StrictDouble o) {
        v -= o.v;
        return *this;
    }
    constexpr StrictDouble& operator*=(StrictDouble o) {
        v *= o.v;
        return *this;
    }
    constexpr StrictDouble& operator/=(StrictDouble o) {
        v /= o.v;
        return *this;
    }
    /// The next representable value (as for a posit's ++).
    StrictDouble& operator++() {
        v = std::nextafter(v, std::numeric_limits<double>::infinity());
        return *this;
    }

    friend constexpr StrictDouble operator+(StrictDouble a, StrictDouble b) {
        return StrictDouble(a.v + b.v);
    }
    friend constexpr StrictDouble operator-(StrictDouble a, StrictDouble b) {
        return StrictDouble(a.v - b.v);
    }
    friend constexpr StrictDouble operator*(StrictDouble a, StrictDouble b) {
        return StrictDouble(a.v * b.v);
    }
    friend constexpr StrictDouble operator/(StrictDouble a, StrictDouble b) {
        return StrictDouble(a.v / b.v);
    }
    friend constexpr bool operator==(StrictDouble a, StrictDouble b) {
        return a.v == b.v;
    }
    friend constexpr std::partial_ordering operator<=>(StrictDouble a, StrictDouble b) {
        return a.v <=> b.v;
    }
    friend std::ostream& operator<<(std::ostream& os, StrictDouble a) {
        return os << a.v;
    }
};

// The math the pipeline calls on T, found by ADL. Each is the double function.
#define BRANES_STRICT_UNARY(fn)                                                                                        \
    inline StrictDouble fn(StrictDouble a) {                                                                           \
        return StrictDouble(std::fn(a.v));                                                                             \
    }
BRANES_STRICT_UNARY(sqrt)
BRANES_STRICT_UNARY(abs)
BRANES_STRICT_UNARY(fabs)
BRANES_STRICT_UNARY(sin)
BRANES_STRICT_UNARY(cos)
BRANES_STRICT_UNARY(tan)
BRANES_STRICT_UNARY(asin)
BRANES_STRICT_UNARY(acos)
BRANES_STRICT_UNARY(atan)
BRANES_STRICT_UNARY(exp)
BRANES_STRICT_UNARY(log)
BRANES_STRICT_UNARY(cbrt)
BRANES_STRICT_UNARY(floor)
BRANES_STRICT_UNARY(ceil)
BRANES_STRICT_UNARY(round)
#undef BRANES_STRICT_UNARY
inline StrictDouble atan2(StrictDouble y, StrictDouble x) {
    return StrictDouble(std::atan2(y.v, x.v));
}
inline StrictDouble hypot(StrictDouble x, StrictDouble y) {
    return StrictDouble(std::hypot(x.v, y.v));
}
inline StrictDouble pow(StrictDouble x, StrictDouble y) {
    return StrictDouble(std::pow(x.v, y.v));
}
inline StrictDouble fmod(StrictDouble x, StrictDouble y) {
    return StrictDouble(std::fmod(x.v, y.v));
}
inline StrictDouble copysign(StrictDouble x, StrictDouble y) {
    return StrictDouble(std::copysign(x.v, y.v));
}
inline bool isfinite(StrictDouble a) {
    return std::isfinite(a.v);
}
inline bool isnan(StrictDouble a) {
    return std::isnan(a.v);
}
inline bool isinf(StrictDouble a) {
    return std::isinf(a.v);
}
inline bool signbit(StrictDouble a) {
    return std::signbit(a.v);
}

}  // namespace strict

using strict::StrictDouble;

}  // namespace branes::tools::bench

/// The limits are double's, so ε_T-scaled tolerances are double's too.
namespace std {
template <>
class numeric_limits<branes::tools::bench::StrictDouble> {
    using S = branes::tools::bench::StrictDouble;
    using D = std::numeric_limits<double>;

public:
    static constexpr bool is_specialized = true;
    static constexpr bool is_signed = true;
    static constexpr bool is_integer = false;
    static constexpr bool is_exact = false;
    static constexpr bool has_infinity = true;
    static constexpr bool has_quiet_NaN = true;
    static constexpr bool has_signaling_NaN = true;
    static constexpr bool is_iec559 = true;
    static constexpr bool is_bounded = true;
    static constexpr bool is_modulo = false;
    static constexpr std::float_denorm_style has_denorm = D::has_denorm;
    static constexpr bool has_denorm_loss = D::has_denorm_loss;
    static constexpr bool traps = D::traps;
    static constexpr bool tinyness_before = D::tinyness_before;
    static constexpr int min_exponent10 = D::min_exponent10;
    static constexpr int max_exponent10 = D::max_exponent10;
    static constexpr int digits = D::digits;
    static constexpr int digits10 = D::digits10;
    static constexpr int max_digits10 = D::max_digits10;
    static constexpr int radix = D::radix;
    static constexpr int min_exponent = D::min_exponent;
    static constexpr int max_exponent = D::max_exponent;
    static constexpr std::float_round_style round_style = D::round_style;
    static constexpr S min() noexcept {
        return S(D::min());
    }
    static constexpr S max() noexcept {
        return S(D::max());
    }
    static constexpr S lowest() noexcept {
        return S(D::lowest());
    }
    static constexpr S epsilon() noexcept {
        return S(D::epsilon());
    }
    static constexpr S round_error() noexcept {
        return S(D::round_error());
    }
    static constexpr S infinity() noexcept {
        return S(D::infinity());
    }
    static constexpr S quiet_NaN() noexcept {
        return S(D::quiet_NaN());
    }
    static constexpr S signaling_NaN() noexcept {
        return S(D::signaling_NaN());
    }
    static constexpr S denorm_min() noexcept {
        return S(D::denorm_min());
    }
};
}  // namespace std

#endif  // BRANES_TOOLS_BENCH_STRICT_DOUBLE_HPP
