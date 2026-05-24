// SPDX-License-Identifier: MIT
//
// branes/math/arithmetic.hpp — arithmetic-type concepts for the math
// layer, refining MTL5's scalar concepts.
//
// The whole math layer is type-generic: solvers and views are
// instantiated against `double`, `float`, and Universal number types
// (posit, cfloat, fixpnt, ...). MTL5 — the library that actually
// instantiates the matrix/vector expressions — constrains its element
// types with *operation-based* concepts (`mtl::Scalar`, `mtl::Field`),
// not with `std::is_arithmetic`. Because those concepts are duck-typed
// on the arithmetic operators, the Universal number types model them
// directly: **no trait specialization or adaptor is required.** That is
// the entire "mapping" — this header just gives the branes math layer
// its own concept names that delegate to MTL5, plus a documented set of
// validated arithmetic types (exercised by the math_type_smoke test).
//
// Header-only, C++20.

#ifndef BRANES_MATH_ARITHMETIC_HPP
#define BRANES_MATH_ARITHMETIC_HPP

#include <mtl/concepts/scalar.hpp>

namespace branes::math {

/// A value type with the additive/multiplicative ring operations the
/// math layer relies on (`+`, `-`, `*`, unary `-`, and a `T{0}`
/// identity). Delegates to `mtl::Scalar`, so any type MTL5 accepts as a
/// scalar — built-in floats and the Universal number types alike —
/// satisfies it.
template <class T>
concept Scalar = mtl::Scalar<T>;

/// A `Scalar` that also supports division, i.e. forms a field. This is
/// the minimum requirement for matrix–vector products, `axpy`, and the
/// normal-equation assembly used across the solvers (`y = A*x + b`).
template <class T>
concept Field = mtl::Field<T>;

/// A `Field` that is totally ordered — required wherever the algorithms
/// compare magnitudes (trust-region radii, line-search predicates,
/// pivot selection). Real floating-point and real Universal types model
/// it; complex types deliberately do not.
template <class T>
concept OrderedField = mtl::OrderedField<T>;

/// The element-type contract for the dense/sparse linear-algebra
/// surface (MTL5 expression templates: matvec, axpy, scaling). `Field`
/// is sufficient for every expression the math layer builds, so this is
/// an intentional alias rather than a stricter concept — it exists to
/// name the requirement at solver call sites.
template <class T>
concept LinearAlgebraScalar = Field<T>;

// ── Validated arithmetic types ──────────────────────────────────────
//
// The configurations below are exercised by the math_type_smoke test
// (`ctest -R math_type_smoke`), which instantiates a real MTL5
// expression (`y = A*x + b`) for each and checks the result. Keep this
// list and the test's type matrix in sync.
//
//   IEEE binary:   double, float
//   Universal posit:   posit<32, 2>, posit<16, 1>
//   Universal cfloat:  cfloat<32, 8, uint32_t, true, false, false>
//                      (IEEE-754 single-precision-equivalent)
//   Universal fixpnt:  fixpnt<32, 16, Saturate>  (Q16.16 saturating)
//
// Types NOT yet validated for the linear-algebra surface (tracked for
// later phases): lns, dd/qd, integer/decimal.

}  // namespace branes::math

#endif  // BRANES_MATH_ARITHMETIC_HPP
