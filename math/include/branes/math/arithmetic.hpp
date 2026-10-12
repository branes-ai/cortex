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

#include <limits>

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

/// A TOLERANCE tuned in double, floored at a few roundoff units of T at the
/// magnitude `scale` it is compared with or added to (#449):
/// max(nominal, 16·ε_T·|scale|).
///
/// NUMERICAL NOTE. Use this only for a tolerance, a value that decides how much
/// rounding to forgive. Examples are the PSD test's slack, or a degeneracy test
/// on a sine. A double-tuned tolerance such as 1e-9 is below float's or a
/// posit's rounding at magnitude 1, so the test would reject what is only
/// rounding. Flooring it at a few ulps of the value it is compared with fixes
/// that.
///
/// Do NOT use it for a REGULARIZER, a ridge λ added to a normal matrix to make
/// it factorable. A ridge shrinks the solution along each eigenvector of H by
/// λ/(σᵢ + λ), so it must stay far below the smallest eigenvalue that carries
/// information. That is a property of the problem, not of T. In the dynamic
/// initializer, the scale-carrying mode has σ ≈ 5.5e-6. A ridge floored at
/// 16·ε_float = 1.9e-6 shrank it by 26 %, a 17 % scale error (the S1 SfM bench,
/// float). The fixed 1e-9 shrinks it by 0.02 %. On the zero diagonals the ridge
/// exists for, 1e-9 is not absorbed: it is nonzero in every bench type (posit16
/// rounds it up to its minpos, 3.7e-9). It is NOT nonzero in IEEE half or Q16.16
/// fixed point, where it rounds to 0, and posit8 rounds it up to 0.016. A port to
/// such a format must rescale the problem, not floor the ridge. See docs-site
/// …/debugging/numerical-thresholds.mdx.
///
/// For double, the floor exceeds 1e-9 only when |scale| > ~2.8e5, so double
/// results are unchanged.
template <OrderedField T>
[[nodiscard]] T at_least_roundoff(const T& nominal, const T& scale = T{1}) {
    const T mag = scale < T{0} ? -scale : scale;
    const T floor = T{16} * std::numeric_limits<T>::epsilon() * mag;
    return nominal < floor ? floor : nominal;
}

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
