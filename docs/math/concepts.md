# Math layer concepts reference

The `branes::math` headers are concept-constrained C++20. The concepts
delegate the actual arithmetic requirements to MTL5's operation-based
scalar concepts, so the built-in floating-point types **and** the
Universal number types (posit, cfloat, fixpnt, …) satisfy them without
any per-type specialization. This page lists every public concept and
its one-line semantics.

## Scalar / arithmetic concepts

Defined in [`branes/math/arithmetic.hpp`](../../math/include/branes/math/arithmetic.hpp).

| Concept | Header | Semantics |
|---|---|---|
| `branes::math::Scalar<T>` | `arithmetic.hpp` | A value type with the ring operations `+ - *`, unary `-`, and a `T{0}` identity. Delegates to `mtl::Scalar`. |
| `branes::math::Field<T>` | `arithmetic.hpp` | A `Scalar` that also supports division — the minimum for matrix–vector products, `axpy`, and normal-equation assembly. Delegates to `mtl::Field`. |
| `branes::math::OrderedField<T>` | `arithmetic.hpp` | A `Field` that is totally ordered — required wherever the algorithms compare magnitudes (trust-region radii, line-search predicates, pivots). Real floats and real Universal types model it; complex types do not. |
| `branes::math::LinearAlgebraScalar<T>` | `arithmetic.hpp` | The element-type contract for the dense/sparse linear-algebra surface (MTL5 matvec / axpy / scaling). An intentional alias of `Field`; it names the requirement at solver call sites. |
| `branes::math::TensorScalar<T>` | `tensor.hpp` | An element type a `TensorView` may store and relocate: `std::is_trivially_copyable_v<std::remove_const_t<T>>`. Storage only — no arithmetic requirement (that is layered on by the solvers). |

### Validated arithmetic types

The `math_type_smoke` test instantiates a real MTL5 expression
(`y = A·x + b`) for each of the following and checks the result:

- IEEE binary: `double`, `float`
- Universal posit: `posit<32, 2>`, `posit<16, 1>`
- Universal cfloat: `cfloat<32, 8, uint32_t, true, false, false>`
- Universal fixpnt: `fixpnt<32, 16, Saturate>`

Types not yet validated for the linear-algebra surface (tracked for
later phases): `lns`, `dd`/`qd`, `integer`/`decimal`.

## View types

Defined in [`branes/math/tensor.hpp`](../../math/include/branes/math/tensor.hpp)
and [`branes/math/sparse.hpp`](../../math/include/branes/math/sparse.hpp).
These are non-owning windows over caller-supplied buffers (the math layer
never owns numeric storage — it operates on buffers handed across the cxx
bridge from the Rust Resource Manager).

| Type | Header | Role |
|---|---|---|
| `TensorView<T, Rank>` | `tensor.hpp` | Dense multi-dimensional view over `std::span<T>` with a DLPack-style shape/stride descriptor (strides in elements). `T` models `TensorScalar`. |
| `VectorView<T>` / `MatrixView<T>` | `tensor.hpp` | Rank-1 / rank-2 aliases of `TensorView`. |
| `CsrView<T, IndexT>` | `sparse.hpp` | Compressed-sparse-row view; `T` models `Scalar`. |
| `CscView<T, IndexT>` | `sparse.hpp` | Compressed-sparse-column view. |
| `CooView<T, IndexT>` | `sparse.hpp` | Coordinate-list view (duplicate coordinates accumulate). |

## Solver surface

| Component | Header | Element-type constraint |
|---|---|---|
| `cg` / `bicgstab` / `gmres` | `krylov.hpp` | `LinearAlgebraScalar` (thin wrappers on MTL5 ITL) |
| `SparseCholesky<T>` / `SparseLdlt<T>` | `sparse_direct.hpp` | `Scalar` (MTL5 native sparse direct) |
| `nls::solve` (Gauss-Newton / Levenberg-Marquardt / Dogleg) | `nls/solver.hpp` | model's `Scalar`; analytical Jacobians only |
| `lie::SO3<T>` / `SE3<T>` / `Sim3<T>` | `lie/*.hpp` | `Scalar` (with ADL `sqrt`/`sin`/`cos`/… for transcendentals) |

## Building the API docs

```bash
cmake -B build
cmake --build build --target docs-math   # requires Doxygen
# HTML output under build/docs/math/html
```

The `docs-math` target is configured with `WARN_AS_ERROR`, so the build
fails if any Doxygen comment is malformed — the headers are kept
warning-clean.
