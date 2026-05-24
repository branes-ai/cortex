// SPDX-License-Identifier: MIT
//
// branes/math/krylov.hpp — type-generic Krylov linear solvers.
//
// Thin wrappers over MTL5's ITL Krylov methods (CG, BiCGSTAB, GMRES)
// that hide the iteration-controller / preconditioner boilerplate behind
// a single call returning a structured result. The math layer's job here
// is *not* to reimplement the iterations — MTL5 owns those — but to give
// the solvers a uniform, concept-constrained entry point so the rest of
// the stack (Krylov inner solves inside the NLS trust-region steps) calls
// them the same way for every scalar type.
//
// The operand and vector types are whatever MTL5 accepts (e.g.
// mtl::mat::compressed2D<T>, mtl::vec::dense_vector<T>); the only
// constraint we impose is that the element type models
// branes::math::LinearAlgebraScalar, which holds for double/float and the
// Universal number types alike.
//
// Header-only, C++20.

#ifndef BRANES_MATH_KRYLOV_HPP
#define BRANES_MATH_KRYLOV_HPP

#include <branes/math/arithmetic.hpp>

#include <mtl/itl/iteration/basic_iteration.hpp>
#include <mtl/itl/krylov/bicgstab.hpp>
#include <mtl/itl/krylov/cg.hpp>
#include <mtl/itl/krylov/gmres.hpp>
#include <mtl/itl/pc/diagonal.hpp>
#include <mtl/itl/pc/identity.hpp>
#include <mtl/vec/dense_vector.hpp>

#include <cstddef>

namespace branes::math {

/// Stopping criteria for a Krylov solve. Convergence is
/// `‖r‖ <= rel_tol·‖r₀‖` or `‖r‖ <= abs_tol`, capped at `max_iterations`.
/// `restart` is the GMRES Krylov-subspace dimension (ignored by CG /
/// BiCGSTAB).
template <Scalar Real>
struct KrylovControl {
    int max_iterations = 1000;
    Real rel_tol = Real{1} / Real{1000000000};  // 1e-9
    Real abs_tol = Real{0};
    int restart = 30;
};

/// Outcome of a Krylov solve.
struct KrylovResult {
    bool converged = false;  ///< true iff the tolerance was met
    int iterations = 0;      ///< iterations actually performed
};

namespace detail {

/// Identity preconditioner over the operator type (no-op).
template <class LinearOp>
using DefaultPc = mtl::itl::pc::identity<LinearOp>;

template <class VecX>
using value_t = typename VecX::value_type;

/// Build the iteration controller seeded with the *true* initial
/// residual r0 = b − A·x, so the relative-tolerance reference is ‖r0‖
/// (as documented) rather than ‖b‖ — these differ whenever the initial
/// guess x is nonzero (e.g. a warm start).
template <class LinearOp, class VecX, class VecB>
mtl::itl::basic_iteration<value_t<VecX>>
make_iteration(const LinearOp& A, const VecX& x, const VecB& b, const KrylovControl<value_t<VecX>>& ctrl) {
    using Real = value_t<VecX>;
    auto Ax = A * x;
    mtl::vec::dense_vector<Real> r0(b.size());
    for (std::size_t i = 0; i < static_cast<std::size_t>(b.size()); ++i) {
        r0[i] = b[i] - Ax[i];
    }
    return mtl::itl::basic_iteration<Real>(r0, ctrl.max_iterations, ctrl.rel_tol, ctrl.abs_tol);
}

}  // namespace detail

// ── Conjugate Gradient (symmetric positive definite) ────────────────

/// CG with an explicit preconditioner `M`. Solves `A x = b`.
template <class LinearOp, class VecX, class VecB, class PC>
    requires LinearAlgebraScalar<detail::value_t<VecX>>
KrylovResult
cg(const LinearOp& A, VecX& x, const VecB& b, const PC& M, const KrylovControl<detail::value_t<VecX>>& ctrl) {
    auto iter = detail::make_iteration(A, x, b, ctrl);
    const int err = mtl::itl::cg(A, x, b, M, iter);
    return KrylovResult{err == 0, iter.iterations()};
}

/// CG with the identity preconditioner.
template <class LinearOp, class VecX, class VecB>
    requires LinearAlgebraScalar<detail::value_t<VecX>>
KrylovResult cg(const LinearOp& A, VecX& x, const VecB& b, const KrylovControl<detail::value_t<VecX>>& ctrl) {
    return cg(A, x, b, detail::DefaultPc<LinearOp>(A), ctrl);
}

// ── BiCGSTAB (general non-symmetric) ────────────────────────────────

template <class LinearOp, class VecX, class VecB, class PC>
    requires LinearAlgebraScalar<detail::value_t<VecX>>
KrylovResult
bicgstab(const LinearOp& A, VecX& x, const VecB& b, const PC& M, const KrylovControl<detail::value_t<VecX>>& ctrl) {
    auto iter = detail::make_iteration(A, x, b, ctrl);
    const int err = mtl::itl::bicgstab(A, x, b, M, iter);
    return KrylovResult{err == 0, iter.iterations()};
}

template <class LinearOp, class VecX, class VecB>
    requires LinearAlgebraScalar<detail::value_t<VecX>>
KrylovResult bicgstab(const LinearOp& A, VecX& x, const VecB& b, const KrylovControl<detail::value_t<VecX>>& ctrl) {
    return bicgstab(A, x, b, detail::DefaultPc<LinearOp>(A), ctrl);
}

// ── GMRES (general non-symmetric, restarted) ────────────────────────

template <class LinearOp, class VecX, class VecB, class PC>
    requires LinearAlgebraScalar<detail::value_t<VecX>>
KrylovResult
gmres(const LinearOp& A, VecX& x, const VecB& b, const PC& M, const KrylovControl<detail::value_t<VecX>>& ctrl) {
    auto iter = detail::make_iteration(A, x, b, ctrl);
    const int err = mtl::itl::gmres(A, x, b, M, iter, ctrl.restart);
    return KrylovResult{err == 0, iter.iterations()};
}

template <class LinearOp, class VecX, class VecB>
    requires LinearAlgebraScalar<detail::value_t<VecX>>
KrylovResult gmres(const LinearOp& A, VecX& x, const VecB& b, const KrylovControl<detail::value_t<VecX>>& ctrl) {
    return gmres(A, x, b, detail::DefaultPc<LinearOp>(A), ctrl);
}

/// Convenience alias for the diagonal (Jacobi) preconditioner, which is
/// a cheap and effective default for the SPD normal equations the NLS
/// solvers form.
template <class Matrix>
using DiagonalPreconditioner = mtl::itl::pc::diagonal<Matrix>;

}  // namespace branes::math

#endif  // BRANES_MATH_KRYLOV_HPP
