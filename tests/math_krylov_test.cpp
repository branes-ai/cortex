// Krylov solver wrapper tests (issue #27).
//
// Exercises the branes::math CG / BiCGSTAB / GMRES wrappers over MTL5
// ITL: CG on an SPD tridiagonal system, BiCGSTAB and GMRES on a
// non-symmetric diagonally-dominant system, the diagonal preconditioner
// path, and a float instantiation for type-genericity. Each solve
// reconstructs a known solution x_true from b = A x_true.

#include <branes/math/krylov.hpp>

#include <catch2/catch_test_macros.hpp>
#include <mtl/mtl.hpp>

#include <cmath>
#include <cstddef>

namespace {

// SPD tridiagonal: 2 on the diagonal, -1 on the off-diagonals.
template <class T>
mtl::mat::dense2D<T> spd_tridiag(std::size_t n) {
    mtl::mat::dense2D<T> A(n, n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            A(i, j) = T{0};
    for (std::size_t i = 0; i < n; ++i) {
        A(i, i) = T{2};
        if (i + 1 < n) {
            A(i, i + 1) = T{-1};
            A(i + 1, i) = T{-1};
        }
    }
    return A;
}

// Non-symmetric, diagonally dominant: diag 4, super -1, sub -2.
template <class T>
mtl::mat::dense2D<T> nonsym(std::size_t n) {
    mtl::mat::dense2D<T> A(n, n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            A(i, j) = T{0};
    for (std::size_t i = 0; i < n; ++i) {
        A(i, i) = T{4};
        if (i + 1 < n)
            A(i, i + 1) = T{-1};
        if (i > 0)
            A(i, i - 1) = T{-2};
    }
    return A;
}

template <class T, class Mat>
mtl::vec::dense_vector<T> rhs_from(const Mat& A, const mtl::vec::dense_vector<T>& x) {
    const std::size_t n = x.size();
    mtl::vec::dense_vector<T> b(n, T{0});
    auto Ax = A * x;
    for (std::size_t i = 0; i < n; ++i)
        b(i) = Ax(i);
    return b;
}

template <class T>
double max_abs_diff(const mtl::vec::dense_vector<T>& a, const mtl::vec::dense_vector<T>& b) {
    double m = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        m = std::max(m, std::abs(double(a(i)) - double(b(i))));
    return m;
}

}  // namespace

TEST_CASE("CG solves an SPD system", "[math][krylov]") {
    constexpr std::size_t n = 24;
    auto A = spd_tridiag<double>(n);
    mtl::vec::dense_vector<double> x_true(n, 0.0);
    for (std::size_t i = 0; i < n; ++i)
        x_true(i) = 1.0 + 0.1 * double(i);
    auto b = rhs_from(A, x_true);

    mtl::vec::dense_vector<double> x(n, 0.0);
    branes::math::KrylovControl<double> ctrl{};
    auto r = branes::math::cg(A, x, b, ctrl);

    REQUIRE(r.converged);
    REQUIRE(r.iterations <= static_cast<int>(n) + 1);
    REQUIRE(max_abs_diff(x, x_true) < 1e-7);
}

TEST_CASE("CG with diagonal preconditioner converges", "[math][krylov]") {
    constexpr std::size_t n = 24;
    auto A = spd_tridiag<double>(n);
    mtl::vec::dense_vector<double> x_true(n, 1.0);
    auto b = rhs_from(A, x_true);

    mtl::vec::dense_vector<double> x(n, 0.0);
    branes::math::DiagonalPreconditioner<decltype(A)> M(A);
    branes::math::KrylovControl<double> ctrl{};
    auto r = branes::math::cg(A, x, b, M, ctrl);

    REQUIRE(r.converged);
    REQUIRE(max_abs_diff(x, x_true) < 1e-7);
}

TEST_CASE("BiCGSTAB solves a non-symmetric system", "[math][krylov]") {
    constexpr std::size_t n = 20;
    auto A = nonsym<double>(n);
    mtl::vec::dense_vector<double> x_true(n, 0.0);
    for (std::size_t i = 0; i < n; ++i)
        x_true(i) = -0.5 + 0.2 * double(i);
    auto b = rhs_from(A, x_true);

    mtl::vec::dense_vector<double> x(n, 0.0);
    branes::math::KrylovControl<double> ctrl{};
    auto r = branes::math::bicgstab(A, x, b, ctrl);

    REQUIRE(r.converged);
    REQUIRE(max_abs_diff(x, x_true) < 1e-6);
}

TEST_CASE("GMRES solves a non-symmetric system", "[math][krylov]") {
    constexpr std::size_t n = 20;
    auto A = nonsym<double>(n);
    mtl::vec::dense_vector<double> x_true(n, 1.0);
    auto b = rhs_from(A, x_true);

    mtl::vec::dense_vector<double> x(n, 0.0);
    branes::math::KrylovControl<double> ctrl{};
    ctrl.restart = 20;
    auto r = branes::math::gmres(A, x, b, ctrl);

    REQUIRE(r.converged);
    REQUIRE(max_abs_diff(x, x_true) < 1e-6);
}

TEST_CASE("CG reports non-convergence within the iteration cap", "[math][krylov]") {
    constexpr std::size_t n = 24;
    auto A = spd_tridiag<double>(n);
    mtl::vec::dense_vector<double> x_true(n, 1.0);
    auto b = rhs_from(A, x_true);

    mtl::vec::dense_vector<double> x(n, 0.0);
    branes::math::KrylovControl<double> ctrl{};
    ctrl.max_iterations = 2;  // far too few for n=24
    auto r = branes::math::cg(A, x, b, ctrl);
    REQUIRE_FALSE(r.converged);
    REQUIRE(r.iterations <= 3);
}

TEST_CASE("CG instantiates and converges for float", "[math][krylov]") {
    constexpr std::size_t n = 12;
    auto A = spd_tridiag<float>(n);
    mtl::vec::dense_vector<float> x_true(n, 1.0f);
    auto b = rhs_from(A, x_true);

    mtl::vec::dense_vector<float> x(n, 0.0f);
    branes::math::KrylovControl<float> ctrl{};
    ctrl.rel_tol = 1e-5f;
    auto r = branes::math::cg(A, x, b, ctrl);

    REQUIRE(r.converged);
    REQUIRE(max_abs_diff(x, x_true) < 1e-3);
}
