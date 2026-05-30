// NLS solver smoke tests (issue #30).
//
// Validates the Gauss-Newton / Levenberg-Marquardt / Dogleg solvers on
// small problems with known minima: a linear least-squares system (GN
// converges in one step) and the Rosenbrock function as a 2-residual NLS
// (the classic non-linear test, solved by LM and Dogleg). The full
// golden-data suite (Powell, Beale, FD Jacobian checks, type matrix)
// lands in #31.

#include <branes/math/nls.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <span>

namespace {

namespace nls = branes::math::nls;

// Rosenbrock as least squares: r1 = 10(x1 - x0²), r2 = (1 - x0).
// Minimum at (1, 1) with zero cost.
struct Rosenbrock {
    using Scalar = double;
    std::size_t num_parameters() const {
        return 2;
    }
    std::size_t num_residuals() const {
        return 2;
    }
    void evaluate(std::span<const double> x, std::span<double> r, std::span<double> J) const {
        r[0] = 10.0 * (x[1] - x[0] * x[0]);
        r[1] = 1.0 - x[0];
        // J row-major 2x2: d r_i / d x_j
        J[0] = -20.0 * x[0];  // dr0/dx0
        J[1] = 10.0;          // dr0/dx1
        J[2] = -1.0;          // dr1/dx0
        J[3] = 0.0;           // dr1/dx1
    }
};

// Linear least squares: r = A x - b, A = [[1,1],[1,2],[1,3]], b chosen so
// the exact solution is x = (1, 2). Overdetermined (m=3, n=2).
struct LinearLS {
    using Scalar = double;
    std::size_t num_parameters() const {
        return 2;
    }
    std::size_t num_residuals() const {
        return 3;
    }
    void evaluate(std::span<const double> x, std::span<double> r, std::span<double> J) const {
        const double A[3][2] = {{1, 1}, {1, 2}, {1, 3}};
        const double b[3] = {3.0, 5.0, 7.0};  // A*(1,2) = (3,5,7)
        for (std::size_t i = 0; i < 3; ++i) {
            r[i] = A[i][0] * x[0] + A[i][1] * x[1] - b[i];
            J[i * 2 + 0] = A[i][0];
            J[i * 2 + 1] = A[i][1];
        }
    }
};

// Rank-deficient (underdetermined) least squares: a single residual in
// two parameters, r = x0 + x1 - 2. JᵀJ is singular, so the Gauss-Newton
// step is unavailable and Dogleg must fall back to the Cauchy direction.
struct RankDeficient {
    using Scalar = double;
    std::size_t num_parameters() const {
        return 2;
    }
    std::size_t num_residuals() const {
        return 1;
    }
    void evaluate(std::span<const double> x, std::span<double> r, std::span<double> J) const {
        r[0] = x[0] + x[1] - 2.0;
        J[0] = 1.0;
        J[1] = 1.0;
    }
};

// Pathological model: residual is constant (cost can't change) but the
// Jacobian is nonzero, so the gradient never vanishes and no step ever
// reduces the cost. Used to drive the solver into its no-progress path.
struct Stuck {
    using Scalar = double;
    std::size_t num_parameters() const {
        return 1;
    }
    std::size_t num_residuals() const {
        return 1;
    }
    void evaluate(std::span<const double>, std::span<double> r, std::span<double> J) const {
        r[0] = 1.0;
        J[0] = 1.0;
    }
};

}  // namespace

TEST_CASE("Gauss-Newton solves a linear least-squares system", "[math][nls]") {
    LinearLS model;
    double x[2] = {0.0, 0.0};
    auto s = nls::solve(model, std::span<double>{x}, nls::Options<double>{}, nls::Method::GaussNewton);
    REQUIRE(s.status == nls::Status::Converged);
    REQUIRE(std::abs(x[0] - 1.0) < 1e-9);
    REQUIRE(std::abs(x[1] - 2.0) < 1e-9);
    REQUIRE(s.final_cost < 1e-18);
}

TEST_CASE("Levenberg-Marquardt solves Rosenbrock", "[math][nls]") {
    Rosenbrock model;
    double x[2] = {-1.2, 1.0};  // classic hard start
    nls::Options<double> opts;
    opts.max_iterations = 200;
    auto s = nls::solve(model, std::span<double>{x}, opts, nls::Method::LevenbergMarquardt);
    REQUIRE(s.status == nls::Status::Converged);
    REQUIRE(std::abs(x[0] - 1.0) < 1e-6);
    REQUIRE(std::abs(x[1] - 1.0) < 1e-6);
    REQUIRE(s.final_cost < 1e-12);
}

TEST_CASE("Dogleg solves Rosenbrock", "[math][nls]") {
    Rosenbrock model;
    double x[2] = {-1.2, 1.0};
    nls::Options<double> opts;
    opts.max_iterations = 200;
    auto s = nls::solve(model, std::span<double>{x}, opts, nls::Method::Dogleg);
    REQUIRE(s.status == nls::Status::Converged);
    REQUIRE(std::abs(x[0] - 1.0) < 1e-6);
    REQUIRE(std::abs(x[1] - 1.0) < 1e-6);
    REQUIRE(s.final_cost < 1e-12);
}

TEST_CASE("solver reports the initial cost and reduces it", "[math][nls]") {
    Rosenbrock model;
    double x[2] = {-1.2, 1.0};
    auto s = nls::solve(model, std::span<double>{x}, nls::Options<double>{}, nls::Method::LevenbergMarquardt);
    REQUIRE(s.initial_cost > s.final_cost);
    REQUIRE(s.iterations > 0);
}

TEST_CASE("a too-small iteration cap reports MaxIterations, not Converged", "[math][nls]") {
    Rosenbrock model;
    double x[2] = {-1.2, 1.0};
    nls::Options<double> opts;
    opts.max_iterations = 2;  // nowhere near enough for the hard start
    auto s = nls::solve(model, std::span<double>{x}, opts, nls::Method::LevenbergMarquardt);
    REQUIRE(s.status == nls::Status::MaxIterations);
}

TEST_CASE("a stalled solve reports NoProgress, not Converged", "[math][nls]") {
    // Regression: when no cost-reducing step exists but the gradient is
    // not yet small, the solver must not claim Converged.
    Stuck model;
    double x[1] = {0.0};
    auto s = nls::solve(model, std::span<double>{x}, nls::Options<double>{}, nls::Method::LevenbergMarquardt);
    REQUIRE(s.status == nls::Status::NoProgress);
}

TEST_CASE("Dogleg handles a rank-deficient system without failing", "[math][nls]") {
    // JᵀJ is singular here, so the Gauss-Newton step is unavailable and
    // Dogleg must fall back to the Cauchy direction (regression for the
    // premature gᵀHg<=0 / no-GN-step failure paths).
    RankDeficient model;
    double x[2] = {0.0, 0.0};
    auto s = nls::solve(model, std::span<double>{x}, nls::Options<double>{}, nls::Method::Dogleg);
    REQUIRE(s.status != nls::Status::Failed);
    REQUIRE(s.final_cost < 1e-10);  // drives r = x0+x1-2 to ~0
    REQUIRE(std::abs(x[0] + x[1] - 2.0) < 1e-5);
}
