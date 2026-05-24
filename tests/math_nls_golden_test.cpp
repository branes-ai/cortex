// Golden-data NLS suite (issue #31).
//
// Three classic least-squares test problems with known minima —
// Rosenbrock, Powell's singular function, and Beale — exercised three
// ways:
//   1. Finite-difference checks of each model's analytical Jacobian
//      (central differences vs. the analytical J).
//   2. Convergence to the published minimum with LM and Dogleg.
//   3. A type-genericity matrix: each problem solved in double, float,
//      and a Universal posit, with per-type tolerances.
//
// Models are templated on the scalar so the same residual/Jacobian code
// instantiates for every arithmetic type.

#include <branes/math/nls.hpp>

#include <catch2/catch_test_macros.hpp>

// clang-format off
#include <universal/number/posit/posit.hpp>
#include <universal/number/posit/mathlib.hpp>
// clang-format on

#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

namespace {

namespace nls = branes::math::nls;

// ── Test problems (templated on scalar) ─────────────────────────────

// Rosenbrock: r0 = 10(x1 - x0²), r1 = (1 - x0). Min (1,1), cost 0.
template <class T>
struct Rosenbrock {
    using Scalar = T;
    static constexpr std::size_t kP = 2, kR = 2;
    std::size_t num_parameters() const {
        return kP;
    }
    std::size_t num_residuals() const {
        return kR;
    }
    void evaluate(std::span<const T> x, std::span<T> r, std::span<T> J) const {
        r[0] = T{10} * (x[1] - x[0] * x[0]);
        r[1] = T{1} - x[0];
        J[0] = T{-20} * x[0];
        J[1] = T{10};
        J[2] = T{-1};
        J[3] = T{0};
    }
};

// Powell's singular function (n=m=4). Min at the origin, cost 0; the
// Hessian is singular at the solution so convergence is only linear.
template <class T>
struct Powell {
    using Scalar = T;
    static constexpr std::size_t kP = 4, kR = 4;
    std::size_t num_parameters() const {
        return kP;
    }
    std::size_t num_residuals() const {
        return kR;
    }
    void evaluate(std::span<const T> x, std::span<T> r, std::span<T> J) const {
        const T s5 = T{2236067977} / T{1000000000};   // sqrt(5)
        const T s10 = T{3162277660} / T{1000000000};  // sqrt(10)
        r[0] = x[0] + T{10} * x[1];
        r[1] = s5 * (x[2] - x[3]);
        r[2] = (x[1] - T{2} * x[2]) * (x[1] - T{2} * x[2]);
        r[3] = s10 * (x[0] - x[3]) * (x[0] - x[3]);
        for (std::size_t k = 0; k < 16; ++k)
            J[k] = T{0};
        // r0
        J[0] = T{1};
        J[1] = T{10};
        // r1
        J[1 * 4 + 2] = s5;
        J[1 * 4 + 3] = -s5;
        // r2 = (x1 - 2 x2)²
        const T a = x[1] - T{2} * x[2];
        J[2 * 4 + 1] = T{2} * a;
        J[2 * 4 + 2] = T{-4} * a;
        // r3 = s10 (x0 - x3)²
        const T b = x[0] - x[3];
        J[3 * 4 + 0] = T{2} * s10 * b;
        J[3 * 4 + 3] = T{-2} * s10 * b;
    }
};

// Beale: r_i = y_i - x0 (1 - x1^i), i = 1,2,3; y = (1.5, 2.25, 2.625).
// Min at (3, 0.5), cost 0.
template <class T>
struct Beale {
    using Scalar = T;
    static constexpr std::size_t kP = 2, kR = 3;
    std::size_t num_parameters() const {
        return kP;
    }
    std::size_t num_residuals() const {
        return kR;
    }
    void evaluate(std::span<const T> x, std::span<T> r, std::span<T> J) const {
        const T y[3] = {T{15} / T{10}, T{225} / T{100}, T{2625} / T{1000}};
        const T x0 = x[0], x1 = x[1];
        const T p1 = x1, p2 = x1 * x1, p3 = x1 * x1 * x1;
        r[0] = y[0] - x0 * (T{1} - p1);
        r[1] = y[1] - x0 * (T{1} - p2);
        r[2] = y[2] - x0 * (T{1} - p3);
        J[0 * 2 + 0] = -(T{1} - p1);
        J[0 * 2 + 1] = x0 * T{1};
        J[1 * 2 + 0] = -(T{1} - p2);
        J[1 * 2 + 1] = x0 * T{2} * x1;
        J[2 * 2 + 0] = -(T{1} - p3);
        J[2 * 2 + 1] = x0 * T{3} * x1 * x1;
    }
};

// ── Finite-difference Jacobian check (double) ───────────────────────

template <class Model>
void check_jacobian_fd(const Model& model, std::vector<double> x) {
    const std::size_t n = model.num_parameters();
    const std::size_t m = model.num_residuals();
    std::vector<double> r(m), J(m * n), rp(m), rm(m), Jp(m * n), Jm(m * n);
    model.evaluate(std::span<const double>{x}, std::span<double>{r}, std::span<double>{J});
    const double h = 1e-6;
    for (std::size_t j = 0; j < n; ++j) {
        auto xp = x, xn = x;
        xp[j] += h;
        xn[j] -= h;
        model.evaluate(std::span<const double>{xp}, std::span<double>{rp}, std::span<double>{Jp});
        model.evaluate(std::span<const double>{xn}, std::span<double>{rm}, std::span<double>{Jm});
        for (std::size_t i = 0; i < m; ++i) {
            const double fd = (rp[i] - rm[i]) / (2 * h);
            REQUIRE(std::abs(J[i * n + j] - fd) < 1e-4);
        }
    }
}

}  // namespace

TEST_CASE("analytical Jacobians match finite differences", "[math][nls][golden]") {
    check_jacobian_fd(Rosenbrock<double>{}, {-1.2, 1.0});
    check_jacobian_fd(Rosenbrock<double>{}, {0.5, -0.3});
    check_jacobian_fd(Powell<double>{}, {3.0, -1.0, 0.0, 1.0});
    check_jacobian_fd(Powell<double>{}, {1.0, 2.0, -1.0, 0.5});
    check_jacobian_fd(Beale<double>{}, {1.0, 1.0});
    check_jacobian_fd(Beale<double>{}, {2.5, 0.4});
}

TEST_CASE("LM and Dogleg reach the published minima (double)", "[math][nls][golden]") {
    nls::Options<double> opts;
    opts.max_iterations = 500;

    for (auto method : {nls::Method::LevenbergMarquardt, nls::Method::Dogleg}) {
        {
            double x[2] = {-1.2, 1.0};
            auto s = nls::solve(Rosenbrock<double>{}, std::span<double>{x}, opts, method);
            REQUIRE(s.final_cost < 1e-12);
            REQUIRE(std::abs(x[0] - 1.0) < 1e-5);
            REQUIRE(std::abs(x[1] - 1.0) < 1e-5);
        }
        {
            double x[2] = {1.0, 1.0};
            auto s = nls::solve(Beale<double>{}, std::span<double>{x}, opts, method);
            REQUIRE(s.final_cost < 1e-12);
            REQUIRE(std::abs(x[0] - 3.0) < 1e-4);
            REQUIRE(std::abs(x[1] - 0.5) < 1e-4);
        }
        {
            // Powell is singular at the min: check the cost drives to ~0.
            double x[4] = {3.0, -1.0, 0.0, 1.0};
            auto s = nls::solve(Powell<double>{}, std::span<double>{x}, opts, method);
            REQUIRE(s.final_cost < 1e-10);
            for (double v : x)
                REQUIRE(std::abs(v) < 1e-2);
        }
    }
}

TEST_CASE("LM solves the suite across arithmetic types", "[math][nls][golden]") {
    using posit32 = sw::universal::posit<32, 2>;

    SECTION("float") {
        nls::Options<float> opts;
        opts.max_iterations = 500;
        opts.gradient_tolerance = 1e-7f;
        float x[2] = {-1.2f, 1.0f};
        auto s = nls::solve(Rosenbrock<float>{}, std::span<float>{x}, opts, nls::Method::LevenbergMarquardt);
        REQUIRE(double(s.final_cost) < 1e-6);
        REQUIRE(std::abs(double(x[0]) - 1.0) < 1e-2);
        REQUIRE(std::abs(double(x[1]) - 1.0) < 1e-2);

        float xb[2] = {1.0f, 1.0f};
        nls::solve(Beale<float>{}, std::span<float>{xb}, opts, nls::Method::LevenbergMarquardt);
        REQUIRE(std::abs(double(xb[0]) - 3.0) < 1e-1);
        REQUIRE(std::abs(double(xb[1]) - 0.5) < 1e-1);
    }

    SECTION("posit<32,2>") {
        nls::Options<posit32> opts;
        opts.max_iterations = 500;
        opts.gradient_tolerance = posit32(1e-8);
        posit32 x[2] = {posit32(-1.2), posit32(1.0)};
        auto s = nls::solve(Rosenbrock<posit32>{}, std::span<posit32>{x}, opts, nls::Method::LevenbergMarquardt);
        REQUIRE(double(s.final_cost) < 1e-6);
        REQUIRE(std::abs(double(x[0]) - 1.0) < 1e-2);
        REQUIRE(std::abs(double(x[1]) - 1.0) < 1e-2);

        posit32 xb[2] = {posit32(1.0), posit32(1.0)};
        nls::solve(Beale<posit32>{}, std::span<posit32>{xb}, opts, nls::Method::LevenbergMarquardt);
        REQUIRE(std::abs(double(xb[0]) - 3.0) < 1e-1);
        REQUIRE(std::abs(double(xb[1]) - 0.5) < 1e-1);
    }
}
