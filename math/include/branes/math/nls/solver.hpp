// SPDX-License-Identifier: MIT
//
// branes/math/nls/solver.hpp — non-linear least-squares solvers.
//
// Minimizes 0.5·‖r(x)‖² for a user-supplied model that provides residuals
// r(x) ∈ Rᵐ and an *analytical* Jacobian J(x) ∈ Rᵐˣⁿ (no automatic
// differentiation in this phase). Three step strategies share one driver
// and the same convergence tests:
//
//   • GaussNewton          — full Newton step on the Gauss–Newton Hessian
//                            JᵀJ, with backtracking line search.
//   • LevenbergMarquardt   — damped (JᵀJ + λ·diag(JᵀJ)) step with Nielsen
//                            λ adaptation by gain ratio.
//   • Dogleg               — trust-region interpolation between the
//                            Gauss–Newton and Cauchy (steepest-descent)
//                            steps.
//
// Clean-room from textbook descriptions (Nocedal & Wright, "Numerical
// Optimization"; Madsen, Nielsen & Tingleff, "Methods for Non-Linear
// Least Squares Problems"). Type-generic over the scalar.
//
// Model interface (duck-typed):
//   using Scalar = T;
//   std::size_t num_parameters() const;
//   std::size_t num_residuals()  const;
//   void evaluate(std::span<const T> x, std::span<T> residual,
//                 std::span<T> jacobian) const;   // jacobian row-major m×n
//
// Header-only, C++20.

#ifndef BRANES_MATH_NLS_SOLVER_HPP
#define BRANES_MATH_NLS_SOLVER_HPP

#include <branes/math/nls/dense_linalg.hpp>

#include <cstddef>
#include <span>
#include <vector>

namespace branes::math::nls {

/// Step strategy.
enum class Method { GaussNewton, LevenbergMarquardt, Dogleg };

/// Why the solve stopped.
enum class Status {
    Converged,      ///< a tolerance test was satisfied
    MaxIterations,  ///< hit the iteration cap
    Failed,         ///< the step could not be computed (e.g. singular system)
};

/// Stopping tolerances and trust-region/damping seeds.
template <Scalar T>
struct Options {
    int max_iterations = 100;
    T gradient_tolerance = T{1} / T{10000000000};     ///< ‖Jᵀr‖_∞  (1e-10)
    T function_tolerance = T{1} / T{1000000000000};   ///< rel. cost drop (1e-12)
    T parameter_tolerance = T{1} / T{1000000000000};  ///< rel. step size (1e-12)
    T initial_trust_radius = T{1};                    ///< Dogleg
    T initial_lambda = T{1} / T{1000};                ///< LM (1e-3)
};

/// Outcome of a solve.
template <Scalar T>
struct Summary {
    Status status = Status::MaxIterations;
    int iterations = 0;
    T initial_cost{0};
    T final_cost{0};
    T gradient_norm{0};
};

namespace detail {

template <Scalar T>
T inf_norm(std::span<const T> v) {
    T m{0};
    for (const T& e : v) {
        const T a = abs_(e);
        if (a > m)
            m = a;
    }
    return m;
}

/// 0.5·‖r + J·d‖² — the local quadratic model's cost at step d.
template <Scalar T>
T model_cost(std::span<const T> r, std::span<const T> J, std::span<const T> d, std::size_t m, std::size_t n) {
    T c{0};
    for (std::size_t k = 0; k < m; ++k) {
        T rk = r[k];
        const std::size_t row = k * n;
        for (std::size_t i = 0; i < n; ++i)
            rk = rk + J[row + i] * d[i];
        c = c + rk * rk;
    }
    return c * T{1} / T{2};
}

}  // namespace detail

/// Solve min 0.5‖r(x)‖². `x` is the in/out parameter vector (length
/// num_parameters()); on return it holds the optimized estimate.
template <class Model>
Summary<typename Model::Scalar> solve(const Model& model,
                                      std::span<typename Model::Scalar> x,
                                      const Options<typename Model::Scalar>& opts = {},
                                      Method method = Method::LevenbergMarquardt) {
    using T = typename Model::Scalar;
    namespace d = detail;
    using branes::math::lie::detail::sqrt_;
    using branes::math::nls::detail::cholesky_solve;
    using branes::math::nls::detail::dot;
    using branes::math::nls::detail::matvec;
    using branes::math::nls::detail::norm;
    using branes::math::nls::detail::normal_equations;

    const std::size_t n = model.num_parameters();
    const std::size_t m = model.num_residuals();

    std::vector<T> r(m), J(m * n), r_trial(m), J_trial(m * n);
    std::vector<T> JtJ, g, step(n), x_trial(n), Hg;

    auto eval = [&](std::span<const T> xv, std::vector<T>& rv, std::vector<T>& Jv) {
        model.evaluate(xv, std::span<T>{rv}, std::span<T>{Jv});
    };
    auto cost_of = [&](std::span<const T> rv) { return dot<T>(rv, rv) * T{1} / T{2}; };

    eval(x, r, J);
    T cost = cost_of(r);

    Summary<T> sum;
    sum.initial_cost = cost;

    T lambda = opts.initial_lambda;
    T nu = T{2};                           // LM rejection growth factor
    T radius = opts.initial_trust_radius;  // Dogleg

    int iter = 0;
    for (; iter < opts.max_iterations; ++iter) {
        normal_equations<T>(J, r, m, n, JtJ, g);  // g = Jᵀr
        const T gnorm = d::inf_norm<T>(g);
        sum.gradient_norm = gnorm;
        if (gnorm <= opts.gradient_tolerance) {
            sum.status = Status::Converged;
            break;
        }

        bool accepted = false;
        bool failed = false;

        // Negated gradient appears throughout (we solve for x += step).
        std::vector<T> neg_g(n);
        for (std::size_t i = 0; i < n; ++i)
            neg_g[i] = -g[i];

        if (method == Method::GaussNewton) {
            // Solve JᵀJ step = −g (ridge fallback if not PD).
            std::vector<T> A = JtJ;
            if (!cholesky_solve<T>(A, n, std::span<const T>{neg_g}, step)) {
                for (std::size_t i = 0; i < n; ++i)
                    JtJ[i * n + i] = JtJ[i * n + i] + T{1} / T{1000000};
                A = JtJ;
                if (!cholesky_solve<T>(A, n, std::span<const T>{neg_g}, step)) {
                    failed = true;
                }
            }
            if (!failed) {
                // Backtracking line search for descent.
                T alpha{1};
                for (int ls = 0; ls < 30; ++ls) {
                    for (std::size_t i = 0; i < n; ++i)
                        x_trial[i] = x[i] + step[i] * alpha;
                    eval(std::span<const T>{x_trial}, r_trial, J_trial);
                    if (cost_of(r_trial) < cost) {
                        accepted = true;
                        break;
                    }
                    alpha = alpha * T{1} / T{2};
                }
                if (accepted)
                    for (std::size_t i = 0; i < n; ++i)
                        step[i] = step[i] * alpha;
            }
        } else if (method == Method::LevenbergMarquardt) {
            for (int tries = 0; tries < 50 && !accepted && !failed; ++tries) {
                std::vector<T> A = JtJ;
                for (std::size_t i = 0; i < n; ++i) {
                    const T dii = JtJ[i * n + i];
                    A[i * n + i] = dii + lambda * (dii > T{0} ? dii : T{1});
                }
                if (!cholesky_solve<T>(A, n, std::span<const T>{neg_g}, step)) {
                    lambda = lambda * nu;
                    nu = nu * T{2};
                    continue;
                }
                for (std::size_t i = 0; i < n; ++i)
                    x_trial[i] = x[i] + step[i];
                eval(std::span<const T>{x_trial}, r_trial, J_trial);
                const T cost_trial = cost_of(r_trial);
                const T pred = cost - d::model_cost<T>(r, J, std::span<const T>{step}, m, n);
                if (pred > T{0}) {
                    const T rho = (cost - cost_trial) / pred;
                    if (rho > T{0}) {
                        accepted = true;
                        // Nielsen λ decrease.
                        const T two_rho_m1 = T{2} * rho - T{1};
                        T factor = T{1} - two_rho_m1 * two_rho_m1 * two_rho_m1;
                        const T third = T{1} / T{3};
                        if (factor < third)
                            factor = third;
                        lambda = lambda * factor;
                        nu = T{2};
                        break;
                    }
                }
                lambda = lambda * nu;
                nu = nu * T{2};
            }
        } else {  // Dogleg
            // Gauss–Newton step.
            std::vector<T> p_gn;
            std::vector<T> A = JtJ;
            const bool gn_ok = cholesky_solve<T>(A, n, std::span<const T>{neg_g}, p_gn);
            // Cauchy (steepest-descent) step: pc = −(gᵀg / gᵀJᵀJ g) g.
            matvec<T>(JtJ, std::span<const T>{g}, n, Hg);
            const T gg = dot<T>(std::span<const T>{g}, std::span<const T>{g});
            const T gHg = dot<T>(std::span<const T>{g}, std::span<const T>{Hg});
            for (int tries = 0; tries < 50 && !accepted && !failed; ++tries) {
                if (gHg <= T{0}) {
                    failed = true;
                    break;
                }
                const T t = gg / gHg;
                std::vector<T> pc(n);
                for (std::size_t i = 0; i < n; ++i)
                    pc[i] = -t * g[i];
                const T norm_pc = norm<T>(std::span<const T>{pc});
                // Choose dogleg step within the trust radius.
                if (gn_ok && norm<T>(std::span<const T>{p_gn}) <= radius) {
                    step = p_gn;
                } else if (norm_pc >= radius) {
                    const T scale = radius / norm_pc;
                    for (std::size_t i = 0; i < n; ++i)
                        step[i] = pc[i] * scale;
                } else if (gn_ok) {
                    // Interpolate pc + τ (p_gn − pc) to hit ‖step‖ = radius.
                    std::vector<T> diff(n);
                    for (std::size_t i = 0; i < n; ++i)
                        diff[i] = p_gn[i] - pc[i];
                    const T a = dot<T>(std::span<const T>{diff}, std::span<const T>{diff});
                    const T b = T{2} * dot<T>(std::span<const T>{pc}, std::span<const T>{diff});
                    const T c = dot<T>(std::span<const T>{pc}, std::span<const T>{pc}) - radius * radius;
                    const T disc = b * b - T{4} * a * c;
                    const T tau = (-b + sqrt_(disc > T{0} ? disc : T{0})) / (T{2} * a);
                    for (std::size_t i = 0; i < n; ++i)
                        step[i] = pc[i] + diff[i] * tau;
                } else {
                    // No GN step available; clip Cauchy to the radius.
                    const T scale = radius / norm_pc;
                    for (std::size_t i = 0; i < n; ++i)
                        step[i] = pc[i] * scale;
                }

                for (std::size_t i = 0; i < n; ++i)
                    x_trial[i] = x[i] + step[i];
                eval(std::span<const T>{x_trial}, r_trial, J_trial);
                const T cost_trial = cost_of(r_trial);
                const T pred = cost - d::model_cost<T>(r, J, std::span<const T>{step}, m, n);
                const T step_norm = norm<T>(std::span<const T>{step});
                if (pred > T{0}) {
                    const T rho = (cost - cost_trial) / pred;
                    if (rho < T{1} / T{4}) {
                        radius = radius * T{1} / T{2};
                    } else if (rho > T{3} / T{4}) {
                        const T two_r = T{2} * radius;
                        radius = (step_norm >= radius) ? two_r : radius;
                    }
                    if (rho > T{0}) {
                        accepted = true;
                        break;
                    }
                } else {
                    radius = radius * T{1} / T{2};
                }
                if (radius < opts.parameter_tolerance) {
                    failed = true;
                }
            }
        }

        if (failed) {
            sum.status = Status::Failed;
            break;
        }
        if (!accepted) {
            // No decrease found; treat as converged at a stationary point.
            sum.status = Status::Converged;
            break;
        }

        // Commit the accepted step. r_trial / J_trial already hold the
        // evaluation at x + step.
        T step_norm{0}, xnorm{0};
        for (std::size_t i = 0; i < n; ++i) {
            x[i] = x[i] + step[i];
            step_norm = step_norm + step[i] * step[i];
            xnorm = xnorm + x[i] * x[i];
        }
        step_norm = sqrt_(step_norm);
        xnorm = sqrt_(xnorm);
        r = r_trial;
        J = J_trial;
        const T new_cost = cost_of(r);

        const T cost_drop = cost - new_cost;
        cost = new_cost;

        // Convergence on cost change or step size.
        if (cost_drop <= opts.function_tolerance * (cost + opts.function_tolerance)) {
            sum.status = Status::Converged;
            ++iter;
            break;
        }
        if (step_norm <= opts.parameter_tolerance * (xnorm + opts.parameter_tolerance)) {
            sum.status = Status::Converged;
            ++iter;
            break;
        }
    }

    sum.iterations = iter;
    sum.final_cost = cost;
    return sum;
}

}  // namespace branes::math::nls

#endif  // BRANES_MATH_NLS_SOLVER_HPP
