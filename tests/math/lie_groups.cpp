// Lie-group tests (issue #29): round-trip exp/log, adjoint consistency,
// and finite-difference checks of the analytical Jacobians, for SO3,
// SE3, and Sim3. Round-trip runs across double/float/posit; the
// finite-difference Jacobian checks run in double (FD needs the
// precision). The exhaustive type-genericity matrix lives in #31.

#include <branes/math/lie/se3.hpp>
#include <branes/math/lie/sim3.hpp>
#include <branes/math/lie/so3.hpp>

#include <catch2/catch_test_macros.hpp>

// posit.hpp must precede mathlib.hpp (the latter defines the math shims
// in terms of posit<>); guard the order against clang-format's regroup.
// clang-format off
#include <universal/number/posit/posit.hpp>
#include <universal/number/posit/mathlib.hpp>
// clang-format on

#include <cmath>
#include <cstddef>

namespace lie = branes::math::lie;
namespace det = branes::math::lie::detail;

namespace {

template <class T, std::size_t N>
void require_vec_close(const det::Vec<T, N>& a, const det::Vec<T, N>& b, double tol) {
    for (std::size_t i = 0; i < N; ++i) {
        REQUIRE(std::abs(double(a[i]) - double(b[i])) < tol);
    }
}

template <class T, std::size_t R, std::size_t C>
void require_mat_close(const det::Mat<T, R, C>& a, const det::Mat<T, R, C>& b, double tol) {
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < C; ++j)
            REQUIRE(std::abs(double(a(i, j)) - double(b(i, j))) < tol);
}

}  // namespace

// ── SO3 ──────────────────────────────────────────────────────────────

TEST_CASE("SO3 exp/log round-trip", "[math][lie][so3]") {
    using V = det::Vec<double, 3>;
    const V phis[] = {
        V{{0.0, 0.0, 0.0}}, V{{0.3, -0.2, 0.7}}, V{{1.5, 0.4, -0.9}}, V{{1e-7, -2e-7, 5e-8}}, V{{2.9, 0.1, 0.05}}};
    for (const auto& phi : phis) {
        auto R = lie::SO3<double>::exp(phi);
        require_vec_close(R.log(), phi, 1e-9);
    }
}

TEST_CASE("SO3 matrix is a proper rotation", "[math][lie][so3]") {
    using V = det::Vec<double, 3>;
    auto R = lie::SO3<double>::exp(V{{0.6, -0.4, 1.2}});
    auto M = R.matrix();
    auto should_be_I = M * det::transpose(M);
    require_mat_close(should_be_I, det::Mat<double, 3, 3>::identity(), 1e-12);
    // det(M) == 1
    const double d = M(0, 0) * (M(1, 1) * M(2, 2) - M(1, 2) * M(2, 1)) -
                     M(0, 1) * (M(1, 0) * M(2, 2) - M(1, 2) * M(2, 0)) +
                     M(0, 2) * (M(1, 0) * M(2, 1) - M(1, 1) * M(2, 0));
    REQUIRE(std::abs(d - 1.0) < 1e-12);
}

TEST_CASE("SO3 compose and inverse", "[math][lie][so3]") {
    using V = det::Vec<double, 3>;
    auto A = lie::SO3<double>::exp(V{{0.3, 0.5, -0.2}});
    auto B = lie::SO3<double>::exp(V{{-0.7, 0.1, 0.4}});
    auto AB = A * B;
    // (AB) B^-1 A^-1 == I
    auto I = AB * B.inverse() * A.inverse();
    require_mat_close(I.matrix(), det::Mat<double, 3, 3>::identity(), 1e-12);
    // adjoint == matrix
    require_mat_close(A.adjoint(), A.matrix(), 1e-15);
}

TEST_CASE("SO3 right Jacobian matches finite differences", "[math][lie][so3]") {
    using V = det::Vec<double, 3>;
    const V phi{{0.4, -0.3, 0.8}};
    const double eps = 1e-6;
    auto base = lie::SO3<double>::exp(phi);
    auto Jr = lie::SO3<double>::right_jacobian(phi);
    // Jr column j ~ log( base^{-1} exp(phi + eps e_j) ) / eps
    for (std::size_t j = 0; j < 3; ++j) {
        V dphi = phi;
        dphi[j] += eps;
        auto delta = (base.inverse() * lie::SO3<double>::exp(dphi)).log();
        for (std::size_t i = 0; i < 3; ++i) {
            REQUIRE(std::abs(Jr(i, j) - delta[i] / eps) < 1e-5);
        }
    }
}

TEST_CASE("SO3 left Jacobian matches finite differences", "[math][lie][so3]") {
    using V = det::Vec<double, 3>;
    const V phi{{0.4, -0.3, 0.8}};
    const double eps = 1e-6;
    auto base = lie::SO3<double>::exp(phi);
    auto Jl = lie::SO3<double>::left_jacobian(phi);
    // Jl column j ~ log( exp(phi + eps e_j) base^{-1} ) / eps
    for (std::size_t j = 0; j < 3; ++j) {
        V dphi = phi;
        dphi[j] += eps;
        auto delta = (lie::SO3<double>::exp(dphi) * base.inverse()).log();
        for (std::size_t i = 0; i < 3; ++i) {
            REQUIRE(std::abs(Jl(i, j) - delta[i] / eps) < 1e-5);
        }
    }
}

TEST_CASE("SO3 Jacobian inverses are actual inverses", "[math][lie][so3]") {
    using V = det::Vec<double, 3>;
    const V phi{{0.7, 0.2, -0.5}};
    auto Jl = lie::SO3<double>::left_jacobian(phi);
    auto Jli = lie::SO3<double>::left_jacobian_inverse(phi);
    require_mat_close(Jl * Jli, det::Mat<double, 3, 3>::identity(), 1e-10);
    auto Jr = lie::SO3<double>::right_jacobian(phi);
    auto Jri = lie::SO3<double>::right_jacobian_inverse(phi);
    require_mat_close(Jr * Jri, det::Mat<double, 3, 3>::identity(), 1e-10);
}

TEST_CASE("SO3 round-trip across arithmetic types", "[math][lie][so3]") {
    using posit32 = sw::universal::posit<32, 2>;
    {
        det::Vec<float, 3> phi{{0.3f, -0.2f, 0.7f}};
        auto R = lie::SO3<float>::exp(phi);
        require_vec_close(R.log(), phi, 1e-4);
    }
    {
        det::Vec<posit32, 3> phi{{posit32{0.3}, posit32{-0.2}, posit32{0.7}}};
        auto R = lie::SO3<posit32>::exp(phi);
        require_vec_close(R.log(), phi, 1e-5);
    }
}

TEST_CASE("SO3 left Jacobian inverse is finite and correct near a half-turn", "[math][lie][so3]") {
    // Regression: the inverse-left-Jacobian's closed form divides by
    // sin(theta), which vanishes at theta = pi. It must stay finite (the
    // singularity is removable) and remain a true inverse there.
    using V = det::Vec<double, 3>;
    for (double ang : {3.0, 3.1, 3.14159265358979}) {
        const V phi{{0.0, 0.0, ang}};  // axis = +z, |phi| = ang ~ pi
        auto Jli = lie::SO3<double>::left_jacobian_inverse(phi);
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                REQUIRE(std::isfinite(Jli(i, j)));
        auto Jl = lie::SO3<double>::left_jacobian(phi);
        require_mat_close(Jl * Jli, det::Mat<double, 3, 3>::identity(), 1e-9);
    }
}

// ── SE3 ──────────────────────────────────────────────────────────────

namespace {

template <class T>
det::Vec<T, 6> vec6(T a, T b, T c, T d, T e, T f) {
    return det::Vec<T, 6>{{a, b, c, d, e, f}};
}

}  // namespace

TEST_CASE("SE3 exp/log round-trip", "[math][lie][se3]") {
    using V6 = det::Vec<double, 6>;
    const V6 xis[] = {vec6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                      vec6(1.0, -2.0, 0.5, 0.3, -0.2, 0.7),
                      vec6(-0.4, 0.8, 2.0, 1.5, 0.4, -0.9),
                      vec6(0.1, 0.2, 0.3, 1e-7, -2e-7, 5e-8)};
    for (const auto& xi : xis) {
        auto T = lie::SE3<double>::exp(xi);
        require_vec_close(T.log(), xi, 1e-9);
    }
}

TEST_CASE("SE3 compose, inverse, and action", "[math][lie][se3]") {
    auto A = lie::SE3<double>::exp(vec6(1.0, -0.5, 0.3, 0.2, 0.4, -0.1));
    auto B = lie::SE3<double>::exp(vec6(-0.2, 0.7, 1.1, -0.3, 0.5, 0.2));
    // (A B) B^-1 A^-1 == identity
    auto I = A * B * B.inverse() * A.inverse();
    require_mat_close(I.matrix(), det::Mat<double, 4, 4>::identity(), 1e-12);
    // action consistent with homogeneous matrix
    det::Vec<double, 3> p{{0.5, -1.0, 2.0}};
    auto q = A * p;
    auto M = A.matrix();
    for (std::size_t i = 0; i < 3; ++i) {
        double e = M(i, 0) * p[0] + M(i, 1) * p[1] + M(i, 2) * p[2] + M(i, 3);
        REQUIRE(std::abs(q[i] - e) < 1e-12);
    }
}

TEST_CASE("SE3 adjoint matches conjugation of the algebra", "[math][lie][se3]") {
    // Adj(T) xi == log( T exp(xi) T^-1 ) to first order; check via the
    // exact identity T exp(xi) T^-1 == exp(Adj(T) xi).
    auto Tr = lie::SE3<double>::exp(vec6(0.3, -0.6, 0.9, 0.4, 0.1, -0.2));
    const auto xi = vec6(0.05, -0.03, 0.02, 0.04, -0.01, 0.03);
    auto Adj = Tr.adjoint();
    det::Vec<double, 6> Adx{};
    for (std::size_t i = 0; i < 6; ++i) {
        double s = 0.0;
        for (std::size_t j = 0; j < 6; ++j)
            s += Adj(i, j) * xi[j];
        Adx[i] = s;
    }
    auto lhs = (Tr * lie::SE3<double>::exp(xi) * Tr.inverse()).log();
    require_vec_close(lhs, Adx, 1e-9);
}

TEST_CASE("SE3 right Jacobian matches finite differences", "[math][lie][se3]") {
    const auto xi = vec6(0.6, -0.4, 0.2, 0.3, 0.5, -0.7);
    const double eps = 1e-6;
    auto base = lie::SE3<double>::exp(xi);
    auto Jr = lie::SE3<double>::right_jacobian(xi);
    for (std::size_t j = 0; j < 6; ++j) {
        auto dxi = xi;
        dxi[j] += eps;
        auto delta = (base.inverse() * lie::SE3<double>::exp(dxi)).log();
        for (std::size_t i = 0; i < 6; ++i) {
            REQUIRE(std::abs(Jr(i, j) - delta[i] / eps) < 1e-4);
        }
    }
}

TEST_CASE("SE3 Jacobian inverse is an actual inverse", "[math][lie][se3]") {
    const auto xi = vec6(0.6, -0.4, 0.2, 0.3, 0.5, -0.7);
    auto Jl = lie::SE3<double>::left_jacobian(xi);
    auto Jli = lie::SE3<double>::left_jacobian_inverse(xi);
    require_mat_close(Jl * Jli, det::Mat<double, 6, 6>::identity(), 1e-9);
}

TEST_CASE("SE3 round-trip across arithmetic types", "[math][lie][se3]") {
    using posit32 = sw::universal::posit<32, 2>;
    det::Vec<posit32, 6> xi{{posit32{0.5}, posit32{-0.3}, posit32{0.2}, posit32{0.3}, posit32{-0.2}, posit32{0.4}}};
    auto Tp = lie::SE3<posit32>::exp(xi);
    require_vec_close(Tp.log(), xi, 1e-5);
}

TEST_CASE("SE3 right Jacobian is accurate at small rotation angles", "[math][lie][se3]") {
    // Regression: the Q-block coefficients divide by theta^4/theta^5, so
    // the closed form is garbage for small-but-nonzero theta (the old
    // 1e-5 series threshold left theta in [1e-5, 1e-3] using it). The
    // FD check below fails pre-fix at theta <= 1e-3.
    const double eps = 1e-7;
    for (double th : {1e-4, 1e-3, 5e-3, 1e-2, 5e-2}) {
        // |phi| = th (0.6^2 + 0.8^2 = 1), with a nonzero translation so
        // the Q block (rho-phi coupling) is exercised.
        const auto xi = vec6(0.5, -0.3, 0.2, th * 0.6, th * 0.8, 0.0);
        auto base = lie::SE3<double>::exp(xi);
        auto Jr = lie::SE3<double>::right_jacobian(xi);
        for (std::size_t j = 0; j < 6; ++j) {
            auto dxi = xi;
            dxi[j] += eps;
            auto delta = (base.inverse() * lie::SE3<double>::exp(dxi)).log();
            for (std::size_t i = 0; i < 6; ++i) {
                REQUIRE(std::abs(Jr(i, j) - delta[i] / eps) < 1e-4);
            }
        }
    }
}

TEST_CASE("SE3 log is finite for a half-turn rotation", "[math][lie][se3]") {
    // Exercises the SO3 inverse-left-Jacobian (used by SE3::log) at theta
    // = pi, which previously produced NaN translation components.
    const auto xi = vec6(0.4, -0.2, 0.1, 3.14159265358979, 0.0, 0.0);
    auto Tr = lie::SE3<double>::exp(xi);
    auto lg = Tr.log();
    for (std::size_t i = 0; i < 6; ++i)
        REQUIRE(std::isfinite(lg[i]));
}

// ── Sim3 ─────────────────────────────────────────────────────────────

namespace {

template <class T>
det::Vec<T, 7> vec7(T a, T b, T c, T d, T e, T f, T g) {
    return det::Vec<T, 7>{{a, b, c, d, e, f, g}};
}

}  // namespace

TEST_CASE("Sim3 exp/log round-trip", "[math][lie][sim3]") {
    using V7 = det::Vec<double, 7>;
    const V7 taus[] = {vec7(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                       vec7(1.0, -2.0, 0.5, 0.3, -0.2, 0.7, 0.25),
                       vec7(-0.4, 0.8, 2.0, 1.5, 0.4, -0.9, -0.4),
                       vec7(0.5, -0.3, 0.2, 1e-7, -2e-7, 5e-8, 1e-7),
                       vec7(0.3, 0.1, -0.2, 0.6, -0.5, 0.4, 0.0)};
    for (const auto& tau : taus) {
        auto S = lie::Sim3<double>::exp(tau);
        require_vec_close(S.log(), tau, 1e-8);
    }
}

TEST_CASE("Sim3 compose, inverse, and action", "[math][lie][sim3]") {
    auto A = lie::Sim3<double>::exp(vec7(1.0, -0.5, 0.3, 0.2, 0.4, -0.1, 0.3));
    auto B = lie::Sim3<double>::exp(vec7(-0.2, 0.7, 1.1, -0.3, 0.5, 0.2, -0.2));
    auto I = A * B * B.inverse() * A.inverse();
    require_mat_close(I.matrix(), det::Mat<double, 4, 4>::identity(), 1e-11);
    // action consistent with homogeneous matrix
    det::Vec<double, 3> p{{0.5, -1.0, 2.0}};
    auto q = A * p;
    auto M = A.matrix();
    for (std::size_t i = 0; i < 3; ++i) {
        double e = M(i, 0) * p[0] + M(i, 1) * p[1] + M(i, 2) * p[2] + M(i, 3);
        REQUIRE(std::abs(q[i] - e) < 1e-12);
    }
}

TEST_CASE("Sim3 adjoint matches conjugation of the algebra", "[math][lie][sim3]") {
    auto S = lie::Sim3<double>::exp(vec7(0.3, -0.6, 0.9, 0.4, 0.1, -0.2, 0.2));
    const auto tau = vec7(0.04, -0.03, 0.02, 0.03, -0.01, 0.02, 0.015);
    auto Adj = S.adjoint();
    det::Vec<double, 7> Adx{};
    for (std::size_t i = 0; i < 7; ++i) {
        double s = 0.0;
        for (std::size_t j = 0; j < 7; ++j)
            s += Adj(i, j) * tau[j];
        Adx[i] = s;
    }
    auto lhs = (S * lie::Sim3<double>::exp(tau) * S.inverse()).log();
    require_vec_close(lhs, Adx, 1e-8);
}

TEST_CASE("Sim3 small adjoint is the derivative of Adj", "[math][lie][sim3]") {
    // ad(τ) == d/dt Adj(exp(tτ)) |_{t=0}, checked by finite differences.
    const auto tau = vec7(0.5, -0.3, 0.2, 0.4, 0.1, -0.2, 0.3);
    const double eps = 1e-6;
    auto adp = lie::Sim3<double>::exp(
        vec7(tau[0] * eps, tau[1] * eps, tau[2] * eps, tau[3] * eps, tau[4] * eps, tau[5] * eps, tau[6] * eps));
    auto Adj_p = adp.adjoint();
    auto ad = lie::Sim3<double>::small_adjoint(tau);
    for (std::size_t i = 0; i < 7; ++i)
        for (std::size_t j = 0; j < 7; ++j) {
            const double fd = (Adj_p(i, j) - (i == j ? 1.0 : 0.0)) / eps;
            REQUIRE(std::abs(ad(i, j) - fd) < 1e-4);
        }
}

TEST_CASE("Sim3 right Jacobian matches finite differences", "[math][lie][sim3]") {
    const auto tau = vec7(0.3, -0.2, 0.15, 0.25, 0.35, -0.4, 0.2);
    const double eps = 1e-6;
    auto base = lie::Sim3<double>::exp(tau);
    auto Jr = lie::Sim3<double>::right_jacobian(tau);
    for (std::size_t j = 0; j < 7; ++j) {
        auto dt = tau;
        dt[j] += eps;
        auto delta = (base.inverse() * lie::Sim3<double>::exp(dt)).log();
        for (std::size_t i = 0; i < 7; ++i) {
            REQUIRE(std::abs(Jr(i, j) - delta[i] / eps) < 1e-4);
        }
    }
}

TEST_CASE("Sim3 Jacobian inverse is an actual inverse", "[math][lie][sim3]") {
    const auto tau = vec7(0.3, -0.2, 0.15, 0.25, 0.35, -0.4, 0.2);
    auto Jl = lie::Sim3<double>::left_jacobian(tau);
    auto Jli = lie::Sim3<double>::left_jacobian_inverse(tau);
    require_mat_close(Jl * Jli, det::Mat<double, 7, 7>::identity(), 1e-9);
}

TEST_CASE("Sim3 round-trip across arithmetic types", "[math][lie][sim3]") {
    using posit32 = sw::universal::posit<32, 2>;
    det::Vec<posit32, 7> tau{
        {posit32{0.5}, posit32{-0.3}, posit32{0.2}, posit32{0.3}, posit32{-0.2}, posit32{0.4}, posit32{0.15}}};
    auto Sp = lie::Sim3<posit32>::exp(tau);
    require_vec_close(Sp.log(), tau, 1e-4);
}

// ── #161: degenerate-input guards must admit all legitimate inputs ──────

TEST_CASE("SO3 normalizes non-unit and small-but-nonzero quaternions", "[math][lie][so3]") {
    // A non-unit quaternion is a documented valid input: it must normalize,
    // not trip the positive-norm guard.
    const lie::SO3<double> a(det::Vec<double, 4>{{2.0, 0.0, 0.0, 0.0}});
    require_mat_close(a.matrix(), det::Mat<double, 3, 3>::identity(), 1e-12);

    // Tiny but non-zero magnitude is still a valid direction once normalized.
    const lie::SO3<double> b(det::Vec<double, 4>{{1e-6, 1e-6, 0.0, 0.0}});
    require_mat_close(b.matrix() * det::transpose(b.matrix()), det::Mat<double, 3, 3>::identity(), 1e-9);
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            REQUIRE(std::isfinite(b.matrix()(i, j)));
}

TEST_CASE("Sim3 admits a small positive scale and inverts it", "[math][lie][sim3]") {
    const double s = 1e-3;
    const lie::Sim3<double> S(
        s, lie::SO3<double>::exp(det::Vec<double, 3>{{0.1, -0.2, 0.05}}), det::Vec<double, 3>{{0.4, -0.1, 0.2}});
    const auto Si = S.inverse();
    REQUIRE(std::abs(double(Si.scale()) - 1.0 / s) < 1e-6);
    // S · S⁻¹ = identity (scale 1, no rotation, no translation).
    const auto I = S * Si;
    REQUIRE(std::abs(double(I.scale()) - 1.0) < 1e-9);
    require_vec_close(I.translation(), det::Vec<double, 3>{{0.0, 0.0, 0.0}}, 1e-9);
}
