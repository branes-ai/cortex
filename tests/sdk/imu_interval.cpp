// S2 over an IMU interval (#616): the interpolation between samples, the RK4
// mean, and the transition Φ, which must be that mean's exact Jacobian.
//
// The Φ oracle is a central finite difference of the shipped mean in the
// body-frame right-perturbation error (R = R̂·Exp(δθ), the rest additive): for
// each of the 15 error coordinates, perturb by ±ε, propagate both, and take
// the error between the two results over 2ε. Its truncation is O(ε²) times the
// map's second derivative, about 1e-12 at ε = 1e-6, and its rounding about
// 1e-16 / ε = 1e-10; Φ must agree to 1e-8.
//
// ASCII-only TEST_CASE names (the Windows MSVC ctest job rejects non-ASCII).

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/stages/s2_propagation.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {
namespace ms = branes::sdk::msckf;
using T = double;
using St = ms::State<T>;
using Vec3 = St::Vec3;
using SO3 = St::SO3;

St tumbling_state() {
    St s(0.1);
    s.R = SO3::exp(Vec3{{0.4, -0.7, 1.1}});
    s.v = Vec3{{1.2, -0.4, 0.3}};
    s.p = Vec3{{3.0, -2.0, 1.5}};
    s.bg = Vec3{{0.01, -0.02, 0.015}};
    s.ba = Vec3{{0.05, 0.03, -0.04}};
    return s;
}

// A fast interval: the rate and the specific force both change across it.
ms::ImuInterval<T> fast_interval() {
    return {Vec3{{1.1, -0.6, 0.9}}, Vec3{{0.8, -0.3, 9.6}}, Vec3{{1.4, -0.2, 0.5}}, Vec3{{1.3, 0.2, 9.9}}, 0.005};
}

// Apply the error coordinate `i` scaled by `e` to the state.
St perturbed(const St& s, std::size_t i, T e) {
    St out = s;
    Vec3 d{};
    d[i % 3] = e;
    switch (i / 3) {
    case 0:
        out.R = s.R * SO3::exp(d);
        break;
    case 1:
        out.p = s.p + d;
        break;
    case 2:
        out.v = s.v + d;
        break;
    case 3:
        out.bg = s.bg + d;
        break;
    default:
        out.ba = s.ba + d;
        break;
    }
    return out;
}

// The 15-vector error of `a` relative to `b`, in the filter's layout.
void error_of(const St& a, const St& b, T* e) {
    const Vec3 th = (b.R.inverse() * a.R).log();
    const Vec3 blocks[5] = {th, a.p - b.p, a.v - b.v, a.bg - b.bg, a.ba - b.ba};
    for (std::size_t k = 0; k < 5; ++k)
        for (std::size_t j = 0; j < 3; ++j)
            e[3 * k + j] = blocks[k][j];
}
}  // namespace

TEST_CASE("S2 interval: Phi is the finite-difference Jacobian of the RK4 mean", "[sdk][s2][propagation]") {
    const ms::Propagator<T> prop;
    const St s = tumbling_state();
    const auto in = fast_interval();
    const auto F = prop.transition(s, in).F;
    REQUIRE(F.rows == 15);
    const T eps = 1e-6;
    double worst = 0.0;
    for (std::size_t i = 0; i < 15; ++i) {
        St plus = perturbed(s, i, eps), minus = perturbed(s, i, -eps);
        prop.propagate(plus, in);
        prop.propagate(minus, in);
        T e[15];
        error_of(plus, minus, e);
        for (std::size_t r = 0; r < 15; ++r) {
            const double fd = e[r] / (2 * eps);
            CAPTURE(r, i, fd, F(r, i));
            worst = std::max(worst, std::abs(fd - F(r, i)));
        }
    }
    CAPTURE(worst);
    REQUIRE(worst < 1e-8);
}

TEST_CASE("S2 interval: interpolate_sample is the line between samples, clamped", "[sdk][s2][propagation]") {
    const Vec3 a{{1.0, 2.0, -1.0}}, b{{3.0, 0.0, 1.0}};
    const auto mid = ms::interpolate_sample<T>(1.0, a, 1.01, b, 1.005);
    REQUIRE(std::abs(mid[0] - 2.0) < 1e-12);
    REQUIRE(std::abs(mid[1] - 1.0) < 1e-12);
    REQUIRE(std::abs(mid[2] - 0.0) < 1e-12);
    REQUIRE(ms::interpolate_sample<T>(1.0, a, 1.01, b, 0.5)[0] == 1.0);  // before: the start
    REQUIRE(ms::interpolate_sample<T>(1.0, a, 1.01, b, 2.0)[0] == 3.0);  // after: the end
    REQUIRE(ms::interpolate_sample<T>(1.0, a, 1.0, b, 1.0)[0] == 3.0);   // coincident: the newer
}

TEST_CASE("S2 interval: the one-sample call is the held interval", "[sdk][s2][propagation]") {
    const ms::Propagator<T> prop;
    St a = tumbling_state(), b = tumbling_state();
    const Vec3 w{{0.3, -0.2, 0.7}}, f{{0.1, 0.4, 9.7}};
    ms::stages::s2_propagation::apply(a, prop, w, f, 0.005);
    ms::stages::s2_propagation::apply(b, prop, ms::ImuInterval<T>::held(w, f, 0.005));
    REQUIRE((a.R.inverse() * b.R).log()[0] == 0.0);
    for (std::size_t i = 0; i < 3; ++i) {
        REQUIRE(a.p[i] == b.p[i]);
        REQUIRE(a.v[i] == b.v[i]);
    }
    for (std::size_t i = 0; i < a.cov.P.d.size(); ++i)
        REQUIRE(a.cov.P.d[i] == b.cov.P.d[i]);
}
