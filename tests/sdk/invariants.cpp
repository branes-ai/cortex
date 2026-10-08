// VIO invariant-check library (issue #445, epic #444 §B).
//
// Every check gets a known-answer regression in both directions: an input that
// satisfies the invariant passes, and a deliberately violating input fails. The
// matrix and manifold checks are instantiated in float, posit<32,2> and double,
// and a dedicated case shows the thresholds follow ε_T: the same small violation
// is inside float's tolerance and outside posit<32,2>'s and double's.

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/features/msckf_nullspace.hpp>
#include <branes/sdk/msckf/qr.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

// clang-format off
#include <universal/number/posit/posit.hpp>
#include <universal/number/posit/mathlib.hpp>
// clang-format on

#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <span>
#include <type_traits>
#include <vector>

namespace {

namespace inv = branes::sdk::eval::inv;
namespace ms = branes::sdk::msckf;
using inv::Stage;
using Posit32 = sw::universal::posit<32, 2>;

template <class T>
ms::DynMat<T> mat(std::size_t r, std::size_t c, std::initializer_list<double> v) {
    ms::DynMat<T> m(r, c);
    std::size_t i = 0;
    for (const double x : v)
        m.d[i++] = T(x);
    return m;
}

// A well-conditioned SPD 4×4: B Bᵀ + I with a fixed B.
template <class T>
ms::DynMat<T> spd4() {
    const auto b = mat<T>(4, 4, {1.0, 0.5, -0.25, 0.0, 0.2, 1.0, 0.3, -0.1, -0.4, 0.1, 0.8, 0.6, 0.3, -0.2, 0.1, 0.9});
    auto p = ms::mul(b, ms::transpose(b));
    for (std::size_t i = 0; i < 4; ++i)
        p(i, i) += T(1);
    ms::symmetrize(p);
    return p;
}

template <class T>
std::vector<T> vec(std::initializer_list<double> v) {
    std::vector<T> out;
    for (const double x : v)
        out.push_back(T(x));
    return out;
}

}  // namespace

#define INV_TYPES float, double, Posit32

// ── Element-wise, symmetry, definiteness ─────────────────────────────────────

TEMPLATE_TEST_CASE("invariant check_finite flags NaN", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const auto ok = vec<T>({1.0, -2.0, 0.5});
    REQUIRE(inv::check_finite<T>(std::span<const T>(ok), Stage::S6e_ekf_update).pass);
    auto bad = ok;
    bad[1] = T(std::numeric_limits<double>::quiet_NaN());
    const auto r = inv::check_finite<T>(std::span<const T>(bad), Stage::S6e_ekf_update, "dx.finite");
    REQUIRE_FALSE(r.pass);
    REQUIRE(r.value == 1.0);
    REQUIRE(r.name == "dx.finite");
}

TEMPLATE_TEST_CASE("invariant check_symmetric", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    auto p = spd4<T>();
    REQUIRE(inv::check_symmetric(p, Stage::S2_propagation).pass);
    p(0, 1) += T(0.1);
    const auto r = inv::check_symmetric(p, Stage::S2_propagation);
    REQUIRE_FALSE(r.pass);
    REQUIRE(r.value > 0.09);
    REQUIRE(r.value < 0.11);
}

TEMPLATE_TEST_CASE("invariant check_psd accepts a rank-deficient PSD matrix and rejects an indefinite one",
                   "[sdk][invariants]",
                   INV_TYPES) {
    using T = TestType;
    // B Bᵀ with B 4×2: PSD of rank 2 — two eigenvalues are zero up to roundoff,
    // the clone-augmentation situation where PSD (not PD) is the invariant.
    const auto b = mat<T>(4, 2, {1.0, 0.5, 0.2, 1.0, -0.4, 0.1, 0.3, -0.2});
    const auto p = ms::mul(b, ms::transpose(b));
    REQUIRE(inv::check_psd(p, Stage::S3_augmentation).pass);
    REQUIRE_FALSE(inv::check_spd(p, Stage::S3_augmentation).pass);  // singular is not SPD

    const auto indef = mat<T>(3, 3, {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.1});
    const auto r = inv::check_psd(indef, Stage::S6e_ekf_update);
    REQUIRE_FALSE(r.pass);
    REQUIRE(r.value > 0.099);  // λ_min = −0.1, reported in matrix units
    REQUIRE(r.value < 0.101);
}

TEMPLATE_TEST_CASE("invariant check_spd", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const auto r = inv::check_spd(spd4<T>(), Stage::S6d_gating);
    REQUIRE(r.pass);
    REQUIRE(r.bound == inv::Bound::Lower);
    REQUIRE(r.value > 0.5);  // λ_min ≥ 1 by construction (B Bᵀ + I)
}

TEMPLATE_TEST_CASE("invariant check_loewner_le: an update never adds uncertainty", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const auto pm = spd4<T>();
    // P⁺ = P⁻ − c c ᵀ for a small c: P⁺ ⪯ P⁻.
    const auto c = mat<T>(4, 1, {0.3, -0.2, 0.1, 0.4});
    const auto pp = inv::la::sub(pm, ms::mul(c, ms::transpose(c)));
    REQUIRE(inv::check_loewner_le(pp, pm, Stage::S6e_ekf_update).pass);
    REQUIRE(inv::check_loewner_le(pm, pm, Stage::S6e_ekf_update).pass);  // equality is allowed
    // P⁺ = P⁻ + c cᵀ: the "update" grew the covariance — violation of size ‖c‖².
    const auto grown = ms::add(pm, ms::mul(c, ms::transpose(c)));
    const auto r = inv::check_loewner_le(grown, pm, Stage::S6e_ekf_update);
    REQUIRE_FALSE(r.pass);
    REQUIRE(r.value > 0.29);  // ‖c‖² = 0.30
    REQUIRE(r.value < 0.31);
}

TEMPLATE_TEST_CASE("invariant report_condition_number never fails", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const auto d = mat<T>(2, 2, {100.0, 0.0, 0.0, 1.0});
    const auto r = inv::report_condition_number(d, Stage::S5_triangulation, "triangulation.normal_kappa");
    REQUIRE(r.pass);
    REQUIRE(r.value > 99.9);
    REQUIRE(r.value < 100.1);
}

// ── S6b_nullspace_projection ───────────────────────────────────────────────────────────

TEMPLATE_TEST_CASE("invariant S6b_nullspace_projection: annihilation, orthonormality, rank",
                   "[sdk][invariants]",
                   INV_TYPES) {
    using T = TestType;
    constexpr std::size_t m = 4, rows = 2 * m;
    std::vector<T> hf(rows * 3);
    for (std::size_t i = 0; i < rows; ++i) {
        hf[i * 3 + 0] = T(1) + T(static_cast<double>(i)) / T(7);
        hf[i * 3 + 1] = T(static_cast<double>(i % 3)) - T(1) + T(static_cast<double>(i)) / T(11);
        hf[i * 3 + 2] = T(1) - T(static_cast<double>(i)) / T(13);
    }
    std::vector<T> ix(rows * rows, T(0));
    for (std::size_t i = 0; i < rows; ++i)
        ix[i * rows + i] = T(1);
    const std::vector<T> r0(rows, T(0));
    const auto proj = branes::sdk::features::msckf_left_nullspace_project<T>(hf, ix, r0, rows, rows);

    ms::DynMat<T> nt(proj.rows, rows);  // Nᵀ: the projected identity
    nt.d = proj.H_x;
    ms::DynMat<T> hfm(rows, 3);
    hfm.d = hf;

    REQUIRE(inv::check_annihilates(nt, hfm, Stage::S6b_nullspace_projection, "nullspace.NtHf").pass);
    REQUIRE(inv::check_orthonormal_rows(nt, Stage::S6b_nullspace_projection, "nullspace.NtN").pass);
    REQUIRE(inv::check_rank(nt, 2 * m - 3, Stage::S6b_nullspace_projection).pass);
    REQUIRE(inv::check_dimension(proj.rows, 2 * m - 3, Stage::S6b_nullspace_projection).pass);

    // Violations: a scaled reflector breaks NᵀN = I; replacing a row with an H_f
    // column breaks NᵀH_f = 0; a duplicated row drops the rank.
    auto scaled = nt;
    for (std::size_t j = 0; j < rows; ++j)
        scaled(0, j) *= T(1.1);
    REQUIRE_FALSE(inv::check_orthonormal_rows(scaled, Stage::S6b_nullspace_projection).pass);
    auto leaky = nt;
    for (std::size_t j = 0; j < rows; ++j)
        leaky(0, j) = hfm(j, 0);
    REQUIRE_FALSE(inv::check_annihilates(leaky, hfm, Stage::S6b_nullspace_projection).pass);
    auto dup = nt;
    for (std::size_t j = 0; j < rows; ++j)
        dup(1, j) = dup(0, j);
    const auto rk = inv::check_rank(dup, 2 * m - 3, Stage::S6b_nullspace_projection);
    REQUIRE_FALSE(rk.pass);
    REQUIRE(rk.value == static_cast<double>(2 * m - 4));
}

// ── S6c_compression, S6e_ekf_update gain solve ─────────────────────────────────────────

TEMPLATE_TEST_CASE("invariant S6c_compression preserves the normal equations", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    constexpr std::size_t rows = 8, n = 4;
    ms::DynMat<T> hr(rows, n + 1);  // [H | r]
    for (std::size_t i = 0; i < rows; ++i)
        for (std::size_t j = 0; j <= n; ++j)
            hr(i, j) = T(std::sin(0.7 * static_cast<double>(i + 1) + 1.3 * static_cast<double>(j)));
    ms::DynMat<T> h(rows, n);
    std::vector<T> r(rows);
    for (std::size_t i = 0; i < rows; ++i) {
        for (std::size_t j = 0; j < n; ++j)
            h(i, j) = hr(i, j);
        r[i] = hr(i, n);
    }
    const auto R = ms::householder_qr_r(hr);  // (n+1)×(n+1) upper triangular
    ms::DynMat<T> hc(n, n);
    std::vector<T> rc(n);
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < n; ++j)
            hc(i, j) = R(i, j);
        rc[i] = R(i, n);
    }
    const auto ok = inv::check_compression<T>(h, r, hc, rc);
    REQUIRE(ok.size() == 3);
    for (const auto& res : ok)
        REQUIRE(res.pass);

    // Corrupt the compressed residual: Hᵀr is no longer preserved and ‖r_c‖ grows.
    auto rc_bad = rc;
    for (auto& v : rc_bad)
        v *= T(3);
    const auto bad = inv::check_compression<T>(h, r, hc, rc_bad);
    REQUIRE(bad[0].pass);  // HᵀH untouched
    REQUIRE_FALSE(bad[1].pass);
    REQUIRE_FALSE(bad[2].pass);
}

TEMPLATE_TEST_CASE("invariant S6e_ekf_update gain solve residual K S = P Ht", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const auto s = spd4<T>();
    const auto b = mat<T>(3, 4, {0.5, -0.2, 0.1, 0.3, 0.0, 0.4, -0.6, 0.2, 1.0, 0.1, 0.2, -0.3});
    // K = B S⁻¹  ⇔  Kᵀ = S⁻¹ Bᵀ (S symmetric).
    const auto k = ms::transpose(ms::spd_solve(s, ms::transpose(b)));
    REQUIRE(inv::check_solve_residual(k, s, b, Stage::S6e_ekf_update).pass);
    auto k_bad = k;
    k_bad(1, 2) += T(0.05);
    REQUIRE_FALSE(inv::check_solve_residual(k_bad, s, b, Stage::S6e_ekf_update).pass);
}

// ── S2_propagation / S3_augmentation / S9_marginalization identities ──────────────────────────────────────

TEMPLATE_TEST_CASE("invariant S2_propagation identity P' = Phi P Phit + Qd", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const auto p = spd4<T>();
    const auto phi = mat<T>(4, 4, {1.0, 0.1, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0, 0.0, 0.0, 1.0, 0.1, 0.0, 0.0, 0.0, 1.0});
    const auto qd = mat<T>(4, 4, {0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.02});
    const auto pn = ms::add(ms::mul(ms::mul(phi, p), ms::transpose(phi)), qd);
    REQUIRE(inv::check_covariance_propagation(p, phi, qd, pn).pass);
    REQUIRE(inv::check_psd(qd, Stage::S2_propagation, "Qd.psd").pass);
    // Dropping Q_d (a propagation that forgets process noise) is caught.
    const auto no_q = ms::mul(ms::mul(phi, p), ms::transpose(phi));
    REQUIRE_FALSE(inv::check_covariance_propagation(p, phi, qd, no_q).pass);
}

TEMPLATE_TEST_CASE("invariant S3_augmentation blocks and +6 dimension", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const auto p = spd4<T>();
    // A 2×4 "clone" Jacobian (the pose clone is 6×n; the block identity is the same).
    const auto j = mat<T>(2, 4, {1.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.0, -0.5});
    const auto jp = ms::mul(j, p);
    const auto jpjt = ms::mul(jp, ms::transpose(j));
    ms::DynMat<T> pa(6, 6);
    for (std::size_t r = 0; r < 4; ++r)
        for (std::size_t c = 0; c < 4; ++c)
            pa(r, c) = p(r, c);
    for (std::size_t r = 0; r < 2; ++r) {
        for (std::size_t c = 0; c < 4; ++c)
            pa(4 + r, c) = pa(c, 4 + r) = jp(r, c);
        for (std::size_t c = 0; c < 2; ++c)
            pa(4 + r, 4 + c) = jpjt(r, c);
    }
    const auto ok = inv::check_augmentation(p, j, pa);
    REQUIRE(ok.size() == 2);
    REQUIRE(ok[0].pass);
    REQUIRE(ok[1].pass);
    REQUIRE(inv::check_psd(pa, Stage::S3_augmentation).pass);  // singular (exact copy), still PSD

    // Augmenting without the cross-covariance (a clone that forgets it is
    // correlated with the IMU state) is caught.
    auto no_cross = pa;
    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 4; ++c)
            no_cross(4 + r, c) = no_cross(c, 4 + r) = T(0);
    REQUIRE_FALSE(inv::check_augmentation(p, j, no_cross)[1].pass);
    // Wrong dimension is a hard violation.
    const auto bad_dim = inv::check_augmentation(p, j, p);
    REQUIRE_FALSE(bad_dim[0].pass);
    REQUIRE_FALSE(bad_dim[1].pass);
}

TEMPLATE_TEST_CASE("invariant S9_marginalization equals the principal submatrix", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const auto p = spd4<T>();
    const std::array<std::size_t, 3> keep{0, 2, 3};
    ms::DynMat<T> pk(3, 3);
    for (std::size_t a = 0; a < 3; ++a)
        for (std::size_t b = 0; b < 3; ++b)
            pk(a, b) = p(keep[a], keep[b]);
    const auto ok = inv::check_principal_submatrix<T>(p, keep, pk);
    REQUIRE(ok[0].pass);
    REQUIRE(ok[1].pass);
    auto bad = pk;
    bad(0, 0) *= T(0.9);  // a "marginalization" that shrank a variance
    REQUIRE_FALSE(inv::check_principal_submatrix<T>(p, keep, bad)[1].pass);
}

// ── Unobservable subspace ───────────────────────────────────────────────────

TEMPLATE_TEST_CASE("invariant unobservable subspace: Phi preserves it, dx has no component along it",
                   "[sdk][invariants]",
                   INV_TYPES) {
    using T = TestType;
    // N spans {(1,1,0,0,0,0), (0,0,1,0,0,0)} — a 2-D stand-in for the 4-D gauge.
    const auto n = mat<T>(6, 2, {1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    // Φ = I + N Cᵀ maps span(N) into itself.
    const auto ct = mat<T>(2, 6, {0.1, 0.2, 0.0, -0.3, 0.1, 0.0, 0.0, 0.1, 0.4, 0.0, 0.0, 0.2});
    auto phi = ms::add(ms::DynMat<T>::identity(6), ms::mul(n, ct));
    REQUIRE(inv::check_subspace_preserved(phi, n, n).pass);
    // A Φ that leaks the gauge into coordinate 3 (outside span N) fails.
    auto leak = phi;
    leak(3, 2) += T(0.2);
    REQUIRE_FALSE(inv::check_subspace_preserved(leak, n, n).pass);

    const auto dx_ok = vec<T>({0.5, -0.5, 0.0, 0.3, -0.2, 0.1});  // ⟂ N
    REQUIRE(inv::check_no_update_along<T>(dx_ok, n).pass);
    const auto dx_bad = vec<T>({0.5, 0.5, 0.1, 0.3, -0.2, 0.1});  // moves along N
    const auto r = inv::check_no_update_along<T>(dx_bad, n);
    REQUIRE_FALSE(r.pass);
    REQUIRE(r.value > 0.70);  // ‖proj‖ = √(0.5 + 0.01) ≈ 0.714 (state units)
    REQUIRE(r.value < 0.72);
}

// ── Manifold, intrinsics, gravity ───────────────────────────────────────────

TEMPLATE_TEST_CASE("invariant check_so3 and unit quaternion", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    using SO3 = branes::math::lie::SO3<T>;
    const auto rot = SO3::exp({{T(0.3), T(-0.2), T(0.7)}});
    for (const auto& r : inv::check_so3(rot, Stage::S2_propagation))
        REQUIRE(r.pass);

    auto refl = inv::Mat3<T>::identity();
    refl(2, 2) = T(-1);
    const auto rr = inv::check_so3<T>(refl, Stage::Inputs, "R_imu_cam.orth", "R_imu_cam.det");
    REQUIRE(rr[0].pass);        // a reflection is orthogonal …
    REQUIRE_FALSE(rr[1].pass);  // … but det = −1
    REQUIRE(rr[1].value > 1.99);

    const auto drift = rot.matrix() * T(1.01);
    REQUIRE_FALSE(inv::check_so3<T>(drift, Stage::S2_propagation)[0].pass);

    const std::array<T, 4> q = rot.quaternion().e;
    REQUIRE(inv::check_unit_quaternion<T>(std::span<const T, 4>(q), Stage::S2_propagation).pass);
    const std::array<T, 4> q_bad{T(1), T(0.1), T(0), T(0)};
    REQUIRE_FALSE(inv::check_unit_quaternion<T>(std::span<const T, 4>(q_bad), Stage::S2_propagation).pass);
}

TEMPLATE_TEST_CASE("invariant intrinsics and gravity contract", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    for (const auto& r : inv::check_intrinsics<T>(T(458), T(457), T(367), T(248), T(752), T(480)))
        REQUIRE(r.pass);
    const auto bad = inv::check_intrinsics<T>(T(-458), T(457), T(800), T(248), T(752), T(480));
    REQUIRE_FALSE(bad[0].pass);
    REQUIRE_FALSE(bad[1].pass);
    REQUIRE(bad[1].value == 48.0);  // px outside the image

    const T g0 = T(9.81), mag_tol = T(0.05), dir_tol = T(0.01);
    for (const auto& r : inv::check_gravity<T>({{T(0), T(0), T(-9.81)}}, g0, mag_tol, dir_tol))
        REQUIRE(r.pass);
    const auto flipped = inv::check_gravity<T>({{T(0), T(0), T(9.81)}}, g0, mag_tol, dir_tol);
    REQUIRE(flipped[0].pass);        // magnitude is fine …
    REQUIRE_FALSE(flipped[1].pass);  // … the sign is not (angle π)
    REQUIRE(flipped[1].value > 3.14);
    REQUIRE_FALSE(inv::check_gravity<T>({{T(0), T(0), T(-9.0)}}, g0, mag_tol, dir_tol)[0].pass);
}

// ── Bounds, tracks, round trips, Jacobians ──────────────────────────────────

TEMPLATE_TEST_CASE("invariant bounds, depths, image bounds, track bookkeeping", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const auto bias = vec<T>({0.01, -0.02, 0.005});
    REQUIRE(inv::check_bounded<T>(bias, T(0.1), Stage::S1_initialization, "bias.gyro", "rad/s").pass);
    REQUIRE_FALSE(inv::check_bounded<T>(bias, T(0.015), Stage::S1_initialization, "bias.gyro", "rad/s").pass);

    const auto depth = vec<T>({4.0, 5.5, 3.2});
    REQUIRE(inv::check_at_least<T>(depth, T(0.1), Stage::S5_triangulation, "depth.positive", "m").pass);
    const auto behind = vec<T>({4.0, -0.5, 3.2});
    REQUIRE_FALSE(inv::check_at_least<T>(behind, T(0.1), Stage::S5_triangulation, "depth.positive", "m").pass);

    const std::vector<std::array<T, 2>> pts{{T(10), T(10)}, {T(700), T(400)}};
    REQUIRE(inv::check_points_in_image<T>(pts, T(752), T(480)).pass);
    const std::vector<std::array<T, 2>> out{{T(10), T(10)}, {T(760), T(400)}, {T(-1), T(5)}};
    REQUIRE(inv::check_points_in_image<T>(out, T(752), T(480)).value == 2.0);

    const std::vector<std::size_t> lens{2, 5, 11};
    REQUIRE(inv::check_track_lengths(std::span<const std::size_t>(lens.data(), 2), 11).pass);
    REQUIRE(inv::check_track_lengths(lens, 11).pass);
    REQUIRE_FALSE(inv::check_track_lengths(lens, 10).pass);
}

TEMPLATE_TEST_CASE("invariant round trip and Jacobian vs finite differences", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const auto a = vec<T>({0.25, -0.5, 1.0});
    auto b = a;
    REQUIRE(inv::check_round_trip<T>(a, b, Stage::S0_sensor_model, "undistort.round_trip", "normalized").pass);
    b[2] += T(1e-2);
    REQUIRE_FALSE(inv::check_round_trip<T>(a, b, Stage::S0_sensor_model, "undistort.round_trip", "normalized").pass);

    // f(x) = [x0·x1 + x2³, x0² − x1·x2]
    auto f = [](std::span<const T> x) {
        return std::vector<T>{x[0] * x[1] + x[2] * x[2] * x[2], x[0] * x[0] - x[1] * x[2]};
    };
    const auto x = vec<T>({0.7, -1.2, 0.4});
    // J = [[x1, x0, 3x2²], [2x0, −x2, −x1]]
    auto j = vec<T>({-1.2, 0.7, 3.0 * 0.16, 1.4, -0.4, 1.2});
    REQUIRE(inv::check_jacobian_fd<T>(f, x, j, Stage::S6a_jacobians, "H_x.fd").pass);
    j[3] = T(-1.4);  // a sign error in one entry
    const auto r = inv::check_jacobian_fd<T>(f, x, j, Stage::S6a_jacobians, "H_x.fd");
    REQUIRE_FALSE(r.pass);
    REQUIRE(r.value > 2.7);
}

// ── Streams and windows (double timestamps, double telemetry) ───────────────

TEST_CASE("invariant timestamps strictly increasing and dt bounds", "[sdk][invariants]") {
    const std::vector<double> ok{0.000, 0.005, 0.010, 0.015, 0.020};
    REQUIRE(inv::check_timestamps_increasing(ok).pass);
    for (const auto& r : inv::check_dt_bounds(ok, 0.004, 0.006))
        REQUIRE(r.pass);

    const std::vector<double> bad{0.000, 0.005, 0.005, 0.004, 0.030};
    const auto mono = inv::check_timestamps_increasing(bad);
    REQUIRE_FALSE(mono.pass);
    REQUIRE(mono.value == 2.0);  // a repeat and a step back
    const auto dt = inv::check_dt_bounds(bad, 0.004, 0.006);
    REQUIRE_FALSE(dt[0].pass);  // min Δt = −1 ms
    REQUIRE_FALSE(dt[1].pass);  // max Δt = 26 ms dropout
}

TEMPLATE_TEST_CASE("invariant static window accel norm approx g", "[sdk][invariants]", INV_TYPES) {
    using T = TestType;
    const std::vector<std::array<T, 3>> still{{T(0.1), T(-0.05), T(9.80)}, {T(0.08), T(-0.04), T(9.82)}};
    REQUIRE(inv::check_static_gravity<T>(still, T(9.81), T(0.05)).pass);
    const std::vector<std::array<T, 3>> moving{{T(1.5), T(0.0), T(9.81)}, {T(2.0), T(0.5), T(10.5)}};
    REQUIRE_FALSE(inv::check_static_gravity<T>(moving, T(9.81), T(0.05)).pass);
}

TEST_CASE("invariant chi2 window: NIS approx dof passes, 3x dof fails", "[sdk][invariants]") {
    branes::sdk::eval::ConsistencyAccumulator good, over;
    for (int i = 0; i < 500; ++i) {
        good.add(5.0, 5);   // statistic at its expectation
        over.add(15.0, 5);  // the over-confident filter
    }
    const auto g = inv::check_chi2_window(good, Stage::S6d_gating, "nis.window");
    REQUIRE(g.pass);
    REQUIRE(g.bound == inv::Bound::Band);
    const auto o = inv::check_chi2_window(over, Stage::S6d_gating, "nis.window");
    REQUIRE_FALSE(o.pass);
    REQUIRE(o.value == 3.0);
    REQUIRE_FALSE(inv::check_chi2_window(branes::sdk::eval::ConsistencyAccumulator{}, Stage::EndToEnd, "nees").pass);
}

// ── Report ───────────────────────────────────────────────────────────────────

TEST_CASE("invariant report: first failure in pipeline order, tightest margin, per stage", "[sdk][invariants]") {
    inv::InvariantReport rep;
    const auto p = spd4<double>();
    rep.add(inv::check_symmetric(p, Stage::S2_propagation, "P.symmetric"));
    rep.add(inv::check_psd(p, Stage::S2_propagation, "P.psd"));
    rep.add(inv::check_scalar(0.4, 0.5, inv::Bound::Upper, Stage::S5_triangulation, "reproj.max", "px"));
    rep.add(inv::check_scalar(2.0, 1.0, inv::Bound::Upper, Stage::S6e_ekf_update, "dx.along_N", "rad"));
    rep.add(inv::check_scalar(9.0, 1.0, inv::Bound::Upper, Stage::S9_marginalization, "late", "count"));
    REQUIRE_FALSE(rep.all_pass());
    REQUIRE(rep.failures() == 2);
    REQUIRE(rep.first_failure()->name == "dx.along_N");
    REQUIRE(rep.first_failure()->stage == Stage::S6e_ekf_update);
    REQUIRE(rep.tightest()->name == "late");
    REQUIRE(rep.for_stage(Stage::S2_propagation).size() == 2);
    REQUIRE(inv::to_string(Stage::S6e_ekf_update) == "S6e_ekf_update");
}

// ── Tolerance scaling with the arithmetic type ──────────────────────────────

TEST_CASE("invariant tolerances scale with epsilon of the arithmetic type", "[sdk][invariants][precision]") {
    const double tf = inv::arithmetic_tolerance<float>(4);
    const double tp = inv::arithmetic_tolerance<Posit32>(4);
    const double td = inv::arithmetic_tolerance<double>(4);
    // The thresholds are exactly the ε ratios apart: float 2^-23, posit<32,2>
    // 2^-27 (at 1), double 2^-52.
    REQUIRE(tf / tp == 16.0);
    REQUIRE(tp / td == std::ldexp(1.0, 25));
    REQUIRE(td == inv::kDefaultSafety * 4.0 * std::numeric_limits<double>::epsilon());
}

TEMPLATE_TEST_CASE("invariant verdict on a small asymmetry follows the type's epsilon",
                   "[sdk][invariants][precision]",
                   INV_TYPES) {
    using T = TestType;
    // An asymmetry of 2e-6 · ‖P‖ is roundoff in float (tolerance 16·4·2^-23 ≈ 7.6e-6)
    // but a genuine defect in posit<32,2> (≈ 4.8e-7) and double (≈ 1.4e-14).
    auto p = spd4<T>();
    const double scale = static_cast<double>(inv::la::max_abs(p));
    p(0, 1) += T(2e-6 * scale);
    const auto r = inv::check_symmetric(p, Stage::S2_propagation);
    if constexpr (std::is_same_v<T, float>)
        REQUIRE(r.pass);
    else
        REQUIRE_FALSE(r.pass);
}
