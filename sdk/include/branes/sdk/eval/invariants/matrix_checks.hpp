// SPDX-License-Identifier: MIT
//
// branes/sdk/eval/invariants/matrix_checks.hpp — the linear-algebra invariants of
// the MSCKF stages (epic #444 §B, issue #445): covariance symmetry and
// definiteness, the Loewner order of an update, the propagation / augmentation /
// marginalization block identities, the null-space projection, QR compression,
// the Kalman-gain solve, and the unobservable subspace.
//
// Each check takes the matrices a stage consumed and produced (as a captured
// fixture or a bench run holds them), so it is independent of how the stage is
// implemented and runs unchanged on the MTL5 port (#448). Thresholds follow the
// policy in core.hpp: safety · n · ε_T · scale.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_EVAL_INVARIANTS_MATRIX_CHECKS_HPP
#define BRANES_SDK_EVAL_INVARIANTS_MATRIX_CHECKS_HPP

#include <branes/sdk/eval/invariants/core.hpp>
#include <branes/sdk/eval/invariants/linalg.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace branes::sdk::eval::inv {

using msckf::DynMat;

namespace detail {
/// A shape mismatch is a hard violation, reported as an infinite residual so it
/// can never pass and sorts to the top of a margin ranking.
[[nodiscard]] inline InvariantResult shape_violation(std::string_view name, Stage stage) {
    return make(name, "shape", stage, Bound::Upper, kInf, 0.0);
}

/// DynMat overload of core.hpp's all_finite.
template <math::Scalar T>
[[nodiscard]] bool all_finite(const DynMat<T>& a) {
    return all_finite<T>(std::span<const T>(a.d));
}
}  // namespace detail

// ── Element-wise ─────────────────────────────────────────────────────────────

/// Every element finite (no NaN / Inf / NaR). Value: the count of non-finite
/// elements; threshold 0.
template <math::Scalar T>
[[nodiscard]] InvariantResult check_finite(std::span<const T> x, Stage stage, std::string_view name = "finite") {
    using std::isfinite;
    std::size_t bad = 0;
    for (const T& v : x)
        bad += isfinite(v) ? 0 : 1;
    return detail::make(name, "count", stage, Bound::Upper, static_cast<double>(bad), 0.0);
}

template <math::Scalar T>
[[nodiscard]] InvariantResult check_finite(const DynMat<T>& a, Stage stage, std::string_view name = "finite") {
    return check_finite<T>(std::span<const T>(a.d), stage, name);
}

// ── Symmetry and definiteness ───────────────────────────────────────────────

/// A = Aᵀ. Value: max |A − Aᵀ| in A's units; threshold: safety·n·ε_T·‖A‖_max.
template <math::Scalar T>
[[nodiscard]] InvariantResult
check_symmetric(const DynMat<T>& a, Stage stage, std::string_view name = "symmetric", double safety = kDefaultSafety) {
    if (a.rows != a.cols)
        return detail::shape_violation(name, stage);
    if (!(detail::all_finite(a)))
        return detail::nonfinite_violation(name, stage);
    T worst{0};
    for (std::size_t i = 0; i < a.rows; ++i)
        for (std::size_t j = i + 1; j < a.cols; ++j)
            worst = std::max(worst, la::abs_(a(i, j) - a(j, i)));
    const double tol = arithmetic_tolerance<T>(a.rows, detail::to_double(la::max_abs(a)), safety);
    return detail::make(name, "matrix units", stage, Bound::Upper, detail::to_double(worst), tol);
}

/// A ⪰ 0. Value: max(0, −λ_min(A)) in A's units — how far the most negative
/// eigenvalue sits below zero; threshold: safety·n·ε_T·‖A‖_F (the eigenvalue
/// perturbation the symmetric part can carry from roundoff alone).
template <math::Scalar T>
[[nodiscard]] InvariantResult
check_psd(const DynMat<T>& a, Stage stage, std::string_view name = "psd", double safety = kDefaultSafety) {
    if (a.rows != a.cols)
        return detail::shape_violation(name, stage);
    if (a.rows == 0)
        return detail::make(name, "matrix units", stage, Bound::Upper, 0.0, 0.0);
    if (!(detail::all_finite(a)))
        return detail::nonfinite_violation(name, stage);
    const auto ev = la::symmetric_eigenvalues(a);
    const double neg = std::max(0.0, -detail::to_double(ev.front()));
    const double tol = arithmetic_tolerance<T>(a.rows, detail::to_double(la::frobenius(a)), safety);
    return detail::make(name, "matrix units", stage, Bound::Upper, neg, tol);
}

/// A ≻ 0 (symmetric positive-definite). Value: λ_min(A); lower bound
/// safety·n·ε_T·‖A‖_F — an eigenvalue indistinguishable from roundoff is not
/// "positive" for a Cholesky solve in T.
template <math::Scalar T>
[[nodiscard]] InvariantResult
check_spd(const DynMat<T>& a, Stage stage, std::string_view name = "spd", double safety = kDefaultSafety) {
    if (a.rows != a.cols || a.rows == 0)
        return detail::shape_violation(name, stage);
    if (!(detail::all_finite(a)))
        return detail::nonfinite_violation(name, stage, Bound::Lower);
    const auto ev = la::symmetric_eigenvalues(a);
    const double tol = arithmetic_tolerance<T>(a.rows, detail::to_double(la::frobenius(a)), safety);
    return detail::make(name, "matrix units", stage, Bound::Lower, detail::to_double(ev.front()), tol);
}

/// Loewner order A ⪯ B (e.g. P⁺ ⪯ P⁻: an update never adds uncertainty).
/// Value: max(0, λ_max(A − B)); threshold: safety·n·ε_T·max(‖A‖_F, ‖B‖_F).
template <math::Scalar T>
[[nodiscard]] InvariantResult check_loewner_le(const DynMat<T>& a,
                                               const DynMat<T>& b,
                                               Stage stage,
                                               std::string_view name = "loewner_le",
                                               double safety = kDefaultSafety) {
    if (!la::same_shape(a, b) || a.rows != a.cols)
        return detail::shape_violation(name, stage);
    if (a.rows == 0)
        return detail::make(name, "matrix units", stage, Bound::Upper, 0.0, 0.0);
    if (!(detail::all_finite(a) && detail::all_finite(b)))
        return detail::nonfinite_violation(name, stage);
    const auto ev = la::symmetric_eigenvalues(la::sub(a, b));
    const double over = std::max(0.0, detail::to_double(ev.back()));
    const double scale = std::max(detail::to_double(la::frobenius(a)), detail::to_double(la::frobenius(b)));
    return detail::make(
        name, "matrix units", stage, Bound::Upper, over, arithmetic_tolerance<T>(a.rows, scale, safety));
}

/// Condition number κ₂ = λ_max / λ_min of a symmetric positive matrix (e.g. the
/// triangulation normal matrix, #265). Reported, never failed: it is the context
/// a tolerance violation is read against. +∞ when λ_min ≤ 0.
template <math::Scalar T>
[[nodiscard]] InvariantResult
report_condition_number(const DynMat<T>& a, Stage stage, std::string_view name = "condition_number") {
    if (a.rows != a.cols || a.rows == 0)
        return detail::shape_violation(name, stage);
    if (!(detail::all_finite(a)))
        return detail::nonfinite_violation(name, stage, Bound::Report);
    const auto ev = la::symmetric_eigenvalues(a);
    const double lmin = detail::to_double(ev.front()), lmax = detail::to_double(ev.back());
    const double kappa = lmin > 0.0 ? lmax / lmin : detail::kInf;
    return detail::make(name, "dimensionless", stage, Bound::Report, kappa, 0.0);
}

// ── Orthogonality, annihilation, rank ───────────────────────────────────────

/// Rows of M orthonormal: M Mᵀ = I (e.g. Nᵀ of the MSCKF null space, so the
/// projected noise stays σ²·I). Value: max |M Mᵀ − I|; threshold:
/// safety·cols·ε_T.
template <math::Scalar T>
[[nodiscard]] InvariantResult check_orthonormal_rows(const DynMat<T>& m,
                                                     Stage stage,
                                                     std::string_view name = "orthonormal_rows",
                                                     double safety = kDefaultSafety) {
    if (!(detail::all_finite(m)))
        return detail::nonfinite_violation(name, stage);
    const DynMat<T> g = msckf::mul(m, msckf::transpose(m));
    T worst{0};
    for (std::size_t i = 0; i < g.rows; ++i)
        for (std::size_t j = 0; j < g.cols; ++j)
            worst = std::max(worst, la::abs_(g(i, j) - (i == j ? T{1} : T{0})));
    return detail::make(name,
                        "dimensionless",
                        stage,
                        Bound::Upper,
                        detail::to_double(worst),
                        arithmetic_tolerance<T>(m.cols, 1.0, safety));
}

/// L·R = 0 (e.g. NᵀH_f = 0: the feature is marginalized; or H_x·N = 0: the
/// measurement sees nothing along the unobservable directions). Value:
/// ‖L R‖_max in the product's units; threshold: safety·k·ε_T·‖L‖_F·‖R‖_F.
template <math::Scalar T>
[[nodiscard]] InvariantResult check_annihilates(const DynMat<T>& l,
                                                const DynMat<T>& r,
                                                Stage stage,
                                                std::string_view name = "annihilates",
                                                double safety = kDefaultSafety) {
    if (l.cols != r.rows)
        return detail::shape_violation(name, stage);
    if (!(detail::all_finite(l) && detail::all_finite(r)))
        return detail::nonfinite_violation(name, stage);
    const double v = detail::to_double(la::max_abs(msckf::mul(l, r)));
    const double scale = detail::to_double(la::frobenius(l)) * detail::to_double(la::frobenius(r));
    return detail::make(name, "product units", stage, Bound::Upper, v, arithmetic_tolerance<T>(l.cols, scale, safety));
}

/// Numerical rank equals `expected` (e.g. 2m − 3 for the projected MSCKF system).
/// Value: the rank (singular values above safety·max(m,n)·ε_T·σ_max).
template <math::Scalar T>
[[nodiscard]] InvariantResult check_rank(const DynMat<T>& a,
                                         std::size_t expected,
                                         Stage stage,
                                         std::string_view name = "rank",
                                         double safety = kDefaultSafety) {
    if (!(detail::all_finite(a)))
        return detail::nonfinite_violation(name, stage, Bound::Band);
    const double r = static_cast<double>(la::numerical_rank(a, safety));
    const double e = static_cast<double>(expected);
    return detail::make(name, "count", stage, Bound::Band, r, e, e);
}

/// An integer bookkeeping identity: `actual == expected` (state dimension
/// 15 + calib + 6·clones, +6 per clone, 2m − 3 projected rows, …).
[[nodiscard]] inline InvariantResult
check_dimension(std::size_t actual, std::size_t expected, Stage stage, std::string_view name = "dimension") {
    const double e = static_cast<double>(expected);
    return detail::make(name, "count", stage, Bound::Band, static_cast<double>(actual), e, e);
}

// ── Kalman gain solve ───────────────────────────────────────────────────────

/// K S = B, i.e. K = B S⁻¹ with B = P Hᵀ (the gain is never formed through an
/// explicit inverse). Value: the normwise backward error
/// ‖K S − B‖_F / (‖K‖_F‖S‖_F + ‖B‖_F) — dimensionless and independent of κ(S),
/// so a stable solve passes in any type however ill-conditioned S is; threshold:
/// safety·k·ε_T with k = dim S.
template <math::Scalar T>
[[nodiscard]] InvariantResult check_solve_residual(const DynMat<T>& k,
                                                   const DynMat<T>& s,
                                                   const DynMat<T>& b,
                                                   Stage stage,
                                                   std::string_view name = "gain_solve_residual",
                                                   double safety = kDefaultSafety) {
    if (k.cols != s.rows || s.rows != s.cols || b.rows != k.rows || b.cols != s.cols)
        return detail::shape_violation(name, stage);
    if (!(detail::all_finite(k) && detail::all_finite(s) && detail::all_finite(b)))
        return detail::nonfinite_violation(name, stage);
    const T num = la::frobenius(la::sub(msckf::mul(k, s), b));
    const T den = la::frobenius(k) * la::frobenius(s) + la::frobenius(b);
    const double v = den > T{0} ? detail::to_double(num / den) : detail::to_double(num);
    return detail::make(name, "dimensionless", stage, Bound::Upper, v, arithmetic_tolerance<T>(s.rows, 1.0, safety));
}

// ── Covariance block identities ─────────────────────────────────────────────

/// P_next = Φ P Φᵀ + Q_d (S2). Value: ‖P_next − (ΦPΦᵀ + Q_d)‖_F relative to
/// ‖Φ‖_F²‖P‖_F + ‖Q_d‖_F; threshold: safety·n·ε_T.
template <math::Scalar T>
[[nodiscard]] InvariantResult check_covariance_propagation(const DynMat<T>& p,
                                                           const DynMat<T>& phi,
                                                           const DynMat<T>& qd,
                                                           const DynMat<T>& p_next,
                                                           Stage stage = Stage::S2_propagation,
                                                           std::string_view name = "propagation_identity",
                                                           double safety = kDefaultSafety) {
    const std::size_t n = p.rows;
    if (p.cols != n || phi.rows != n || phi.cols != n || !la::same_shape(qd, p) || !la::same_shape(p_next, p))
        return detail::shape_violation(name, stage);
    if (!(detail::all_finite(p) && detail::all_finite(phi) && detail::all_finite(qd) && detail::all_finite(p_next)))
        return detail::nonfinite_violation(name, stage);
    const DynMat<T> pred = msckf::add(msckf::mul(msckf::mul(phi, p), msckf::transpose(phi)), qd);
    const T fphi = la::frobenius(phi);
    const T den = fphi * fphi * la::frobenius(p) + la::frobenius(qd);
    const T num = la::frobenius(la::sub(p_next, pred));
    const double v = den > T{0} ? detail::to_double(num / den) : detail::to_double(num);
    return detail::make(name, "dimensionless", stage, Bound::Upper, v, arithmetic_tolerance<T>(n, 1.0, safety));
}

/// Clone augmentation (S3): with J the (c×n) Jacobian of the new clone w.r.t. the
/// state, P_aug = [[P, P Jᵀ], [J P, J P Jᵀ]]. Two results: the dimension grew by
/// exactly c (6 for a pose clone), and the block identity holds to
/// safety·n·ε_T relative to ‖P‖_F(1 + ‖J‖_F)².
template <math::Scalar T>
[[nodiscard]] std::vector<InvariantResult> check_augmentation(const DynMat<T>& p,
                                                              const DynMat<T>& j,
                                                              const DynMat<T>& p_aug,
                                                              Stage stage = Stage::S3_augmentation,
                                                              double safety = kDefaultSafety) {
    const std::size_t n = p.rows, c = j.rows;
    std::vector<InvariantResult> out;
    out.push_back(check_dimension(p_aug.rows, n + c, stage, "augmentation.dimension"));
    if (p.cols != n || j.cols != n || p_aug.rows != n + c || p_aug.cols != n + c) {
        out.push_back(detail::shape_violation("augmentation.blocks", stage));
        return out;
    }
    if (!(detail::all_finite(p) && detail::all_finite(j) && detail::all_finite(p_aug))) {
        out.push_back(detail::nonfinite_violation("augmentation.blocks", stage));
        return out;
    }
    const DynMat<T> jp = msckf::mul(j, p);
    const DynMat<T> jpjt = msckf::mul(jp, msckf::transpose(j));
    DynMat<T> expect(n + c, n + c);
    for (std::size_t r = 0; r < n; ++r)
        for (std::size_t q = 0; q < n; ++q)
            expect(r, q) = p(r, q);
    for (std::size_t r = 0; r < c; ++r)
        for (std::size_t q = 0; q < n; ++q) {
            expect(n + r, q) = jp(r, q);
            expect(q, n + r) = jp(r, q);  // P Jᵀ = (J P)ᵀ for symmetric P
        }
    for (std::size_t r = 0; r < c; ++r)
        for (std::size_t q = 0; q < c; ++q)
            expect(n + r, n + q) = jpjt(r, q);
    const T one_j = T{1} + la::frobenius(j);
    const T den = la::frobenius(p) * one_j * one_j;
    const T num = la::frobenius(la::sub(p_aug, expect));
    const double v = den > T{0} ? detail::to_double(num / den) : detail::to_double(num);
    out.push_back(detail::make(
        "augmentation.blocks", "dimensionless", stage, Bound::Upper, v, arithmetic_tolerance<T>(n, 1.0, safety)));
    return out;
}

/// Marginalization (S9): P_after equals the principal submatrix P_before[keep,
/// keep]. Two results: the dimension equals |keep|, and ‖P_after −
/// P_before[keep,keep]‖_F / ‖P_before‖_F ≤ safety·n·ε_T. (The dense filter's
/// gather is exact; the square-root filter re-triangularizes, hence a tolerance.)
template <math::Scalar T>
[[nodiscard]] std::vector<InvariantResult> check_principal_submatrix(const DynMat<T>& p_before,
                                                                     std::span<const std::size_t> keep,
                                                                     const DynMat<T>& p_after,
                                                                     Stage stage = Stage::S9_marginalization,
                                                                     double safety = kDefaultSafety) {
    std::vector<InvariantResult> out;
    const std::size_t k = keep.size();
    out.push_back(check_dimension(p_after.rows, k, stage, "marginalization.dimension"));
    bool ok = p_after.rows == k && p_after.cols == k && p_before.rows == p_before.cols;
    for (const std::size_t i : keep)
        ok = ok && i < p_before.rows;
    if (!ok) {
        out.push_back(detail::shape_violation("marginalization.principal_submatrix", stage));
        return out;
    }
    if (!(detail::all_finite(p_before) && detail::all_finite(p_after))) {
        out.push_back(detail::nonfinite_violation("marginalization.principal_submatrix", stage));
        return out;
    }
    T num{0};
    for (std::size_t a = 0; a < k; ++a)
        for (std::size_t b = 0; b < k; ++b) {
            const T d = p_after(a, b) - p_before(keep[a], keep[b]);
            num += d * d;
        }
    const T den = la::frobenius(p_before);
    const T res = la::sqrt_(num);
    const double v = den > T{0} ? detail::to_double(res / den) : detail::to_double(res);
    out.push_back(detail::make("marginalization.principal_submatrix",
                               "dimensionless",
                               stage,
                               Bound::Upper,
                               v,
                               arithmetic_tolerance<T>(p_before.rows, 1.0, safety)));
    return out;
}

/// QR measurement compression (S6c): [H | r] → Q₁ᵀ[H | r] with Q₁ an orthonormal
/// basis of range(H). Compression is lossless for the update: the normal
/// equations are preserved exactly — Hᵀ H = H_cᵀ H_c and Hᵀ r = H_cᵀ r_c — while
/// the residual can only shrink, ‖r_c‖ ≤ ‖r‖ (the dropped part of r is orthogonal
/// to range(H) and carries no information about δx; ‖r_c‖ = ‖r‖ iff r ∈ range(H)).
/// Three results, each relative and held to safety·rows·ε_T.
template <math::Scalar T>
[[nodiscard]] std::vector<InvariantResult> check_compression(const DynMat<T>& h,
                                                             std::span<const T> r,
                                                             const DynMat<T>& h_c,
                                                             std::span<const T> r_c,
                                                             Stage stage = Stage::S6c_compression,
                                                             double safety = kDefaultSafety) {
    std::vector<InvariantResult> out;
    constexpr std::string_view kNames[] = {
        "compression.normal_matrix", "compression.normal_rhs", "compression.residual_norm_nonincreasing"};
    if (h.cols != h_c.cols || r.size() != h.rows || r_c.size() != h_c.rows) {
        for (const auto nm : kNames)
            out.push_back(detail::shape_violation(nm, stage));
        return out;
    }
    if (!(detail::all_finite(h) && detail::all_finite(h_c) && detail::all_finite(r) && detail::all_finite(r_c))) {
        for (const auto nm : kNames)
            out.push_back(detail::nonfinite_violation(nm, stage));
        return out;
    }
    auto as_col = [](std::span<const T> v) {
        DynMat<T> c(v.size(), 1);
        for (std::size_t i = 0; i < v.size(); ++i)
            c(i, 0) = v[i];
        return c;
    };
    const DynMat<T> rv = as_col(r), rcv = as_col(r_c);
    const DynMat<T> ht = msckf::transpose(h), hct = msckf::transpose(h_c);
    const double tol = arithmetic_tolerance<T>(h.rows, 1.0, safety);

    const T fh = la::frobenius(h), fr = la::frobenius(rv);
    const T d_hth = la::frobenius(la::sub(msckf::mul(ht, h), msckf::mul(hct, h_c)));
    const double v_hth = fh > T{0} ? detail::to_double(d_hth / (fh * fh)) : detail::to_double(d_hth);
    out.push_back(detail::make(kNames[0], "dimensionless", stage, Bound::Upper, v_hth, tol));

    const T d_htr = la::frobenius(la::sub(msckf::mul(ht, rv), msckf::mul(hct, rcv)));
    const T den_htr = fh * fr;
    const double v_htr = den_htr > T{0} ? detail::to_double(d_htr / den_htr) : detail::to_double(d_htr);
    out.push_back(detail::make(kNames[1], "dimensionless", stage, Bound::Upper, v_htr, tol));

    const T grow = la::frobenius(rcv) - fr;  // ≤ 0 when compression is correct
    const double v_grow =
        fr > T{0} ? std::max(0.0, detail::to_double(grow / fr)) : std::max(0.0, detail::to_double(grow));
    out.push_back(detail::make(kNames[2], "dimensionless", stage, Bound::Upper, v_grow, tol));
    return out;
}

// ── Unobservable subspace (the observability / over-confidence question) ─────

/// Φ maps the unobservable subspace onto itself: span(Φ N_k) ⊆ span(N_{k+1})
/// (S2; for a time-invariant basis pass N_next = N). Basis-free: value is
/// ‖(I − Q Qᵀ) Φ N_k‖_F / ‖Φ N_k‖_F with Q an orthonormal basis of N_{k+1};
/// threshold safety·n·ε_T. A Φ that leaks the yaw direction into the observable
/// subspace (the body-frame MSCKF linearized at the current estimate) fails.
template <math::Scalar T>
[[nodiscard]] InvariantResult check_subspace_preserved(const DynMat<T>& phi,
                                                       const DynMat<T>& n_k,
                                                       const DynMat<T>& n_next,
                                                       Stage stage = Stage::S2_propagation,
                                                       std::string_view name = "unobservable_subspace_preserved",
                                                       double safety = kDefaultSafety) {
    if (phi.cols != n_k.rows || phi.rows != n_next.rows)
        return detail::shape_violation(name, stage);
    if (!(detail::all_finite(phi) && detail::all_finite(n_k) && detail::all_finite(n_next)))
        return detail::nonfinite_violation(name, stage);
    const DynMat<T> pn = msckf::mul(phi, n_k);
    const DynMat<T> q = la::orthonormal_columns(n_next, safety);
    const DynMat<T> proj = msckf::mul(q, msckf::mul(msckf::transpose(q), pn));
    const T num = la::frobenius(la::sub(pn, proj));
    const T den = la::frobenius(pn);
    const double v = den > T{0} ? detail::to_double(num / den) : detail::to_double(num);
    return detail::make(name, "dimensionless", stage, Bound::Upper, v, arithmetic_tolerance<T>(phi.rows, 1.0, safety));
}

/// The update δx has no component along the unobservable directions N (S6e): a
/// consistent filter cannot gain information about global position or yaw.
/// Value: ‖Q Qᵀ δx‖ in δx's units (Q an orthonormal basis of N); threshold
/// safety·n·ε_T·‖δx‖. The headline check of the #212 over-confidence.
template <math::Scalar T>
[[nodiscard]] InvariantResult check_no_update_along(std::span<const T> dx,
                                                    const DynMat<T>& n,
                                                    Stage stage = Stage::S6e_ekf_update,
                                                    std::string_view name = "no_update_along_unobservable",
                                                    double safety = kDefaultSafety) {
    if (n.rows != dx.size())
        return detail::shape_violation(name, stage);
    if (!(detail::all_finite(dx) && detail::all_finite(n)))
        return detail::nonfinite_violation(name, stage);
    const DynMat<T> q = la::orthonormal_columns(n, safety);
    T along2{0}, dx2{0};
    for (std::size_t j = 0; j < q.cols; ++j) {
        T c{0};
        for (std::size_t i = 0; i < q.rows; ++i)
            c += q(i, j) * dx[i];
        along2 += c * c;
    }
    for (const T& v : dx)
        dx2 += v * v;
    const double tol = arithmetic_tolerance<T>(dx.size(), detail::to_double(la::sqrt_(dx2)), safety);
    return detail::make(name, "state units", stage, Bound::Upper, detail::to_double(la::sqrt_(along2)), tol);
}

}  // namespace branes::sdk::eval::inv

#endif  // BRANES_SDK_EVAL_INVARIANTS_MATRIX_CHECKS_HPP
