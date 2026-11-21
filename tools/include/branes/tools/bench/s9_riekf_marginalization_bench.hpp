// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s9_riekf_marginalization_bench.hpp — S9_marginalization
// stage bench for the Right-Invariant EKF backend (issue #612).
//
// Oracle: principal submatrix.  After marginalize_clone(idx) the surviving
// covariance must equal the rows/cols of P_pre that correspond to the kept
// dimensions — identical oracle to the body-frame S9 bench.
//
// Stage under test: MsckfInvariantBackend<T>::marginalize_clone(idx).
// Construction: seed_covariance(I) + N × augment_clone() builds a valid
// backend with N clones without needing any private test hooks.

#pragma once

#include <branes/math/arithmetic.hpp>
#include <branes/sdk/msckf/covariance.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/msckf_invariant_backend.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

namespace branes::tools::bench {

using sdk::msckf::DynMat;

// ── oracle ────────────────────────────────────────────────────────────────────

/// Return the principal submatrix of P after removing the 6-dim block at [off, off+6).
template <math::Scalar T>
DynMat<T> principal_submatrix(const DynMat<T>& P, std::size_t off) {
    constexpr std::size_t kCD = 6;
    assert(P.rows == P.cols);
    const std::size_t d = P.rows;
    const std::size_t d2 = d - kCD;

    std::vector<std::size_t> keep;
    keep.reserve(d2);
    for (std::size_t i = 0; i < d; ++i)
        if (i < off || i >= off + kCD)
            keep.push_back(i);

    DynMat<T> Q(d2, d2);
    for (std::size_t r = 0; r < d2; ++r)
        for (std::size_t c = 0; c < d2; ++c)
            Q(r, c) = P(keep[r], keep[c]);
    return Q;
}

/// Max |a(i,j) - b(i,j)| over all entries.
template <math::Scalar T>
double max_abs_diff(const DynMat<T>& a, const DynMat<T>& b) {
    assert(a.rows == b.rows && a.cols == b.cols);
    double mx = 0.0;
    for (std::size_t i = 0; i < a.rows * a.cols; ++i)
        mx = std::max(mx, std::abs(static_cast<double>(a.d[i] - b.d[i])));
    return mx;
}

/// Max |P(i,j) - P(j,i)|.
template <math::Scalar T>
double symmetry_error(const DynMat<T>& P) {
    assert(P.rows == P.cols);
    double mx = 0.0;
    for (std::size_t r = 0; r < P.rows; ++r)
        for (std::size_t c = r + 1; c < P.cols; ++c)
            mx = std::max(mx, std::abs(static_cast<double>(P(r, c) - P(c, r))));
    return mx;
}

// ── bench ─────────────────────────────────────────────────────────────────────

template <math::Scalar T>
struct S9RiEkfMargBench {
    static constexpr std::size_t kNavDim = 15;
    static constexpr std::size_t kCloneDim = 6;

    using Backend = sdk::msckf::MsckfInvariantBackend<T>;

    struct Result {
        bool dim_ok = false;
        bool psub_ok = false;
        bool sym_ok = false;
        double psub_err = 0.0;
        double sym_err = 0.0;
        bool all_ok() const {
            return dim_ok && psub_ok && sym_ok;
        }
    };

    /// Build a backend with `n_clones` clones, snapshot P, drop `drop_idx`,
    /// check the three invariants.  `seed` varies the random nav rotation so
    /// clone poses differ across fixtures.
    static Result run(std::size_t n_clones, std::size_t drop_idx, unsigned seed = 42) {
        // tolerance: double ~1e-10, float ~1e-4
        const double tol = (sizeof(T) >= 8) ? 1e-10 : 1e-4;

        // ── construct backend ────────────────────────────────────────────────
        typename Backend::Config cfg;
        cfg.max_clones = n_clones + 1;  // prevent auto-marginalization
        Backend be(cfg);

        // seed nav covariance = I₁₅ (seed_covariance takes G s.t. P = G·Gᵀ)
        DynMat<T> G = DynMat<T>::identity(kNavDim);
        be.seed_covariance(G);

        // give nav a non-trivial rotation so clone poses are real SE(3) elements
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> ud(-0.3, 0.3);
        const typename Backend::Vec3 zero{};
        const auto small_angle = [&]() -> typename Backend::Vec3 {
            return {{static_cast<T>(ud(rng)), static_cast<T>(ud(rng)), static_cast<T>(ud(rng))}};
        };
        // set nav to identity SE₂(3) with zero biases
        be.set_nav(typename Backend::SE23{}, zero, zero, 0.0);

        // augment clones — vary nav rotation between augmentations so each
        // clone's pose (and therefore its covariance block) is distinguishable.
        // This lets the principal-submatrix oracle detect incorrect clone-index
        // handling (e.g. always dropping clone 0 regardless of drop_idx).
        for (std::size_t i = 0; i < n_clones; ++i) {
            const typename Backend::Vec3 phi = small_angle();
            const typename Backend::SE23 X_i(typename Backend::SO3::exp(phi), zero, zero);
            be.set_nav(X_i, zero, zero, static_cast<double>(i));
            be.augment_clone();
        }

        // ── snapshot pre-marginalization covariance ──────────────────────────
        const DynMat<T> P_pre = be.covariance();
        const std::size_t d_pre = P_pre.rows;
        const std::size_t d_post_expected = d_pre - kCloneDim;

        // ── run the stage under test ─────────────────────────────────────────
        be.marginalize_clone(drop_idx);
        const DynMat<T> P_post = be.covariance();

        // ── invariant 1: dimension ───────────────────────────────────────────
        Result res;
        res.dim_ok =
            (P_post.rows == d_post_expected) && (P_post.cols == d_post_expected) && (be.num_clones() == n_clones - 1);

        // ── invariant 2: principal submatrix ─────────────────────────────────
        const std::size_t off = kNavDim + kCloneDim * drop_idx;
        const DynMat<T> Q = principal_submatrix(P_pre, off);
        res.psub_err = max_abs_diff(P_post, Q);
        res.psub_ok = (res.psub_err < tol);

        // ── invariant 3: symmetry ────────────────────────────────────────────
        res.sym_err = symmetry_error(P_post);
        res.sym_ok = (res.sym_err < tol);

        return res;
    }
};

}  // namespace branes::tools::bench
