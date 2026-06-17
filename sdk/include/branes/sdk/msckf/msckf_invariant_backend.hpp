// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/msckf_invariant_backend.hpp — the Right-Invariant EKF MSCKF
// backend (issue #347, Phase B). Wires the three validated pieces into one filter:
//   • SE₂(3) state + biases               (invariant_propagator.hpp)
//   • invariant IMU propagation Φ          (invariant_propagator.hpp)
//   • invariant camera measurement update  (invariant_update.hpp)
// over a SINGLE joint right-invariant covariance [nav(15) ⊕ 6·#clones].
//
// The point of the whole R-IEKF programme: the measurement Jacobian and the
// propagation Φ both annihilate / preserve the 4-DoF unobservable gauge (global
// translation + gravity-yaw) BY CONSTRUCTION, with no FEJ. So a camera update can
// never inject information along the yaw gauge — the structural cure for the #212
// over-confidence that no first-estimates variant could fix. This class is the
// world-frame mirror of the body-frame State + StateHelper + CameraUpdater stack;
// it is built alongside (not replacing) that stack until the EuRoC verdict clears.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_MSCKF_INVARIANT_BACKEND_HPP
#define BRANES_SDK_MSCKF_MSCKF_INVARIANT_BACKEND_HPP

#include <branes/sdk/msckf/covariance.hpp>
#include <branes/sdk/msckf/invariant_propagator.hpp>
#include <branes/sdk/msckf/invariant_update.hpp>

#include <array>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

namespace branes::sdk::msckf {

/// A feature track for the invariant backend: the per-clone normalized observations.
template <math::Scalar T>
using InvariantTrack = std::vector<InvariantObs<T>>;

/// Right-Invariant MSCKF backend. `Cov` is a covariance representation policy
/// (FullCovariance by default; the sqrt form satisfies the same interface).
template <math::Scalar T, class Cov = FullCovariance<T>>
class MsckfInvariantBackend {
public:
    using Nav = InvariantNavState<T>;
    using Clone = InvariantClone<T>;
    using Calib = InvariantCalib<T>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using SE23 = math::lie::SE23<T>;
    using SO3 = math::lie::SO3<T>;

    struct Config {
        ImuNoise<T> noise{};
        Vec3 gravity{{T{0}, T{0}, T{-981} / T{100}}};
        std::size_t max_clones = 11;         ///< sliding-window size
        T initial_sigma = T{1} / T{10};      ///< nav prior σ
        T normalized_sigma = T{1} / T{100};  ///< per-pixel (normalized) measurement σ
        /// χ² innovation gate: reject a track when γ = rᵀS⁻¹r exceeds
        /// `chi2_per_dof · (#projected rows)`. Loose default, matches the body path.
        T chi2_per_dof = T{5};
        bool enable_gating = true;
        SO3 R_imu_cam{};   ///< camera↔IMU extrinsic rotation
        Vec3 p_imu_cam{};  ///< camera origin in the IMU frame
    };

    explicit MsckfInvariantBackend(const Config& cfg = {})
        : cfg_(cfg), prop_(cfg.noise, cfg_.gravity), cov_(cfg.initial_sigma, Nav::kDim) {}

    // ── accessors ────────────────────────────────────────────────────
    [[nodiscard]] const Nav& nav() const noexcept {
        return nav_;
    }
    [[nodiscard]] const std::vector<Clone>& clones() const noexcept {
        return clones_;
    }
    [[nodiscard]] const std::vector<Calib>& calib() const noexcept {
        return calib_;
    }
    [[nodiscard]] std::size_t num_clones() const noexcept {
        return clones_.size();
    }
    [[nodiscard]] std::size_t calib_dim() const noexcept {
        return kCalibBlock * calib_.size();
    }
    [[nodiscard]] std::size_t dim() const noexcept {
        return Nav::kDim + calib_dim() + kCloneDim * clones_.size();
    }
    [[nodiscard]] std::size_t calib_offset(std::size_t j) const noexcept {
        return Nav::kDim + kCalibBlock * j;
    }
    [[nodiscard]] std::size_t clone_offset(std::size_t i) const noexcept {
        return Nav::kDim + calib_dim() + kCloneDim * i;
    }
    [[nodiscard]] DynMat<T> covariance() const {
        return cov_.covariance();
    }
    void set_nav(const SE23& X, const Vec3& bg, const Vec3& ba, double t = 0.0) {
        nav_.X = X;
        nav_.bg = bg;
        nav_.ba = ba;
        nav_.timestamp = t;
    }

    /// Turn on online extrinsic calibration: append a calibration block.
    void enable_calibration(std::vector<Calib> init, T rot_sigma, T trans_sigma) {
        if (!clones_.empty())
            throw std::logic_error("enable_calibration must precede any clone");
        if (!calib_.empty())
            throw std::logic_error("enable_calibration: calibration already enabled");
        const std::size_t add = kCalibBlock * init.size();
        if (add == 0)
            return;
        const std::size_t d0 = cov_.dim();  // 15
        DynMat<T> G(d0 + add, d0);
        for (std::size_t i = 0; i < d0; ++i)
            G(i, i) = T{1};
        cov_.transform(G);
        DynMat<T> Iden(d0 + add, d0 + add);
        for (std::size_t i = 0; i < d0 + add; ++i)
            Iden(i, i) = T{1};
        std::vector<NoiseTerm<T>> q;
        q.reserve(add);
        for (std::size_t j = 0; j < init.size(); ++j) {
            const std::size_t off = Nav::kDim + kCalibBlock * j;
            for (std::size_t a = 0; a < 3; ++a)
                q.push_back({off + a, rot_sigma * rot_sigma});
            for (std::size_t a = 0; a < 3; ++a)
                q.push_back({off + 3 + a, trans_sigma * trans_sigma});
        }
        // To preserve positive semidefiniteness and comply with the generic CovarianceModel
        // interface policy (supporting both FullCovariance and SqrtCovariance), we inject
        // the initial calibration priors as process noise via `predict()` with identity.
        cov_.predict(Iden, q);
        calib_ = std::move(init);
    }

    // ── filter operations ────────────────────────────────────────────
    /// Propagate the live nav state and the joint covariance over one IMU sample.
    /// The clone block is static under propagation (cross-covariance via the joint
    /// Φ = blkdiag(Φ_nav, I)); only the nav rows/cols evolve.
    void propagate(const Vec3& gyro, const Vec3& accel, T dt) {
        if (!(dt > T{0}))
            return;
        const std::size_t d = dim();
        // Joint Φ: nav block from the invariant propagator, clones identity.
        DynMat<T> F = DynMat<T>::identity(d);
        const DynMat<T> Fn = prop_.phi(nav_.X.rotation(), nav_.X.velocity(), nav_.X.position(), dt);
        for (std::size_t i = 0; i < Nav::kDim; ++i)
            for (std::size_t j = 0; j < Nav::kDim; ++j)
                F(i, j) = Fn(i, j);
        const std::array<NoiseTerm<T>, 12> q = nav_process_noise(dt);
        cov_.predict(F, std::span<const NoiseTerm<T>>{q});
        prop_.propagate_mean(nav_, gyro, accel, dt);  // joint covariance handled above
        nav_.timestamp += static_cast<double>(dt);
    }

    /// Append a clone of the current nav pose to the window. The clone shares the
    /// nav error at clone time: δθ_c = δθ_nav, δρ_c = δp_nav.
    void augment_clone() {
        const std::size_t d = dim();
        DynMat<T> G(d + kCloneDim, d);
        for (std::size_t i = 0; i < d; ++i)
            G(i, i) = T{1};
        for (std::size_t i = 0; i < 3; ++i) {
            G(d + i, Nav::kTheta + i) = T{1};    // clone δθ_c = δθ
            G(d + 3 + i, Nav::kPos + i) = T{1};  // clone δρ_c = δp
        }
        cov_.transform(G);
        clones_.push_back({nav_.X.rotation(), nav_.X.position()});
        if (clones_.size() > cfg_.max_clones)
            marginalize_clone(0);  // drop the oldest
    }

    /// Drop clone `idx`: delete its 6 joint rows/cols and erase it.
    void marginalize_clone(std::size_t idx) {
        if (idx >= clones_.size())
            return;
        const std::size_t d = dim();
        const std::size_t off = clone_offset(idx);
        std::vector<std::size_t> keep;
        keep.reserve(d - kCloneDim);
        for (std::size_t i = 0; i < d; ++i)
            if (i < off || i >= off + kCloneDim)
                keep.push_back(i);
        cov_.marginalize(keep);
        clones_.erase(clones_.begin() + static_cast<std::ptrdiff_t>(idx));
    }

    /// Process one feature track over the current window: triangulate, build the
    /// invariant measurement on the joint state (nav columns zero — the feature
    /// constrains only the clones; the nav is corrected through cross-covariance),
    /// marginalize the feature, EKF-update, and retract nav + clones in the
    /// right-invariant box-plus. Returns true iff the state was updated.
    bool update(const InvariantTrack<T>& obs) {
        if (obs.size() < 2 || clones_.empty())
            return false;
        // Keep fixed projection extrinsics separate from "estimated calibration columns" fallback.
        // If calib_ is empty (calibration is disabled), we fall back to static non-estimable cfg_ parameters.
        const auto& active_cal = calib_.empty() ? std::vector<Calib>{{cfg_.R_imu_cam, cfg_.p_imu_cam}} : calib_;
        const bool estimate_cal = !calib_.empty();

        const auto tri = triangulate_invariant<T>(clones_, obs, active_cal);
        if (!tri.ok)
            return false;
        const auto M = build_invariant_measurement<T>(clones_, obs, tri.p_f, active_cal, estimate_cal);
        if (!M.ok)
            return false;

        // Scatter the clone-only H_x (2m × nc_calib) into the joint H (2m × dim),
        // placing each clone/calib block at its joint offset; nav columns stay zero.
        const std::size_t d = dim();
        const std::size_t nc6 = kCloneDim * clones_.size();
        const std::size_t n_calib = kCalibBlock * calib_.size();
        const std::size_t nc_calib = nc6 + n_calib;
        std::vector<T> Hj(M.rows2 * d, T{0});
        for (std::size_t row = 0; row < M.rows2; ++row) {
            for (std::size_t c = 0; c < clones_.size(); ++c)
                for (std::size_t k = 0; k < kCloneDim; ++k)
                    Hj[row * d + clone_offset(c) + k] = M.H_x[row * nc_calib + kCloneDim * c + k];
            for (std::size_t j = 0; j < calib_.size(); ++j)
                for (std::size_t k = 0; k < kCalibBlock; ++k)
                    Hj[row * d + calib_offset(j) + k] = M.H_x[row * nc_calib + nc6 + kCalibBlock * j + k];
        }

        const auto proj = features::msckf_left_nullspace_project<T>(M.H_f, Hj, M.r, M.rows2, d);
        if (proj.rows == 0)
            return false;
        DynMat<T> H(proj.rows, d);
        for (std::size_t i = 0; i < proj.rows; ++i)
            for (std::size_t j = 0; j < d; ++j)
                H(i, j) = proj.H_x[i * d + j];
        const T r2 = cfg_.normalized_sigma * cfg_.normalized_sigma;
        // χ² innovation gate: a single bad/low-parallax track can otherwise drive a
        // large wrong correction. γ = rᵀS⁻¹r; reject if ill-conditioned or too large.
        if (cfg_.enable_gating) {
            T gamma = T{0};
            const bool ok = cov_.mahalanobis(H, std::span<const T>{proj.r}, r2, gamma);
            if (!ok || gamma > cfg_.chi2_per_dof * static_cast<T>(proj.rows))
                return false;
        }
        const std::vector<T> R_diag(proj.rows, r2);
        const std::vector<T> dx = cov_.update(H, std::span<const T>{proj.r}, std::span<const T>{R_diag});

        retract(dx);
        return true;
    }

    static constexpr std::size_t kCloneDim = 6;
    static constexpr std::size_t kCalibBlock = 6;

private:
    [[nodiscard]] std::array<NoiseTerm<T>, 12> nav_process_noise(T dt) const {
        std::array<NoiseTerm<T>, 12> q;
        auto fill = [&](std::size_t at, std::size_t off, T val) {
            for (std::size_t i = 0; i < 3; ++i)
                q[at + i] = NoiseTerm<T>{off + i, val};
        };
        fill(0, Nav::kTheta, cfg_.noise.gyro * cfg_.noise.gyro * dt);
        fill(3, Nav::kVel, cfg_.noise.accel * cfg_.noise.accel * dt);
        fill(6, Nav::kBg, cfg_.noise.gyro_bias * cfg_.noise.gyro_bias * dt);
        fill(9, Nav::kBa, cfg_.noise.accel_bias * cfg_.noise.accel_bias * dt);
        return q;
    }

    /// Box-plus the joint correction: nav on SE₂(3) (left/world-frame), biases
    /// additively (imperfect IEKF), each clone in the right-invariant retraction.
    void retract(const std::vector<T>& dx) {
        // Simplified Left-Invariant retraction (dR*p + rho) matching the propagator.
        const Vec3 dphi{{dx[Nav::kTheta], dx[Nav::kTheta + 1], dx[Nav::kTheta + 2]}};
        const Vec3 dnu{{dx[Nav::kVel], dx[Nav::kVel + 1], dx[Nav::kVel + 2]}};
        const Vec3 drho{{dx[Nav::kPos], dx[Nav::kPos + 1], dx[Nav::kPos + 2]}};
        const SO3 dR = SO3::exp(dphi);
        nav_.X = SE23(dR * nav_.X.rotation(), dR * nav_.X.velocity() + dnu, dR * nav_.X.position() + drho);
        nav_.bg = nav_.bg + Vec3{{dx[Nav::kBg], dx[Nav::kBg + 1], dx[Nav::kBg + 2]}};
        nav_.ba = nav_.ba + Vec3{{dx[Nav::kBa], dx[Nav::kBa + 1], dx[Nav::kBa + 2]}};
        if (!calib_.empty()) {
            for (std::size_t j = 0; j < calib_.size(); ++j) {
                const std::size_t off = calib_offset(j);
                const Vec3 dtheta{{dx[off], dx[off + 1], dx[off + 2]}};
                const Vec3 dp{{dx[off + 3], dx[off + 4], dx[off + 5]}};
                // Rotation retracts via right-multiplication semantics matching the Jacobian's perturbation convention
                calib_[j].R_imu_cam = calib_[j].R_imu_cam * SO3::exp(dtheta);
                calib_[j].p_imu_cam = calib_[j].p_imu_cam + dp;
            }
        }
        for (std::size_t c = 0; c < clones_.size(); ++c) {
            const std::size_t off = clone_offset(c);
            retract_invariant<T>(
                clones_[c], Vec3{{dx[off], dx[off + 1], dx[off + 2]}}, Vec3{{dx[off + 3], dx[off + 4], dx[off + 5]}});
        }
    }

    Config cfg_;
    InvariantPropagator<T> prop_;
    Nav nav_{};
    std::vector<Clone> clones_;
    std::vector<Calib> calib_;
    Cov cov_;
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_MSCKF_INVARIANT_BACKEND_HPP
