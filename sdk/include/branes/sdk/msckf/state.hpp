// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/state.hpp — the MSCKF state vector: the inertial
// navigation state (orientation, position, velocity, gyro/accel biases)
// plus a sliding window of cloned IMU poses (one per kept image), and the
// joint error-state covariance.
//
// Error-state layout (world-centric, R = R_world_imu, right perturbation
// R ← R·Exp(δθ)):
//   [ δθ(3) δp(3) δv(3) δbg(3) δba(3) | per clone: δθ_c(3) δp_c(3) ... ]
// so the covariance is (15 + 6·#clones) square.
//
// The covariance representation is a policy (`Cov`): `FullCovariance` carries
// the dense P, `SqrtCovariance` carries the Cholesky factor S (P = S·Sᵀ). The
// mean, clone window, and feature management are identical either way — only
// the covariance algebra differs, and it lives behind `cov`.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STATE_HPP
#define BRANES_SDK_MSCKF_STATE_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/covariance.hpp>
#include <branes/sdk/msckf/dense.hpp>

#include <cstddef>
#include <vector>

namespace branes::sdk::msckf {

template <math::Scalar T, CovarianceModel<T> Cov = FullCovariance<T>>
struct State {
    using SO3 = math::lie::SO3<T>;
    using Vec3 = math::lie::detail::Vec<T, 3>;

    // Error-state offsets within the IMU block.
    static constexpr std::size_t kTheta = 0;
    static constexpr std::size_t kPos = 3;
    static constexpr std::size_t kVel = 6;
    static constexpr std::size_t kBg = 9;
    static constexpr std::size_t kBa = 12;
    static constexpr std::size_t kImuDim = 15;
    static constexpr std::size_t kCloneDim = 6;

    /// A cloned IMU pose (taken at an image time), kept in the window.
    /// Timestamps are wall-clock seconds — intentionally `double` (not the
    /// state scalar T), matching VioBackend / ImuMeasurement.
    ///
    /// `R_fej`/`p_fej` are the First-Estimates-Jacobian linearization point: the
    /// pose at the moment this clone was created, frozen thereafter. EKF updates
    /// correct `R`/`p` (the mean) but never the `_fej` anchors, so the camera
    /// measurement Jacobians can be evaluated at a fixed point — preserving the
    /// VIO observability subspace (global position + yaw) and stopping the filter
    /// from gaining spurious information in it (#280).
    struct Clone {
        SO3 R{};
        Vec3 p{};
        double timestamp = 0.0;
        SO3 R_fej{};
        Vec3 p_fej{};
    };

    // Inertial navigation state (world frame).
    SO3 R{};    ///< world ← imu
    Vec3 p{};   ///< world position
    Vec3 v{};   ///< world velocity
    Vec3 bg{};  ///< gyro bias
    Vec3 ba{};  ///< accel bias
    double timestamp = 0.0;

    // First-Estimates-Jacobian linearization point for the IMU propagation
    // Jacobian Φ: the rotation and biases as of the last propagation's prior.
    // `Propagator` refreshes them to the propagated mean each step but `ekf_update`
    // never corrects them, so the one propagation step after each measurement
    // update keeps the *pre-update* linearization — matching the frozen-clone
    // measurement Jacobians so Φ and H share one point (#280). `sync_fej()` resets
    // them to the current mean (called once the IMU state is initialized).
    SO3 R_fej{};
    Vec3 bg_fej{};
    Vec3 ba_fej{};
    void sync_fej() {
        R_fej = R;
        bg_fej = bg;
        ba_fej = ba;
    }

    std::vector<Clone> clones;
    Cov cov;  ///< joint error-state covariance (representation-policy), dim() square

    /// Construct with an initial IMU covariance (15×15 by default).
    explicit State(T initial_sigma = T{1}) : cov(initial_sigma, kImuDim) {}

    [[nodiscard]] std::size_t num_clones() const {
        return clones.size();
    }
    [[nodiscard]] std::size_t dim() const {
        return kImuDim + kCloneDim * clones.size();
    }
    /// Error-state offset of clone `i`'s δθ block.
    [[nodiscard]] std::size_t clone_offset(std::size_t i) const {
        return kImuDim + kCloneDim * i;
    }

    /// Dense error-state covariance P for inspection / PSD checks. For the
    /// full representation this is the stored matrix; for the square-root
    /// representation it is S·Sᵀ.
    [[nodiscard]] DynMat<T> covariance() const {
        return cov.covariance();
    }
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_STATE_HPP
