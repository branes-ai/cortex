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
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STATE_HPP
#define BRANES_SDK_MSCKF_STATE_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/dense.hpp>

#include <cstddef>
#include <vector>

namespace branes::sdk::msckf {

template <math::Scalar T>
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
    struct Clone {
        SO3 R{};
        Vec3 p{};
        double timestamp = 0.0;
    };

    // Inertial navigation state (world frame).
    SO3 R{};    ///< world ← imu
    Vec3 p{};   ///< world position
    Vec3 v{};   ///< world velocity
    Vec3 bg{};  ///< gyro bias
    Vec3 ba{};  ///< accel bias
    double timestamp = 0.0;

    std::vector<Clone> clones;
    DynMat<T> P;  ///< joint error-state covariance, dim() square

    /// Construct with an initial IMU covariance (15×15 by default).
    explicit State(T initial_sigma = T{1}) : P(kImuDim, kImuDim) {
        for (std::size_t i = 0; i < kImuDim; ++i)
            P(i, i) = initial_sigma * initial_sigma;
    }

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
};

}  // namespace branes::sdk::msckf

#endif  // BRANES_SDK_MSCKF_STATE_HPP
