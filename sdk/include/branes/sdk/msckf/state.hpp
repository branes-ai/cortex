// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/state.hpp — the MSCKF state vector: the inertial
// navigation state (orientation, position, velocity, gyro/accel biases)
// plus a sliding window of cloned IMU poses (one per kept image), and the
// joint error-state covariance.
//
// Error-state layout (world-centric, R = R_world_imu, right perturbation
// R ← R·Exp(δθ)):
//   [ δθ(3) δp(3) δv(3) δbg(3) δba(3) | (opt) per cam: δθ_ic(3) δp_ic(3)
//     | per clone: δθ_c(3) δp_c(3) ... ]
// so the covariance is (15 + 6·#cams_calibrated + 6·#clones) square. The
// optional per-camera calibration block (S10 online extrinsics) sits right
// after the IMU and is absent unless `enable_calibration` was called.
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

#include <cassert>
#include <cstddef>
#include <stdexcept>
#include <utility>
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
    /// Per-camera online-calibration error-state block: extrinsic rotation
    /// δθ_ic(3) + translation δp_ic(3). Optional (S10) — present only when
    /// online extrinsic estimation is enabled (see `enable_calibration`).
    static constexpr std::size_t kCalibBlock = 6;

    /// A cloned IMU pose (taken at an image time), kept in the window.
    /// Timestamps are wall-clock seconds — intentionally `double` (not the
    /// state scalar T), matching VioBackend / ImuMeasurement.
    struct Clone {
        SO3 R{};
        Vec3 p{};
        double timestamp = 0.0;
    };

    /// One camera's estimated extrinsics (camera pose in the IMU frame) when
    /// online calibration is on — the in-state mirror of `CameraExtrinsics`.
    /// `R_imu_cam` rotates camera axes into the IMU frame.
    struct CalibState {
        SO3 R_imu_cam{};
        Vec3 p_imu_cam{};
    };

    // Inertial navigation state (world frame).
    SO3 R{};    ///< world ← imu
    Vec3 p{};   ///< world position
    Vec3 v{};   ///< world velocity
    Vec3 bg{};  ///< gyro bias
    Vec3 ba{};  ///< accel bias
    double timestamp = 0.0;

    /// Optional in-state camera↔IMU extrinsics (one per camera). Empty unless
    /// online calibration is enabled; when empty the layout and covariance are
    /// bit-for-bit the fixed-calibration filter (the S10 term is inert).
    std::vector<CalibState> calib;
    std::vector<Clone> clones;
    Cov cov;  ///< joint error-state covariance (representation-policy), dim() square

    /// Construct with an initial IMU covariance (15×15 by default).
    explicit State(T initial_sigma = T{1}) : cov(initial_sigma, kImuDim) {}

    [[nodiscard]] std::size_t num_clones() const {
        return clones.size();
    }
    /// Total error-state dimension of the optional calibration block.
    [[nodiscard]] std::size_t calib_dim() const {
        return kCalibBlock * calib.size();
    }
    [[nodiscard]] std::size_t dim() const {
        return kImuDim + calib_dim() + kCloneDim * clones.size();
    }
    /// Error-state offset of camera `j`'s calibration block (right after the IMU).
    [[nodiscard]] std::size_t calib_offset(std::size_t j) const {
        return kImuDim + kCalibBlock * j;
    }
    /// Error-state offset of clone `i`'s δθ block (after IMU + calibration).
    [[nodiscard]] std::size_t clone_offset(std::size_t i) const {
        return kImuDim + calib_dim() + kCloneDim * i;
    }

    /// Turn on online extrinsic calibration: append a fixed calibration block
    /// right after the IMU, seeded with the given extrinsics mean and an
    /// isotropic prior (rotation σ in rad, translation σ in metres). Must be
    /// called before any clone exists (at initialization), so the new block
    /// lands at `kImuDim` with no clone columns to shift. No-op for an empty
    /// `init`. The prior is what lets the filter *correct* the calibration
    /// rather than trust it perfectly (S10, issue #332).
    void enable_calibration(std::vector<CalibState> init, T rot_sigma, T trans_sigma) {
        // Layout invariants enforced in ALL builds (not just under assert): a late
        // or repeated call would expand `cov` while every offset helper assumes the
        // calibration block sits fixed right after the IMU.
        if (!clones.empty())
            throw std::logic_error("enable_calibration must precede any clone");
        if (!calib.empty())
            throw std::logic_error("enable_calibration: calibration already enabled");
        const std::size_t add = kCalibBlock * init.size();
        if (add == 0)
            return;
        const std::size_t d0 = cov.dim();  // == kImuDim here (no clones yet)
        // Expand P with a zero calibration block: P ← G P Gᵀ, G = [[I],[0]].
        DynMat<T> G(d0 + add, d0);
        for (std::size_t i = 0; i < d0; ++i)
            G(i, i) = T{1};
        cov.transform(G);
        // Seed the prior on the calibration diagonal via P ← I P Iᵀ + diag(σ²).
        DynMat<T> Iden(d0 + add, d0 + add);
        for (std::size_t i = 0; i < d0 + add; ++i)
            Iden(i, i) = T{1};
        std::vector<NoiseTerm<T>> q;
        q.reserve(add);
        for (std::size_t j = 0; j < init.size(); ++j) {
            const std::size_t off = kImuDim + kCalibBlock * j;
            for (std::size_t a = 0; a < 3; ++a)
                q.push_back({off + a, rot_sigma * rot_sigma});
            for (std::size_t a = 0; a < 3; ++a)
                q.push_back({off + 3 + a, trans_sigma * trans_sigma});
        }
        cov.predict(Iden, q);
        calib = std::move(init);
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
