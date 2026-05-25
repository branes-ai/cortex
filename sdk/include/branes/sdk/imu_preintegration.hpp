// SPDX-License-Identifier: MIT
//
// branes/sdk/imu_preintegration.hpp — on-manifold IMU preintegration.
//
// Accumulates gyro+accel samples between two keyframes into the
// preintegrated rotation ΔR, velocity Δv, and position Δp, together with
// their first-order Jacobians w.r.t. the gyro/accel biases (so a bias
// update can be applied without re-integrating). Clean-room from the
// papers only — Forster et al., "On-Manifold Preintegration for Real-Time
// Visual-Inertial Odometry" (2016/2017), and Solà, "Quaternion kinematics
// for the error-state Kalman filter" (2017). No third-party CPI source.
//
// Header-only, C++20, type-generic. Reuses the math-layer Lie groups.

#ifndef BRANES_SDK_IMU_PREINTEGRATION_HPP
#define BRANES_SDK_IMU_PREINTEGRATION_HPP

#include <branes/math/lie/so3.hpp>

namespace branes::sdk {

/// Preintegrates an IMU window. Construct with the bias linearization
/// point, feed samples with `integrate`, then read ΔR/Δv/Δp or `predict`
/// the next nav-state. Convention: the accelerometer measures specific
/// force a_m = Rᵀ(a_world − g); preintegration is gravity- and
/// initial-state-independent.
template <math::Scalar T>
class ImuPreintegrator {
public:
    using SO3 = math::lie::SO3<T>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Mat3 = math::lie::detail::Mat<T, 3, 3>;

    explicit ImuPreintegrator(const Vec3& gyro_bias = {}, const Vec3& accel_bias = {})
        : bg_(gyro_bias), ba_(accel_bias) {}

    /// Fold one IMU sample (gyro rad/s, accel m/s²) over interval `dt`.
    /// A non-positive or non-finite `dt` (e.g. out-of-order or duplicate
    /// timestamps) is ignored rather than corrupting the accumulators.
    void integrate(const Vec3& gyro, const Vec3& accel, T dt) {
        if (!(dt > T{0}))
            return;  // also rejects NaN
        const Vec3 w = gyro - bg_;
        const Vec3 a = accel - ba_;
        const SO3 dRi = SO3::exp(w * dt);
        const Mat3 R = dR_.matrix();
        const Mat3 Jr = SO3::right_jacobian(w * dt);
        const Mat3 a_hat = math::lie::detail::hat(a);
        const T dt2 = dt * dt;

        // All updates use the pre-update accumulators, so compute the new
        // values first, then commit.
        const Mat3 new_dp_dba = dp_dba_ + dv_dba_ * dt + R * (-T{0.5} * dt2);
        const Mat3 new_dp_dbg = dp_dbg_ + dv_dbg_ * dt + (R * a_hat * dR_dbg_) * (-T{0.5} * dt2);
        const Mat3 new_dv_dba = dv_dba_ + R * (-dt);
        const Mat3 new_dv_dbg = dv_dbg_ + (R * a_hat * dR_dbg_) * (-dt);
        const Mat3 new_dR_dbg = math::lie::detail::transpose(dRi.matrix()) * dR_dbg_ + Jr * (-dt);

        const Vec3 Ra = R * a;
        const Vec3 new_dp = dp_ + dv_ * dt + Ra * (T{0.5} * dt2);
        const Vec3 new_dv = dv_ + Ra * dt;
        const SO3 new_dR = dR_ * dRi;

        dp_dba_ = new_dp_dba;
        dp_dbg_ = new_dp_dbg;
        dv_dba_ = new_dv_dba;
        dv_dbg_ = new_dv_dbg;
        dR_dbg_ = new_dR_dbg;
        dp_ = new_dp;
        dv_ = new_dv;
        dR_ = new_dR;
        dt_ += dt;
    }

    // ── Preintegrated measurements ───────────────────────────────────
    [[nodiscard]] const SO3& delta_rotation() const noexcept {
        return dR_;
    }
    [[nodiscard]] const Vec3& delta_velocity() const noexcept {
        return dv_;
    }
    [[nodiscard]] const Vec3& delta_position() const noexcept {
        return dp_;
    }
    [[nodiscard]] T delta_time() const noexcept {
        return dt_;
    }

    // ── Bias Jacobians (∂Δ/∂bias at the linearization point) ─────────
    [[nodiscard]] const Mat3& d_rotation_d_gyro_bias() const noexcept {
        return dR_dbg_;
    }
    [[nodiscard]] const Mat3& d_velocity_d_gyro_bias() const noexcept {
        return dv_dbg_;
    }
    [[nodiscard]] const Mat3& d_velocity_d_accel_bias() const noexcept {
        return dv_dba_;
    }
    [[nodiscard]] const Mat3& d_position_d_gyro_bias() const noexcept {
        return dp_dbg_;
    }
    [[nodiscard]] const Mat3& d_position_d_accel_bias() const noexcept {
        return dp_dba_;
    }

    // ── First-order bias-corrected measurements ──────────────────────
    [[nodiscard]] SO3 corrected_delta_rotation(const Vec3& new_gyro_bias) const {
        return dR_ * SO3::exp(dR_dbg_ * (new_gyro_bias - bg_));
    }
    [[nodiscard]] Vec3 corrected_delta_velocity(const Vec3& new_gyro_bias, const Vec3& new_accel_bias) const {
        return dv_ + dv_dbg_ * (new_gyro_bias - bg_) + dv_dba_ * (new_accel_bias - ba_);
    }
    [[nodiscard]] Vec3 corrected_delta_position(const Vec3& new_gyro_bias, const Vec3& new_accel_bias) const {
        return dp_ + dp_dbg_ * (new_gyro_bias - bg_) + dp_dba_ * (new_accel_bias - ba_);
    }

    /// Predict the next nav-state from the keyframe-i state and gravity.
    /// `R_i`/`v_i`/`p_i` are world←body rotation, world velocity, world
    /// position; `gravity` is the world gravity vector (e.g. (0,0,−9.81)).
    void predict(
        const SO3& R_i, const Vec3& v_i, const Vec3& p_i, const Vec3& gravity, SO3& R_j, Vec3& v_j, Vec3& p_j) const {
        const Mat3 Ri = R_i.matrix();
        R_j = R_i * dR_;
        v_j = v_i + gravity * dt_ + Ri * dv_;
        p_j = p_i + v_i * dt_ + gravity * (T{0.5} * dt_ * dt_) + Ri * dp_;
    }

private:
    SO3 dR_{};
    Vec3 dv_{};
    Vec3 dp_{};
    T dt_ = T{0};
    Vec3 bg_{};
    Vec3 ba_{};
    Mat3 dR_dbg_ = Mat3::zero();
    Mat3 dv_dbg_ = Mat3::zero();
    Mat3 dv_dba_ = Mat3::zero();
    Mat3 dp_dbg_ = Mat3::zero();
    Mat3 dp_dba_ = Mat3::zero();
};

}  // namespace branes::sdk

#endif  // BRANES_SDK_IMU_PREINTEGRATION_HPP
