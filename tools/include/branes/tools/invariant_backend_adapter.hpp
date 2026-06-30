// SPDX-License-Identifier: MIT
//
// branes/tools/invariant_backend_adapter.hpp — adapts the right-invariant
// (R-IEKF) VIO backend to the VioEstimator<T, Backend> contract so it can run
// through the real EuRoC pipeline and be compared against the standard MSCKF
// backend (#347/#348, the yaw-leak fix; validated at the Jacobian level by
// obs_inspect, this is the end-to-end harness).
//
// InvariantVioBackend has a deliberately minimal, parameterization-honest API
// (set_nav / process_imu(gyro,accel,t) / process_camera(t, NormalizedObs) /
// nav() / covariance()) and no auto-init. VioEstimator instead wants
// initialize(VioConfig) + process_imu(ImuMeasurement) + process_camera(t,
// FrontendObservation-in-PIXELS) + current_state(). This thin adapter bridges
// the two:
//   • init — accumulates IMU until a static / gravity-aligned bootstrap succeeds
//     (the same ImuInitializer the standard backend uses), then set_nav() with a
//     zero-velocity, origin seed (the dynamic-VI scale path was measured to wash
//     out, so a gravity-aligned static seed is the fair, simple choice);
//   • observations — unprojects each pixel through the camera model to the
//     normalized bearing the invariant backend expects;
//   • state — maps the SE₂(3) nav back to a NavState<T> (T_world_imu + velocity).
//
// Lives in tools/ (not sdk/) — it is a validation harness, not a shipped path,
// and makes NO change to the load-bearing filter. Header-only, C++20.

#ifndef BRANES_TOOLS_INVARIANT_BACKEND_ADAPTER_HPP
#define BRANES_TOOLS_INVARIANT_BACKEND_ADAPTER_HPP

#include <branes/math/cameras/pinhole_radtan.hpp>
#include <branes/math/lie/se23.hpp>
#include <branes/math/lie/se3.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/imu_init.hpp>
#include <branes/sdk/msckf/invariant_vio_backend.hpp>
#include <branes/sdk/msckf/sqrt_covariance.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <cstddef>
#include <span>
#include <vector>

namespace branes::tools {

/// Drop-in `Backend` for `VioEstimator<T, InvariantBackendAdapter<T>>` that runs
/// the right-invariant filter on the real pipeline.
template <math::Scalar T>
class InvariantBackendAdapter {
public:
    // Joseph full-covariance form — the invariant backend's default and the same
    // representation the standard MSCKF uses on real data. The square-root form
    // (run_synthetic_invariant's choice) is numerically finicky and diverges on
    // real EuRoC; the Joseph form is the robust one for this harness.
    using Cov = sdk::msckf::FullCovariance<T>;
    using Impl = sdk::msckf::InvariantVioBackend<T, Cov>;
    using Camera = math::cameras::PinholeRadtanCamera<T>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Vec2 = math::lie::detail::Vec<T, 2>;
    using SO3 = math::lie::SO3<T>;
    using SE3 = math::lie::SE3<T>;
    using SE23 = math::lie::SE23<T>;
    using NormalizedObs = sdk::msckf::NormalizedObs<T>;

    /// Built with the EuRoC cam0 calibration: intrinsics (to unproject pixels) +
    /// the camera↔IMU extrinsic (handed to the invariant backend).
    InvariantBackendAdapter(Camera cam, SO3 R_imu_cam, Vec3 p_imu_cam)
        : cam_(cam), R_imu_cam_(R_imu_cam), p_imu_cam_(p_imu_cam) {}

    // ── VioEstimator<T, Backend> contract ────────────────────────────────────

    void initialize(const sdk::VioConfig& cfg) {
        typename Impl::Config icfg;
        icfg.max_clones = 11;
        icfg.backend.initial_sigma = T{1} / T{20};
        icfg.backend.normalized_sigma = static_cast<T>(cfg.camera_noise_normalized);
        icfg.backend.noise = sdk::msckf::ImuNoise<T>{static_cast<T>(cfg.gyro_noise_density),
                                                     static_cast<T>(cfg.accel_noise_density),
                                                     static_cast<T>(cfg.gyro_bias_random_walk),
                                                     static_cast<T>(cfg.accel_bias_random_walk)};
        icfg.backend.enable_gating = true;
        icfg.backend.chi2_per_dof = T{16};  // 4-sigma gate (matches run_synthetic_invariant)
        icfg.backend.R_imu_cam = R_imu_cam_;
        icfg.backend.p_imu_cam = p_imu_cam_;
        impl_ = Impl(icfg);
        initialized_ = false;
        init_gyro_.clear();
        init_accel_.clear();
        last_imu_t_ = 0.0;
    }

    void process_imu(const sdk::ImuMeasurement<T>& m) {
        // ImuMeasurement carries std::array; the backend / initializer take the
        // Lie Vec3 — convert element-wise (as run_synthetic_invariant does).
        const Vec3 gyro = to_vec3(m.angular_velocity);
        const Vec3 accel = to_vec3(m.linear_acceleration);
        if (!initialized_) {
            // Keep a sliding window of the most recent samples and try to bootstrap.
            init_gyro_.push_back(gyro);
            init_accel_.push_back(accel);
            last_imu_t_ = m.timestamp_s;
            if (init_gyro_.size() > kInitSamples) {
                init_gyro_.erase(init_gyro_.begin());
                init_accel_.erase(init_accel_.begin());
            }
            if (init_gyro_.size() >= kInitSamples)
                try_init();
            return;
        }
        impl_.process_imu(gyro, accel, m.timestamp_s);
    }

    void process_camera(double t, std::span<const sdk::FrontendObservation<T>> obs) {
        if (!initialized_)
            return;  // no clones before the nav seed exists
        std::vector<NormalizedObs> nobs;
        nobs.reserve(obs.size());
        for (const auto& o : obs) {
            if (o.camera_id != 0)
                continue;
            const auto bearing = cam_.unproject(math::cameras::Vec2<T>{o.u, o.v});
            if (!(bearing[2] > T{0}))
                continue;  // ray not in front of the camera
            nobs.push_back(NormalizedObs{o.feature_id, 0, Vec2{{bearing[0] / bearing[2], bearing[1] / bearing[2]}}});
        }
        impl_.process_camera(t, std::span<const NormalizedObs>{nobs});
    }

    [[nodiscard]] sdk::NavState<T> current_state() const {
        sdk::NavState<T> ns;
        const auto& nav = impl_.nav();
        ns.timestamp_s = nav.timestamp;
        ns.T_world_imu = SE3(nav.X.rotation(), nav.X.position());
        const auto& v = nav.X.velocity();
        ns.velocity_world = {v[0], v[1], v[2]};  // Lie Vec3 → std::array
        return ns;
    }

    // ── Telemetry passthroughs (used by the euroc-invariant harness) ──────────

    [[nodiscard]] bool initialized() const noexcept {
        return initialized_;
    }
    [[nodiscard]] sdk::msckf::DynMat<T> covariance() const {
        return impl_.covariance();
    }
    [[nodiscard]] std::size_t num_clones() const noexcept {
        return impl_.num_clones();
    }
    [[nodiscard]] const Impl& impl() const noexcept {
        return impl_;
    }

private:
    static Vec3 to_vec3(const sdk::Vec3<T>& a) {
        return Vec3{{a[0], a[1], a[2]}};
    }

    void try_init() {
        const sdk::ImuInitializer<T> init;
        const std::span<const Vec3> g{init_gyro_}, a{init_accel_};
        auto r = init.try_static(g, a);
        if (!r.success)
            r = init.try_gravity_align(g, a);  // looser band: roll/pitch from mean specific force
        if (!r.success)
            return;  // keep accumulating; retry on the next sample
        // Zero-velocity, origin seed — a near-static EuRoC start; the dynamic-VI
        // scale path was measured to wash out end-to-end, so this is the fair seed.
        impl_.set_nav(SE23(r.R_world_imu, Vec3{}, Vec3{}), r.gyro_bias, r.accel_bias, last_imu_t_);
        initialized_ = true;
    }

    static constexpr std::size_t kInitSamples = 150;  // ~0.75 s @ 200 Hz

    Camera cam_;
    SO3 R_imu_cam_;
    Vec3 p_imu_cam_;
    Impl impl_{typename Impl::Config{}};
    bool initialized_ = false;
    std::vector<Vec3> init_gyro_, init_accel_;
    double last_imu_t_ = 0.0;
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_INVARIANT_BACKEND_ADAPTER_HPP
