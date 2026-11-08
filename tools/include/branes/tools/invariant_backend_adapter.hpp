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
// FrontendObservation-in-PIXELS) + current_state(). This adapter bridges the
// two, and since #608 runs the R-IEKF through the shipped real-data path of the
// body-frame MSCKF, so a comparison on EuRoC is between estimators and not
// between harnesses:
//
//   • S1 init — the adapter runs the shipped MsckfBackend on the IMU and the
//     frames until it initializes, then takes its seed: every init path the
//     MSCKF has (static, the vibrating-rest gyro bias, gravity alignment and
//     its timeouts, dynamic VI alignment with its velocity, --prefer-dynamic)
//     with no copy of the logic. The seed's body-frame prior is mapped into the
//     right-invariant error: δθ_world = R̂·δθ_body, ν = δv + [v̂]×δθ_world,
//     ρ = δp + [p̂]×δθ_world, biases unchanged;
//   • S6d gate — the χ²₁₋α(dof) quantile from VioConfig (gate_alpha), or the
//     5-per-dof line with gate_per_dof, as the MSCKF (#481);
//   • S8 — the shipped stillness detector; at rest, the v = 0 and b_g = ω̄
//     pseudo-measurements in the right-invariant error, and no clone (#600);
//   • observations — unprojects each pixel through the camera model to the
//     normalized bearing the invariant backend expects;
//   • state — maps the SE₂(3) nav back to a NavState<T> (T_world_imu + velocity).
//
// Each piece can be turned back to the pre-#608 harness (`Options`) so its
// effect is measured on its own: a 16-per-dof gate, a static / gravity-aligned
// window of 150 samples seeding zero velocity with an isotropic σ = 0.05 prior,
// and no S8.
//
// Lives in tools/ (not sdk/): a validation harness, not a shipped path. Whether
// the R-IEKF becomes a shipped VioBackend is decided on the #608 measurements.
// Header-only, C++20.

#ifndef BRANES_TOOLS_INVARIANT_BACKEND_ADAPTER_HPP
#define BRANES_TOOLS_INVARIANT_BACKEND_ADAPTER_HPP

#include <branes/math/cameras/pinhole_radtan.hpp>
#include <branes/math/lie/detail.hpp>
#include <branes/math/lie/se23.hpp>
#include <branes/math/lie/se3.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/imu_init.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/invariant_vio_backend.hpp>
#include <branes/sdk/msckf/stages/s8_zero_velocity.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

namespace branes::tools {

/// Map a body-frame MSCKF nav covariance (δθ_body, δp, δv, δb_g, δb_a; 15 × 15)
/// at the estimate (R̂, v̂, p̂) into the right-invariant nav error of
/// MsckfInvariantBackend (δθ, ν, ρ, δb_g, δb_a): returns J with
/// P_invariant = J · P_body · Jᵀ. From R = R̂·Exp(δθ_body) = Exp(R̂δθ_body)·R̂,
/// v = Exp(δθ)·v̂ + ν and p = Exp(δθ)·p̂ + ρ, to first order.
template <math::Scalar T>
[[nodiscard]] sdk::msckf::DynMat<T> body_to_invariant_jacobian(const math::lie::SO3<T>& R,
                                                               const math::lie::detail::Vec<T, 3>& v,
                                                               const math::lie::detail::Vec<T, 3>& p) {
    using Body = sdk::msckf::State<T>;
    using Inv = sdk::msckf::InvariantNavState<T>;
    namespace ld = math::lie::detail;
    sdk::msckf::DynMat<T> J(Inv::kDim, Inv::kDim);
    const auto Rm = R.matrix();
    const auto vR = ld::hat(v) * Rm;
    const auto pR = ld::hat(p) * Rm;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            J(Inv::kTheta + i, Body::kTheta + j) = Rm(i, j);
            J(Inv::kVel + i, Body::kTheta + j) = vR(i, j);
            J(Inv::kPos + i, Body::kTheta + j) = pR(i, j);
        }
        J(Inv::kVel + i, Body::kVel + i) = T{1};
        J(Inv::kPos + i, Body::kPos + i) = T{1};
        J(Inv::kBg + i, Body::kBg + i) = T{1};
        J(Inv::kBa + i, Body::kBa + i) = T{1};
    }
    return J;
}

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
    using Boot = sdk::MsckfBackend<T>;
    using Camera = math::cameras::PinholeRadtanCamera<T>;
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using Vec2 = math::lie::detail::Vec<T, 2>;
    using SO3 = math::lie::SO3<T>;
    using SE3 = math::lie::SE3<T>;
    using SE23 = math::lie::SE23<T>;
    using NormalizedObs = sdk::msckf::NormalizedObs<T>;
    using Detector = sdk::msckf::stages::s8_zero_velocity::Detector<T>;

    /// Which pieces run as the pre-#608 harness, to measure each on its own.
    struct Options {
        bool legacy_gate = false;  ///< 16 per dof instead of VioConfig's gate
        bool legacy_init = false;  ///< a 150-sample static / gravity window, v = 0, σ = 0.05 prior
        bool legacy_zupt = false;  ///< no S8 (VioConfig::zero_velocity_update off has the same effect)
    };

    /// Built with the EuRoC cam0 calibration: intrinsics (to unproject pixels) +
    /// the camera↔IMU extrinsic (handed to the invariant backend and the
    /// bootstrap MSCKF).
    InvariantBackendAdapter(Camera cam, SO3 R_imu_cam, Vec3 p_imu_cam, Options opt = {})
        : cam_(cam), R_imu_cam_(R_imu_cam), p_imu_cam_(p_imu_cam), opt_(opt),
          boot_(std::vector<typename Boot::CameraCalibration>{
              typename Boot::CameraCalibration{cam, sdk::msckf::CameraExtrinsics<T>{R_imu_cam, p_imu_cam}}}) {}

    // ── VioEstimator<T, Backend> contract ────────────────────────────────────

    void initialize(const sdk::VioConfig& cfg) {
        if (cfg.estimate_extrinsics)
            throw std::invalid_argument("InvariantBackendAdapter: online extrinsic calibration is not wired");
        cfg_ = cfg;
        typename Impl::Config icfg;
        icfg.max_clones = cfg.max_clones > 1 ? static_cast<std::size_t>(cfg.max_clones) : 11;
        icfg.backend.initial_sigma = opt_.legacy_init ? T{1} / T{20} : T{1} / T{10};
        icfg.backend.normalized_sigma = static_cast<T>(cfg.camera_noise_normalized);
        icfg.backend.noise = sdk::msckf::ImuNoise<T>{static_cast<T>(cfg.gyro_noise_density),
                                                     static_cast<T>(cfg.accel_noise_density),
                                                     static_cast<T>(cfg.gyro_bias_random_walk),
                                                     static_cast<T>(cfg.accel_bias_random_walk)};
        icfg.backend.gravity = Vec3{{T{0}, T{0}, -static_cast<T>(cfg.gravity_magnitude)}};
        icfg.backend.enable_gating = true;
        if (opt_.legacy_gate) {
            icfg.backend.gate_rule = sdk::msckf::GateRule::per_dof;
            icfg.backend.chi2_per_dof = T{16};  // the pre-#608 4-sigma line
        } else {
            icfg.backend.gate_rule =
                cfg.gate_per_dof ? sdk::msckf::GateRule::per_dof : sdk::msckf::GateRule::chi2_quantile;
            icfg.backend.gate_alpha = cfg.gate_alpha;
            icfg.backend.chi2_per_dof = T{5};  // the MSCKF's per-dof line (CameraUpdater)
        }
        icfg.backend.R_imu_cam = R_imu_cam_;
        icfg.backend.p_imu_cam = p_imu_cam_;
        impl_ = Impl(icfg);
        if (!opt_.legacy_init)
            boot_.initialize(cfg);
        sdk::msckf::stages::s8_zero_velocity::Options<T> zopt;
        zopt.max_disparity_px = static_cast<T>(cfg.zupt_max_disparity_px);
        zupt_ = Detector{zopt};
        zupt_frames_ = 0;
        zupt_bias_age_ = zupt_.options().imu_samples;
        initialized_ = false;
        init_gyro_.clear();
        init_accel_.clear();
        last_imu_t_ = 0.0;
        have_imu_ = false;
    }

    void process_imu(const sdk::ImuMeasurement<T>& m) {
        // ImuMeasurement carries std::array; the backend / initializer take the
        // Lie Vec3 — convert element-wise (as run_synthetic_invariant does).
        const Vec3 gyro = to_vec3(m.angular_velocity);
        const Vec3 accel = to_vec3(m.linear_acceleration);
        if (!initialized_) {
            if (opt_.legacy_init) {
                legacy_process_imu(gyro, accel, m.timestamp_s);
                return;
            }
            boot_.process_imu(m);
            hold(gyro, accel, m.timestamp_s);
            if (boot_.initialized())
                seed_from_boot();
            return;
        }
        impl_.process_imu(gyro, accel, m.timestamp_s);
        if (zupt_on()) {
            zupt_.push_imu(gyro);
            ++zupt_bias_age_;
        }
    }

    void process_camera(double t, std::span<const sdk::FrontendObservation<T>> obs) {
        if (!initialized_) {
            if (opt_.legacy_init)
                return;  // no clones before the nav seed exists
            // The dynamic VI alignment runs on frames; as in the MSCKF, the
            // frame that completes it adds no clone.
            boot_.process_camera(t, obs);
            if (boot_.initialized())
                seed_from_boot();
            return;
        }
        // S8: at rest, hold the velocity at zero and leave the frame out of the
        // window, with the MSCKF's detector and σ's (#600).
        if (zupt_on()) {
            zupt_.push_frame(obs);
            if (zupt_.check().still) {
                (void)impl_.process_still_frame(t, static_cast<T>(cfg_.zupt_velocity_sigma));
                if (zupt_bias_age_ >= zupt_.options().imu_samples) {
                    (void)impl_.gyro_bias_update(zupt_.mean_rate(), static_cast<T>(cfg_.zupt_gyro_bias_sigma));
                    zupt_bias_age_ = 0;
                }
                ++zupt_frames_;
                return;
            }
        }
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
    /// How the shipped initializer resolved the seed (method None with
    /// legacy_init, or until initialized).
    [[nodiscard]] const sdk::InitDiagnostics<T>& init_diagnostics() const noexcept {
        return boot_.init_diagnostics();
    }
    /// Frames at which S8 found the platform at rest (#600).
    [[nodiscard]] std::size_t zero_velocity_frames() const noexcept {
        return zupt_frames_;
    }
    [[nodiscard]] const Options& options() const noexcept {
        return opt_;
    }

private:
    static Vec3 to_vec3(const sdk::Vec3<T>& a) {
        return Vec3{{a[0], a[1], a[2]}};
    }

    [[nodiscard]] bool zupt_on() const noexcept {
        return !opt_.legacy_zupt && cfg_.zero_velocity_update;
    }

    // The IMU sample in effect at the filter clock, handed over with the seed.
    void hold(const Vec3& gyro, const Vec3& accel, double t) {
        hold_gyro_ = gyro;
        hold_accel_ = accel;
        last_imu_t_ = t;
        have_imu_ = true;
    }

    // Take the bootstrap MSCKF's S1 seed: the mean as it is, the body-frame
    // prior mapped into the right-invariant error.
    void seed_from_boot() {
        const auto& s = boot_.state();
        const auto P = s.covariance();
        sdk::msckf::DynMat<T> Pn(Impl::Backend::Nav::kDim, Impl::Backend::Nav::kDim);
        for (std::size_t i = 0; i < Pn.rows; ++i)
            for (std::size_t j = 0; j < Pn.cols; ++j)
                Pn(i, j) = P(i, j);
        sdk::msckf::DynMat<T> L;
        if (!sdk::msckf::cholesky(Pn, L))
            throw std::runtime_error("InvariantBackendAdapter: the bootstrap prior is not positive definite");
        const auto G = sdk::msckf::mul(body_to_invariant_jacobian<T>(s.R, s.v, s.p), L);
        const SE23 X(s.R, s.v, s.p);
        if (have_imu_)
            impl_.set_nav(X, s.bg, s.ba, s.timestamp, hold_gyro_, hold_accel_);
        else
            impl_.set_nav(X, s.bg, s.ba, s.timestamp);
        impl_.seed_covariance(G);
        initialized_ = true;
    }

    // The pre-#608 bootstrap: a sliding 150-sample window, static then gravity
    // alignment, seeding zero velocity at the origin.
    void legacy_process_imu(const Vec3& gyro, const Vec3& accel, double t) {
        init_gyro_.push_back(gyro);
        init_accel_.push_back(accel);
        last_imu_t_ = t;
        if (init_gyro_.size() > kInitSamples) {
            init_gyro_.erase(init_gyro_.begin());
            init_accel_.erase(init_accel_.begin());
        }
        if (init_gyro_.size() < kInitSamples)
            return;
        const sdk::ImuInitializer<T> init;
        const std::span<const Vec3> g{init_gyro_}, a{init_accel_};
        auto r = init.try_static(g, a);
        if (!r.success)
            r = init.try_gravity_align(g, a);  // looser band: roll/pitch from mean specific force
        if (!r.success)
            return;  // keep accumulating; retry on the next sample
        impl_.set_nav(SE23(r.R_world_imu, Vec3{}, Vec3{}), r.gyro_bias, r.accel_bias, last_imu_t_);
        initialized_ = true;
    }

    static constexpr std::size_t kInitSamples = 150;  // legacy window, ~0.75 s @ 200 Hz

    Camera cam_;
    SO3 R_imu_cam_;
    Vec3 p_imu_cam_;
    Options opt_;
    Boot boot_;
    sdk::VioConfig cfg_{};
    Impl impl_{typename Impl::Config{}};
    Detector zupt_{};
    std::size_t zupt_frames_ = 0;
    std::size_t zupt_bias_age_ = 0;
    bool initialized_ = false;
    std::vector<Vec3> init_gyro_, init_accel_;
    Vec3 hold_gyro_{}, hold_accel_{};
    double last_imu_t_ = 0.0;
    bool have_imu_ = false;
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_INVARIANT_BACKEND_ADAPTER_HPP
