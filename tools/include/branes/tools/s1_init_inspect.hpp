// SPDX-License-Identifier: MIT
//
// branes/tools/s1_init_inspect.hpp — the S1 (initialization / bootstrap)
// inspector (epic #371, issue #376). The real-data companion to the synthetic S1
// probe (sdk/eval/initialization_probe.hpp); renders on the filter-internal tier,
// built on the S4 inspector template (issue #374).
//
// S1 bootstraps the manifold: gravity-aligned orientation + gyro bias (static),
// and — on the dynamic path — gravity, per-keyframe velocity, and the metric
// SCALE. The real operators (`ImuInitializer::try_static`/`try_dynamic`) are
// already decoupled; the backend seeds the initial state + covariance and exposes
// it via `initialized()` / `state()` / `init_diagnostics()`. This inspector turns
// that captured moment into a record: the recovered state, the per-block initial
// covariance σ (is the seed honestly structured, or isotropic σ·I?), and — the
// headline — gravity / scale / velocity / bias error against ground truth.
//
// `build(diag, state, gt)` is a pure record builder (the GT-comparison math + the
// covariance extraction), so it is unit-testable on a constructed state without
// the ~1.5 GB EuRoC dataset; the tool (s1_inspect.cpp) feeds it the live capture.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_S1_INIT_INSPECT_HPP
#define BRANES_TOOLS_S1_INIT_INSPECT_HPP

#include <branes/sdk/eval/nav_consistency.hpp>  // eval::NavSample
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/msckf_backend.hpp>  // InitDiagnostics, InitMethod, to_string

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace branes::tools {

/// Everything the bootstrap produced — the inspector's record.
struct S1InitRecord {
    std::string method = "none";  ///< static | dynamic | gravity_align | identity | none
    double t_s = 0.0;

    // Recovered initial state (the seed the filter starts from).
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;  ///< attitude (ZYX) of R_world_imu
    std::array<double, 3> velocity{};                       ///< world velocity (0 for static)
    std::array<double, 3> gyro_bias{};
    std::array<double, 3> accel_bias{};
    double scale = 1.0;  ///< metric scale (dynamic only; 1 otherwise)

    // Initial covariance: the full block (for the heatmap) + per-block σ.
    std::vector<double> cov;  ///< row-major dim×dim (15×15 right after init)
    std::size_t dim = 0;
    double sigma_theta_deg = 0.0, sigma_pos_m = 0.0, sigma_vel_ms = 0.0, sigma_bg = 0.0, sigma_ba = 0.0;
    bool isotropic_seed = false;  ///< the 15 IMU diagonals are ~equal (σ·I), not structured per observability

    // Init diagnostics (native).
    double gravity_residual = 0.0;  ///< |‖accel_mean‖−g|/g — stationarity quality (static)
    double dyn_seed_speed = 0.0;    ///< ‖v‖ of the seed (dynamic)
    int dyn_keyframes = 0;
    double dyn_tilt_vs_accel_deg = 0.0;

    // Ground-truth comparison (the headline).
    bool have_gt = false;
    double gravity_dir_error_deg = 0.0;  ///< yaw-invariant roll/pitch error vs GT (the leveling error)
    double yaw_error_deg = 0.0;          ///< unobservable gauge freedom — reported, not a fault
    double velocity_error_ms = 0.0;
    double gyro_bias_error = 0.0;
    double accel_bias_error = 0.0;
    double scale_error_pct = 0.0;  ///< dynamic only (EuRoC is metric ⇒ true scale = 1)
};

[[nodiscard]] inline nlohmann::json to_json(const S1InitRecord& r) {
    using nlohmann::json;
    return json{{"method", r.method},
                {"t", r.t_s},
                {"attitude_deg", json{{"roll", r.roll_deg}, {"pitch", r.pitch_deg}, {"yaw", r.yaw_deg}}},
                {"velocity", r.velocity},
                {"gyro_bias", r.gyro_bias},
                {"accel_bias", r.accel_bias},
                {"scale", r.scale},
                {"cov", r.cov},
                {"dim", r.dim},
                {"sigma",
                 json{{"theta_deg", r.sigma_theta_deg},
                      {"pos_m", r.sigma_pos_m},
                      {"vel_ms", r.sigma_vel_ms},
                      {"bg", r.sigma_bg},
                      {"ba", r.sigma_ba}}},
                {"isotropic_seed", r.isotropic_seed},
                {"gravity_residual", r.gravity_residual},
                {"dyn_seed_speed", r.dyn_seed_speed},
                {"dyn_keyframes", r.dyn_keyframes},
                {"dyn_tilt_vs_accel_deg", r.dyn_tilt_vs_accel_deg},
                {"have_gt", r.have_gt},
                {"gravity_dir_error_deg", r.gravity_dir_error_deg},
                {"yaw_error_deg", r.yaw_error_deg},
                {"velocity_error_ms", r.velocity_error_ms},
                {"gyro_bias_error", r.gyro_bias_error},
                {"accel_bias_error", r.accel_bias_error},
                {"scale_error_pct", r.scale_error_pct}};
}

/// Builds an S1 record from a captured init (diagnostics + seeded state) and,
/// optionally, the ground-truth nav state at the init time.
class S1InitInspector {
public:
    using State = branes::sdk::msckf::State<double>;
    using NavSample = branes::sdk::eval::NavSample<double>;
    using InitDiagnostics = branes::sdk::InitDiagnostics<double>;
    using Vec3 = branes::math::lie::detail::Vec<double, 3>;

    /// `gt` may be null (no ground truth available) — then the *_error fields stay 0.
    [[nodiscard]] S1InitRecord build(const InitDiagnostics& diag, const State& state, const NavSample* gt) const {
        S1InitRecord r;
        r.method = branes::sdk::to_string(diag.method);
        r.t_s = diag.t_s;

        const auto euler = euler_zyx(state.R.matrix());
        r.roll_deg = euler[0] * kRad2Deg;
        r.pitch_deg = euler[1] * kRad2Deg;
        r.yaw_deg = euler[2] * kRad2Deg;
        r.velocity = {state.v[0], state.v[1], state.v[2]};
        r.gyro_bias = {state.bg[0], state.bg[1], state.bg[2]};
        r.accel_bias = {state.ba[0], state.ba[1], state.ba[2]};
        r.scale = diag.method == branes::sdk::InitMethod::Dynamic ? diag.dyn_scale : 1.0;

        r.gravity_residual = diag.gravity_residual;
        r.dyn_seed_speed = diag.dyn_seed_speed;
        r.dyn_keyframes = diag.dyn_keyframes;
        r.dyn_tilt_vs_accel_deg = diag.dyn_tilt_vs_accel_deg;

        // Initial covariance + per-block σ (√diag). θ is rad → deg.
        const auto P = state.covariance();
        r.dim = P.rows;
        r.cov.reserve(P.rows * P.cols);
        for (std::size_t i = 0; i < P.rows; ++i)
            for (std::size_t j = 0; j < P.cols; ++j)
                r.cov.push_back(P(i, j));
        auto sig = [&](std::size_t i) { return P.rows > i ? std::sqrt(std::max(0.0, P(i, i))) : 0.0; };
        r.sigma_theta_deg = sig(State::kTheta) * kRad2Deg;
        r.sigma_pos_m = sig(State::kPos);
        r.sigma_vel_ms = sig(State::kVel);
        r.sigma_bg = sig(State::kBg);
        r.sigma_ba = sig(State::kBa);
        r.isotropic_seed = is_isotropic(P);

        if (gt) {
            r.have_gt = true;
            // Gravity-direction (roll/pitch) error: the angle between world-down
            // expressed in body by each attitude. Yaw-invariant — the observable part.
            const Vec3 down{{0.0, 0.0, -1.0}};
            r.gravity_dir_error_deg = angle_deg(state.R.inverse() * down, gt->R.inverse() * down);
            r.yaw_error_deg = yaw_error_deg(state.R.matrix(), gt->R.matrix());
            // Speed (magnitude) error: the filter's world frame carries an arbitrary
            // yaw gauge vs the GT world, so a velocity *vector* difference would be
            // gauge-polluted; the speed magnitude is gauge-invariant.
            r.velocity_error_ms = std::abs(norm3(state.v) - norm3(gt->v));
            r.gyro_bias_error = norm3(sub(state.bg, gt->bg));
            r.accel_bias_error = norm3(sub(state.ba, gt->ba));
            if (diag.method == branes::sdk::InitMethod::Dynamic)
                r.scale_error_pct = std::abs(r.scale - 1.0) * 100.0;  // EuRoC GT is metric
        }
        return r;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kRad2Deg = 180.0 / kPi;

    using Mat3 = branes::math::lie::detail::Mat<double, 3, 3>;

    static Vec3 sub(const Vec3& a, const Vec3& b) {
        return Vec3{{a[0] - b[0], a[1] - b[1], a[2] - b[2]}};
    }
    static double norm3(const Vec3& a) {
        return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    }
    static double angle_deg(const Vec3& a, const Vec3& b) {
        const double na = norm3(a), nb = norm3(b);
        if (!(na > 0.0) || !(nb > 0.0))
            return 0.0;
        double c = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb);
        c = c > 1.0 ? 1.0 : (c < -1.0 ? -1.0 : c);
        return std::acos(c) * kRad2Deg;
    }
    /// ZYX (roll about x, pitch about y, yaw about z) from a world←body matrix.
    static std::array<double, 3> euler_zyx(const Mat3& R) {
        // pitch = asin(-R(2,0)); clamp the SINE argument (after negation) so numeric
        // drift outside [-1,1] saturates to the correct ±90°, not the wrong sign.
        double sp = -R(2, 0);
        sp = sp < -1.0 ? -1.0 : (sp > 1.0 ? 1.0 : sp);
        const double pitch = std::asin(sp);
        const double roll = std::atan2(R(2, 1), R(2, 2));
        const double yaw = std::atan2(R(1, 0), R(0, 0));
        return {roll, pitch, yaw};
    }
    /// Yaw difference (deg) between two world←body attitudes.
    static double yaw_error_deg(const Mat3& a, const Mat3& b) {
        const double ya = std::atan2(a(1, 0), a(0, 0));
        const double yb = std::atan2(b(1, 0), b(0, 0));
        double d = (ya - yb) * kRad2Deg;
        while (d > 180.0)
            d -= 360.0;
        while (d < -180.0)
            d += 360.0;
        return std::abs(d);
    }
    /// Is the IMU covariance block an isotropic σ·I (all 15 diagonals ~equal)?
    static bool is_isotropic(const branes::sdk::msckf::DynMat<double>& P) {
        const std::size_t n = P.rows < State::kImuDim ? P.rows : State::kImuDim;
        if (n == 0)
            return false;
        double lo = P(0, 0), hi = P(0, 0);
        for (std::size_t i = 0; i < n; ++i) {
            lo = std::min(lo, P(i, i));
            hi = std::max(hi, P(i, i));
        }
        return lo > 0.0 && hi / lo < 1.01;  // within 1% ⇒ effectively σ·I
    }
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_S1_INIT_INSPECT_HPP
