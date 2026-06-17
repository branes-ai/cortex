// SPDX-License-Identifier: MIT
//
// branes/tools/s2_propagation_inspect.hpp — the S2 (IMU propagation) inspector
// (epic #371, issue #377). The real-data companion to the synthetic S2 probe
// (sdk/eval/propagation_probe.hpp); renders on the filter-internal tier, built on
// the S4 inspector template (issue #374).
//
// S2 advances the mean (strapdown integration) and the covariance
// (P ← Φ P Φᵀ + Q_d) between measurement updates. The real operator
// (`msckf::Propagator`) is already standalone — `propagate(state, gyro, accel,
// dt)` advances the mean and the cortex-Q covariance, reflected in
// `state.covariance()`. This inspector decouples it behind an S2 I/O struct: a
// prior state + an IMU window in, a per-step record out — the covariance GROWTH
// over the window (per-block σ, and the full diagonal for a heatmap) and the
// propagated pose, so the propagated-vs-ground-truth drift can be checked against
// the filter's own growing ±σ envelope.
//
// `run(S2Input)` is a pure function of the prior + IMU window, so it is
// unit-testable on a synthetic window without the ~1.5 GB EuRoC dataset.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_S2_PROPAGATION_INSPECT_HPP
#define BRANES_TOOLS_S2_PROPAGATION_INSPECT_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/propagator.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/vio_backend.hpp>  // ImuMeasurement

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace branes::tools {

/// Drift of a propagated pose from a reference pose (native units).
struct S2Drift {
    double pos_mm = 0.0;
    double att_deg = 0.0;
};

/// One propagation step's record.
struct S2Step {
    double t = 0.0;  ///< elapsed seconds from the window start
    // per-block covariance σ (√trace of the 3×3 diagonal block), native units
    double pos_sigma_mm = 0.0, vel_sigma_mm_s = 0.0, att_sigma_deg = 0.0, bg_sigma = 0.0, ba_sigma = 0.0;
    std::array<double, 15> diag_sigma{};  ///< √diag(P) per IMU error state (for the growth heatmap)
    // propagated mean (world frame)
    std::array<double, 3> p{};
    std::array<double, 3> v{};
    std::array<double, 4> q{};  ///< attitude quaternion [w,x,y,z]
    double min_diag = 0.0;      ///< smallest covariance diagonal (PSD sanity)
    // drift vs ground truth (filled when GT is supplied)
    bool have_gt = false;
    double pos_drift_mm = 0.0;
    double att_drift_deg = 0.0;
    bool pos_within_3sigma = false;  ///< pos drift ≤ 3·pos σ — is the envelope honest?
};

/// The whole propagation over the window.
struct S2Record {
    double duration_s = 0.0;
    std::uint64_t n_imu = 0;
    double rate_hz = 0.0;
    std::size_t dim = 15;
    std::vector<S2Step> steps;
};

[[nodiscard]] inline nlohmann::json to_json(const S2Record& r) {
    using nlohmann::json;
    json steps = json::array();
    for (const auto& s : r.steps) {
        json j{{"t", s.t},
               {"pos_sigma_mm", s.pos_sigma_mm},
               {"vel_sigma_mm_s", s.vel_sigma_mm_s},
               {"att_sigma_deg", s.att_sigma_deg},
               {"bg_sigma", s.bg_sigma},
               {"ba_sigma", s.ba_sigma},
               {"diag_sigma", s.diag_sigma},
               {"p", s.p},
               {"v", s.v},
               {"q", s.q},
               {"min_diag", s.min_diag}};
        if (s.have_gt) {
            j["pos_drift_mm"] = s.pos_drift_mm;
            j["att_drift_deg"] = s.att_drift_deg;
            j["pos_within_3sigma"] = s.pos_within_3sigma;
        }
        steps.push_back(std::move(j));
    }
    return json{{"duration_s", r.duration_s},
                {"n_imu", r.n_imu},
                {"rate_hz", r.rate_hz},
                {"dim", r.dim},
                {"have_gt", !r.steps.empty() && r.steps.front().have_gt},
                {"steps", std::move(steps)}};
}

/// Runs the shipped IMU propagator over a real IMU window and records the
/// covariance growth + the propagated mean.
class S2PropagationInspector {
public:
    using State = branes::sdk::msckf::State<double>;
    using Propagator = branes::sdk::msckf::Propagator<double>;
    using ImuNoise = branes::sdk::msckf::ImuNoise<double>;
    using Imu = branes::sdk::ImuMeasurement<double>;
    using Vec3 = branes::math::lie::detail::Vec<double, 3>;
    using SO3 = branes::math::lie::SO3<double>;

    /// Decoupled S2 input: the prior state (mean + covariance) and the IMU window
    /// to advance it through. Extrinsics-free — propagation is IMU-only.
    struct S2Input {
        State prior;
        std::vector<Imu> imu;
        ImuNoise noise{};
        Vec3 gravity{{0.0, 0.0, -9.81}};
        std::size_t max_samples = 120;  ///< cap recorded steps (stride the window)
    };

    [[nodiscard]] S2Record run(const S2Input& in) const {
        S2Record rec;
        rec.n_imu = static_cast<std::uint64_t>(in.imu.size());
        if (in.imu.size() < 2)
            return rec;
        rec.duration_s = in.imu.back().timestamp_s - in.imu.front().timestamp_s;
        rec.rate_hz = rec.duration_s > 0.0 ? static_cast<double>(in.imu.size() - 1) / rec.duration_s : 0.0;

        Propagator prop(in.noise, in.gravity);
        State s = in.prior;
        const double t0 = in.imu.front().timestamp_s;
        const std::size_t n = in.imu.size();
        const std::size_t stride = in.max_samples > 0 && n / in.max_samples > 0 ? n / in.max_samples : 1;

        rec.steps.reserve(n / stride + 2);
        double prev_t = t0;
        for (std::size_t k = 0; k < n; ++k) {
            // dt from consecutive stamps (first sample advances by the nominal step).
            const double dt = k == 0 ? (n > 1 ? in.imu[1].timestamp_s - t0 : 0.0) : in.imu[k].timestamp_s - prev_t;
            prev_t = in.imu[k].timestamp_s;
            if (dt > 0.0) {
                // ImuMeasurement carries std::array fields; the propagator takes the
                // lie Vec3 — lift them across.
                const auto& g = in.imu[k].angular_velocity;
                const auto& a = in.imu[k].linear_acceleration;
                prop.propagate(s, Vec3{{g[0], g[1], g[2]}}, Vec3{{a[0], a[1], a[2]}}, dt);
            }

            if (k % stride == 0 || k + 1 == n)
                rec.steps.push_back(snapshot(s, in.imu[k].timestamp_s - t0));
        }
        return rec;
    }

    /// Drift of a propagated mean from a reference (GT) pose.
    [[nodiscard]] static S2Drift drift(const Vec3& p_est, const SO3& R_est, const Vec3& p_gt, const SO3& R_gt) {
        S2Drift d;
        const Vec3 dp{{p_est[0] - p_gt[0], p_est[1] - p_gt[1], p_est[2] - p_gt[2]}};
        d.pos_mm = std::sqrt(dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]) * 1000.0;
        const Vec3 dth = (R_gt.inverse() * R_est).log();
        d.att_deg = std::sqrt(dth[0] * dth[0] + dth[1] * dth[1] + dth[2] * dth[2]) * (180.0 / kPi);
        return d;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    [[nodiscard]] static double block_sigma(const branes::sdk::msckf::DynMat<double>& P, std::size_t o) {
        double tr = 0.0;
        for (std::size_t i = 0; i < 3; ++i)
            tr += P(o + i, o + i);
        return std::sqrt(tr < 0.0 ? 0.0 : tr);
    }

    [[nodiscard]] static S2Step snapshot(const State& s, double t) {
        using St = State;
        const auto P = s.covariance();
        S2Step st;
        st.t = t;
        st.pos_sigma_mm = block_sigma(P, St::kPos) * 1000.0;
        st.vel_sigma_mm_s = block_sigma(P, St::kVel) * 1000.0;
        st.att_sigma_deg = block_sigma(P, St::kTheta) * (180.0 / kPi);
        st.bg_sigma = block_sigma(P, St::kBg);
        st.ba_sigma = block_sigma(P, St::kBa);
        double mind = P.rows > 0 ? P(0, 0) : 0.0;
        for (std::size_t i = 0; i < St::kImuDim && i < P.rows; ++i) {
            const double d = P(i, i);
            st.diag_sigma[i] = std::sqrt(d < 0.0 ? 0.0 : d);
            if (d < mind)
                mind = d;
        }
        st.min_diag = mind;
        st.p = {s.p[0], s.p[1], s.p[2]};
        st.v = {s.v[0], s.v[1], s.v[2]};
        const auto q = s.R.quaternion();
        st.q = {q[0], q[1], q[2], q[3]};
        return st;
    }
};

}  // namespace branes::tools

#endif  // BRANES_TOOLS_S2_PROPAGATION_INSPECT_HPP
