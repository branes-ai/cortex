// SPDX-License-Identifier: MIT
//
// branes/tools/gt_spline.hpp — a smooth truth through the EuRoC ground truth,
// and an IMU synthesized from it (#613; nodes G1 and G2 of
// docs/methodology/specs/euroc-consistency-gate.yaml).
//
// The consistency gate (#605) feeds a filter ideal tracks made from the ground
// truth and asks whether, told the true noise, it does better. With the real
// IMU the "true noise" was not true: the ground truth drifts 0.25–0.45 px
// against the gyro over 0.25 s in the Vicon rooms, so the filter's R and Q were
// wrong by an unmodelled, time-correlated error. Here one truth feeds
// everything: the IMU, the tracks and the score all come from this spline.
//
//   PoseSpline       a uniform cumulative cubic B-spline on SO(3) × ℝ³ whose
//                    control poses are ground-truth samples at the knots
//                    (Kim, Kim and Shin 1995). It approximates the ground truth
//                    and does not interpolate it: the spline itself is the
//                    experiment's truth. Pose, body angular velocity (the
//                    recursion of Sommer et al. 2020) and world acceleration
//                    are analytic.
//   synthesize_imu   ω_m = ω + b_g + n_g and a_m = Rᵀ(p̈ − g) + b_a + n_a at
//                    the given times, the noise white at the configured
//                    densities (σ = density / √Δt) and the biases random walks
//                    from zero (Mueggler et al. 2018 build an IMU the same way).
//
// Clean room, from the papers. Header-only, C++20, tools/ (host-only).

#ifndef BRANES_TOOLS_GT_SPLINE_HPP
#define BRANES_TOOLS_GT_SPLINE_HPP

#include <branes/math/lie/detail.hpp>
#include <branes/math/lie/se3.hpp>
#include <branes/math/lie/so3.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <vector>

namespace branes::tools {

/// A uniform cumulative cubic B-spline on SO(3) × ℝ³.
template <math::Scalar T>
class PoseSpline {
public:
    using Vec3 = math::lie::detail::Vec<T, 3>;
    using SO3 = math::lie::SO3<T>;
    using SE3 = math::lie::SE3<T>;
    using StampedPose = sdk::eval::StampedPose<T>;

    /// Control poses every `knot_dt` seconds, each the ground-truth sample
    /// nearest its knot time. Throws if the ground truth is shorter than four
    /// knots or has a gap longer than one knot interval at a knot.
    PoseSpline(const std::vector<StampedPose>& gt, double knot_dt) : dt_(knot_dt) {
        if (!(knot_dt > 0.0))
            throw std::invalid_argument("PoseSpline: knot_dt must be positive");
        if (gt.size() < 2)
            throw std::invalid_argument("PoseSpline: needs ground truth");
        t0_ = gt.front().t_s;
        const auto n = static_cast<std::size_t>(std::floor((gt.back().t_s - t0_) / dt_)) + 1;
        if (n < 4)
            throw std::invalid_argument("PoseSpline: ground truth shorter than four knots");
        std::size_t j = 0;
        R_.reserve(n);
        p_.reserve(n);
        for (std::size_t k = 0; k < n; ++k) {
            const double tk = t0_ + dt_ * static_cast<double>(k);
            while (j + 1 < gt.size() && std::abs(gt[j + 1].t_s - tk) <= std::abs(gt[j].t_s - tk))
                ++j;
            if (std::abs(gt[j].t_s - tk) > dt_)
                throw std::invalid_argument("PoseSpline: ground-truth gap longer than a knot interval");
            R_.push_back(gt[j].pose.rotation());
            p_.push_back(gt[j].pose.translation());
        }
        // The relative rotation of each knot pair, used in every segment.
        phi_.reserve(n - 1);
        for (std::size_t k = 0; k + 1 < n; ++k)
            phi_.push_back((R_[k].inverse() * R_[k + 1]).log());
    }

    /// The span where all four control poses of a segment exist.
    [[nodiscard]] double t_begin() const noexcept {
        return t0_ + dt_;
    }
    [[nodiscard]] double t_end() const noexcept {
        return t0_ + dt_ * static_cast<double>(R_.size() - 2);
    }
    [[nodiscard]] bool contains(double t) const noexcept {
        return t >= t_begin() && t <= t_end();
    }
    [[nodiscard]] double knot_dt() const noexcept {
        return dt_;
    }

    /// T_world_imu at `t` (in the span).
    [[nodiscard]] SE3 pose(double t) const {
        const Eval e = eval(t);
        return SE3(e.R, e.p);
    }
    /// Body-frame angular velocity ω = (Rᵀ Ṙ)∨ at `t`.
    [[nodiscard]] Vec3 angular_velocity(double t) const {
        return eval(t).w;
    }
    /// World-frame velocity ṗ at `t`.
    [[nodiscard]] Vec3 velocity(double t) const {
        return eval(t).v;
    }
    /// World-frame acceleration p̈ at `t`.
    [[nodiscard]] Vec3 acceleration(double t) const {
        return eval(t).a;
    }

    /// The spline's poses at `times` (those in the span), as a trajectory.
    [[nodiscard]] std::vector<StampedPose> sample(const std::vector<double>& times) const {
        std::vector<StampedPose> out;
        out.reserve(times.size());
        for (const double t : times)
            if (contains(t))
                out.push_back(StampedPose{t, pose(t)});
        return out;
    }

private:
    struct Eval {
        SO3 R{};
        Vec3 p{}, v{}, a{}, w{};
    };

    // Segment base index i and u ∈ [0, 1]: control poses i … i+3 cover
    // [t0 + (i+1)Δ, t0 + (i+2)Δ].
    [[nodiscard]] Eval eval(double t) const {
        if (!contains(t))
            throw std::out_of_range("PoseSpline: time outside the spline's span");
        const double s = (t - t0_) / dt_ - 1.0;
        auto i = static_cast<std::size_t>(std::floor(s));
        if (i + 3 >= R_.size())
            i = R_.size() - 4;  // t == t_end: the last segment at u = 1
        const T u = static_cast<T>(s - static_cast<double>(i));
        const T u2 = u * u, u3 = u2 * u;
        // Cumulative cubic basis λ_j(u), j = 1..3, and its first two derivatives in u.
        const T six = T{6};
        const T l[3] = {
            (T{5} + T{3} * u - T{3} * u2 + u3) / six, (T{1} + T{3} * u + T{3} * u2 - T{2} * u3) / six, u3 / six};
        const T dl[3] = {(T{3} - T{6} * u + T{3} * u2) / six, (T{3} + T{6} * u - T{6} * u2) / six, T{3} * u2 / six};
        const T ddl[3] = {(T{-6} + T{6} * u) / six, (T{6} - T{12} * u) / six, T{6} * u / six};
        const T h = static_cast<T>(dt_);

        Eval e;
        // Position: p = p_i + Σ λ_j (p_{i+j} − p_{i+j−1}).
        e.p = p_[i];
        for (std::size_t j = 0; j < 3; ++j) {
            const Vec3 d = p_[i + j + 1] - p_[i + j];
            e.p = e.p + d * l[j];
            e.v = e.v + d * (dl[j] / h);
            e.a = e.a + d * (ddl[j] / (h * h));
        }
        // Rotation: R = R_i · Π Exp(λ_j φ_j); body rate by the recursion
        // ω_j = A_jᵀ ω_{j−1} + λ_j' φ_j (Sommer et al. 2020), divided by Δ.
        SO3 R = R_[i];
        Vec3 w{};
        for (std::size_t j = 0; j < 3; ++j) {
            const Vec3& phi = phi_[i + j];
            const SO3 A = SO3::exp(phi * l[j]);
            R = R * A;
            w = A.inverse() * w + phi * dl[j];
        }
        e.R = R;
        e.w = w * (T{1} / h);
        return e;
    }

    double dt_ = 0.0;
    double t0_ = 0.0;
    std::vector<SO3> R_;
    std::vector<Vec3> p_;
    std::vector<Vec3> phi_;
};

/// The IMU's noise model, as VioConfig states it (continuous-time densities).
struct ImuSynthesisOptions {
    double gyro_noise_density = 0.0;      ///< rad/s/√Hz
    double accel_noise_density = 0.0;     ///< m/s²/√Hz
    double gyro_bias_random_walk = 0.0;   ///< rad/s²/√Hz
    double accel_bias_random_walk = 0.0;  ///< m/s³/√Hz
    double gravity_magnitude = 9.81;      ///< world −z is down
    std::uint64_t seed = 1;
};

/// IMU samples from the spline at `times` (those in its span, ascending):
/// ω_m = ω + b_g + n_g, a_m = Rᵀ(p̈ − g) + b_a + n_a with g = (0, 0, −|g|).
/// The white noise has σ = density / √Δt, Δt the interval to the previous
/// sample (the first uses the next one's); the biases start at zero and walk
/// with σ = random_walk · √Δt. All densities zero gives the exact IMU.
template <math::Scalar T>
[[nodiscard]] std::vector<sdk::ImuMeasurement<T>>
synthesize_imu(const PoseSpline<T>& spline, const std::vector<double>& times, const ImuSynthesisOptions& opt) {
    using Vec3 = math::lie::detail::Vec<T, 3>;
    std::vector<double> ts;
    ts.reserve(times.size());
    for (const double t : times)
        if (spline.contains(t))
            ts.push_back(t);
    std::mt19937_64 rng(opt.seed);
    std::normal_distribution<double> n01(0.0, 1.0);
    const auto draw = [&](double sigma) {
        return Vec3{
            {static_cast<T>(sigma * n01(rng)), static_cast<T>(sigma * n01(rng)), static_cast<T>(sigma * n01(rng))}};
    };
    const Vec3 g{{T{0}, T{0}, static_cast<T>(-opt.gravity_magnitude)}};
    Vec3 bg{}, ba{};
    std::vector<sdk::ImuMeasurement<T>> out;
    out.reserve(ts.size());
    for (std::size_t k = 0; k < ts.size(); ++k) {
        const double dt = k > 0 ? ts[k] - ts[k - 1] : (ts.size() > 1 ? ts[1] - ts[0] : 0.005);
        if (!(dt > 0.0))
            throw std::invalid_argument("synthesize_imu: times must be strictly increasing");
        if (k > 0) {
            bg = bg + draw(opt.gyro_bias_random_walk * std::sqrt(dt));
            ba = ba + draw(opt.accel_bias_random_walk * std::sqrt(dt));
        }
        const double t = ts[k];
        const auto R = spline.pose(t).rotation();
        const Vec3 w = spline.angular_velocity(t) + bg + draw(opt.gyro_noise_density / std::sqrt(dt));
        const Vec3 f = R.inverse() * (spline.acceleration(t) - g) + ba + draw(opt.accel_noise_density / std::sqrt(dt));
        sdk::ImuMeasurement<T> m;
        m.timestamp_s = t;
        m.angular_velocity = {w[0], w[1], w[2]};
        m.linear_acceleration = {f[0], f[1], f[2]};
        out.push_back(m);
    }
    return out;
}

}  // namespace branes::tools

#endif  // BRANES_TOOLS_GT_SPLINE_HPP
