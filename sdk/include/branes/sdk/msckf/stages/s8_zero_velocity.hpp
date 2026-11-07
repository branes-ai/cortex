// SPDX-License-Identifier: MIT
//
// branes/sdk/msckf/stages/s8_zero_velocity.hpp — stage S8_zero_velocity
// (issue #600, docs/pipeline/canonical-pipeline.md §S8).
//
//   Detector::push_imu(gyro)        → the recent angular rates
//   Detector::push_frame(obs, cam)  → the recent pixel tracks of one camera
//   Detector::check()               → is the platform at rest? (both tests)
//   apply(state, sigma)             → the v = 0 pseudo-measurement update
//   apply_gyro_bias(state, ω̄, σ)    → the b_g = ω̄ pseudo-measurement update
//
// At rest a monocular MSCKF sees no translational parallax, so vision cannot
// hold the velocity, and the IMU's errors integrate freely: on EuRoC MH_01 the
// estimated speed grows from 0.06 to 1.6 m/s over the 22 s the drone rests on
// the ground. A zero-velocity update pins it.
//
// The detector needs both tests, and a wrong v = 0 would bias the filter, so
// both must agree:
// - the IMU's, the rate variation below a 20 Hz low-pass (as ImuInitializer's
//   vibration-robust rest test, #591). It cannot tell rest from motion at a
//   constant velocity: MH_01 at 46 s moves at 0.17 m/s with a rate variation
//   of 0.010 rad/s;
// - the image's, the median feature displacement over 0.25 s. It sees a
//   translation, which moves the features, but a pure rotation moves them as
//   well, and the IMU's test rejects that.
// On the 11 EuRoC sequences the pair fires at no frame where ground truth
// moves faster than 0.05 m/s, and at 47–78 % of the frames at rest.
//
// Header-only, C++20, type-generic.

#ifndef BRANES_SDK_MSCKF_STAGES_S8_ZERO_VELOCITY_HPP
#define BRANES_SDK_MSCKF_STAGES_S8_ZERO_VELOCITY_HPP

#include <branes/math/lie/detail.hpp>
#include <branes/sdk/msckf/dense.hpp>
#include <branes/sdk/msckf/state_helper.hpp>
#include <branes/sdk/vio_backend.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <utility>
#include <vector>

namespace branes::sdk::msckf::stages::s8_zero_velocity {

template <math::Scalar T>
struct Options {
    std::size_t imu_samples = 50;     ///< the IMU test's window (0.25 s at 200 Hz)
    std::size_t gyro_lowpass = 10;    ///< block length of the low-pass (50 ms: 20 Hz)
    T max_rate_variation = T(0.008);  ///< rad/s: at rest on EuRoC 0.001–0.005, moving ≥ 0.010
    std::size_t frames = 5;           ///< the disparity is measured over this many frames back (0.25 s at 20 Hz)
    T max_disparity_px = T{1};        ///< median feature displacement over those frames, pixels
    std::size_t min_features = 10;    ///< features seen in both frames, else no visual verdict (not still)
    T velocity_sigma = T(0.01);       ///< m/s: σ of the v = 0 pseudo-measurement, per axis
};

/// What the detector saw.
template <math::Scalar T>
struct Diagnostics {
    bool still = false;  ///< both tests passed
    bool imu_still = false;
    bool vision_still = false;
    T rate_variation = T{0};   ///< rad/s
    T disparity_px = T{0};     ///< median, pixels
    std::size_t features = 0;  ///< features the disparity is the median of
};

/// The stillness detector. Feed it every IMU sample and every frame after
/// initialization; ask it at each frame.
template <math::Scalar T>
class Detector {
public:
    using Vec3 = math::lie::detail::Vec<T, 3>;

    explicit Detector(const Options<T>& opt = {}) : opt_(opt) {}

    void reset() {
        gyro_.clear();
        frames_.clear();
    }

    void push_imu(const Vec3& gyro) {
        gyro_.push_back(gyro);
        while (gyro_.size() > opt_.imu_samples)
            gyro_.pop_front();
    }

    /// Keep camera `camera`'s pixel observations of this frame.
    void push_frame(std::span<const FrontendObservation<T>> obs, std::uint32_t camera = 0) {
        std::vector<std::pair<std::uint64_t, std::pair<T, T>>> f;
        f.reserve(obs.size());
        for (const auto& o : obs)
            if (o.camera_id == camera)
                f.emplace_back(o.feature_id, std::pair<T, T>{o.u, o.v});
        std::sort(f.begin(), f.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        frames_.push_back(std::move(f));
        while (frames_.size() > opt_.frames + 1)
            frames_.pop_front();
    }

    [[nodiscard]] Diagnostics<T> check() const {
        Diagnostics<T> d;
        if (gyro_.size() >= opt_.imu_samples) {
            d.rate_variation = rate_variation();
            d.imu_still = d.rate_variation <= opt_.max_rate_variation;
        }
        if (frames_.size() == opt_.frames + 1) {
            std::vector<T> disp;
            const auto& a = frames_.front();
            const auto& b = frames_.back();
            std::size_t i = 0, j = 0;
            while (i < a.size() && j < b.size()) {
                if (a[i].first < b[j].first) {
                    ++i;
                } else if (b[j].first < a[i].first) {
                    ++j;
                } else {
                    const T du = b[j].second.first - a[i].second.first;
                    const T dv = b[j].second.second - a[i].second.second;
                    disp.push_back(math::lie::detail::sqrt_(du * du + dv * dv));
                    ++i;
                    ++j;
                }
            }
            d.features = disp.size();
            if (d.features >= opt_.min_features) {
                const auto mid = disp.begin() + static_cast<std::ptrdiff_t>(disp.size() / 2);
                std::nth_element(disp.begin(), mid, disp.end());
                d.disparity_px = *mid;
                d.vision_still = d.disparity_px <= opt_.max_disparity_px;
            }
        }
        d.still = d.imu_still && d.vision_still;
        return d;
    }

    [[nodiscard]] const Options<T>& options() const noexcept {
        return opt_;
    }

    /// The mean angular rate over the IMU window: at rest, the gyro bias.
    [[nodiscard]] Vec3 mean_rate() const {
        Vec3 s{};
        for (const auto& g : gyro_)
            s = s + g;
        return gyro_.empty() ? s : s * (T{1} / static_cast<T>(gyro_.size()));
    }

private:
    /// The std (max over axes) of the means of consecutive blocks of
    /// `gyro_lowpass` samples: how much the rate varies below the low-pass.
    [[nodiscard]] T rate_variation() const {
        const std::size_t k = opt_.gyro_lowpass > 0 ? opt_.gyro_lowpass : 1;
        const std::size_t blocks = gyro_.size() / k;
        if (blocks < 2)
            return T{0};
        std::vector<Vec3> means(blocks);
        const std::size_t first = gyro_.size() - blocks * k;
        for (std::size_t b = 0; b < blocks; ++b) {
            Vec3 s{};
            for (std::size_t i = 0; i < k; ++i)
                s = s + gyro_[first + b * k + i];
            means[b] = s * (T{1} / static_cast<T>(k));
        }
        Vec3 m{};
        for (const auto& x : means)
            m = m + x;
        m = m * (T{1} / static_cast<T>(blocks));
        T worst = T{0};
        for (std::size_t c = 0; c < 3; ++c) {
            T v = T{0};
            for (const auto& x : means)
                v += (x[c] - m[c]) * (x[c] - m[c]);
            const T s = math::lie::detail::sqrt_(v / static_cast<T>(blocks));
            if (s > worst)
                worst = s;
        }
        return worst;
    }

    Options<T> opt_;
    std::deque<Vec3> gyro_;
    std::deque<std::vector<std::pair<std::uint64_t, std::pair<T, T>>>> frames_;
};

/// The v = 0 pseudo-measurement: H selects the velocity block, r = −v, and each
/// axis has variance σ². Returns the error-state correction.
template <math::Scalar T, class Cov>
std::vector<T> apply(State<T, Cov>& s, T velocity_sigma) {
    const std::size_t n = s.dim();
    DynMat<T> H(3, n);
    std::vector<T> r(3), R(3, velocity_sigma * velocity_sigma);
    for (std::size_t i = 0; i < 3; ++i) {
        H(i, State<T>::kVel + i) = T{1};
        r[i] = -s.v[i];
    }
    return StateHelper<T>::ekf_update(s, H, std::span<const T>{r}, std::span<const T>{R});
}

/// At rest the true angular rate is zero, so the mean measured rate is the gyro
/// bias: H selects the gyro-bias block, r = ω̄ − b_g, variance σ² per axis.
template <math::Scalar T, class Cov>
std::vector<T> apply_gyro_bias(State<T, Cov>& s, const math::lie::detail::Vec<T, 3>& mean_rate, T sigma) {
    const std::size_t n = s.dim();
    DynMat<T> H(3, n);
    std::vector<T> r(3), R(3, sigma * sigma);
    for (std::size_t i = 0; i < 3; ++i) {
        H(i, State<T>::kBg + i) = T{1};
        r[i] = mean_rate[i] - s.bg[i];
    }
    return StateHelper<T>::ekf_update(s, H, std::span<const T>{r}, std::span<const T>{R});
}

}  // namespace branes::sdk::msckf::stages::s8_zero_velocity

#endif  // BRANES_SDK_MSCKF_STAGES_S8_ZERO_VELOCITY_HPP
