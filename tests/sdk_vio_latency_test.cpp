// VIO per-frame latency budget enforcement (issue #47).
//
// Wraps VioEstimator::feed_image in a microbenchmark and fails if the
// median or 99th-percentile per-frame latency exceeds the budget recorded
// in eval/latency_budget.hpp (a tunable knob — tighten as the SDK speeds
// up). The CI guard runs on a stable synthetic stream at EuRoC cam0
// resolution (752×480); the real V1_01_easy microbench runs only when the
// dataset is available (env var CORTEX_EUROC_V101), so this test stays
// green in CI without the ~1.5 GB sequence.

#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/eval/latency_budget.hpp>
#include <branes/sdk/vio_estimator.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ev = branes::sdk::eval;
using T = double;
using Backend = bs::MsckfBackend<T>;
using Estimator = bs::VioEstimator<T, Backend>;

constexpr std::size_t kW = 752;  // EuRoC cam0 resolution
constexpr std::size_t kH = 480;

// EuRoC-sized synthetic frame: bright 8×8 squares on a 48px grid, shifted.
branes::cv::OwnedImage<std::uint8_t> render(double ox, double oy) {
    branes::cv::OwnedImage<std::uint8_t> img(kW, kH, 0);
    for (std::size_t gy = 48; gy + 12 < kH; gy += 48)
        for (std::size_t gx = 48; gx + 12 < kW; gx += 48) {
            const auto x0 = static_cast<long>(gx + ox);
            const auto y0 = static_cast<long>(gy + oy);
            for (long dy = 0; dy < 8; ++dy)
                for (long dx = 0; dx < 8; ++dx) {
                    const long px = x0 + dx, py = y0 + dy;
                    if (px >= 0 && py >= 0 && px < static_cast<long>(kW) && py < static_cast<long>(kH))
                        img(static_cast<std::size_t>(py), static_cast<std::size_t>(px)) = 255;
                }
        }
    return img;
}

bs::ImuMeasurement<T> level_sample(double t) {
    bs::ImuMeasurement<T> m;
    m.timestamp_s = t;
    m.angular_velocity = bs::Vec3<T>{{0, 0, 0}};
    m.linear_acceleration = bs::Vec3<T>{{0, 0, 9.81}};
    return m;
}

Estimator make_estimator() {
    Backend::CameraCalibration cal;
    cal.intrinsics =
        Backend::Camera(458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    return Estimator(Backend(std::vector<Backend::CameraCalibration>{cal}));
}

double ms_since(std::chrono::steady_clock::time_point t0) {
    const auto dt = std::chrono::steady_clock::now() - t0;
    return std::chrono::duration<double, std::milli>(dt).count();
}

}  // namespace

TEST_CASE("percentile picks nearest-rank order statistics", "[sdk][vio][latency]") {
    std::vector<double> v = {5, 1, 3, 2, 4};  // sorts to 1..5
    // Nearest-rank (⌈p·n⌉): p=0→min, p=1→max, p=0.5→⌈2.5⌉=3rd=3, p=0.99→5th=5.
    REQUIRE_THAT(ev::percentile(v, 0.0), Catch::Matchers::WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(ev::percentile(v, 1.0), Catch::Matchers::WithinAbs(5.0, 1e-12));
    REQUIRE_THAT(ev::percentile(v, 0.5), Catch::Matchers::WithinAbs(3.0, 1e-12));
    REQUIRE_THAT(ev::percentile(v, 0.99), Catch::Matchers::WithinAbs(5.0, 1e-12));
    REQUIRE_THROWS_AS(ev::percentile(std::vector<double>{}, 0.5), std::invalid_argument);
}

TEST_CASE("feed_image stays within the per-frame latency budget", "[sdk][vio][latency]") {
    auto est = make_estimator();
    bs::VioConfig cfg;
    cfg.max_clones = 11;
    est.configure(cfg);
    est.activate();

    // Initialize from a stationary IMU window.
    double t = 0.0;
    std::vector<bs::ImuMeasurement<T>> init;
    for (int k = 0; k < 80; ++k, t += 0.005)
        init.push_back(level_sample(t));
    est.feed_imu(std::span<const bs::ImuMeasurement<T>>{init});
    REQUIRE(est.backend().initialized());

    // A few warm-up frames (first-touch allocations) excluded from timing.
    for (int f = 0; f < 3; ++f, t += 0.05) {
        const auto img = render(1.0 * f, 0.5 * f);
        est.feed_image(t, img.view());
    }

    std::vector<double> latencies_ms;
    constexpr int kFrames = 60;
    for (int f = 3; f < 3 + kFrames; ++f) {
        std::vector<bs::ImuMeasurement<T>> imus;
        for (int k = 0; k < 10; ++k, t += 0.005)
            imus.push_back(level_sample(t));
        est.feed_imu(std::span<const bs::ImuMeasurement<T>>{imus});

        const auto img = render(1.0 * f, 0.5 * f);
        const auto t0 = std::chrono::steady_clock::now();
        est.feed_image(t, img.view());
        latencies_ms.push_back(ms_since(t0));
    }

    const double budget = ev::FrameLatencyBudget<T>::per_frame_ms;
    const double median = ev::percentile(latencies_ms, 0.5);
    const double p99 = ev::percentile(latencies_ms, 0.99);
    INFO("median = " << median << " ms, p99 = " << p99 << " ms, budget = " << budget << " ms");
#ifdef NDEBUG
    REQUIRE(median <= budget);
    REQUIRE(p99 <= budget);
#else
    // A real-time budget is only meaningful for an optimized build; CI
    // builds Debug (-O0), so enforce it only under NDEBUG.
    SKIP("per-frame latency budget is enforced only in optimized (NDEBUG) builds");
#endif
}

TEST_CASE("V1_01_easy feed_image latency is within budget", "[sdk][vio][latency][dataset]") {
    const char* env = std::getenv("CORTEX_EUROC_V101");
    if (env == nullptr || std::string(env).empty())
        SKIP("set CORTEX_EUROC_V101 to the V1_01_easy/mav0 directory to run this benchmark");

    const std::string root(env);
    const auto imu = bs::euroc::parse_imu<T>(root);
    const auto images = bs::euroc::parse_images(root);

    auto est = make_estimator();
    bs::VioConfig cfg;
    cfg.max_clones = 11;
    est.configure(cfg);
    est.activate();

    std::vector<double> latencies_ms;
    std::size_t imu_idx = 0;
    int frame = 0;
    for (const auto& f : images) {
        if (frame++ >= 400)
            break;  // a stable subset is enough to characterize latency
        std::size_t end = imu_idx;
        while (end < imu.size() && imu[end].timestamp_s <= f.t_s)
            ++end;
        if (end > imu_idx) {
            est.feed_imu(std::span<const bs::ImuMeasurement<T>>{imu.data() + imu_idx, end - imu_idx});
            imu_idx = end;
        }
        const auto img = branes::cv::read_png(f.path);
        // An empty image makes feed_image return early (~0 ms), which would
        // falsely satisfy the budget — don't time it.
        if (img.view().empty())
            continue;
        const auto t0 = std::chrono::steady_clock::now();
        est.feed_image(f.t_s, img.view());
        if (frame > 5)  // skip warm-up frames
            latencies_ms.push_back(ms_since(t0));
    }
    REQUIRE(latencies_ms.size() > 50);

    const double budget = ev::FrameLatencyBudget<T>::per_frame_ms;
    const double median = ev::percentile(latencies_ms, 0.5);
    const double p99 = ev::percentile(latencies_ms, 0.99);
    INFO("EuRoC median = " << median << " ms, p99 = " << p99 << " ms, budget = " << budget << " ms");
#ifdef NDEBUG
    REQUIRE(median <= budget);
    REQUIRE(p99 <= budget);
#else
    SKIP("per-frame latency budget is enforced only in optimized (NDEBUG) builds");
#endif
}
