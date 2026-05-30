// Square-root covariance MSCKF backend tests (issue #187).
//
// MsckfSqrtBackend carries the Cholesky factor S of the covariance
// (P = S·Sᵀ) and updates it with a QR (Householder) array algorithm instead
// of the Joseph form. These tests assert the three acceptance properties:
//
//   1. It runs the same synthetic end-to-end scenario as the full-covariance
//      backend and stays stable — covariance PSD, window bounded, state
//      finite, forward motion.
//   2. It produces poses equivalent to the full-covariance backend within a
//      tight numerical tolerance on the *same* input stream (the two forms
//      are algebraically identical; only their rounding differs).
//   3. Its covariance factor stays well-conditioned over a long run — the
//      reconstructed covariance never loses positive semidefiniteness and its
//      trace stays finite and bounded.
//
// The scenario mirrors sdk_msckf_backend_test.cpp: a stationary init, then a
// constant +x acceleration with six world features whose observations are
// projected through the backend's own clone poses (self-consistent).

#include <branes/sdk/msckf_backend.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ms = branes::sdk::msckf;
using T = double;
using FullBackend = bs::MsckfBackend<T>;
using SqrtBackend = bs::MsckfSqrtBackend<T>;
using DVec3 = FullBackend::DVec3;

bs::ImuMeasurement<T> imu_sample(double t, const DVec3& gyro, const DVec3& accel) {
    bs::ImuMeasurement<T> m;
    m.timestamp_s = t;
    m.angular_velocity = bs::Vec3<T>{{gyro[0], gyro[1], gyro[2]}};
    m.linear_acceleration = bs::Vec3<T>{{accel[0], accel[1], accel[2]}};
    return m;
}

bs::VioConfig default_config() {
    bs::VioConfig c;
    c.max_clones = 8;
    return c;
}

T trace(const ms::DynMat<T>& m) {
    T s = T{0};
    for (std::size_t i = 0; i < m.rows; ++i)
        s += m(i, i);
    return s;
}

const std::vector<DVec3> kFeatures = {
    {{1.0, 0.3, 4.0}}, {{1.5, -0.2, 5.0}}, {{2.0, 0.4, 6.0}}, {{0.8, -0.4, 3.5}}, {{2.5, 0.1, 5.5}}, {{1.2, 0.0, 4.5}}};

// One recorded frame: the IMU position and the covariance trace at that frame.
struct Frame {
    std::array<T, 3> pos{};
    T cov_trace = T{0};
    bool psd = false;
};

// Initialize `backend`, run a stationary window, then drive `accel_steps` of
// motion emitting a camera frame every 10 IMU samples. With `oscillate`
// false the platform accelerates steadily along +x (matches the full-cov
// end-to-end test); with it true the platform follows a bounded sinusoid
// aₓ = 2·cos(2t) — zero-mean velocity, so it stays near the features with
// continuous parallax indefinitely, the regime for the long-run conditioning
// check. Returns one Frame per camera frame.
template <class Backend>
std::vector<Frame> run_scenario(Backend& backend, int accel_steps, bool oscillate = false) {
    backend.initialize(default_config());

    double t = 0.0;
    for (int k = 0; k < 80; ++k, t += 0.005)
        backend.process_imu(imu_sample(t, DVec3{{0, 0, 0}}, DVec3{{0, 0, 9.81}}));

    const double t0 = t;
    std::vector<Frame> frames;
    for (int step = 0; step < accel_steps; ++step, t += 0.005) {
        const T ax = oscillate ? static_cast<T>(2.0 * std::cos(2.0 * (t - t0))) : T{1.0};
        backend.process_imu(imu_sample(t, DVec3{{0, 0, 0}}, DVec3{{ax, 0.0, 9.81}}));
        if (step % 10 != 0)
            continue;

        const auto ns = backend.current_state();
        const auto& Rwi = ns.T_world_imu.rotation();
        const auto pwi = ns.T_world_imu.translation();
        std::vector<bs::FrontendObservation<T>> obs;
        for (std::size_t f = 0; f < kFeatures.size(); ++f) {
            const DVec3 pc = Rwi.inverse() * (kFeatures[f] - pwi);  // identity extrinsics
            if (!(pc[2] > 0.0))
                continue;
            bs::FrontendObservation<T> o;
            o.feature_id = f;
            o.camera_id = 0;
            o.u = pc[0] / pc[2];  // identity pinhole ⇒ pixel == normalized
            o.v = pc[1] / pc[2];
            obs.push_back(o);
        }
        backend.process_camera(t, obs);

        const auto p = backend.current_state().T_world_imu.translation();
        const auto cov = backend.state().covariance();
        frames.push_back(Frame{{p[0], p[1], p[2]}, trace(cov), ms::is_positive_semidefinite(cov)});
    }
    return frames;
}

}  // namespace

TEST_CASE("the square-root backend runs end-to-end and stays stable under motion", "[sdk][msckf][sqrt][backend]") {
    SqrtBackend backend;
    const auto frames = run_scenario(backend, 600);

    REQUIRE(frames.size() > 0);
    for (const auto& fr : frames) {
        REQUIRE(fr.psd);  // S·Sᵀ stays positive semidefinite
        for (std::size_t i = 0; i < 3; ++i)
            REQUIRE(std::isfinite(fr.pos[i]));
        REQUIRE(std::isfinite(fr.cov_trace));
    }
    REQUIRE(backend.state().clones.size() <= 8);
    // It produces forward motion (accelerating along +x), not a frozen state.
    REQUIRE(frames.back().pos[0] > frames.front().pos[0] + 0.05);
    REQUIRE(std::isfinite(backend.current_state().velocity_world[0]));
}

TEST_CASE("the square-root backend matches the full-covariance backend within tolerance",
          "[sdk][msckf][sqrt][backend]") {
    FullBackend full;
    SqrtBackend sqrt;
    const auto ff = run_scenario(full, 600);
    const auto sf = run_scenario(sqrt, 600);

    REQUIRE(ff.size() == sf.size());
    REQUIRE(ff.size() > 0);

    // The two forms are algebraically identical on a self-consistent stream;
    // only floating-point rounding separates them. Poses must agree tightly,
    // and the covariance traces must track each other.
    T max_pos_diff = T{0};
    T max_rel_trace_diff = T{0};
    for (std::size_t i = 0; i < ff.size(); ++i) {
        for (std::size_t a = 0; a < 3; ++a)
            max_pos_diff = std::max(max_pos_diff, std::abs(ff[i].pos[a] - sf[i].pos[a]));
        const T denom = std::abs(ff[i].cov_trace) + T{1e-12};
        max_rel_trace_diff = std::max(max_rel_trace_diff, std::abs(ff[i].cov_trace - sf[i].cov_trace) / denom);
    }
    CAPTURE(max_pos_diff, max_rel_trace_diff);
    REQUIRE(max_pos_diff < 1e-8);
    REQUIRE(max_rel_trace_diff < 1e-6);
}

TEST_CASE("the square-root covariance factor stays well-conditioned over a long run", "[sdk][msckf][sqrt][backend]") {
    // A bounded oscillation: thousands of predict / augment / update /
    // marginalize cycles in a well-posed viewing geometry — the regime where
    // a square-root factor is expected to retain conditioning a Joseph-form P
    // can erode.
    SqrtBackend backend;
    const auto frames = run_scenario(backend, 4000, /*oscillate=*/true);

    REQUIRE(frames.size() > 100);
    T max_trace = T{0};
    for (const auto& fr : frames) {
        REQUIRE(fr.psd);                       // never loses PSD over the run
        REQUIRE(std::isfinite(fr.cov_trace));  // never blows up to inf/nan
        max_trace = std::max(max_trace, fr.cov_trace);
    }
    CAPTURE(max_trace);
    // The covariance stays bounded — a diverging filter would grow the trace
    // without limit. This loose bound only catches blow-up, not drift.
    REQUIRE(max_trace < 1e4);
    REQUIRE(ms::is_positive_semidefinite(backend.state().covariance()));
}
