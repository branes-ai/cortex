// VioBackend interface tests (issue #34).
//
// The acceptance for #34 is that two different backends — an MSCKF stub
// and a sliding-window-optimization stub — both satisfy the same
// VioBackend interface and can be driven by a single, backend-agnostic
// front-end loop without modification. This test encodes exactly that:
// one templated `run_frontend` driver feeds IMU + camera measurements to
// any VioBackend<T>, and both stubs are exercised through it.

#include <branes/sdk/vio_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace {

using branes::sdk::FrontendObservation;
using branes::sdk::ImuMeasurement;
using branes::sdk::NavState;
using branes::sdk::VioBackend;
using branes::sdk::VioConfig;

// Minimal MSCKF stub: records how many measurements it has seen and
// advances a trivial dead-reckoned position so current_state() changes.
template <class T>
class MsckfStub final : public VioBackend<T> {
public:
    void initialize(const VioConfig& config) override {
        initialized_ = true;
        max_clones_ = config.max_clones;
    }
    void process_imu(const ImuMeasurement<T>& imu) override {
        ++imu_count_;
        last_t_ = imu.timestamp_s;
    }
    void process_camera(double timestamp_s, std::span<const FrontendObservation<T>> obs) override {
        ++cam_count_;
        obs_count_ += obs.size();
        last_t_ = timestamp_s;
    }
    [[nodiscard]] NavState<T> current_state() const override {
        NavState<T> s;
        s.timestamp_s = last_t_;
        return s;
    }

    bool initialized_ = false;
    int max_clones_ = 0;
    std::size_t imu_count_ = 0;
    std::size_t cam_count_ = 0;
    std::size_t obs_count_ = 0;
    double last_t_ = 0.0;
};

// Sliding-window-optimization stub: a different implementation of the
// same interface (here a no-op estimator) — proves the front end is
// decoupled from the backend choice.
template <class T>
class SwOptStub final : public VioBackend<T> {
public:
    void initialize(const VioConfig&) override {
        initialized_ = true;
    }
    void process_imu(const ImuMeasurement<T>&) override {
        ++imu_count_;
    }
    void process_camera(double, std::span<const FrontendObservation<T>>) override {
        ++cam_count_;
    }
    [[nodiscard]] NavState<T> current_state() const override {
        return NavState<T>{};
    }

    bool initialized_ = false;
    std::size_t imu_count_ = 0;
    std::size_t cam_count_ = 0;
};

// The backend-agnostic front-end driver: it knows nothing about the
// concrete backend type, only the VioBackend<T> interface.
template <class T>
void run_frontend(VioBackend<T>& backend) {
    backend.initialize(VioConfig{});
    // Two IMU samples then a camera frame, repeated.
    for (int frame = 0; frame < 3; ++frame) {
        for (int k = 0; k < 5; ++k) {
            ImuMeasurement<T> imu;
            imu.timestamp_s = frame * 0.1 + k * 0.02;
            imu.angular_velocity = {T{0}, T{0}, T{0}};
            imu.linear_acceleration = {T{0}, T{0}, T{9.81}};
            backend.process_imu(imu);
        }
        std::vector<FrontendObservation<T>> obs = {
            {/*id*/ 1, /*cam*/ 0, T{100}, T{120}},
            {/*id*/ 2, /*cam*/ 0, T{200}, T{220}},
        };
        backend.process_camera(frame * 0.1 + 0.1, std::span<const FrontendObservation<T>>{obs});
    }
}

}  // namespace

TEST_CASE("MSCKF stub is driven through the VioBackend interface", "[sdk][vio]") {
    MsckfStub<double> backend;
    run_frontend(backend);
    REQUIRE(backend.initialized_);
    REQUIRE(backend.max_clones_ == VioConfig{}.max_clones);
    REQUIRE(backend.imu_count_ == 15);  // 3 frames * 5 samples
    REQUIRE(backend.cam_count_ == 3);
    REQUIRE(backend.obs_count_ == 6);  // 3 frames * 2 observations
    REQUIRE(backend.current_state().timestamp_s > 0.0);
}

TEST_CASE("SW-opt stub is driven through the same front-end unchanged", "[sdk][vio]") {
    SwOptStub<double> backend;
    run_frontend(backend);  // identical driver, different backend
    REQUIRE(backend.initialized_);
    REQUIRE(backend.imu_count_ == 15);
    REQUIRE(backend.cam_count_ == 3);
}

TEST_CASE("backends are usable polymorphically via VioBackend<T>*", "[sdk][vio]") {
    MsckfStub<double> msckf;
    SwOptStub<double> swopt;
    VioBackend<double>* backends[] = {&msckf, &swopt};
    for (auto* b : backends) {
        run_frontend(*b);
        REQUIRE(b->current_state().timestamp_s >= 0.0);
    }
    REQUIRE(msckf.imu_count_ == 15);
    REQUIRE(swopt.imu_count_ == 15);
}

TEST_CASE("VioBackend instantiates for float as well as double", "[sdk][vio]") {
    MsckfStub<float> backend;
    run_frontend(backend);
    REQUIRE(backend.imu_count_ == 15);
}
