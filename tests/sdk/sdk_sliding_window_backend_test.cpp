// Sliding-window backend skeleton tests (issue #36).
//
// Proves the VioBackend abstraction is real and substitutable: the
// skeleton implements the interface, can stand in for MSCKF behind a
// VioBackend<T> handle and inside a VioEstimator templated on it, and
// surfaces a NotImplemented marker for every estimation operation.

#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/sliding_window_backend.hpp>
#include <branes/sdk/vio_estimator.hpp>

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <span>
#include <type_traits>
#include <vector>

namespace {

namespace bs = branes::sdk;
using T = double;

}  // namespace

TEST_CASE("the skeleton is a substitutable VioBackend", "[sdk][swopt]") {
    // It satisfies the interface and can be held by a base handle.
    std::unique_ptr<bs::VioBackend<T>> backend = std::make_unique<bs::SlidingWindowBackend<T>>();

    bs::VioConfig cfg;
    backend->initialize(cfg);  // accepted (no-op), proves configurability

    bs::ImuMeasurement<T> imu;
    imu.timestamp_s = 0.0;
    REQUIRE_THROWS_AS(backend->process_imu(imu), bs::NotImplemented);

    std::vector<bs::FrontendObservation<T>> obs;
    REQUIRE_THROWS_AS(backend->process_camera(0.0, std::span<const bs::FrontendObservation<T>>{obs}),
                      bs::NotImplemented);

    REQUIRE_THROWS_AS(backend->current_state(), bs::NotImplemented);
}

TEST_CASE("the skeleton reports itself unimplemented", "[sdk][swopt]") {
    bs::SlidingWindowBackend<T> sw;
    REQUIRE_FALSE(sw.implemented());
    STATIC_REQUIRE(bs::SlidingWindowBackend<T>::kImplemented == false);
    // The functional backend, by contrast, estimates and is also a VioBackend.
    STATIC_REQUIRE(bs::MsckfBackend<T>::kImplemented == true);
    STATIC_REQUIRE(std::is_base_of_v<bs::VioBackend<T>, bs::MsckfBackend<T>>);
}

TEST_CASE("a VioEstimator can be templated on the skeleton", "[sdk][swopt]") {
    // Substitutes the skeleton for MSCKF as the estimator's backend. It
    // configures fine; driving it surfaces the NotImplemented response.
    bs::VioEstimator<T, bs::SlidingWindowBackend<T>> est;
    bs::VioConfig cfg;
    est.configure(cfg);
    est.activate();
    REQUIRE(est.lifecycle() == decltype(est)::Lifecycle::Active);

    std::vector<bs::ImuMeasurement<T>> imus(1);
    REQUIRE_THROWS_AS(est.feed_imu(std::span<const bs::ImuMeasurement<T>>{imus}), bs::NotImplemented);
}
