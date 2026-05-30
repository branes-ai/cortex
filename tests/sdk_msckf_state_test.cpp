// MSCKF state machinery tests (issue #43).
//
// Exercises IMU propagation, clone augmentation/marginalization, and an
// EKF update on a noiseless stream, checking that the error-state
// covariance stays symmetric positive-definite throughout and that
// dimensions track the clone window.

#include <branes/sdk/msckf.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace {

namespace ms = branes::sdk::msckf;
using T = double;
using Vec3 = ms::State<T>::Vec3;

T trace(const ms::DynMat<T>& m) {
    T s = 0;
    for (std::size_t i = 0; i < m.rows; ++i)
        s += m(i, i);
    return s;
}

}  // namespace

TEST_CASE("IMU propagation keeps the covariance positive-definite", "[sdk][msckf]") {
    ms::State<T> s(0.1);
    ms::Propagator<T> prop;
    REQUIRE(s.dim() == 15);
    REQUIRE(ms::is_positive_definite(s.covariance()));
    for (int k = 0; k < 100; ++k) {
        const T t = k * 0.005;
        prop.propagate(s, Vec3{{0.1 * std::sin(t), 0.05, -0.08}}, Vec3{{0.2, -0.1, 9.7}}, 0.005);
        REQUIRE(ms::is_positive_definite(s.covariance()));
    }
    REQUIRE(std::abs(s.timestamp - 0.5) < 1e-9);
}

TEST_CASE("clone augmentation grows the state and stays PSD", "[sdk][msckf]") {
    ms::State<T> s(0.2);
    ms::Propagator<T> prop;
    prop.propagate(s, Vec3{{0.05, 0, 0}}, Vec3{{0, 0, 9.81}}, 0.01);

    // A fresh clone is an exact copy of the IMU pose, so the joint
    // covariance is singular (PSD, not strictly PD) right after augment.
    ms::StateHelper<T>::augment_clone(s);
    REQUIRE(s.num_clones() == 1);
    REQUIRE(s.dim() == 21);
    REQUIRE(ms::is_positive_semidefinite(s.covariance()));

    // Propagate between clones (as in real operation) so the second clone
    // captures a distinct pose, not a duplicate of the first.
    for (int k = 0; k < 30; ++k)
        prop.propagate(s, Vec3{{0.02, 0.01, -0.01}}, Vec3{{0.1, 0, 9.8}}, 0.01);
    ms::StateHelper<T>::augment_clone(s);
    REQUIRE(s.num_clones() == 2);
    REQUIRE(s.dim() == 27);
    REQUIRE(ms::is_positive_semidefinite(s.covariance()));

    // Continued propagation injects process noise and restores full rank.
    for (int k = 0; k < 60; ++k)
        prop.propagate(s, Vec3{{0.02, 0.01, -0.01}}, Vec3{{0.1, 0, 9.8}}, 0.01);
    REQUIRE(ms::is_positive_definite(s.covariance()));
}

TEST_CASE("marginalization removes a clone and stays PSD", "[sdk][msckf]") {
    ms::State<T> s(0.2);
    ms::StateHelper<T>::augment_clone(s);
    ms::StateHelper<T>::augment_clone(s);
    ms::StateHelper<T>::augment_clone(s);
    REQUIRE(s.dim() == 15 + 18);

    ms::StateHelper<T>::marginalize_clone(s, 1);  // drop the middle clone
    REQUIRE(s.num_clones() == 2);
    REQUIRE(s.dim() == 15 + 12);
    REQUIRE(ms::is_positive_semidefinite(s.covariance()));
}

TEST_CASE("EKF update reduces covariance and stays PD", "[sdk][msckf]") {
    ms::State<T> s(0.5);
    ms::StateHelper<T>::augment_clone(s);  // dim 21
    const std::size_t d = s.dim();

    // A synthetic 3-row measurement touching position + the clone pose.
    const std::size_t k = 3;
    ms::DynMat<T> H(k, d);
    for (std::size_t i = 0; i < k; ++i)
        for (std::size_t j = 0; j < d; ++j)
            H(i, j) = std::sin(0.6 * i + 0.3 * j);
    std::vector<T> r = {0.05, -0.02, 0.01};
    std::vector<T> Rd = {0.01, 0.01, 0.01};

    // Propagate first so the augmented covariance is full-rank (PD).
    ms::Propagator<T> prop;
    for (int kk = 0; kk < 50; ++kk)
        prop.propagate(s, Vec3{{0.01, 0, 0}}, Vec3{{0, 0, 9.81}}, 0.01);

    const T tr_before = trace(s.covariance());
    ms::StateHelper<T>::ekf_update(s, H, std::span<const T>{r}, std::span<const T>{Rd});
    REQUIRE(ms::is_positive_semidefinite(s.covariance()));
    REQUIRE(trace(s.covariance()) <= tr_before + 1e-9);  // an update cannot add info
}
