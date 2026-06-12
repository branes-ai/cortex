// MSCKF state machinery tests (issue #43).
//
// Exercises IMU propagation, clone augmentation/marginalization, and an
// EKF update on a noiseless stream, checking that the error-state
// covariance stays symmetric positive-definite throughout and that
// dimensions track the clone window.

#include <branes/sdk/msckf.hpp>

#include <catch2/catch_approx.hpp>
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

// ── S10 online-calibration state layout (issue #332) ──────────────────────
//
// The optional in-state extrinsics block sits right after the IMU. These lock
// the layout invariants: inert when off (bit-for-bit the fixed filter), correct
// offsets/dimension when on, prior seeded on the diagonal, PSD preserved, and
// the offsets stay right as clones are augmented after the calibration block.

TEST_CASE("S10 calibration is inert by default: layout matches the fixed filter", "[sdk][s10][state]") {
    ms::State<T> s(1.0);
    REQUIRE(s.calib.empty());
    REQUIRE(s.calib_dim() == 0);
    REQUIRE(s.dim() == ms::State<T>::kImuDim);
    REQUIRE(s.clone_offset(0) == ms::State<T>::kImuDim);  // unchanged
    REQUIRE(s.covariance().rows == ms::State<T>::kImuDim);
}

TEST_CASE("S10 enable_calibration appends a primed block after the IMU", "[sdk][s10][state]") {
    using Calib = ms::State<T>::CalibState;
    ms::State<T> s(1.0);
    const T rot_sigma = 0.0175;                                        // ~1°
    const T trans_sigma = 0.01;                                        // 1 cm
    s.enable_calibration({Calib{}, Calib{}}, rot_sigma, trans_sigma);  // 2 cameras

    // Layout: IMU(15) + 2×6 calibration, no clones yet.
    REQUIRE(s.calib.size() == 2);
    REQUIRE(s.calib_dim() == 12);
    REQUIRE(s.dim() == ms::State<T>::kImuDim + 12);
    REQUIRE(s.calib_offset(0) == ms::State<T>::kImuDim);
    REQUIRE(s.calib_offset(1) == ms::State<T>::kImuDim + 6);
    REQUIRE(s.clone_offset(0) == ms::State<T>::kImuDim + 12);  // shifted past calibration

    const auto P = s.covariance();
    REQUIRE(P.rows == ms::State<T>::kImuDim + 12);
    REQUIRE(ms::is_positive_definite(P));
    // Prior landed on the calibration diagonal (rotation then translation).
    const std::size_t c0 = s.calib_offset(0);
    REQUIRE(P(c0, c0) == Catch::Approx(rot_sigma * rot_sigma));
    REQUIRE(P(c0 + 3, c0 + 3) == Catch::Approx(trans_sigma * trans_sigma));
    // The IMU block is untouched by enabling calibration.
    REQUIRE(P(ms::State<T>::kTheta, ms::State<T>::kTheta) == Catch::Approx(1.0));
}

TEST_CASE("S10 clone augmentation lands after the calibration block", "[sdk][s10][state]") {
    using Calib = ms::State<T>::CalibState;
    ms::State<T> s(1.0);
    s.enable_calibration({Calib{}}, 0.0175, 0.01);  // 1 camera
    ms::StateHelper<T>::augment_clone(s);
    ms::StateHelper<T>::augment_clone(s);

    REQUIRE(s.clones.size() == 2);
    REQUIRE(s.dim() == ms::State<T>::kImuDim + 6 + 2 * 6);
    REQUIRE(s.calib_offset(0) == ms::State<T>::kImuDim);      // calibration fixed in place
    REQUIRE(s.clone_offset(0) == ms::State<T>::kImuDim + 6);  // first clone after calibration
    REQUIRE(s.clone_offset(1) == ms::State<T>::kImuDim + 6 + 6);
    // A just-cloned pose is an exact copy ⇒ covariance is PSD (rank-deficient)
    // until propagation adds independent information.
    REQUIRE(ms::is_positive_semidefinite(s.covariance()));
    // Marginalizing the oldest clone leaves the calibration block intact.
    ms::StateHelper<T>::marginalize_clone(s, 0);
    REQUIRE(s.clones.size() == 1);
    REQUIRE(s.calib.size() == 1);
    REQUIRE(s.calib_offset(0) == ms::State<T>::kImuDim);
    REQUIRE(ms::is_positive_semidefinite(s.covariance()));
}
