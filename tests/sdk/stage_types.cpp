// Per-stage arithmetic types on the MSCKF backend (issue #450): a stage-type
// policy names the type each stage computes in, while the state is stored in
// one type. Covers the conversion between types (msckf/rebind.hpp), that the
// default policy is the single-type backend, and the estimator end to end with
// each stage in turn in float, and with mixed storage and update types.
//
// ASCII-only TEST_CASE names (the Windows MSVC ctest job rejects non-ASCII).

#include <branes/sdk/msckf/rebind.hpp>
#include <branes/sdk/msckf/stage_types.hpp>
#include <branes/sdk/vio_estimator.hpp>
#include <branes/test/synthetic_frames.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <universal/number/posit/posit.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ms = branes::sdk::msckf;

using posit32 = sw::universal::posit<32, 2>;
using posit16 = sw::universal::posit<16, 1>;

constexpr double kEpsF = std::numeric_limits<float>::epsilon();
constexpr double kEpsD = std::numeric_limits<double>::epsilon();

// A state with calibration, two clones and a dense SPD covariance.
ms::State<double> sample_state() {
    using St = ms::State<double>;
    St s(0.1);
    s.R = St::SO3::exp({{0.3, -0.2, 0.5}});
    s.p = {{1.0 / 3.0, -2.0, 0.7}};
    s.v = {{0.1, 0.2, -0.3}};
    s.bg = {{1e-3, -2e-3, 3e-4}};
    s.ba = {{0.01, 0.02, -0.03}};
    s.timestamp = 12.345;
    s.calib.push_back({St::SO3::exp({{0.01, 0.02, -0.01}}), {{0.05, -0.01, 0.002}}});
    s.clones.push_back({St::SO3::exp({{0.1, 0.0, 0.2}}), {{0.1, 0.2, 0.3}}, 12.0});
    s.clones.push_back({St::SO3::exp({{-0.1, 0.3, 0.0}}), {{0.4, 0.5, 0.6}}, 12.2});
    const std::size_t n = s.dim();
    ms::DynMat<double> P(n, n);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = 0; j < n; ++j)
            P(i, j) = (i == j ? 1.0 : 0.0) + 0.01 * std::sin(double(i + 1) * double(j + 1)) / double(n);
    ms::symmetrize(P);
    s.cov.P = P;
    return s;
}

double quat_norm(const auto& r) {
    const auto& q = r.quaternion();
    double n2 = 0.0;
    for (std::size_t i = 0; i < 4; ++i)
        n2 += static_cast<double>(q[i]) * static_cast<double>(q[i]);
    return std::sqrt(n2);
}

// Max relative difference, entry by entry, of two rotations' quaternions.
template <class A, class B>
double quat_diff(const A& a, const B& b) {
    double d = 0.0;
    for (std::size_t i = 0; i < 4; ++i)
        d = std::max(d, std::abs(static_cast<double>(a.quaternion()[i]) - static_cast<double>(b.quaternion()[i])));
    return d;
}

}  // namespace

TEST_CASE("stage types: scalar_cast rounds to the target type", "[sdk][msckf][stage_types]") {
    CHECK(ms::scalar_cast<double>(1.0 / 3.0) == 1.0 / 3.0);
    CHECK(ms::scalar_cast<float>(1.0 / 3.0) == static_cast<float>(1.0 / 3.0));
    CHECK(ms::scalar_cast<double>(ms::scalar_cast<long double>(0.1)) == 0.1);
    // Posit to posit goes through long double.
    const posit32 a(0.375);
    CHECK(static_cast<double>(ms::scalar_cast<posit16>(a)) == 0.375);
    CHECK(static_cast<double>(ms::scalar_cast<posit32>(1.5f)) == 1.5);
}

TEST_CASE("stage types: a state through a wider type comes back to double", "[sdk][msckf][stage_types]") {
    const auto s = sample_state();
    const auto w = ms::rebind<long double>(s);
    static_assert(std::is_same_v<std::remove_cvref_t<decltype(w.cov)>, ms::FullCovariance<long double>>);
    const auto back = ms::rebind<double>(w);
    // Means and covariance are exact: long double holds every double.
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK(back.p[i] == s.p[i]);
        CHECK(back.v[i] == s.v[i]);
        CHECK(back.bg[i] == s.bg[i]);
        CHECK(back.ba[i] == s.ba[i]);
    }
    CHECK(back.cov.P.d == s.cov.P.d);
    CHECK(back.timestamp == s.timestamp);
    REQUIRE(back.clones.size() == 2);
    CHECK(back.clones[1].timestamp == s.clones[1].timestamp);
    REQUIRE(back.calib.size() == 1);
    // Rotations are renormalized in each type: within an ulp of double.
    CHECK(quat_diff(back.R, s.R) <= kEpsD);
    CHECK(quat_diff(back.clones[0].R, s.clones[0].R) <= kEpsD);
    CHECK(quat_diff(back.calib[0].R_imu_cam, s.calib[0].R_imu_cam) <= kEpsD);
}

TEST_CASE("stage types: a state rebound to float is float's rounding of it", "[sdk][msckf][stage_types]") {
    const auto s = sample_state();
    const auto f = ms::rebind<float>(s);
    for (std::size_t i = 0; i < 3; ++i)
        CHECK(f.p[i] == static_cast<float>(s.p[i]));
    for (std::size_t i = 0; i < s.cov.P.d.size(); ++i)
        CHECK(f.cov.P.d[i] == static_cast<float>(s.cov.P.d[i]));
    // Unit in float, and within a few float ulps of the double rotation.
    CHECK(std::abs(quat_norm(f.R) - 1.0) <= 4 * kEpsF);
    CHECK(quat_diff(f.R, s.R) <= 2 * kEpsF);
    CHECK(f.dim() == s.dim());
}

TEST_CASE("stage types: a square-root covariance rebinds its factor", "[sdk][msckf][stage_types]") {
    ms::State<double, ms::SqrtCovariance<double>> s(0.5);
    s.cov.S(1, 0) = 1.0 / 3.0;
    const auto f = ms::rebind<float>(s);
    static_assert(std::is_same_v<std::remove_cvref_t<decltype(f.cov)>, ms::SqrtCovariance<float>>);
    CHECK(f.cov.S(0, 0) == 0.5f);
    CHECK(f.cov.S(1, 0) == static_cast<float>(1.0 / 3.0));
    CHECK(ms::rebind<double>(f).cov.S(1, 0) == static_cast<double>(static_cast<float>(1.0 / 3.0)));
}

TEST_CASE("stage types: the uniform policy is the single-type backend", "[sdk][msckf][stage_types]") {
    static_assert(ms::StageTypePolicy<ms::UniformStageTypes<double>>);
    static_assert(
        std::is_same_v<bs::MsckfMixedBackend<double, ms::UniformStageTypes<double>>, bs::MsckfBackend<double>>);
    static_assert(std::is_same_v<bs::MsckfBackend<float>::StageTypes, ms::UniformStageTypes<float>>);
    SUCCEED();
}

// ── End to end ──────────────────────────────────────────────────────────────

namespace {

// One stage in float, the rest (and the storage) in double.
struct S0Float : ms::UniformStageTypes<double> {
    using sensor_model = float;
};
struct S2Float : ms::UniformStageTypes<double> {
    using propagation = float;
};
struct S3Float : ms::UniformStageTypes<double> {
    using augmentation = float;
};
struct S6Float : ms::UniformStageTypes<double> {
    using update = float;
};
struct S9Float : ms::UniformStageTypes<double> {
    using marginalization = float;
};
struct S10Float : ms::UniformStageTypes<double> {
    using calibration = float;
};
// Every stage in long double, stored in double.
struct AllLongDouble {
    using sensor_model = long double;
    using propagation = long double;
    using augmentation = long double;
    using update = long double;
    using marginalization = long double;
    using calibration = long double;
};
// Stored in float, the update in double.
struct FloatUpdateDouble : ms::UniformStageTypes<float> {
    using update = double;
};

bs::VioConfig drive_config() {
    bs::VioConfig cfg;
    cfg.max_clones = 5;
    cfg.estimate_extrinsics = true;  // so S10 runs
    return cfg;
}

// The ε of the narrowest type the state passes through: storage or any stage.
template <class S, class P>
double narrowest_eps() {
    const auto e = [](auto x) { return static_cast<double>(std::numeric_limits<decltype(x)>::epsilon()); };
    return std::max({e(S{}),
                     e(typename P::sensor_model{}),
                     e(typename P::propagation{}),
                     e(typename P::augmentation{}),
                     e(typename P::update{}),
                     e(typename P::marginalization{}),
                     e(typename P::calibration{})});
}

// PSD to the precision of the narrowest type the state went through: a clone
// makes P exactly singular, and a stage in float leaves its null directions at
// −ε_float·‖P‖, below the storage type's own slack (16·ε_T·max diag).
template <class S>
bool psd_to(const ms::DynMat<S>& P, double eps) {
    double scale = 0.0;
    for (std::size_t i = 0; i < P.rows; ++i)
        scale = std::max(scale, std::abs(static_cast<double>(P(i, i))));
    return ms::is_positive_semidefinite(P, static_cast<S>(16.0 * eps * scale));
}

struct Run {
    std::vector<std::array<double, 3>> positions;
    std::size_t accepted = 0;
    bool psd = true;
};

template <class S, class B>
Run drive() {
    auto est = branes::test::make_estimator<S, B>();
    est.configure(drive_config());
    est.activate();
    Run run;
    est.backend().set_update_observer(
        [&](const auto&, const auto&, const auto&, bool ok, const auto&) { run.accepted += ok ? 1 : 0; });
    branes::test::drive_estimator(est, 8, [&](int, double) {
        REQUIRE(est.backend().initialized());
        run.psd = run.psd && psd_to(est.backend().state().covariance(), narrowest_eps<S, typename B::StageTypes>());
        const auto p = est.current_pose().translation();
        run.positions.push_back({static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2])});
    });
    return run;
}

const Run& double_run() {
    static const Run r = drive<double, bs::MsckfBackend<double>>();
    return r;
}

double worst_vs_double(const Run& r) {
    const auto& ref = double_run().positions;
    REQUIRE(r.positions.size() == ref.size());
    double worst = 0.0;
    for (std::size_t f = 0; f < ref.size(); ++f) {
        const double e =
            std::hypot(r.positions[f][0] - ref[f][0], r.positions[f][1] - ref[f][1], r.positions[f][2] - ref[f][2]);
        REQUIRE(std::isfinite(e));
        worst = std::max(worst, e);
    }
    return worst;
}

}  // namespace

TEMPLATE_TEST_CASE("stage types: one stage in float, the rest in double",
                   "[sdk][msckf][stage_types]",
                   S0Float,
                   S2Float,
                   S3Float,
                   S6Float,
                   S9Float,
                   S10Float) {
    const auto& ref = double_run();
    REQUIRE(ref.accepted > 0);
    const Run r = drive<double, bs::MsckfMixedBackend<double, TestType>>();
    const double worst = worst_vs_double(r);
    INFO("worst |p - p_double| " << worst << " m");
    CHECK(r.psd);
    CHECK(r.accepted > 0);
    // The stage really ran in float: the run is not the double run ...
    CHECK(worst > 0.0);
    // ... and is as close to it as the all-float run (1000 float ulps at the
    // scene's 1 m unit; tests/sdk/vio_estimator.cpp). Measured: update 2.4e-5 m,
    // propagation 9.6e-6, augmentation 2.8e-7, sensor model 9.2e-8,
    // marginalization 1.9e-9, calibration 3.1e-10 (bound 1.2e-4).
    CHECK(worst <= 1e3 * kEpsF);
}

TEST_CASE("stage types: every stage in long double, stored in double", "[sdk][msckf][stage_types]") {
    const Run r = drive<double, bs::MsckfMixedBackend<double, AllLongDouble>>();
    const double worst = worst_vs_double(r);
    INFO("worst |p - p_double| " << worst << " m");
    CHECK(r.psd);
    CHECK(r.accepted == double_run().accepted);
    // Within double's own rounding, amplified by the filter. Measured 1.6e-14 m
    // (bound 2.2e-13).
    CHECK(worst <= 1e3 * kEpsD);
}

TEST_CASE("stage types: stored in float, the update in double", "[sdk][msckf][stage_types]") {
    const Run r = drive<float, bs::MsckfMixedBackend<float, FloatUpdateDouble>>();
    const double worst = worst_vs_double(r);
    INFO("worst |p - p_double| " << worst << " m");
    CHECK(r.psd);
    CHECK(r.accepted > 0);
    CHECK(worst <= 1e3 * kEpsF);  // measured 9.1e-6 m
}
