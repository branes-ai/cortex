// The S1 seed's tilt-accelerometer-bias prior (issue #609).
//
// A static window measures the accelerometer's mean specific force
// f = R^T g e_z + b_a. Taking "up" from it with b_a = 0 makes the tilt error and
// the horizontal bias one quantity, seen once: the seed must say so in P0
// (stages::s1_initialization::TiltBiasPrior), and the filter then learns the
// bias as the platform rotates. Before #609 P0 was an isotropic sigma = 0.1
// with no correlation, and a 0.55 m/s^2 bias (EuRoC V1_01) left both filters
// tilted by 2.4-2.6 deg for the whole run.
//
// ASCII-only TEST_CASE names (the Windows MSVC ctest job rejects non-ASCII).

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/imu_init.hpp>
#include <branes/sdk/msckf/sqrt_covariance.hpp>
#include <branes/sdk/msckf/stages/s1_initialization.hpp>
#include <branes/sdk/msckf/state.hpp>
#include <branes/sdk/msckf_backend.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <random>
#include <span>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ms = branes::sdk::msckf;
namespace s1 = branes::sdk::msckf::stages::s1_initialization;
namespace ld = branes::math::lie::detail;
using T = double;
using Vec3 = ld::Vec<T, 3>;
using SO3 = branes::math::lie::SO3<T>;
using State = ms::State<T>;

constexpr T kG = 9.81;
constexpr T kRate = 200.0;      // Hz
constexpr std::size_t kN = 50;  // the backend's static window
constexpr T kDensity = 2.0e-3;  // VioConfig::accel_noise_density
constexpr T kSigmaBa = 0.2;     // VioConfig::accel_bias_prior_sigma

// Gaussian draws from the engine's raw output (Box-Muller): std::normal_distribution
// is implementation-defined, and the draws must match across platforms (#551).
struct Gauss {
    std::mt19937_64 eng;
    explicit Gauss(std::uint64_t seed) : eng(seed) {}
    double operator()() {
        constexpr double k = 1.0 / 18446744073709551616.0;  // 2^-64
        const double u1 = (static_cast<double>(eng()) + 0.5) * k;
        const double u2 = (static_cast<double>(eng()) + 0.5) * k;
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
    }
};

// One static seed: the window the accelerometer reads at R_true with bias b_a,
// seeded the way the backend seeds it. Empty when the static gate rejects the
// window (|f| more than 5% from g).
std::optional<State>
try_seed_static(const SO3& R_true, const Vec3& ba, Gauss& n, bs::InitDiagnostics<T>& diag, T g_local = kG) {
    const T s = kDensity * std::sqrt(kRate);  // per-sample sigma
    const Vec3 f = R_true.inverse() * Vec3{{0, 0, g_local}} + ba;
    std::vector<Vec3> gyro(kN, Vec3{}), accel(kN);
    for (auto& a : accel)
        a = f + Vec3{{s * n(), s * n(), s * n()}};
    bs::ImuInitConfig<T> cfg;
    cfg.gravity_magnitude = kG;
    const bs::ImuInitializer<T> init(cfg);
    const auto r = init.try_static(std::span<const Vec3>{gyro}, std::span<const Vec3>{accel});
    if (!r.success)
        return std::nullopt;
    State st(T{1} / T{10});
    const auto prior = s1::tilt_bias_prior<T>(
        std::span<const Vec3>{accel}, bs::InitMethod::Static, kSigmaBa, kDensity, static_cast<T>(kN) / kRate);
    s1::seed_from_imu(st, r, bs::InitMethod::Static, 0.0, std::span<const Vec3>{accel}, kG, diag, prior);
    return st;
}

State seed_static(const SO3& R_true, const Vec3& ba, Gauss& n, bs::InitDiagnostics<T>& diag) {
    auto st = try_seed_static(R_true, ba, n, diag);
    REQUIRE(st.has_value());
    return *st;
}

// e^T P^-1 e over the tilt (two components orthogonal to the seed's up) and the
// three accelerometer-bias components. The yaw about up is a gauge, and the
// truth's yaw is not the seed's, so the tilt error comes from the two up
// vectors: R = R_hat Exp(d) takes the seed's up u_hat to the truth's u when
// d_perp = -(u_hat x u) phi / sin(phi), phi the angle between them.
double tilt_bias_nees(const State& st, const SO3& R_true, const Vec3& ba_true) {
    const auto P = st.covariance();
    const Vec3 u = st.R.inverse() * Vec3{{0, 0, 1}};
    Vec3 a = ld::cross(u, Vec3{{1, 0, 0}});
    if (ld::norm(a) < 0.1)
        a = ld::cross(u, Vec3{{0, 1, 0}});
    a = a * (1.0 / ld::norm(a));
    const Vec3 b = ld::cross(u, a);
    // Basis rows: tilt along a, b; bias x, y, z.
    std::array<std::array<double, 15>, 5> B{};
    for (std::size_t i = 0; i < 3; ++i) {
        B[0][State::kTheta + i] = a[i];
        B[1][State::kTheta + i] = b[i];
        B[2 + i][State::kBa + i] = 1.0;
    }
    const Vec3 ut = R_true.inverse() * Vec3{{0, 0, 1}};
    const Vec3 c = ld::cross(u, ut);
    const double sphi = ld::norm(c);
    const double phi = std::atan2(sphi, ld::dot(u, ut));
    const Vec3 th = sphi > 0 ? c * (-phi / sphi) : Vec3{};
    const Vec3 db = ba_true - st.ba;
    const std::array<double, 5> e{ld::dot(a, th), ld::dot(b, th), db[0], db[1], db[2]};
    std::array<std::array<double, 5>, 5> S{};
    for (std::size_t i = 0; i < 5; ++i)
        for (std::size_t j = 0; j < 5; ++j)
            for (std::size_t k = 0; k < 15; ++k)
                for (std::size_t l = 0; l < 15; ++l)
                    S[i][j] += B[i][k] * P(k, l) * B[j][l];
    // Solve S y = e by Gaussian elimination (S is SPD, 5 x 5).
    std::array<double, 5> y = e;
    for (std::size_t c = 0; c < 5; ++c) {
        for (std::size_t r = c + 1; r < 5; ++r) {
            const double m = S[r][c] / S[c][c];
            for (std::size_t k = c; k < 5; ++k)
                S[r][k] -= m * S[c][k];
            y[r] -= m * y[c];
        }
    }
    for (std::size_t c = 5; c-- > 0;) {
        for (std::size_t k = c + 1; k < 5; ++k)
            y[c] -= S[c][k] * y[k];
        y[c] /= S[c][c];
    }
    double q = 0;
    for (std::size_t i = 0; i < 5; ++i)
        q += e[i] * y[i];
    return q;
}

}  // namespace

TEST_CASE("S1 prior: P0 is the closed-form tilt-bias block about the seed's up", "[sdk][s1][init][609]") {
    Gauss n(7);
    const SO3 R_true = SO3::exp(Vec3{{0.3, -0.2, 0.9}});
    bs::InitDiagnostics<T> diag;
    const State st = seed_static(R_true, Vec3{{0.05, 0.1, -0.03}}, n, diag);
    const auto P = st.covariance();
    const Vec3 u = st.R.inverse() * Vec3{{0, 0, 1}};
    const auto ux = ld::hat(u);
    const T sb2 = kSigmaBa * kSigmaBa;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            INFO("i = " << i << ", j = " << j);
            // P_theta_b = sigma_ba^2 / g [u]x, and its transpose.
            CHECK(std::abs(P(State::kTheta + i, State::kBa + j) - sb2 / kG * ux(i, j)) < 1e-12);
            CHECK(std::abs(P(State::kBa + j, State::kTheta + i) - sb2 / kG * ux(i, j)) < 1e-12);
            // Nothing else correlates with the tilt: position, velocity, gyro bias.
            for (const std::size_t k : {State::kPos, State::kVel, State::kBg})
                CHECK(P(State::kTheta + i, k + j) == 0.0);
        }
    }
    // The yaw about up keeps the fresh state's variance; the tilt is sigma_ba / g.
    T yaw2 = 0, tilt_trace = 0;
    for (std::size_t i = 0; i < 3; ++i) {
        tilt_trace += P(State::kTheta + i, State::kTheta + i);
        for (std::size_t j = 0; j < 3; ++j)
            yaw2 += u[i] * P(State::kTheta + i, State::kTheta + j) * u[j];
    }
    CHECK(std::abs(yaw2 - 0.01) < 1e-12);
    const T tilt2 = (tilt_trace - yaw2) / 2;
    CHECK(tilt2 > sb2 / (kG * kG));
    CHECK(tilt2 < 1.01 * sb2 / (kG * kG));  // the window's own noise adds under 1%
    // The bias along up is seeded from |f| - g, and the window resolves it to the
    // spread of surface gravity, not to sigma_ba.
    T par2 = 0;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            par2 += u[i] * P(State::kBa + i, State::kBa + j) * u[j];
    CHECK(std::sqrt(par2) < 0.03);
    CHECK(std::sqrt(par2) >= s1::kGravityMagnitudeSigma<T>);
}

TEST_CASE("S1 prior: the square-root covariance carries the same block", "[sdk][s1][init][609]") {
    // apply_tilt_bias_prior maps P through G = L_M L_cur^-1 with cov.transform(G).
    // The square-root backend's transform is S -> G S, retriangularized: the
    // block it carries must be the dense one, entry by entry.
    Gauss n(11);
    const SO3 R_true = SO3::exp(Vec3{{-0.4, 0.25, 2.1}});
    const T s = kDensity * std::sqrt(kRate);
    const Vec3 f = R_true.inverse() * Vec3{{0, 0, kG}} + Vec3{{0.08, -0.12, 0.04}};
    std::vector<Vec3> gyro(kN, Vec3{}), accel(kN);
    for (auto& a : accel)
        a = f + Vec3{{s * n(), s * n(), s * n()}};
    bs::ImuInitConfig<T> cfg;
    cfg.gravity_magnitude = kG;
    const auto r = bs::ImuInitializer<T>(cfg).try_static(std::span<const Vec3>{gyro}, std::span<const Vec3>{accel});
    REQUIRE(r.success);
    const auto prior = s1::tilt_bias_prior<T>(
        std::span<const Vec3>{accel}, bs::InitMethod::Static, kSigmaBa, kDensity, static_cast<T>(kN) / kRate);
    REQUIRE(prior.accel_bias_sigma == kSigmaBa);

    State full(T{1} / T{10});
    ms::State<T, ms::SqrtCovariance<T>> sq(T{1} / T{10});
    bs::InitDiagnostics<T> d1, d2;
    s1::seed_from_imu(full, r, bs::InitMethod::Static, 0.0, std::span<const Vec3>{accel}, kG, d1, prior);
    s1::seed_from_imu(sq, r, bs::InitMethod::Static, 0.0, std::span<const Vec3>{accel}, kG, d2, prior);

    const auto Pf = full.covariance();
    const auto Ps = sq.covariance();
    REQUIRE(Ps.rows == Pf.rows);
    for (std::size_t i = 0; i < Pf.rows; ++i)
        for (std::size_t j = 0; j < Pf.cols; ++j) {
            INFO("i = " << i << ", j = " << j);
            CHECK(std::abs(Ps(i, j) - Pf(i, j)) < 1e-12);
        }
    // And it is the block, not the isotropic prior: the tilt-bias correlation is there.
    CHECK(std::abs(Ps(State::kTheta, State::kBa + 1)) + std::abs(Ps(State::kTheta, State::kBa + 2)) > 1e-4);
}

TEST_CASE("S1 prior: only a static window sets it", "[sdk][s1][init][609]") {
    // A gravity-aligned window moves and a dynamic seed's tilt comes from the
    // alignment: both keep the fresh state's isotropic prior. So does a disabled
    // prior (sigma_ba = 0, the P0 before #609).
    const std::vector<Vec3> accel(kN, Vec3{{0.1, 0.2, kG}});
    const std::span<const Vec3> a{accel};
    const T w = static_cast<T>(kN) / kRate;
    CHECK(s1::tilt_bias_prior<T>(a, bs::InitMethod::Static, kSigmaBa, kDensity, w).accel_bias_sigma == kSigmaBa);
    for (const auto m : {bs::InitMethod::GravityAlign, bs::InitMethod::Dynamic, bs::InitMethod::Identity})
        CHECK(s1::tilt_bias_prior<T>(a, m, kSigmaBa, kDensity, w).accel_bias_sigma == 0.0);
    CHECK(s1::tilt_bias_prior<T>(a, bs::InitMethod::Static, 0.0, kDensity, w).accel_bias_sigma == 0.0);
    State st(T{1} / T{10});
    const auto before = st.covariance().d;
    CHECK_FALSE(s1::apply_tilt_bias_prior(st, kG, s1::TiltBiasPrior<T>{}));
    CHECK(st.covariance().d == before);
}

TEST_CASE("S1 prior: P0 covers a static seed's tilt and bias error (NEES 5)", "[sdk][s1][init][609]") {
    // The truth's bias is drawn from the prior; each seed's 5-dof error (tilt
    // orthogonal to up, the three bias components) must have NEES 5 on average.
    // The static gate bounds the bias along up (|f| within 5% of g), which
    // truncates that component's draws; the window resolves it to 0.026 m/s^2
    // whatever its value, so the truncation does not move the NEES.
    Gauss n(11);
    constexpr int kDraws = 600;
    double sum = 0;
    for (int k = 0; k < kDraws; ++k) {
        const SO3 R_true = SO3::exp(Vec3{{0.4 * n(), 0.4 * n(), 3.0 * n()}});
        bs::InitDiagnostics<T> diag;
        std::optional<State> st;
        Vec3 ba{};
        // The site's gravity is not the configured 9.81: it is drawn from the
        // spread the prior assumes for the bias along up.
        const T g_local = kG + s1::kGravityMagnitudeSigma<T> * n();
        while (!st) {  // a draw the static gate rejects is drawn again
            ba = Vec3{{kSigmaBa * n(), kSigmaBa * n(), kSigmaBa * n()}};
            st = try_seed_static(R_true, ba, n, diag, g_local);
        }
        sum += tilt_bias_nees(*st, R_true, ba);
    }
    const double anees = sum / kDraws;
    INFO("ANEES " << anees << " (target 5; the mean of 600 draws has sigma 0.13)");
    CHECK(anees > 4.5);
    CHECK(anees < 5.5);
}

TEST_CASE("S1 prior: a 0.5 m/s^2 bias leaves the seed tilted, and P0 says how much", "[sdk][s1][init][609]") {
    Gauss n(3);
    const SO3 R_true = SO3::exp(Vec3{{0.05, -0.1, 0.7}});
    const Vec3 ba{{0.0, 0.5, 0.0}};
    bs::InitDiagnostics<T> diag;
    const State st = seed_static(R_true, ba, n, diag);
    const Vec3 u_true = R_true.inverse() * Vec3{{0, 0, 1}};
    const Vec3 u_seed = st.R.inverse() * Vec3{{0, 0, 1}};
    const double tilt = std::atan2(ld::norm(ld::cross(u_true, u_seed)), ld::dot(u_true, u_seed));
    const auto P = st.covariance();
    T tilt_trace = 0, yaw2 = 0;
    for (std::size_t i = 0; i < 3; ++i) {
        tilt_trace += P(State::kTheta + i, State::kTheta + i);
        for (std::size_t j = 0; j < 3; ++j)
            yaw2 += u_seed[i] * P(State::kTheta + i, State::kTheta + j) * u_seed[j];
    }
    const double tilt_sigma = std::sqrt((tilt_trace - yaw2) / 2);
    INFO("seed tilt " << tilt * 57.29578 << " deg, P0 tilt sigma " << tilt_sigma * 57.29578 << " deg");
    CHECK(tilt > 0.04);                            // about 0.5 / g: the bias is in the tilt
    CHECK(tilt < 3.0 * tilt_sigma);                // and P0 covers it
    CHECK(tilt_bias_nees(st, R_true, ba) < 25.0);  // 2.5 sigma on 5 dof, from one draw
}

TEST_CASE("MSCKF backend: a 0.5 m/s^2 accelerometer bias is learned once the motion rotates",
          "[sdk][msckf][backend][609]") {
    // The synthetic world starts at rest (static init fires) and then loops with
    // an oscillating yaw. Its accelerometer bias is 0.5 m/s^2 on y.
    namespace ev = branes::sdk::eval;
    ev::SyntheticConfig<T> cfg;
    cfg.accel_bias = Vec3{{0.0, 0.5, 0.0}};
    cfg.duration_s = 30.0;
    const auto world = ev::generate_world<T>(cfg);

    const auto run = [&](double prior_sigma) {
        using Backend = bs::MsckfBackend<T>;
        Backend::CameraCalibration cal;
        cal.intrinsics = world.camera;
        cal.extrinsics.R_imu_cam = world.R_imu_cam;
        cal.extrinsics.p_imu_cam = world.p_imu_cam;
        Backend backend(std::vector<Backend::CameraCalibration>{cal});
        bs::VioConfig vcfg;
        vcfg.accel_bias_prior_sigma = prior_sigma;
        backend.initialize(vcfg);
        struct Out {
            double tilt_first = -1, sigma_first = 0, tilt_last = 0, ba_y = 0;
        } o;
        std::size_t imu_idx = 0;
        for (std::size_t f = 0; f < world.frames.size(); ++f) {
            const double t = world.frames[f].t;
            const Vec3 u_true = world.gt[f].R.inverse() * Vec3{{0, 0, 1}};
            for (; imu_idx < world.imu.size() && world.imu[imu_idx].timestamp_s <= t; ++imu_idx) {
                backend.process_imu(world.imu[imu_idx]);
                // The seed itself, before propagation grows P (the start is at
                // rest, so the truth's attitude is the same at every warm-up time).
                if (backend.initialized() && o.tilt_first < 0) {
                    const auto& st = backend.state();
                    const Vec3 u = st.R.inverse() * Vec3{{0, 0, 1}};
                    const auto P = st.covariance();
                    T tr = 0, yaw2 = 0;
                    for (std::size_t i = 0; i < 3; ++i) {
                        tr += P(State::kTheta + i, State::kTheta + i);
                        for (std::size_t j = 0; j < 3; ++j)
                            yaw2 += u[i] * P(State::kTheta + i, State::kTheta + j) * u[j];
                    }
                    o.tilt_first = std::atan2(ld::norm(ld::cross(u_true, u)), ld::dot(u_true, u));
                    o.sigma_first = std::sqrt((tr - yaw2) / 2);
                }
            }
            backend.process_camera(t, std::span<const bs::FrontendObservation<T>>{world.frames[f].obs});
            if (!backend.initialized())
                continue;
            const auto& st = backend.state();
            const Vec3 u_est = st.R.inverse() * Vec3{{0, 0, 1}};
            o.tilt_last = std::atan2(ld::norm(ld::cross(u_true, u_est)), ld::dot(u_true, u_est));
            o.ba_y = st.ba[1];
        }
        return o;
    };
    const auto shipped = run(0.2);
    const auto isotropic = run(0.0);
    constexpr double kDeg = 57.29577951308232;
    INFO("window-derived P0: tilt " << shipped.tilt_first * kDeg << " deg (sigma " << shipped.sigma_first * kDeg
                                    << ") -> " << shipped.tilt_last * kDeg << " deg, b_a y " << shipped.ba_y);
    INFO("isotropic P0:      tilt " << isotropic.tilt_first * kDeg << " deg (sigma " << isotropic.sigma_first * kDeg
                                    << ") -> " << isotropic.tilt_last * kDeg << " deg, b_a y " << isotropic.ba_y);
    REQUIRE(world.frames.front().t >= static_cast<double>(cfg.warmup_s) / 2);  // the seed is at rest
    CHECK(shipped.tilt_first > 0.04);                                          // the seed takes the bias as tilt
    CHECK(shipped.tilt_first < 3.0 * shipped.sigma_first);                     // and its P0 covers it
    CHECK(shipped.sigma_first < 0.03);                                         // with sigma_ba / g, about 1.2 deg
    CHECK(shipped.tilt_last < 0.5 / kDeg);                                     // the motion separates them
    CHECK(std::abs(shipped.ba_y - 0.5) < 0.1);                                 // and the bias is learned
    CHECK(shipped.tilt_last < isotropic.tilt_last);
}
