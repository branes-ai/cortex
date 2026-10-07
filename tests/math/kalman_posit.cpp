// Kalman filter over Universal posits — an end-to-end example that exercises
// MTL5 and Universal together (issue #435 follow-up).
//
// A 1-D constant-velocity tracker: state x = [position, velocity], driven by
// white acceleration noise and observed through noisy position measurements.
// The same templated filter runs in double, float, posit<32,2> and
// posit<16,1>, written entirely in MTL5 dense matrix expressions (products,
// transposed views, sums), so a Universal or MTL5 bump that breaks the
// type-generic linear-algebra path fails here.
//
//   predict:  x <- F x              P <- F P F^T + Q
//   update:   S = H P H^T + R       K = P H^T S^-1
//             x <- x + K (z - H x)
//             P <- (I - K H) P (I - K H)^T + K R K^T     (Joseph form)
//
// The Joseph form keeps P symmetric positive-definite under rounding, which
// is what lets the low-precision posit<16,1> run stay stable.
//
// The scenario's noise comes from a self-contained splitmix64 + Box-Muller
// generator, so the measurement sequence is identical on libstdc++, libc++,
// and the MSVC STL (std::normal_distribution is implementation-defined).

#include <catch2/catch_test_macros.hpp>
#include <mtl/mtl.hpp>
#include <universal/number/posit/posit.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <vector>

namespace {

using posit32 = sw::universal::posit<32, 2>;
using posit16 = sw::universal::posit<16, 1>;

constexpr std::size_t kSteps = 200;
constexpr double kDt = 0.1;         // s
constexpr double kSigmaAcc = 0.5;   // m/s^2, process (acceleration) noise
constexpr double kSigmaMeas = 0.5;  // m, position measurement noise

// Deterministic, portable N(0, 1) samples.
class Gaussian {
public:
    explicit Gaussian(std::uint64_t seed) : state_{seed} {}

    double operator()() {
        if (has_spare_) {
            has_spare_ = false;
            return spare_;
        }
        const double u1 = uniform();
        const double u2 = uniform();
        const double r = std::sqrt(-2.0 * std::log(u1));
        const double theta = 2.0 * std::numbers::pi * u2;
        spare_ = r * std::sin(theta);
        has_spare_ = true;
        return r * std::cos(theta);
    }

private:
    // splitmix64 -> uniform in (0, 1], never 0 so log() is finite.
    double uniform() {
        state_ += 0x9e3779b97f4a7c15ULL;
        std::uint64_t z = state_;
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        z ^= z >> 31;
        return (static_cast<double>(z >> 11) + 1.0) * 0x1.0p-53;
    }

    std::uint64_t state_;
    double spare_{0.0};
    bool has_spare_{false};
};

struct Scenario {
    std::vector<double> position;  // truth
    std::vector<double> velocity;  // truth
    std::vector<double> z;         // measured position
};

Scenario make_scenario() {
    Gaussian noise{20261007};
    Scenario s;
    double p = 0.0;
    double v = 1.0;
    for (std::size_t k = 0; k < kSteps; ++k) {
        if (k > 0) {
            const double a = kSigmaAcc * noise();
            p += v * kDt + 0.5 * a * kDt * kDt;
            v += a * kDt;
        }
        s.position.push_back(p);
        s.velocity.push_back(v);
        s.z.push_back(p + kSigmaMeas * noise());
    }
    return s;
}

struct Result {
    double rms_position{};   // filter estimate vs truth [m]
    double rms_velocity{};   // filter estimate vs truth [m/s]
    double max_asymmetry{};  // max |P01 - P10| over the run
    bool positive_definite{true};
};

template <typename T>
Result run_kalman(const Scenario& s) {
    using mtl::trans;
    using mtl::mat::dense2D;
    using mtl::vec::dense_vector;

    const T dt(kDt);
    const T q(kSigmaAcc * kSigmaAcc);
    const T r(kSigmaMeas * kSigmaMeas);

    dense2D<T> F(2, 2);
    F(0, 0) = T(1);
    F(0, 1) = dt;
    F(1, 0) = T(0);
    F(1, 1) = T(1);

    // Discrete white-acceleration process noise.
    dense2D<T> Q(2, 2);
    Q(0, 0) = q * dt * dt * dt * dt / T(4);
    Q(0, 1) = q * dt * dt * dt / T(2);
    Q(1, 0) = Q(0, 1);
    Q(1, 1) = q * dt * dt;

    dense2D<T> H(1, 2);
    H(0, 0) = T(1);
    H(0, 1) = T(0);

    dense2D<T> R(1, 1);
    R(0, 0) = r;

    dense2D<T> I(2, 2);
    I(0, 0) = T(1);
    I(0, 1) = T(0);
    I(1, 0) = T(0);
    I(1, 1) = T(1);

    // Initialize from the first measurement; velocity unknown.
    dense_vector<T> x(2);
    x[0] = T(s.z[0]);
    x[1] = T(0);
    dense2D<T> P(2, 2);
    P(0, 0) = r;
    P(0, 1) = T(0);
    P(1, 0) = T(0);
    P(1, 1) = T(4);

    Result out;
    double sum_p2 = 0.0;
    double sum_v2 = 0.0;
    for (std::size_t k = 1; k < kSteps; ++k) {
        // Predict. Products are materialized into temporaries: MTL5's
        // expression templates must not alias their own destination.
        dense_vector<T> x_pred(2);
        x_pred = F * x;
        dense2D<T> FP(2, 2);
        FP = F * P;
        dense2D<T> P_pred(2, 2);
        P_pred = FP * trans(F);
        P_pred += Q;

        // Update.
        dense2D<T> PHt(2, 1);
        PHt = P_pred * trans(H);
        dense2D<T> S(1, 1);
        S = H * PHt;
        S += R;
        // K = P H^T S^-1, with S^-1 as a 1x1 matrix: the general
        // multi-measurement form, and also the only one that works for
        // Universal types here. MTL5's scalar*matrix operators require
        // std::is_arithmetic_v<S>, so `PHt * s_inv` does not compile for a
        // posit scalar even though posits model MTL5's Scalar concept.
        dense2D<T> S_inv(1, 1);
        S_inv(0, 0) = T(1) / S(0, 0);
        dense2D<T> K(2, 1);
        K = PHt * S_inv;

        dense_vector<T> Hx(1);
        Hx = H * x_pred;
        const T innovation = T(s.z[k]) - Hx[0];
        x[0] = x_pred[0] + K(0, 0) * innovation;
        x[1] = x_pred[1] + K(1, 0) * innovation;

        dense2D<T> KH(2, 2);
        KH = K * H;
        dense2D<T> A(2, 2);
        A = I - KH;
        dense2D<T> AP(2, 2);
        AP = A * P_pred;
        P = AP * trans(A);
        dense2D<T> KR(2, 1);
        KR = K * R;
        dense2D<T> KRKt(2, 2);
        KRKt = KR * trans(K);
        P += KRKt;

        const double ep = double(x[0]) - s.position[k];
        const double ev = double(x[1]) - s.velocity[k];
        sum_p2 += ep * ep;
        sum_v2 += ev * ev;

        const double p00 = double(P(0, 0));
        const double p01 = double(P(0, 1));
        const double p10 = double(P(1, 0));
        const double p11 = double(P(1, 1));
        out.max_asymmetry = std::max(out.max_asymmetry, std::abs(p01 - p10));
        if (!(p00 > 0.0 && p11 > 0.0 && p00 * p11 - p01 * p10 > 0.0)) {
            out.positive_definite = false;
        }
    }
    const auto n = static_cast<double>(kSteps - 1);
    out.rms_position = std::sqrt(sum_p2 / n);
    out.rms_velocity = std::sqrt(sum_v2 / n);
    return out;
}

double raw_measurement_rms(const Scenario& s) {
    double sum = 0.0;
    for (std::size_t k = 1; k < kSteps; ++k) {
        const double e = s.z[k] - s.position[k];
        sum += e * e;
    }
    return std::sqrt(sum / static_cast<double>(kSteps - 1));
}

void report(const char* type, const Result& r) {
    std::printf("  %-14s RMS pos %.6f m   RMS vel %.6f m/s   max|P01-P10| %.3g   PD %s\n",
                type,
                r.rms_position,
                r.rms_velocity,
                r.max_asymmetry,
                r.positive_definite ? "yes" : "NO");
}

struct Runs {
    double raw{};
    Result r64, r32f, rp32, rp16;
};

// Run every arithmetic once and print the comparison table; the SECTIONs
// below re-enter the test case, so cache the runs instead of recomputing.
const Runs& runs() {
    static const Runs cached = [] {
        const Scenario s = make_scenario();
        Runs r{raw_measurement_rms(s),
               run_kalman<double>(s),
               run_kalman<float>(s),
               run_kalman<posit32>(s),
               run_kalman<posit16>(s)};
        std::printf("Kalman CV tracker, %zu steps, raw measurement RMS %.6f m\n", kSteps, r.raw);
        report("double", r.r64);
        report("float", r.r32f);
        report("posit<32,2>", r.rp32);
        report("posit<16,1>", r.rp16);
        return r;
    }();
    return cached;
}

}  // namespace

TEST_CASE("kalman filter tracks a constant-velocity target in IEEE and posit arithmetic", "[math][kalman][posit]") {
    const Runs& r = runs();

    SECTION("double reference filters the measurement noise") {
        REQUIRE(r.r64.positive_definite);
        REQUIRE(r.r64.rms_position < 0.5 * r.raw);
    }
    SECTION("32-bit types match the double reference") {
        REQUIRE(r.r32f.positive_definite);
        REQUIRE(r.rp32.positive_definite);
        REQUIRE(std::abs(r.r32f.rms_position - r.r64.rms_position) < 1e-4);
        REQUIRE(std::abs(r.rp32.rms_position - r.r64.rms_position) < 1e-4);
        REQUIRE(std::abs(r.rp32.rms_velocity - r.r64.rms_velocity) < 1e-4);
    }
    SECTION("posit<16,1> stays stable and still beats the raw measurements") {
        REQUIRE(r.rp16.positive_definite);
        REQUIRE(r.rp16.rms_position < 0.5 * r.raw);
        // Observed ~1e-3 m from the double reference; 1e-2 leaves margin for
        // libm differences across platforms.
        REQUIRE(std::abs(r.rp16.rms_position - r.r64.rms_position) < 1e-2);
    }
}
