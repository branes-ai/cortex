// S2_propagation and S3_augmentation stage benches (issue #455).
//
// Locks what the benches establish: every fixture kind passes the stages'
// invariants in the default arithmetic types for every variant; propagation
// reproduces an analytic trajectory and preserves the translation directions of
// the unobservable subspace exactly while the yaw direction leaks (the
// body-frame filter's first-order leak, reported); the canonical Q_d grows the
// position block the diagonal Q_d leaves out; square-root variants start from
// a singular (cloned) covariance; and the committed fixture files replay. The
// invariant variant (#471) preserves the yaw direction too, and a higher-order
// body-frame Φ leaks more of it, not less.

#include <branes/tools/bench/s2_propagation_bench.hpp>
#include <branes/tools/bench/s3_augmentation_bench.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
using S2 = bn::S2PropagationBench;
using S3 = bn::S3AugmentationBench;

template <class B>
void require_pass(const bn::NamedFixture& nf, std::string_view variant, const std::vector<std::string>& types) {
    const auto run = bn::run_fixture<B>(nf, variant, bn::AllTypes{}, types);
    REQUIRE(run.types.size() == types.size());
    for (const auto& t : run.types) {
        const auto bad = t.report.first_failure();
        INFO(nf.name << " / " << variant << " / " << t.type << " / "
                     << (bad ? std::string(bad->name) + " = " + std::to_string(bad->value) : std::string("-")));
        REQUIRE(t.report.all_pass());
    }
}

double value_of(const bn::FixtureRun& run, std::string_view name) {
    for (const auto& r : run.types.front().report.results())
        if (r.name == name)
            return r.value;
    FAIL("no result named " << name);
    return 0.0;
}

}  // namespace

TEST_CASE("S2 bench: every fixture passes for every variant (double, float)", "[tools][bench][s2]") {
    for (const auto& v : S2::variants())
        for (const auto& nf : S2::builtin_fixtures())
            require_pass<S2>(nf, v.name, {"double", "float", "long_double"});
}

TEST_CASE("S2 bench: the shipped propagation passes in posit<32,2>", "[tools][bench][s2]") {
    for (const auto& nf : S2::builtin_fixtures())
        require_pass<S2>(nf, bn::kShipped, {"posit32"});
}

TEST_CASE("S2 bench: analytic truth within what the camera resolves, the gauge preserved", "[tools][bench][s2]") {
    const auto run = bn::run_fixture<S2>({"gt", S2::ground_truth()}, bn::kShipped, bn::DefaultTypes{}, {"double"});
    REQUIRE(run.pass());
    // The body-frame specific force is not linear between samples, so the
    // interpolated input leaves an O(dt^2) error, far inside the bound (#616).
    REQUIRE(value_of(run, "truth.position_error") > 0.0);
    REQUIRE(value_of(run, "truth.position_error") < 0.01 * S2::kResolvedPosition);
    REQUIRE(value_of(run, "truth.attitude_error") < 1e-12);  // constant rate: no coning, exact
    // The shipped Phi is the RK4 mean's exact Jacobian: no leak along the gauge.
    REQUIRE(value_of(run, "observability.translation_preserved") < 1e-12);
    REQUIRE(value_of(run, "observability.yaw_leak") < 1e-12);
}

TEST_CASE("S2 bench sweep: the canonical Q_d grows the position sigma the diagonal Q_d drops", "[tools][bench][s2]") {
    const auto m = S2::sweep_point<double>({{"dt", 0.005}, {"dynamics", 1.0}, {"q_scale", 1.0}});
    REQUIRE(m.size() == S2::sweep_columns().size());
    const auto cols = S2::sweep_columns();
    const auto col = [&](std::string_view name) {
        const auto it = std::find(cols.begin(), cols.end(), name);
        REQUIRE(it != cols.end());
        return m[static_cast<std::size_t>(it - cols.begin())];
    };
    REQUIRE(col("pos_sigma_gap_pct") > 0.0);
    REQUIRE(col("lambda_min_rel") > 0.0);  // λ_min/‖P‖ stays positive
    REQUIRE_THROWS_AS(S2::sweep_point<double>({{"dt", 0.005}, {"dynamics", 3.0}, {"q_scale", 1.0}}),
                      std::invalid_argument);
}

TEST_CASE("S2 bench: the invariant error maps the body-frame gauge onto a state-independent one",
          "[tools][bench][s2]") {
    // The captured fixture's state: tumbling, moving, three clones.
    const auto in = S2::decode_input<double>(S2::captured().input);
    const auto& s = in.state;
    REQUIRE(s.clones.size() == 3);
    const auto J = S2::to_invariant(s);
    const auto Ji = S2::to_invariant(s, /*inverse=*/true);
    const auto JN = branes::sdk::msckf::mul(J, S2::unobservable_basis(s));
    const auto Ninv = S2::invariant_basis(s);
    const auto I = branes::sdk::msckf::mul(Ji, J);
    double basis_err = 0.0, inverse_err = 0.0;
    for (std::size_t i = 0; i < JN.d.size(); ++i)
        basis_err = std::max(basis_err, std::abs(JN.d[i] - Ninv.d[i]));
    for (std::size_t i = 0; i < I.rows; ++i)
        for (std::size_t j = 0; j < I.cols; ++j)
            inverse_err = std::max(inverse_err, std::abs(I(i, j) - (i == j ? 1.0 : 0.0)));
    CAPTURE(basis_err, inverse_err);
    REQUIRE(basis_err < 1e-12);
    REQUIRE(inverse_err < 1e-12);

    // On the tumbling trajectory both Phis preserve yaw to roundoff: the
    // invariant one by construction, the body-frame one since #616 because it
    // is the exact Jacobian of the RK4 mean. The pre-#616 first-order Phi leaked
    // it at O(dt^2) (the higher_order_phi comparison still does).
    const auto gt = S2::ground_truth();
    const auto inv_run = bn::run_fixture<S2>({"gt", gt}, "invariant", bn::AllTypes{}, {"double"});
    const auto body_run = bn::run_fixture<S2>({"gt", gt}, bn::kShipped, bn::AllTypes{}, {"double"});
    const auto first_order = bn::run_fixture<S2>({"gt", gt}, "higher_order_phi", bn::AllTypes{}, {"double"});
    REQUIRE(value_of(inv_run, "observability.yaw_leak") < 1e-15);
    REQUIRE(value_of(body_run, "observability.yaw_leak") < 1e-12);
    REQUIRE(value_of(first_order, "observability.yaw_leak") > 1e-7);
}

TEST_CASE("S2 bench sweep: the mean's own Jacobian leaks no yaw, a truncated Phi does", "[tools][bench][s2]") {
    const auto cols = S2::sweep_columns();
    auto at = [&](const std::vector<double>& m, std::string_view name) {
        const auto it = std::find(cols.begin(), cols.end(), name);
        REQUIRE(it != cols.end());
        return m[static_cast<std::size_t>(it - cols.begin())];
    };
    for (const double dt : {0.0025, 0.01}) {
        const auto m = S2::sweep_point<double>({{"dt", dt}, {"dynamics", 1.0}, {"q_scale", 1.0}});
        CAPTURE(dt, at(m, "yaw_leak"), at(m, "yaw_leak_1st_order"), at(m, "yaw_leak_2nd_order"), at(m, "yaw_leak_exp"));
        // A leak is Phi disagreeing with the mean it propagates, not Phi's
        // truncation (#471, #616). The shipped Phi is the RK4 mean's exact
        // Jacobian and leaks nothing. The pre-#616 first-order Phi does: it was
        // linearized against the one-sample Euler step, and expanding it toward
        // exp(A dt) only moves it to a third mean, the continuous one.
        REQUIRE(at(m, "yaw_leak") < 1e-12);
        REQUIRE(at(m, "yaw_leak_1st_order") > 1e-7);
        REQUIRE(at(m, "yaw_leak_2nd_order") > 1e-7);
        REQUIRE(std::abs(at(m, "yaw_leak_exp") - at(m, "yaw_leak_2nd_order")) < 0.01 * at(m, "yaw_leak_2nd_order"));
        REQUIRE(at(m, "yaw_leak_invariant") < 1e-15);
        // Same strapdown physics: the invariant mean lands where the body-frame one does.
        REQUIRE(std::abs(at(m, "pos_err_invariant_m") - at(m, "pos_err_m")) < 1e-12);
        REQUIRE(at(m, "pos_err_m") < S2::kResolvedPosition);
    }
}

TEST_CASE("S3 bench: every fixture passes for every variant (double, float, long double)", "[tools][bench][s3]") {
    for (const auto& v : S3::variants())
        for (const auto& nf : S3::builtin_fixtures())
            require_pass<S3>(nf, v.name, {"double", "float", "long_double"});
}

TEST_CASE("S3 bench: the shipped augmentation passes in posit<32,2>", "[tools][bench][s3][slow]") {
    for (const auto& nf : S3::builtin_fixtures())
        require_pass<S3>(nf, bn::kShipped, {"posit32"});
}

TEST_CASE("S3 bench sweep: augmentation is an exact block copy at every conditioning", "[tools][bench][s3]") {
    std::size_t rows = 0;
    S3::default_sweep().run([&](const bn::Point& p) {
        const auto m = S3::sweep_point<double>(p);
        REQUIRE(m.size() == S3::sweep_columns().size());
        REQUIRE(m[1] == 0.0);  // block residual
        ++rows;
    });
    REQUIRE(rows == 12);
}

TEST_CASE("psd_factor factors a singular (cloned) covariance", "[tools][bench]") {
    // P with an exact duplicate row/col (a clone): singular, PSD.
    auto P = S2::correlated_spd(6);
    branes::sdk::msckf::DynMat<double> Pd(7, 7);
    for (std::size_t i = 0; i < 7; ++i)
        for (std::size_t j = 0; j < 7; ++j)
            Pd(i, j) = P(i < 6 ? i : 0, j < 6 ? j : 0);
    branes::sdk::msckf::DynMat<double> L;
    // (Plain Cholesky may fail here or squeak through on a roundoff-positive pivot.)
    REQUIRE(bn::psd_factor(Pd, L));
    const auto LLt = branes::sdk::msckf::mul(L, branes::sdk::msckf::transpose(L));
    double worst = 0.0;
    for (std::size_t i = 0; i < Pd.d.size(); ++i)
        worst = std::max(worst, std::abs(LLt.d[i] - Pd.d[i]));
    REQUIRE(worst < 1e-12);
    // A clearly indefinite matrix is refused.
    Pd(3, 3) = -1.0;
    REQUIRE_FALSE(bn::psd_factor(Pd, L));
}

TEST_CASE("S2 and S3 committed fixture files load and pass", "[tools][bench][s2][s3]") {
    const std::filesystem::path dir(BRANES_BENCH_FIXTURE_DIR);
    for (const char* name :
         {"known_answer_stationary_level", "ground_truth_tumbling_accel", "captured_synthetic_segment"}) {
        INFO(name);
        require_pass<S2>(
            {name, bn::load(dir / "s2_propagation" / (std::string(name) + ".json"))}, bn::kShipped, {"double"});
    }
    for (const char* name : {"known_answer_two_clones", "ground_truth_synthetic_pose", "captured_stage_sequence"}) {
        INFO(name);
        require_pass<S3>(
            {name, bn::load(dir / "s3_augmentation" / (std::string(name) + ".json"))}, bn::kShipped, {"double"});
    }
}

TEST_CASE("S2 bench rejects a sweep dt that would exhaust memory, with exit code 2", "[tools][bench][s2]") {
    REQUIRE_THROWS_AS(S2::sweep_point<double>({{"dt", 1e-6}, {"dynamics", 0.0}, {"q_scale", 1.0}}),
                      std::invalid_argument);
    std::vector<std::string> args{"s2_propagation_bench", "--types", "double", "--sweep", "dt=1e-6"};
    std::vector<char*> argv;
    for (auto& a : args)
        argv.push_back(a.data());
    REQUIRE(bn::bench_main<S2>(static_cast<int>(argv.size()), argv.data()) == 2);
}
