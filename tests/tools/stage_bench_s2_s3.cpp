// S2_propagation and S3_augmentation stage benches (issue #455).
//
// Locks what the benches establish: every fixture kind passes the stages'
// invariants in the default arithmetic types for every variant; propagation
// reproduces an analytic trajectory and preserves the translation directions of
// the unobservable subspace exactly while the yaw direction leaks (the
// body-frame filter's first-order leak, reported); the canonical Q_d grows the
// position block the diagonal Q_d leaves out; square-root variants start from
// a singular (cloned) covariance; and the committed fixture files replay.

#include <branes/tools/bench/s2_propagation_bench.hpp>
#include <branes/tools/bench/s3_augmentation_bench.hpp>

#include <catch2/catch_test_macros.hpp>

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

TEST_CASE("S2 bench: analytic truth, exact translation subspace, a reported yaw leak", "[tools][bench][s2]") {
    const auto run = bn::run_fixture<S2>({"gt", S2::ground_truth()}, bn::kShipped, bn::DefaultTypes{}, {"double"});
    REQUIRE(run.pass());
    REQUIRE(value_of(run, "truth.position_error") < 1e-12);  // constant ω, a_w: ZOH integration is exact here
    REQUIRE(value_of(run, "observability.translation_preserved") == 0.0);
    REQUIRE(value_of(run, "observability.yaw_leak") > 0.0);  // first-order Φ at the current estimate
    REQUIRE(value_of(run, "observability.yaw_leak") < 1e-4);
}

TEST_CASE("S2 bench sweep: the canonical Q_d grows the position sigma the diagonal Q_d drops", "[tools][bench][s2]") {
    const auto m = S2::sweep_point<double>({{"dt", 0.005}, {"dynamics", 1.0}, {"q_scale", 1.0}});
    REQUIRE(m.size() == S2::sweep_columns().size());
    REQUIRE(m[2] > 0.0);  // pos_sigma_gap_pct
    REQUIRE(m[3] > 0.0);  // λ_min/‖P‖ stays positive
    REQUIRE_THROWS_AS(S2::sweep_point<double>({{"dt", 0.005}, {"dynamics", 3.0}, {"q_scale", 1.0}}),
                      std::invalid_argument);
}

TEST_CASE("S3 bench: every fixture passes for every variant (double, float, long double)", "[tools][bench][s3]") {
    for (const auto& v : S3::variants())
        for (const auto& nf : S3::builtin_fixtures())
            require_pass<S3>(nf, v.name, {"double", "float", "long_double"});
}

TEST_CASE("S3 bench: the shipped augmentation passes in posit<32,2>", "[tools][bench][s3]") {
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
