// S0_sensor_model and S1_initialization stage benches (issue #454).
//
// Locks what the benches establish: every fixture kind passes the stage's
// invariants in the default arithmetic types for the shipped variant; the S0
// variants agree; the S1 dynamic alignment resolves in double and long double;
// the sweeps emit their rows; and the fixture variant tag binds an expected
// output to the implementation that produced it.

#include <branes/tools/bench/s0_sensor_model_bench.hpp>
#include <branes/tools/bench/s1_initialization_bench.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
using S0 = bn::S0SensorModelBench;
using S1 = bn::S1InitializationBench;

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

}  // namespace

TEST_CASE("S0 bench: every fixture passes in every type, both variants", "[tools][bench][s0]") {
    const auto fixtures = S0::builtin_fixtures();
    REQUIRE(fixtures.size() == 3);
    for (const auto& v : S0::variants())
        for (const auto& nf : fixtures)
            require_pass<S0>(nf, v.name, {"double", "float", "posit32", "posit16", "long_double"});
}

TEST_CASE("S0 bench: ground truth is the camera-frame normalized landmark", "[tools][bench][s0]") {
    const auto gt = S0::ground_truth();
    const auto run = bn::run_fixture<S0>({"gt", gt}, bn::kShipped, bn::DefaultTypes{}, {"double"});
    bool checked = false;
    for (const auto& r : run.types[0].report.results())
        if (r.name == "truth.normalized_point") {
            checked = true;
            REQUIRE(r.value < 1e-12);
        }
    REQUIRE(checked);
}

TEST_CASE("S0 bench sweep: round trip and Jacobian stay tight across distortion and radius", "[tools][bench][s0]") {
    std::size_t rows = 0;
    S0::default_sweep().run([&](const bn::Point& p) {
        const auto m = S0::sweep_point<double>(p);
        REQUIRE(m.size() == S0::sweep_columns().size());
        REQUIRE(m[0] < 1e-9);  // round-trip residual, px
        REQUIRE(m[2] > 1.0);   // 1 deg of extrinsic rotation moves a pixel by several px
        REQUIRE(m[3] < m[2]);  // 1 ms at 1 rad/s is a smaller shift than 1 deg
        ++rows;
    });
    REQUIRE(rows == 16);
}

TEST_CASE("S1 bench: static paths pass every fixture in every default type", "[tools][bench][s1]") {
    const auto fixtures = S1::builtin_fixtures();
    REQUIRE(fixtures.size() == 3);
    for (const char* v : {"shipped", "gravity_align"})
        for (const auto& nf : fixtures)
            require_pass<S1>(nf, v, {"double", "float", "posit32", "long_double"});
}

TEST_CASE("S1 bench: the dynamic alignment resolves and recovers scale in double and long double",
          "[tools][bench][s1]") {
    for (const auto& nf : S1::builtin_fixtures())
        require_pass<S1>(nf, "dynamic", {"double", "long_double"});
}

TEST_CASE("S1 bench: a known answer binds only the variant it belongs to", "[tools][bench][s1]") {
    const auto ka = S1::known_answer();
    REQUIRE(ka.variant == "shipped");
    // The dynamic variant estimates something else from the same input; the
    // static closed-form answer is not held against it.
    const auto run = bn::run_fixture<S1>({"ka", ka}, "dynamic", bn::DefaultTypes{}, {"double"});
    for (const auto& r : run.types[0].report.results())
        REQUIRE(r.name != "known_answer.residual");
    REQUIRE(run.pass());
}

TEST_CASE("S1 bench sweep: static init resolves and levels at low noise", "[tools][bench][s1]") {
    const auto m = S1::sweep_point<double>({{"window", 50.0}, {"imu_noise", 0.01}, {"excitation", 1.5}});
    REQUIRE(m.size() == S1::sweep_columns().size());
    REQUIRE(m[0] == 1.0);  // static resolved
    REQUIRE(m[1] < 0.1);   // roll/pitch error, deg
    REQUIRE(m[2] == 1.0);  // dynamic resolved
    REQUIRE(m[3] < 5.0);   // scale error, %
    REQUIRE_THROWS_AS(S1::sweep_point<double>({{"window", 1.5}, {"imu_noise", 0.0}, {"excitation", 1.0}}),
                      std::invalid_argument);
}

TEST_CASE("S0 and S1 committed fixture files load and pass", "[tools][bench][s0][s1]") {
    const std::filesystem::path dir(BRANES_BENCH_FIXTURE_DIR);
    for (const char* name : {"known_answer_pinhole_grid", "ground_truth_synthetic_frame", "captured_synthetic_frame"}) {
        INFO(name);
        require_pass<S0>(
            {name, bn::load(dir / "s0_sensor_model" / (std::string(name) + ".json"))}, bn::kShipped, {"double"});
    }
    for (const char* name : {"known_answer_tilted_static", "ground_truth_synthetic_warmup", "captured_noisy_window"}) {
        INFO(name);
        require_pass<S1>(
            {name, bn::load(dir / "s1_initialization" / (std::string(name) + ".json"))}, bn::kShipped, {"double"});
    }
}
