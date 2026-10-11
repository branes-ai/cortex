// S0_sensor_model and S1_initialization stage benches (issue #454).
//
// Locks what the benches establish: every fixture kind passes the stage's
// invariants in the default arithmetic types for the shipped variant; the S0
// variants agree; the S1 dynamic alignment resolves in double and long double;
// the sweeps emit their rows; and the fixture variant tag binds an expected
// output to the implementation that produced it.
//
// The S1 SfM front end (#472): an exact window is recovered to rounding; on
// noisy and real (EuRoC) windows the bench pins what it found — #499, #500,
// #501 — so a fix shows up as a test to update, not a silent change.

#include <branes/tools/bench/s0_sensor_model_bench.hpp>
#include <branes/tools/bench/s1_initialization_bench.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <stdexcept>
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

/// The named result of a single-type run (fails the test if absent).
const branes::sdk::eval::inv::InvariantResult& result(const bn::FixtureRun& run, std::string_view name) {
    for (const auto& r : run.types.front().report.results())
        if (r.name == name)
            return r;
    FAIL("no result named " << name);
    return run.types.front().report.results().front();
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
            // Held to the bench's own threshold (tolerance_vs_double), not a fixed number.
            REQUIRE(r.pass);
            REQUIRE(r.value <= r.threshold);
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

TEST_CASE("S0 bench rejects a ragged output fixture", "[tools][bench][s0]") {
    auto f = S0::known_answer();
    f.expected.at("valid").push_back(1);  // one more flag than points
    REQUIRE_THROWS_AS(S0::decode_output<double>(f.expected), std::invalid_argument);
}

TEST_CASE("S1 SfM bench: an exact window is recovered to rounding in every native type", "[tools][bench][s1][sfm]") {
    require_pass<S1>({"ka", S1::known_answer()}, "sfm_two_view_pnp", {"double", "float", "long_double"});
}

TEST_CASE("S1 SfM bench: the 0.1 px window builds and matches the truth, but its scale is off (#501)",
          "[tools][bench][s1][sfm]") {
    const auto run = bn::run_fixture<S1>({"gt", S1::ground_truth()}, "sfm_two_view_pnp", bn::AllTypes{}, {"double"});
    REQUIRE(result(run, "sfm.window_built").pass);
    REQUIRE(result(run, "sfm.pnp_reprojection_rms").pass);
    REQUIRE(result(run, "init.resolved").pass);
    REQUIRE(result(run, "truth.sfm_rotation_error").pass);
    REQUIRE(result(run, "truth.sfm_position_error").pass);
    // The alignment accepts a ~21 % scale error from a ~1 % track error.
    const auto& scale = result(run, "truth.sfm_scale_error");
    REQUIRE_FALSE(scale.pass);
    REQUIRE(scale.value > 10.0);
}

TEST_CASE("S1 SfM bench: a 1 px window fails the PnP inlier gate (#500)", "[tools][bench][s1][sfm]") {
    const auto f = S1::captured();
    const auto out = S1::run<double>(S1::decode_input<double>(f.input), "sfm_two_view_pnp");
    REQUIRE_FALSE(out.window.success);
    REQUIRE(out.window.status == branes::sdk::sfm::InitWindowStatus::PnpFailed);
}

TEST_CASE("S1 SfM bench: the alternatives on the same windows", "[tools][bench][s1][sfm]") {
    // The pre-#247 bootstrap on frame 1 loses the 0.1 px window the shipped one builds.
    const auto gt = S1::ground_truth();
    const auto in = S1::decode_input<double>(gt.input);
    REQUIRE(S1::run<double>(in, "sfm_two_view_pnp").window.success);
    const auto adj = S1::run<double>(in, "sfm_adjacent_bootstrap");
    REQUIRE_FALSE(adj.window.success);
    REQUIRE(adj.window.bootstrap_frame == 1);
    // Keyframes >= 0.25 s apart: the composed preintegration still aligns an
    // exact window, from fewer keyframes.
    const auto ka = S1::known_answer();
    require_pass<S1>({"ka", ka}, "sfm_keyframes_4hz", {"double"});
    const auto k4 = S1::run<double>(S1::decode_input<double>(ka.input), "sfm_keyframes_4hz");
    REQUIRE(k4.window.keyframes.size() < S1::decode_input<double>(ka.input).frames.size());
    REQUIRE(k4.window.keyframe_frames.front() == 0);
}

TEST_CASE("S1 SfM bench: the captured EuRoC windows replay and stop where they did", "[tools][bench][s1][sfm]") {
    const std::filesystem::path dir = std::filesystem::path(BRANES_BENCH_FIXTURE_DIR) / "s1_initialization";
    using St = branes::sdk::sfm::InitWindowStatus;
    const std::pair<const char*, St> cases[] = {
        {"captured_euroc_mh05_at_rest", St::PnpFailed},                     // ~0.05 px of disparity
        {"captured_euroc_mh05_moving_two_view_failed", St::TwoViewFailed},  // > 100 px of disparity (#500)
        {"captured_euroc_mh05_aligned", St::Ok},
    };
    for (const auto& [name, status] : cases) {
        INFO(name);
        const bn::NamedFixture nf{name, bn::load(dir / (std::string(name) + ".json"))};
        REQUIRE(nf.fixture.variant == "sfm_two_view_pnp");
        const auto run = bn::run_fixture<S1>(nf, nf.fixture.variant, bn::AllTypes{}, {"double"});
        // Bit-identical on the platform it was captured on (Linux); elsewhere the
        // math library differs, and the replay must keep its shape and finiteness.
        if (nf.fixture.platform == bn::current_platform())
            REQUIRE(result(run, "replay.bit_identical").value == 0.0);
        else
            REQUIRE(result(run, "replay.same_shape_and_finiteness").pass);
        const auto out = S1::run<double>(S1::decode_input<double>(nf.fixture.input), "sfm_two_view_pnp");
        REQUIRE(out.window.status == status);
        REQUIRE((out.success == 1) == (status == St::Ok));
    }
}

TEST_CASE("S1 SfM sweep: an exact window builds and aligns; 2 px of noise never builds (#500)",
          "[tools][bench][s1][sfm]") {
    const bn::Point base{{"window", 50.0}, {"imu_noise", 0.01}, {"excitation", 1.5}, {"frames", 10.0}, {"speed", 0.5}};
    auto exact = base;
    exact["obs_noise_px"] = 0.0;
    const auto m = S1::sweep_point<double>(exact);
    REQUIRE(m.size() == S1::sweep_columns().size());
    REQUIRE(m[4] == 1.0);  // sfm_window_built
    REQUIRE(m[7] == 1.0);  // sfm_resolved
    REQUIRE(m[8] < 1e-6);  // rotation error, deg
    auto noisy = base;
    noisy["obs_noise_px"] = 2.0;
    const auto n = S1::sweep_point<double>(noisy);
    REQUIRE(n[4] == 0.0);
    REQUIRE(n[5] == static_cast<double>(branes::sdk::sfm::InitWindowStatus::TwoViewFailed));
    // Emulated types skip the SfM columns (its RANSAC is far too costly there).
    const auto p = S1::sweep_point<bn::Posit32>(exact);
    REQUIRE(std::isnan(p[4]));
    REQUIRE_FALSE(S1::runs_by_default("sfm_two_view_pnp", "posit32"));
    REQUIRE(S1::runs_by_default("shipped", "posit32"));
}
