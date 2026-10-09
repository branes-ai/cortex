// S4_frontend and S5_triangulation stage benches (issue #456).
//
// Locks what the benches establish: every fixture kind passes the stages'
// invariants for every variant (S4 in double, the type its KLT runs in; S5 in
// the default arithmetic types); tracking an unchanged image is an exact fixed
// point; a clean integer translation tracks to a small fraction of a pixel; the
// image codec is lossless; S5 recovers a landmark from exact observations within
// its conditioning bound; and the committed fixture files replay.

#include <branes/tools/bench/s4_frontend_bench.hpp>
#include <branes/tools/bench/s5_triangulation_bench.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
using S4 = bn::S4FrontendBench;
using S5 = bn::S5TriangulationBench;

template <class B>
bn::FixtureRun
require_pass(const bn::NamedFixture& nf, std::string_view variant, const std::vector<std::string>& types) {
    const auto run = bn::run_fixture<B>(nf, variant, bn::bench_types<B>(), types);
    REQUIRE(run.types.size() == types.size());
    for (const auto& t : run.types) {
        const auto bad = t.report.first_failure();
        INFO(nf.name << " / " << variant << " / " << t.type << " / "
                     << (bad ? std::string(bad->name) + " = " + std::to_string(bad->value) : std::string("-")));
        REQUIRE(t.report.all_pass());
    }
    return run;
}

double value_of(const bn::FixtureRun& run, std::string_view name) {
    for (const auto& r : run.types.front().report.results())
        if (r.name == name)
            return r.value;
    FAIL("no result named " << name);
    return 0.0;
}

}  // namespace

TEST_CASE("S4 bench: every fixture passes for every variant", "[tools][bench][s4]") {
    for (const auto& v : S4::variants())
        for (const auto& nf : S4::builtin_fixtures())
            require_pass<S4>(nf, v.name, {"double"});
}

TEST_CASE("S4 bench: an unchanged image is a fixed point; a clean translation tracks sub-pixel", "[tools][bench][s4]") {
    const auto ka = require_pass<S4>({"ka", S4::known_answer()}, bn::kShipped, {"double"});
    REQUIRE(value_of(ka, "known_answer.residual") == 0.0);
    const auto gt = require_pass<S4>({"gt", S4::ground_truth()}, bn::kShipped, {"double"});
    REQUIRE(value_of(gt, "truth.endpoint_error") < 0.05);
    // Border churn is real and reported: most new detections in a 160x120 frame
    // sit where the coarsest KLT level cannot track them.
    REQUIRE(value_of(gt, "tracks.lost_on_first_frame") > 50.0);
}

TEST_CASE("S4 bench: the image codec is lossless", "[tools][bench][s4]") {
    const auto img = S4::render(37, 23, 1.0, 2.0, 9.0, 7u);
    const auto back = bn::unpack_image(bn::pack(img.view()));
    REQUIRE(back.width() == img.width());
    REQUIRE(back.height() == img.height());
    for (std::size_t y = 0; y < img.height(); ++y)
        for (std::size_t x = 0; x < img.width(); ++x)
            REQUIRE(back(y, x) == img(y, x));
    REQUIRE_THROWS_AS(bn::unpack_image(bn::json{{"width", 4}, {"height", 4}, {"data", "AAAA"}}), std::invalid_argument);
}

TEST_CASE("S4 bench sweep: survival and accuracy degrade with noise", "[tools][bench][s4]") {
    const auto clean = S4::sweep_point<double>({{"noise", 0.0}, {"shift_px", 2.0}});
    const auto noisy = S4::sweep_point<double>({{"noise", 25.0}, {"shift_px", 2.0}});
    REQUIRE(clean.size() == S4::sweep_columns().size());
    REQUIRE(clean[1] < noisy[1]);  // end-point RMS grows with noise
    REQUIRE_THROWS_AS(S4::sweep_point<double>({{"noise", -1.0}, {"shift_px", 2.0}}), std::invalid_argument);
}

TEST_CASE("S5 bench: every fixture passes for every variant and default type", "[tools][bench][s5]") {
    for (const auto& v : S5::variants())
        for (const auto& nf : S5::builtin_fixtures())
            require_pass<S5>(nf, v.name, {"double", "float", "posit32", "posit16", "long_double"});
}

TEST_CASE("S5 bench: exact observations recover the landmark; parallax and kappa reported", "[tools][bench][s5]") {
    const auto gt = require_pass<S5>({"gt", S5::ground_truth()}, bn::kShipped, {"double"});
    REQUIRE(value_of(gt, "truth.feature_position") < 1e-12);
    REQUIRE(value_of(gt, "parallax.max_inter_view") > 0.0);
    REQUIRE(value_of(gt, "normal_matrix.condition_number") > 1.0);
}

TEST_CASE("S5 bench sweep: low parallax is ill-conditioned and gated", "[tools][bench][s5]") {
    const auto low = S5::sweep_point<double>({{"parallax_deg", 0.25}, {"px_noise", 1.0}});
    const auto high = S5::sweep_point<double>({{"parallax_deg", 15.0}, {"px_noise", 1.0}});
    REQUIRE(low.size() == S5::sweep_columns().size());
    REQUIRE(low[1] > high[1]);  // κ grows as parallax shrinks
    REQUIRE(low[0] > high[0]);  // so does the depth error
    REQUIRE(low[2] == 1.0);     // the 2 deg gate rejects every 0.25 deg draw
    REQUIRE(high[2] == 0.0);    // and none at 15 deg
}

TEST_CASE("S4 and S5 committed fixture files load and pass", "[tools][bench][s4][s5]") {
    const std::filesystem::path dir(BRANES_BENCH_FIXTURE_DIR);
    for (const char* name : {"known_answer_static_scene", "ground_truth_known_shifts", "captured_noisy_translation"}) {
        INFO(name);
        require_pass<S4>(
            {name, bn::load(dir / "s4_frontend" / (std::string(name) + ".json"))}, bn::kShipped, {"double"});
    }
    for (const char* name :
         {"known_answer_wide_baseline", "ground_truth_synthetic_landmark", "captured_estimated_poses"}) {
        INFO(name);
        require_pass<S5>(
            {name, bn::load(dir / "s5_triangulation" / (std::string(name) + ".json"))}, bn::kShipped, {"double"});
    }
}

TEST_CASE("bench review hardening: base64 padding, vacuous truth, gate rejections", "[tools][bench][s4][s5]") {
    // Padding only at the end of the final group.
    REQUIRE(bn::detail::base64_decode("AAA=").size() == 2);
    REQUIRE(bn::detail::base64_decode("AA==").size() == 1);
    REQUIRE_THROWS_AS(bn::detail::base64_decode("AA=A"), std::invalid_argument);
    REQUIRE_THROWS_AS(bn::detail::base64_decode("=AAA"), std::invalid_argument);
    REQUIRE_THROWS_AS(bn::detail::base64_decode("AA==AAAA"), std::invalid_argument);

    // S4: a ground truth with no continuing track checks nothing, so it fails.
    auto gt = S4::ground_truth();
    auto in = S4::decode_input<double>(gt.input);
    S4::Output<double> none;
    none.frames.assign(in.frames.size(), {});
    none.diag.assign(in.frames.size(), {});
    const auto r = S4::invariants<double>(in, none, gt);
    bool saw = false;
    for (const auto& x : r)
        if (x.name == "truth.endpoint_error") {
            saw = true;
            REQUIRE_FALSE(x.pass);
        }
    REQUIRE(saw);

    // S5: a gate variant that rejects a wide-parallax track has failed, not gated.
    auto ka = S5::decode_input<double>(S5::known_answer().input);
    S5::Output<double> rejected{0, {}, 2.0};  // "rejected" at 2 deg on a ~53 deg track
    bool resolved_failed = false;
    for (const auto& x : S5::invariants<double>(ka, rejected, S5::known_answer()))
        if (x.name == "triangulation.resolved")
            resolved_failed = !x.pass;
    REQUIRE(resolved_failed);

    // S5: a camera the bench does not supply is an input error, not a rejection.
    auto bad = S5::known_answer().input;
    bad.at("observations").at(0).at(1) = 1;
    REQUIRE_THROWS_AS(S5::decode_input<double>(bad), std::invalid_argument);
}

TEST_CASE("S5 bench: the alternative triangulation methods, measured", "[tools][bench][s5]") {
    // Columns: 0 shipped, 3 midpoint, 4 DLT, 5 inverse depth (depth error, m).
    const auto low = S5::sweep_point<double>({{"parallax_deg", 0.25}, {"px_noise", 0.5}});
    const auto mid = S5::sweep_point<double>({{"parallax_deg", 5.0}, {"px_noise", 1.0}});
    // Inverse depth minimizes the same reprojection error as the shipped
    // Gauss-Newton, from a different seed and parameterization: the same answer.
    REQUIRE(std::abs(mid[5] - mid[0]) < 1e-9);
    REQUIRE(std::abs(low[5] - low[0]) < 0.05 * low[0]);
    // The algebraic DLT is shrunk toward the cameras: better at grazing
    // parallax, a little worse where depth is observable.
    REQUIRE(low[4] < low[0]);
    REQUIRE(mid[4] > mid[0]);
    // The two-view midpoint discards views: the worst of the four, and it
    // loses the track outright at grazing parallax.
    REQUIRE(mid[3] > mid[4]);
    REQUIRE(std::isinf(low[3]));
}
