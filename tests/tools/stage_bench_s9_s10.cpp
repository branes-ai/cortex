// S9 window policies and the S10_online_calibration stage bench (issue #458).
//
// Locks what the benches establish: the S9 policy variants remove the clone
// their policy names and keep the principal-submatrix contract; S10 adds its
// prior block and, with the extrinsic estimated, stays consistent with the
// true calibration; and the S10 finding — over one clone window the
// projected update barely sees a constant extrinsic rotation error, so NIS
// stays near zero and the estimate moves only part of the way.

#include <branes/tools/bench/s10_online_calibration_bench.hpp>
#include <branes/tools/bench/s9_marginalization_bench.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
using S9 = bn::S9MarginalizationBench;
using S10 = bn::S10OnlineCalibrationBench;

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

TEST_CASE("S9 policies remove the clone they name and keep the contract", "[tools][bench][s9]") {
    const auto in = S9::decode_input<double>(S9::captured().input);
    const std::size_t n = in.state.clones.size();
    REQUIRE(n >= 3);
    const auto oldest = S9::run<double>(in, "policy_oldest");
    REQUIRE(oldest.removed_index == 0);
    REQUIRE(oldest.diag.removed_time == in.state.clones.front().timestamp);
    const auto second = S9::run<double>(in, "policy_second_newest");
    REQUIRE(second.removed_index == n - 2);
    REQUIRE(second.state.clones.back().timestamp == in.state.clones.back().timestamp);
    for (const char* v : {"policy_oldest", "policy_second_newest"})
        for (const auto& nf : S9::builtin_fixtures())
            require_pass<S9>(nf, v, {"double", "float"});
}

TEST_CASE("a fixture's expected output binds the variants it lists", "[tools][bench]") {
    REQUIRE(bn::binds_variant("", "anything"));
    REQUIRE(bn::binds_variant("shipped", "shipped"));
    REQUIRE(bn::binds_variant("shipped,direct_gather", "direct_gather"));
    REQUIRE_FALSE(bn::binds_variant("shipped,direct_gather", "policy_oldest"));
    REQUIRE_FALSE(bn::binds_variant("shipped", "ship"));
}

TEST_CASE("S10 bench: every fixture passes for every variant in the IEEE types", "[tools][bench][s10]") {
    for (const auto& v : S10::variants())
        for (const auto& nf : S10::builtin_fixtures())
            require_pass<S10>(nf, v.name, {"double", "float", "long_double"});
    // Posits are software arithmetic: slow at -O0, so the known answer only.
    require_pass<S10>({"ka", S10::known_answer()}, bn::kShipped, {"posit32"});
}

TEST_CASE("S10: estimating the extrinsic moves it toward the truth, consistently", "[tools][bench][s10]") {
    const bn::NamedFixture gt{"gt", S10::ground_truth()};
    const auto est = require_pass<S10>(gt, bn::kShipped, {"double"});
    const auto fixed = require_pass<S10>(gt, "fixed", {"double"});
    REQUIRE(value_of(est, "truth.rotation_error_after") < value_of(est, "truth.rotation_error_before"));
    REQUIRE(value_of(fixed, "truth.rotation_error_after") == value_of(fixed, "truth.rotation_error_before"));
    REQUIRE(value_of(est, "calibration.nees_chi2_999") < 22.458);
}

TEST_CASE("S10 finding: one window barely sees a constant extrinsic rotation error", "[tools][bench][s10]") {
    // Columns: 0 remaining error (deg), 1-3 NIS/dof estimated, fixed, folded.
    const auto p = S10::sweep_point<double>({{"rot_error_deg", 5.0}, {"rot_prior_deg", 5.0}});
    REQUIRE(p[2] < 0.01);       // a 5 deg wrong extrinsic, trusted: NIS/dof stays near 0
    REQUIRE(p[0] > 0.7 * 5.0);  // estimated, one window removes under 30% of it
    REQUIRE(p[0] < 5.0);        // but it does move toward the truth
    REQUIRE(p[3] < p[2]);       // folding the prior into R only lowers NIS further
}

TEST_CASE("S10 bench codec rejects malformed inputs", "[tools][bench][s10]") {
    auto j = S10::known_answer().input;
    j.at("state").at("calib") = bn::json::array();
    REQUIRE_THROWS_AS(S10::decode_input<double>(j), std::invalid_argument);
    auto k = S10::known_answer().input;
    k.at("rot_sigma") = bn::pack(0.0);
    REQUIRE_THROWS_AS(S10::decode_input<double>(k), std::invalid_argument);
}

TEST_CASE("S9 and S10 committed fixture files load and pass", "[tools][bench][s9][s10]") {
    const std::filesystem::path dir(BRANES_BENCH_FIXTURE_DIR);
    for (const char* name :
         {"known_answer_fresh_state", "ground_truth_perturbed_extrinsic", "captured_estimated_poses"}) {
        INFO(name);
        require_pass<S10>(
            {name, bn::load(dir / "s10_online_calibration" / (std::string(name) + ".json"))}, bn::kShipped, {"double"});
    }
    for (const char* name : {"known_answer_three_clones", "ground_truth_synthetic_world", "captured_stage_sequence"})
        for (const auto& v : S9::variants()) {
            INFO(name << " / " << v.name);
            require_pass<S9>(
                {name, bn::load(dir / "s9_marginalization" / (std::string(name) + ".json"))}, v.name, {"double"});
        }
}
