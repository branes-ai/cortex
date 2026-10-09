// Stage test-bench framework (issue #453), via its worked example: the
// S9_marginalization bench.
//
// Locks the framework's guarantees: every fixture kind loads (built-in and from
// the committed files), a captured fixture replays bit-identically to the run it
// came from, the fixture codec is exact for edge values, the bench catches a
// corrupted fixture, the variants agree on identical fixtures, the stage runs in
// several arithmetic types against the double reference, and the sweep emits one
// row per point.

#include <branes/tools/bench/s9_marginalization_bench.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
using Bench = bn::S9MarginalizationBench;

const std::vector<std::string> kTypes{"double", "float", "posit32"};

std::uint64_t bits(double x) {
    std::uint64_t b = 0;
    std::memcpy(&b, &x, sizeof b);
    return b;
}

}  // namespace

TEST_CASE("bench fixture codec round-trips edge values bit-exactly", "[tools][bench]") {
    const std::vector<double> edge{0.0,
                                   -0.0,
                                   1.0 / 3.0,
                                   std::numeric_limits<double>::denorm_min(),
                                   std::numeric_limits<double>::max(),
                                   -std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::quiet_NaN()};
    bn::Fixture f;
    f.stage = "S9_marginalization";
    f.kind = bn::FixtureKind::Captured;
    f.input = bn::pack_vec<double>(edge);
    f.expected = f.input;
    const auto back = bn::from_json(bn::json::parse(bn::to_json(f).dump(1)));
    const auto v = bn::unpack_vec<double>(back.input);
    REQUIRE(v.size() == edge.size());
    for (std::size_t i = 0; i < edge.size(); ++i)
        REQUIRE(bits(v[i]) == bits(edge[i]));
    REQUIRE(back.kind == bn::FixtureKind::Captured);

    // A state round-trips exactly, rotations included (no renormalization).
    const auto cap = Bench::captured();
    const auto s = bn::unpack_state<double>(cap.input.at("state"));
    REQUIRE(bn::pack(s) == cap.input.at("state"));
}

TEST_CASE("bench fixture loader rejects malformed fixtures", "[tools][bench]") {
    auto j = bn::to_json(Bench::known_answer());
    j["schema"] = "something/else";
    REQUIRE_THROWS_AS(bn::from_json(j), std::invalid_argument);
    auto k = bn::to_json(Bench::known_answer());
    k.erase("expected");
    REQUIRE_THROWS_AS(bn::from_json(k), std::invalid_argument);
}

TEST_CASE("S9 bench: all three built-in fixture kinds pass in every type and variant", "[tools][bench][s9]") {
    const auto fixtures = Bench::builtin_fixtures();
    REQUIRE(fixtures.size() == 3);
    REQUIRE(fixtures[0].fixture.kind == bn::FixtureKind::KnownAnswer);
    REQUIRE(fixtures[1].fixture.kind == bn::FixtureKind::GroundTruth);
    REQUIRE(fixtures[2].fixture.kind == bn::FixtureKind::Captured);
    for (const auto& v : Bench::variants())
        for (const auto& nf : fixtures) {
            const auto run = bn::run_fixture<Bench>(nf, v.name, bn::AllTypes{}, kTypes);
            REQUIRE(run.types.size() == 3);
            for (const auto& t : run.types) {
                INFO(nf.name << " / " << v.name << " / " << t.type);
                const auto bad = t.report.first_failure();
                INFO((bad ? std::string(bad->name) : std::string("-")));
                REQUIRE(t.report.all_pass());
            }
            // double is the reference; float and posit32 deviate, but only at roundoff.
            REQUIRE(run.types[0].max_diff_vs_double == 0.0);
            REQUIRE(run.types[1].max_diff_vs_double < 1e-5);
            REQUIRE(run.types[2].max_diff_vs_double < 1e-6);
        }
}

TEST_CASE("S9 bench: a captured fixture replays bit-identically, from memory and from file", "[tools][bench][s9]") {
    // Capture → save → load → replay.
    const auto dir = std::filesystem::temp_directory_path() / "branes_bench_test";
    const auto path = dir / "captured.json";
    bn::save(Bench::captured(), path);
    const bn::NamedFixture loaded{"captured", bn::load(path)};
    const auto run = bn::run_fixture<Bench>(loaded, bn::kShipped, bn::DefaultTypes{}, {"double"});
    bool replayed = false;
    for (const auto& r : run.types[0].report.results())
        if (r.name == "replay.bit_identical") {
            replayed = true;
            REQUIRE(r.pass);
            REQUIRE(r.value == 0.0);
        }
    REQUIRE(replayed);
    std::filesystem::remove_all(dir);

    // The committed fixture files load and pass too (all three kinds).
    for (const char* name : {"known_answer_three_clones", "ground_truth_synthetic_world", "captured_stage_sequence"}) {
        const auto file =
            std::filesystem::path(BRANES_BENCH_FIXTURE_DIR) / "s9_marginalization" / (std::string(name) + ".json");
        const bn::NamedFixture nf{name, bn::load(file)};
        const auto r = bn::run_fixture<Bench>(nf, bn::kShipped, bn::DefaultTypes{}, {"double"});
        INFO(name);
        REQUIRE(r.pass());
    }
}

TEST_CASE("S9 bench catches a fixture whose recorded output is wrong", "[tools][bench][s9]") {
    // Captured: one recorded covariance entry nudged by one ulp → replay fails.
    auto cap = Bench::captured();
    auto& d = cap.expected.at("state").at("P").at("data");
    d[5] = std::nextafter(d[5].get<double>(), 1.0);
    const auto run = bn::run_fixture<Bench>({"tampered", cap}, bn::kShipped, bn::DefaultTypes{}, {"double"});
    REQUIRE_FALSE(run.pass());
    REQUIRE(run.types[0].report.first_failure()->name == "replay.bit_identical");

    // Known-answer: a wrong expected submatrix entry → the residual check fails.
    auto ka = Bench::known_answer();
    ka.expected.at("state").at("P").at("data")[0] = 1.0;
    REQUIRE_FALSE(bn::run_fixture<Bench>({"wrong_answer", ka}, bn::kShipped, bn::DefaultTypes{}, {"double"}).pass());
}

TEST_CASE("S9 bench sweep: one row per point, a pure gather at every conditioning", "[tools][bench][s9]") {
    auto sw = Bench::default_sweep();
    sw.axis_spec("clones=2,4");
    REQUIRE(sw.size() == 6);  // 2 clone counts × 3 conditionings
    std::size_t rows = 0;
    sw.run([&](const bn::Point& p) {
        const auto m = Bench::sweep_point<double>(p);
        REQUIRE(m.size() == Bench::sweep_columns().size());
        REQUIRE(m[1] == 0.0);  // principal-submatrix residual
        REQUIRE(m[2] > 0.0);   // λ_min after removal: still positive-definite
        ++rows;
    });
    REQUIRE(rows == 6);

    bn::CsvTable t({"a", "b"});
    t.row({1.0, 0.1});
    std::ostringstream out;
    t.write(out);
    REQUIRE(out.str() == "a,b\n1,0.10000000000000001\n");
}

TEST_CASE("bench variant selection", "[tools][bench]") {
    const auto all = Bench::variants();
    REQUIRE(bn::select_variants(all, "").front().name == "shipped");
    REQUIRE(bn::select_variants(all, "all").size() == all.size());
    REQUIRE(bn::select_variants(all, "direct_gather").front().name == "direct_gather");
    REQUIRE_THROWS_AS(bn::select_variants(all, "nope"), std::invalid_argument);
}

// ── PR #463 review hardening ────────────────────────────────────────────────

TEST_CASE("bench rejects malformed rotations, sweep values and non-finite known answers", "[tools][bench]") {
    // A non-unit or non-canonical quaternion is a fixture error, not an assert.
    REQUIRE_THROWS_AS(bn::unpack_so3<double>(bn::json::array({1.0, 0.5, 0.0, 0.0})), std::invalid_argument);
    REQUIRE_THROWS_AS(bn::unpack_so3<double>(bn::json::array({-1.0, 0.0, 0.0, 0.0})), std::invalid_argument);
    REQUIRE_THROWS_AS(bn::unpack_so3<double>(bn::json::array({"nan", 0.0, 0.0, 0.0})), std::invalid_argument);
    REQUIRE(bn::unpack_so3<double>(bn::json::array({1.0, 0.0, 0.0, 0.0})).quaternion()[0] == 1.0);

    // Sweep values must be finite; the S9 sweep's clone count must be a small integer.
    bn::Sweep sw;
    REQUIRE_THROWS_AS(sw.axis_spec("x=1,nan"), std::invalid_argument);
    REQUIRE_THROWS_AS(sw.axis_spec("x=inf"), std::invalid_argument);
    REQUIRE_THROWS_AS(Bench::sweep_point<double>({{"clones", 2.5}, {"log10_cond", 0.0}}), std::invalid_argument);
    REQUIRE_THROWS_AS(Bench::sweep_point<double>({{"clones", 0.0}, {"log10_cond", 0.0}}), std::invalid_argument);

    // A known answer containing NaN never passes the residual check.
    auto ka = Bench::known_answer();
    ka.expected.at("state").at("P").at("data")[3] = "nan";
    const auto run = bn::run_fixture<Bench>({"nan_answer", ka}, bn::kShipped, bn::DefaultTypes{}, {"double"});
    REQUIRE_FALSE(run.pass());
    REQUIRE(run.types[0].report.first_failure()->name == "known_answer.residual");
}

TEST_CASE("bench ground truth checks kept-clone rotations as well as positions", "[tools][bench][s9]") {
    auto gt = Bench::ground_truth();
    // Rotate the first kept clone's truth by ~0.1 rad about z: canonical, unit, but wrong.
    gt.truth[0]["R"] = bn::json::array({std::cos(0.05), 0.0, 0.0, std::sin(0.05)});
    const auto run = bn::run_fixture<Bench>({"wrong_truth_rotation", gt}, bn::kShipped, bn::DefaultTypes{}, {"double"});
    REQUIRE_FALSE(run.pass());
    REQUIRE(run.types[0].report.first_failure()->name == "truth.kept_clone_rotation");
}

TEST_CASE("bench CSV escapes text fields; an empty selection fails the bench", "[tools][bench]") {
    REQUIRE(bn::csv_field("plain") == "plain");
    REQUIRE(bn::csv_field("a,b") == "\"a,b\"");
    REQUIRE(bn::csv_field("say \"hi\"") == "\"say \"\"hi\"\"\"");
    bn::CsvTable t({"x,y", "z"});
    t.row({1.0, 2.0});
    std::ostringstream out;
    t.write(out);
    REQUIRE(out.str().rfind("\"x,y\",z\n", 0) == 0);

    // --types naming no known type runs nothing: exit code 1, never a vacuous PASS.
    std::vector<std::string> args{"s9_marginalization_bench", "--types", "bogus"};
    std::vector<char*> argv;
    for (auto& a : args)
        argv.push_back(a.data());
    REQUIRE(bn::bench_main<Bench>(static_cast<int>(argv.size()), argv.data()) == 1);
}
