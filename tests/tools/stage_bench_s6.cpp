// S6 sub-step stage benches: S6a_jacobians … S6e_ekf_update (issue #457).
//
// Locks what the benches establish: every fixture kind passes each sub-step's
// invariants for every variant (in double, float and long double; posit32 on the
// shipped variants); the update carries no information along the unobservable
// directions at a consistent linearization point, estimated or true; Givens and
// Householder null spaces give the same normal equations; QR compression of a
// stacked system keeps n + 1 rows; the shipped χ² gate is loose at low dof; a
// square-root filter that carries its factor matches the Joseph form in posit32,
// while re-factoring P in a narrow type loses the clone directions; and the
// committed fixture files replay.

#include <branes/tools/bench/s6a_jacobians_bench.hpp>
#include <branes/tools/bench/s6b_nullspace_projection_bench.hpp>
#include <branes/tools/bench/s6c_compression_bench.hpp>
#include <branes/tools/bench/s6d_gating_bench.hpp>
#include <branes/tools/bench/s6e_ekf_update_bench.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
using S6a = bn::S6aJacobiansBench;
using S6b = bn::S6bNullspaceProjectionBench;
using S6c = bn::S6cCompressionBench;
using S6d = bn::S6dGatingBench;
using S6e = bn::S6eEkfUpdateBench;

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

template <class B>
void require_all(const std::vector<std::string>& types) {
    for (const auto& v : B::variants())
        for (const auto& nf : B::builtin_fixtures())
            require_pass<B>(nf, v.name, types);
}

double value_of(const bn::FixtureRun& run, std::string_view name, std::size_t type = 0) {
    for (const auto& r : run.types.at(type).report.results())
        if (r.name == name)
            return r.value;
    FAIL("no result named " << name);
    return 0.0;
}

const std::vector<std::string> kIeee{"double", "float", "long_double"};

}  // namespace

TEST_CASE("S6 benches: every fixture passes for every variant in the IEEE types", "[tools][bench][s6]") {
    require_all<S6a>(kIeee);
    require_all<S6b>(kIeee);
    require_all<S6c>(kIeee);
    require_all<S6d>(kIeee);
    require_all<S6e>(kIeee);
}

TEST_CASE("S6 benches: the shipped sub-steps pass in posit32", "[tools][bench][s6]") {
    // Posits are software arithmetic: slow at -O0, so the shipped variant only.
    for (const auto& nf : S6a::builtin_fixtures())
        require_pass<S6a>(nf, bn::kShipped, {"posit32"});
    for (const auto& nf : S6b::builtin_fixtures())
        require_pass<S6b>(nf, bn::kShipped, {"posit32"});
    for (const auto& nf : S6d::builtin_fixtures())
        require_pass<S6d>(nf, bn::kShipped, {"posit32"});
    require_pass<S6e>({"ka", S6e::known_answer()}, bn::kShipped, {"posit32"});
}

TEST_CASE("S6 update: no information along the unobservable directions, true or estimated", "[tools][bench][s6]") {
    // A camera update evaluated at one consistent point satisfies H0 N = 0 to
    // roundoff, at the estimated state as much as at the true one: the update is
    // not where yaw and global position gain spurious information (#212).
    for (const auto& f : {S6b::ground_truth(), S6b::captured()}) {
        const auto run = require_pass<S6b>({"f", f}, bn::kShipped, {"double"});
        REQUIRE(value_of(run, "projected.annihilates_unobservable") < 1e-14);
    }
    for (const auto& f : {S6e::ground_truth(), S6e::captured()})
        require_pass<S6e>({"f", f}, bn::kShipped, {"double"});
}

TEST_CASE("S6b: Givens and Householder null spaces give the same normal equations", "[tools][bench][s6]") {
    const auto in = S6b::decode_input<double>(S6b::captured().input);
    const auto h = S6b::run<double>(in, "shipped");
    const auto g = S6b::run<double>(in, "givens");
    namespace m = branes::sdk::msckf;
    const auto hh = m::mul(m::transpose(h.projected.H), h.projected.H);
    const auto gg = m::mul(m::transpose(g.projected.H), g.projected.H);
    REQUIRE(bn::s6::rel_diff(gg, hh) < 1e-13);
}

TEST_CASE("S6c: QR compresses a stacked system to n + 1 rows", "[tools][bench][s6]") {
    const auto in = S6c::decode_input<double>(S6c::captured().input);
    const auto qr = S6c::run<double>(in, "qr");
    REQUIRE(qr.rows_in == 72);
    REQUIRE(qr.rows_out == in.stacked.H.cols + 1);
    const auto id = S6c::run<double>(in, bn::kShipped);
    REQUIRE(id.rows_out == id.rows_in);
}

TEST_CASE("S6d sweep: inliers are chi-square consistent; the shipped gate is loose at low dof", "[tools][bench][s6]") {
    // Columns: 0 NIS/dof mean, 1 shipped accept rate, 2 chi2_95 accept rate, 3-4 band.
    const auto in = S6d::sweep_point<double>({{"dof", 9.0}, {"outlier_sigma", 0.0}});
    REQUIRE(in[0] > in[3]);
    REQUIRE(in[0] < in[4]);
    REQUIRE(std::abs(in[2] - 0.95) < 0.05);
    const auto out = S6d::sweep_point<double>({{"dof", 3.0}, {"outlier_sigma", 2.0}});
    REQUIRE(out[1] > 0.5);           // 5 per dof admits most 2-sigma-biased measurements
    REQUIRE(out[2] < out[1] - 0.3);  // a 95% chi-square gate admits far fewer
}

TEST_CASE("S6e: a square-root filter that carries its factor matches Joseph in posit32", "[tools][bench][s6]") {
    // One run per variant and type (the full fixture run, with timing reps and
    // every invariant, is minutes of software posit arithmetic at -O0).
    const auto f = S6e::captured();
    const auto in_d = S6e::decode_input<double>(f.input);
    const auto in_p = S6e::decode_input<bn::Posit32>(f.input);
    auto gap = [&](std::string_view variant) {
        const auto ref = S6e::flatten(S6e::run<double>(in_d, variant));
        const auto got = S6e::flatten(S6e::run<bn::Posit32>(in_p, variant));
        REQUIRE(ref.size() == got.size());
        double worst = 0.0;
        for (std::size_t i = 0; i < ref.size(); ++i) {
            REQUIRE(std::isfinite(ref[i]));
            REQUIRE(std::isfinite(got[i]));
            worst = std::max(worst, std::abs(got[i] - ref[i]));
        }
        return worst;
    };
    const double dj = gap("shipped"), ds = gap("sqrt_array");
    INFO("Joseph " << dj << ", sqrt array " << ds);
    REQUIRE(dj < 1e-7);
    REQUIRE(ds < 4.0 * dj);
}

TEST_CASE("S6e: re-factoring the covariance in a narrow type loses the clone directions", "[tools][bench][s6]") {
    // The clone blocks are almost perfectly correlated with the IMU pose: their
    // Schur-complement pivots are below float's resolution of P. Factoring P
    // in float zeroes them — a different covariance — where double zeroes only
    // the 6 exact ones cloning makes. A square-root filter must carry its factor.
    const auto in_d = S6e::decode_input<double>(S6e::captured().input);
    const auto in_f = S6e::decode_input<float>(S6e::captured().input);
    auto zero_pivots = [](const auto& P) {
        std::decay_t<decltype(P)> L;
        REQUIRE(bn::psd_factor(P, L));
        std::size_t z = 0;
        for (std::size_t j = 0; j < L.rows; ++j)
            z += L(j, j) == 0 ? 1 : 0;
        return z;
    };
    REQUIRE(zero_pivots(in_d.state.cov.P) == 6);
    REQUIRE(zero_pivots(in_f.state.cov.P) > 30);
}

TEST_CASE("S6e: the carried factor survives an encode/decode round trip", "[tools][bench][s6]") {
    // Decoded in float, the factor is chol(P) in double rounded once; re-encoding
    // must carry it rather than let a later decode re-derive it from the rounded P.
    const auto in = S6e::decode_input<float>(S6e::captured().input);
    REQUIRE(in.factor.rows == in.state.dim());
    const auto back = S6e::decode_input<float>(S6e::encode_input(in));
    REQUIRE(back.factor.rows == in.factor.rows);
    for (std::size_t i = 0; i < in.factor.d.size(); ++i)
        REQUIRE(back.factor.d[i] == in.factor.d[i]);
    auto bad = S6e::encode_input(in);
    bad["factor"] = bn::pack(bn::s6::Mat<float>(2, 2));
    REQUIRE_THROWS_AS(S6e::decode_input<float>(bad), std::invalid_argument);
}

TEST_CASE("S6 bench codecs reject malformed inputs", "[tools][bench][s6]") {
    auto a = S6a::known_answer().input;
    a.at("observations").at(0).at(1) = 1;
    REQUIRE_THROWS_AS(S6a::decode_input<double>(a), std::invalid_argument);
    auto b = S6b::known_answer().input;
    b.at("system").at("rows") = 5;
    REQUIRE_THROWS_AS(S6b::decode_input<double>(b), std::invalid_argument);
    auto d = S6d::known_answer().input;
    d.at("projected").at("r") = bn::json::array({"0x0p+0"});
    REQUIRE_THROWS_AS(S6d::decode_input<double>(d), std::invalid_argument);
}

TEST_CASE("S6 committed fixture files load and pass", "[tools][bench][s6]") {
    const std::filesystem::path dir(BRANES_BENCH_FIXTURE_DIR);
    auto each = [&](auto tag, const char* stage, std::initializer_list<const char*> names) {
        using B = typename decltype(tag)::type;
        for (const char* name : names) {
            INFO(stage << "/" << name);
            require_pass<B>({name, bn::load(dir / stage / (std::string(name) + ".json"))}, bn::kShipped, {"double"});
        }
    };
    each(std::type_identity<S6a>{},
         "s6a_jacobians",
         {"known_answer_three_clones", "ground_truth_synthetic_track", "captured_estimated_poses"});
    each(std::type_identity<S6b>{},
         "s6b_nullspace_projection",
         {"known_answer_identity_feature_block", "ground_truth_synthetic_track", "captured_estimated_poses"});
    each(std::type_identity<S6c>{},
         "s6c_compression",
         {"known_answer_small_system", "ground_truth_stacked_tracks", "captured_estimated_poses"});
    each(std::type_identity<S6d>{},
         "s6d_gating",
         {"known_answer_two_rows", "ground_truth_synthetic_track", "captured_estimated_poses"});
    each(std::type_identity<S6e>{},
         "s6e_ekf_update",
         {"known_answer_two_rows", "ground_truth_synthetic_track", "captured_estimated_poses"});
}
