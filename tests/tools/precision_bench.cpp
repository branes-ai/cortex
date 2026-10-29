// Stage-bench precision characterization (issue #450): `--precision` runs a
// stage in every arithmetic type and measures the error against the double run,
// per output group, in bits of the type's precision.
//
// Locks the metric (normwise relative error, bits lost, the treatment of
// non-finite values), the rounded-input reference that separates the stage's
// own arithmetic from the input's rounding, and the condition-number helper,
// and checks the mode end to end on S9: marginalization is a gather, so it adds
// no rounding of its own in any type.
//
// ASCII-only TEST_CASE names (the Windows MSVC ctest job rejects non-ASCII).

#include <branes/tools/bench/s9_marginalization_bench.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
using Bench = bn::S9MarginalizationBench;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

}  // namespace

TEST_CASE("precision: normwise relative error and its non-finite cases", "[tools][bench][precision]") {
    CHECK(bn::normwise_rel_err({1.0, 2.0, -4.0}, {1.0, 2.0, -4.0}) == 0.0);
    // ‖Δ‖∞ / ‖ref‖∞: the largest difference over the largest reference entry.
    CHECK(bn::normwise_rel_err({1.5, 2.0, -4.0}, {1.0, 2.0, -4.0}) == Catch::Approx(0.125));
    // An all-zero reference: the absolute difference.
    CHECK(bn::normwise_rel_err({0.0, 1e-3}, {0.0, 0.0}) == Catch::Approx(1e-3));
    // A shape change, or a value finite on one side only, is infinitely wrong.
    CHECK(bn::normwise_rel_err({1.0}, {1.0, 2.0}) == kInf);
    CHECK(bn::normwise_rel_err({kNaN, 2.0}, {1.0, 2.0}) == kInf);
    CHECK(bn::normwise_rel_err({1.0, 2.0}, {kInf, 2.0}) == kInf);
    // Non-finite in both: not a difference.
    CHECK(bn::normwise_rel_err({kNaN, 2.0}, {kNaN, 2.0}) == 0.0);
    // A noise floor: a ~0 reference is measured against the floor, not itself.
    CHECK(bn::normwise_rel_err({1e-9}, {1e-15}, 1e-3) == Catch::Approx(1e-6));
    CHECK(bn::normwise_rel_err({3.0}, {2.0}, 1e-3) == Catch::Approx(0.5));  // the floor only floors
    CHECK(bn::normwise_abs_err({1.5, 2.0}, {1.0, 2.25}) == Catch::Approx(0.5));
}

TEST_CASE("precision: bits lost is log2 of the error in units of eps", "[tools][bench][precision]") {
    const double eps = std::numeric_limits<float>::epsilon();
    CHECK(bn::PrecisionRow::bits_lost(0.0, eps) == 0.0);
    CHECK(bn::PrecisionRow::bits_lost(eps / 4, eps) == 0.0);  // below one ulp: nothing lost
    CHECK(bn::PrecisionRow::bits_lost(8 * eps, eps) == Catch::Approx(3.0));
    // Capped at the precision: an error as large as the value leaves no bit.
    CHECK(bn::PrecisionRow::bits_lost(kInf, eps) == Catch::Approx(23.0));
    CHECK(bn::PrecisionRow::bits_lost(10.0, eps) == Catch::Approx(23.0));
    CHECK(bn::PrecisionRow::bits_lost(kNaN, eps) == Catch::Approx(23.0));
    CHECK(bn::PrecisionRow::amplification(8 * eps, eps) == Catch::Approx(8.0));
}

TEST_CASE("precision: log10 condition number of a symmetric matrix", "[tools][bench][precision]") {
    branes::sdk::msckf::DynMat<double> a(3, 3);
    a(0, 0) = 1.0;
    a(1, 1) = 1e-3;
    a(2, 2) = 1e-6;
    CHECK(bn::log10_cond_sym(a) == Catch::Approx(6.0).margin(1e-9));
    // Singular: lambda_min floors at eps_double * lambda_max.
    a(2, 2) = 0.0;
    CHECK(bn::log10_cond_sym(a) == Catch::Approx(-std::log10(std::numeric_limits<double>::epsilon())));
    a(0, 0) = kNaN;
    CHECK(std::isnan(bn::log10_cond_sym(a)));
}

TEST_CASE("precision: a quaternion rounded in float decodes in double only as rounded input",
          "[tools][bench][precision]") {
    // A unit quaternion as float holds it: unit-norm to float, not to double.
    const float h = std::sqrt(0.5f);
    const bn::json q = bn::json::array({static_cast<double>(h), static_cast<double>(h), 0.0, 0.0});
    REQUIRE(std::abs(2.0 * static_cast<double>(h) * static_cast<double>(h) - 1.0) >
            64.0 * std::numeric_limits<double>::epsilon());
    CHECK_THROWS(bn::unpack_so3<double>(q));
    {
        const bn::DecodeFromRounded scope(std::numeric_limits<float>::epsilon());
        const auto r = bn::unpack_so3<double>(q);
        const auto& d = r.quaternion();
        CHECK(std::abs(d[0] * d[0] + d[1] * d[1] + d[2] * d[2] + d[3] * d[3] - 1.0) <=
              4.0 * std::numeric_limits<double>::epsilon());
    }
    // The scope ends with the guard: strict again.
    CHECK_THROWS(bn::unpack_so3<double>(q));
}

TEST_CASE("precision: S9 adds no rounding of its own in any type", "[tools][bench][precision][s9]") {
    // The built-ins and the ladder's two ends (kappa ~ 1 and ~ 1e12), in the
    // IEEE types and one posit (emulated arithmetic is slow in a debug build).
    auto fixtures = Bench::builtin_fixtures();
    const auto ladder = Bench::precision_fixtures();
    REQUIRE(ladder.size() == 4);
    fixtures.push_back(ladder.front());
    fixtures.push_back(ladder.back());
    const std::vector<std::string> types{"double", "float", "posit32", "long_double"};
    std::size_t rows = 0;
    for (const auto& nf : fixtures) {
        const auto pr = bn::precision_rows<Bench>(nf, "shipped", bn::bench_types<Bench>(), types);
        REQUIRE(pr.size() == types.size());
        for (const auto& r : pr) {
            INFO(nf.name << " in " << r.type);
            CHECK(r.status == "ok");
            CHECK(r.failed == 0);
            CHECK(r.conditioning.size() == Bench::conditioning_columns().size());
            REQUIRE(r.rel_err.size() == 4);  // rot, pos, vel_bias, P
            for (std::size_t g = 0; g < r.rel_err.size(); ++g) {
                INFO("group " << r.rel_err[g].first);
                // A gather: the T run equals double on the input as T holds it —
                // exactly, but for the attitudes, which that reference
                // renormalizes in double (less than one unit of T).
                if (r.rel_err[g].first == "rot")
                    CHECK(bn::PrecisionRow::bits_lost(r.stage_err[g], r.eps) == 0.0);
                else
                    CHECK(r.stage_err[g] == 0.0);
                // And the input's own rounding is within one unit of T.
                CHECK(r.rel_err[g].second <= r.eps);
                if (r.type == "double")
                    CHECK(r.rel_err[g].second == 0.0);
            }
            ++rows;
        }
    }
    CHECK(rows == fixtures.size() * types.size());
}

TEST_CASE("precision: the CSV has one row per fixture, type and output group", "[tools][bench][precision]") {
    const auto nf = Bench::builtin_fixtures().front();
    const auto rows = bn::precision_rows<Bench>(nf, "shipped", bn::bench_types<Bench>(), {"double", "float"});
    const auto path = std::filesystem::temp_directory_path() / "branes_precision_test" / "S9_precision.csv";
    bn::write_precision_csv(rows, Bench::conditioning_columns(), path);
    std::ifstream in(path);
    std::string header, line;
    std::getline(in, header);
    CHECK(header.rfind("fixture,kind,variant,type,dim,log10_cond_P,group,eps,abs_err,rel_err,", 0) == 0);
    std::size_t n = 0;
    while (std::getline(in, line))
        ++n;
    CHECK(n == 2 * 4);
    std::filesystem::remove_all(path.parent_path());
}
