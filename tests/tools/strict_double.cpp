// The genericity audit's strict-conversion type (issue #449).
//
// `StrictDouble` is double with only explicit conversions out. Every stage bench
// instantiates its stage in it (it is in `AllTypes`), so an implicit T → double
// narrowing on the estimation path is a compile error. Here every bench's
// fixtures, in every variant, must give the same bits in it as in double — the
// composition bench (C_filter_loop) runs the whole estimator — so being generic
// costs nothing at T = double.

#include <branes/tools/bench/c_filter_loop_bench.hpp>
#include <branes/tools/bench/s0_sensor_model_bench.hpp>
#include <branes/tools/bench/s10_online_calibration_bench.hpp>
#include <branes/tools/bench/s1_initialization_bench.hpp>
#include <branes/tools/bench/s2_propagation_bench.hpp>
#include <branes/tools/bench/s3_augmentation_bench.hpp>
#include <branes/tools/bench/s4_frontend_bench.hpp>
#include <branes/tools/bench/s5_triangulation_bench.hpp>
#include <branes/tools/bench/s6a_jacobians_bench.hpp>
#include <branes/tools/bench/s6b_nullspace_projection_bench.hpp>
#include <branes/tools/bench/s6c_compression_bench.hpp>
#include <branes/tools/bench/s6d_gating_bench.hpp>
#include <branes/tools/bench/s6e_ekf_update_bench.hpp>
#include <branes/tools/bench/s9_marginalization_bench.hpp>
#include <branes/tools/bench/strict_double.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
using S = bn::StrictDouble;

// Into T implicitly (as a posit allows); out of T only by an explicit cast.
static_assert(std::is_convertible_v<double, S>);
static_assert(std::is_convertible_v<int, S>);
static_assert(!std::is_convertible_v<S, double>);
static_assert(!std::is_convertible_v<S, float>);
static_assert(!std::is_convertible_v<S, int>);
static_assert(std::is_constructible_v<double, S>);  // static_cast<double>(t) still works
static_assert(std::is_trivially_default_constructible_v<S>);
static_assert(branes::math::OrderedField<S>);

bool same_bits(double a, double b) {
    return std::memcmp(&a, &b, sizeof a) == 0;
}

/// Every built-in fixture × variant of B: the strict_double output against the
/// double output, bit for bit. Returns the number of runs compared.
template <class B>
std::size_t require_bit_identical() {
    std::size_t runs = 0;
    for (const auto& nf : B::builtin_fixtures())
        for (const auto& v : B::variants()) {
            const auto d = B::template flatten<double>(
                B::template run<double>(B::template decode_input<double>(nf.fixture.input), v.name));
            const auto s =
                B::template flatten<S>(B::template run<S>(B::template decode_input<S>(nf.fixture.input), v.name));
            INFO(B::kStage << " / " << nf.name << " / " << v.name);
            REQUIRE(s.size() == d.size());
            std::size_t differ = 0;
            for (std::size_t i = 0; i < d.size(); ++i)
                differ += same_bits(s[i], d[i]) ? 0 : 1;
            REQUIRE(differ == 0);
            ++runs;
        }
    return runs;
}

}  // namespace

TEST_CASE("strict_double reproduces double bit for bit in every stage bench", "[tools][bench][generic]") {
    std::size_t runs = 0;
    runs += require_bit_identical<bn::S0SensorModelBench>();
    runs += require_bit_identical<bn::S1InitializationBench>();
    runs += require_bit_identical<bn::S2PropagationBench>();
    runs += require_bit_identical<bn::S3AugmentationBench>();
    runs += require_bit_identical<bn::S4FrontendBench>();
    runs += require_bit_identical<bn::S5TriangulationBench>();
    runs += require_bit_identical<bn::S6aJacobiansBench>();
    runs += require_bit_identical<bn::S6bNullspaceProjectionBench>();
    runs += require_bit_identical<bn::S6cCompressionBench>();
    runs += require_bit_identical<bn::S6dGatingBench>();
    runs += require_bit_identical<bn::S6eEkfUpdateBench>();
    runs += require_bit_identical<bn::S9MarginalizationBench>();
    runs += require_bit_identical<bn::S10OnlineCalibrationBench>();
    runs += require_bit_identical<bn::CFilterLoopBench>();  // the estimator end to end
    REQUIRE(runs >= 126);  // every output bench_snapshot tracks, and the S2 variants since
}

TEST_CASE("strict_double is a selectable bench type with double's limits", "[tools][bench][generic]") {
    REQUIRE(bn::type_name<S>() == "strict_double");
    const auto all = bn::type_names(bn::AllTypes{});
    REQUIRE(std::find(all.begin(), all.end(), std::string("strict_double")) != all.end());
    REQUIRE(static_cast<double>(std::numeric_limits<S>::epsilon()) == std::numeric_limits<double>::epsilon());
    REQUIRE(bn::eps_at<S>(1e6) == bn::eps_at<double>(1e6));
    S x(1.0);
    ++x;
    REQUIRE(static_cast<double>(x) == 1.0 + std::numeric_limits<double>::epsilon());
}
