// Smoke test for the FetchContent-pinned foundational deps.
//
// This test does not exercise behavior — it only confirms that every
// dep's headers + link symbols are reachable through the cmake target
// graph. If a dependency bump breaks any of these includes, this is
// the first thing CI will catch.

#include <catch2/catch_test_macros.hpp>

// Branes test support — verify the timing + budgets headers compile
// and the measure_n helper produces sensible stats. Real budget
// assertions land with the operator PRs that populate budgets.hpp.
#include <branes/test/budgets.hpp>
#include <branes/test/timing.hpp>

// MTL5 — pull in the lightweight config header.
#include <mtl/config.hpp>

// Universal — instantiate one number type from the posit family.
#include <universal/number/posit/posit.hpp>

// yaml-cpp — parse a trivial inline document.
#include <yaml-cpp/yaml.h>

// stb_image — define the implementation in this single TU. The bigger
// load/decode path stays unexercised here; just verify the header
// compiles and exports the expected free functions.
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

TEST_CASE("foundational deps are reachable", "[smoke][deps]") {
    SECTION("Universal posit instantiates") {
        sw::universal::posit<32, 2> p{1.5};
        REQUIRE(double(p) == 1.5);
    }

    SECTION("yaml-cpp parses a trivial document") {
        auto node = YAML::Load("key: 42");
        REQUIRE(node["key"].as<int>() == 42);
    }

    SECTION("stb_image exposes its loader symbol") {
        // We don't call it (no fixture data here); referencing it
        // forces the linker to resolve the symbol, which is the real
        // smoke check.
        auto fn = &stbi_load;
        REQUIRE(fn != nullptr);
    }

    SECTION("branes::test::measure_n produces sane stats") {
        // Trivial workload; we only check that the helper compiles,
        // returns the right sample count, and orders min <= median
        // <= p99 <= max. Budget assertions land with Phase 3.
        auto stats = branes::test::measure_n(20, [] {
            volatile int x = 0;
            for (int i = 0; i < 100; ++i) {
                x += i;
            }
            (void)x;
        });
        REQUIRE(stats.samples == 20);
        REQUIRE(stats.min <= stats.median);
        REQUIRE(stats.median <= stats.p99);
        REQUIRE(stats.p99 <= stats.max);
        // budgets::unset is the placeholder — any positive measurement
        // satisfies it.
        REQUIRE(stats.p99 < branes::test::budgets::unset);
    }
}
