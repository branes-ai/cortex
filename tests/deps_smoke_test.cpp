// Smoke test for the FetchContent-pinned foundational deps.
//
// This test does not exercise behavior — it only confirms that every
// dep's headers + link symbols are reachable through the cmake target
// graph. If a dependency bump breaks any of these includes, this is
// the first thing CI will catch.

#include <catch2/catch_test_macros.hpp>

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
}
