# cmake/deps.cmake — FetchContent declarations for foundational deps.
#
# Anything fetched here is permissively-licensed (MIT / BSL / BSD / dual).
# Strong-copyleft dependencies are rejected — see docs/adr/0001.
#
# Pinned versions are intentional. Bumps go via dedicated PRs so the
# release-please changelog records dependency churn separately from
# feature work.

include(FetchContent)

# Don't re-run `git fetch` on every build. Devs/CI bump tags here
# explicitly when they want new code.
set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "" FORCE)

# ── stillwater-sc/mtl5 + stillwater-sc/universal ────────────────────
# Both are header-only Stillwater sister projects, integrated with the
# pattern from the Stillwater mixed-precision repos (mp-iterative, mp-blas):
#
#   1. find_package(<dep> CONFIG QUIET) first — an installed copy wins.
#   2. Otherwise a header-only FetchContent: SOURCE_SUBDIR names a directory
#      with no CMakeLists.txt, so FetchContent_MakeAvailable populates the
#      sources but never add_subdirectory()s the dependency. Its own targets,
#      tests, and configure logic stay out of our build, and no deprecated
#      FetchContent_Populate (CMP0169) is needed.
#   3. Either path yields the same namespaced targets, MTL5::mtl5 and
#      universal::universal, so consumers never care which one ran.
#
# Co-develop against local sister checkouts (no network) with:
#   -DFETCHCONTENT_SOURCE_DIR_MTL5=/path/to/mtl5
#   -DFETCHCONTENT_SOURCE_DIR_UNIVERSAL=/path/to/universal
#
# Third-party warnings: the include dirs are plain INTERFACE, not SYSTEM, on
# purpose. Universal and MTL5 are sister projects we can fix, and keeping
# their warnings visible has surfaced real bugs (stillwater-sc/universal#1259,
# #1262). Switch to SYSTEM INTERFACE once the warning-clean epic
# stillwater-sc/universal#1265 closes.
set(BRANES_MTL5_VERSION      5.12.0)
set(BRANES_UNIVERSAL_VERSION 5.1.0)

find_package(MTL5 CONFIG QUIET)
if(MTL5_FOUND)
    message(STATUS "MTL5: using installed package ${MTL5_VERSION} (${MTL5_DIR})")
else()
    if(FETCHCONTENT_SOURCE_DIR_MTL5)
        message(STATUS "MTL5: using local checkout ${FETCHCONTENT_SOURCE_DIR_MTL5} (pin v${BRANES_MTL5_VERSION} bypassed)")
    else()
        message(STATUS "MTL5: fetching v${BRANES_MTL5_VERSION} headers")
    endif()
    FetchContent_Declare(
        mtl5
        GIT_REPOSITORY https://github.com/stillwater-sc/mtl5.git
        GIT_TAG        v${BRANES_MTL5_VERSION}
        GIT_SHALLOW    TRUE
        SOURCE_SUBDIR  _header_only_no_build
    )
    FetchContent_MakeAvailable(mtl5)

    # MTL5's configure step normally generates mtl/version.hpp, which the
    # <mtl/mtl.hpp> umbrella includes. We skip that configure step, so
    # generate it here from MTL5's own template. (build_info.hpp and
    # testsuite_config.hpp, the other generated headers, are only used by
    # MTL5's benchmark/test-matrix utilities, which cortex doesn't include.)
    # With a local checkout the macros still carry the pinned version.
    string(REPLACE "." ";" _mtl5_ver "${BRANES_MTL5_VERSION}")
    list(GET _mtl5_ver 0 MTL5_VERSION_MAJOR)
    list(GET _mtl5_ver 1 MTL5_VERSION_MINOR)
    list(GET _mtl5_ver 2 MTL5_VERSION_PATCH)
    set(MTL5_VERSION "${BRANES_MTL5_VERSION}")
    configure_file(
        ${mtl5_SOURCE_DIR}/include/mtl/version.hpp.in
        ${mtl5_BINARY_DIR}/include/mtl/version.hpp
        @ONLY)

    add_library(mtl5 INTERFACE)
    add_library(MTL5::mtl5 ALIAS mtl5)
    target_include_directories(mtl5 INTERFACE
        ${mtl5_SOURCE_DIR}/include
        ${mtl5_BINARY_DIR}/include)
    target_compile_features(mtl5 INTERFACE cxx_std_20)
endif()

find_package(universal CONFIG QUIET)
if(universal_FOUND)
    message(STATUS "Universal: using installed package ${universal_VERSION} (${universal_DIR})")
else()
    if(FETCHCONTENT_SOURCE_DIR_UNIVERSAL)
        message(STATUS "Universal: using local checkout ${FETCHCONTENT_SOURCE_DIR_UNIVERSAL} (pin v${BRANES_UNIVERSAL_VERSION} bypassed)")
    else()
        message(STATUS "Universal: fetching v${BRANES_UNIVERSAL_VERSION} headers")
    endif()
    FetchContent_Declare(
        universal
        GIT_REPOSITORY https://github.com/stillwater-sc/universal.git
        GIT_TAG        v${BRANES_UNIVERSAL_VERSION}
        GIT_SHALLOW    TRUE
        SOURCE_SUBDIR  _header_only_no_build
    )
    FetchContent_MakeAvailable(universal)

    # Universal headers use two include conventions:
    #   - external: #include <sw/universal/...>  (needs include/)
    #   - internal: #include <universal/...>     (needs include/sw/)
    # cortex uses the internal form; expose both.
    add_library(universal INTERFACE)
    add_library(universal::universal ALIAS universal)
    target_include_directories(universal INTERFACE
        ${universal_SOURCE_DIR}/include
        ${universal_SOURCE_DIR}/include/sw)
    target_compile_features(universal INTERFACE cxx_std_20)
endif()

# ── jbeder/yaml-cpp ─────────────────────────────────────────────────
set(YAML_CPP_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL       OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        yaml-cpp-0.9.0
    GIT_SHALLOW    TRUE
)

# ── catchorg/Catch2 v3 — testing framework ──────────────────────────
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.15.0
    GIT_SHALLOW    TRUE
)

# ── wolfpld/tracy — profiler client ────────────────────────────────
# A no-op stub unless TRACY_ENABLE is set, so it's safe to always fetch.
# Phase 9 (#90-#94) wires up zones in SDK + Rust RM.
FetchContent_Declare(
    Tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG        v0.13.1
    GIT_SHALLOW    TRUE
)

# ── nothings/stb — single-header image loader (stb_image only) ──────
# stb has no releases, so we pin by commit. GIT_SHALLOW is not allowed
# when the tag is a raw SHA.
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        31c1ad37456438565541f4919958214b6e762fb4
)

# ── nlohmann/json — header-only JSON (the VIO trace bus, #372) ──────
# The inter-stage trace records (tools/include/branes/tools/vio_trace.hpp)
# are written AND read back by the per-stage inspectors, so the tools layer
# needs a real parser — the rest of the repo only ever wrote JSON by hand.
# Header-only; tests are off so it doesn't expand the build.
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(yaml-cpp Catch2 Tracy stb nlohmann_json)

# stb has no CMakeLists, so wrap stb_image as an INTERFACE target
# rooted at the repo dir.
if(NOT TARGET branes_stb_image)
    add_library(branes_stb_image INTERFACE)
    target_include_directories(branes_stb_image INTERFACE ${stb_SOURCE_DIR})
    add_library(branes::stb_image ALIAS branes_stb_image)
endif()

# Make Catch2's catch_discover_tests helper available to tests/.
list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras)
