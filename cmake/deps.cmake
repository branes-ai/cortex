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

# ── stillwater-sc/mtl5 — header-only matrix template library ────────
set(MTL5_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(MTL5_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    mtl5
    GIT_REPOSITORY https://github.com/stillwater-sc/mtl5.git
    GIT_TAG        v5.2.1
    GIT_SHALLOW    TRUE
)

# ── stillwater-sc/universal — number systems (posits, cfloat, …) ────
# Universal's CMakeLists.txt uses ${CMAKE_SOURCE_DIR} (which under
# FetchContent resolves to the parent project, not Universal). Skipping
# its add_subdirectory and exposing it as a manual INTERFACE target —
# Universal is header-only, so we only need its include dir.
FetchContent_Declare(
    universal
    GIT_REPOSITORY https://github.com/stillwater-sc/universal.git
    GIT_TAG        v4.7.0
    GIT_SHALLOW    TRUE
)

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

FetchContent_MakeAvailable(mtl5 yaml-cpp Catch2 Tracy stb nlohmann_json)

# Universal: populate without adding the subdirectory (its CMakeLists
# uses CMAKE_SOURCE_DIR which is wrong under FetchContent). CMake 3.30+
# warns about FetchContent_Populate; there is no non-deprecated way to
# fetch-without-add_subdirectory yet, so we opt into the OLD policy
# locally. Track upstream Universal fix to remove this hack.
FetchContent_GetProperties(universal)
if(NOT universal_POPULATED)
    if(POLICY CMP0169)
        cmake_policy(PUSH)
        cmake_policy(SET CMP0169 OLD)
    endif()
    FetchContent_Populate(universal)
    if(POLICY CMP0169)
        cmake_policy(POP)
    endif()
endif()
if(NOT TARGET universal)
    add_library(universal INTERFACE)
    target_include_directories(universal INTERFACE ${universal_SOURCE_DIR}/include/sw)
    target_compile_features(universal INTERFACE cxx_std_20)
    add_library(Universal::universal ALIAS universal)
endif()

# stb has no CMakeLists, so wrap stb_image as an INTERFACE target
# rooted at the repo dir.
if(NOT TARGET branes_stb_image)
    add_library(branes_stb_image INTERFACE)
    target_include_directories(branes_stb_image INTERFACE ${stb_SOURCE_DIR})
    add_library(branes::stb_image ALIAS branes_stb_image)
endif()

# Make Catch2's catch_discover_tests helper available to tests/.
list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras)
