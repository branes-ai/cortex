cmake_minimum_required(VERSION 3.22)
project(branes_cortex LANGUAGES CXX)

# Enforce modern C++ for MTL5 and std::span
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC needs two opt-ins that gcc/clang give by default:
#   /Zc:preprocessor — the conformant preprocessor; the legacy one mis-parses
#       raw-string literals passed as macro arguments (Catch2 REQUIRE(R"(...)")).
#   /utf-8           — treat sources and string literals as UTF-8; the codebase
#       uses Unicode in comments and report strings (e.g. ✅, °, →).
if(MSVC)
    add_compile_options(/Zc:preprocessor /utf-8)
endif()

# 1. Define the Hardware Target Flag (SITL vs KPU)
option(BUILD_TARGET_KPU "Build for the custom KPU silicon (DMA + kernel driver)" OFF)
if(BUILD_TARGET_KPU)
    add_compile_definitions(TARGET_KPU)
    # This environment variable tells Rust to compile the real kernel bindings
    set(ENV{RUST_HAL_TARGET} "KPU")
else()
    add_compile_definitions(TARGET_SITL)
    set(ENV{RUST_HAL_TARGET} "SITL")
endif()

# 2. Fetch and configure Corrosion (Rust <-> CMake bridge)
include(FetchContent)
FetchContent_Declare(
    Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    GIT_TAG v0.6.1 # v0.4.2 had a rustc --version parse bug on Rust 1.80+
)
FetchContent_MakeAvailable(Corrosion)

# 3. Import the Rust Core Resource Manager
corrosion_import_crate(MANIFEST_PATH core/Cargo.toml)

# 3a. Generate the C++ side of the cxx::bridge defined in
#     core/src/bridge.rs. Corrosion installs a matching cxxbridge-cmd
#     (cxx version pinned to 1.0.130 in core/Cargo.toml), runs it on
#     bridge.rs to emit bridge.rs.h + bridge.rs.cc, and exposes the
#     generated library via the cortex_core_cxx target with proper
#     INTERFACE include paths.
corrosion_add_cxxbridge(cortex_core_cxx
    CRATE cortex_core
    FILES bridge.rs  # path relative to <crate>/src/, auto-prefixed by Corrosion
)

# 4. Fetch foundational third-party deps (MTL5, Universal, yaml-cpp,
#    Catch2, Tracy, stb_image). All permissive licenses; see docs/adr/0001.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/deps.cmake)

# 5. Tests (enabled by default; use -DBUILD_TESTING=OFF to skip).
include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# 6. Project subdirectories
add_subdirectory(math)
add_subdirectory(cv)
add_subdirectory(sdk)
add_subdirectory(bench)
add_subdirectory(daemons)
