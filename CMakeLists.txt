cmake_minimum_required(VERSION 3.22)

# Opt into CMP0141 *before* project(), so MSVC debug-info format is governed by
# CMAKE_MSVC_DEBUG_INFORMATION_FORMAT instead of a hardcoded /Zi. CMake bakes
# the debug-info flag into CMAKE_CXX_FLAGS_DEBUG during project()'s language
# init under the policy in effect *then*; set after project() it is too late —
# /Zi is already baked in and inherited by every target, including the
# FetchContent deps (Catch2, yaml-cpp), which then fail the msvc-ninja /Z7 Debug
# build with C1041 (many cl.exe contending on one shared .pdb under Ninja).
# CMAKE_POLICY_DEFAULT_CMP0141 carries the NEW default into the deps' own
# project() calls. No-op pre-3.25 / on non-MSVC.
if(POLICY CMP0141)
    cmake_policy(SET CMP0141 NEW)
endif()
set(CMAKE_POLICY_DEFAULT_CMP0141 NEW)

project(branes_cortex LANGUAGES CXX)

# Enforce modern C++ for MTL5 and std::span
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Organize targets into folders in IDEs (Visual Studio, Xcode). The tests/
# tree groups its generated per-file targets under "Tests/<layer>" via this.
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# MSVC needs two opt-ins that gcc/clang give by default:
#   /Zc:preprocessor — the conformant preprocessor; the legacy one mis-parses
#       raw-string literals passed as macro arguments (Catch2 REQUIRE(R"(...)")).
#   /utf-8           — treat sources and string literals as UTF-8; the codebase
#       uses Unicode in comments and report strings (e.g. ✅, °, →).
if(MSVC)
    add_compile_options(/Zc:preprocessor /utf-8)

    # Use the dynamic *release* CRT (/MD) in every configuration, including
    # Debug. rustc on x86_64-pc-windows-msvc has no debug-CRT variant: the Rust
    # staticlib (cortex_core) and its cc-compiled cxx glue always link /MD
    # (_ITERATOR_DEBUG_LEVEL=0). A C++ Debug build's default /MDd (IDL=2) clashes
    # → LNK2038/LNK1319 when any target links the bridge (e.g. tests/integration/
    # cxx_bridge). Pinning /MD project-wide keeps every object's CRT consistent
    # with the Rust core, so the Visual Studio IDE can build & link the Debug
    # config (and run the debugger) — not just the CI Release build. Requires
    # CMP0091 (NEW by default at our cmake_minimum_required).
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

    # /RTC1 (runtime checks, in CMake's default MSVC Debug flags) requires the
    # debug CRT, so it is incompatible with the /MD pin above — strip it. A
    # Debug build keeps /Od + debug info, so source-level debugging is intact;
    # only the CRT debug heap and /RTC stack checks are given up (the price of
    # linking a Rust staticlib, which mandates the release CRT).
    string(REGEX REPLACE "/RTC[1csu]+" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REGEX REPLACE "/RTC[1csu]+" "" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
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
