cmake_minimum_required(VERSION 3.22)
project(branes_cortex LANGUAGES CXX)

# Enforce modern C++ for MTL5 and std::span
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
    GIT_TAG v0.4.2 # Use a stable release
)
FetchContent_MakeAvailable(Corrosion)

# 3. Import the Rust Core Resource Manager
corrosion_import_crate(MANIFEST_PATH core/Cargo.toml)

# 4. Add Subdirectories
add_subdirectory(math)
add_subdirectory(sdk)
add_subdirectory(daemons)
