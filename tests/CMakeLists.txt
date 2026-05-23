# Top-level test targets that aren't owned by a single subdirectory.

include(Catch)

# INTERFACE library exposing the latency-budget + timing helpers under
# branes/test/. Test executables link against this rather than each
# duplicating the include-path setup.
add_library(branes_test_support INTERFACE)
target_include_directories(branes_test_support INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(branes_test_support INTERFACE cxx_std_20)
add_library(branes::test_support ALIAS branes_test_support)

add_executable(deps_smoke_test deps_smoke_test.cpp)
target_link_libraries(deps_smoke_test PRIVATE
    Catch2::Catch2WithMain
    MTL5::mtl5
    universal
    branes::stb_image
    yaml-cpp::yaml-cpp
    branes::test_support
)
target_compile_features(deps_smoke_test PRIVATE cxx_std_20)

catch_discover_tests(deps_smoke_test)
