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

# C++↔Rust cxx::bridge integration test. Links against the Corrosion-
# generated `cortex_core_cxx` target (which carries proper include
# paths for the cxx-generated `cortex-core/src/bridge.rs.h` header)
# plus `cortex_core` (the Rust staticlib).
add_executable(cxx_bridge_test cxx_bridge_test.cpp)
target_link_libraries(cxx_bridge_test PRIVATE
    Catch2::Catch2WithMain
    cortex_core_cxx
    cortex_core
    branes::test_support
)
target_compile_features(cxx_bridge_test PRIVATE cxx_std_20)

catch_discover_tests(cxx_bridge_test)

# Math layer — TensorView round-trip + stride-math (issue #24). Links
# branes::math (header-only) plus Universal for the posit element-type
# coverage.
add_executable(math_tensor_test math_tensor_test.cpp)
target_link_libraries(math_tensor_test PRIVATE
    Catch2::Catch2WithMain
    branes::math
    universal
    branes::test_support
)
target_compile_features(math_tensor_test PRIVATE cxx_std_20)

catch_discover_tests(math_tensor_test)

# Math layer — arithmetic-type plumbing (issue #25). Instantiates an
# MTL5 expression (y = A*x + b) against IEEE + Universal scalar types to
# prove type-genericity. Registered as `math_type_smoke`.
add_executable(math_type_smoke math_type_smoke_test.cpp)
target_link_libraries(math_type_smoke PRIVATE
    Catch2::Catch2WithMain
    branes::math
    universal
    branes::test_support
)
target_compile_features(math_type_smoke PRIVATE cxx_std_20)

# Prefix the discovered ctest names so `ctest -R math_type_smoke`
# (the issue #25 acceptance command) selects this suite.
catch_discover_tests(math_type_smoke TEST_PREFIX "math_type_smoke.")

# Math layer — sparse storage views CSR/CSC/COO (issue #26). Element
# access + conversions validated against a dense reference.
add_executable(math_sparse_test math_sparse_test.cpp)
target_link_libraries(math_sparse_test PRIVATE
    Catch2::Catch2WithMain
    branes::math
    universal
    branes::test_support
)
target_compile_features(math_sparse_test PRIVATE cxx_std_20)

catch_discover_tests(math_sparse_test)

# Math layer — Lie groups SO3/SE3/Sim3 (issue #29). Round-trip + FD
# Jacobian checks.
add_executable(math_lie_test math_lie_test.cpp)
target_link_libraries(math_lie_test PRIVATE
    Catch2::Catch2WithMain
    branes::math
    universal
    branes::test_support
)
target_compile_features(math_lie_test PRIVATE cxx_std_20)

catch_discover_tests(math_lie_test)
