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

# Math layer — Krylov solver wrappers over MTL5 ITL (issue #27).
add_executable(math_krylov_test math_krylov_test.cpp)
target_link_libraries(math_krylov_test PRIVATE
    Catch2::Catch2WithMain
    branes::math
    universal
    branes::test_support
)
target_compile_features(math_krylov_test PRIVATE cxx_std_20)

catch_discover_tests(math_krylov_test)

# Math layer — direct sparse Cholesky/LDLT (issue #28).
add_executable(math_sparse_direct_test math_sparse_direct_test.cpp)
target_link_libraries(math_sparse_direct_test PRIVATE
    Catch2::Catch2WithMain
    branes::math
    universal
    branes::test_support
)
target_compile_features(math_sparse_direct_test PRIVATE cxx_std_20)

catch_discover_tests(math_sparse_direct_test)

# Math layer — NLS solvers GN/LM/Dogleg (issue #30).
add_executable(math_nls_test math_nls_test.cpp)
target_link_libraries(math_nls_test PRIVATE
    Catch2::Catch2WithMain
    branes::math
    universal
    branes::test_support
)
target_compile_features(math_nls_test PRIVATE cxx_std_20)

catch_discover_tests(math_nls_test)

# Math layer — golden-data NLS suite + FD Jacobian + type matrix (#31).
add_executable(math_nls_golden_test math_nls_golden_test.cpp)
target_link_libraries(math_nls_golden_test PRIVATE
    Catch2::Catch2WithMain
    branes::math
    universal
    branes::test_support
)
target_compile_features(math_nls_golden_test PRIVATE cxx_std_20)

catch_discover_tests(math_nls_golden_test)

# SDK layer — VioBackend interface (issue #34).
add_executable(sdk_vio_backend_test sdk_vio_backend_test.cpp)
target_link_libraries(sdk_vio_backend_test PRIVATE
    Catch2::Catch2WithMain
    branes::sdk
    branes::test_support
)
target_compile_features(sdk_vio_backend_test PRIVATE cxx_std_20)

catch_discover_tests(sdk_vio_backend_test)

# SDK layer — IMU preintegration (issue #40).
add_executable(sdk_imu_preintegration_test sdk_imu_preintegration_test.cpp)
target_link_libraries(sdk_imu_preintegration_test PRIVATE
    Catch2::Catch2WithMain
    branes::sdk
    branes::test_support
)
target_compile_features(sdk_imu_preintegration_test PRIVATE cxx_std_20)

catch_discover_tests(sdk_imu_preintegration_test)

# SDK layer — feature representations + MSCKF null-space (issue #41).
add_executable(sdk_features_test sdk_features_test.cpp)
target_link_libraries(sdk_features_test PRIVATE
    Catch2::Catch2WithMain
    branes::sdk
    branes::test_support
)
target_compile_features(sdk_features_test PRIVATE cxx_std_20)

catch_discover_tests(sdk_features_test)

# SDK layer — MSCKF State + Propagator + StateHelper (issue #43).
add_executable(sdk_msckf_state_test sdk_msckf_state_test.cpp)
target_link_libraries(sdk_msckf_state_test PRIVATE
    Catch2::Catch2WithMain
    branes::sdk
    branes::test_support
)
target_compile_features(sdk_msckf_state_test PRIVATE cxx_std_20)

catch_discover_tests(sdk_msckf_state_test)

# SDK layer — IMU initialization (static + dynamic) (issue #42).
add_executable(sdk_imu_init_test sdk_imu_init_test.cpp)
target_link_libraries(sdk_imu_init_test PRIVATE
    Catch2::Catch2WithMain
    branes::sdk
    branes::test_support
)
target_compile_features(sdk_imu_init_test PRIVATE cxx_std_20)

catch_discover_tests(sdk_imu_init_test)

# SDK layer — MSCKF camera updater (mono/stereo null-space) (issue #44).
add_executable(sdk_msckf_camera_test sdk_msckf_camera_test.cpp)
target_link_libraries(sdk_msckf_camera_test PRIVATE
    Catch2::Catch2WithMain
    branes::sdk
    branes::test_support
)
target_compile_features(sdk_msckf_camera_test PRIVATE cxx_std_20)

catch_discover_tests(sdk_msckf_camera_test)

# CV layer — Image container + I/O (issue #48). stb_image provides the
# stb_image_write.h header used to encode PNG fixtures in the test.
add_executable(cv_image_test cv_image_test.cpp)
target_link_libraries(cv_image_test PRIVATE
    Catch2::Catch2WithMain
    branes::cv
    branes::stb_image
    branes::test_support
)
target_compile_features(cv_image_test PRIVATE cxx_std_20)

catch_discover_tests(cv_image_test)

# CV layer — Gaussian image pyramid (issue #49).
add_executable(cv_pyramid_test cv_pyramid_test.cpp)
target_link_libraries(cv_pyramid_test PRIVATE
    Catch2::Catch2WithMain
    branes::cv
    branes::test_support
)
target_compile_features(cv_pyramid_test PRIVATE cxx_std_20)

catch_discover_tests(cv_pyramid_test)

# Math layer — camera models (issue #39).
add_executable(math_cameras_test math_cameras_test.cpp)
target_link_libraries(math_cameras_test PRIVATE
    Catch2::Catch2WithMain
    branes::math
    universal
    branes::test_support
)
target_compile_features(math_cameras_test PRIVATE cxx_std_20)

catch_discover_tests(math_cameras_test)

# CV layer — FAST corner detector (issue #37).
add_executable(cv_fast_test cv_fast_test.cpp)
target_link_libraries(cv_fast_test PRIVATE
    Catch2::Catch2WithMain
    branes::cv
    branes::test_support
)
target_compile_features(cv_fast_test PRIVATE cxx_std_20)

catch_discover_tests(cv_fast_test)

# CV layer — KLT pyramidal tracker (issue #38).
add_executable(cv_klt_test cv_klt_test.cpp)
target_link_libraries(cv_klt_test PRIVATE
    Catch2::Catch2WithMain
    branes::cv
    branes::test_support
)
target_compile_features(cv_klt_test PRIVATE cxx_std_20)

catch_discover_tests(cv_klt_test)
