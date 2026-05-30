# Regression tests. Each .cpp under a layer subdirectory becomes a Catch2 test
# executable automatically — see cmake/compile_all.cmake. To add a test, drop a
# `<layer>_<name>_test.cpp` into the matching subdirectory; no edit here is
# needed unless it pulls in an unusual library or needs a ctest name prefix.

include(${CMAKE_SOURCE_DIR}/cmake/compile_all.cmake)

# INTERFACE library exposing the latency-budget + timing helpers under
# branes/test/. Every test target links this (compile_all adds it), so tests
# don't each duplicate the include-path setup.
add_library(branes_test_support INTERFACE)
target_include_directories(branes_test_support INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(branes_test_support INTERFACE cxx_std_20)
add_library(branes::test_support ALIAS branes_test_support)

# ── Math layer (issues #24-#31, #39) ───────────────────────────────────────
# branes::math is header-only; Universal supplies the posit/cfloat element
# types the math suite instantiates against. math_type_smoke carries a prefix
# so `ctest -R math_type_smoke` (the issue #25 acceptance) selects it.
file(GLOB MATH_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/math/*_test.cpp)
list(REMOVE_ITEM MATH_SRC ${CMAKE_CURRENT_SOURCE_DIR}/math/math_type_smoke_test.cpp)
compile_all(FOLDER "Tests/math" LIBS branes::math universal SOURCES ${MATH_SRC})
compile_all(FOLDER "Tests/math" LIBS branes::math universal
    TEST_PREFIX "math_type_smoke."
    SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/math/math_type_smoke_test.cpp)

# ── CV layer (issues #37, #38, #48, #49) ───────────────────────────────────
# stb_image provides the PNG encode header cv_image uses for fixtures; linking
# it to the whole group is harmless (header-only) and keeps the glob uniform.
file(GLOB CV_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cv/*_test.cpp)
compile_all(FOLDER "Tests/cv" LIBS branes::cv branes::stb_image SOURCES ${CV_SRC})

# ── SDK layer (issues #34-#36, #40-#47, #187) ──────────────────────────────
# Some SDK tests drive the cv front end; linking branes::cv across the group is
# harmless (header-only) and keeps the glob uniform. vio_euroc carries a prefix
# so its dataset-gated cases group under `ctest -R vio_euroc` (issue #46).
file(GLOB SDK_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sdk/*_test.cpp)
list(REMOVE_ITEM SDK_SRC ${CMAKE_CURRENT_SOURCE_DIR}/sdk/sdk_vio_euroc_test.cpp)
compile_all(FOLDER "Tests/sdk" LIBS branes::sdk branes::cv SOURCES ${SDK_SRC})
compile_all(FOLDER "Tests/sdk" LIBS branes::sdk branes::cv
    TEST_PREFIX "vio_euroc."
    SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/sdk/sdk_vio_euroc_test.cpp)

# ── Bench layer (issue #200) ───────────────────────────────────────────────
file(GLOB BENCH_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/bench/*_test.cpp)
compile_all(FOLDER "Tests/bench" LIBS branes::bench branes::sdk SOURCES ${BENCH_SRC})

# ── Integration: foundational deps + the C++↔Rust cxx::bridge ──────────────
# These two have bespoke link sets (the deps smoke test reaches every third-
# party library; the bridge test links the Corrosion-generated cxx glue plus
# the Rust staticlib), so they are listed explicitly rather than globbed.
compile_all(FOLDER "Tests/integration"
    LIBS MTL5::mtl5 universal branes::stb_image yaml-cpp::yaml-cpp
    SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/integration/deps_smoke_test.cpp)
compile_all(FOLDER "Tests/integration"
    LIBS cortex_core_cxx cortex_core
    SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/integration/cxx_bridge_test.cpp)
