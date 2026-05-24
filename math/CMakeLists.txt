# math/ — header-only C++20 math layer (NLS + linear algebra).
#
# Everything here is headers only (enforced: no .cpp files in math/).
# The layer is exposed as an INTERFACE library that carries the include
# path and the C++20 requirement. Downstream targets (sdk/, tests/)
# link `branes::math` to pull in <branes/math/...> headers.

add_library(branes_math INTERFACE)
add_library(branes::math ALIAS branes_math)

target_include_directories(branes_math INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(branes_math INTERFACE cxx_std_20)

# MTL5 + Universal are part of the math layer's public surface: callers
# instantiate solvers against Universal arithmetic types and MTL5 views.
target_link_libraries(branes_math INTERFACE MTL5::mtl5 universal)

# Header-only guard: fail configuration if a .cpp ever lands in math/.
# Math-layer tests live under the top-level tests/ tree precisely so
# this directory stays free of translation units.
file(GLOB_RECURSE _math_cpp_sources CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
if(_math_cpp_sources)
    message(FATAL_ERROR
        "math/ must stay header-only, but found .cpp files: ${_math_cpp_sources}")
endif()
unset(_math_cpp_sources)

# ── API documentation (issue #32) ──────────────────────────────────
# `docs-math` runs Doxygen over the math/ headers + the concepts page,
# emitting HTML under build/docs/math. WARN_AS_ERROR keeps the headers
# warning-clean. The target is only defined when Doxygen is present and
# is never part of the default (ALL) build, so it's a no-op for ordinary
# builds and CI that lack Doxygen.
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/docs/math)
    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_GENERATE_LATEX NO)
    set(DOXYGEN_QUIET YES)
    set(DOXYGEN_WARN_AS_ERROR FAIL_ON_WARNINGS)
    # Treat malformed comments / broken references as errors, but don't
    # demand a doc block on every trivial accessor — the public types and
    # their notable members are documented; getters are self-evident.
    set(DOXYGEN_WARN_IF_UNDOCUMENTED NO)
    set(DOXYGEN_RECURSIVE YES)
    set(DOXYGEN_MARKDOWN_SUPPORT YES)
    set(DOXYGEN_EXTRACT_ALL NO)
    # Internal helpers live in detail namespaces; they are not public API
    # and are excluded from the docs (and from undocumented-warnings).
    set(DOXYGEN_EXCLUDE_SYMBOLS "detail" "detail::*")
    set(DOXYGEN_PROJECT_NAME "branes::math")
    doxygen_add_docs(docs-math
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/docs/math/concepts.md
        COMMENT "Generating math-layer API docs with Doxygen")
endif()
