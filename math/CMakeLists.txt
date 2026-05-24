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
