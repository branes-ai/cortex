# cv/ — clean-room computer-vision operators for the VIO front end
# (image containers + I/O, pyramid, FAST, KLT). Lives between math/ and
# sdk/. Mostly header-only (templated operators), with a small compiled
# library for the I/O that needs a translation unit (stb_image).

add_library(branes_cv STATIC src/image_io.cpp)
add_library(branes::cv ALIAS branes_cv)

target_include_directories(branes_cv PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(branes_cv PUBLIC cxx_std_20)
target_link_libraries(branes_cv PRIVATE branes::stb_image)
