# Top-level test targets that aren't owned by a single subdirectory.

include(Catch)

add_executable(deps_smoke_test deps_smoke_test.cpp)
target_link_libraries(deps_smoke_test PRIVATE
    Catch2::Catch2WithMain
    MTL5::mtl5
    universal
    branes::stb_image
    yaml-cpp::yaml-cpp
)
target_compile_features(deps_smoke_test PRIVATE cxx_std_20)

catch_discover_tests(deps_smoke_test)
