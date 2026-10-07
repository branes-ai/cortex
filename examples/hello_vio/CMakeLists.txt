# hello_vio — white-box SDK consumer (issue #103). See README.md.
#
# yaml-cpp is an *application* dependency here: the example parses the EuRoC
# sensor.yaml files into SDK structs itself. The SDK never sees YAML.

add_executable(hello_vio main.cpp)
target_link_libraries(hello_vio PRIVATE branes::sdk branes::cv yaml-cpp::yaml-cpp)
target_compile_features(hello_vio PRIVATE cxx_std_20)
set_target_properties(hello_vio PROPERTIES FOLDER "Examples")

# Smoke test: run against a EuRoC-format fixture (the real cam0/imu0
# sensor.yaml, no frames). It checks the YAML -> SDK-struct mapping and the
# estimator lifecycle on every CI run; replaying real frames needs a
# downloaded sequence (see README.md).
if(BUILD_TESTING)
    add_test(NAME hello_vio.smoke
             COMMAND hello_vio ${CMAKE_CURRENT_SOURCE_DIR}/testdata/mav0)
    set_tests_properties(hello_vio.smoke PROPERTIES
        PASS_REGULAR_EXPRESSION "replayed 0 frames"
        FAIL_REGULAR_EXPRESSION "hello_vio: ")
endif()
