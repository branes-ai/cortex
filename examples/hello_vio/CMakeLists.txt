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

    # Replay test: a tiny synthetic sequence (5 frames, static IMU; see
    # testdata/README.md) that runs main.cpp's own loop end to end: CSV
    # parsing, IMU/frame ordering, PNG loading, feed_imu/feed_image,
    # current_pose. It checks the loop, not accuracy (the estimator is still
    # initializing after 5 frames); accuracy lives in tests/sdk/vio_euroc.cpp.
    add_test(NAME hello_vio.frames
             COMMAND hello_vio ${CMAKE_CURRENT_SOURCE_DIR}/testdata/frames/mav0)
    set_tests_properties(hello_vio.frames PROPERTIES
        PASS_REGULAR_EXPRESSION "replayed 5 frames"
        FAIL_REGULAR_EXPRESSION "hello_vio: |nan|inf")
endif()
