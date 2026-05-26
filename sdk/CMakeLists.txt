# sdk/ — VIO / SLAM / Scene-Graph operators (Surface 1: the "white box").
#
# Starts header-only with the VioBackend interface (#34). As compiled
# operators land (MSCKF backend #35, VioEstimator #45, ...) this gains a
# STATIC library target; for now `branes::sdk` is an INTERFACE target
# carrying the include path, the C++20 requirement, and the math-layer
# dependency. No middleware here (Zenoh/ROS live in daemons/).

add_library(branes_sdk INTERFACE)
add_library(branes::sdk ALIAS branes_sdk)

target_include_directories(branes_sdk INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(branes_sdk INTERFACE cxx_std_20)
# branes::math for the estimator/operator math; branes::cv for the VIO
# front end (FAST + KLT + pyramid) that VioEstimator (#45) owns.
target_link_libraries(branes_sdk INTERFACE branes::math branes::cv)
