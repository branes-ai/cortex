# bench/ — VIO "intelligence per Watt" benchmark (epic #199).
#
# branes::bench is header-only (RAPL reader, operator FLOP/byte model,
# report serialization); vio_bench is the runnable tool that drives the
# EuRoC replay and emits the accuracy/latency/energy report.

add_library(branes_bench INTERFACE)
add_library(branes::bench ALIAS branes_bench)
target_include_directories(branes_bench INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(branes_bench INTERFACE cxx_std_20)

add_executable(vio_bench src/vio_bench.cpp)
target_link_libraries(vio_bench PRIVATE branes::bench branes::sdk branes::cv)
target_compile_features(vio_bench PRIVATE cxx_std_20)
