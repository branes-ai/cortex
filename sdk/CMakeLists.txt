
add_library(cortex_sdk SHARED src/vio.cpp src/scene_graph.cpp)
target_link_libraries(cortex_sdk PUBLIC cortex_core_rust branes_math)
