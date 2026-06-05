# tools/ — VIO stage-probe study utilities (NOT tests, NOT daemons).
#
# One executable per pipeline stage S0–S10 (docs/arch/vio-pipeline-canonical.md).
# Each runs one stage in isolation, prints its contract + native-unit
# assessments, and writes CSV artifacts that docs-site/scripts/
# gen-sensor-model-figures.mjs renders to SVG. These are host-only developer
# tools: they link branes::sdk (which carries math + cv) plus the tools harness;
# no middleware. Every tools/src/*.cpp is an executable named after the file.

add_library(branes_tools INTERFACE)
add_library(branes::tools ALIAS branes_tools)
target_include_directories(branes_tools INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(branes_tools INTERFACE cxx_std_20)
target_link_libraries(branes_tools INTERFACE branes::sdk)
# vio_pipeline writes annotated scene PNGs via stb_image_write (header-only). The
# include path is harmless on the other tools (none define the implementation).
target_link_libraries(branes_tools INTERFACE branes::stb_image)

file(GLOB STAGE_PROBE_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
foreach(src ${STAGE_PROBE_SRC})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    target_link_libraries(${name} PRIVATE branes::tools)
    set_target_properties(${name} PROPERTIES FOLDER "Tools/stage-probes")
endforeach()
