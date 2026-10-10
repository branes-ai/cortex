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
# The inter-stage trace bus (vio_trace.hpp, #372) serializes per-stage I/O to
# JSONL and reads it back; nlohmann/json supplies the parser the inspectors need.
target_link_libraries(branes_tools INTERFACE nlohmann_json::nlohmann_json)
# The composition bench instantiates every stage bench in one translation unit;
# MSVC's default COFF section limit is too small for that (C1128).
target_compile_options(branes_tools INTERFACE $<$<CXX_COMPILER_ID:MSVC>:/bigobj>)

# Stage test benches (#453): one small executable per bench, tools/benches/*.cpp,
# each including only its stage's headers. Fixture files live next to them; the
# path is exported so the Catch2 bench tests can load the committed fixtures.
target_compile_definitions(branes_tools INTERFACE
    BRANES_BENCH_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/benches/fixtures")
file(GLOB STAGE_BENCH_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/benches/*.cpp)
foreach(src ${STAGE_BENCH_SRC})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    target_link_libraries(${name} PRIVATE branes::tools)
    set_target_properties(${name} PROPERTIES FOLDER "Tools/stage-benches")
endforeach()

file(GLOB STAGE_PROBE_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
foreach(src ${STAGE_PROBE_SRC})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    target_link_libraries(${name} PRIVATE branes::tools)
    set_target_properties(${name} PROPERTIES FOLDER "Tools/stage-probes")
endforeach()
