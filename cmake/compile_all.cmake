# compile_all.cmake — auto-generate one Catch2 test (or example) executable per
# source file in a directory.
#
# Adapted from the compile_all() pattern in MTL5 and Universal, with two cortex
# specializations: targets register every Catch2 TEST_CASE with CTest via
# catch_discover_tests (so `ctest` lists individual cases, not just one entry
# per binary), and an optional TEST_PREFIX namespaces those case names for
# suites whose acceptance commands filter on a prefix (e.g. `ctest -R vio_euroc`).
#
# The point is productivity: drop a `<name>.cpp` into the right tests/<layer>/
# subdirectory and it is built, linked, IDE-grouped, and registered with no edit
# to any CMakeLists. Each target is named after its file stem (which must be
# globally unique) and placed under an IDE FOLDER so Visual Studio's Solution
# Explorer mirrors the layout.
#
#   compile_all(
#     FOLDER       <ide-folder>          # e.g. "Tests/sdk" (Solution Explorer group)
#     LIBS         <lib> [<lib> ...]     # libraries to link beyond Catch2 + test_support
#     SOURCES      <file.cpp> [...]      # typically a file(GLOB ...) result
#     [TEST_PREFIX <prefix>]             # optional ctest name prefix, e.g. "vio_euroc."
#   )
#
# Catch2::Catch2WithMain and branes::test_support are linked into every target,
# and C++20 is required, so callers only list the layer libraries they need.
include_guard(GLOBAL)

include(Catch)

function(compile_all)
    cmake_parse_arguments(CA "" "FOLDER;TEST_PREFIX" "LIBS;SOURCES" ${ARGN})
    if(NOT CA_SOURCES)
        message(FATAL_ERROR "compile_all: SOURCES is required (FOLDER='${CA_FOLDER}')")
    endif()

    foreach(source ${CA_SOURCES})
        get_filename_component(name ${source} NAME_WE)
        add_executable(${name} ${source})
        target_link_libraries(${name} PRIVATE
            Catch2::Catch2WithMain
            ${CA_LIBS}
            branes::test_support
        )
        target_compile_features(${name} PRIVATE cxx_std_20)
        if(CA_FOLDER)
            set_target_properties(${name} PROPERTIES FOLDER "${CA_FOLDER}")
        endif()
        if(CA_TEST_PREFIX)
            catch_discover_tests(${name} TEST_PREFIX "${CA_TEST_PREFIX}")
        else()
            catch_discover_tests(${name})
        endif()
    endforeach()
endfunction()
