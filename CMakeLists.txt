cmake_minimum_required(VERSION 3.16.0)

if(ESR_BUILD_DESKTOP OR NOT DEFINED ENV{IDF_PATH})
    project(ESR C CXX)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    message(STATUS "Configuring ESR for Native Desktop target (C11 / C++17)...")

    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/desktop
    )

    file(GLOB_RECURSE CORE_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.cpp
    )

    file(GLOB DESKTOP_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/desktop/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/desktop/*.cpp
    )

    add_executable(esr_desktop ${DESKTOP_SOURCES} ${CORE_SOURCES})
    target_compile_options(esr_desktop PRIVATE -Wall -Wextra -O2)
    target_link_libraries(esr_desktop PRIVATE m)

    # -------------------------------------------------------------
    # Automated Tests
    # -------------------------------------------------------------
    enable_testing()

    add_executable(test_esr_math
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_esr_math.cpp
    )
    target_compile_options(test_esr_math PRIVATE -Wall -Wextra -O2)
    target_link_libraries(test_esr_math PRIVATE m)
    add_test(NAME test_esr_math COMMAND test_esr_math)

    add_executable(test_esr_pipeline
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_esr_pipeline.cpp
        ${CORE_SOURCES}
    )
    target_compile_options(test_esr_pipeline PRIVATE -Wall -Wextra -O2)
    target_link_libraries(test_esr_pipeline PRIVATE m)
    add_test(NAME test_esr_pipeline COMMAND test_esr_pipeline)

    add_executable(test_esr_raster
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_esr_raster.cpp
        ${CORE_SOURCES}
    )
    target_compile_options(test_esr_raster PRIVATE -Wall -Wextra -O2)
    target_link_libraries(test_esr_raster PRIVATE m)
    add_test(NAME test_esr_raster COMMAND test_esr_raster)

    add_executable(test_esr_mesh
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_esr_mesh.cpp
        ${CORE_SOURCES}
    )
    target_compile_options(test_esr_mesh PRIVATE -Wall -Wextra -O2)
    target_link_libraries(test_esr_mesh PRIVATE m)
    add_test(NAME test_esr_mesh COMMAND test_esr_mesh)

    add_executable(test_esr_depth
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_esr_depth.cpp
        ${CORE_SOURCES}
    )
    target_compile_options(test_esr_depth PRIVATE -Wall -Wextra -O2)
    target_link_libraries(test_esr_depth PRIVATE m)
    add_test(NAME test_esr_depth COMMAND test_esr_depth)

    # -------------------------------------------------------------
    # Benchmarks (not part of ctest: timings are machine dependent)
    # -------------------------------------------------------------
    add_executable(bench_esr_raster
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/03_pipeline_benchmark/bench_raster.cpp
        ${CORE_SOURCES}
    )
    target_compile_options(bench_esr_raster PRIVATE -Wall -Wextra -O2)
    target_link_libraries(bench_esr_raster PRIVATE m)

else()
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(ESR)
endif()
