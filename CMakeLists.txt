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

    add_executable(esr_desktop
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/desktop/main.cpp
        ${CORE_SOURCES}
    )
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

    add_executable(test_esr_culling
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_esr_culling.cpp
        ${CORE_SOURCES}
    )
    target_compile_options(test_esr_culling PRIVATE -Wall -Wextra -O2)
    target_link_libraries(test_esr_culling PRIVATE m)
    add_test(NAME test_esr_culling COMMAND test_esr_culling)

    add_executable(test_esr_shader
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_esr_shader.cpp
        ${CORE_SOURCES}
    )
    target_compile_options(test_esr_shader PRIVATE -Wall -Wextra -O2)
    target_link_libraries(test_esr_shader PRIVATE m)
    add_test(NAME test_esr_shader COMMAND test_esr_shader)

    add_executable(test_esr_profiler
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_esr_profiler.cpp
        ${CORE_SOURCES}
    )
    target_compile_options(test_esr_profiler PRIVATE -Wall -Wextra -O2)
    target_link_libraries(test_esr_profiler PRIVATE m)
    add_test(NAME test_esr_profiler COMMAND test_esr_profiler)

    add_executable(test_esr_interp
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_esr_interp.cpp
        ${CORE_SOURCES}
    )
    target_compile_options(test_esr_interp PRIVATE -Wall -Wextra -O2)
    target_link_libraries(test_esr_interp PRIVATE m)
    add_test(NAME test_esr_interp COMMAND test_esr_interp)

    # -------------------------------------------------------------
    # Benchmarks (not part of ctest: timings are machine dependent)
    # -------------------------------------------------------------
    add_executable(bench_esr_raster
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/03_pipeline_benchmark/bench_raster.cpp
        ${CORE_SOURCES}
    )
    target_compile_options(bench_esr_raster PRIVATE -Wall -Wextra -O2)
    target_link_libraries(bench_esr_raster PRIVATE m)

    # -------------------------------------------------------------
    # Interactive Desktop 3D Live Viewer with Real-Time HUD
    # -------------------------------------------------------------
    find_package(SDL2 QUIET)
    if(SDL2_FOUND)
        message(STATUS "Found SDL2 - Building interactive viewer target 'esr_viewer'...")
        add_executable(esr_viewer
            ${CMAKE_CURRENT_SOURCE_DIR}/platform/desktop/viewer.cpp
            ${CORE_SOURCES}
        )
        target_include_directories(esr_viewer PRIVATE ${SDL2_INCLUDE_DIRS})
        target_compile_options(esr_viewer PRIVATE -Wall -Wextra -O2)
        target_link_libraries(esr_viewer PRIVATE ${SDL2_LIBRARIES} m)
    endif()

else()
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(ESR)
endif()
