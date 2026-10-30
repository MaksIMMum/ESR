cmake_minimum_required(VERSION 3.16.0)

if(ESR_BUILD_DESKTOP OR NOT DEFINED ENV{IDF_PATH})
    project(ESR C)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_C_STANDARD_REQUIRED ON)

    message(STATUS "Configuring ESR for Native Desktop target...")

    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/desktop
    )

    file(GLOB_RECURSE CORE_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.c
    )

    file(GLOB DESKTOP_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/platform/desktop/*.c
    )

    add_executable(esr_desktop ${DESKTOP_SOURCES} ${CORE_SOURCES})
    target_compile_options(esr_desktop PRIVATE -Wall -Wextra -O2)
    target_link_libraries(esr_desktop PRIVATE m)

else()
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(ESR)
endif()
