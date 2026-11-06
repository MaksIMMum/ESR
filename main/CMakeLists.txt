FILE(GLOB_RECURSE app_sources 
    ${CMAKE_SOURCE_DIR}/main/*.* 
    ${CMAKE_SOURCE_DIR}/platform/esp32/*.*
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/include" "${CMAKE_SOURCE_DIR}/platform/esp32"
    REQUIRES driver esp_hw_support esp_timer
)
