# The driver is kept as an archive and its port is unfinished: with ESP-IDF 5 and later (no "esp32"
# component, no esp_idf_lib_helpers, ets_delay_us() without its header) it is not built - the
# component stays empty, so the examples that have examples/common in their component dirs build.
if(DEFINED IDF_VERSION_MAJOR AND IDF_VERSION_MAJOR GREATER_EQUAL 5)
    idf_component_register()
    return()
endif()

if(${IDF_TARGET} STREQUAL esp8266)
    set(req esp8266 freertos esp_idf_lib_helpers)
else()
    set(req driver esp32 freertos esp_idf_lib_helpers)
endif()

idf_component_register(
    SRCS "dht.c"
    INCLUDE_DIRS .
    REQUIRES ${req}
)
