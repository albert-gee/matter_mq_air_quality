idf_component_register(
    SRCS "src/analog_mux.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common log driver freertos
)
