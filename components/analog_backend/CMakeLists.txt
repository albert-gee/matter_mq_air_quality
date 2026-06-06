idf_component_register(
    SRCS "src/analog_backend.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common log adc_service analog_mux freertos
)
