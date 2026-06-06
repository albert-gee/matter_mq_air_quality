idf_component_register(
    SRCS "src/adc_service.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common esp_adc freertos log
)
