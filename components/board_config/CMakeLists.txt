idf_component_register(
    SRCS "src/board_config.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common log adc_service analog_backend analog_mux mq_sensor mq_runtime_config
)
