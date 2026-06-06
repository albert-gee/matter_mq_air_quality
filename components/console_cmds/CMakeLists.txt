idf_component_register(
    SRCS "src/mq_console.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common log console adc_service analog_backend analog_mux board_config mq_sensor mq_calibration_nvs mq_runtime_config air_quality_service matter_interface
)
