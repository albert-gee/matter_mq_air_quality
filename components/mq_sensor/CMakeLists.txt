idf_component_register(
    SRCS "src/mq_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common log analog_backend mq_calibration_nvs freertos
)
