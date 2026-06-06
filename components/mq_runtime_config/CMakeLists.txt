idf_component_register(
    SRCS "src/mq_runtime_config.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common log nvs_flash driver mq_sensor analog_backend analog_mux air_quality_service
)
