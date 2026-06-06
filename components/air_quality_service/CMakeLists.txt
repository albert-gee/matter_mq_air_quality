idf_component_register(
    SRCS "src/air_quality_service.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common log freertos mq_sensor matter_interface
)
