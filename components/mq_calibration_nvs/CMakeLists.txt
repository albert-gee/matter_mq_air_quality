idf_component_register(
    SRCS "src/mq_calibration_nvs.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common log nvs_flash
)
