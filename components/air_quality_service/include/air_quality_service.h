#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "matter_air_quality.h"
#include "mq_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t matter_endpoint_id;
    uint8_t primary_sensor_id;
    uint32_t sample_interval_ms;
    uint32_t stale_after_ms;
    float ema_alpha;
    bool publish_to_matter;
} air_quality_service_config_t;

typedef struct {
    bool initialized;
    bool running;
    uint16_t matter_endpoint_id;
    uint8_t primary_sensor_id;
    matter_air_quality_level_t current_level;
    matter_air_quality_level_t last_published_level;
    mq_sensor_sample_t last_sample;
    float filtered_ratio;
    uint32_t last_success_ms;
    uint32_t last_sample_age_ms;
    uint32_t successful_reads;
    uint32_t failed_reads;
    uint32_t matter_updates;
    esp_err_t last_error;
} air_quality_service_status_t;

esp_err_t air_quality_service_init(const air_quality_service_config_t *config);
esp_err_t air_quality_service_start(void);
esp_err_t air_quality_service_stop(void);
esp_err_t air_quality_service_get_status(air_quality_service_status_t *out);
const char *air_quality_service_level_to_string(matter_air_quality_level_t level);

#ifdef __cplusplus
}
#endif
