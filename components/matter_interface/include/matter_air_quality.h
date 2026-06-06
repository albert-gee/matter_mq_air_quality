#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MATTER_MQ_DIAGNOSTICS_VENDOR_ID 0xFFF1U
#define MATTER_MQ_DIAGNOSTICS_CLUSTER_SUFFIX 0xFC01U
#define MATTER_MQ_DIAGNOSTICS_CLUSTER_ID \
    ((uint32_t)((MATTER_MQ_DIAGNOSTICS_VENDOR_ID << 16) | MATTER_MQ_DIAGNOSTICS_CLUSTER_SUFFIX))
#define MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT 9U
#define MATTER_MQ_DIAGNOSTICS_ATTRS_PER_SENSOR 13U

typedef enum {
    MATTER_AIR_QUALITY_UNKNOWN = 0,
    MATTER_AIR_QUALITY_GOOD,
    MATTER_AIR_QUALITY_FAIR,
    MATTER_AIR_QUALITY_MODERATE,
    MATTER_AIR_QUALITY_POOR,
    MATTER_AIR_QUALITY_VERY_POOR,
    MATTER_AIR_QUALITY_EXTREMELY_POOR
} matter_air_quality_level_t;

typedef struct {
    uint8_t sensor_type;
    bool enabled;
    uint8_t state;
    int32_t raw_adc;
    int32_t adc_mv;
    int32_t vrl_mv;
    int32_t baseline_vrl_mv;
    int32_t rs_norm_milli;
    int32_t rs_ratio_milli;
    uint8_t threshold_state;
    uint32_t fault_bitmap;
    uint32_t last_update_age_ms;
    bool baseline_valid;
} matter_mq_sensor_diagnostics_t;

typedef struct {
    matter_mq_sensor_diagnostics_t sensors[MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT];
} matter_air_quality_diagnostics_t;

esp_err_t matter_update_air_quality(uint16_t endpoint_id, matter_air_quality_level_t level);
esp_err_t matter_update_air_quality_diagnostics(uint16_t endpoint_id,
                                                const matter_air_quality_diagnostics_t *diagnostics);
esp_err_t matter_air_quality_diagnostics_validate(void);
uint32_t matter_air_quality_diagnostics_cluster_id(void);
size_t matter_air_quality_diagnostics_attribute_count(void);

#ifdef __cplusplus
}
#endif
