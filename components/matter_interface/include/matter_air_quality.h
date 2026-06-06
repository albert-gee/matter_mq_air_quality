#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

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
    uint8_t sensor_id;
    uint8_t sensor_type;
    uint8_t enabled;
    uint8_t state;
    int32_t raw_adc;
    int32_t adc_mv;
    int32_t vrl_mv;
    int32_t baseline_vrl_mv;
    int32_t rs_norm_milli;
    int32_t rs_ratio_milli;
    uint8_t baseline_valid;
    uint8_t threshold_state;
    uint32_t fault_bitmap;
    uint32_t last_update_age_ms;
} matter_air_quality_diagnostics_t;

esp_err_t matter_update_air_quality(uint16_t endpoint_id, matter_air_quality_level_t level);
esp_err_t matter_update_air_quality_diagnostics(uint16_t endpoint_id,
                                                const matter_air_quality_diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif
