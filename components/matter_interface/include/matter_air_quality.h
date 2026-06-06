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

esp_err_t matter_update_air_quality(uint16_t endpoint_id, matter_air_quality_level_t level);

#ifdef __cplusplus
}
#endif
