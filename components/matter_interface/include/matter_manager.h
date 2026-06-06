#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "matter_air_quality.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t matter_init(uint16_t *air_quality_endpoint_id);

#ifdef __cplusplus
}
#endif
