#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int adc_unit;
    int adc_channel;
    int attenuation;
    int bitwidth;
} adc_service_channel_config_t;

esp_err_t adc_service_init(void);
esp_err_t adc_service_register_channel(uint8_t logical_channel,
                                       const adc_service_channel_config_t *config);
esp_err_t adc_service_read_raw(uint8_t logical_channel, int *raw);
esp_err_t adc_service_read_mv(uint8_t logical_channel, int *raw, int *mv);

#ifdef __cplusplus
}
#endif
