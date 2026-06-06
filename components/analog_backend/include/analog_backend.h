#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ANALOG_BACKEND_INTERNAL_ADC = 0,
    ANALOG_BACKEND_EXTERNAL_ADC = 1,
    ANALOG_BACKEND_MUX_ADC = 2,
} analog_backend_type_t;

typedef struct {
    uint8_t source_id;
    analog_backend_type_t type;
    uint8_t adc_logical_channel;
    float input_divider_ratio;

    uint8_t mux_id;
    uint8_t mux_channel;
} analog_source_config_t;

esp_err_t analog_backend_init(void);
esp_err_t analog_backend_register_source(const analog_source_config_t *config);
esp_err_t analog_backend_read_mv(uint8_t source_id, int *raw, int *mv);
esp_err_t analog_backend_select_mux_channel(uint8_t mux_id, uint8_t mux_channel);
esp_err_t analog_backend_read_mux_channel_mv(uint8_t mux_id, uint8_t mux_channel, int *raw, int *mv);
esp_err_t analog_backend_get_source_config(uint8_t source_id, analog_source_config_t *out);
bool analog_backend_source_is_registered(uint8_t source_id);

#ifdef __cplusplus
}
#endif
