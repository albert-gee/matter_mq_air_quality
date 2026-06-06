#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ANALOG_MUX_GPIO_UNUSED (-1)

typedef struct {
    uint8_t mux_id;
    bool enabled;
    uint8_t signal_adc_logical_channel;
    int gpio_s0;
    int gpio_s1;
    int gpio_s2;
    int gpio_s3;
    int gpio_en;
    bool en_active_low;
    uint32_t settle_time_us;
} analog_mux_config_t;

esp_err_t analog_mux_init(void);
esp_err_t analog_mux_register(const analog_mux_config_t *config);
esp_err_t analog_mux_get_config(uint8_t mux_id, analog_mux_config_t *out);
esp_err_t analog_mux_is_ready(uint8_t mux_id, bool *ready);
esp_err_t analog_mux_select(uint8_t mux_id, uint8_t channel);

#ifdef __cplusplus
}
#endif
