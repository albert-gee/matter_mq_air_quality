#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "analog_backend.h"
#include "analog_mux.h"
#include "esp_err.h"
#include "mq_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t board_config_register_internal_adc_channels(void);
esp_err_t board_config_init_muxes(void);
esp_err_t board_config_register_analog_sources(void);
esp_err_t board_config_init_mq_sensors(void);

size_t board_config_internal_adc_channel_count(void);
size_t board_config_analog_source_count(void);
size_t board_config_mq_sensor_count(void);
esp_err_t board_config_get_default_sensor_config_by_index(size_t index, mq_sensor_config_t *out);
esp_err_t board_config_get_effective_sensor_config_by_index(size_t index, mq_sensor_config_t *out);
esp_err_t board_config_get_default_source_config_by_index(size_t index, analog_source_config_t *out);
esp_err_t board_config_get_effective_source_config_by_index(size_t index, analog_source_config_t *out);
esp_err_t board_config_get_default_mux_config(uint8_t mux_id, analog_mux_config_t *out);
esp_err_t board_config_get_effective_mux_config(uint8_t mux_id, analog_mux_config_t *out);
bool board_config_mux_is_enabled(uint8_t mux_id);
bool board_config_sensor_can_be_enabled(uint8_t sensor_id);

#ifdef __cplusplus
}
#endif
