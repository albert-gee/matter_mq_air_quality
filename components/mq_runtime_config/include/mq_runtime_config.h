#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "analog_backend.h"
#include "analog_mux.h"
#include "air_quality_service.h"
#include "esp_err.h"
#include "mq_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mq_runtime_config_init(void);

esp_err_t mq_runtime_config_apply_sensor(uint8_t sensor_id,
                                         const mq_sensor_config_t *defaults,
                                         mq_sensor_config_t *effective);

esp_err_t mq_runtime_config_apply_source(uint8_t source_id,
                                         const analog_source_config_t *defaults,
                                         analog_source_config_t *effective);

esp_err_t mq_runtime_config_apply_mux(uint8_t mux_id,
                                      const analog_mux_config_t *defaults,
                                      analog_mux_config_t *effective);

esp_err_t mq_runtime_config_apply_air_quality(const air_quality_service_config_t *defaults,
                                              air_quality_service_config_t *effective);

esp_err_t mq_runtime_config_save_sensor(const mq_sensor_config_t *config);
esp_err_t mq_runtime_config_save_source(const analog_source_config_t *config);
esp_err_t mq_runtime_config_save_mux(const analog_mux_config_t *config);
esp_err_t mq_runtime_config_save_air_quality(const air_quality_service_config_t *config);

esp_err_t mq_runtime_config_erase_sensor(uint8_t sensor_id);
esp_err_t mq_runtime_config_erase_source(uint8_t source_id);
esp_err_t mq_runtime_config_erase_mux(uint8_t mux_id);
esp_err_t mq_runtime_config_erase_air_quality(void);
esp_err_t mq_runtime_config_erase_all(void);

#ifdef __cplusplus
}
#endif
