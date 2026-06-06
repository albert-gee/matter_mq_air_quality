#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    float r0_ohms;
    uint32_t calibrated_at_unix;
} mq_calibration_record_t;

esp_err_t mq_calibration_nvs_init(void);
esp_err_t mq_calibration_nvs_load(uint8_t sensor_id, mq_calibration_record_t *out);
esp_err_t mq_calibration_nvs_save(uint8_t sensor_id, const mq_calibration_record_t *record);
esp_err_t mq_calibration_nvs_erase(uint8_t sensor_id);
esp_err_t mq_calibration_nvs_erase_all(void);

#ifdef __cplusplus
}
#endif
