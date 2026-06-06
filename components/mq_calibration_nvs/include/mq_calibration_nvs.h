#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MQ_CALIBRATION_RECORD_MAGIC 0x4d514231U
#define MQ_CALIBRATION_RECORD_VERSION 4U
#define MQ_CALIBRATION_FIRMWARE_CONFIG_VERSION 4U

typedef struct {
    bool valid;
    uint32_t baseline_vrl_mv;
    uint32_t baseline_at_unix;
    uint32_t magic;
    uint16_t version;
    uint16_t firmware_config_version;
    uint8_t sensor_type;
    uint8_t source_id;
    uint8_t mux_id;
    uint8_t mux_channel;
    uint16_t sample_count;
    uint32_t vc_mv;
    float input_divider_ratio;
    float baseline_rs_norm;
    float baseline_vrl_mean_mv;
    float baseline_vrl_stddev_mv;
} mq_calibration_record_t;

esp_err_t mq_calibration_nvs_init(void);
esp_err_t mq_calibration_nvs_load(uint8_t sensor_id, mq_calibration_record_t *out);
esp_err_t mq_calibration_nvs_save(uint8_t sensor_id, const mq_calibration_record_t *record);
esp_err_t mq_calibration_nvs_erase(uint8_t sensor_id);
esp_err_t mq_calibration_nvs_erase_all(void);

#ifdef __cplusplus
}
#endif
