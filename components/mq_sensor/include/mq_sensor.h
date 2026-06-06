#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "mq_calibration_nvs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MQ_SENSOR_MQ2 = 0,
    MQ_SENSOR_MQ3,
    MQ_SENSOR_MQ4,
    MQ_SENSOR_MQ5,
    MQ_SENSOR_MQ6,
    MQ_SENSOR_MQ7,
    MQ_SENSOR_MQ8,
    MQ_SENSOR_MQ9,
    MQ_SENSOR_MQ135,
    MQ_SENSOR_TYPE_COUNT
} mq_sensor_type_t;

typedef struct {
    uint8_t id;
    mq_sensor_type_t type;
    const char *name;
    uint8_t analog_source_id;
    uint32_t vc_mv;
    uint32_t warmup_seconds;
    bool enabled;
    bool supports_baseline_calibration;
    float warning_rs_ratio;
    float critical_rs_ratio;
} mq_sensor_config_t;

typedef enum {
    MQ_SENSOR_STATE_DISABLED = 0,
    MQ_SENSOR_STATE_WARMING,
    MQ_SENSOR_STATE_UNCALIBRATED,
    MQ_SENSOR_STATE_READY,
    MQ_SENSOR_STATE_STALE,
    MQ_SENSOR_STATE_FAULT,
    MQ_SENSOR_STATE_DIAGNOSTIC_ONLY
} mq_sensor_state_t;

typedef enum {
    MQ_SENSOR_THRESHOLD_NONE = 0,
    MQ_SENSOR_THRESHOLD_NORMAL,
    MQ_SENSOR_THRESHOLD_WARNING,
    MQ_SENSOR_THRESHOLD_CRITICAL,
} mq_sensor_threshold_state_t;

typedef enum {
    MQ_SENSOR_FAULT_NONE = 0,
    MQ_SENSOR_FAULT_SOURCE_MISSING = 1U << 0,
    MQ_SENSOR_FAULT_DIVIDER_UNCONFIGURED = 1U << 1,
    MQ_SENSOR_FAULT_WARMUP = 1U << 2,
    MQ_SENSOR_FAULT_ADC_READ = 1U << 3,
    MQ_SENSOR_FAULT_INVALID_VRL = 1U << 4,
    MQ_SENSOR_FAULT_CALIBRATION_MISMATCH = 1U << 5,
} mq_sensor_fault_bits_t;

typedef struct {
    uint8_t id;
    mq_sensor_state_t state;
    int raw_adc;
    int adc_mv;
    int vrl_mv;
    int baseline_vrl_mv;
    float rs_norm;
    float baseline_rs_norm;
    float rs_ratio;
    mq_sensor_threshold_state_t threshold_state;
    uint32_t fault_bitmap;
} mq_sensor_sample_t;

esp_err_t mq_sensor_init(const mq_sensor_config_t *configs, size_t count);
size_t mq_sensor_count(void);
esp_err_t mq_sensor_read(uint8_t id, mq_sensor_sample_t *out);
esp_err_t mq_sensor_read_all(void);
esp_err_t mq_sensor_get_last(uint8_t id, mq_sensor_sample_t *out);
const char *mq_sensor_type_to_string(mq_sensor_type_t type);
const char *mq_sensor_state_to_string(mq_sensor_state_t state);
esp_err_t mq_sensor_get_config_by_index(size_t index, mq_sensor_config_t *out);
esp_err_t mq_sensor_calibrate_baseline(uint8_t id,
                                       size_t sample_count,
                                       uint32_t sample_delay_ms,
                                       mq_calibration_record_t *saved_record);
esp_err_t mq_sensor_reload_calibration(uint8_t id);
esp_err_t mq_sensor_erase_calibration(uint8_t id);
esp_err_t mq_sensor_compute_rs_norm(uint32_t vc_mv, int vrl_mv, float *rs_norm);
esp_err_t mq_sensor_compute_rs_ratio(uint32_t vc_mv, int vrl_mv, int baseline_vrl_mv, float *rs_ratio);
esp_err_t mq_sensor_apply_divider_mv(int adc_mv, float divider_ratio, int *corrected_mv);
esp_err_t mq_sensor_self_check(void);

#ifdef __cplusplus
}
#endif
