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
    uint32_t rl_ohms;
    uint32_t warmup_seconds;
    bool enabled;
    float clean_air_rs_r0_factor;
    bool supports_clean_air_calibration;
} mq_sensor_config_t;

typedef enum {
    MQ_SENSOR_STATE_DISABLED = 0,
    MQ_SENSOR_STATE_WARMING,
    MQ_SENSOR_STATE_UNCALIBRATED,
    MQ_SENSOR_STATE_READY,
    MQ_SENSOR_STATE_STALE,
    MQ_SENSOR_STATE_ERROR
} mq_sensor_state_t;

typedef struct {
    uint8_t id;
    mq_sensor_state_t state;
    int raw_adc;
    int measured_mv;
    int corrected_vrl_mv;
    float rs_ohms;
    float r0_ohms;
    float rs_r0_ratio;
} mq_sensor_sample_t;

esp_err_t mq_sensor_init(const mq_sensor_config_t *configs, size_t count);
size_t mq_sensor_count(void);
esp_err_t mq_sensor_read(uint8_t id, mq_sensor_sample_t *out);
esp_err_t mq_sensor_read_all(void);
esp_err_t mq_sensor_get_last(uint8_t id, mq_sensor_sample_t *out);
const char *mq_sensor_type_to_string(mq_sensor_type_t type);
const char *mq_sensor_state_to_string(mq_sensor_state_t state);
esp_err_t mq_sensor_get_config_by_index(size_t index, mq_sensor_config_t *out);
esp_err_t mq_sensor_calibrate_clean_air(uint8_t id,
                                        size_t sample_count,
                                        uint32_t sample_delay_ms,
                                        mq_calibration_record_t *saved_record);
esp_err_t mq_sensor_reload_calibration(uint8_t id);
esp_err_t mq_sensor_erase_calibration(uint8_t id);

#ifdef __cplusplus
}
#endif
