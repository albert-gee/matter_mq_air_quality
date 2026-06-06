#include "mq_sensor.h"

#include <stdbool.h>
#include <string.h>
#include <time.h>

#include "analog_backend.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mq_calibration_nvs.h"

#define MQ_SENSOR_MAX_SENSORS 9
#define MQ_SENSOR_DEFAULT_VC_MV 5000U
#define MQ_SENSOR_DEFAULT_RL_OHMS 10000U
#define MQ_SENSOR_LOCK_TIMEOUT pdMS_TO_TICKS(5000)

static const char *TAG = "mq_sensor";

static const mq_sensor_config_t s_default_configs[MQ_SENSOR_MAX_SENSORS] = {
    {0, MQ_SENSOR_MQ2, "MQ-2", 0, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_DEFAULT_RL_OHMS, 0, false, 1.0f, true},
    {1, MQ_SENSOR_MQ3, "MQ-3", 1, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_DEFAULT_RL_OHMS, 0, false, 0.0f, false},
    {2, MQ_SENSOR_MQ4, "MQ-4", 2, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_DEFAULT_RL_OHMS, 0, false, 1.0f, true},
    {3, MQ_SENSOR_MQ5, "MQ-5", 3, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_DEFAULT_RL_OHMS, 0, false, 0.0f, false},
    {4, MQ_SENSOR_MQ6, "MQ-6", 4, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_DEFAULT_RL_OHMS, 0, false, 1.0f, true},
    {5, MQ_SENSOR_MQ7, "MQ-7", 5, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_DEFAULT_RL_OHMS, 0, false, 0.0f, false},
    {6, MQ_SENSOR_MQ8, "MQ-8", 6, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_DEFAULT_RL_OHMS, 0, false, 1.0f, true},
    {7, MQ_SENSOR_MQ9, "MQ-9", 7, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_DEFAULT_RL_OHMS, 0, false, 0.0f, false},
    {8, MQ_SENSOR_MQ135, "MQ-135", 8, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_DEFAULT_RL_OHMS, 0, false, 1.0f, true},
};

static bool s_initialized;
static mq_sensor_config_t s_configs[MQ_SENSOR_MAX_SENSORS];
static mq_sensor_sample_t s_last_samples[MQ_SENSOR_MAX_SENSORS];
static mq_calibration_record_t s_calibrations[MQ_SENSOR_MAX_SENSORS];
static TickType_t s_start_ticks[MQ_SENSOR_MAX_SENSORS];
static SemaphoreHandle_t s_lock;
static size_t s_sensor_count;

static esp_err_t ensure_lock_created(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateRecursiveMutex();
        ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create MQ sensor lock");
    }
    return ESP_OK;
}

static esp_err_t mq_sensor_lock(void)
{
    ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_INVALID_STATE, TAG, "MQ sensor lock is not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTakeRecursive(s_lock, MQ_SENSOR_LOCK_TIMEOUT) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "timed out waiting for MQ sensor lock");
    return ESP_OK;
}

static void mq_sensor_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGiveRecursive(s_lock);
    }
}

static int find_sensor_index(uint8_t id)
{
    for (size_t i = 0; i < s_sensor_count; ++i) {
        if (s_configs[i].id == id) {
            return (int)i;
        }
    }
    return -1;
}

static void clear_calibration(size_t index)
{
    memset(&s_calibrations[index], 0, sizeof(s_calibrations[index]));
    s_last_samples[index].r0_ohms = 0.0f;
    s_last_samples[index].rs_r0_ratio = 0.0f;
    if (s_last_samples[index].state == MQ_SENSOR_STATE_READY) {
        s_last_samples[index].state = MQ_SENSOR_STATE_UNCALIBRATED;
    }
}

static void reset_sample(size_t index)
{
    s_last_samples[index].id = s_configs[index].id;
    s_last_samples[index].state = s_configs[index].enabled
                                      ? (s_configs[index].warmup_seconds > 0 ? MQ_SENSOR_STATE_WARMING
                                                                             : MQ_SENSOR_STATE_UNCALIBRATED)
                                      : MQ_SENSOR_STATE_DISABLED;
    s_last_samples[index].raw_adc = -1;
    s_last_samples[index].measured_mv = -1;
    s_last_samples[index].corrected_vrl_mv = -1;
    s_last_samples[index].rs_ohms = 0.0f;
    s_last_samples[index].r0_ohms = 0.0f;
    s_last_samples[index].rs_r0_ratio = 0.0f;
}

static bool is_still_warming(size_t index)
{
    if (!s_configs[index].enabled || s_configs[index].warmup_seconds == 0) {
        return false;
    }

    const TickType_t elapsed_ticks = xTaskGetTickCount() - s_start_ticks[index];
    const uint64_t elapsed_ms = (uint64_t)elapsed_ticks * (uint64_t)portTICK_PERIOD_MS;
    const uint64_t warmup_ms = (uint64_t)s_configs[index].warmup_seconds * 1000ULL;
    return elapsed_ms < warmup_ms;
}

static esp_err_t validate_config(const mq_sensor_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "sensor config is required");
    ESP_RETURN_ON_FALSE(config->type < MQ_SENSOR_TYPE_COUNT, ESP_ERR_INVALID_ARG, TAG,
                        "invalid sensor type %d", (int)config->type);
    ESP_RETURN_ON_FALSE(config->name != NULL, ESP_ERR_INVALID_ARG, TAG, "sensor name is required");
    ESP_RETURN_ON_FALSE(config->vc_mv > 0, ESP_ERR_INVALID_ARG, TAG, "sensor vc_mv must be positive");
    ESP_RETURN_ON_FALSE(config->rl_ohms > 0, ESP_ERR_INVALID_ARG, TAG, "sensor rl_ohms must be positive");
    ESP_RETURN_ON_FALSE(config->clean_air_rs_r0_factor >= 0.0f, ESP_ERR_INVALID_ARG, TAG,
                        "clean-air factor must not be negative");
    ESP_RETURN_ON_FALSE(!config->supports_clean_air_calibration || config->clean_air_rs_r0_factor > 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "clean-air capable sensors need a positive factor");
    return ESP_OK;
}

esp_err_t mq_sensor_init(const mq_sensor_config_t *configs, size_t count)
{
    ESP_RETURN_ON_ERROR(ensure_lock_created(), TAG, "failed to initialize MQ sensor lock");

    const mq_sensor_config_t *source = configs;
    size_t source_count = count;

    if (source == NULL || source_count == 0) {
        source = s_default_configs;
        source_count = MQ_SENSOR_MAX_SENSORS;
    }

    ESP_RETURN_ON_FALSE(source_count <= MQ_SENSOR_MAX_SENSORS, ESP_ERR_INVALID_ARG, TAG,
                        "too many MQ sensor configs: %u", (unsigned)source_count);

    for (size_t i = 0; i < source_count; ++i) {
        ESP_RETURN_ON_ERROR(validate_config(&source[i]), TAG, "invalid config at index %u", (unsigned)i);
        for (size_t j = 0; j < i; ++j) {
            ESP_RETURN_ON_FALSE(source[i].id != source[j].id, ESP_ERR_INVALID_ARG, TAG,
                                "duplicate sensor id %u", (unsigned)source[i].id);
        }
    }

    ESP_RETURN_ON_ERROR(mq_sensor_lock(), TAG, "failed to lock MQ sensor manager");

    memset(s_configs, 0, sizeof(s_configs));
    memset(s_last_samples, 0, sizeof(s_last_samples));
    memset(s_calibrations, 0, sizeof(s_calibrations));
    memset(s_start_ticks, 0, sizeof(s_start_ticks));
    memcpy(s_configs, source, source_count * sizeof(mq_sensor_config_t));
    s_sensor_count = source_count;
    s_initialized = true;

    for (size_t i = 0; i < s_sensor_count; ++i) {
        if (s_configs[i].enabled) {
            s_start_ticks[i] = xTaskGetTickCount();
        }
        reset_sample(i);
        esp_err_t err = mq_calibration_nvs_load(s_configs[i].id, &s_calibrations[i]);
        if (err == ESP_ERR_NOT_FOUND) {
            clear_calibration(i);
        } else if (err == ESP_OK) {
            if (s_calibrations[i].valid && s_calibrations[i].r0_ohms > 0.0f) {
                s_last_samples[i].r0_ohms = s_calibrations[i].r0_ohms;
            } else {
                clear_calibration(i);
            }
        } else {
            ESP_LOGW(TAG, "failed to load calibration for sensor %u: %s",
                     (unsigned)s_configs[i].id, esp_err_to_name(err));
            clear_calibration(i);
        }
    }

    ESP_LOGI(TAG, "MQ sensor manager initialized with %u configured sensors", (unsigned)s_sensor_count);
    mq_sensor_unlock();
    return ESP_OK;
}

size_t mq_sensor_count(void)
{
    if (s_lock == NULL) {
        return 0;
    }
    if (xSemaphoreTakeRecursive(s_lock, MQ_SENSOR_LOCK_TIMEOUT) != pdTRUE) {
        return 0;
    }
    const size_t count = s_initialized ? s_sensor_count : 0;
    xSemaphoreGiveRecursive(s_lock);
    return count;
}

esp_err_t mq_sensor_read(uint8_t id, mq_sensor_sample_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "sample output is required");

    ESP_RETURN_ON_ERROR(mq_sensor_lock(), TAG, "failed to lock MQ sensor manager");
    if (!s_initialized) {
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    const int index = find_sensor_index(id);
    if (index < 0) {
        mq_sensor_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    mq_sensor_sample_t sample = s_last_samples[index];
    if (!s_configs[index].enabled) {
        sample.state = MQ_SENSOR_STATE_DISABLED;
        s_last_samples[index] = sample;
        *out = sample;
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    if (is_still_warming((size_t)index)) {
        sample.state = MQ_SENSOR_STATE_WARMING;
        sample.raw_adc = -1;
        sample.measured_mv = -1;
        sample.corrected_vrl_mv = -1;
        sample.rs_ohms = 0.0f;
        sample.r0_ohms = s_calibrations[index].valid ? s_calibrations[index].r0_ohms : 0.0f;
        sample.rs_r0_ratio = 0.0f;
        s_last_samples[index] = sample;
        *out = sample;
        mq_sensor_unlock();
        return ESP_OK;
    }

    int raw = 0;
    int measured_mv = 0;
    const esp_err_t err = analog_backend_read_mv(s_configs[index].analog_source_id, &raw, &measured_mv);
    if (err != ESP_OK) {
        sample.state = MQ_SENSOR_STATE_ERROR;
        s_last_samples[index] = sample;
        *out = sample;
        mq_sensor_unlock();
        return err;
    }

    sample.raw_adc = raw;
    sample.measured_mv = measured_mv;
    sample.corrected_vrl_mv = measured_mv;

    if (sample.corrected_vrl_mv <= 0 ||
        (uint32_t)sample.corrected_vrl_mv >= s_configs[index].vc_mv) {
        sample.rs_ohms = 0.0f;
        sample.r0_ohms = s_calibrations[index].r0_ohms;
        sample.rs_r0_ratio = 0.0f;
        sample.state = MQ_SENSOR_STATE_ERROR;
        s_last_samples[index] = sample;
        *out = sample;
        mq_sensor_unlock();
        return ESP_ERR_INVALID_RESPONSE;
    }

    sample.rs_ohms = ((float)s_configs[index].vc_mv / (float)sample.corrected_vrl_mv - 1.0f) *
                     (float)s_configs[index].rl_ohms;

    if (!s_calibrations[index].valid || s_calibrations[index].r0_ohms <= 0.0f) {
        sample.r0_ohms = 0.0f;
        sample.rs_r0_ratio = 0.0f;
        sample.state = MQ_SENSOR_STATE_UNCALIBRATED;
    } else {
        sample.r0_ohms = s_calibrations[index].r0_ohms;
        sample.rs_r0_ratio = sample.rs_ohms / sample.r0_ohms;
        sample.state = MQ_SENSOR_STATE_READY;
    }

    s_last_samples[index] = sample;
    *out = sample;
    mq_sensor_unlock();
    return ESP_OK;
}

esp_err_t mq_sensor_read_all(void)
{
    ESP_RETURN_ON_ERROR(mq_sensor_lock(), TAG, "failed to lock MQ sensor manager");
    if (!s_initialized) {
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t first_error = ESP_OK;
    for (size_t i = 0; i < s_sensor_count; ++i) {
        mq_sensor_sample_t sample;
        esp_err_t err = mq_sensor_read(s_configs[i].id, &sample);
        if (err == ESP_ERR_INVALID_STATE && !s_configs[i].enabled) {
            continue;
        }
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }
    mq_sensor_unlock();
    return first_error;
}

esp_err_t mq_sensor_get_last(uint8_t id, mq_sensor_sample_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "sample output is required");

    ESP_RETURN_ON_ERROR(mq_sensor_lock(), TAG, "failed to lock MQ sensor manager");
    if (!s_initialized) {
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    const int index = find_sensor_index(id);
    if (index < 0) {
        mq_sensor_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    *out = s_last_samples[index];
    mq_sensor_unlock();
    return ESP_OK;
}

const char *mq_sensor_type_to_string(mq_sensor_type_t type)
{
    switch (type) {
    case MQ_SENSOR_MQ2:
        return "MQ-2";
    case MQ_SENSOR_MQ3:
        return "MQ-3";
    case MQ_SENSOR_MQ4:
        return "MQ-4";
    case MQ_SENSOR_MQ5:
        return "MQ-5";
    case MQ_SENSOR_MQ6:
        return "MQ-6";
    case MQ_SENSOR_MQ7:
        return "MQ-7";
    case MQ_SENSOR_MQ8:
        return "MQ-8";
    case MQ_SENSOR_MQ9:
        return "MQ-9";
    case MQ_SENSOR_MQ135:
        return "MQ-135";
    default:
        return "unknown";
    }
}

const char *mq_sensor_state_to_string(mq_sensor_state_t state)
{
    switch (state) {
    case MQ_SENSOR_STATE_DISABLED:
        return "disabled";
    case MQ_SENSOR_STATE_WARMING:
        return "warming";
    case MQ_SENSOR_STATE_UNCALIBRATED:
        return "uncalibrated";
    case MQ_SENSOR_STATE_READY:
        return "ready";
    case MQ_SENSOR_STATE_STALE:
        return "stale";
    case MQ_SENSOR_STATE_ERROR:
        return "error";
    default:
        return "unknown";
    }
}

esp_err_t mq_sensor_get_config_by_index(size_t index, mq_sensor_config_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "config output is required");

    ESP_RETURN_ON_ERROR(mq_sensor_lock(), TAG, "failed to lock MQ sensor manager");
    if (!s_initialized) {
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    if (index >= s_sensor_count) {
        mq_sensor_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    *out = s_configs[index];
    mq_sensor_unlock();
    return ESP_OK;
}

esp_err_t mq_sensor_calibrate_clean_air(uint8_t id,
                                        size_t sample_count,
                                        uint32_t sample_delay_ms,
                                        mq_calibration_record_t *saved_record)
{
    ESP_RETURN_ON_FALSE(sample_count >= 1 && sample_count <= 256, ESP_ERR_INVALID_ARG, TAG,
                        "sample_count must be between 1 and 256");
    ESP_RETURN_ON_FALSE(sample_delay_ms <= 5000, ESP_ERR_INVALID_ARG, TAG,
                        "sample_delay_ms must be <= 5000");

    ESP_RETURN_ON_ERROR(mq_sensor_lock(), TAG, "failed to lock MQ sensor manager");
    if (!s_initialized) {
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    const int index = find_sensor_index(id);
    if (index < 0) {
        mq_sensor_unlock();
        return ESP_ERR_NOT_FOUND;
    }
    if (!s_configs[index].supports_clean_air_calibration ||
        s_configs[index].clean_air_rs_r0_factor <= 0.0f) {
        mq_sensor_unlock();
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!s_configs[index].enabled) {
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    double sum_rs = 0.0;
    size_t valid_count = 0;
    for (size_t i = 0; i < sample_count; ++i) {
        mq_sensor_sample_t sample;
        const esp_err_t err = mq_sensor_read(id, &sample);
        if (err == ESP_OK && sample.rs_ohms > 0.0f) {
            sum_rs += sample.rs_ohms;
            ++valid_count;
        }

        if (sample_delay_ms > 0 && i + 1 < sample_count) {
            vTaskDelay(pdMS_TO_TICKS(sample_delay_ms));
        }
    }

    if (valid_count == 0) {
        mq_sensor_unlock();
        ESP_LOGW(TAG, "no valid Rs samples collected for sensor %u", (unsigned)id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    const float avg_rs = (float)(sum_rs / (double)valid_count);
    const float r0 = avg_rs / s_configs[index].clean_air_rs_r0_factor;
    time_t now = time(NULL);
    const uint32_t calibrated_at_unix = (now > 0 && (uint64_t)now <= UINT32_MAX) ? (uint32_t)now : 0;

    mq_calibration_record_t record = {
        .valid = true,
        .r0_ohms = r0,
        .calibrated_at_unix = calibrated_at_unix,
    };

    esp_err_t err = mq_calibration_nvs_save(id, &record);
    if (err != ESP_OK) {
        mq_sensor_unlock();
        ESP_LOGE(TAG, "failed to save clean-air calibration for sensor %u: %s",
                 (unsigned)id, esp_err_to_name(err));
        return err;
    }
    s_calibrations[index] = record;
    s_last_samples[index].r0_ohms = record.r0_ohms;
    if (s_last_samples[index].rs_ohms > 0.0f) {
        s_last_samples[index].rs_r0_ratio = s_last_samples[index].rs_ohms / record.r0_ohms;
        s_last_samples[index].state = MQ_SENSOR_STATE_READY;
    }

    if (saved_record != NULL) {
        *saved_record = record;
    }

    ESP_LOGW(TAG,
             "Experimental clean-air calibration saved for sensor %u: assumes clean air, correct RL/divider, and adequate MQ preheat/aging",
             (unsigned)id);
    mq_sensor_unlock();
    return ESP_OK;
}

esp_err_t mq_sensor_reload_calibration(uint8_t id)
{
    ESP_RETURN_ON_ERROR(mq_sensor_lock(), TAG, "failed to lock MQ sensor manager");
    if (!s_initialized) {
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    const int index = find_sensor_index(id);
    if (index < 0) {
        mq_sensor_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = mq_calibration_nvs_load(id, &s_calibrations[index]);
    if (err == ESP_ERR_NOT_FOUND) {
        clear_calibration(index);
        mq_sensor_unlock();
        return ESP_OK;
    }
    if (err != ESP_OK) {
        mq_sensor_unlock();
        ESP_LOGE(TAG, "failed to reload calibration for sensor %u: %s", (unsigned)id, esp_err_to_name(err));
        return err;
    }

    if (!s_calibrations[index].valid || s_calibrations[index].r0_ohms <= 0.0f) {
        clear_calibration(index);
    } else {
        s_last_samples[index].r0_ohms = s_calibrations[index].r0_ohms;
        if (s_last_samples[index].rs_ohms > 0.0f) {
            s_last_samples[index].rs_r0_ratio = s_last_samples[index].rs_ohms / s_calibrations[index].r0_ohms;
            s_last_samples[index].state = MQ_SENSOR_STATE_READY;
        }
    }

    mq_sensor_unlock();
    return ESP_OK;
}

esp_err_t mq_sensor_erase_calibration(uint8_t id)
{
    ESP_RETURN_ON_ERROR(mq_sensor_lock(), TAG, "failed to lock MQ sensor manager");
    if (!s_initialized) {
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    const int index = find_sensor_index(id);
    if (index < 0) {
        mq_sensor_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    const esp_err_t err = mq_calibration_nvs_erase(id);
    if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "failed to erase calibration for sensor %u: %s", (unsigned)id, esp_err_to_name(err));
        mq_sensor_unlock();
        return err;
    }

    clear_calibration(index);
    mq_sensor_unlock();
    return ESP_OK;
}
