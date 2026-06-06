#include "mq_sensor.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>

#include "analog_backend.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define MQ_SENSOR_MAX_SENSORS 9
#define MQ_SENSOR_DEFAULT_VC_MV 5000U
#define MQ_SENSOR_LOCK_TIMEOUT pdMS_TO_TICKS(5000)
#define MQ_SENSOR_WARMUP_24H_SECONDS 86400U
#define MQ_SENSOR_WARMUP_48H_SECONDS 172800U
#define MQ_SENSOR_FLOAT_EPSILON 0.0001f

static const char *TAG = "mq_sensor";

static const mq_sensor_config_t s_default_configs[MQ_SENSOR_MAX_SENSORS] = {
    {0, MQ_SENSOR_MQ135, "MQ-135", 0, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_WARMUP_48H_SECONDS, true,  true,  0.70f, 0.50f},
    {1, MQ_SENSOR_MQ2,   "MQ-2",   1, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_WARMUP_48H_SECONDS, true,  true,  0.70f, 0.50f},
    {2, MQ_SENSOR_MQ3,   "MQ-3",   2, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_WARMUP_24H_SECONDS, true,  true,  0.70f, 0.50f},
    {3, MQ_SENSOR_MQ4,   "MQ-4",   3, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_WARMUP_48H_SECONDS, true,  true,  0.70f, 0.50f},
    {4, MQ_SENSOR_MQ5,   "MQ-5",   4, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_WARMUP_24H_SECONDS, true,  true,  0.70f, 0.50f},
    {5, MQ_SENSOR_MQ6,   "MQ-6",   5, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_WARMUP_48H_SECONDS, true,  true,  0.70f, 0.50f},
    {6, MQ_SENSOR_MQ7,   "MQ-7",   6, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_WARMUP_48H_SECONDS, true,  false, 0.0f,  0.0f},
    {7, MQ_SENSOR_MQ8,   "MQ-8",   7, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_WARMUP_48H_SECONDS, true,  true,  0.70f, 0.50f},
    {8, MQ_SENSOR_MQ9,   "MQ-9",   8, MQ_SENSOR_DEFAULT_VC_MV, MQ_SENSOR_WARMUP_48H_SECONDS, true,  false, 0.0f,  0.0f},
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

static bool float_close(float a, float b)
{
    return fabsf(a - b) <= MQ_SENSOR_FLOAT_EPSILON;
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
    s_last_samples[index].baseline_vrl_mv = 0;
    s_last_samples[index].baseline_rs_norm = 0.0f;
    s_last_samples[index].rs_ratio = 0.0f;
    if (s_last_samples[index].state == MQ_SENSOR_STATE_READY) {
        s_last_samples[index].state = MQ_SENSOR_STATE_UNCALIBRATED;
    }
}

static void reset_sample(size_t index)
{
    memset(&s_last_samples[index], 0, sizeof(s_last_samples[index]));
    s_last_samples[index].id = s_configs[index].id;
    s_last_samples[index].state = s_configs[index].enabled ? MQ_SENSOR_STATE_WARMING : MQ_SENSOR_STATE_DISABLED;
    s_last_samples[index].raw_adc = -1;
    s_last_samples[index].adc_mv = -1;
    s_last_samples[index].vrl_mv = -1;
}

static bool is_still_warming(size_t index)
{
    if (!s_configs[index].enabled || s_configs[index].warmup_seconds == 0U) {
        return false;
    }

    const TickType_t elapsed_ticks = xTaskGetTickCount() - s_start_ticks[index];
    const uint64_t elapsed_ms = (uint64_t)elapsed_ticks * (uint64_t)portTICK_PERIOD_MS;
    const uint64_t required_ms = (uint64_t)s_configs[index].warmup_seconds * 1000ULL;
    return elapsed_ms < required_ms;
}

static mq_sensor_threshold_state_t threshold_state_for_sample(const mq_sensor_config_t *config,
                                                              const mq_sensor_sample_t *sample)
{
    if (config == NULL || sample == NULL || sample->state != MQ_SENSOR_STATE_READY) {
        return MQ_SENSOR_THRESHOLD_NONE;
    }
    if (config->critical_rs_ratio > 0.0f && sample->rs_ratio <= config->critical_rs_ratio) {
        return MQ_SENSOR_THRESHOLD_CRITICAL;
    }
    if (config->warning_rs_ratio > 0.0f && sample->rs_ratio <= config->warning_rs_ratio) {
        return MQ_SENSOR_THRESHOLD_WARNING;
    }
    return MQ_SENSOR_THRESHOLD_NORMAL;
}

static esp_err_t get_registered_source(size_t index, analog_source_config_t *source)
{
    esp_err_t err = analog_backend_get_source_config(s_configs[index].analog_source_id, source);
    if (err != ESP_OK) {
        return err;
    }
    if (source->type != ANALOG_BACKEND_MUX_ADC) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (source->input_divider_ratio < 0.1f || source->input_divider_ratio > 20.0f) {
        return ESP_ERR_INVALID_STATE;
    }
    if (source->mux_channel > 15U) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static bool calibration_matches_config(size_t index, const analog_source_config_t *source)
{
    const mq_calibration_record_t *record = &s_calibrations[index];
    return record->valid &&
           record->magic == MQ_CALIBRATION_RECORD_MAGIC &&
           record->version == MQ_CALIBRATION_RECORD_VERSION &&
           record->firmware_config_version == MQ_CALIBRATION_FIRMWARE_CONFIG_VERSION &&
           record->baseline_vrl_mv > 0U &&
           record->baseline_vrl_mv < s_configs[index].vc_mv &&
           record->baseline_rs_norm > 0.0f &&
           record->sensor_type == (uint8_t)s_configs[index].type &&
           record->source_id == s_configs[index].analog_source_id &&
           record->vc_mv == s_configs[index].vc_mv &&
           source != NULL &&
           source->type == ANALOG_BACKEND_MUX_ADC &&
           record->mux_id == source->mux_id &&
           record->mux_channel == source->mux_channel &&
           float_close(record->input_divider_ratio, source->input_divider_ratio);
}

static esp_err_t validate_config(const mq_sensor_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "sensor config is required");
    ESP_RETURN_ON_FALSE(config->type < MQ_SENSOR_TYPE_COUNT, ESP_ERR_INVALID_ARG, TAG,
                        "invalid sensor type %d", (int)config->type);
    ESP_RETURN_ON_FALSE(config->name != NULL, ESP_ERR_INVALID_ARG, TAG, "sensor name is required");
    ESP_RETURN_ON_FALSE(config->vc_mv >= 1000U && config->vc_mv <= 10000U,
                        ESP_ERR_INVALID_ARG, TAG, "sensor vc_mv outside 1000..10000");
    ESP_RETURN_ON_FALSE(config->warning_rs_ratio >= 0.0f && config->critical_rs_ratio >= 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "threshold ratios must not be negative");
    return ESP_OK;
}

esp_err_t mq_sensor_compute_rs_norm(uint32_t vc_mv, int vrl_mv, float *rs_norm)
{
    ESP_RETURN_ON_FALSE(rs_norm != NULL, ESP_ERR_INVALID_ARG, TAG, "normalized output is required");
    ESP_RETURN_ON_FALSE(vc_mv > 0, ESP_ERR_INVALID_ARG, TAG, "Vc must be positive");
    if (vrl_mv <= 0 || (uint32_t)vrl_mv >= vc_mv) {
        *rs_norm = 0.0f;
        return ESP_ERR_INVALID_RESPONSE;
    }
    *rs_norm = ((float)vc_mv / (float)vrl_mv) - 1.0f;
    return ESP_OK;
}

esp_err_t mq_sensor_compute_rs_ratio(uint32_t vc_mv, int vrl_mv, int baseline_vrl_mv, float *rs_ratio)
{
    ESP_RETURN_ON_FALSE(rs_ratio != NULL, ESP_ERR_INVALID_ARG, TAG, "ratio output is required");
    float current = 0.0f;
    float baseline = 0.0f;
    ESP_RETURN_ON_ERROR(mq_sensor_compute_rs_norm(vc_mv, vrl_mv, &current), TAG, "invalid current VRL");
    ESP_RETURN_ON_ERROR(mq_sensor_compute_rs_norm(vc_mv, baseline_vrl_mv, &baseline), TAG, "invalid baseline VRL");
    ESP_RETURN_ON_FALSE(baseline > 0.0f, ESP_ERR_INVALID_RESPONSE, TAG, "baseline normalized value is invalid");
    *rs_ratio = current / baseline;
    return ESP_OK;
}

esp_err_t mq_sensor_apply_divider_mv(int adc_mv, float divider_ratio, int *corrected_mv)
{
    ESP_RETURN_ON_FALSE(corrected_mv != NULL, ESP_ERR_INVALID_ARG, TAG, "corrected voltage output is required");
    ESP_RETURN_ON_FALSE(adc_mv >= 0, ESP_ERR_INVALID_ARG, TAG, "ADC millivolts must not be negative");
    ESP_RETURN_ON_FALSE(divider_ratio >= 0.1f && divider_ratio <= 20.0f,
                        ESP_ERR_INVALID_STATE, TAG, "divider ratio is not configured");
    *corrected_mv = (int)((float)adc_mv * divider_ratio + 0.5f);
    return ESP_OK;
}

esp_err_t mq_sensor_self_check(void)
{
    int corrected = 0;
    ESP_RETURN_ON_ERROR(mq_sensor_apply_divider_mv(1250, 2.0f, &corrected),
                        TAG, "divider self-check failed");
    ESP_RETURN_ON_FALSE(corrected == 2500, ESP_FAIL, TAG,
                        "divider self-check expected 2500 got %d", corrected);

    float norm = 0.0f;
    ESP_RETURN_ON_ERROR(mq_sensor_compute_rs_norm(5000U, 2500, &norm),
                        TAG, "normalized Rs self-check failed");
    ESP_RETURN_ON_FALSE(fabsf(norm - 1.0f) < 0.0001f, ESP_FAIL, TAG,
                        "normalized Rs self-check expected 1.0 got %.4f", (double)norm);
    ESP_RETURN_ON_FALSE(mq_sensor_compute_rs_norm(5000U, 0, &norm) == ESP_ERR_INVALID_RESPONSE,
                        ESP_FAIL, TAG, "VRL<=0 self-check failed");
    ESP_RETURN_ON_FALSE(mq_sensor_compute_rs_norm(5000U, 5000, &norm) == ESP_ERR_INVALID_RESPONSE,
                        ESP_FAIL, TAG, "VRL>=Vc self-check failed");

    float ratio = 0.0f;
    ESP_RETURN_ON_ERROR(mq_sensor_compute_rs_ratio(5000U, 2500, 2500, &ratio),
                        TAG, "ratio self-check failed");
    ESP_RETURN_ON_FALSE(fabsf(ratio - 1.0f) < 0.0001f, ESP_FAIL, TAG,
                        "ratio self-check expected 1.0 got %.4f", (double)ratio);
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
        if (err == ESP_OK) {
            if (!s_calibrations[i].valid ||
                s_calibrations[i].baseline_vrl_mv == 0U ||
                s_calibrations[i].magic != MQ_CALIBRATION_RECORD_MAGIC ||
                s_calibrations[i].version != MQ_CALIBRATION_RECORD_VERSION) {
                clear_calibration(i);
            }
        } else {
            if (err != ESP_ERR_NOT_FOUND && err != ESP_ERR_INVALID_SIZE) {
                ESP_LOGW(TAG, "failed to load baseline for sensor %u: %s",
                         (unsigned)s_configs[i].id, esp_err_to_name(err));
            }
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
    sample.fault_bitmap = MQ_SENSOR_FAULT_NONE;
    sample.threshold_state = MQ_SENSOR_THRESHOLD_NONE;
    if (!s_configs[index].enabled) {
        sample.state = MQ_SENSOR_STATE_DISABLED;
        s_last_samples[index] = sample;
        *out = sample;
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    analog_source_config_t source = {0};
    esp_err_t source_err = get_registered_source((size_t)index, &source);
    if (source_err != ESP_OK) {
        sample.fault_bitmap |= source_err == ESP_ERR_NOT_FOUND ? MQ_SENSOR_FAULT_SOURCE_MISSING
                                                               : MQ_SENSOR_FAULT_DIVIDER_UNCONFIGURED;
        sample.state = MQ_SENSOR_STATE_FAULT;
        s_last_samples[index] = sample;
        *out = sample;
        mq_sensor_unlock();
        return source_err;
    }

    if (is_still_warming((size_t)index)) {
        sample.state = MQ_SENSOR_STATE_WARMING;
        sample.raw_adc = -1;
        sample.adc_mv = -1;
        sample.vrl_mv = -1;
        sample.rs_norm = 0.0f;
        sample.baseline_vrl_mv = s_calibrations[index].valid ? (int)s_calibrations[index].baseline_vrl_mv : 0;
        sample.baseline_rs_norm = s_calibrations[index].valid ? s_calibrations[index].baseline_rs_norm : 0.0f;
        sample.rs_ratio = 0.0f;
        s_last_samples[index] = sample;
        *out = sample;
        mq_sensor_unlock();
        return ESP_OK;
    }

    int raw = 0;
    int adc_mv = 0;
    esp_err_t err = analog_backend_read_mv(s_configs[index].analog_source_id, &raw, &adc_mv);
    if (err != ESP_OK) {
        sample.fault_bitmap |= MQ_SENSOR_FAULT_ADC_READ;
        sample.state = MQ_SENSOR_STATE_FAULT;
        s_last_samples[index] = sample;
        *out = sample;
        mq_sensor_unlock();
        return err;
    }

    sample.raw_adc = raw;
    sample.adc_mv = adc_mv;

    err = mq_sensor_apply_divider_mv(adc_mv, source.input_divider_ratio, &sample.vrl_mv);
    if (err != ESP_OK) {
        sample.fault_bitmap |= MQ_SENSOR_FAULT_DIVIDER_UNCONFIGURED;
        sample.state = MQ_SENSOR_STATE_FAULT;
        s_last_samples[index] = sample;
        *out = sample;
        mq_sensor_unlock();
        return err;
    }

    err = mq_sensor_compute_rs_norm(s_configs[index].vc_mv, sample.vrl_mv, &sample.rs_norm);
    if (err != ESP_OK) {
        sample.fault_bitmap |= MQ_SENSOR_FAULT_INVALID_VRL;
        sample.rs_ratio = 0.0f;
        sample.state = MQ_SENSOR_STATE_FAULT;
        s_last_samples[index] = sample;
        *out = sample;
        mq_sensor_unlock();
        return err;
    }

    if (!s_configs[index].supports_baseline_calibration) {
        sample.baseline_vrl_mv = s_calibrations[index].valid ? (int)s_calibrations[index].baseline_vrl_mv : 0;
        sample.baseline_rs_norm = s_calibrations[index].valid ? s_calibrations[index].baseline_rs_norm : 0.0f;
        sample.rs_ratio = 0.0f;
        sample.state = MQ_SENSOR_STATE_DIAGNOSTIC_ONLY;
    } else if (!calibration_matches_config((size_t)index, &source)) {
        sample.baseline_vrl_mv = s_calibrations[index].valid ? (int)s_calibrations[index].baseline_vrl_mv : 0;
        sample.baseline_rs_norm = s_calibrations[index].valid ? s_calibrations[index].baseline_rs_norm : 0.0f;
        sample.rs_ratio = 0.0f;
        sample.fault_bitmap |= s_calibrations[index].valid ? MQ_SENSOR_FAULT_CALIBRATION_MISMATCH
                                                           : MQ_SENSOR_FAULT_NONE;
        sample.state = MQ_SENSOR_STATE_UNCALIBRATED;
    } else {
        sample.baseline_vrl_mv = (int)s_calibrations[index].baseline_vrl_mv;
        sample.baseline_rs_norm = s_calibrations[index].baseline_rs_norm;
        sample.rs_ratio = sample.rs_norm / sample.baseline_rs_norm;
        sample.state = MQ_SENSOR_STATE_READY;
    }
    sample.threshold_state = threshold_state_for_sample(&s_configs[index], &sample);

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
    case MQ_SENSOR_STATE_FAULT:
        return "fault";
    case MQ_SENSOR_STATE_DIAGNOSTIC_ONLY:
        return "diagnostic-only";
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

esp_err_t mq_sensor_calibrate_baseline(uint8_t id,
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
    if (!s_configs[index].supports_baseline_calibration) {
        mq_sensor_unlock();
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!s_configs[index].enabled || is_still_warming((size_t)index)) {
        mq_sensor_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    analog_source_config_t source = {0};
    esp_err_t err = get_registered_source((size_t)index, &source);
    if (err != ESP_OK) {
        mq_sensor_unlock();
        return err;
    }

    double sum_vrl = 0.0;
    double sum_sq_vrl = 0.0;
    size_t valid_count = 0;
    for (size_t i = 0; i < sample_count; ++i) {
        mq_sensor_sample_t sample;
        err = mq_sensor_read(id, &sample);
        if (err == ESP_OK &&
            (sample.state == MQ_SENSOR_STATE_UNCALIBRATED || sample.state == MQ_SENSOR_STATE_READY) &&
            sample.vrl_mv > 0 &&
            (uint32_t)sample.vrl_mv < s_configs[index].vc_mv) {
            sum_vrl += sample.vrl_mv;
            sum_sq_vrl += (double)sample.vrl_mv * (double)sample.vrl_mv;
            ++valid_count;
        } else {
            mq_sensor_unlock();
            return err == ESP_OK ? ESP_ERR_INVALID_RESPONSE : err;
        }

        if (sample_delay_ms > 0 && i + 1 < sample_count) {
            vTaskDelay(pdMS_TO_TICKS(sample_delay_ms));
        }
    }

    if (valid_count == 0) {
        mq_sensor_unlock();
        ESP_LOGW(TAG, "no valid baseline samples collected for sensor %u", (unsigned)id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    const double avg_vrl_double = sum_vrl / (double)valid_count;
    double variance = (sum_sq_vrl / (double)valid_count) - (avg_vrl_double * avg_vrl_double);
    if (variance < 0.0) {
        variance = 0.0;
    }
    const uint32_t baseline_vrl_mv = (uint32_t)(avg_vrl_double + 0.5);
    const float stdev_vrl_mv = (float)sqrt(variance);
    float baseline_rs_norm = 0.0f;
    err = mq_sensor_compute_rs_norm(s_configs[index].vc_mv, (int)baseline_vrl_mv, &baseline_rs_norm);
    if (err != ESP_OK) {
        mq_sensor_unlock();
        return err;
    }

    time_t now = time(NULL);
    const uint32_t baseline_at_unix = (now > 0 && (uint64_t)now <= UINT32_MAX) ? (uint32_t)now : 0;

    mq_calibration_record_t record = {
        .valid = true,
        .baseline_vrl_mv = baseline_vrl_mv,
        .baseline_at_unix = baseline_at_unix,
        .magic = MQ_CALIBRATION_RECORD_MAGIC,
        .version = MQ_CALIBRATION_RECORD_VERSION,
        .firmware_config_version = MQ_CALIBRATION_FIRMWARE_CONFIG_VERSION,
        .sensor_type = (uint8_t)s_configs[index].type,
        .source_id = s_configs[index].analog_source_id,
        .mux_id = source.mux_id,
        .mux_channel = source.mux_channel,
        .sample_count = (uint16_t)valid_count,
        .vc_mv = s_configs[index].vc_mv,
        .input_divider_ratio = source.input_divider_ratio,
        .baseline_rs_norm = baseline_rs_norm,
        .baseline_vrl_mean_mv = (float)avg_vrl_double,
        .baseline_vrl_stddev_mv = stdev_vrl_mv,
    };

    err = mq_calibration_nvs_save(id, &record);
    if (err != ESP_OK) {
        mq_sensor_unlock();
        ESP_LOGE(TAG, "failed to save baseline for sensor %u: %s",
                 (unsigned)id, esp_err_to_name(err));
        return err;
    }
    s_calibrations[index] = record;
    s_last_samples[index].baseline_vrl_mv = (int)record.baseline_vrl_mv;
    s_last_samples[index].baseline_rs_norm = record.baseline_rs_norm;
    if (s_last_samples[index].rs_norm > 0.0f) {
        s_last_samples[index].rs_ratio = s_last_samples[index].rs_norm / record.baseline_rs_norm;
        s_last_samples[index].state = MQ_SENSOR_STATE_READY;
        s_last_samples[index].threshold_state = threshold_state_for_sample(&s_configs[index], &s_last_samples[index]);
    }

    if (saved_record != NULL) {
        *saved_record = record;
    }

    ESP_LOGW(TAG,
             "Baseline saved for sensor %u: qualitative MQ ratio only; requires correct divider and complete module aging",
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
    if (err == ESP_ERR_NOT_FOUND || err == ESP_ERR_INVALID_SIZE) {
        clear_calibration(index);
        mq_sensor_unlock();
        return ESP_OK;
    }
    if (err != ESP_OK) {
        mq_sensor_unlock();
        ESP_LOGE(TAG, "failed to reload baseline for sensor %u: %s", (unsigned)id, esp_err_to_name(err));
        return err;
    }

    if (!s_calibrations[index].valid ||
        s_calibrations[index].baseline_vrl_mv == 0U ||
        s_calibrations[index].magic != MQ_CALIBRATION_RECORD_MAGIC ||
        s_calibrations[index].version != MQ_CALIBRATION_RECORD_VERSION) {
        clear_calibration(index);
    } else {
        s_last_samples[index].baseline_vrl_mv = (int)s_calibrations[index].baseline_vrl_mv;
        s_last_samples[index].baseline_rs_norm = s_calibrations[index].baseline_rs_norm;
        if (s_last_samples[index].rs_norm > 0.0f && s_calibrations[index].baseline_rs_norm > 0.0f) {
            s_last_samples[index].rs_ratio = s_last_samples[index].rs_norm / s_calibrations[index].baseline_rs_norm;
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
        ESP_LOGE(TAG, "failed to erase baseline for sensor %u: %s", (unsigned)id, esp_err_to_name(err));
        mq_sensor_unlock();
        return err;
    }

    clear_calibration(index);
    mq_sensor_unlock();
    return ESP_OK;
}
