#include "air_quality_service.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define AQ_SERVICE_TASK_STACK 4096
#define AQ_SERVICE_TASK_PRIORITY 5
#define AQ_SERVICE_LOCK_TIMEOUT pdMS_TO_TICKS(1000)
#define AQ_SERVICE_STOP_TIMEOUT_MS 2000
#define AQ_SERVICE_DELAY_SLICE_MS 100

static const char *TAG = "air_quality_service";

static air_quality_service_config_t s_config;
static air_quality_service_status_t s_status;
static SemaphoreHandle_t s_lock;
static TaskHandle_t s_task_handle;
static bool s_have_filtered_ratio;

static esp_err_t aq_lock(void)
{
    ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_INVALID_STATE, TAG, "service lock is not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, AQ_SERVICE_LOCK_TIMEOUT) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "timed out waiting for service lock");
    return ESP_OK;
}

static void aq_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

static matter_air_quality_level_t map_ratio_to_level(float ratio)
{
    if (ratio >= 0.80f) {
        return MATTER_AIR_QUALITY_GOOD;
    }
    if (ratio >= 0.65f) {
        return MATTER_AIR_QUALITY_FAIR;
    }
    if (ratio >= 0.50f) {
        return MATTER_AIR_QUALITY_MODERATE;
    }
    if (ratio >= 0.35f) {
        return MATTER_AIR_QUALITY_POOR;
    }
    if (ratio >= 0.20f) {
        return MATTER_AIR_QUALITY_VERY_POOR;
    }
    return MATTER_AIR_QUALITY_EXTREMELY_POOR;
}

static int32_t ratio_to_milli(float value)
{
    if (value <= 0.0f) {
        return 0;
    }
    return (int32_t)(value * 1000.0f + 0.5f);
}

static void init_diagnostics_defaults(matter_air_quality_diagnostics_t *diagnostics)
{
    memset(diagnostics, 0, sizeof(*diagnostics));
    for (uint8_t i = 0; i < MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT; ++i) {
        diagnostics->sensors[i].state = (uint8_t)MQ_SENSOR_STATE_FAULT;
        diagnostics->sensors[i].raw_adc = -1;
        diagnostics->sensors[i].adc_mv = -1;
        diagnostics->sensors[i].vrl_mv = -1;
    }
}

static void fill_sensor_diagnostics(matter_mq_sensor_diagnostics_t *out,
                                    const mq_sensor_config_t *config,
                                    const mq_sensor_sample_t *sample,
                                    uint32_t age_ms)
{
    out->sensor_type = (uint8_t)config->type;
    out->enabled = config->enabled;
    out->state = (uint8_t)sample->state;
    out->raw_adc = sample->raw_adc;
    out->adc_mv = sample->adc_mv;
    out->vrl_mv = sample->vrl_mv;
    out->baseline_vrl_mv = sample->baseline_vrl_mv;
    out->rs_norm_milli = ratio_to_milli(sample->rs_norm);
    out->rs_ratio_milli = ratio_to_milli(sample->rs_ratio);
    out->threshold_state = (uint8_t)sample->threshold_state;
    out->fault_bitmap = sample->fault_bitmap;
    out->last_update_age_ms = age_ms;
    out->baseline_valid = sample->baseline_vrl_mv > 0 && sample->baseline_rs_norm > 0.0f;
}

static uint32_t aq_now_ms(void)
{
    return (uint32_t)((uint64_t)xTaskGetTickCount() * (uint64_t)portTICK_PERIOD_MS);
}

static uint32_t sample_age_ms(uint32_t now_ms, const mq_sensor_sample_t *sample)
{
    if (sample == NULL || sample->updated_at_ms == 0U) {
        return UINT32_MAX;
    }
    if (now_ms < sample->updated_at_ms) {
        return 0;
    }
    return now_ms - sample->updated_at_ms;
}

static bool aq_is_running(void)
{
    bool running = false;
    if (aq_lock() == ESP_OK) {
        running = s_status.running;
        aq_unlock();
    }
    return running;
}

static void aq_delay_interval(uint32_t interval_ms)
{
    uint32_t waited_ms = 0;
    while (aq_is_running() && waited_ms < interval_ms) {
        uint32_t delay_ms = interval_ms - waited_ms;
        if (delay_ms > AQ_SERVICE_DELAY_SLICE_MS) {
            delay_ms = AQ_SERVICE_DELAY_SLICE_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        waited_ms += delay_ms;
    }
}

static void aq_sample_once(void)
{
    air_quality_service_config_t config;
    if (aq_lock() != ESP_OK) {
        return;
    }
    config = s_config;
    aq_unlock();

    matter_air_quality_diagnostics_t diagnostics;
    init_diagnostics_defaults(&diagnostics);

    mq_sensor_config_t sensor_configs[MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT];
    mq_sensor_sample_t sensor_samples[MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT];
    bool sensor_present[MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT] = {false};
    memset(sensor_configs, 0, sizeof(sensor_configs));
    memset(sensor_samples, 0, sizeof(sensor_samples));

    mq_sensor_sample_t sample;
    memset(&sample, 0, sizeof(sample));
    sample.id = config.primary_sensor_id;
    sample.state = MQ_SENSOR_STATE_FAULT;
    sample.raw_adc = -1;
    sample.adc_mv = -1;
    sample.vrl_mv = -1;
    esp_err_t read_err = ESP_ERR_NOT_FOUND;

    const size_t count = mq_sensor_count();
    for (size_t i = 0; i < count; ++i) {
        mq_sensor_config_t sensor_config;
        if (mq_sensor_get_config_by_index(i, &sensor_config) != ESP_OK) {
            continue;
        }

        mq_sensor_sample_t sensor_sample;
        memset(&sensor_sample, 0, sizeof(sensor_sample));
        sensor_sample.id = sensor_config.id;
        sensor_sample.state = sensor_config.enabled ? MQ_SENSOR_STATE_FAULT : MQ_SENSOR_STATE_DISABLED;
        sensor_sample.raw_adc = -1;
        sensor_sample.adc_mv = -1;
        sensor_sample.vrl_mv = -1;
        const esp_err_t sensor_err = mq_sensor_read(sensor_config.id, &sensor_sample);
        if (sensor_config.id < MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT) {
            sensor_configs[sensor_config.id] = sensor_config;
            sensor_samples[sensor_config.id] = sensor_sample;
            sensor_present[sensor_config.id] = true;
        }
        if (sensor_config.id == config.primary_sensor_id) {
            sample = sensor_sample;
            read_err = sensor_err;
        }
    }

    const uint32_t now_ms = aq_now_ms();
    matter_air_quality_level_t next_level = MATTER_AIR_QUALITY_UNKNOWN;
    bool should_schedule_air_quality = false;

    if (aq_lock() != ESP_OK) {
        return;
    }

    s_status.last_error = read_err;

    if (read_err == ESP_OK && sample.state == MQ_SENSOR_STATE_READY) {
        ++s_status.successful_reads;
        s_status.last_success_ms = now_ms;
        s_status.last_sample_age_ms = sample_age_ms(now_ms, &sample);
        if (!s_have_filtered_ratio) {
            s_status.filtered_ratio = sample.rs_ratio;
            s_have_filtered_ratio = true;
        } else {
            s_status.filtered_ratio = config.ema_alpha * sample.rs_ratio +
                                      (1.0f - config.ema_alpha) * s_status.filtered_ratio;
        }
        next_level = map_ratio_to_level(s_status.filtered_ratio);
    } else {
        if (read_err == ESP_OK) {
            ++s_status.successful_reads;
        } else {
            ++s_status.failed_reads;
        }
        if (s_status.last_success_ms != 0) {
            s_status.last_sample_age_ms = now_ms - s_status.last_success_ms;
        } else {
            s_status.last_sample_age_ms = sample_age_ms(now_ms, &sample);
        }
        s_have_filtered_ratio = false;
        s_status.filtered_ratio = 0.0f;
    }

    if (s_status.last_success_ms == 0) {
        next_level = MATTER_AIR_QUALITY_UNKNOWN;
    } else if (s_status.last_sample_age_ms > config.stale_after_ms) {
        sample.state = MQ_SENSOR_STATE_STALE;
        next_level = MATTER_AIR_QUALITY_UNKNOWN;
        s_have_filtered_ratio = false;
        s_status.filtered_ratio = 0.0f;
    }
    if (config.primary_sensor_id < MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT && sensor_present[config.primary_sensor_id]) {
        sensor_samples[config.primary_sensor_id] = sample;
    }

    s_status.last_sample = sample;
    s_status.current_level = next_level;
    should_schedule_air_quality = config.publish_to_matter;
    aq_unlock();

    for (uint8_t i = 0; i < MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT; ++i) {
        if (!sensor_present[i]) {
            continue;
        }
        fill_sensor_diagnostics(&diagnostics.sensors[i],
                                &sensor_configs[i],
                                &sensor_samples[i],
                                sample_age_ms(now_ms, &sensor_samples[i]));
    }

    if (config.publish_to_matter) {
        const esp_err_t diagnostics_err = matter_update_air_quality_diagnostics(config.matter_endpoint_id,
                                                                               &diagnostics);
        if (diagnostics_err != ESP_OK) {
            ESP_LOGW(TAG, "failed to publish MQ diagnostics: %s", esp_err_to_name(diagnostics_err));
        }
    }

    if (!should_schedule_air_quality) {
        return;
    }

    const esp_err_t matter_err = matter_update_air_quality(config.matter_endpoint_id, next_level);
    if (aq_lock() != ESP_OK) {
        return;
    }
    if (matter_err == ESP_OK) {
        s_status.last_scheduled_level = next_level;
        ++s_status.matter_update_schedules;
        s_status.last_error = read_err;
        ESP_LOGI(TAG, "Scheduled Air Quality level %s",
                 air_quality_service_level_to_string(next_level));
    } else {
        s_status.last_error = matter_err;
        ESP_LOGE(TAG, "failed to schedule Air Quality level %s: %s",
                 air_quality_service_level_to_string(next_level), esp_err_to_name(matter_err));
    }
    aq_unlock();
}

static void aq_task(void *arg)
{
    (void)arg;

    while (aq_is_running()) {
        aq_sample_once();

        uint32_t interval_ms = 0;
        if (aq_lock() == ESP_OK) {
            interval_ms = s_config.sample_interval_ms;
            aq_unlock();
        }
        aq_delay_interval(interval_ms);
    }

    if (aq_lock() == ESP_OK) {
        s_status.running = false;
        s_task_handle = NULL;
        aq_unlock();
    }

    vTaskDelete(NULL);
}

esp_err_t air_quality_service_init(const air_quality_service_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "service config is required");
    ESP_RETURN_ON_FALSE(config->sample_interval_ms > 0, ESP_ERR_INVALID_ARG, TAG,
                        "sample_interval_ms must be positive");
    ESP_RETURN_ON_FALSE(config->stale_after_ms > 0, ESP_ERR_INVALID_ARG, TAG,
                        "stale_after_ms must be positive");
    ESP_RETURN_ON_FALSE(config->ema_alpha > 0.0f && config->ema_alpha <= 1.0f,
                        ESP_ERR_INVALID_ARG, TAG, "ema_alpha must be in (0, 1]");
    ESP_RETURN_ON_FALSE(config->primary_sensor_id == 0U,
                        ESP_ERR_INVALID_ARG, TAG, "Matter Air Quality primary must be sensor 0");
    ESP_RETURN_ON_FALSE(!config->publish_to_matter || config->matter_endpoint_id != 0,
                        ESP_ERR_INVALID_ARG, TAG, "Matter endpoint is required when publishing");

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create service lock");
    }

    ESP_RETURN_ON_ERROR(aq_lock(), TAG, "failed to lock service");
    if (s_status.running) {
        aq_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    s_config = *config;
    memset(&s_status, 0, sizeof(s_status));
    s_status.initialized = true;
    s_status.running = false;
    s_status.matter_endpoint_id = config->matter_endpoint_id;
    s_status.primary_sensor_id = config->primary_sensor_id;
    s_status.current_level = MATTER_AIR_QUALITY_UNKNOWN;
    s_status.last_scheduled_level = MATTER_AIR_QUALITY_UNKNOWN;
    s_status.last_sample.id = config->primary_sensor_id;
    s_status.last_sample.state = MQ_SENSOR_STATE_STALE;
    s_status.last_error = ESP_OK;
    s_have_filtered_ratio = false;
    s_task_handle = NULL;
    aq_unlock();

    ESP_LOGI(TAG, "Air Quality service initialized for sensor %u endpoint %u",
             (unsigned)config->primary_sensor_id, (unsigned)config->matter_endpoint_id);
    return ESP_OK;
}

esp_err_t air_quality_service_start(void)
{
    ESP_RETURN_ON_ERROR(aq_lock(), TAG, "failed to lock service");
    if (!s_status.initialized) {
        aq_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    if (s_status.running) {
        aq_unlock();
        return ESP_OK;
    }

    s_status.running = true;
    const BaseType_t created = xTaskCreate(aq_task,
                                           "aq_service",
                                           AQ_SERVICE_TASK_STACK,
                                           NULL,
                                           AQ_SERVICE_TASK_PRIORITY,
                                           &s_task_handle);
    if (created != pdPASS) {
        s_status.running = false;
        s_task_handle = NULL;
        aq_unlock();
        return ESP_ERR_NO_MEM;
    }
    aq_unlock();

    ESP_LOGI(TAG, "Air Quality service started");
    return ESP_OK;
}

esp_err_t air_quality_service_stop(void)
{
    ESP_RETURN_ON_ERROR(aq_lock(), TAG, "failed to lock service");
    if (!s_status.initialized) {
        aq_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_status.running) {
        aq_unlock();
        return ESP_OK;
    }
    s_status.running = false;
    aq_unlock();

    uint32_t waited_ms = 0;
    while (waited_ms < AQ_SERVICE_STOP_TIMEOUT_MS) {
        ESP_RETURN_ON_ERROR(aq_lock(), TAG, "failed to lock service");
        const bool stopped = s_task_handle == NULL;
        aq_unlock();
        if (stopped) {
            ESP_LOGI(TAG, "Air Quality service stopped");
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        waited_ms += 50;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t air_quality_service_get_status(air_quality_service_status_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "status output is required");
    ESP_RETURN_ON_ERROR(aq_lock(), TAG, "failed to lock service");
    if (!s_status.initialized) {
        aq_unlock();
        return ESP_ERR_INVALID_STATE;
    }
    *out = s_status;
    aq_unlock();
    return ESP_OK;
}

const char *air_quality_service_level_to_string(matter_air_quality_level_t level)
{
    switch (level) {
    case MATTER_AIR_QUALITY_UNKNOWN:
        return "UNKNOWN";
    case MATTER_AIR_QUALITY_GOOD:
        return "GOOD";
    case MATTER_AIR_QUALITY_FAIR:
        return "FAIR";
    case MATTER_AIR_QUALITY_MODERATE:
        return "MODERATE";
    case MATTER_AIR_QUALITY_POOR:
        return "POOR";
    case MATTER_AIR_QUALITY_VERY_POOR:
        return "VERY_POOR";
    case MATTER_AIR_QUALITY_EXTREMELY_POOR:
        return "EXTREMELY_POOR";
    default:
        return "UNKNOWN";
    }
}
