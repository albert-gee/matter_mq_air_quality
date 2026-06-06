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

static void fill_diagnostics(uint16_t endpoint_id,
                             const mq_sensor_sample_t *sample,
                             uint32_t age_ms,
                             matter_air_quality_diagnostics_t *out)
{
    memset(out, 0, sizeof(*out));
    if (sample == NULL) {
        return;
    }

    mq_sensor_config_t config;
    for (size_t i = 0; i < mq_sensor_count(); ++i) {
        if (mq_sensor_get_config_by_index(i, &config) == ESP_OK && config.id == sample->id) {
            out->sensor_type = (uint8_t)config.type;
            out->enabled = config.enabled ? 1 : 0;
            break;
        }
    }

    (void)endpoint_id;
    out->sensor_id = sample->id;
    out->state = (uint8_t)sample->state;
    out->raw_adc = sample->raw_adc;
    out->adc_mv = sample->adc_mv;
    out->vrl_mv = sample->vrl_mv;
    out->baseline_vrl_mv = sample->baseline_vrl_mv;
    out->rs_norm_milli = (int32_t)(sample->rs_norm * 1000.0f + 0.5f);
    out->rs_ratio_milli = (int32_t)(sample->rs_ratio * 1000.0f + 0.5f);
    out->baseline_valid = sample->state == MQ_SENSOR_STATE_READY ? 1 : 0;
    out->threshold_state = (uint8_t)sample->threshold_state;
    out->fault_bitmap = sample->fault_bitmap;
    out->last_update_age_ms = age_ms;
}

static uint32_t aq_now_ms(void)
{
    return (uint32_t)((uint64_t)xTaskGetTickCount() * (uint64_t)portTICK_PERIOD_MS);
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

    mq_sensor_sample_t sample;
    memset(&sample, 0, sizeof(sample));
    sample.id = config.primary_sensor_id;
    sample.state = MQ_SENSOR_STATE_FAULT;

    esp_err_t read_err = mq_sensor_read(config.primary_sensor_id, &sample);
    const uint32_t now_ms = aq_now_ms();
    matter_air_quality_level_t next_level = MATTER_AIR_QUALITY_UNKNOWN;
    bool should_publish = false;

    if (aq_lock() != ESP_OK) {
        return;
    }

    s_status.last_error = read_err;

    if (read_err == ESP_OK && sample.state == MQ_SENSOR_STATE_READY) {
        ++s_status.successful_reads;
        s_status.last_success_ms = now_ms;
        s_status.last_sample_age_ms = 0;
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
            s_status.last_sample_age_ms = 0;
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

    s_status.last_sample = sample;
    s_status.current_level = next_level;
    should_publish = config.publish_to_matter && next_level != s_status.last_published_level;
    matter_air_quality_diagnostics_t diagnostics;
    fill_diagnostics(config.matter_endpoint_id, &sample, s_status.last_sample_age_ms, &diagnostics);
    aq_unlock();

    if (config.publish_to_matter) {
        const esp_err_t diag_err = matter_update_air_quality_diagnostics(config.matter_endpoint_id, &diagnostics);
        if (diag_err != ESP_OK) {
            ESP_LOGW(TAG, "failed to publish MQ diagnostics: %s", esp_err_to_name(diag_err));
        }
    }

    if (!should_publish) {
        return;
    }

    const esp_err_t matter_err = matter_update_air_quality(config.matter_endpoint_id, next_level);
    if (aq_lock() != ESP_OK) {
        return;
    }
    if (matter_err == ESP_OK) {
        s_status.last_published_level = next_level;
        ++s_status.matter_updates;
        s_status.last_error = read_err;
        ESP_LOGI(TAG, "Published Air Quality level %s",
                 air_quality_service_level_to_string(next_level));
    } else {
        s_status.last_error = matter_err;
        ESP_LOGE(TAG, "failed to publish Air Quality level %s: %s",
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
    s_status.last_published_level = MATTER_AIR_QUALITY_UNKNOWN;
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
