#include "analog_backend.h"

#include <stdbool.h>
#include <string.h>

#include "adc_service.h"
#include "analog_mux.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define ANALOG_BACKEND_MAX_SOURCES 16
#define ANALOG_BACKEND_LOCK_TIMEOUT pdMS_TO_TICKS(1000)

static const char *TAG = "analog_backend";

typedef struct {
    bool registered;
    analog_source_config_t config;
} analog_source_entry_t;

static bool s_initialized;
static SemaphoreHandle_t s_lock;
static analog_source_entry_t s_sources[ANALOG_BACKEND_MAX_SOURCES];

static esp_err_t lock_backend(void)
{
    ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_INVALID_STATE, TAG, "analog backend lock is not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, ANALOG_BACKEND_LOCK_TIMEOUT) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "timed out waiting for analog backend lock");
    return ESP_OK;
}

static void unlock_backend(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

esp_err_t analog_backend_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create analog backend lock");
    }

    ESP_RETURN_ON_ERROR(lock_backend(), TAG, "failed to lock analog backend during init");
    memset(s_sources, 0, sizeof(s_sources));
    s_initialized = true;
    unlock_backend();

    ESP_LOGI(TAG, "Analog backend initialized");
    return ESP_OK;
}

esp_err_t analog_backend_register_source(const analog_source_config_t *config)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "analog backend is not initialized");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "source config is required");
    ESP_RETURN_ON_FALSE(config->source_id < ANALOG_BACKEND_MAX_SOURCES, ESP_ERR_INVALID_ARG, TAG,
                        "source id %u is out of range", (unsigned)config->source_id);
    ESP_RETURN_ON_FALSE(config->type <= ANALOG_BACKEND_MUX_ADC, ESP_ERR_INVALID_ARG, TAG,
                        "unsupported analog backend type %d", (int)config->type);
    ESP_RETURN_ON_FALSE(config->input_divider_ratio > 0.0f, ESP_ERR_INVALID_ARG, TAG,
                        "input divider ratio must be positive");

    ESP_RETURN_ON_ERROR(lock_backend(), TAG, "failed to lock analog backend for source registration");
    s_sources[config->source_id].config = *config;
    s_sources[config->source_id].registered = true;
    unlock_backend();

    ESP_LOGI(TAG, "Registered analog source %u", (unsigned)config->source_id);
    return ESP_OK;
}

esp_err_t analog_backend_read_mv(uint8_t source_id, int *raw, int *mv)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "analog backend is not initialized");
    ESP_RETURN_ON_FALSE(raw != NULL, ESP_ERR_INVALID_ARG, TAG, "raw output is required");
    ESP_RETURN_ON_FALSE(mv != NULL, ESP_ERR_INVALID_ARG, TAG, "millivolt output is required");
    ESP_RETURN_ON_FALSE(source_id < ANALOG_BACKEND_MAX_SOURCES, ESP_ERR_INVALID_ARG, TAG,
                        "source id %u is out of range", (unsigned)source_id);

    ESP_RETURN_ON_ERROR(lock_backend(), TAG, "failed to lock analog backend for read");

    esp_err_t err = ESP_OK;
    int adc_mv = 0;
    analog_mux_config_t mux_config = {0};
    const analog_source_config_t *config = &s_sources[source_id].config;
    if (!s_sources[source_id].registered) {
        err = ESP_ERR_NOT_FOUND;
        ESP_LOGE(TAG, "analog source %u is not registered", (unsigned)source_id);
        goto out;
    }

    switch (config->type) {
    case ANALOG_BACKEND_INTERNAL_ADC: {
        err = adc_service_read_mv(config->adc_logical_channel, raw, &adc_mv);
        if (err != ESP_OK) {
            goto out;
        }
        *mv = (int)((float)adc_mv * config->input_divider_ratio + 0.5f);
        break;
    }
    case ANALOG_BACKEND_MUX_ADC: {
        err = analog_mux_get_config(config->mux_id, &mux_config);
        if (err != ESP_OK) {
            goto out;
        }
        err = analog_mux_select(config->mux_id, config->mux_channel);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to select mux %u channel %u: %s",
                     (unsigned)config->mux_id,
                     (unsigned)config->mux_channel,
                     esp_err_to_name(err));
            goto out;
        }
        err = adc_service_read_mv(mux_config.signal_adc_logical_channel, raw, &adc_mv);
        if (err != ESP_OK) {
            goto out;
        }
        *mv = (int)((float)adc_mv * config->input_divider_ratio + 0.5f);
        break;
    }
    case ANALOG_BACKEND_EXTERNAL_ADC:
        /* TODO: add external ADC driver integration. */
        ESP_LOGW(TAG, "external ADC backend is not implemented yet");
        err = ESP_ERR_NOT_SUPPORTED;
        break;
    default:
        err = ESP_ERR_INVALID_STATE;
        break;
    }

out:
    unlock_backend();
    return err;
}

esp_err_t analog_backend_read_mux_channel_mv(uint8_t mux_id, uint8_t mux_channel, int *raw, int *mv)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "analog backend is not initialized");
    ESP_RETURN_ON_FALSE(raw != NULL, ESP_ERR_INVALID_ARG, TAG, "raw output is required");
    ESP_RETURN_ON_FALSE(mv != NULL, ESP_ERR_INVALID_ARG, TAG, "millivolt output is required");

    ESP_RETURN_ON_ERROR(lock_backend(), TAG, "failed to lock analog backend for mux diagnostic read");

    analog_mux_config_t mux_config = {0};
    esp_err_t err = analog_mux_get_config(mux_id, &mux_config);
    if (err != ESP_OK) {
        goto out;
    }
    err = analog_mux_select(mux_id, mux_channel);
    if (err != ESP_OK) {
        goto out;
    }
    err = adc_service_read_mv(mux_config.signal_adc_logical_channel, raw, mv);

out:
    unlock_backend();
    return err;
}

esp_err_t analog_backend_select_mux_channel(uint8_t mux_id, uint8_t mux_channel)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "analog backend is not initialized");

    ESP_RETURN_ON_ERROR(lock_backend(), TAG, "failed to lock analog backend for mux selection");
    const esp_err_t err = analog_mux_select(mux_id, mux_channel);
    unlock_backend();
    return err;
}

esp_err_t analog_backend_get_source_config(uint8_t source_id, analog_source_config_t *out)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "analog backend is not initialized");
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "source config output is required");
    ESP_RETURN_ON_FALSE(source_id < ANALOG_BACKEND_MAX_SOURCES, ESP_ERR_INVALID_ARG, TAG,
                        "source id %u is out of range", (unsigned)source_id);

    ESP_RETURN_ON_ERROR(lock_backend(), TAG, "failed to lock analog backend for source lookup");
    esp_err_t err = ESP_OK;
    if (!s_sources[source_id].registered) {
        err = ESP_ERR_NOT_FOUND;
    } else {
        *out = s_sources[source_id].config;
    }
    unlock_backend();
    return err;
}

bool analog_backend_source_is_registered(uint8_t source_id)
{
    if (!s_initialized || source_id >= ANALOG_BACKEND_MAX_SOURCES || lock_backend() != ESP_OK) {
        return false;
    }

    const bool registered = s_sources[source_id].registered;
    unlock_backend();
    return registered;
}
