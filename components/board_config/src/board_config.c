#include "board_config.h"

#include <stddef.h>
#include <stdint.h>

#include "adc_service.h"
#include "analog_backend.h"
#include "analog_mux.h"
#include "esp_check.h"
#include "esp_log.h"
#include "hal/adc_types.h"
#include "mq_sensor.h"
#include "mq_runtime_config.h"

#define BOARD_CONFIG_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#define BOARD_CONFIG_WARMUP_24H_SECONDS 86400U
#define BOARD_CONFIG_WARMUP_48H_SECONDS 172800U
#define BOARD_CONFIG_KIT_DIVIDER_RATIO 2.0f

static const char *TAG = "board_config";

typedef struct {
    uint8_t logical_channel;
    int adc_unit;
    int adc_channel;
    int attenuation;
    int bitwidth;
} board_adc_channel_t;

static const board_adc_channel_t k_adc_channels[] = {
    {0, ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12, ADC_BITWIDTH_DEFAULT},
};

static const analog_mux_config_t k_mux0_default = {
    .mux_id = 0,
    .enabled = true,
    .signal_adc_logical_channel = 0,
    .gpio_s0 = 10,
    .gpio_s1 = 11,
    .gpio_s2 = 12,
    .gpio_s3 = 22,
    .gpio_en = ANALOG_MUX_GPIO_UNUSED,
    .en_active_low = true,
    .settle_time_us = 500,
};

static analog_mux_config_t s_effective_mux0;

static const analog_source_config_t k_analog_sources[] = {
    {0, ANALOG_BACKEND_MUX_ADC, 0, BOARD_CONFIG_KIT_DIVIDER_RATIO, 0, 0},
    {1, ANALOG_BACKEND_MUX_ADC, 0, BOARD_CONFIG_KIT_DIVIDER_RATIO, 0, 1},
    {2, ANALOG_BACKEND_MUX_ADC, 0, BOARD_CONFIG_KIT_DIVIDER_RATIO, 0, 2},
    {3, ANALOG_BACKEND_MUX_ADC, 0, BOARD_CONFIG_KIT_DIVIDER_RATIO, 0, 3},
    {4, ANALOG_BACKEND_MUX_ADC, 0, BOARD_CONFIG_KIT_DIVIDER_RATIO, 0, 4},
    {5, ANALOG_BACKEND_MUX_ADC, 0, BOARD_CONFIG_KIT_DIVIDER_RATIO, 0, 5},
    {6, ANALOG_BACKEND_MUX_ADC, 0, BOARD_CONFIG_KIT_DIVIDER_RATIO, 0, 6},
    {7, ANALOG_BACKEND_MUX_ADC, 0, BOARD_CONFIG_KIT_DIVIDER_RATIO, 0, 7},
    {8, ANALOG_BACKEND_MUX_ADC, 0, BOARD_CONFIG_KIT_DIVIDER_RATIO, 0, 8},
};

static analog_source_config_t s_effective_sources[BOARD_CONFIG_ARRAY_SIZE(k_analog_sources)];

static const mq_sensor_config_t k_mq_sensors[] = {
    {0, MQ_SENSOR_MQ135, "MQ-135", 0, 5000, BOARD_CONFIG_WARMUP_48H_SECONDS, true, true,  0.70f, 0.50f},
    {1, MQ_SENSOR_MQ2,   "MQ-2",   1, 5000, BOARD_CONFIG_WARMUP_48H_SECONDS, true, true,  0.70f, 0.50f},
    {2, MQ_SENSOR_MQ3,   "MQ-3",   2, 5000, BOARD_CONFIG_WARMUP_24H_SECONDS, true, true,  0.70f, 0.50f},
    {3, MQ_SENSOR_MQ4,   "MQ-4",   3, 5000, BOARD_CONFIG_WARMUP_48H_SECONDS, true, true,  0.70f, 0.50f},
    {4, MQ_SENSOR_MQ5,   "MQ-5",   4, 5000, BOARD_CONFIG_WARMUP_24H_SECONDS, true, true,  0.70f, 0.50f},
    {5, MQ_SENSOR_MQ6,   "MQ-6",   5, 5000, BOARD_CONFIG_WARMUP_48H_SECONDS, true, true,  0.70f, 0.50f},
    {6, MQ_SENSOR_MQ7,   "MQ-7",   6, 5000, BOARD_CONFIG_WARMUP_48H_SECONDS, true, false, 0.0f,  0.0f},
    {7, MQ_SENSOR_MQ8,   "MQ-8",   7, 5000, BOARD_CONFIG_WARMUP_48H_SECONDS, true, true,  0.70f, 0.50f},
    {8, MQ_SENSOR_MQ9,   "MQ-9",   8, 5000, BOARD_CONFIG_WARMUP_48H_SECONDS, true, false, 0.0f,  0.0f},
};

static mq_sensor_config_t s_effective_sensors[BOARD_CONFIG_ARRAY_SIZE(k_mq_sensors)];

esp_err_t board_config_register_internal_adc_channels(void)
{
    for (size_t i = 0; i < BOARD_CONFIG_ARRAY_SIZE(k_adc_channels); ++i) {
        const board_adc_channel_t *board_channel = &k_adc_channels[i];
        const adc_service_channel_config_t config = {
            .adc_unit = board_channel->adc_unit,
            .adc_channel = board_channel->adc_channel,
            .attenuation = board_channel->attenuation,
            .bitwidth = board_channel->bitwidth,
        };

        ESP_RETURN_ON_ERROR(adc_service_register_channel(board_channel->logical_channel, &config),
                            TAG,
                            "Failed to register ADC logical channel %u",
                            board_channel->logical_channel);
    }

    ESP_LOGI(TAG, "Registered %u internal ADC channels",
             (unsigned int)BOARD_CONFIG_ARRAY_SIZE(k_adc_channels));
    return ESP_OK;
}

esp_err_t board_config_init_muxes(void)
{
    ESP_RETURN_ON_ERROR(mq_runtime_config_apply_mux(k_mux0_default.mux_id,
                                                    &k_mux0_default,
                                                    &s_effective_mux0),
                        TAG,
                        "Failed to apply mux override");
    ESP_RETURN_ON_ERROR(analog_mux_register(&s_effective_mux0), TAG, "Failed to register mux 0");
    return ESP_OK;
}

esp_err_t board_config_register_analog_sources(void)
{
    for (size_t i = 0; i < BOARD_CONFIG_ARRAY_SIZE(k_analog_sources); ++i) {
        ESP_RETURN_ON_ERROR(mq_runtime_config_apply_source(k_analog_sources[i].source_id,
                                                           &k_analog_sources[i],
                                                           &s_effective_sources[i]),
                            TAG,
                            "Failed to apply analog source override %u",
                            k_analog_sources[i].source_id);
        ESP_RETURN_ON_ERROR(analog_backend_register_source(&s_effective_sources[i]),
                            TAG,
                            "Failed to register analog source %u",
                            s_effective_sources[i].source_id);
    }

    ESP_LOGI(TAG, "Registered %u analog sources",
             (unsigned int)BOARD_CONFIG_ARRAY_SIZE(k_analog_sources));
    return ESP_OK;
}

esp_err_t board_config_init_mq_sensors(void)
{
    for (size_t i = 0; i < BOARD_CONFIG_ARRAY_SIZE(k_mq_sensors); ++i) {
        ESP_RETURN_ON_ERROR(mq_runtime_config_apply_sensor(k_mq_sensors[i].id,
                                                           &k_mq_sensors[i],
                                                           &s_effective_sensors[i]),
                            TAG,
                            "Failed to apply MQ sensor override %u",
                            k_mq_sensors[i].id);
    }

    ESP_RETURN_ON_ERROR(mq_sensor_init(s_effective_sensors, BOARD_CONFIG_ARRAY_SIZE(s_effective_sensors)),
                        TAG,
                        "Failed to initialize MQ sensor table");

    ESP_LOGI(TAG, "Initialized %u MQ sensor configs",
             (unsigned int)BOARD_CONFIG_ARRAY_SIZE(k_mq_sensors));
    return ESP_OK;
}

size_t board_config_internal_adc_channel_count(void)
{
    return BOARD_CONFIG_ARRAY_SIZE(k_adc_channels);
}

size_t board_config_analog_source_count(void)
{
    return BOARD_CONFIG_ARRAY_SIZE(k_analog_sources);
}

size_t board_config_mq_sensor_count(void)
{
    return BOARD_CONFIG_ARRAY_SIZE(k_mq_sensors);
}

esp_err_t board_config_get_default_sensor_config_by_index(size_t index, mq_sensor_config_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "sensor config output is required");
    ESP_RETURN_ON_FALSE(index < BOARD_CONFIG_ARRAY_SIZE(k_mq_sensors), ESP_ERR_NOT_FOUND, TAG,
                        "sensor index %u is out of range", (unsigned)index);
    *out = k_mq_sensors[index];
    return ESP_OK;
}

esp_err_t board_config_get_effective_sensor_config_by_index(size_t index, mq_sensor_config_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "sensor config output is required");
    ESP_RETURN_ON_FALSE(index < BOARD_CONFIG_ARRAY_SIZE(s_effective_sensors), ESP_ERR_NOT_FOUND, TAG,
                        "sensor index %u is out of range", (unsigned)index);
    *out = s_effective_sensors[index];
    return ESP_OK;
}

esp_err_t board_config_get_default_source_config_by_index(size_t index, analog_source_config_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "source config output is required");
    ESP_RETURN_ON_FALSE(index < BOARD_CONFIG_ARRAY_SIZE(k_analog_sources), ESP_ERR_NOT_FOUND, TAG,
                        "source index %u is out of range", (unsigned)index);
    *out = k_analog_sources[index];
    return ESP_OK;
}

esp_err_t board_config_get_effective_source_config_by_index(size_t index, analog_source_config_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "source config output is required");
    ESP_RETURN_ON_FALSE(index < BOARD_CONFIG_ARRAY_SIZE(s_effective_sources), ESP_ERR_NOT_FOUND, TAG,
                        "source index %u is out of range", (unsigned)index);
    *out = s_effective_sources[index];
    return ESP_OK;
}

bool board_config_sensor_can_be_enabled(uint8_t sensor_id)
{
    for (size_t i = 0; i < BOARD_CONFIG_ARRAY_SIZE(k_mq_sensors); ++i) {
        if (k_mq_sensors[i].id == sensor_id) {
            const uint8_t source_id = s_effective_sensors[i].analog_source_id;
            for (size_t j = 0; j < BOARD_CONFIG_ARRAY_SIZE(s_effective_sources); ++j) {
                if (s_effective_sources[j].source_id == source_id) {
                    if (s_effective_sources[j].input_divider_ratio < 0.1f) {
                        return false;
                    }
                    if (s_effective_sources[j].type == ANALOG_BACKEND_MUX_ADC) {
                        return s_effective_mux0.enabled;
                    }
                    return false;
                }
            }
            return false;
        }
    }
    return false;
}

esp_err_t board_config_get_default_mux_config(uint8_t mux_id, analog_mux_config_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "mux config output is required");
    ESP_RETURN_ON_FALSE(mux_id == k_mux0_default.mux_id, ESP_ERR_NOT_FOUND, TAG,
                        "mux id %u is not configured", (unsigned)mux_id);
    *out = k_mux0_default;
    return ESP_OK;
}

esp_err_t board_config_get_effective_mux_config(uint8_t mux_id, analog_mux_config_t *out)
{
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "mux config output is required");
    ESP_RETURN_ON_FALSE(mux_id == s_effective_mux0.mux_id, ESP_ERR_NOT_FOUND, TAG,
                        "mux id %u is not configured", (unsigned)mux_id);
    *out = s_effective_mux0;
    return ESP_OK;
}

bool board_config_mux_is_enabled(uint8_t mux_id)
{
    return mux_id == s_effective_mux0.mux_id && s_effective_mux0.enabled;
}
