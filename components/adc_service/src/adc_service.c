#include "adc_service.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hal/adc_types.h"
#include "soc/soc_caps.h"

#define ADC_SERVICE_MAX_CHANNELS 16
#define ADC_SERVICE_LOCK_TIMEOUT pdMS_TO_TICKS(1000)

static const char *TAG = "adc_service";

typedef struct {
    bool registered;
    adc_channel_t channel;
    adc_atten_t attenuation;
    adc_bitwidth_t bitwidth;
    adc_cali_handle_t calibration;
} adc_service_channel_t;

static bool s_initialized;
static adc_oneshot_unit_handle_t s_adc_unit;
static SemaphoreHandle_t s_lock;
static adc_service_channel_t s_channels[ADC_SERVICE_MAX_CHANNELS];

static esp_err_t adc_service_lock(void)
{
    ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_INVALID_STATE, TAG, "ADC service lock is not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, ADC_SERVICE_LOCK_TIMEOUT) == pdTRUE, ESP_ERR_TIMEOUT, TAG,
                        "timed out waiting for ADC service lock");
    return ESP_OK;
}

static void adc_service_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

static bool adc_service_channel_is_valid_for_h2_adc1(int channel)
{
    return channel >= ADC_CHANNEL_0 && channel < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1);
}

static void adc_service_delete_calibration(adc_cali_handle_t calibration)
{
    if (calibration != NULL) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        const esp_err_t err = adc_cali_delete_scheme_curve_fitting(calibration);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to delete old ADC calibration handle: %s", esp_err_to_name(err));
        }
#endif
    }
}

esp_err_t adc_service_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    s_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create ADC service lock");

    const adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = 0,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    const esp_err_t err = adc_oneshot_new_unit(&init_config, &s_adc_unit);
    if (err != ESP_OK) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        ESP_LOGE(TAG, "failed to create ADC oneshot unit: %s", esp_err_to_name(err));
        return err;
    }

    memset(s_channels, 0, sizeof(s_channels));
    s_initialized = true;
    ESP_LOGI(TAG, "ADC service initialized for ESP32-H2 ADC_UNIT_1 oneshot reads");
    return ESP_OK;
}

esp_err_t adc_service_register_channel(uint8_t logical_channel,
                                       const adc_service_channel_config_t *config)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "ADC service is not initialized");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "channel config is required");
    ESP_RETURN_ON_FALSE(logical_channel < ADC_SERVICE_MAX_CHANNELS, ESP_ERR_INVALID_ARG, TAG,
                        "logical channel %u is out of range", (unsigned)logical_channel);
    ESP_RETURN_ON_FALSE(config->adc_unit == ADC_UNIT_1, ESP_ERR_NOT_SUPPORTED, TAG,
                        "only ADC_UNIT_1 is supported on ESP32-H2");
    ESP_RETURN_ON_FALSE(adc_service_channel_is_valid_for_h2_adc1(config->adc_channel), ESP_ERR_INVALID_ARG, TAG,
                        "ADC1 channel %d is not usable on ESP32-H2", config->adc_channel);

    const adc_channel_t channel = (adc_channel_t)config->adc_channel;
    const adc_atten_t attenuation = (adc_atten_t)config->attenuation;
    const adc_bitwidth_t bitwidth = (adc_bitwidth_t)config->bitwidth;
    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = attenuation,
        .bitwidth = bitwidth,
    };

    ESP_RETURN_ON_ERROR(adc_service_lock(), TAG, "failed to lock ADC service");

    esp_err_t err = adc_oneshot_config_channel(s_adc_unit, channel, &channel_config);
    if (err != ESP_OK) {
        adc_service_unlock();
        ESP_LOGE(TAG, "failed to configure ADC logical channel %u: %s", (unsigned)logical_channel,
                 esp_err_to_name(err));
        return err;
    }

    adc_service_delete_calibration(s_channels[logical_channel].calibration);
    s_channels[logical_channel].calibration = NULL;

    adc_cali_handle_t calibration = NULL;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    const adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = ADC_UNIT_1,
        .chan = channel,
        .atten = attenuation,
        .bitwidth = bitwidth,
    };
    err = adc_cali_create_scheme_curve_fitting(&calibration_config, &calibration);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Created ADC calibration for logical channel %u", (unsigned)logical_channel);
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable for logical channel %u: %s", (unsigned)logical_channel,
                 esp_err_to_name(err));
        calibration = NULL;
    }
#else
    ESP_LOGW(TAG, "ADC curve-fitting calibration scheme is not supported by this target");
#endif

    s_channels[logical_channel].channel = channel;
    s_channels[logical_channel].attenuation = attenuation;
    s_channels[logical_channel].bitwidth = bitwidth;
    s_channels[logical_channel].calibration = calibration;
    s_channels[logical_channel].registered = true;
    ESP_LOGI(TAG, "Registered ADC logical channel %u as ADC1 channel %d", (unsigned)logical_channel,
             (int)channel);
    adc_service_unlock();
    return ESP_OK;
}

esp_err_t adc_service_read_raw(uint8_t logical_channel, int *raw)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "ADC service is not initialized");
    ESP_RETURN_ON_FALSE(raw != NULL, ESP_ERR_INVALID_ARG, TAG, "raw output is required");
    ESP_RETURN_ON_FALSE(logical_channel < ADC_SERVICE_MAX_CHANNELS, ESP_ERR_INVALID_ARG, TAG,
                        "logical channel %u is out of range", (unsigned)logical_channel);

    ESP_RETURN_ON_ERROR(adc_service_lock(), TAG, "failed to lock ADC service");
    if (!s_channels[logical_channel].registered) {
        adc_service_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    const esp_err_t err = adc_oneshot_read(s_adc_unit, s_channels[logical_channel].channel, raw);
    adc_service_unlock();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC raw read failed for logical channel %u: %s", (unsigned)logical_channel,
                 esp_err_to_name(err));
    }
    return err;
}

esp_err_t adc_service_read_mv(uint8_t logical_channel, int *raw, int *mv)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "ADC service is not initialized");
    ESP_RETURN_ON_FALSE(raw != NULL, ESP_ERR_INVALID_ARG, TAG, "raw output is required");
    ESP_RETURN_ON_FALSE(mv != NULL, ESP_ERR_INVALID_ARG, TAG, "millivolt output is required");
    ESP_RETURN_ON_FALSE(logical_channel < ADC_SERVICE_MAX_CHANNELS, ESP_ERR_INVALID_ARG, TAG,
                        "logical channel %u is out of range", (unsigned)logical_channel);

    ESP_RETURN_ON_ERROR(adc_service_lock(), TAG, "failed to lock ADC service");
    if (!s_channels[logical_channel].registered) {
        adc_service_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    int raw_value = 0;
    esp_err_t err = adc_oneshot_read(s_adc_unit, s_channels[logical_channel].channel, &raw_value);
    if (err != ESP_OK) {
        adc_service_unlock();
        ESP_LOGE(TAG, "ADC raw read failed for logical channel %u: %s", (unsigned)logical_channel,
                 esp_err_to_name(err));
        return err;
    }

    *raw = raw_value;
    if (s_channels[logical_channel].calibration == NULL) {
        adc_service_unlock();
        return ESP_ERR_NOT_SUPPORTED;
    }

    int voltage_mv = 0;
    err = adc_cali_raw_to_voltage(s_channels[logical_channel].calibration, raw_value, &voltage_mv);
    adc_service_unlock();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC calibration conversion failed for logical channel %u: %s",
                 (unsigned)logical_channel, esp_err_to_name(err));
        return err;
    }

    *mv = voltage_mv;
    return ESP_OK;
}
