#include "analog_mux.h"

#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define ANALOG_MUX_MAX_MUXES 1
#define ANALOG_MUX_MAX_CHANNEL 15
#define ANALOG_MUX_MIN_SETTLE_US 50U
#define ANALOG_MUX_MAX_SETTLE_US 5000U
#define ANALOG_MUX_LOCK_TIMEOUT pdMS_TO_TICKS(1000)

static const char *TAG = "analog_mux";

typedef struct {
    bool registered;
    analog_mux_config_t config;
} analog_mux_entry_t;

static bool s_initialized;
static SemaphoreHandle_t s_lock;
static analog_mux_entry_t s_muxes[ANALOG_MUX_MAX_MUXES];

static esp_err_t lock_mux(void)
{
    ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_INVALID_STATE, TAG, "analog mux lock is not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_lock, ANALOG_MUX_LOCK_TIMEOUT) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "timed out waiting for analog mux lock");
    return ESP_OK;
}

static void unlock_mux(void)
{
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

static bool is_valid_gpio_or_unused(int gpio)
{
    return gpio == ANALOG_MUX_GPIO_UNUSED || (gpio >= 0 && gpio < GPIO_NUM_MAX);
}

static bool is_valid_output_gpio(int gpio)
{
    return gpio >= 0 && gpio < GPIO_NUM_MAX;
}

static bool config_has_valid_select_gpios(const analog_mux_config_t *config)
{
    return is_valid_output_gpio(config->gpio_s0) &&
           is_valid_output_gpio(config->gpio_s1) &&
           is_valid_output_gpio(config->gpio_s2) &&
           is_valid_output_gpio(config->gpio_s3);
}

static bool config_has_duplicate_gpios(const analog_mux_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    const int gpios[] = {
        config->gpio_s0,
        config->gpio_s1,
        config->gpio_s2,
        config->gpio_s3,
        config->gpio_en,
    };
    const size_t gpio_count = sizeof(gpios) / sizeof(gpios[0]);
    for (size_t i = 0; i < gpio_count; ++i) {
        if (gpios[i] == ANALOG_MUX_GPIO_UNUSED) {
            continue;
        }
        for (size_t j = i + 1; j < gpio_count; ++j) {
            if (gpios[i] == gpios[j]) {
                return true;
            }
        }
    }
    return false;
}

static bool config_is_ready(const analog_mux_config_t *config)
{
    return config != NULL &&
           config->enabled &&
           config_has_valid_select_gpios(config) &&
           !config_has_duplicate_gpios(config) &&
           is_valid_gpio_or_unused(config->gpio_en);
}

static void add_gpio_to_mask(uint64_t *mask, int gpio)
{
    if (is_valid_output_gpio(gpio)) {
        *mask |= (1ULL << (uint32_t)gpio);
    }
}

static esp_err_t configure_output_gpios(const analog_mux_config_t *config)
{
    uint64_t pin_mask = 0;
    add_gpio_to_mask(&pin_mask, config->gpio_s0);
    add_gpio_to_mask(&pin_mask, config->gpio_s1);
    add_gpio_to_mask(&pin_mask, config->gpio_s2);
    add_gpio_to_mask(&pin_mask, config->gpio_s3);
    add_gpio_to_mask(&pin_mask, config->gpio_en);

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "failed to configure mux GPIOs");

    if (config->gpio_en != ANALOG_MUX_GPIO_UNUSED) {
        const uint32_t enabled_level = config->en_active_low ? 0U : 1U;
        ESP_RETURN_ON_ERROR(gpio_set_level((gpio_num_t)config->gpio_en, enabled_level),
                            TAG, "failed to set mux EN level");
    }

    return ESP_OK;
}

esp_err_t analog_mux_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create analog mux lock");
    }

    ESP_RETURN_ON_ERROR(lock_mux(), TAG, "failed to lock analog mux during init");
    memset(s_muxes, 0, sizeof(s_muxes));
    s_initialized = true;
    unlock_mux();

    ESP_LOGI(TAG, "analog mux manager initialized");
    return ESP_OK;
}

esp_err_t analog_mux_register(const analog_mux_config_t *config)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "analog mux is not initialized");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "mux config is required");
    ESP_RETURN_ON_FALSE(config->mux_id < ANALOG_MUX_MAX_MUXES, ESP_ERR_NOT_SUPPORTED, TAG,
                        "only mux_id 0 is supported for now");
    ESP_RETURN_ON_FALSE(config->settle_time_us >= ANALOG_MUX_MIN_SETTLE_US &&
                            config->settle_time_us <= ANALOG_MUX_MAX_SETTLE_US,
                        ESP_ERR_INVALID_ARG, TAG, "settle_time_us must be 50..5000");
    ESP_RETURN_ON_FALSE(is_valid_gpio_or_unused(config->gpio_s0) &&
                            is_valid_gpio_or_unused(config->gpio_s1) &&
                            is_valid_gpio_or_unused(config->gpio_s2) &&
                            is_valid_gpio_or_unused(config->gpio_s3) &&
                            is_valid_gpio_or_unused(config->gpio_en),
                        ESP_ERR_INVALID_ARG, TAG, "mux GPIO out of range");
    ESP_RETURN_ON_FALSE(!config_has_duplicate_gpios(config), ESP_ERR_INVALID_ARG, TAG,
                        "mux GPIO assignments must be unique");

    ESP_RETURN_ON_ERROR(lock_mux(), TAG, "failed to lock analog mux for registration");
    esp_err_t err = ESP_OK;
    if (config->enabled) {
        if (!config_has_valid_select_gpios(config)) {
            err = ESP_ERR_INVALID_ARG;
            ESP_LOGE(TAG, "enabled mux requires valid S0/S1/S2/S3 GPIOs");
            goto out;
        }
        err = configure_output_gpios(config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to configure mux GPIOs: %s", esp_err_to_name(err));
            goto out;
        }
    }

    s_muxes[config->mux_id].config = *config;
    s_muxes[config->mux_id].registered = true;

out:
    unlock_mux();
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "registered mux %u enabled=%s", (unsigned)config->mux_id,
             config->enabled ? "true" : "false");
    return ESP_OK;
}

esp_err_t analog_mux_get_config(uint8_t mux_id, analog_mux_config_t *out)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "analog mux is not initialized");
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "mux config output is required");
    ESP_RETURN_ON_FALSE(mux_id < ANALOG_MUX_MAX_MUXES, ESP_ERR_NOT_SUPPORTED, TAG,
                        "only mux_id 0 is supported for now");

    ESP_RETURN_ON_ERROR(lock_mux(), TAG, "failed to lock analog mux for config lookup");
    esp_err_t err = ESP_OK;
    if (!s_muxes[mux_id].registered) {
        err = ESP_ERR_NOT_FOUND;
        ESP_LOGE(TAG, "mux %u is not registered", (unsigned)mux_id);
    } else {
        *out = s_muxes[mux_id].config;
    }
    unlock_mux();
    return err;
}

esp_err_t analog_mux_is_ready(uint8_t mux_id, bool *ready)
{
    ESP_RETURN_ON_FALSE(ready != NULL, ESP_ERR_INVALID_ARG, TAG, "ready output is required");
    *ready = false;
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "analog mux is not initialized");
    ESP_RETURN_ON_FALSE(mux_id < ANALOG_MUX_MAX_MUXES, ESP_ERR_NOT_SUPPORTED, TAG,
                        "only mux_id 0 is supported for now");

    ESP_RETURN_ON_ERROR(lock_mux(), TAG, "failed to lock analog mux for ready check");
    esp_err_t err = ESP_OK;
    if (!s_muxes[mux_id].registered) {
        err = ESP_ERR_NOT_FOUND;
        ESP_LOGE(TAG, "mux %u is not registered", (unsigned)mux_id);
    } else {
        *ready = config_is_ready(&s_muxes[mux_id].config);
    }
    unlock_mux();
    return err;
}

esp_err_t analog_mux_select(uint8_t mux_id, uint8_t channel)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "analog mux is not initialized");
    ESP_RETURN_ON_FALSE(mux_id < ANALOG_MUX_MAX_MUXES, ESP_ERR_NOT_SUPPORTED, TAG,
                        "only mux_id 0 is supported for now");
    ESP_RETURN_ON_FALSE(channel <= ANALOG_MUX_MAX_CHANNEL, ESP_ERR_INVALID_ARG, TAG,
                        "mux channel %u is out of range", (unsigned)channel);

    ESP_RETURN_ON_ERROR(lock_mux(), TAG, "failed to lock analog mux for channel selection");
    esp_err_t err = ESP_OK;
    const analog_mux_config_t *config = &s_muxes[mux_id].config;
    if (!s_muxes[mux_id].registered) {
        err = ESP_ERR_NOT_FOUND;
        ESP_LOGE(TAG, "mux %u is not registered", (unsigned)mux_id);
        goto out;
    }
    if (!config_is_ready(config)) {
        err = ESP_ERR_INVALID_STATE;
        ESP_LOGE(TAG, "mux %u is disabled or not fully configured", (unsigned)mux_id);
        goto out;
    }

    err = gpio_set_level((gpio_num_t)config->gpio_s0, (channel >> 0) & 0x01);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to set mux S0: %s", esp_err_to_name(err));
        goto out;
    }
    err = gpio_set_level((gpio_num_t)config->gpio_s1, (channel >> 1) & 0x01);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to set mux S1: %s", esp_err_to_name(err));
        goto out;
    }
    err = gpio_set_level((gpio_num_t)config->gpio_s2, (channel >> 2) & 0x01);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to set mux S2: %s", esp_err_to_name(err));
        goto out;
    }
    err = gpio_set_level((gpio_num_t)config->gpio_s3, (channel >> 3) & 0x01);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to set mux S3: %s", esp_err_to_name(err));
        goto out;
    }

    esp_rom_delay_us(config->settle_time_us);

out:
    unlock_mux();
    return err;
}
