#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "adc_service.h"
#include "air_quality_service.h"
#include "analog_backend.h"
#include "analog_mux.h"
#include "board_config.h"
#include "matter_manager.h"
#include "mq_calibration_nvs.h"
#include "mq_console.h"
#include "mq_runtime_config.h"
#include "mq_sensor.h"

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

static const char *TAG = "mq_air_app";
static constexpr TickType_t kStatusDelay = pdMS_TO_TICKS(10000);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#define ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG() \
    {                                         \
        .radio_mode = RADIO_MODE_NATIVE,      \
    }

#define ESP_OPENTHREAD_DEFAULT_HOST_CONFIG()          \
    {                                                 \
        .host_connection_mode = HOST_CONNECTION_MODE_NONE, \
    }

#define ESP_OPENTHREAD_DEFAULT_PORT_CONFIG()                                            \
    {                                                                                   \
        .storage_partition_name = "nvs", .netif_queue_size = 10, .task_queue_size = 10, \
    }
#endif

static esp_err_t init_nvs()
{
    ESP_LOGI(TAG, "Initializing NVS");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "Erasing NVS partition");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

static bool app_get_source_config(uint8_t source_id, analog_source_config_t *out)
{
    const size_t source_count = board_config_analog_source_count();
    for (size_t i = 0; i < source_count; ++i) {
        analog_source_config_t source = {};
        if (board_config_get_effective_source_config_by_index(i, &source) == ESP_OK &&
            source.source_id == source_id) {
            if (out != nullptr) {
                *out = source;
            }
            return true;
        }
    }
    return false;
}

static bool app_mux_ready(uint8_t mux_id)
{
    bool ready = false;
    return analog_mux_is_ready(mux_id, &ready) == ESP_OK && ready;
}

static bool app_get_sensor_config(uint8_t sensor_id, mq_sensor_config_t *out)
{
    const size_t sensor_count = board_config_mq_sensor_count();
    for (size_t i = 0; i < sensor_count; ++i) {
        mq_sensor_config_t sensor = {};
        if (board_config_get_effective_sensor_config_by_index(i, &sensor) == ESP_OK &&
            sensor.id == sensor_id) {
            if (out != nullptr) {
                *out = sensor;
            }
            return true;
        }
    }
    return false;
}

static void log_sensor_boot_warnings()
{
    const size_t source_count = board_config_analog_source_count();
    for (size_t i = 0; i < source_count; ++i) {
        analog_source_config_t source = {};
        if (board_config_get_effective_source_config_by_index(i, &source) != ESP_OK) {
            continue;
        }
        if (source.type == ANALOG_BACKEND_MUX_ADC && !board_config_mux_is_enabled(source.mux_id)) {
            ESP_LOGW(TAG, "Analog source %u uses mux %u while mux is disabled",
                     static_cast<unsigned>(source.source_id), static_cast<unsigned>(source.mux_id));
        }
    }

    const size_t sensor_count = board_config_mq_sensor_count();
    for (size_t i = 0; i < sensor_count; ++i) {
        mq_sensor_config_t sensor = {};
        if (board_config_get_effective_sensor_config_by_index(i, &sensor) != ESP_OK || !sensor.enabled) {
            continue;
        }

        analog_source_config_t source = {};
        if (!app_get_source_config(sensor.analog_source_id, &source)) {
            ESP_LOGW(TAG, "Enabled sensor %u uses missing analog source %u",
                     static_cast<unsigned>(sensor.id), static_cast<unsigned>(sensor.analog_source_id));
        } else if (source.type == ANALOG_BACKEND_MUX_ADC) {
            if (!board_config_mux_is_enabled(source.mux_id)) {
                ESP_LOGW(TAG, "Enabled sensor %u uses mux source %u while mux %u is disabled",
                         static_cast<unsigned>(sensor.id),
                         static_cast<unsigned>(source.source_id),
                         static_cast<unsigned>(source.mux_id));
            } else if (!app_mux_ready(source.mux_id)) {
                ESP_LOGW(TAG, "Enabled sensor %u uses mux source %u while mux %u is not ready",
                         static_cast<unsigned>(sensor.id),
                         static_cast<unsigned>(source.source_id),
                         static_cast<unsigned>(source.mux_id));
            }
        }

        if (sensor.type == MQ_SENSOR_MQ7 || sensor.type == MQ_SENSOR_MQ9) {
            ESP_LOGW(TAG, "%s is raw-diagnostic only in this firmware; do not use it for calibrated AQ/Matter",
                     mq_sensor_type_to_string(sensor.type));
        }

        mq_calibration_record_t calibration = {};
        const esp_err_t cal_err = mq_calibration_nvs_load(sensor.id, &calibration);
        if (cal_err != ESP_OK || !calibration.valid || calibration.r0_ohms <= 0.0f) {
            ESP_LOGW(TAG, "Enabled sensor %u has no valid calibration", static_cast<unsigned>(sensor.id));
        }
    }
}

static void log_aq_boot_warnings(const air_quality_service_config_t &aq_config)
{
    if (aq_config.primary_sensor_id > 8) {
        ESP_LOGW(TAG, "AQ primary sensor %u is outside configured sensor range 0..8",
                 static_cast<unsigned>(aq_config.primary_sensor_id));
    }

    mq_sensor_config_t primary = {};
    if (!app_get_sensor_config(aq_config.primary_sensor_id, &primary)) {
        ESP_LOGW(TAG, "AQ primary sensor %u does not exist", static_cast<unsigned>(aq_config.primary_sensor_id));
        return;
    }
    if (!primary.enabled) {
        ESP_LOGW(TAG, "AQ primary sensor %u is not enabled", static_cast<unsigned>(aq_config.primary_sensor_id));
    }

    analog_source_config_t source = {};
    if (app_get_source_config(primary.analog_source_id, &source) && source.type == ANALOG_BACKEND_MUX_ADC) {
        if (!board_config_mux_is_enabled(source.mux_id)) {
            ESP_LOGW(TAG, "AQ primary sensor %u uses mux source %u while mux %u is disabled",
                     static_cast<unsigned>(primary.id),
                     static_cast<unsigned>(source.source_id),
                     static_cast<unsigned>(source.mux_id));
        } else if (!app_mux_ready(source.mux_id)) {
            ESP_LOGW(TAG, "AQ primary sensor %u uses mux source %u while mux %u is not ready",
                     static_cast<unsigned>(primary.id),
                     static_cast<unsigned>(source.source_id),
                     static_cast<unsigned>(source.mux_id));
        }
    }
}

extern "C" void app_main()
{
    ESP_ERROR_CHECK(init_nvs());

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    ESP_ERROR_CHECK(mq_calibration_nvs_init());
    ESP_ERROR_CHECK(mq_runtime_config_init());
    ESP_ERROR_CHECK(adc_service_init());
    ESP_ERROR_CHECK(analog_backend_init());
    ESP_ERROR_CHECK(analog_mux_init());
    ESP_ERROR_CHECK(board_config_register_internal_adc_channels());
    ESP_ERROR_CHECK(board_config_init_muxes());
    ESP_ERROR_CHECK(board_config_register_analog_sources());
    ESP_ERROR_CHECK(board_config_init_mq_sensors());
    log_sensor_boot_warnings();

    uint16_t air_quality_endpoint_id = 0;
    ESP_ERROR_CHECK(matter_init(&air_quality_endpoint_id));
    ESP_ERROR_CHECK(mq_console_register_commands());

    air_quality_service_config_t aq_defaults = {};
    aq_defaults.matter_endpoint_id = air_quality_endpoint_id;
    aq_defaults.primary_sensor_id = 0;
    aq_defaults.sample_interval_ms = 5000;
    aq_defaults.stale_after_ms = 20000;
    aq_defaults.ema_alpha = 0.25f;
    aq_defaults.publish_to_matter = true;

    air_quality_service_config_t aq_config = {};
    ESP_ERROR_CHECK(mq_runtime_config_apply_air_quality(&aq_defaults, &aq_config));
    log_aq_boot_warnings(aq_config);
    ESP_ERROR_CHECK(air_quality_service_init(&aq_config));
    ESP_ERROR_CHECK(air_quality_service_start());

    while (true) {
        air_quality_service_status_t aq_status = {};
        const esp_err_t aq_status_err = air_quality_service_get_status(&aq_status);
        ESP_LOGI(TAG,
                 "MQ air quality runtime: sensors=%u, endpoint=%u, aq_running=%s, aq_level=%s, aq_status=%s",
                 static_cast<unsigned>(mq_sensor_count()),
                 static_cast<unsigned>(air_quality_endpoint_id),
                 aq_status_err == ESP_OK && aq_status.running ? "true" : "false",
                 aq_status_err == ESP_OK ? air_quality_service_level_to_string(aq_status.current_level) : "UNKNOWN",
                 esp_err_to_name(aq_status_err));
        vTaskDelay(kStatusDelay);
    }
}
