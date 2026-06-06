#include "matter_manager.h"

#include <inttypes.h>

#include <esp_log.h>
#include <esp_matter.h>
#include <esp_matter_console.h>
#include <sdkconfig.h>

static const char *TAG = "matter_manager";

esp_err_t matter_create_air_quality_endpoint(esp_matter::node_t *matter_node, uint16_t *endpoint_id);

static void matter_event_callback(const ChipDeviceEvent *event, intptr_t arg)
{
    (void)arg;

    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kThreadConnectivityChange:
        ESP_LOGI(TAG, "Thread connectivity changed");
        break;
    case chip::DeviceLayer::DeviceEventType::kThreadStateChange:
        ESP_LOGI(TAG, "Thread state changed");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kServerReady:
        ESP_LOGI(TAG, "Matter server ready");
        break;
    default:
        ESP_LOGI(TAG, "Matter event type: %d", event->Type);
        break;
    }
}

static esp_err_t identification_callback(esp_matter::identification::callback_type_t type,
                                         uint16_t endpoint_id,
                                         uint8_t effect_id,
                                         uint8_t effect_variant,
                                         void *priv_data)
{
    (void)priv_data;
    ESP_LOGI(TAG, "Identification callback: type=%d endpoint=%u effect=%u variant=%u",
             (int)type,
             (unsigned)endpoint_id,
             (unsigned)effect_id,
             (unsigned)effect_variant);
    return ESP_OK;
}

static esp_err_t attribute_update_callback(esp_matter::attribute::callback_type_t type,
                                           uint16_t endpoint_id,
                                           uint32_t cluster_id,
                                           uint32_t attribute_id,
                                           esp_matter_attr_val_t *val,
                                           void *priv_data)
{
    (void)val;
    (void)priv_data;
    ESP_LOGI(TAG, "Attribute callback: type=%d endpoint=%u cluster=0x%08" PRIx32 " attribute=0x%08" PRIx32,
             (int)type,
             (unsigned)endpoint_id,
             cluster_id,
             attribute_id);
    return ESP_OK;
}

esp_err_t matter_init(uint16_t *air_quality_endpoint_id)
{
    if (air_quality_endpoint_id != nullptr) {
        *air_quality_endpoint_id = 0;
    }

    esp_matter::node::config_t node_config;
    esp_matter::node_t *matter_node = esp_matter::node::create(&node_config,
                                                               attribute_update_callback,
                                                               identification_callback);
    if (matter_node == nullptr) {
        ESP_LOGE(TAG, "failed to create Matter node");
        return ESP_FAIL;
    }

    esp_err_t err = matter_create_air_quality_endpoint(matter_node, air_quality_endpoint_id);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_matter::start(matter_event_callback);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to start Matter: %s", esp_err_to_name(err));
        return err;
    }

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
#if CONFIG_OPENTHREAD_CLI
    console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif

    ESP_LOGI(TAG, "Matter initialized");
    return ESP_OK;
}
