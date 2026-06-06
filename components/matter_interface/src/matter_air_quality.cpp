#include "matter_air_quality.h"

#include <esp_check.h>
#include <esp_log.h>
#include <esp_matter.h>

static const char *TAG = "matter_air_quality";

esp_err_t matter_create_air_quality_endpoint(esp_matter::node_t *matter_node, uint16_t *endpoint_id)
{
    ESP_RETURN_ON_FALSE(matter_node != nullptr, ESP_ERR_INVALID_ARG, TAG, "Matter node is required");
    ESP_RETURN_ON_FALSE(endpoint_id != nullptr, ESP_ERR_INVALID_ARG, TAG, "endpoint id output is required");

    esp_matter::endpoint::air_quality_sensor::config_t endpoint_config;
    endpoint_config.air_quality.air_quality = static_cast<uint8_t>(MATTER_AIR_QUALITY_UNKNOWN);

    esp_matter::endpoint_t *endpoint = esp_matter::endpoint::air_quality_sensor::create(
        matter_node, &endpoint_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    ESP_RETURN_ON_FALSE(endpoint != nullptr, ESP_FAIL, TAG, "failed to create Air Quality endpoint");

    *endpoint_id = esp_matter::endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Air Quality endpoint created with id %u", (unsigned)*endpoint_id);
    return ESP_OK;
}

esp_err_t matter_update_air_quality(uint16_t endpoint_id, matter_air_quality_level_t level)
{
    ESP_RETURN_ON_FALSE(endpoint_id != 0, ESP_ERR_INVALID_ARG, TAG, "endpoint id is required");
    ESP_RETURN_ON_FALSE(level >= MATTER_AIR_QUALITY_UNKNOWN &&
                            level <= MATTER_AIR_QUALITY_EXTREMELY_POOR,
                        ESP_ERR_INVALID_ARG, TAG, "invalid air quality level %d", (int)level);

    esp_matter_attr_val_t value = esp_matter_enum8(static_cast<uint8_t>(level));
    return esp_matter::attribute::update(endpoint_id,
                                         chip::app::Clusters::AirQuality::Id,
                                         chip::app::Clusters::AirQuality::Attributes::AirQuality::Id,
                                         &value);
}
