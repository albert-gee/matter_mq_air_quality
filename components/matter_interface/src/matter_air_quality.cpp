#include "matter_air_quality.h"

#include <cstdlib>
#include <cstring>

#include <esp_check.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <platform/PlatformManager.h>

static const char *TAG = "matter_air_quality";

static constexpr uint32_t kMqDiagnosticsClusterId = 0x0000FC01;
static constexpr uint32_t kAttrSensorId = 0x00000000;
static constexpr uint32_t kAttrSensorType = 0x00000001;
static constexpr uint32_t kAttrEnabled = 0x00000002;
static constexpr uint32_t kAttrState = 0x00000003;
static constexpr uint32_t kAttrRawAdc = 0x00000004;
static constexpr uint32_t kAttrAdcMv = 0x00000005;
static constexpr uint32_t kAttrVrlMv = 0x00000006;
static constexpr uint32_t kAttrBaselineVrlMv = 0x00000007;
static constexpr uint32_t kAttrRsNormMilli = 0x00000008;
static constexpr uint32_t kAttrRsRatioMilli = 0x00000009;
static constexpr uint32_t kAttrBaselineValid = 0x0000000A;
static constexpr uint32_t kAttrThresholdState = 0x0000000B;
static constexpr uint32_t kAttrFaultBitmap = 0x0000000C;
static constexpr uint32_t kAttrLastAgeMs = 0x0000000D;

typedef struct {
    uint16_t endpoint_id;
    matter_air_quality_level_t level;
} air_quality_update_work_t;

typedef struct {
    uint16_t endpoint_id;
    matter_air_quality_diagnostics_t diagnostics;
} diagnostics_update_work_t;

static esp_err_t update_attribute(uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id,
                                  esp_matter_attr_val_t *value)
{
    return esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, value);
}

static void air_quality_update_work(intptr_t arg)
{
    auto *work = reinterpret_cast<air_quality_update_work_t *>(arg);
    esp_matter_attr_val_t value = esp_matter_enum8(static_cast<uint8_t>(work->level));
    const esp_err_t err = update_attribute(work->endpoint_id,
                                           chip::app::Clusters::AirQuality::Id,
                                           chip::app::Clusters::AirQuality::Attributes::AirQuality::Id,
                                           &value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Air Quality Matter update failed: %s", esp_err_to_name(err));
    }
    free(work);
}

static void diagnostics_update_work(intptr_t arg)
{
    auto *work = reinterpret_cast<diagnostics_update_work_t *>(arg);
    const matter_air_quality_diagnostics_t &d = work->diagnostics;
    esp_err_t first_error = ESP_OK;

    struct attr_update_t {
        uint32_t id;
        esp_matter_attr_val_t value;
    } updates[] = {
        {kAttrSensorId, esp_matter_uint8(d.sensor_id)},
        {kAttrSensorType, esp_matter_uint8(d.sensor_type)},
        {kAttrEnabled, esp_matter_bool(d.enabled != 0)},
        {kAttrState, esp_matter_uint8(d.state)},
        {kAttrRawAdc, esp_matter_int32(d.raw_adc)},
        {kAttrAdcMv, esp_matter_int32(d.adc_mv)},
        {kAttrVrlMv, esp_matter_int32(d.vrl_mv)},
        {kAttrBaselineVrlMv, esp_matter_int32(d.baseline_vrl_mv)},
        {kAttrRsNormMilli, esp_matter_int32(d.rs_norm_milli)},
        {kAttrRsRatioMilli, esp_matter_int32(d.rs_ratio_milli)},
        {kAttrBaselineValid, esp_matter_bool(d.baseline_valid != 0)},
        {kAttrThresholdState, esp_matter_uint8(d.threshold_state)},
        {kAttrFaultBitmap, esp_matter_uint32(d.fault_bitmap)},
        {kAttrLastAgeMs, esp_matter_uint32(d.last_update_age_ms)},
    };

    for (const auto &update : updates) {
        esp_matter_attr_val_t value = update.value;
        esp_err_t err = update_attribute(work->endpoint_id, kMqDiagnosticsClusterId, update.id, &value);
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
    }

    if (first_error != ESP_OK) {
        ESP_LOGE(TAG, "MQ diagnostics Matter update failed: %s", esp_err_to_name(first_error));
    }
    free(work);
}

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
    esp_matter::cluster_t *diagnostics_cluster = esp_matter::cluster::create(endpoint,
                                                                             kMqDiagnosticsClusterId,
                                                                             esp_matter::CLUSTER_FLAG_SERVER);
    ESP_RETURN_ON_FALSE(diagnostics_cluster != nullptr, ESP_FAIL, TAG,
                        "failed to create MQ diagnostics cluster");
    esp_matter::attribute::create(diagnostics_cluster, kAttrSensorId,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_uint8(0));
    esp_matter::attribute::create(diagnostics_cluster, kAttrSensorType,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_uint8(0));
    esp_matter::attribute::create(diagnostics_cluster, kAttrEnabled,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_bool(false));
    esp_matter::attribute::create(diagnostics_cluster, kAttrState,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_uint8(0));
    esp_matter::attribute::create(diagnostics_cluster, kAttrRawAdc,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_int32(-1));
    esp_matter::attribute::create(diagnostics_cluster, kAttrAdcMv,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_int32(-1));
    esp_matter::attribute::create(diagnostics_cluster, kAttrVrlMv,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_int32(-1));
    esp_matter::attribute::create(diagnostics_cluster, kAttrBaselineVrlMv,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_int32(0));
    esp_matter::attribute::create(diagnostics_cluster, kAttrRsNormMilli,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_int32(0));
    esp_matter::attribute::create(diagnostics_cluster, kAttrRsRatioMilli,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_int32(0));
    esp_matter::attribute::create(diagnostics_cluster, kAttrBaselineValid,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_bool(false));
    esp_matter::attribute::create(diagnostics_cluster, kAttrThresholdState,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_uint8(0));
    esp_matter::attribute::create(diagnostics_cluster, kAttrFaultBitmap,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_uint32(0));
    esp_matter::attribute::create(diagnostics_cluster, kAttrLastAgeMs,
                                  esp_matter::ATTRIBUTE_FLAG_NONE, esp_matter_uint32(0));
    ESP_LOGI(TAG, "Air Quality endpoint created with id %u", (unsigned)*endpoint_id);
    return ESP_OK;
}

esp_err_t matter_update_air_quality(uint16_t endpoint_id, matter_air_quality_level_t level)
{
    ESP_RETURN_ON_FALSE(endpoint_id != 0, ESP_ERR_INVALID_ARG, TAG, "endpoint id is required");
    ESP_RETURN_ON_FALSE(level >= MATTER_AIR_QUALITY_UNKNOWN &&
                            level <= MATTER_AIR_QUALITY_EXTREMELY_POOR,
                        ESP_ERR_INVALID_ARG, TAG, "invalid air quality level %d", (int)level);

    auto *work = static_cast<air_quality_update_work_t *>(calloc(1, sizeof(air_quality_update_work_t)));
    ESP_RETURN_ON_FALSE(work != nullptr, ESP_ERR_NO_MEM, TAG, "failed to allocate Matter update work");
    work->endpoint_id = endpoint_id;
    work->level = level;

    CHIP_ERROR chip_err = chip::DeviceLayer::PlatformMgr().ScheduleWork(air_quality_update_work,
                                                                        reinterpret_cast<intptr_t>(work));
    if (chip_err != CHIP_NO_ERROR) {
        free(work);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t matter_update_air_quality_diagnostics(uint16_t endpoint_id,
                                                const matter_air_quality_diagnostics_t *diagnostics)
{
    ESP_RETURN_ON_FALSE(endpoint_id != 0, ESP_ERR_INVALID_ARG, TAG, "endpoint id is required");
    ESP_RETURN_ON_FALSE(diagnostics != nullptr, ESP_ERR_INVALID_ARG, TAG, "diagnostics are required");

    auto *work = static_cast<diagnostics_update_work_t *>(calloc(1, sizeof(diagnostics_update_work_t)));
    ESP_RETURN_ON_FALSE(work != nullptr, ESP_ERR_NO_MEM, TAG, "failed to allocate diagnostics work");
    work->endpoint_id = endpoint_id;
    work->diagnostics = *diagnostics;

    CHIP_ERROR chip_err = chip::DeviceLayer::PlatformMgr().ScheduleWork(diagnostics_update_work,
                                                                        reinterpret_cast<intptr_t>(work));
    if (chip_err != CHIP_NO_ERROR) {
        free(work);
        return ESP_FAIL;
    }

    return ESP_OK;
}
