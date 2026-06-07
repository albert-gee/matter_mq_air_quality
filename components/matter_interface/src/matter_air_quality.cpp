#include "matter_air_quality.h"

#include <cstdlib>

#include <esp_check.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <platform/PlatformManager.h>

static const char *TAG = "matter_air_quality";

static constexpr uint32_t kAttrSensorTypeSuffix = 0x00;
static constexpr uint32_t kAttrEnabledSuffix = 0x01;
static constexpr uint32_t kAttrStateSuffix = 0x02;
static constexpr uint32_t kAttrRawAdcSuffix = 0x03;
static constexpr uint32_t kAttrAdcMvSuffix = 0x04;
static constexpr uint32_t kAttrVrlMvSuffix = 0x05;
static constexpr uint32_t kAttrBaselineVrlMvSuffix = 0x06;
static constexpr uint32_t kAttrRsNormMilliSuffix = 0x07;
static constexpr uint32_t kAttrRsRatioMilliSuffix = 0x08;
static constexpr uint32_t kAttrThresholdStateSuffix = 0x09;
static constexpr uint32_t kAttrFaultBitmapSuffix = 0x0A;
static constexpr uint32_t kAttrLastUpdateAgeMsSuffix = 0x0B;
static constexpr uint32_t kAttrBaselineValidSuffix = 0x0C;
static constexpr uint32_t kAttrBlockStride = 0x20;
static constexpr size_t kExpectedDiagnosticsAttributeCount =
    MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT * MATTER_MQ_DIAGNOSTICS_ATTRS_PER_SENSOR;
static constexpr uint8_t kDefaultSensorTypes[MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT] = {
    8, 0, 1, 2, 3, 4, 5, 6, 7,
};

typedef struct {
    uint16_t endpoint_id;
    matter_air_quality_level_t level;
} air_quality_update_work_t;

typedef struct {
    uint16_t endpoint_id;
    matter_air_quality_diagnostics_t diagnostics;
} diagnostics_update_work_t;

static bool s_diagnostics_ready;
static size_t s_diagnostics_attribute_count;

static esp_err_t update_attribute(uint16_t endpoint_id, uint32_t cluster_id, uint32_t attribute_id,
                                  esp_matter_attr_val_t *value)
{
    return esp_matter::attribute::update(endpoint_id, cluster_id, attribute_id, value);
}

static uint32_t diagnostics_attr_id(uint8_t sensor_id, uint32_t suffix)
{
    const uint32_t mei_prefix = ((uint32_t)MATTER_MQ_DIAGNOSTICS_VENDOR_ID << 16);
    return mei_prefix | ((uint32_t)sensor_id * kAttrBlockStride) | suffix;
}

static bool create_diagnostics_attr(esp_matter::cluster_t *cluster, uint32_t attr_id,
                                    esp_matter_attr_val_t value)
{
    return esp_matter::attribute::create(cluster, attr_id, esp_matter::ATTRIBUTE_FLAG_NONE, value) != nullptr;
}

static esp_err_t create_diagnostics_attributes(esp_matter::cluster_t *cluster)
{
    ESP_RETURN_ON_FALSE(cluster != nullptr, ESP_ERR_INVALID_ARG, TAG, "diagnostics cluster is required");

    size_t created = 0;
    for (uint8_t sensor_id = 0; sensor_id < MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT; ++sensor_id) {
        const struct {
            uint32_t suffix;
            esp_matter_attr_val_t value;
        } attrs[] = {
            {kAttrSensorTypeSuffix, esp_matter_uint8(kDefaultSensorTypes[sensor_id])},
            {kAttrEnabledSuffix, esp_matter_bool(false)},
            {kAttrStateSuffix, esp_matter_uint8(0)},
            {kAttrRawAdcSuffix, esp_matter_int32(-1)},
            {kAttrAdcMvSuffix, esp_matter_int32(-1)},
            {kAttrVrlMvSuffix, esp_matter_int32(-1)},
            {kAttrBaselineVrlMvSuffix, esp_matter_int32(0)},
            {kAttrRsNormMilliSuffix, esp_matter_int32(0)},
            {kAttrRsRatioMilliSuffix, esp_matter_int32(0)},
            {kAttrThresholdStateSuffix, esp_matter_uint8(0)},
            {kAttrFaultBitmapSuffix, esp_matter_uint32(0)},
            {kAttrLastUpdateAgeMsSuffix, esp_matter_uint32(0)},
            {kAttrBaselineValidSuffix, esp_matter_bool(false)},
        };

        for (const auto &attr : attrs) {
            const uint32_t attr_id = diagnostics_attr_id(sensor_id, attr.suffix);
            ESP_RETURN_ON_FALSE(create_diagnostics_attr(cluster, attr_id, attr.value),
                                ESP_FAIL, TAG,
                                "failed to create MQ diagnostics attr sensor=%u attr=0x%08lx",
                                (unsigned)sensor_id,
                                (unsigned long)attr_id);
            ++created;
        }
    }

    s_diagnostics_attribute_count = created;
    s_diagnostics_ready = created == kExpectedDiagnosticsAttributeCount;
    ESP_RETURN_ON_FALSE(s_diagnostics_ready, ESP_FAIL, TAG,
                        "created %u MQ diagnostics attrs, expected %u",
                        (unsigned)created,
                        (unsigned)kExpectedDiagnosticsAttributeCount);
    return ESP_OK;
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
    esp_err_t first_error = ESP_OK;
    size_t failure_count = 0;

    for (uint8_t sensor_id = 0; sensor_id < MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT; ++sensor_id) {
        const matter_mq_sensor_diagnostics_t &d = work->diagnostics.sensors[sensor_id];
        const struct {
            uint32_t suffix;
            esp_matter_attr_val_t value;
        } attrs[] = {
            {kAttrSensorTypeSuffix, esp_matter_uint8(d.sensor_type)},
            {kAttrEnabledSuffix, esp_matter_bool(d.enabled)},
            {kAttrStateSuffix, esp_matter_uint8(d.state)},
            {kAttrRawAdcSuffix, esp_matter_int32(d.raw_adc)},
            {kAttrAdcMvSuffix, esp_matter_int32(d.adc_mv)},
            {kAttrVrlMvSuffix, esp_matter_int32(d.vrl_mv)},
            {kAttrBaselineVrlMvSuffix, esp_matter_int32(d.baseline_vrl_mv)},
            {kAttrRsNormMilliSuffix, esp_matter_int32(d.rs_norm_milli)},
            {kAttrRsRatioMilliSuffix, esp_matter_int32(d.rs_ratio_milli)},
            {kAttrThresholdStateSuffix, esp_matter_uint8(d.threshold_state)},
            {kAttrFaultBitmapSuffix, esp_matter_uint32(d.fault_bitmap)},
            {kAttrLastUpdateAgeMsSuffix, esp_matter_uint32(d.last_update_age_ms)},
            {kAttrBaselineValidSuffix, esp_matter_bool(d.baseline_valid)},
        };

        for (const auto &attr : attrs) {
            esp_matter_attr_val_t value = attr.value;
            const uint32_t attr_id = diagnostics_attr_id(sensor_id, attr.suffix);
            const esp_err_t err = update_attribute(work->endpoint_id,
                                                   MATTER_MQ_DIAGNOSTICS_CLUSTER_ID,
                                                   attr_id,
                                                   &value);
            if (err != ESP_OK) {
                if (first_error == ESP_OK) {
                    first_error = err;
                }
                ++failure_count;
            }
        }
    }

    if (first_error != ESP_OK) {
        ESP_LOGE(TAG, "MQ diagnostics Matter update failed: first=%s failures=%u",
                 esp_err_to_name(first_error), (unsigned)failure_count);
    }
    free(work);
}

esp_err_t matter_create_air_quality_endpoint(esp_matter::node_t *matter_node, uint16_t *endpoint_id)
{
    ESP_RETURN_ON_FALSE(matter_node != nullptr, ESP_ERR_INVALID_ARG, TAG, "Matter node is required");
    ESP_RETURN_ON_FALSE(endpoint_id != nullptr, ESP_ERR_INVALID_ARG, TAG, "endpoint id output is required");
    s_diagnostics_ready = false;
    s_diagnostics_attribute_count = 0;

    esp_matter::endpoint::air_quality_sensor::config_t endpoint_config;

    esp_matter::endpoint_t *endpoint = esp_matter::endpoint::air_quality_sensor::create(
        matter_node, &endpoint_config, esp_matter::ENDPOINT_FLAG_NONE, nullptr);
    ESP_RETURN_ON_FALSE(endpoint != nullptr, ESP_FAIL, TAG, "failed to create Air Quality endpoint");

    *endpoint_id = esp_matter::endpoint::get_id(endpoint);
    esp_matter::cluster_t *diagnostics_cluster = esp_matter::cluster::create(
        endpoint,
        MATTER_MQ_DIAGNOSTICS_CLUSTER_ID,
        esp_matter::CLUSTER_FLAG_SERVER);
    ESP_RETURN_ON_FALSE(diagnostics_cluster != nullptr, ESP_FAIL, TAG,
                        "failed to create MQ diagnostics cluster 0x%08lx",
                        (unsigned long)MATTER_MQ_DIAGNOSTICS_CLUSTER_ID);
    ESP_RETURN_ON_ERROR(create_diagnostics_attributes(diagnostics_cluster),
                        TAG, "failed to create MQ diagnostics attributes");

    ESP_LOGI(TAG, "Air Quality endpoint created with id %u and MQ diagnostics cluster 0x%08lx",
             (unsigned)*endpoint_id,
             (unsigned long)MATTER_MQ_DIAGNOSTICS_CLUSTER_ID);
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
    ESP_RETURN_ON_ERROR(matter_air_quality_diagnostics_validate(),
                        TAG, "MQ diagnostics cluster is not ready");

    auto *work = static_cast<diagnostics_update_work_t *>(calloc(1, sizeof(diagnostics_update_work_t)));
    ESP_RETURN_ON_FALSE(work != nullptr, ESP_ERR_NO_MEM, TAG, "failed to allocate diagnostics work");
    work->endpoint_id = endpoint_id;
    work->diagnostics = *diagnostics;

    CHIP_ERROR chip_err = chip::DeviceLayer::PlatformMgr().ScheduleWork(diagnostics_update_work,
                                                                        reinterpret_cast<intptr_t>(work));
    if (chip_err != CHIP_NO_ERROR) {
        free(work);
        ESP_LOGE(TAG, "failed to schedule MQ diagnostics Matter update");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t matter_air_quality_diagnostics_validate(void)
{
    ESP_RETURN_ON_FALSE(MATTER_MQ_DIAGNOSTICS_CLUSTER_ID ==
                            (((uint32_t)MATTER_MQ_DIAGNOSTICS_VENDOR_ID << 16) |
                             MATTER_MQ_DIAGNOSTICS_CLUSTER_SUFFIX),
                        ESP_FAIL, TAG, "MQ diagnostics cluster id is invalid");
    ESP_RETURN_ON_FALSE(s_diagnostics_ready, ESP_ERR_INVALID_STATE, TAG,
                        "MQ diagnostics cluster is not ready");
    ESP_RETURN_ON_FALSE(s_diagnostics_attribute_count == kExpectedDiagnosticsAttributeCount,
                        ESP_ERR_INVALID_STATE, TAG,
                        "MQ diagnostics attr count %u expected %u",
                        (unsigned)s_diagnostics_attribute_count,
                        (unsigned)kExpectedDiagnosticsAttributeCount);
    return ESP_OK;
}

uint32_t matter_air_quality_diagnostics_cluster_id(void)
{
    return MATTER_MQ_DIAGNOSTICS_CLUSTER_ID;
}

size_t matter_air_quality_diagnostics_attribute_count(void)
{
    return s_diagnostics_attribute_count;
}
