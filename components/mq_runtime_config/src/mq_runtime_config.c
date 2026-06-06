#include "mq_runtime_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"

#define MQ_RUNTIME_CONFIG_NAMESPACE "mq_cfg"
#define MQ_RUNTIME_SENSOR_MAGIC 0x4d515330U
#define MQ_RUNTIME_SOURCE_MAGIC 0x4d514130U
#define MQ_RUNTIME_MUX_MAGIC 0x4d514d30U
#define MQ_RUNTIME_AQ_MAGIC 0x4d514151U
#define MQ_RUNTIME_CONFIG_VERSION 1U
#define MQ_RUNTIME_SENSOR_VERSION 3U
#define MQ_RUNTIME_SOURCE_VERSION 3U
#define MQ_RUNTIME_MAX_SENSOR_ID 8U
#define MQ_RUNTIME_MAX_SOURCE_ID 8U
#define MQ_RUNTIME_SIGNAL_ADC_ID 0U
#define MQ_RUNTIME_MAX_MUX_CHANNEL 15U
#define MQ_RUNTIME_MUX_ID 0U
#define MQ_RUNTIME_MAX_WARMUP_SECONDS 172800U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t valid;
    uint8_t enabled;
    uint32_t vc_mv;
    uint32_t warmup_seconds;
    float warning_rs_ratio;
    float critical_rs_ratio;
} mq_runtime_sensor_record_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t valid;
    uint8_t type;
    uint8_t adc_logical_channel;
    float input_divider_ratio;
    uint8_t mux_id;
    uint8_t mux_channel;
} mq_runtime_source_record_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t valid;
    uint8_t enabled;
    uint8_t signal_adc_logical_channel;
    int32_t gpio_s0;
    int32_t gpio_s1;
    int32_t gpio_s2;
    int32_t gpio_s3;
    int32_t gpio_en;
    uint8_t en_active_low;
    uint32_t settle_time_us;
} mq_runtime_mux_record_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint8_t valid;
    uint8_t primary_sensor_id;
    uint32_t sample_interval_ms;
    uint32_t stale_after_ms;
    float ema_alpha;
    uint8_t publish_to_matter;
} mq_runtime_aq_record_t;

static const char *TAG = "mq_runtime_cfg";
static bool s_initialized;

static void make_indexed_key(char prefix, uint8_t id, char *key, size_t key_size)
{
    snprintf(key, key_size, "%c%02u", prefix, (unsigned)id);
}

static esp_err_t open_namespace(nvs_open_mode_t mode, nvs_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "runtime config is not initialized");
    return nvs_open(MQ_RUNTIME_CONFIG_NAMESPACE, mode, handle);
}

static bool is_sensor_id_valid(uint8_t sensor_id)
{
    return sensor_id <= MQ_RUNTIME_MAX_SENSOR_ID;
}

static bool is_source_id_valid(uint8_t source_id)
{
    return source_id <= MQ_RUNTIME_MAX_SOURCE_ID;
}

static bool is_gpio_valid_or_unused(int32_t gpio)
{
    return gpio == ANALOG_MUX_GPIO_UNUSED || (gpio >= 0 && gpio < GPIO_NUM_MAX);
}

static bool is_mux_id_valid(uint8_t mux_id)
{
    return mux_id == MQ_RUNTIME_MUX_ID;
}

static bool mux_record_has_duplicate_gpios(const mq_runtime_mux_record_t *record)
{
    if (record == NULL) {
        return false;
    }

    const int32_t gpios[] = {
        record->gpio_s0,
        record->gpio_s1,
        record->gpio_s2,
        record->gpio_s3,
        record->gpio_en,
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

static bool is_sensor_record_valid(uint8_t sensor_id, const mq_runtime_sensor_record_t *record)
{
    if (record == NULL || record->magic != MQ_RUNTIME_SENSOR_MAGIC ||
        record->version != MQ_RUNTIME_SENSOR_VERSION || record->valid == 0) {
        return false;
    }
    if (!is_sensor_id_valid(sensor_id)) {
        return false;
    }
    if (record->vc_mv < 1000U || record->vc_mv > 10000U) {
        return false;
    }
    const uint32_t max_warmup = MQ_RUNTIME_MAX_WARMUP_SECONDS;
    if (record->warmup_seconds > max_warmup) {
        return false;
    }
    if (record->warning_rs_ratio < 0.0f || record->critical_rs_ratio < 0.0f) {
        return false;
    }
    return true;
}

static bool is_source_record_valid(uint8_t source_id, const mq_runtime_source_record_t *record)
{
    if (record == NULL || record->magic != MQ_RUNTIME_SOURCE_MAGIC ||
        record->version != MQ_RUNTIME_SOURCE_VERSION || record->valid == 0) {
        return false;
    }
    if (!is_source_id_valid(source_id)) {
        return false;
    }
    if (record->input_divider_ratio < 0.1f || record->input_divider_ratio > 20.0f) {
        return false;
    }
    if (record->type == ANALOG_BACKEND_MUX_ADC) {
        return record->adc_logical_channel == MQ_RUNTIME_SIGNAL_ADC_ID &&
               record->mux_id == MQ_RUNTIME_MUX_ID &&
               record->mux_channel <= MQ_RUNTIME_MAX_MUX_CHANNEL;
    }
    return false;
}

static bool is_mux_record_valid(uint8_t mux_id, const mq_runtime_mux_record_t *record)
{
    if (record == NULL || record->magic != MQ_RUNTIME_MUX_MAGIC ||
        record->version != MQ_RUNTIME_CONFIG_VERSION || record->valid == 0) {
        return false;
    }
    if (!is_mux_id_valid(mux_id)) {
        return false;
    }
    if (record->signal_adc_logical_channel != MQ_RUNTIME_SIGNAL_ADC_ID) {
        return false;
    }
    if (record->settle_time_us < 50U || record->settle_time_us > 5000U) {
        return false;
    }
    if (!is_gpio_valid_or_unused(record->gpio_s0) ||
        !is_gpio_valid_or_unused(record->gpio_s1) ||
        !is_gpio_valid_or_unused(record->gpio_s2) ||
        !is_gpio_valid_or_unused(record->gpio_s3) ||
        !is_gpio_valid_or_unused(record->gpio_en)) {
        return false;
    }
    if (mux_record_has_duplicate_gpios(record)) {
        return false;
    }
    if (record->enabled != 0 &&
        (record->gpio_s0 == ANALOG_MUX_GPIO_UNUSED ||
         record->gpio_s1 == ANALOG_MUX_GPIO_UNUSED ||
         record->gpio_s2 == ANALOG_MUX_GPIO_UNUSED ||
         record->gpio_s3 == ANALOG_MUX_GPIO_UNUSED)) {
        return false;
    }
    return true;
}

static bool is_aq_record_valid(const mq_runtime_aq_record_t *record)
{
    if (record == NULL || record->magic != MQ_RUNTIME_AQ_MAGIC ||
        record->version != MQ_RUNTIME_CONFIG_VERSION || record->valid == 0) {
        return false;
    }
    if (record->primary_sensor_id != 0U) {
        return false;
    }
    if (record->sample_interval_ms < 1000U || record->sample_interval_ms > 60000U) {
        return false;
    }
    if (record->stale_after_ms < record->sample_interval_ms || record->stale_after_ms > 300000U) {
        return false;
    }
    if (record->ema_alpha <= 0.0f || record->ema_alpha > 1.0f) {
        return false;
    }
    return true;
}

static esp_err_t read_blob(const char *key, void *record, size_t record_size)
{
    nvs_handle_t handle;
    esp_err_t err = open_namespace(NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to open runtime config namespace");

    size_t required_size = record_size;
    err = nvs_get_blob(handle, key, record, &required_size);
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to read runtime config key %s", key);
    ESP_RETURN_ON_FALSE(required_size == record_size, ESP_ERR_INVALID_SIZE, TAG,
                        "runtime config size mismatch for key %s", key);
    return ESP_OK;
}

static esp_err_t write_blob(const char *key, const void *record, size_t record_size)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(open_namespace(NVS_READWRITE, &handle), TAG,
                        "failed to open runtime config namespace");

    esp_err_t err = nvs_set_blob(handle, key, record, record_size);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to save runtime config key %s", key);
    return ESP_OK;
}

static esp_err_t erase_key_if_present(const char *key)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(open_namespace(NVS_READWRITE, &handle), TAG,
                        "failed to open runtime config namespace");

    esp_err_t err = nvs_erase_key(handle, key);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to erase runtime config key %s", key);
    return ESP_OK;
}

esp_err_t mq_runtime_config_init(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MQ_RUNTIME_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to open NVS namespace %s: %s",
                 MQ_RUNTIME_CONFIG_NAMESPACE, esp_err_to_name(err));
        return err;
    }
    nvs_close(handle);
    s_initialized = true;
    ESP_LOGI(TAG, "runtime config NVS initialized");
    return ESP_OK;
}

esp_err_t mq_runtime_config_apply_sensor(uint8_t sensor_id,
                                         const mq_sensor_config_t *defaults,
                                         mq_sensor_config_t *effective)
{
    ESP_RETURN_ON_FALSE(defaults != NULL, ESP_ERR_INVALID_ARG, TAG, "sensor defaults are required");
    ESP_RETURN_ON_FALSE(effective != NULL, ESP_ERR_INVALID_ARG, TAG, "sensor output is required");
    ESP_RETURN_ON_FALSE(is_sensor_id_valid(sensor_id), ESP_ERR_INVALID_ARG, TAG,
                        "sensor id %u is out of range", (unsigned)sensor_id);

    *effective = *defaults;

    char key[8];
    make_indexed_key('s', sensor_id, key, sizeof(key));

    mq_runtime_sensor_record_t record;
    esp_err_t err = read_blob(key, &record, sizeof(record));
    if (err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }
    if (err == ESP_ERR_INVALID_SIZE || err == ESP_ERR_NVS_INVALID_LENGTH) {
        ESP_LOGW(TAG, "ignoring incompatible sensor override %u", (unsigned)sensor_id);
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to load sensor override %u", (unsigned)sensor_id);
    if (!is_sensor_record_valid(sensor_id, &record)) {
        ESP_LOGW(TAG, "ignoring invalid sensor override %u", (unsigned)sensor_id);
        return ESP_OK;
    }

    effective->enabled = record.enabled != 0;
    effective->vc_mv = record.vc_mv;
    effective->warmup_seconds = record.warmup_seconds;
    effective->warning_rs_ratio = record.warning_rs_ratio;
    effective->critical_rs_ratio = record.critical_rs_ratio;
    return ESP_OK;
}

esp_err_t mq_runtime_config_apply_source(uint8_t source_id,
                                         const analog_source_config_t *defaults,
                                         analog_source_config_t *effective)
{
    ESP_RETURN_ON_FALSE(defaults != NULL, ESP_ERR_INVALID_ARG, TAG, "source defaults are required");
    ESP_RETURN_ON_FALSE(effective != NULL, ESP_ERR_INVALID_ARG, TAG, "source output is required");
    ESP_RETURN_ON_FALSE(is_source_id_valid(source_id), ESP_ERR_INVALID_ARG, TAG,
                        "source id %u is out of range", (unsigned)source_id);

    *effective = *defaults;

    char key[8];
    make_indexed_key('a', source_id, key, sizeof(key));

    mq_runtime_source_record_t record;
    esp_err_t err = read_blob(key, &record, sizeof(record));
    if (err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }
    if (err == ESP_ERR_INVALID_SIZE || err == ESP_ERR_NVS_INVALID_LENGTH) {
        ESP_LOGW(TAG, "ignoring incompatible source override %u", (unsigned)source_id);
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to load source override %u", (unsigned)source_id);
    if (!is_source_record_valid(source_id, &record)) {
        ESP_LOGW(TAG, "ignoring invalid source override %u", (unsigned)source_id);
        return ESP_OK;
    }

    effective->type = (analog_backend_type_t)record.type;
    effective->adc_logical_channel = record.adc_logical_channel;
    effective->input_divider_ratio = record.input_divider_ratio;
    effective->mux_id = record.mux_id;
    effective->mux_channel = record.mux_channel;
    return ESP_OK;
}

esp_err_t mq_runtime_config_apply_mux(uint8_t mux_id,
                                      const analog_mux_config_t *defaults,
                                      analog_mux_config_t *effective)
{
    ESP_RETURN_ON_FALSE(defaults != NULL, ESP_ERR_INVALID_ARG, TAG, "mux defaults are required");
    ESP_RETURN_ON_FALSE(effective != NULL, ESP_ERR_INVALID_ARG, TAG, "mux output is required");
    ESP_RETURN_ON_FALSE(is_mux_id_valid(mux_id), ESP_ERR_NOT_SUPPORTED, TAG,
                        "only mux_id 0 is supported for now");

    *effective = *defaults;

    char key[8];
    make_indexed_key('m', mux_id, key, sizeof(key));

    mq_runtime_mux_record_t record;
    esp_err_t err = read_blob(key, &record, sizeof(record));
    if (err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }
    if (err == ESP_ERR_INVALID_SIZE || err == ESP_ERR_NVS_INVALID_LENGTH) {
        ESP_LOGW(TAG, "ignoring incompatible mux override %u", (unsigned)mux_id);
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to load mux override %u", (unsigned)mux_id);
    if (!is_mux_record_valid(mux_id, &record)) {
        ESP_LOGW(TAG, "ignoring invalid mux override %u", (unsigned)mux_id);
        return ESP_OK;
    }

    effective->enabled = record.enabled != 0;
    effective->signal_adc_logical_channel = record.signal_adc_logical_channel;
    effective->gpio_s0 = record.gpio_s0;
    effective->gpio_s1 = record.gpio_s1;
    effective->gpio_s2 = record.gpio_s2;
    effective->gpio_s3 = record.gpio_s3;
    effective->gpio_en = record.gpio_en;
    effective->en_active_low = record.en_active_low != 0;
    effective->settle_time_us = record.settle_time_us;
    return ESP_OK;
}

esp_err_t mq_runtime_config_apply_air_quality(const air_quality_service_config_t *defaults,
                                              air_quality_service_config_t *effective)
{
    ESP_RETURN_ON_FALSE(defaults != NULL, ESP_ERR_INVALID_ARG, TAG, "AQ defaults are required");
    ESP_RETURN_ON_FALSE(effective != NULL, ESP_ERR_INVALID_ARG, TAG, "AQ output is required");

    *effective = *defaults;

    mq_runtime_aq_record_t record;
    esp_err_t err = read_blob("aq", &record, sizeof(record));
    if (err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }
    if (err == ESP_ERR_INVALID_SIZE || err == ESP_ERR_NVS_INVALID_LENGTH) {
        ESP_LOGW(TAG, "ignoring incompatible AQ override");
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to load AQ override");
    if (!is_aq_record_valid(&record)) {
        ESP_LOGW(TAG, "ignoring invalid AQ override");
        return ESP_OK;
    }

    effective->primary_sensor_id = record.primary_sensor_id;
    effective->sample_interval_ms = record.sample_interval_ms;
    effective->stale_after_ms = record.stale_after_ms;
    effective->ema_alpha = record.ema_alpha;
    effective->publish_to_matter = record.publish_to_matter != 0;
    return ESP_OK;
}

esp_err_t mq_runtime_config_save_sensor(const mq_sensor_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "sensor config is required");
    ESP_RETURN_ON_FALSE(is_sensor_id_valid(config->id), ESP_ERR_INVALID_ARG, TAG,
                        "sensor id %u is out of range", (unsigned)config->id);
    ESP_RETURN_ON_FALSE(config->vc_mv >= 1000U && config->vc_mv <= 10000U, ESP_ERR_INVALID_ARG,
                        TAG, "vc_mv must be between 1000 and 10000");
    ESP_RETURN_ON_FALSE(config->warmup_seconds <= MQ_RUNTIME_MAX_WARMUP_SECONDS,
                        ESP_ERR_INVALID_ARG, TAG, "warm-up seconds exceed maximum");
    ESP_RETURN_ON_FALSE(config->warning_rs_ratio >= 0.0f &&
                            config->critical_rs_ratio >= 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "threshold ratios must not be negative");

    const mq_runtime_sensor_record_t record = {
        .magic = MQ_RUNTIME_SENSOR_MAGIC,
        .version = MQ_RUNTIME_SENSOR_VERSION,
        .valid = 1,
        .enabled = config->enabled ? 1 : 0,
        .vc_mv = config->vc_mv,
        .warmup_seconds = config->warmup_seconds,
        .warning_rs_ratio = config->warning_rs_ratio,
        .critical_rs_ratio = config->critical_rs_ratio,
    };

    char key[8];
    make_indexed_key('s', config->id, key, sizeof(key));
    return write_blob(key, &record, sizeof(record));
}

esp_err_t mq_runtime_config_save_source(const analog_source_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "source config is required");
    ESP_RETURN_ON_FALSE(is_source_id_valid(config->source_id), ESP_ERR_INVALID_ARG, TAG,
                        "source id %u is out of range", (unsigned)config->source_id);
    ESP_RETURN_ON_FALSE(config->type == ANALOG_BACKEND_MUX_ADC,
                        ESP_ERR_NOT_SUPPORTED, TAG,
                        "only mux ADC source overrides are supported");
    ESP_RETURN_ON_FALSE(config->adc_logical_channel == MQ_RUNTIME_SIGNAL_ADC_ID, ESP_ERR_INVALID_ARG,
                        TAG, "adc_logical_channel must be 0");
    ESP_RETURN_ON_FALSE(config->mux_id == MQ_RUNTIME_MUX_ID &&
                            config->mux_channel <= MQ_RUNTIME_MAX_MUX_CHANNEL,
                        ESP_ERR_INVALID_ARG, TAG,
                        "mux sources require mux_id=0 and mux_channel 0..15");
    ESP_RETURN_ON_FALSE(config->input_divider_ratio >= 0.1f &&
                            config->input_divider_ratio <= 20.0f,
                        ESP_ERR_INVALID_ARG, TAG, "input divider ratio must be 0.1..20.0");

    const mq_runtime_source_record_t record = {
        .magic = MQ_RUNTIME_SOURCE_MAGIC,
        .version = MQ_RUNTIME_SOURCE_VERSION,
        .valid = 1,
        .type = (uint8_t)config->type,
        .adc_logical_channel = config->adc_logical_channel,
        .input_divider_ratio = config->input_divider_ratio,
        .mux_id = config->mux_id,
        .mux_channel = config->mux_channel,
    };

    char key[8];
    make_indexed_key('a', config->source_id, key, sizeof(key));
    return write_blob(key, &record, sizeof(record));
}

esp_err_t mq_runtime_config_save_mux(const analog_mux_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "mux config is required");
    ESP_RETURN_ON_FALSE(is_mux_id_valid(config->mux_id), ESP_ERR_NOT_SUPPORTED, TAG,
                        "only mux_id 0 is supported for now");
    ESP_RETURN_ON_FALSE(config->signal_adc_logical_channel == MQ_RUNTIME_SIGNAL_ADC_ID,
                        ESP_ERR_INVALID_ARG, TAG, "signal_adc_logical_channel must be 0");
    ESP_RETURN_ON_FALSE(config->settle_time_us >= 50U && config->settle_time_us <= 5000U,
                        ESP_ERR_INVALID_ARG, TAG, "settle_time_us must be 50..5000");
    ESP_RETURN_ON_FALSE(is_gpio_valid_or_unused(config->gpio_s0) &&
                            is_gpio_valid_or_unused(config->gpio_s1) &&
                            is_gpio_valid_or_unused(config->gpio_s2) &&
                            is_gpio_valid_or_unused(config->gpio_s3) &&
                            is_gpio_valid_or_unused(config->gpio_en),
                        ESP_ERR_INVALID_ARG, TAG, "mux GPIO out of range");

    const mq_runtime_mux_record_t record = {
        .magic = MQ_RUNTIME_MUX_MAGIC,
        .version = MQ_RUNTIME_CONFIG_VERSION,
        .valid = 1,
        .enabled = config->enabled ? 1 : 0,
        .signal_adc_logical_channel = config->signal_adc_logical_channel,
        .gpio_s0 = config->gpio_s0,
        .gpio_s1 = config->gpio_s1,
        .gpio_s2 = config->gpio_s2,
        .gpio_s3 = config->gpio_s3,
        .gpio_en = config->gpio_en,
        .en_active_low = config->en_active_low ? 1 : 0,
        .settle_time_us = config->settle_time_us,
    };
    ESP_RETURN_ON_FALSE(!mux_record_has_duplicate_gpios(&record),
                        ESP_ERR_INVALID_ARG, TAG, "mux GPIO assignments must be unique");
    ESP_RETURN_ON_FALSE(!config->enabled ||
                            (config->gpio_s0 != ANALOG_MUX_GPIO_UNUSED &&
                             config->gpio_s1 != ANALOG_MUX_GPIO_UNUSED &&
                             config->gpio_s2 != ANALOG_MUX_GPIO_UNUSED &&
                             config->gpio_s3 != ANALOG_MUX_GPIO_UNUSED),
                        ESP_ERR_INVALID_ARG, TAG,
                        "enabled mux requires S0/S1/S2/S3 GPIOs");

    char key[8];
    make_indexed_key('m', config->mux_id, key, sizeof(key));
    return write_blob(key, &record, sizeof(record));
}

esp_err_t mq_runtime_config_save_air_quality(const air_quality_service_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "AQ config is required");
    ESP_RETURN_ON_FALSE(config->primary_sensor_id == 0U,
                        ESP_ERR_INVALID_ARG, TAG,
                        "primary sensor id must be 0 (MQ-135)");
    ESP_RETURN_ON_FALSE(config->sample_interval_ms >= 1000U &&
                            config->sample_interval_ms <= 60000U,
                        ESP_ERR_INVALID_ARG, TAG, "sample_interval_ms must be 1000..60000");
    ESP_RETURN_ON_FALSE(config->stale_after_ms >= config->sample_interval_ms &&
                            config->stale_after_ms <= 300000U,
                        ESP_ERR_INVALID_ARG, TAG,
                        "stale_after_ms must be >= sample_interval_ms and <= 300000");
    ESP_RETURN_ON_FALSE(config->ema_alpha > 0.0f && config->ema_alpha <= 1.0f,
                        ESP_ERR_INVALID_ARG, TAG, "ema_alpha must be in (0, 1]");

    const mq_runtime_aq_record_t record = {
        .magic = MQ_RUNTIME_AQ_MAGIC,
        .version = MQ_RUNTIME_CONFIG_VERSION,
        .valid = 1,
        .primary_sensor_id = config->primary_sensor_id,
        .sample_interval_ms = config->sample_interval_ms,
        .stale_after_ms = config->stale_after_ms,
        .ema_alpha = config->ema_alpha,
        .publish_to_matter = config->publish_to_matter ? 1 : 0,
    };

    return write_blob("aq", &record, sizeof(record));
}

esp_err_t mq_runtime_config_erase_sensor(uint8_t sensor_id)
{
    ESP_RETURN_ON_FALSE(is_sensor_id_valid(sensor_id), ESP_ERR_INVALID_ARG, TAG,
                        "sensor id %u is out of range", (unsigned)sensor_id);
    char key[8];
    make_indexed_key('s', sensor_id, key, sizeof(key));
    return erase_key_if_present(key);
}

esp_err_t mq_runtime_config_erase_source(uint8_t source_id)
{
    ESP_RETURN_ON_FALSE(is_source_id_valid(source_id), ESP_ERR_INVALID_ARG, TAG,
                        "source id %u is out of range", (unsigned)source_id);
    char key[8];
    make_indexed_key('a', source_id, key, sizeof(key));
    return erase_key_if_present(key);
}

esp_err_t mq_runtime_config_erase_mux(uint8_t mux_id)
{
    ESP_RETURN_ON_FALSE(is_mux_id_valid(mux_id), ESP_ERR_NOT_SUPPORTED, TAG,
                        "only mux_id 0 is supported for now");
    char key[8];
    make_indexed_key('m', mux_id, key, sizeof(key));
    return erase_key_if_present(key);
}

esp_err_t mq_runtime_config_erase_air_quality(void)
{
    return erase_key_if_present("aq");
}

esp_err_t mq_runtime_config_erase_all(void)
{
    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(open_namespace(NVS_READWRITE, &handle), TAG,
                        "failed to open runtime config namespace");

    esp_err_t err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to erase runtime config namespace");
    return ESP_OK;
}
