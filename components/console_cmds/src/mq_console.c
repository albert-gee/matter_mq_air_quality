#include "mq_console.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "adc_service.h"
#include "air_quality_service.h"
#include "analog_backend.h"
#include "analog_mux.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mq_calibration_nvs.h"
#include "mq_runtime_config.h"
#include "mq_sensor.h"
#include "sdkconfig.h"

static const char *TAG = "mq_console";
static const size_t MQ_CONSOLE_DEFAULT_CAL_SAMPLES = 32;
static const uint32_t MQ_CONSOLE_DEFAULT_CAL_DELAY_MS = 100;
static const uint8_t MQ_CONSOLE_DEFAULT_PRIMARY_SENSOR_ID = 0;
static const uint32_t MQ_CONSOLE_DEFAULT_SAMPLE_INTERVAL_MS = 5000;
static const uint32_t MQ_CONSOLE_DEFAULT_STALE_AFTER_MS = 20000;
static const float MQ_CONSOLE_DEFAULT_EMA_ALPHA = 0.25f;
static const bool MQ_CONSOLE_DEFAULT_PUBLISH_TO_MATTER = true;

static bool analog_source_exists(uint8_t source_id);

static bool parse_u8_arg(const char *text, uint8_t *out)
{
    if (text == NULL || out == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const long value = strtol(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || value < 0 || value > UINT8_MAX) {
        return false;
    }

    *out = (uint8_t)value;
    return true;
}

static bool parse_size_arg(const char *text, size_t min_value, size_t max_value, size_t *out)
{
    if (text == NULL || out == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const unsigned long value = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || value < min_value || value > max_value) {
        return false;
    }

    *out = (size_t)value;
    return true;
}

static bool parse_u32_arg(const char *text, uint32_t max_value, uint32_t *out)
{
    if (text == NULL || out == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const unsigned long value = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || value > max_value) {
        return false;
    }

    *out = (uint32_t)value;
    return true;
}

static bool parse_i32_arg(const char *text, int32_t min_value, int32_t max_value, int32_t *out)
{
    if (text == NULL || out == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const long value = strtol(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || value < min_value || value > max_value) {
        return false;
    }

    *out = (int32_t)value;
    return true;
}

static bool parse_float_arg(const char *text, float min_value, float max_value, float *out)
{
    if (text == NULL || out == NULL) {
        return false;
    }

    errno = 0;
    char *end = NULL;
    const float value = strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || value < min_value || value > max_value) {
        return false;
    }

    *out = value;
    return true;
}

static const char *source_type_to_string(analog_backend_type_t type)
{
    switch (type) {
    case ANALOG_BACKEND_INTERNAL_ADC:
        return "internal-adc";
    case ANALOG_BACKEND_EXTERNAL_ADC:
        return "external-adc";
    case ANALOG_BACKEND_MUX_ADC:
        return "mux-adc";
    default:
        return "unknown";
    }
}

static void print_save_result(esp_err_t err)
{
    if (err == ESP_OK) {
        printf("saved=true reboot_required=true\n");
    } else {
        printf("saved=false error=%s\n", esp_err_to_name(err));
    }
}

static void print_save_result_with_calibration(esp_err_t save_err, esp_err_t calibration_err, size_t erased_count)
{
    if (save_err != ESP_OK) {
        printf("saved=false error=%s\n", esp_err_to_name(save_err));
        return;
    }
    if (calibration_err != ESP_OK) {
        printf("saved=true reboot_required=true calibration_erased=%u calibration_error=%s\n",
               (unsigned)erased_count, esp_err_to_name(calibration_err));
        return;
    }
    printf("saved=true reboot_required=true calibration_erased=%u\n", (unsigned)erased_count);
}

static void get_calibration_summary(uint8_t sensor_id, bool *valid, float *r0_ohms, uint32_t *calibrated_at_unix)
{
    if (valid != NULL) {
        *valid = false;
    }
    if (r0_ohms != NULL) {
        *r0_ohms = 0.0f;
    }
    if (calibrated_at_unix != NULL) {
        *calibrated_at_unix = 0;
    }

    mq_calibration_record_t record;
    const esp_err_t err = mq_calibration_nvs_load(sensor_id, &record);
    if (err != ESP_OK) {
        return;
    }

    if (valid != NULL) {
        *valid = record.valid && record.r0_ohms > 0.0f;
    }
    if (r0_ohms != NULL) {
        *r0_ohms = record.r0_ohms;
    }
    if (calibrated_at_unix != NULL) {
        *calibrated_at_unix = record.calibrated_at_unix;
    }
}

static esp_err_t erase_calibration_for_sensor(uint8_t sensor_id, bool *erased)
{
    bool had_record = false;
    mq_calibration_record_t record;
    esp_err_t err = mq_calibration_nvs_load(sensor_id, &record);
    if (err == ESP_OK) {
        had_record = true;
    } else if (err != ESP_ERR_NOT_FOUND) {
        return err;
    }

    err = mq_sensor_erase_calibration(sensor_id);
    if (err == ESP_ERR_NOT_FOUND || err == ESP_ERR_INVALID_STATE) {
        err = mq_calibration_nvs_erase(sensor_id);
        if (err == ESP_ERR_NOT_FOUND) {
            err = ESP_OK;
        }
    }

    if (err == ESP_OK && erased != NULL) {
        *erased = had_record;
    }
    return err;
}

static esp_err_t erase_calibrations_for_source(uint8_t source_id, size_t *erased_count)
{
    size_t count = 0;
    esp_err_t first_error = ESP_OK;
    const size_t sensor_count = board_config_mq_sensor_count();
    for (size_t i = 0; i < sensor_count; ++i) {
        mq_sensor_config_t defaults;
        mq_sensor_config_t effective;
        esp_err_t err = board_config_get_default_sensor_config_by_index(i, &defaults);
        if (err == ESP_OK) {
            err = mq_runtime_config_apply_sensor(defaults.id, &defaults, &effective);
        }
        if (err != ESP_OK) {
            if (first_error == ESP_OK) {
                first_error = err;
            }
            continue;
        }
        if (effective.analog_source_id != source_id) {
            continue;
        }

        bool erased = false;
        err = erase_calibration_for_sensor(effective.id, &erased);
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
        if (erased) {
            ++count;
        }
    }

    if (erased_count != NULL) {
        *erased_count = count;
    }
    return first_error;
}

static esp_err_t erase_all_sensor_calibrations(size_t *erased_count)
{
    size_t count = 0;
    esp_err_t first_error = ESP_OK;
    const size_t sensor_count = board_config_mq_sensor_count();
    for (size_t i = 0; i < sensor_count; ++i) {
        mq_sensor_config_t defaults;
        const esp_err_t config_err = board_config_get_default_sensor_config_by_index(i, &defaults);
        if (config_err != ESP_OK) {
            if (first_error == ESP_OK) {
                first_error = config_err;
            }
            continue;
        }

        bool erased = false;
        const esp_err_t err = erase_calibration_for_sensor(defaults.id, &erased);
        if (err != ESP_OK && first_error == ESP_OK) {
            first_error = err;
        }
        if (erased) {
            ++count;
        }
    }

    if (erased_count != NULL) {
        *erased_count = count;
    }
    return first_error;
}

static void print_adc_read(uint8_t logical_channel)
{
    int raw = 0;
    int mv = 0;
    esp_err_t err = adc_service_read_mv(logical_channel, &raw, &mv);
    if (err == ESP_OK) {
        printf("adc %u: raw=%d mV=%d\n", (unsigned)logical_channel, raw, mv);
        return;
    }

    if (err == ESP_ERR_NOT_SUPPORTED) {
        const esp_err_t raw_err = adc_service_read_raw(logical_channel, &raw);
        if (raw_err == ESP_OK) {
            printf("adc %u: raw=%d calibrated_mV=not-supported\n", (unsigned)logical_channel, raw);
            return;
        }
        err = raw_err;
    }

    printf("adc %u read failed: %s\n", (unsigned)logical_channel, esp_err_to_name(err));
}

static void print_all_adc_reads(void)
{
    const size_t count = board_config_internal_adc_channel_count();
    for (size_t channel = 0; channel < count && channel <= UINT8_MAX; ++channel) {
        print_adc_read((uint8_t)channel);
    }
}

static bool sensor_configs_differ(const mq_sensor_config_t *defaults, const mq_sensor_config_t *effective)
{
    return defaults->enabled != effective->enabled ||
           defaults->vc_mv != effective->vc_mv ||
           defaults->rl_ohms != effective->rl_ohms ||
           defaults->warmup_seconds != effective->warmup_seconds ||
           defaults->clean_air_rs_r0_factor != effective->clean_air_rs_r0_factor ||
           defaults->supports_clean_air_calibration != effective->supports_clean_air_calibration;
}

static bool source_configs_differ(const analog_source_config_t *defaults, const analog_source_config_t *effective)
{
    return defaults->type != effective->type ||
           defaults->adc_logical_channel != effective->adc_logical_channel ||
           defaults->input_divider_ratio != effective->input_divider_ratio ||
           defaults->mux_id != effective->mux_id ||
           defaults->mux_channel != effective->mux_channel;
}

static bool mux_configs_differ(const analog_mux_config_t *defaults, const analog_mux_config_t *effective)
{
    return defaults->enabled != effective->enabled ||
           defaults->signal_adc_logical_channel != effective->signal_adc_logical_channel ||
           defaults->gpio_s0 != effective->gpio_s0 ||
           defaults->gpio_s1 != effective->gpio_s1 ||
           defaults->gpio_s2 != effective->gpio_s2 ||
           defaults->gpio_s3 != effective->gpio_s3 ||
           defaults->gpio_en != effective->gpio_en ||
           defaults->en_active_low != effective->en_active_low ||
           defaults->settle_time_us != effective->settle_time_us;
}

static bool aq_configs_differ(const air_quality_service_config_t *defaults,
                              const air_quality_service_config_t *effective)
{
    return defaults->primary_sensor_id != effective->primary_sensor_id ||
           defaults->sample_interval_ms != effective->sample_interval_ms ||
           defaults->stale_after_ms != effective->stale_after_ms ||
           defaults->ema_alpha != effective->ema_alpha ||
           defaults->publish_to_matter != effective->publish_to_matter;
}

static esp_err_t get_sensor_config_for_save(uint8_t id, mq_sensor_config_t *out)
{
    const size_t count = board_config_mq_sensor_count();
    for (size_t i = 0; i < count; ++i) {
        mq_sensor_config_t defaults;
        esp_err_t err = board_config_get_default_sensor_config_by_index(i, &defaults);
        if (err != ESP_OK) {
            return err;
        }
        if (defaults.id == id) {
            return mq_runtime_config_apply_sensor(id, &defaults, out);
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t get_source_config_for_save(uint8_t id, analog_source_config_t *out)
{
    const size_t count = board_config_analog_source_count();
    for (size_t i = 0; i < count; ++i) {
        analog_source_config_t defaults;
        esp_err_t err = board_config_get_default_source_config_by_index(i, &defaults);
        if (err != ESP_OK) {
            return err;
        }
        if (defaults.source_id == id) {
            return mq_runtime_config_apply_source(id, &defaults, out);
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t get_effective_source_config_by_id(uint8_t id, analog_source_config_t *out)
{
    esp_err_t err = analog_backend_get_source_config(id, out);
    if (err == ESP_OK) {
        return ESP_OK;
    }

    const size_t count = board_config_analog_source_count();
    for (size_t i = 0; i < count; ++i) {
        analog_source_config_t source;
        err = board_config_get_effective_source_config_by_index(i, &source);
        if (err != ESP_OK) {
            return err;
        }
        if (source.source_id == id) {
            if (out != NULL) {
                *out = source;
            }
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t get_mux_config_for_save(analog_mux_config_t *out, analog_mux_config_t *defaults_out)
{
    analog_mux_config_t defaults;
    esp_err_t err = board_config_get_default_mux_config(0, &defaults);
    if (err != ESP_OK) {
        return err;
    }
    err = mq_runtime_config_apply_mux(defaults.mux_id, &defaults, out);
    if (err != ESP_OK) {
        return err;
    }
    if (defaults_out != NULL) {
        *defaults_out = defaults;
    }
    return ESP_OK;
}

static void fill_aq_defaults(uint16_t endpoint_id, air_quality_service_config_t *defaults)
{
    memset(defaults, 0, sizeof(*defaults));
    defaults->matter_endpoint_id = endpoint_id;
    defaults->primary_sensor_id = MQ_CONSOLE_DEFAULT_PRIMARY_SENSOR_ID;
    defaults->sample_interval_ms = MQ_CONSOLE_DEFAULT_SAMPLE_INTERVAL_MS;
    defaults->stale_after_ms = MQ_CONSOLE_DEFAULT_STALE_AFTER_MS;
    defaults->ema_alpha = MQ_CONSOLE_DEFAULT_EMA_ALPHA;
    defaults->publish_to_matter = MQ_CONSOLE_DEFAULT_PUBLISH_TO_MATTER;
}

static esp_err_t get_aq_config_for_save(air_quality_service_config_t *out,
                                        air_quality_service_config_t *defaults_out)
{
    air_quality_service_status_t status;
    esp_err_t err = air_quality_service_get_status(&status);
    if (err != ESP_OK) {
        return err;
    }

    air_quality_service_config_t defaults;
    fill_aq_defaults(status.matter_endpoint_id, &defaults);
    err = mq_runtime_config_apply_air_quality(&defaults, out);
    if (err != ESP_OK) {
        return err;
    }
    if (defaults_out != NULL) {
        *defaults_out = defaults;
    }
    return ESP_OK;
}

static esp_err_t find_config_by_id(uint8_t id, mq_sensor_config_t *out)
{
    const size_t count = mq_sensor_count();
    for (size_t i = 0; i < count; ++i) {
        mq_sensor_config_t config;
        const esp_err_t err = mq_sensor_get_config_by_index(i, &config);
        if (err != ESP_OK) {
            return err;
        }
        if (config.id == id) {
            if (out != NULL) {
                *out = config;
            }
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

static void print_sample(const mq_sensor_config_t *config, const mq_sensor_sample_t *sample)
{
    printf("id=%u type=%s state=%s raw=%d measured_mv=%d vrl_mv=%d rs_ohms=%.2f r0_ohms=%.2f ratio=%.3f\n",
           (unsigned)sample->id,
           config != NULL ? mq_sensor_type_to_string(config->type) : "unknown",
           mq_sensor_state_to_string(sample->state),
           sample->raw_adc,
           sample->measured_mv,
           sample->corrected_vrl_mv,
           (double)sample->rs_ohms,
           (double)sample->r0_ohms,
           (double)sample->rs_r0_ratio);
}

static void print_sensors(void)
{
    printf("id type name enabled source source_exists vc_mv rl_ohms clean_air_supported clean_air_factor state r0_ohms\n");
    const size_t count = mq_sensor_count();
    for (size_t i = 0; i < count; ++i) {
        mq_sensor_config_t config;
        esp_err_t err = mq_sensor_get_config_by_index(i, &config);
        if (err != ESP_OK) {
            printf("sensor index %u config error: %s\n", (unsigned)i, esp_err_to_name(err));
            continue;
        }

        mq_sensor_sample_t sample;
        err = mq_sensor_get_last(config.id, &sample);
        if (err != ESP_OK) {
            memset(&sample, 0, sizeof(sample));
            sample.id = config.id;
            sample.state = MQ_SENSOR_STATE_ERROR;
        }

        printf("%u %s %s %s %u %s %u %u %s %.3f %s %.2f\n",
               (unsigned)config.id,
               mq_sensor_type_to_string(config.type),
               config.name,
               config.enabled ? "true" : "false",
               (unsigned)config.analog_source_id,
               analog_source_exists(config.analog_source_id) ? "true" : "false",
               (unsigned)config.vc_mv,
               (unsigned)config.rl_ohms,
               config.supports_clean_air_calibration ? "true" : "false",
               (double)config.clean_air_rs_r0_factor,
               mq_sensor_state_to_string(sample.state),
               (double)sample.r0_ohms);
    }
}

static void print_one_raw(uint8_t id)
{
    mq_sensor_config_t config;
    esp_err_t err = find_config_by_id(id, &config);
    if (err != ESP_OK) {
        printf("sensor %u not found: %s\n", (unsigned)id, esp_err_to_name(err));
        return;
    }

    mq_sensor_sample_t sample;
    err = mq_sensor_read(id, &sample);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE || err == ESP_ERR_INVALID_RESPONSE) {
        print_sample(&config, &sample);
    }
    if (err != ESP_OK) {
        printf("mq raw %u returned: %s\n", (unsigned)id, esp_err_to_name(err));
    }
}

static void print_enabled_raw(void)
{
    size_t skipped = 0;
    size_t printed = 0;
    const size_t count = mq_sensor_count();
    for (size_t i = 0; i < count; ++i) {
        mq_sensor_config_t config;
        esp_err_t err = mq_sensor_get_config_by_index(i, &config);
        if (err != ESP_OK) {
            printf("sensor index %u config error: %s\n", (unsigned)i, esp_err_to_name(err));
            continue;
        }
        if (!config.enabled) {
            ++skipped;
            continue;
        }
        print_one_raw(config.id);
        ++printed;
    }
    printf("raw summary: printed=%u skipped_disabled=%u\n", (unsigned)printed, (unsigned)skipped);
}

static void calibrate_clean(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: mq calibrate-clean <id> [samples] [delay_ms]\n");
        return;
    }

    uint8_t id = 0;
    if (!parse_u8_arg(argv[2], &id)) {
        printf("invalid sensor id: %s\n", argv[2]);
        return;
    }

    size_t samples = MQ_CONSOLE_DEFAULT_CAL_SAMPLES;
    if (argc >= 4 && !parse_size_arg(argv[3], 1, 256, &samples)) {
        printf("invalid sample count: %s\n", argv[3]);
        return;
    }

    uint32_t delay_ms = MQ_CONSOLE_DEFAULT_CAL_DELAY_MS;
    if (argc >= 5 && !parse_u32_arg(argv[4], 5000, &delay_ms)) {
        printf("invalid delay_ms: %s\n", argv[4]);
        return;
    }

    bool restart_aq = false;
    air_quality_service_status_t aq_status;
    esp_err_t aq_err = air_quality_service_get_status(&aq_status);
    if (aq_err == ESP_OK && aq_status.running) {
        aq_err = air_quality_service_stop();
        if (aq_err != ESP_OK) {
            printf("failed to stop AQ service before calibration: %s\n", esp_err_to_name(aq_err));
            return;
        }
        restart_aq = true;
    }

    mq_calibration_record_t record;
    const esp_err_t err = mq_sensor_calibrate_clean_air(id, samples, delay_ms, &record);
    if (err == ESP_OK) {
        printf("calibrated id=%u r0_ohms=%.2f samples=%u note=experimental-clean-air-calibration\n",
               (unsigned)id, (double)record.r0_ohms, (unsigned)samples);
    } else {
        printf("calibrate-clean id=%u failed: %s\n", (unsigned)id, esp_err_to_name(err));
    }

    if (restart_aq) {
        aq_err = air_quality_service_start();
        if (aq_err != ESP_OK) {
            printf("failed to restart AQ service after calibration: %s\n", esp_err_to_name(aq_err));
        }
    }
}

static void print_config(void)
{
    printf("sensor id type enabled source vc_mv rl_ohms warmup clean_factor clean_supported override "
           "cal_valid r0_ohms calibrated_at_unix "
           "default_enabled default_vc_mv default_rl_ohms default_warmup default_clean_factor\n");
    const size_t sensor_count = board_config_mq_sensor_count();
    for (size_t i = 0; i < sensor_count; ++i) {
        mq_sensor_config_t defaults;
        mq_sensor_config_t effective;
        esp_err_t err = board_config_get_default_sensor_config_by_index(i, &defaults);
        if (err == ESP_OK) {
            err = mq_runtime_config_apply_sensor(defaults.id, &defaults, &effective);
        }
        if (err != ESP_OK) {
            printf("sensor index %u config error: %s\n", (unsigned)i, esp_err_to_name(err));
            continue;
        }

        bool cal_valid = false;
        float r0_ohms = 0.0f;
        uint32_t calibrated_at_unix = 0;
        get_calibration_summary(effective.id, &cal_valid, &r0_ohms, &calibrated_at_unix);

        printf("sensor %u %s %s %u %u %u %u %.3f %s %s %s %.2f %u %s %u %u %u %.3f\n",
               (unsigned)effective.id,
               mq_sensor_type_to_string(effective.type),
               effective.enabled ? "true" : "false",
               (unsigned)effective.analog_source_id,
               (unsigned)effective.vc_mv,
               (unsigned)effective.rl_ohms,
               (unsigned)effective.warmup_seconds,
               (double)effective.clean_air_rs_r0_factor,
               effective.supports_clean_air_calibration ? "true" : "false",
               sensor_configs_differ(&defaults, &effective) ? "true" : "false",
               cal_valid ? "true" : "false",
               (double)r0_ohms,
               (unsigned)calibrated_at_unix,
               defaults.enabled ? "true" : "false",
               (unsigned)defaults.vc_mv,
               (unsigned)defaults.rl_ohms,
               (unsigned)defaults.warmup_seconds,
               (double)defaults.clean_air_rs_r0_factor);
    }

    printf("source id type adc_channel divider mux mux_channel override default_type default_adc_channel "
           "default_divider default_mux default_mux_channel\n");
    const size_t source_count = board_config_analog_source_count();
    for (size_t i = 0; i < source_count; ++i) {
        analog_source_config_t defaults;
        analog_source_config_t effective;
        esp_err_t err = board_config_get_default_source_config_by_index(i, &defaults);
        if (err == ESP_OK) {
            err = mq_runtime_config_apply_source(defaults.source_id, &defaults, &effective);
        }
        if (err != ESP_OK) {
            printf("source index %u config error: %s\n", (unsigned)i, esp_err_to_name(err));
            continue;
        }

        printf("source %u %s %u %.3f %u %u %s %s %u %.3f %u %u\n",
               (unsigned)effective.source_id,
               source_type_to_string(effective.type),
               (unsigned)effective.adc_logical_channel,
               (double)effective.input_divider_ratio,
               (unsigned)effective.mux_id,
               (unsigned)effective.mux_channel,
               source_configs_differ(&defaults, &effective) ? "true" : "false",
               source_type_to_string(defaults.type),
               (unsigned)defaults.adc_logical_channel,
               (double)defaults.input_divider_ratio,
               (unsigned)defaults.mux_id,
               (unsigned)defaults.mux_channel);
    }

    analog_mux_config_t mux_defaults;
    analog_mux_config_t mux_effective;
    esp_err_t err = get_mux_config_for_save(&mux_effective, &mux_defaults);
    if (err == ESP_OK) {
        bool ready = false;
        (void)analog_mux_is_ready(mux_effective.mux_id, &ready);
        printf("mux id enabled adc_channel s0 s1 s2 s3 en en_active_low settle_us override ready\n");
        printf("mux %u %s %u %d %d %d %d %d %s %u %s %s\n",
               (unsigned)mux_effective.mux_id,
               mux_effective.enabled ? "true" : "false",
               (unsigned)mux_effective.signal_adc_logical_channel,
               mux_effective.gpio_s0,
               mux_effective.gpio_s1,
               mux_effective.gpio_s2,
               mux_effective.gpio_s3,
               mux_effective.gpio_en,
               mux_effective.en_active_low ? "true" : "false",
               (unsigned)mux_effective.settle_time_us,
               mux_configs_differ(&mux_defaults, &mux_effective) ? "true" : "false",
               ready ? "true" : "false");
    } else {
        printf("mux config error: %s\n", esp_err_to_name(err));
    }

    air_quality_service_config_t defaults;
    air_quality_service_config_t effective;
    err = get_aq_config_for_save(&effective, &defaults);
    if (err != ESP_OK) {
        printf("aq config error: %s\n", esp_err_to_name(err));
        return;
    }

    printf("aq primary sample_interval stale_after ema_alpha publish override "
           "default_primary default_sample_interval default_stale_after default_ema_alpha default_publish\n");
    printf("aq %u %u %u %.3f %u %s %u %u %u %.3f %u\n",
           (unsigned)effective.primary_sensor_id,
           (unsigned)effective.sample_interval_ms,
           (unsigned)effective.stale_after_ms,
           (double)effective.ema_alpha,
           effective.publish_to_matter ? 1U : 0U,
           aq_configs_differ(&defaults, &effective) ? "true" : "false",
           (unsigned)defaults.primary_sensor_id,
           (unsigned)defaults.sample_interval_ms,
           (unsigned)defaults.stale_after_ms,
           (double)defaults.ema_alpha,
           defaults.publish_to_matter ? 1U : 0U);
}

static void print_mux_config(void)
{
    analog_mux_config_t defaults;
    analog_mux_config_t effective;
    const esp_err_t err = get_mux_config_for_save(&effective, &defaults);
    if (err != ESP_OK) {
        printf("mux config error: %s\n", esp_err_to_name(err));
        return;
    }

    bool ready = false;
    (void)analog_mux_is_ready(effective.mux_id, &ready);
    printf("mux id enabled adc_channel s0 s1 s2 s3 en en_active_low settle_us override ready\n");
    printf("mux %u %s %u %d %d %d %d %d %s %u %s %s\n",
           (unsigned)effective.mux_id,
           effective.enabled ? "true" : "false",
           (unsigned)effective.signal_adc_logical_channel,
           effective.gpio_s0,
           effective.gpio_s1,
           effective.gpio_s2,
           effective.gpio_s3,
           effective.gpio_en,
           effective.en_active_low ? "true" : "false",
           (unsigned)effective.settle_time_us,
           mux_configs_differ(&defaults, &effective) ? "true" : "false",
           ready ? "true" : "false");
}

static void print_source_read(uint8_t source_id)
{
    analog_source_config_t source;
    esp_err_t err = get_effective_source_config_by_id(source_id, &source);
    if (err != ESP_OK) {
        printf("source %u not found: %s\n", (unsigned)source_id, esp_err_to_name(err));
        return;
    }

    int actual_adc = source.adc_logical_channel;
    analog_mux_config_t mux_config;
    if (source.type == ANALOG_BACKEND_MUX_ADC && analog_mux_get_config(source.mux_id, &mux_config) == ESP_OK) {
        actual_adc = mux_config.signal_adc_logical_channel;
    }

    int raw = 0;
    int mv = 0;
    err = analog_backend_read_mv(source_id, &raw, &mv);
    if (err == ESP_OK) {
        printf("source=%u type=%s raw=%d mv=%d divider=%.3f configured_adc=%u actual_adc=%d mux=%u channel=%u\n",
               (unsigned)source.source_id,
               source_type_to_string(source.type),
               raw,
               mv,
               (double)source.input_divider_ratio,
               (unsigned)source.adc_logical_channel,
               actual_adc,
               (unsigned)source.mux_id,
               (unsigned)source.mux_channel);
        return;
    }

    printf("source=%u type=%s read_failed=%s divider=%.3f configured_adc=%u actual_adc=%d mux=%u channel=%u\n",
           (unsigned)source.source_id,
           source_type_to_string(source.type),
           esp_err_to_name(err),
           (double)source.input_divider_ratio,
           (unsigned)source.adc_logical_channel,
           actual_adc,
           (unsigned)source.mux_id,
           (unsigned)source.mux_channel);
}

static void print_all_source_reads(void)
{
    const size_t count = board_config_analog_source_count();
    for (size_t i = 0; i < count; ++i) {
        analog_source_config_t source;
        const esp_err_t err = board_config_get_effective_source_config_by_index(i, &source);
        if (err != ESP_OK) {
            printf("source index %u config error: %s\n", (unsigned)i, esp_err_to_name(err));
            continue;
        }
        print_source_read(source.source_id);
    }
}

static void mux_select_command(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: mq mux-select <channel>\n");
        return;
    }

    uint8_t channel = 0;
    if (!parse_u8_arg(argv[2], &channel) || channel > 15) {
        printf("invalid mux channel: %s\n", argv[2]);
        return;
    }

    const esp_err_t err = analog_backend_select_mux_channel(0, channel);
    if (err == ESP_OK) {
        printf("mux-select channel=%u ok=true\n", (unsigned)channel);
    } else {
        printf("mux-select channel=%u ok=false error=%s\n", (unsigned)channel, esp_err_to_name(err));
    }
}

static void mux_scan_command(int argc, char **argv)
{
    uint8_t first = 0;
    uint8_t last = 8;
    size_t samples = 8;

    if (argc >= 3 && (!parse_u8_arg(argv[2], &first) || first > 15)) {
        printf("invalid first mux channel: %s\n", argv[2]);
        return;
    }
    if (argc >= 4 && (!parse_u8_arg(argv[3], &last) || last > 15 || last < first)) {
        printf("invalid last mux channel: %s\n", argv[3]);
        return;
    }
    if (argc >= 5 && !parse_size_arg(argv[4], 1, 256, &samples)) {
        printf("invalid sample count: %s\n", argv[4]);
        return;
    }

    bool ready = false;
    esp_err_t err = analog_mux_is_ready(0, &ready);
    if (err != ESP_OK || !ready) {
        printf("mux-scan failed: mux-not-ready error=%s\n", esp_err_to_name(err));
        return;
    }

    analog_mux_config_t mux_config;
    err = analog_mux_get_config(0, &mux_config);
    if (err != ESP_OK) {
        printf("mux-scan failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("mux-scan adc_channel=%u samples=%u\n",
           (unsigned)mux_config.signal_adc_logical_channel,
           (unsigned)samples);
    for (uint8_t channel = first; channel <= last; ++channel) {
        int64_t raw_sum = 0;
        int64_t mv_sum = 0;
        int raw_min = INT32_MAX;
        int raw_max = INT32_MIN;
        size_t ok_count = 0;
        size_t failures = 0;

        for (size_t i = 0; i < samples; ++i) {
            int raw = 0;
            int mv = 0;
            err = analog_backend_read_mux_channel_mv(0, channel, &raw, &mv);
            if (err != ESP_OK) {
                ++failures;
                continue;
            }
            raw_sum += raw;
            mv_sum += mv;
            if (raw < raw_min) {
                raw_min = raw;
            }
            if (raw > raw_max) {
                raw_max = raw;
            }
            ++ok_count;
        }

        if (ok_count == 0) {
            printf("channel=%u raw_avg=0 mv_avg=0 raw_min=0 raw_max=0 failures=%u\n",
                   (unsigned)channel, (unsigned)failures);
        } else {
            printf("channel=%u raw_avg=%d mv_avg=%d raw_min=%d raw_max=%d failures=%u\n",
                   (unsigned)channel,
                   (int)(raw_sum / (int64_t)ok_count),
                   (int)(mv_sum / (int64_t)ok_count),
                   raw_min,
                   raw_max,
                   (unsigned)failures);
        }
    }
}

static bool analog_source_exists(uint8_t source_id)
{
    if (analog_backend_source_is_registered(source_id)) {
        return true;
    }

    const size_t source_count = board_config_analog_source_count();
    for (size_t i = 0; i < source_count; ++i) {
        analog_source_config_t source;
        if (board_config_get_default_source_config_by_index(i, &source) == ESP_OK &&
            source.source_id == source_id) {
            return true;
        }
    }
    return false;
}

static bool sensor_exists(uint8_t sensor_id)
{
    const size_t sensor_count = board_config_mq_sensor_count();
    for (size_t i = 0; i < sensor_count; ++i) {
        mq_sensor_config_t sensor;
        if (board_config_get_default_sensor_config_by_index(i, &sensor) == ESP_OK &&
            sensor.id == sensor_id) {
            return true;
        }
    }
    return false;
}

static void print_cal_status(void)
{
    printf("id type enabled source cal_valid r0_ohms calibrated_at_unix state\n");
    const size_t sensor_count = board_config_mq_sensor_count();
    for (size_t i = 0; i < sensor_count; ++i) {
        mq_sensor_config_t config;
        esp_err_t err = mq_sensor_get_config_by_index(i, &config);
        if (err != ESP_OK) {
            err = board_config_get_effective_sensor_config_by_index(i, &config);
        }
        if (err != ESP_OK) {
            printf("sensor index %u config error: %s\n", (unsigned)i, esp_err_to_name(err));
            continue;
        }

        bool cal_valid = false;
        float r0_ohms = 0.0f;
        uint32_t calibrated_at_unix = 0;
        get_calibration_summary(config.id, &cal_valid, &r0_ohms, &calibrated_at_unix);

        mq_sensor_sample_t sample;
        err = mq_sensor_get_last(config.id, &sample);
        if (err != ESP_OK) {
            memset(&sample, 0, sizeof(sample));
            sample.id = config.id;
            sample.state = MQ_SENSOR_STATE_ERROR;
        }

        printf("%u %s %s %u %s %.2f %u %s\n",
               (unsigned)config.id,
               mq_sensor_type_to_string(config.type),
               config.enabled ? "true" : "false",
               (unsigned)config.analog_source_id,
               cal_valid ? "true" : "false",
               (double)r0_ohms,
               (unsigned)calibrated_at_unix,
               mq_sensor_state_to_string(sample.state));
    }
}

static bool mux_config_ready_for_boot(const analog_mux_config_t *mux)
{
    return mux != NULL &&
           mux->enabled &&
           mux->signal_adc_logical_channel <= 4 &&
           mux->settle_time_us >= 50U &&
           mux->settle_time_us <= 5000U &&
           mux->gpio_s0 != ANALOG_MUX_GPIO_UNUSED &&
           mux->gpio_s1 != ANALOG_MUX_GPIO_UNUSED &&
           mux->gpio_s2 != ANALOG_MUX_GPIO_UNUSED &&
           mux->gpio_s3 != ANALOG_MUX_GPIO_UNUSED &&
           mux->gpio_s0 >= 0 && mux->gpio_s0 < GPIO_NUM_MAX &&
           mux->gpio_s1 >= 0 && mux->gpio_s1 < GPIO_NUM_MAX &&
           mux->gpio_s2 >= 0 && mux->gpio_s2 < GPIO_NUM_MAX &&
           mux->gpio_s3 >= 0 && mux->gpio_s3 < GPIO_NUM_MAX &&
           (mux->gpio_en == ANALOG_MUX_GPIO_UNUSED ||
            (mux->gpio_en >= 0 && mux->gpio_en < GPIO_NUM_MAX));
}

static bool mux_has_duplicate_gpios(const analog_mux_config_t *mux)
{
    if (mux == NULL) {
        return false;
    }

    const int gpios[] = {
        mux->gpio_s0,
        mux->gpio_s1,
        mux->gpio_s2,
        mux->gpio_s3,
        mux->gpio_en,
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

static void print_config_validate(void)
{
    size_t errors = 0;
    size_t warnings = 0;

    analog_mux_config_t mux;
    esp_err_t mux_err = get_mux_config_for_save(&mux, NULL);
    const bool mux_config_available = mux_err == ESP_OK;
    const bool mux_ready_after_reboot = mux_config_ready_for_boot(&mux);
    if (!mux_config_available) {
        printf("error mux config: %s\n", esp_err_to_name(mux_err));
        ++errors;
    } else if (mux.enabled && !mux_ready_after_reboot) {
        printf("error mux 0 enabled but pins or timing are incomplete\n");
        ++errors;
    }
    if (mux_config_available && mux_has_duplicate_gpios(&mux)) {
        printf("error mux 0 has duplicate GPIO assignments\n");
        ++errors;
    }

    const size_t source_count = board_config_analog_source_count();
    for (size_t i = 0; i < source_count; ++i) {
        analog_source_config_t defaults;
        analog_source_config_t source;
        esp_err_t err = board_config_get_default_source_config_by_index(i, &defaults);
        if (err == ESP_OK) {
            err = mq_runtime_config_apply_source(defaults.source_id, &defaults, &source);
        }
        if (err != ESP_OK) {
            printf("error source index %u: %s\n", (unsigned)i, esp_err_to_name(err));
            ++errors;
            continue;
        }
        if (source.input_divider_ratio < 0.1f || source.input_divider_ratio > 20.0f) {
            printf("error source %u divider %.3f outside 0.1..20.0\n",
                   (unsigned)source.source_id, (double)source.input_divider_ratio);
            ++errors;
        }
        if (source.type == ANALOG_BACKEND_EXTERNAL_ADC) {
            printf("error source %u external ADC backend is not implemented\n", (unsigned)source.source_id);
            ++errors;
        }
        if (source.type == ANALOG_BACKEND_INTERNAL_ADC && source.adc_logical_channel > 4) {
            printf("error source %u internal ADC channel %u outside 0..4\n",
                   (unsigned)source.source_id, (unsigned)source.adc_logical_channel);
            ++errors;
        }
        if (source.type == ANALOG_BACKEND_MUX_ADC) {
            if (source.mux_id != 0) {
                printf("error source %u mux id %u unsupported\n",
                       (unsigned)source.source_id, (unsigned)source.mux_id);
                ++errors;
            }
            if (source.mux_channel > 15) {
                printf("error source %u mux channel %u outside 0..15\n",
                       (unsigned)source.source_id, (unsigned)source.mux_channel);
                ++errors;
            }
            if (!mux_config_available || !mux.enabled) {
                printf("warning source %u is mux-backed while mux 0 is disabled\n",
                       (unsigned)source.source_id);
                ++warnings;
            }
        }
    }

    const size_t sensor_count = board_config_mq_sensor_count();
    for (size_t i = 0; i < sensor_count; ++i) {
        mq_sensor_config_t defaults;
        mq_sensor_config_t sensor;
        esp_err_t err = board_config_get_default_sensor_config_by_index(i, &defaults);
        if (err == ESP_OK) {
            err = mq_runtime_config_apply_sensor(defaults.id, &defaults, &sensor);
        }
        if (err != ESP_OK) {
            printf("error sensor index %u: %s\n", (unsigned)i, esp_err_to_name(err));
            ++errors;
            continue;
        }

        analog_source_config_t sensor_source;
        const esp_err_t source_err = get_source_config_for_save(sensor.analog_source_id, &sensor_source);
        if (sensor.enabled && source_err != ESP_OK) {
            printf("error sensor %u enabled with missing analog source %u\n",
                   (unsigned)sensor.id, (unsigned)sensor.analog_source_id);
            ++errors;
        } else if (sensor.enabled && sensor_source.type == ANALOG_BACKEND_MUX_ADC) {
            if (!mux_config_available || !mux.enabled) {
                printf("error sensor %u enabled with mux source while mux 0 is disabled\n",
                       (unsigned)sensor.id);
                ++errors;
            } else if (!mux_ready_after_reboot) {
                printf("error sensor %u enabled with mux source while mux 0 is not ready\n",
                       (unsigned)sensor.id);
                ++errors;
            }
        } else if (sensor.enabled && sensor.id > 4) {
            printf("error sensor %u enabled without mux-backed source support\n", (unsigned)sensor.id);
            ++errors;
        }
        if (sensor.vc_mv < 1000U || sensor.vc_mv > 10000U) {
            printf("error sensor %u vc_mv %u outside 1000..10000\n",
                   (unsigned)sensor.id, (unsigned)sensor.vc_mv);
            ++errors;
        }
        if (sensor.rl_ohms < 1000U || sensor.rl_ohms > 1000000U) {
            printf("error sensor %u rl_ohms %u outside 1000..1000000\n",
                   (unsigned)sensor.id, (unsigned)sensor.rl_ohms);
            ++errors;
        }
        if (sensor.warmup_seconds > 172800U) {
            printf("error sensor %u warmup_seconds %u > 172800\n",
                   (unsigned)sensor.id, (unsigned)sensor.warmup_seconds);
            ++errors;
        }
        if (sensor.supports_clean_air_calibration && sensor.clean_air_rs_r0_factor <= 0.0f) {
            printf("error sensor %u clean-air capable with non-positive factor %.3f\n",
                   (unsigned)sensor.id, (double)sensor.clean_air_rs_r0_factor);
            ++errors;
        }
    }

    air_quality_service_config_t aq_config;
    esp_err_t err = get_aq_config_for_save(&aq_config, NULL);
    if (err != ESP_OK) {
        printf("error aq config: %s\n", esp_err_to_name(err));
        ++errors;
    } else {
        if (!sensor_exists(aq_config.primary_sensor_id)) {
            printf("error aq primary sensor %u does not exist\n", (unsigned)aq_config.primary_sensor_id);
            ++errors;
        } else {
            mq_sensor_config_t primary;
            err = get_sensor_config_for_save(aq_config.primary_sensor_id, &primary);
            if (err == ESP_OK && (primary.type == MQ_SENSOR_MQ7 || primary.type == MQ_SENSOR_MQ9)) {
                printf("error aq primary sensor %u is MQ-7/MQ-9 raw-diagnostic only\n",
                       (unsigned)aq_config.primary_sensor_id);
                ++errors;
            }
            if (err == ESP_OK && !primary.enabled) {
                printf("warning aq primary sensor %u is not enabled\n", (unsigned)aq_config.primary_sensor_id);
                ++warnings;
            }
            if (err == ESP_OK && !analog_source_exists(primary.analog_source_id)) {
                printf("error aq primary sensor %u uses missing source %u\n",
                       (unsigned)aq_config.primary_sensor_id, (unsigned)primary.analog_source_id);
                ++errors;
            }
            if (err == ESP_OK) {
                analog_source_config_t primary_source;
                const esp_err_t source_err = get_source_config_for_save(primary.analog_source_id, &primary_source);
                if (source_err == ESP_OK &&
                    primary_source.type == ANALOG_BACKEND_MUX_ADC &&
                    (!mux_config_available || !mux.enabled)) {
                    printf("error aq primary sensor %u uses mux source while mux 0 is disabled\n",
                           (unsigned)aq_config.primary_sensor_id);
                    ++errors;
                }
            }
        }
        if (aq_config.stale_after_ms < aq_config.sample_interval_ms) {
            printf("error aq stale_after_ms %u < sample_interval_ms %u\n",
                   (unsigned)aq_config.stale_after_ms, (unsigned)aq_config.sample_interval_ms);
            ++errors;
        }
    }

    printf("validation errors=%u warnings=%u\n", (unsigned)errors, (unsigned)warnings);
}

static void erase_calibration(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: mq erase-cal <id|all>\n");
        return;
    }

    if (strcmp(argv[2], "all") == 0) {
        size_t cleared = 0;
        const size_t count = mq_sensor_count();
        for (size_t i = 0; i < count; ++i) {
            mq_sensor_config_t config;
            esp_err_t err = mq_sensor_get_config_by_index(i, &config);
            if (err != ESP_OK) {
                printf("sensor index %u config error: %s\n", (unsigned)i, esp_err_to_name(err));
                continue;
            }
            err = mq_sensor_erase_calibration(config.id);
            if (err == ESP_OK) {
                ++cleared;
            } else {
                printf("erase-cal id=%u failed: %s\n", (unsigned)config.id, esp_err_to_name(err));
            }
        }
        printf("erased calibration records for %u sensors\n", (unsigned)cleared);
        return;
    }

    uint8_t id = 0;
    if (!parse_u8_arg(argv[2], &id)) {
        printf("invalid sensor id: %s\n", argv[2]);
        return;
    }

    const esp_err_t err = mq_sensor_erase_calibration(id);
    if (err == ESP_OK) {
        printf("erased calibration id=%u\n", (unsigned)id);
    } else {
        printf("erase-cal id=%u failed: %s\n", (unsigned)id, esp_err_to_name(err));
    }
}

static void config_reset(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: mq config-reset <all|sensor|source|mux|aq> [id]\n");
        return;
    }

    esp_err_t err = ESP_ERR_INVALID_ARG;
    esp_err_t calibration_err = ESP_OK;
    size_t calibration_erased = 0;
    bool report_calibration = false;
    if (strcmp(argv[2], "all") == 0) {
        err = mq_runtime_config_erase_all();
        if (err == ESP_OK) {
            calibration_err = erase_all_sensor_calibrations(&calibration_erased);
            report_calibration = true;
        }
    } else if (strcmp(argv[2], "sensor") == 0) {
        if (argc < 4) {
            printf("usage: mq config-reset sensor <id>\n");
            return;
        }
        uint8_t id = 0;
        if (!parse_u8_arg(argv[3], &id)) {
            printf("invalid sensor id: %s\n", argv[3]);
            return;
        }
        err = mq_runtime_config_erase_sensor(id);
        if (err == ESP_OK) {
            bool erased = false;
            calibration_err = erase_calibration_for_sensor(id, &erased);
            calibration_erased = erased ? 1U : 0U;
            report_calibration = true;
        }
    } else if (strcmp(argv[2], "source") == 0) {
        if (argc < 4) {
            printf("usage: mq config-reset source <id>\n");
            return;
        }
        uint8_t id = 0;
        if (!parse_u8_arg(argv[3], &id)) {
            printf("invalid source id: %s\n", argv[3]);
            return;
        }
        err = mq_runtime_config_erase_source(id);
        if (err == ESP_OK) {
            calibration_err = erase_calibrations_for_source(id, &calibration_erased);
            report_calibration = true;
        }
    } else if (strcmp(argv[2], "mux") == 0) {
        uint8_t id = 0;
        if (argc >= 4 && !parse_u8_arg(argv[3], &id)) {
            printf("invalid mux id: %s\n", argv[3]);
            return;
        }
        err = mq_runtime_config_erase_mux(id);
        if (err == ESP_OK) {
            calibration_err = erase_all_sensor_calibrations(&calibration_erased);
            report_calibration = true;
        }
    } else if (strcmp(argv[2], "aq") == 0) {
        err = mq_runtime_config_erase_air_quality();
    } else {
        printf("unknown config-reset target: %s\n", argv[2]);
        return;
    }

    if (report_calibration) {
        print_save_result_with_calibration(err, calibration_err, calibration_erased);
    } else {
        print_save_result(err);
    }
}

static void save_sensor_enabled(int argc, char **argv, bool enabled)
{
    if (argc < 3) {
        printf("usage: mq %s <id>\n", enabled ? "enable" : "disable");
        return;
    }

    uint8_t id = 0;
    if (!parse_u8_arg(argv[2], &id)) {
        printf("invalid sensor id: %s\n", argv[2]);
        return;
    }
    if (enabled && !board_config_sensor_can_be_enabled(id)) {
        printf("saved=false error=%s note=sensors-5-8-require-mux-or-external-adc\n",
               esp_err_to_name(ESP_ERR_NOT_SUPPORTED));
        return;
    }

    mq_sensor_config_t config;
    esp_err_t err = get_sensor_config_for_save(id, &config);
    if (err == ESP_OK) {
        config.enabled = enabled;
        err = mq_runtime_config_save_sensor(&config);
    }
    print_save_result(err);
}

static void set_sensor_u32(int argc, char **argv, const char *name)
{
    if (argc < 4) {
        printf("usage: mq %s <id> <value>\n", argv[1]);
        return;
    }

    uint8_t id = 0;
    if (!parse_u8_arg(argv[2], &id)) {
        printf("invalid sensor id: %s\n", argv[2]);
        return;
    }

    uint32_t value = 0;
    if (!parse_u32_arg(argv[3], UINT32_MAX, &value)) {
        printf("invalid value: %s\n", argv[3]);
        return;
    }

    mq_sensor_config_t config;
    esp_err_t err = get_sensor_config_for_save(id, &config);
    const bool invalidate_calibration = strcmp(name, "rl") == 0 || strcmp(name, "vc") == 0;
    if (err == ESP_OK) {
        if (strcmp(name, "rl") == 0) {
            config.rl_ohms = value;
        } else if (strcmp(name, "vc") == 0) {
            config.vc_mv = value;
        } else if (strcmp(name, "warmup") == 0) {
            config.warmup_seconds = value;
        }
        err = mq_runtime_config_save_sensor(&config);
    }
    if (err == ESP_OK && invalidate_calibration) {
        bool erased = false;
        const esp_err_t erase_err = erase_calibration_for_sensor(id, &erased);
        print_save_result_with_calibration(err, erase_err, erased ? 1U : 0U);
    } else {
        print_save_result(err);
    }
}

static void set_clean_factor(int argc, char **argv)
{
    if (argc < 4) {
        printf("usage: mq set-clean-factor <id> <factor>\n");
        return;
    }

    uint8_t id = 0;
    if (!parse_u8_arg(argv[2], &id)) {
        printf("invalid sensor id: %s\n", argv[2]);
        return;
    }

    float factor = 0.0f;
    if (!parse_float_arg(argv[3], 0.0f, 100.0f, &factor)) {
        printf("invalid clean factor: %s\n", argv[3]);
        return;
    }

    mq_sensor_config_t config;
    esp_err_t err = get_sensor_config_for_save(id, &config);
    if (err == ESP_OK) {
        config.clean_air_rs_r0_factor = factor;
        err = mq_runtime_config_save_sensor(&config);
    }
    if (err == ESP_OK) {
        bool erased = false;
        const esp_err_t erase_err = erase_calibration_for_sensor(id, &erased);
        print_save_result_with_calibration(err, erase_err, erased ? 1U : 0U);
    } else {
        print_save_result(err);
    }
}

static void set_source_divider(int argc, char **argv)
{
    if (argc < 4) {
        printf("usage: mq set-source-divider <source_id> <ratio>\n");
        return;
    }

    uint8_t source_id = 0;
    if (!parse_u8_arg(argv[2], &source_id)) {
        printf("invalid source id: %s\n", argv[2]);
        return;
    }

    float ratio = 0.0f;
    if (!parse_float_arg(argv[3], 0.1f, 20.0f, &ratio)) {
        printf("invalid source divider: %s\n", argv[3]);
        return;
    }

    analog_source_config_t config;
    esp_err_t err = get_source_config_for_save(source_id, &config);
    if (err == ESP_OK) {
        config.input_divider_ratio = ratio;
        err = mq_runtime_config_save_source(&config);
    }
    if (err == ESP_OK) {
        size_t erased_count = 0;
        const esp_err_t erase_err = erase_calibrations_for_source(source_id, &erased_count);
        print_save_result_with_calibration(err, erase_err, erased_count);
    } else {
        print_save_result(err);
    }
}

static void mux_set_pins(int argc, char **argv)
{
    if (argc < 7) {
        printf("usage: mq mux-set-pins <adc_logical_channel> <s0> <s1> <s2> <s3> [en|-1] [settle_us]\n");
        return;
    }

    uint8_t adc_logical_channel = 0;
    if (!parse_u8_arg(argv[2], &adc_logical_channel)) {
        printf("invalid adc logical channel: %s\n", argv[2]);
        return;
    }

    int32_t gpio_s0 = 0;
    int32_t gpio_s1 = 0;
    int32_t gpio_s2 = 0;
    int32_t gpio_s3 = 0;
    if (!parse_i32_arg(argv[3], ANALOG_MUX_GPIO_UNUSED, GPIO_NUM_MAX - 1, &gpio_s0) ||
        !parse_i32_arg(argv[4], ANALOG_MUX_GPIO_UNUSED, GPIO_NUM_MAX - 1, &gpio_s1) ||
        !parse_i32_arg(argv[5], ANALOG_MUX_GPIO_UNUSED, GPIO_NUM_MAX - 1, &gpio_s2) ||
        !parse_i32_arg(argv[6], ANALOG_MUX_GPIO_UNUSED, GPIO_NUM_MAX - 1, &gpio_s3)) {
        printf("invalid mux select GPIO\n");
        return;
    }

    analog_mux_config_t config;
    esp_err_t err = get_mux_config_for_save(&config, NULL);
    if (err == ESP_OK) {
        config.signal_adc_logical_channel = adc_logical_channel;
        config.gpio_s0 = (int)gpio_s0;
        config.gpio_s1 = (int)gpio_s1;
        config.gpio_s2 = (int)gpio_s2;
        config.gpio_s3 = (int)gpio_s3;
        if (argc >= 8) {
            int32_t gpio_en = 0;
            if (!parse_i32_arg(argv[7], ANALOG_MUX_GPIO_UNUSED, GPIO_NUM_MAX - 1, &gpio_en)) {
                printf("invalid mux EN GPIO: %s\n", argv[7]);
                return;
            }
            config.gpio_en = (int)gpio_en;
        }
        if (argc >= 9 && !parse_u32_arg(argv[8], 5000U, &config.settle_time_us)) {
            printf("invalid settle_us: %s\n", argv[8]);
            return;
        }
        err = mq_runtime_config_save_mux(&config);
    }

    if (err == ESP_OK) {
        size_t erased_count = 0;
        const esp_err_t erase_err = erase_all_sensor_calibrations(&erased_count);
        print_save_result_with_calibration(err, erase_err, erased_count);
    } else {
        print_save_result(err);
    }
}

static void mux_enable(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: mq mux-enable <0|1>\n");
        return;
    }

    uint8_t enabled = 0;
    if (!parse_u8_arg(argv[2], &enabled) || enabled > 1) {
        printf("invalid mux-enable value: %s\n", argv[2]);
        return;
    }

    analog_mux_config_t config;
    esp_err_t err = get_mux_config_for_save(&config, NULL);
    if (err == ESP_OK) {
        config.enabled = enabled != 0;
        err = mq_runtime_config_save_mux(&config);
    }
    print_save_result(err);
}

static void set_source_mux(int argc, char **argv)
{
    if (argc < 4) {
        printf("usage: mq set-source-mux <source_id> <mux_channel>\n");
        return;
    }

    uint8_t source_id = 0;
    uint8_t mux_channel = 0;
    if (!parse_u8_arg(argv[2], &source_id)) {
        printf("invalid source id: %s\n", argv[2]);
        return;
    }
    if (!parse_u8_arg(argv[3], &mux_channel)) {
        printf("invalid mux channel: %s\n", argv[3]);
        return;
    }

    analog_mux_config_t mux;
    analog_source_config_t config;
    esp_err_t err = get_mux_config_for_save(&mux, NULL);
    if (err == ESP_OK) {
        err = get_source_config_for_save(source_id, &config);
    }
    if (err == ESP_OK) {
        config.type = ANALOG_BACKEND_MUX_ADC;
        config.adc_logical_channel = mux.signal_adc_logical_channel;
        config.mux_id = mux.mux_id;
        config.mux_channel = mux_channel;
        err = mq_runtime_config_save_source(&config);
    }

    if (err == ESP_OK) {
        size_t erased_count = 0;
        const esp_err_t erase_err = erase_calibrations_for_source(source_id, &erased_count);
        print_save_result_with_calibration(err, erase_err, erased_count);
    } else {
        print_save_result(err);
    }
}

static void set_source_internal(int argc, char **argv)
{
    if (argc < 4) {
        printf("usage: mq set-source-internal <source_id> <adc_logical_channel>\n");
        return;
    }

    uint8_t source_id = 0;
    uint8_t adc_logical_channel = 0;
    if (!parse_u8_arg(argv[2], &source_id)) {
        printf("invalid source id: %s\n", argv[2]);
        return;
    }
    if (!parse_u8_arg(argv[3], &adc_logical_channel)) {
        printf("invalid adc logical channel: %s\n", argv[3]);
        return;
    }

    analog_source_config_t config;
    esp_err_t err = get_source_config_for_save(source_id, &config);
    if (err == ESP_OK) {
        config.type = ANALOG_BACKEND_INTERNAL_ADC;
        config.adc_logical_channel = adc_logical_channel;
        config.mux_id = 0;
        config.mux_channel = 0;
        err = mq_runtime_config_save_source(&config);
    }

    if (err == ESP_OK) {
        size_t erased_count = 0;
        const esp_err_t erase_err = erase_calibrations_for_source(source_id, &erased_count);
        print_save_result_with_calibration(err, erase_err, erased_count);
    } else {
        print_save_result(err);
    }
}

static void mux_use_kit_default(void)
{
    analog_mux_config_t mux;
    esp_err_t first_error = get_mux_config_for_save(&mux, NULL);
    const size_t source_count = board_config_analog_source_count();
    for (size_t i = 0; first_error == ESP_OK && i < source_count; ++i) {
        analog_source_config_t defaults;
        esp_err_t err = board_config_get_default_source_config_by_index(i, &defaults);
        if (err != ESP_OK) {
            first_error = err;
            break;
        }
        analog_source_config_t config;
        err = get_source_config_for_save(defaults.source_id, &config);
        if (err != ESP_OK) {
            first_error = err;
            break;
        }
        config.type = ANALOG_BACKEND_MUX_ADC;
        config.adc_logical_channel = mux.signal_adc_logical_channel;
        config.mux_id = mux.mux_id;
        config.mux_channel = config.source_id;
        err = mq_runtime_config_save_source(&config);
        if (err != ESP_OK) {
            first_error = err;
            break;
        }
    }

    if (first_error == ESP_OK) {
        size_t erased_count = 0;
        const esp_err_t erase_err = erase_all_sensor_calibrations(&erased_count);
        print_save_result_with_calibration(first_error, erase_err, erased_count);
    } else {
        print_save_result(first_error);
    }
}

static void save_aq_config(air_quality_service_config_t *config)
{
    print_save_result(mq_runtime_config_save_air_quality(config));
}

static void set_primary_sensor(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: mq set-primary <id>\n");
        return;
    }

    uint8_t id = 0;
    if (!parse_u8_arg(argv[2], &id)) {
        printf("invalid sensor id: %s\n", argv[2]);
        return;
    }
    mq_sensor_config_t sensor;
    esp_err_t err = get_sensor_config_for_save(id, &sensor);
    if (err != ESP_OK) {
        print_save_result(err);
        return;
    }
    if (sensor.type == MQ_SENSOR_MQ7 || sensor.type == MQ_SENSOR_MQ9) {
        printf("saved=false error=%s note=MQ-7/MQ-9-are-raw-diagnostic-only\n",
               esp_err_to_name(ESP_ERR_NOT_SUPPORTED));
        return;
    }
    if (id > 4) {
        analog_mux_config_t mux;
        analog_source_config_t source;
        err = get_mux_config_for_save(&mux, NULL);
        if (err == ESP_OK) {
            err = get_source_config_for_save(sensor.analog_source_id, &source);
        }
        if (err != ESP_OK || !sensor.enabled || !mux.enabled || source.type != ANALOG_BACKEND_MUX_ADC) {
            printf("saved=false error=%s note=primary-sensors-5-8-require-enabled-mux-sensor-after-reboot\n",
                   esp_err_to_name(err == ESP_OK ? ESP_ERR_NOT_SUPPORTED : err));
            return;
        }
    }

    air_quality_service_config_t config;
    err = get_aq_config_for_save(&config, NULL);
    if (err == ESP_OK) {
        config.primary_sensor_id = id;
        save_aq_config(&config);
    } else {
        print_save_result(err);
    }
}

static void set_aq_u32(int argc, char **argv, const char *name)
{
    if (argc < 3) {
        printf("usage: mq %s <ms>\n", argv[1]);
        return;
    }

    uint32_t value = 0;
    if (!parse_u32_arg(argv[2], UINT32_MAX, &value)) {
        printf("invalid value: %s\n", argv[2]);
        return;
    }

    air_quality_service_config_t config;
    esp_err_t err = get_aq_config_for_save(&config, NULL);
    if (err == ESP_OK) {
        if (strcmp(name, "sample") == 0) {
            config.sample_interval_ms = value;
        } else if (strcmp(name, "stale") == 0) {
            config.stale_after_ms = value;
        }
        save_aq_config(&config);
    } else {
        print_save_result(err);
    }
}

static void set_ema_alpha(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: mq set-ema-alpha <alpha>\n");
        return;
    }

    float alpha = 0.0f;
    if (!parse_float_arg(argv[2], 0.000001f, 1.0f, &alpha)) {
        printf("invalid ema alpha: %s\n", argv[2]);
        return;
    }

    air_quality_service_config_t config;
    esp_err_t err = get_aq_config_for_save(&config, NULL);
    if (err == ESP_OK) {
        config.ema_alpha = alpha;
        save_aq_config(&config);
    } else {
        print_save_result(err);
    }
}

static void set_matter_publish(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: mq matter-publish <0|1>\n");
        return;
    }

    uint8_t publish = 0;
    if (!parse_u8_arg(argv[2], &publish) || publish > 1) {
        printf("invalid matter-publish value: %s\n", argv[2]);
        return;
    }

    air_quality_service_config_t config;
    esp_err_t err = get_aq_config_for_save(&config, NULL);
    if (err == ESP_OK) {
        config.publish_to_matter = publish != 0;
        save_aq_config(&config);
    } else {
        print_save_result(err);
    }
}

static void print_aq_status(void)
{
    air_quality_service_status_t status;
    const esp_err_t err = air_quality_service_get_status(&status);
    if (err != ESP_OK) {
        printf("aq-status failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("running=%s endpoint=%u primary_sensor=%u level=%s last_published=%s "
           "filtered_ratio=%.3f last_success_ms=%u last_sample_age_ms=%u "
           "raw=%d measured_mv=%d vrl_mv=%d rs=%.2f r0=%.2f ratio=%.3f "
           "ok_reads=%u fail_reads=%u matter_updates=%u last_error=%s\n",
           status.running ? "true" : "false",
           (unsigned)status.matter_endpoint_id,
           (unsigned)status.primary_sensor_id,
           air_quality_service_level_to_string(status.current_level),
           air_quality_service_level_to_string(status.last_published_level),
           (double)status.filtered_ratio,
           (unsigned)status.last_success_ms,
           (unsigned)status.last_sample_age_ms,
           status.last_sample.raw_adc,
           status.last_sample.measured_mv,
           status.last_sample.corrected_vrl_mv,
           (double)status.last_sample.rs_ohms,
           (double)status.last_sample.r0_ohms,
           (double)status.last_sample.rs_r0_ratio,
           (unsigned)status.successful_reads,
           (unsigned)status.failed_reads,
           (unsigned)status.matter_updates,
           esp_err_to_name(status.last_error));
}

static int mq_command(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: mq <status|sensors|adc|raw|source-read|calibrate-clean|erase-cal|cal-status|config|config-validate|config-reset|mux-config|mux-set-pins|mux-enable|mux-select|mux-scan|mux-use-kit-default|enable|disable|set-rl|set-vc|set-warmup|set-clean-factor|set-source-divider|set-source-mux|set-source-internal|set-primary|set-sample-interval|set-stale-after|set-ema-alpha|matter-publish|aq-status|aq-start|aq-stop>\n");
        return 0;
    }

    if (strcmp(argv[1], "status") == 0) {
        printf("MQ scaffold active, configured sensors: %u\n", (unsigned)mq_sensor_count());
    } else if (strcmp(argv[1], "config") == 0) {
        print_config();
    } else if (strcmp(argv[1], "config-validate") == 0) {
        print_config_validate();
    } else if (strcmp(argv[1], "mux-config") == 0) {
        print_mux_config();
    } else if (strcmp(argv[1], "mux-set-pins") == 0) {
        mux_set_pins(argc, argv);
    } else if (strcmp(argv[1], "mux-enable") == 0) {
        mux_enable(argc, argv);
    } else if (strcmp(argv[1], "mux-select") == 0) {
        mux_select_command(argc, argv);
    } else if (strcmp(argv[1], "mux-scan") == 0) {
        mux_scan_command(argc, argv);
    } else if (strcmp(argv[1], "mux-use-kit-default") == 0) {
        mux_use_kit_default();
    } else if (strcmp(argv[1], "cal-status") == 0) {
        print_cal_status();
    } else if (strcmp(argv[1], "config-reset") == 0) {
        config_reset(argc, argv);
    } else if (strcmp(argv[1], "enable") == 0) {
        save_sensor_enabled(argc, argv, true);
    } else if (strcmp(argv[1], "disable") == 0) {
        save_sensor_enabled(argc, argv, false);
    } else if (strcmp(argv[1], "set-rl") == 0) {
        set_sensor_u32(argc, argv, "rl");
    } else if (strcmp(argv[1], "set-vc") == 0) {
        set_sensor_u32(argc, argv, "vc");
    } else if (strcmp(argv[1], "set-warmup") == 0) {
        set_sensor_u32(argc, argv, "warmup");
    } else if (strcmp(argv[1], "set-clean-factor") == 0) {
        set_clean_factor(argc, argv);
    } else if (strcmp(argv[1], "set-source-divider") == 0) {
        set_source_divider(argc, argv);
    } else if (strcmp(argv[1], "set-source-mux") == 0) {
        set_source_mux(argc, argv);
    } else if (strcmp(argv[1], "set-source-internal") == 0) {
        set_source_internal(argc, argv);
    } else if (strcmp(argv[1], "set-primary") == 0) {
        set_primary_sensor(argc, argv);
    } else if (strcmp(argv[1], "set-sample-interval") == 0) {
        set_aq_u32(argc, argv, "sample");
    } else if (strcmp(argv[1], "set-stale-after") == 0) {
        set_aq_u32(argc, argv, "stale");
    } else if (strcmp(argv[1], "set-ema-alpha") == 0) {
        set_ema_alpha(argc, argv);
    } else if (strcmp(argv[1], "matter-publish") == 0) {
        set_matter_publish(argc, argv);
    } else if (strcmp(argv[1], "sensors") == 0) {
        print_sensors();
    } else if (strcmp(argv[1], "adc") == 0) {
        if (argc < 3) {
            printf("usage: mq adc <logical_channel|all>\n");
            return 0;
        }
        if (strcmp(argv[2], "all") == 0) {
            print_all_adc_reads();
            return 0;
        }
        uint8_t logical_channel = 0;
        if (!parse_u8_arg(argv[2], &logical_channel)) {
            printf("invalid logical channel: %s\n", argv[2]);
            return 0;
        }
        print_adc_read(logical_channel);
    } else if (strcmp(argv[1], "raw") == 0) {
        if (argc >= 3) {
            uint8_t id = 0;
            if (!parse_u8_arg(argv[2], &id)) {
                printf("invalid sensor id: %s\n", argv[2]);
                return 0;
            }
            print_one_raw(id);
        } else {
            print_enabled_raw();
        }
    } else if (strcmp(argv[1], "calibrate-clean") == 0) {
        calibrate_clean(argc, argv);
    } else if (strcmp(argv[1], "source-read") == 0) {
        if (argc < 3) {
            printf("usage: mq source-read <source_id|all>\n");
            return 0;
        }
        if (strcmp(argv[2], "all") == 0) {
            print_all_source_reads();
            return 0;
        }
        uint8_t source_id = 0;
        if (!parse_u8_arg(argv[2], &source_id)) {
            printf("invalid source id: %s\n", argv[2]);
            return 0;
        }
        print_source_read(source_id);
    } else if (strcmp(argv[1], "erase-cal") == 0) {
        erase_calibration(argc, argv);
    } else if (strcmp(argv[1], "aq-status") == 0) {
        print_aq_status();
    } else if (strcmp(argv[1], "aq-start") == 0) {
        const esp_err_t err = air_quality_service_start();
        printf("aq-start: %s\n", esp_err_to_name(err));
    } else if (strcmp(argv[1], "aq-stop") == 0) {
        const esp_err_t err = air_quality_service_stop();
        printf("aq-stop: %s\n", esp_err_to_name(err));
    } else {
        printf("unknown mq command: %s\n", argv[1]);
    }

    return 0;
}

esp_err_t mq_console_register_commands(void)
{
#if CONFIG_ENABLE_CHIP_SHELL
    const esp_console_cmd_t mq_cmd = {
        .command = "mq",
        .help = "MQ air quality development commands",
        .hint = NULL,
        .func = &mq_command,
    };

    const esp_err_t err = esp_console_cmd_register(&mq_cmd);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "registered mq console command");
    }
    return err;
#else
    ESP_LOGI(TAG, "CHIP shell disabled; MQ console commands not registered");
    return ESP_OK;
#endif
}
