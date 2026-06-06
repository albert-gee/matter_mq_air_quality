#include "mq_console.h"

#include <errno.h>
#include <math.h>
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
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "matter_air_quality.h"
#include "mq_calibration_nvs.h"
#include "mq_runtime_config.h"
#include "mq_sensor.h"
#include "sdkconfig.h"

static const char *TAG = "mq_console";

static const size_t DEFAULT_BASELINE_SAMPLES = 32;
static const uint32_t DEFAULT_BASELINE_DELAY_MS = 100;
static const uint8_t KIT_PRIMARY_SENSOR_ID = 0;
static const uint32_t DEFAULT_SAMPLE_INTERVAL_MS = 5000;
static const uint32_t DEFAULT_STALE_AFTER_MS = 20000;
static const float DEFAULT_EMA_ALPHA = 0.25f;

static const mq_sensor_type_t k_expected_types[] = {
    MQ_SENSOR_MQ135,
    MQ_SENSOR_MQ2,
    MQ_SENSOR_MQ3,
    MQ_SENSOR_MQ4,
    MQ_SENSOR_MQ5,
    MQ_SENSOR_MQ6,
    MQ_SENSOR_MQ7,
    MQ_SENSOR_MQ8,
    MQ_SENSOR_MQ9,
};

static const uint32_t k_expected_warmup_seconds[] = {
    172800U,
    172800U,
    86400U,
    172800U,
    86400U,
    172800U,
    172800U,
    172800U,
    172800U,
};

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

static bool parse_size_arg(const char *text, size_t min_value, size_t max_value, size_t *out)
{
    uint32_t parsed = 0;
    if (!parse_u32_arg(text, (uint32_t)max_value, &parsed) || parsed < min_value) {
        return false;
    }
    *out = (size_t)parsed;
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
    case ANALOG_BACKEND_MUX_ADC:
        return "mux-adc";
    default:
        return "unknown";
    }
}

static const char *threshold_state_to_string(mq_sensor_threshold_state_t state)
{
    switch (state) {
    case MQ_SENSOR_THRESHOLD_NONE:
        return "none";
    case MQ_SENSOR_THRESHOLD_NORMAL:
        return "normal";
    case MQ_SENSOR_THRESHOLD_WARNING:
        return "warning";
    case MQ_SENSOR_THRESHOLD_CRITICAL:
        return "critical";
    default:
        return "unknown";
    }
}

static bool float_close(float a, float b)
{
    return fabsf(a - b) <= 0.0001f;
}

static bool get_effective_sensor_by_id(uint8_t id, mq_sensor_config_t *out, size_t *index_out)
{
    for (size_t i = 0; i < board_config_mq_sensor_count(); ++i) {
        mq_sensor_config_t sensor = {0};
        if (board_config_get_effective_sensor_config_by_index(i, &sensor) == ESP_OK && sensor.id == id) {
            if (out != NULL) {
                *out = sensor;
            }
            if (index_out != NULL) {
                *index_out = i;
            }
            return true;
        }
    }
    return false;
}

static bool get_effective_source_by_id(uint8_t id, analog_source_config_t *out, size_t *index_out)
{
    for (size_t i = 0; i < board_config_analog_source_count(); ++i) {
        analog_source_config_t source = {0};
        if (board_config_get_effective_source_config_by_index(i, &source) == ESP_OK && source.source_id == id) {
            if (out != NULL) {
                *out = source;
            }
            if (index_out != NULL) {
                *index_out = i;
            }
            return true;
        }
    }
    return false;
}

static air_quality_service_config_t default_aq_config(void)
{
    air_quality_service_config_t config = {
        .matter_endpoint_id = 0,
        .primary_sensor_id = KIT_PRIMARY_SENSOR_ID,
        .sample_interval_ms = DEFAULT_SAMPLE_INTERVAL_MS,
        .stale_after_ms = DEFAULT_STALE_AFTER_MS,
        .ema_alpha = DEFAULT_EMA_ALPHA,
        .publish_to_matter = true,
    };
    return config;
}

static esp_err_t get_aq_config_for_save(air_quality_service_config_t *out)
{
    air_quality_service_config_t defaults = default_aq_config();
    return mq_runtime_config_apply_air_quality(&defaults, out);
}

static bool baseline_record_matches_sensor(const mq_calibration_record_t *record,
                                           const mq_sensor_config_t *sensor,
                                           const analog_source_config_t *source)
{
    return record != NULL &&
           sensor != NULL &&
           source != NULL &&
           record->valid &&
           record->magic == MQ_CALIBRATION_RECORD_MAGIC &&
           record->version == MQ_CALIBRATION_RECORD_VERSION &&
           record->firmware_config_version == MQ_CALIBRATION_FIRMWARE_CONFIG_VERSION &&
           record->baseline_vrl_mv > 0U &&
           record->baseline_vrl_mv < sensor->vc_mv &&
           record->baseline_rs_norm > 0.0f &&
           record->sensor_type == (uint8_t)sensor->type &&
           record->source_id == sensor->analog_source_id &&
           record->mux_id == source->mux_id &&
           record->mux_channel == source->mux_channel &&
           record->vc_mv == sensor->vc_mv &&
           float_close(record->input_divider_ratio, source->input_divider_ratio);
}

static esp_err_t erase_baseline_for_sensor(uint8_t sensor_id, bool *erased)
{
    bool had_record = false;
    mq_calibration_record_t record = {0};
    esp_err_t err = mq_calibration_nvs_load(sensor_id, &record);
    if (err == ESP_OK) {
        had_record = true;
    } else if (err != ESP_ERR_NOT_FOUND) {
        return err;
    }

    err = mq_sensor_erase_calibration(sensor_id);
    if (err == ESP_ERR_INVALID_STATE || err == ESP_ERR_NOT_FOUND) {
        err = mq_calibration_nvs_erase(sensor_id);
    }
    if (err == ESP_ERR_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK && erased != NULL) {
        *erased = had_record;
    }
    return err;
}

static esp_err_t erase_all_baselines(size_t *erased_count)
{
    size_t count = 0;
    for (size_t i = 0; i < board_config_mq_sensor_count(); ++i) {
        mq_sensor_config_t sensor = {0};
        if (board_config_get_effective_sensor_config_by_index(i, &sensor) != ESP_OK) {
            continue;
        }
        bool erased = false;
        esp_err_t err = erase_baseline_for_sensor(sensor.id, &erased);
        if (err != ESP_OK) {
            return err;
        }
        if (erased) {
            ++count;
        }
    }
    if (erased_count != NULL) {
        *erased_count = count;
    }
    return ESP_OK;
}

static void print_save_result(esp_err_t err)
{
    if (err == ESP_OK) {
        printf("saved=true reboot_required=true\n");
    } else {
        printf("saved=false error=%s\n", esp_err_to_name(err));
    }
}

static void print_save_result_with_baselines(esp_err_t save_err, esp_err_t erase_err, size_t erased_count)
{
    if (save_err != ESP_OK) {
        printf("saved=false error=%s\n", esp_err_to_name(save_err));
        return;
    }
    if (erase_err != ESP_OK) {
        printf("saved=true reboot_required=true baseline_erased=%u baseline_error=%s\n",
               (unsigned)erased_count, esp_err_to_name(erase_err));
        return;
    }
    printf("saved=true reboot_required=true baseline_erased=%u\n", (unsigned)erased_count);
}

static void print_sample(const mq_sensor_config_t *sensor, const mq_sensor_sample_t *sample, esp_err_t err)
{
    printf("id=%u type=%s state=%s err=%s raw=%d adc_mv=%d vrl_mv=%d "
           "baseline_vrl_mv=%d rs_norm_milli=%ld rs_ratio_milli=%ld "
           "threshold=%s faults=0x%08lx\n",
           (unsigned)sample->id,
           sensor != NULL ? mq_sensor_type_to_string(sensor->type) : "unknown",
           mq_sensor_state_to_string(sample->state),
           esp_err_to_name(err),
           sample->raw_adc,
           sample->adc_mv,
           sample->vrl_mv,
           sample->baseline_vrl_mv,
           (long)(sample->rs_norm * 1000.0f + 0.5f),
           (long)(sample->rs_ratio * 1000.0f + 0.5f),
           threshold_state_to_string(sample->threshold_state),
           (unsigned long)sample->fault_bitmap);
}

static void print_one_read(uint8_t id)
{
    mq_sensor_config_t sensor = {0};
    (void)get_effective_sensor_by_id(id, &sensor, NULL);
    mq_sensor_sample_t sample = {0};
    const esp_err_t err = mq_sensor_read(id, &sample);
    if (err == ESP_ERR_NOT_FOUND) {
        printf("read failed id=%u error=%s\n", (unsigned)id, esp_err_to_name(err));
        return;
    }
    print_sample(&sensor, &sample, err);
}

static void print_all_reads(void)
{
    for (size_t i = 0; i < board_config_mq_sensor_count(); ++i) {
        mq_sensor_config_t sensor = {0};
        if (board_config_get_effective_sensor_config_by_index(i, &sensor) == ESP_OK) {
            print_one_read(sensor.id);
        }
    }
}

static void print_adc_read(uint8_t logical_channel)
{
    int raw = 0;
    int mv = 0;
    const esp_err_t err = adc_service_read_mv(logical_channel, &raw, &mv);
    if (err == ESP_OK) {
        printf("adc logical=%u raw_avg=%d adc_mv=%d\n", (unsigned)logical_channel, raw, mv);
        return;
    }

    if (err == ESP_ERR_NOT_SUPPORTED) {
        (void)adc_service_read_raw(logical_channel, &raw);
    }
    printf("adc logical=%u error=%s raw=%d\n", (unsigned)logical_channel, esp_err_to_name(err), raw);
}

static void print_source_read(uint8_t source_id)
{
    analog_source_config_t source = {0};
    if (!get_effective_source_by_id(source_id, &source, NULL)) {
        printf("source=%u error=%s\n", (unsigned)source_id, esp_err_to_name(ESP_ERR_NOT_FOUND));
        return;
    }

    int raw = 0;
    int adc_mv = 0;
    int vrl_mv = 0;
    const esp_err_t err = analog_backend_read_mv(source_id, &raw, &adc_mv);
    if (err == ESP_OK) {
        (void)mq_sensor_apply_divider_mv(adc_mv, source.input_divider_ratio, &vrl_mv);
    }
    printf("source=%u type=%s err=%s raw=%d adc_mv=%d vrl_mv=%d divider=%.3f mux=%u mux_channel=%u\n",
           (unsigned)source_id,
           source_type_to_string(source.type),
           esp_err_to_name(err),
           raw,
           adc_mv,
           vrl_mv,
           (double)source.input_divider_ratio,
           (unsigned)source.mux_id,
           (unsigned)source.mux_channel);
}

static void print_all_source_reads(void)
{
    for (size_t i = 0; i < board_config_analog_source_count(); ++i) {
        analog_source_config_t source = {0};
        if (board_config_get_effective_source_config_by_index(i, &source) == ESP_OK) {
            print_source_read(source.source_id);
        }
    }
}

static void print_mux_config(void)
{
    analog_mux_config_t mux = {0};
    bool ready = false;
    const esp_err_t err = board_config_get_effective_mux_config(0, &mux);
    if (err == ESP_OK) {
        (void)analog_mux_is_ready(0, &ready);
    }
    printf("mux id=0 err=%s enabled=%s signal_adc=%u s0=%d s1=%d s2=%d s3=%d en=%d settle_us=%u ready=%s\n",
           esp_err_to_name(err),
           mux.enabled ? "true" : "false",
           (unsigned)mux.signal_adc_logical_channel,
           mux.gpio_s0,
           mux.gpio_s1,
           mux.gpio_s2,
           mux.gpio_s3,
           mux.gpio_en,
           (unsigned)mux.settle_time_us,
           ready ? "true" : "false");
}

static void print_config(void)
{
    printf("sensor id type enabled source vc_mv warmup_s baseline_supported warn_ratio crit_ratio "
           "baseline_valid baseline_vrl_mv baseline_at_unix samples stdev_vrl_mv\n");
    for (size_t i = 0; i < board_config_mq_sensor_count(); ++i) {
        mq_sensor_config_t sensor = {0};
        analog_source_config_t source = {0};
        mq_calibration_record_t record = {0};
        if (board_config_get_effective_sensor_config_by_index(i, &sensor) != ESP_OK) {
            continue;
        }
        const bool source_ok = get_effective_source_by_id(sensor.analog_source_id, &source, NULL);
        const bool record_ok = mq_calibration_nvs_load(sensor.id, &record) == ESP_OK &&
                               baseline_record_matches_sensor(&record, &sensor, source_ok ? &source : NULL);
        printf("sensor %u %s %s %u %u %u %s %.3f %.3f %s %u %u %u %.2f\n",
               (unsigned)sensor.id,
               mq_sensor_type_to_string(sensor.type),
               sensor.enabled ? "true" : "false",
               (unsigned)sensor.analog_source_id,
               (unsigned)sensor.vc_mv,
               (unsigned)sensor.warmup_seconds,
               sensor.supports_baseline_calibration ? "true" : "false",
               (double)sensor.warning_rs_ratio,
               (double)sensor.critical_rs_ratio,
               record_ok ? "true" : "false",
               record_ok ? (unsigned)record.baseline_vrl_mv : 0U,
               record_ok ? (unsigned)record.baseline_at_unix : 0U,
               record_ok ? (unsigned)record.sample_count : 0U,
               record_ok ? (double)record.baseline_vrl_stddev_mv : 0.0);
    }

    printf("source id type adc_channel divider mux mux_channel\n");
    for (size_t i = 0; i < board_config_analog_source_count(); ++i) {
        analog_source_config_t source = {0};
        if (board_config_get_effective_source_config_by_index(i, &source) != ESP_OK) {
            continue;
        }
        printf("source %u %s %u %.3f %u %u\n",
               (unsigned)source.source_id,
               source_type_to_string(source.type),
               (unsigned)source.adc_logical_channel,
               (double)source.input_divider_ratio,
               (unsigned)source.mux_id,
               (unsigned)source.mux_channel);
    }

    air_quality_service_config_t aq = {0};
    const esp_err_t aq_err = get_aq_config_for_save(&aq);
    printf("aq err=%s primary_sensor_id = %u sample_interval=%u stale_after=%u ema_alpha=%.3f publish=%s\n",
           esp_err_to_name(aq_err),
           (unsigned)aq.primary_sensor_id,
           (unsigned)aq.sample_interval_ms,
           (unsigned)aq.stale_after_ms,
           (double)aq.ema_alpha,
           aq.publish_to_matter ? "true" : "false");
}

static void print_baseline_status(void)
{
    printf("id type enabled source baseline_valid baseline_vrl_mv baseline_at_unix samples state\n");
    for (size_t i = 0; i < board_config_mq_sensor_count(); ++i) {
        mq_sensor_config_t sensor = {0};
        analog_source_config_t source = {0};
        mq_calibration_record_t record = {0};
        mq_sensor_sample_t sample = {0};
        if (board_config_get_effective_sensor_config_by_index(i, &sensor) != ESP_OK) {
            continue;
        }
        const bool source_ok = get_effective_source_by_id(sensor.analog_source_id, &source, NULL);
        const bool baseline_valid = mq_calibration_nvs_load(sensor.id, &record) == ESP_OK &&
                                    baseline_record_matches_sensor(&record, &sensor, source_ok ? &source : NULL);
        (void)mq_sensor_get_last(sensor.id, &sample);
        printf("%u %s %s %u %s %u %u %u %s\n",
               (unsigned)sensor.id,
               mq_sensor_type_to_string(sensor.type),
               sensor.enabled ? "true" : "false",
               (unsigned)sensor.analog_source_id,
               baseline_valid ? "true" : "false",
               baseline_valid ? (unsigned)record.baseline_vrl_mv : 0U,
               baseline_valid ? (unsigned)record.baseline_at_unix : 0U,
               baseline_valid ? (unsigned)record.sample_count : 0U,
               mq_sensor_state_to_string(sample.state));
    }
}

static void validate_config(void)
{
    size_t errors = 0;
    size_t warnings = 0;

    analog_mux_config_t mux = {0};
    bool mux_ready = false;
    if (board_config_get_effective_mux_config(0, &mux) != ESP_OK) {
        printf("error mux 0 missing\n");
        ++errors;
    } else {
        (void)analog_mux_is_ready(0, &mux_ready);
        if (!mux.enabled) {
            printf("error mux 0 disabled\n");
            ++errors;
        }
        if (!mux_ready) {
            printf("error mux 0 not ready\n");
            ++errors;
        }
        if (mux.signal_adc_logical_channel != 0 || mux.gpio_s0 != 10 || mux.gpio_s1 != 11 ||
            mux.gpio_s2 != 12 || mux.gpio_s3 != 22 || mux.gpio_en != ANALOG_MUX_GPIO_UNUSED ||
            mux.settle_time_us < 500U) {
            printf("error mux 0 does not match kit wiring adc=0 s0=10 s1=11 s2=12 s3=22 en=-1 settle>=500\n");
            ++errors;
        }
    }

    for (size_t i = 0; i < board_config_analog_source_count(); ++i) {
        analog_source_config_t source = {0};
        if (board_config_get_effective_source_config_by_index(i, &source) != ESP_OK) {
            printf("error source index %u missing\n", (unsigned)i);
            ++errors;
            continue;
        }
        if (source.source_id != i || source.type != ANALOG_BACKEND_MUX_ADC ||
            source.adc_logical_channel != 0 || source.mux_id != 0 || source.mux_channel != i ||
            !float_close(source.input_divider_ratio, 2.0f)) {
            printf("error source %u must be mux-backed adc=0 divider=2.0 mux=0 mux_channel=%u\n",
                   (unsigned)source.source_id, (unsigned)source.source_id);
            ++errors;
        }
    }

    for (size_t i = 0; i < board_config_mq_sensor_count(); ++i) {
        mq_sensor_config_t sensor = {0};
        analog_source_config_t source = {0};
        mq_calibration_record_t record = {0};
        if (board_config_get_effective_sensor_config_by_index(i, &sensor) != ESP_OK) {
            printf("error sensor index %u missing\n", (unsigned)i);
            ++errors;
            continue;
        }
        if (sensor.id >= sizeof(k_expected_types) / sizeof(k_expected_types[0]) ||
            sensor.type != k_expected_types[sensor.id] ||
            sensor.analog_source_id != sensor.id) {
            printf("error sensor %u mapping mismatch type=%s source=%u\n",
                   (unsigned)sensor.id,
                   mq_sensor_type_to_string(sensor.type),
                   (unsigned)sensor.analog_source_id);
            ++errors;
        }
        if (!sensor.enabled) {
            printf("warning sensor %u disabled\n", (unsigned)sensor.id);
            ++warnings;
        }
        if (sensor.id < sizeof(k_expected_warmup_seconds) / sizeof(k_expected_warmup_seconds[0]) &&
            sensor.warmup_seconds < k_expected_warmup_seconds[sensor.id]) {
            printf("error sensor %u warmup %u below required default %u\n",
                   (unsigned)sensor.id,
                   (unsigned)sensor.warmup_seconds,
                   (unsigned)k_expected_warmup_seconds[sensor.id]);
            ++errors;
        }
        if (!get_effective_source_by_id(sensor.analog_source_id, &source, NULL)) {
            printf("error sensor %u uses missing source %u\n",
                   (unsigned)sensor.id, (unsigned)sensor.analog_source_id);
            ++errors;
            continue;
        }
        if (sensor.enabled && source.type != ANALOG_BACKEND_MUX_ADC) {
            printf("error enabled sensor %u source %u is not mux-backed\n",
                   (unsigned)sensor.id, (unsigned)source.source_id);
            ++errors;
        }
        if (sensor.enabled && source.input_divider_ratio < 0.1f) {
            printf("error enabled sensor %u source %u divider invalid\n",
                   (unsigned)sensor.id, (unsigned)source.source_id);
            ++errors;
        }
        if (sensor.supports_baseline_calibration &&
            (mq_calibration_nvs_load(sensor.id, &record) != ESP_OK ||
             !baseline_record_matches_sensor(&record, &sensor, &source))) {
            printf("warning sensor %u has no matching baseline\n", (unsigned)sensor.id);
            ++warnings;
        }
    }

    air_quality_service_config_t aq = {0};
    if (get_aq_config_for_save(&aq) != ESP_OK || aq.primary_sensor_id != KIT_PRIMARY_SENSOR_ID) {
        printf("error aq primary sensor must be 0 MQ-135\n");
        ++errors;
    } else {
        mq_sensor_config_t primary = {0};
        if (!get_effective_sensor_by_id(aq.primary_sensor_id, &primary, NULL) ||
            primary.type != MQ_SENSOR_MQ135 ||
            !primary.enabled) {
            printf("error aq primary sensor 0 must be enabled MQ-135\n");
            ++errors;
        }
    }

    const uint32_t diagnostics_cluster_id = matter_air_quality_diagnostics_cluster_id();
    if (diagnostics_cluster_id != MATTER_MQ_DIAGNOSTICS_CLUSTER_ID) {
        printf("error Matter diagnostics cluster id invalid actual=0x%08lx expected=0x%08lx\n",
               (unsigned long)diagnostics_cluster_id,
               (unsigned long)MATTER_MQ_DIAGNOSTICS_CLUSTER_ID);
        ++errors;
    }
    if (diagnostics_cluster_id == MATTER_MQ_DIAGNOSTICS_CLUSTER_SUFFIX) {
        printf("error Matter diagnostics cluster id is suffix-only, not a valid MEI\n");
        ++errors;
    }
    const size_t expected_diag_attrs = MATTER_MQ_DIAGNOSTICS_SENSOR_COUNT *
                                       MATTER_MQ_DIAGNOSTICS_ATTRS_PER_SENSOR;
    const size_t actual_diag_attrs = matter_air_quality_diagnostics_attribute_count();
    if (matter_air_quality_diagnostics_validate() != ESP_OK) {
        printf("error Matter diagnostics cluster or attributes missing\n");
        ++errors;
    }
    if (actual_diag_attrs != expected_diag_attrs) {
        printf("error Matter diagnostics attr count actual=%u expected=%u\n",
               (unsigned)actual_diag_attrs,
               (unsigned)expected_diag_attrs);
        ++errors;
    }

    printf("validation errors=%u warnings=%u\n", (unsigned)errors, (unsigned)warnings);
}

static void use_kit_defaults(void)
{
    esp_err_t first_err = ESP_OK;
    size_t erased_count = 0;

    analog_mux_config_t mux = {0};
    if (board_config_get_default_mux_config(0, &mux) == ESP_OK) {
        esp_err_t err = mq_runtime_config_save_mux(&mux);
        if (err != ESP_OK && first_err == ESP_OK) {
            first_err = err;
        }
    }

    for (size_t i = 0; i < board_config_analog_source_count(); ++i) {
        analog_source_config_t source = {0};
        if (board_config_get_default_source_config_by_index(i, &source) == ESP_OK) {
            esp_err_t err = mq_runtime_config_save_source(&source);
            if (err != ESP_OK && first_err == ESP_OK) {
                first_err = err;
            }
        }
    }

    for (size_t i = 0; i < board_config_mq_sensor_count(); ++i) {
        mq_sensor_config_t sensor = {0};
        if (board_config_get_default_sensor_config_by_index(i, &sensor) == ESP_OK) {
            esp_err_t err = mq_runtime_config_save_sensor(&sensor);
            if (err != ESP_OK && first_err == ESP_OK) {
                first_err = err;
            }
        }
    }

    air_quality_service_config_t aq = default_aq_config();
    esp_err_t err = mq_runtime_config_save_air_quality(&aq);
    if (err != ESP_OK && first_err == ESP_OK) {
        first_err = err;
    }
    err = erase_all_baselines(&erased_count);
    print_save_result_with_baselines(first_err, err, erased_count);
}

static void calibrate_one(uint8_t id, size_t samples, uint32_t delay_ms)
{
    mq_calibration_record_t record = {0};
    const esp_err_t err = mq_sensor_calibrate_baseline(id, samples, delay_ms, &record);
    if (err == ESP_OK) {
        printf("baseline id=%u baseline_vrl_mv=%u samples=%u stdev_vrl_mv=%.2f note=baseline-calibration\n",
               (unsigned)id,
               (unsigned)record.baseline_vrl_mv,
               (unsigned)record.sample_count,
               (double)record.baseline_vrl_stddev_mv);
    } else {
        printf("baseline id=%u error=%s\n", (unsigned)id, esp_err_to_name(err));
    }
}

static void calibrate_baseline_command(int argc, char **argv, bool alias)
{
    if (argc < 3) {
        printf("usage: mq %s <id|all> [samples] [delay_ms]\n", argv[1]);
        return;
    }
    if (alias) {
        printf("note=baseline-calibration-alias\n");
    }

    size_t samples = DEFAULT_BASELINE_SAMPLES;
    uint32_t delay_ms = DEFAULT_BASELINE_DELAY_MS;
    if (argc >= 4 && !parse_size_arg(argv[3], 1, 256, &samples)) {
        printf("invalid samples: %s\n", argv[3]);
        return;
    }
    if (argc >= 5 && !parse_u32_arg(argv[4], 5000, &delay_ms)) {
        printf("invalid delay_ms: %s\n", argv[4]);
        return;
    }

    air_quality_service_status_t aq_status = {0};
    const bool was_running = air_quality_service_get_status(&aq_status) == ESP_OK && aq_status.running;
    if (was_running) {
        (void)air_quality_service_stop();
    }

    if (strcmp(argv[2], "all") == 0) {
        for (size_t i = 0; i < board_config_mq_sensor_count(); ++i) {
            mq_sensor_config_t sensor = {0};
            if (board_config_get_effective_sensor_config_by_index(i, &sensor) == ESP_OK) {
                if (!sensor.supports_baseline_calibration) {
                    printf("baseline id=%u skipped=diagnostic-only\n", (unsigned)sensor.id);
                    continue;
                }
                calibrate_one(sensor.id, samples, delay_ms);
            }
        }
    } else {
        uint8_t id = 0;
        if (!parse_u8_arg(argv[2], &id)) {
            printf("invalid sensor id: %s\n", argv[2]);
        } else {
            calibrate_one(id, samples, delay_ms);
        }
    }

    if (was_running) {
        (void)air_quality_service_start();
    }
}

static void erase_baseline_command(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: mq %s <id|all>\n", argv[1]);
        return;
    }

    if (strcmp(argv[2], "all") == 0) {
        size_t erased_count = 0;
        const esp_err_t err = erase_all_baselines(&erased_count);
        printf("baseline_erased=%u status=%s\n", (unsigned)erased_count, esp_err_to_name(err));
        return;
    }

    uint8_t id = 0;
    if (!parse_u8_arg(argv[2], &id)) {
        printf("invalid sensor id: %s\n", argv[2]);
        return;
    }
    bool erased = false;
    const esp_err_t err = erase_baseline_for_sensor(id, &erased);
    printf("id=%u baseline_erased=%u status=%s\n", (unsigned)id, erased ? 1U : 0U, esp_err_to_name(err));
}

static void set_primary_command(int argc, char **argv)
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
    if (id != KIT_PRIMARY_SENSOR_ID) {
        printf("saved=false error=%s note=primary-must-be-sensor-0-MQ135\n",
               esp_err_to_name(ESP_ERR_NOT_SUPPORTED));
        return;
    }

    air_quality_service_config_t aq = {0};
    esp_err_t err = get_aq_config_for_save(&aq);
    if (err == ESP_OK) {
        aq.primary_sensor_id = KIT_PRIMARY_SENSOR_ID;
        err = mq_runtime_config_save_air_quality(&aq);
    }
    print_save_result(err);
}

static void set_threshold_command(int argc, char **argv, bool critical)
{
    if (argc < 4) {
        printf("usage: mq %s <id> <ratio>\n", argv[1]);
        return;
    }
    uint8_t id = 0;
    float ratio = 0.0f;
    if (!parse_u8_arg(argv[2], &id) || !parse_float_arg(argv[3], 0.0f, 10.0f, &ratio)) {
        printf("invalid threshold arguments\n");
        return;
    }

    mq_sensor_config_t sensor = {0};
    if (!get_effective_sensor_by_id(id, &sensor, NULL)) {
        printf("saved=false error=%s\n", esp_err_to_name(ESP_ERR_NOT_FOUND));
        return;
    }
    if (critical) {
        sensor.critical_rs_ratio = ratio;
    } else {
        sensor.warning_rs_ratio = ratio;
    }
    print_save_result(mq_runtime_config_save_sensor(&sensor));
}

static void print_thresholds(void)
{
    printf("id type warning_ratio critical_ratio\n");
    for (size_t i = 0; i < board_config_mq_sensor_count(); ++i) {
        mq_sensor_config_t sensor = {0};
        if (board_config_get_effective_sensor_config_by_index(i, &sensor) == ESP_OK) {
            printf("%u %s %.3f %.3f\n",
                   (unsigned)sensor.id,
                   mq_sensor_type_to_string(sensor.type),
                   (double)sensor.warning_rs_ratio,
                   (double)sensor.critical_rs_ratio);
        }
    }
}

static void print_aq_status(void)
{
    air_quality_service_status_t status = {0};
    const esp_err_t err = air_quality_service_get_status(&status);
    if (err != ESP_OK) {
        printf("aq-status failed: %s\n", esp_err_to_name(err));
        return;
    }

    printf("running=%s endpoint=%u primary_sensor_id = %u level=%s last_scheduled=%s "
           "filtered_ratio=%.3f last_success_ms=%u last_sample_age_ms=%u "
           "raw=%d adc_mv=%d vrl_mv=%d baseline_vrl_mv=%d rs_norm_milli=%ld rs_ratio_milli=%ld "
           "ok_reads=%u fail_reads=%u matter_update_schedules=%u last_error=%s\n",
           status.running ? "true" : "false",
           (unsigned)status.matter_endpoint_id,
           (unsigned)status.primary_sensor_id,
           air_quality_service_level_to_string(status.current_level),
           air_quality_service_level_to_string(status.last_scheduled_level),
           (double)status.filtered_ratio,
           (unsigned)status.last_success_ms,
           (unsigned)status.last_sample_age_ms,
           status.last_sample.raw_adc,
           status.last_sample.adc_mv,
           status.last_sample.vrl_mv,
           status.last_sample.baseline_vrl_mv,
           (long)(status.last_sample.rs_norm * 1000.0f + 0.5f),
           (long)(status.last_sample.rs_ratio * 1000.0f + 0.5f),
           (unsigned)status.successful_reads,
           (unsigned)status.failed_reads,
           (unsigned)status.matter_update_schedules,
           esp_err_to_name(status.last_error));
}

static void mux_select_command(int argc, char **argv)
{
    if (argc < 3) {
        printf("usage: mq mux-select <channel>\n");
        return;
    }
    uint8_t channel = 0;
    if (!parse_u8_arg(argv[2], &channel)) {
        printf("invalid mux channel: %s\n", argv[2]);
        return;
    }
    const esp_err_t err = analog_backend_select_mux_channel(0, channel);
    printf("mux-select channel=%u status=%s\n", (unsigned)channel, esp_err_to_name(err));
}

static void mux_scan_command(int argc, char **argv)
{
    uint8_t first = 0;
    uint8_t last = 8;
    uint8_t samples = 8;
    if (argc >= 3 && !parse_u8_arg(argv[2], &first)) {
        printf("invalid first channel\n");
        return;
    }
    if (argc >= 4 && !parse_u8_arg(argv[3], &last)) {
        printf("invalid last channel\n");
        return;
    }
    if (argc >= 5 && (!parse_u8_arg(argv[4], &samples) || samples == 0)) {
        printf("invalid sample count\n");
        return;
    }
    if (first > last || last > 15) {
        printf("invalid channel range\n");
        return;
    }

    bool ready = false;
    if (analog_mux_is_ready(0, &ready) != ESP_OK || !ready) {
        printf("mux-scan failed: %s\n", esp_err_to_name(ESP_ERR_INVALID_STATE));
        return;
    }

    for (uint8_t channel = first; channel <= last; ++channel) {
        int64_t raw_sum = 0;
        int64_t mv_sum = 0;
        int raw_min = INT32_MAX;
        int raw_max = INT32_MIN;
        uint32_t failures = 0;
        for (uint8_t i = 0; i < samples; ++i) {
            int raw = 0;
            int mv = 0;
            const esp_err_t err = analog_backend_read_mux_channel_mv(0, channel, &raw, &mv);
            if (err != ESP_OK) {
                ++failures;
                continue;
            }
            raw_sum += raw;
            mv_sum += mv;
            raw_min = raw < raw_min ? raw : raw_min;
            raw_max = raw > raw_max ? raw : raw_max;
        }
        const uint32_t ok = samples - failures;
        printf("channel=%u raw_avg=%ld adc_mv_avg=%ld raw_min=%d raw_max=%d failures=%lu\n",
               (unsigned)channel,
               ok > 0 ? (long)(raw_sum / ok) : 0L,
               ok > 0 ? (long)(mv_sum / ok) : 0L,
               ok > 0 ? raw_min : 0,
               ok > 0 ? raw_max : 0,
               (unsigned long)failures);
    }
}

static void config_reset_command(int argc, char **argv)
{
    if (argc < 3 || strcmp(argv[2], "all") != 0) {
        printf("usage: mq config-reset all\n");
        return;
    }
    const esp_err_t cfg_err = mq_runtime_config_erase_all();
    size_t erased_count = 0;
    const esp_err_t erase_err = erase_all_baselines(&erased_count);
    print_save_result_with_baselines(cfg_err, erase_err, erased_count);
}

static int mq_command(int argc, char **argv)
{
    if (argc < 2) {
        printf("usage: mq <status|config|config-validate|use-kit-defaults|read|source-read|adc|mux-config|mux-select|mux-scan|baseline-status|calibrate-baseline|calibrate-clean|erase-baseline|erase-cal|thresholds|set-warning-ratio|set-critical-ratio|set-primary|aq-status|aq-start|aq-stop|config-reset>\n");
        return 0;
    }

    if (strcmp(argv[1], "status") == 0) {
        printf("MQ kit active sensors=%u primary_sensor_id = 0 MQ135 mux_channel=0 divider=2.0\n",
               (unsigned)mq_sensor_count());
    } else if (strcmp(argv[1], "config") == 0) {
        print_config();
    } else if (strcmp(argv[1], "config-validate") == 0) {
        validate_config();
    } else if (strcmp(argv[1], "use-kit-defaults") == 0 || strcmp(argv[1], "mux-use-kit-default") == 0) {
        use_kit_defaults();
    } else if (strcmp(argv[1], "read") == 0 || strcmp(argv[1], "raw") == 0) {
        if (argc < 3 || strcmp(argv[2], "all") == 0) {
            print_all_reads();
        } else {
            uint8_t id = 0;
            if (!parse_u8_arg(argv[2], &id)) {
                printf("invalid sensor id: %s\n", argv[2]);
            } else {
                print_one_read(id);
            }
        }
    } else if (strcmp(argv[1], "source-read") == 0) {
        if (argc < 3) {
            printf("usage: mq source-read <source_id|all>\n");
        } else if (strcmp(argv[2], "all") == 0) {
            print_all_source_reads();
        } else {
            uint8_t source_id = 0;
            if (!parse_u8_arg(argv[2], &source_id)) {
                printf("invalid source id: %s\n", argv[2]);
            } else {
                print_source_read(source_id);
            }
        }
    } else if (strcmp(argv[1], "adc") == 0) {
        if (argc < 3) {
            printf("usage: mq adc <logical_channel>\n");
        } else {
            uint8_t logical_channel = 0;
            if (!parse_u8_arg(argv[2], &logical_channel)) {
                printf("invalid logical channel: %s\n", argv[2]);
            } else {
                print_adc_read(logical_channel);
            }
        }
    } else if (strcmp(argv[1], "mux-config") == 0) {
        print_mux_config();
    } else if (strcmp(argv[1], "mux-select") == 0) {
        mux_select_command(argc, argv);
    } else if (strcmp(argv[1], "mux-scan") == 0) {
        mux_scan_command(argc, argv);
    } else if (strcmp(argv[1], "baseline-status") == 0 || strcmp(argv[1], "cal-status") == 0) {
        print_baseline_status();
    } else if (strcmp(argv[1], "calibrate-baseline") == 0) {
        calibrate_baseline_command(argc, argv, false);
    } else if (strcmp(argv[1], "calibrate-clean") == 0) {
        calibrate_baseline_command(argc, argv, true);
    } else if (strcmp(argv[1], "erase-baseline") == 0 || strcmp(argv[1], "erase-cal") == 0) {
        erase_baseline_command(argc, argv);
    } else if (strcmp(argv[1], "thresholds") == 0) {
        print_thresholds();
    } else if (strcmp(argv[1], "set-warning-ratio") == 0) {
        set_threshold_command(argc, argv, false);
    } else if (strcmp(argv[1], "set-critical-ratio") == 0) {
        set_threshold_command(argc, argv, true);
    } else if (strcmp(argv[1], "set-primary") == 0) {
        set_primary_command(argc, argv);
    } else if (strcmp(argv[1], "aq-status") == 0) {
        print_aq_status();
    } else if (strcmp(argv[1], "aq-start") == 0) {
        printf("aq-start: %s\n", esp_err_to_name(air_quality_service_start()));
    } else if (strcmp(argv[1], "aq-stop") == 0) {
        printf("aq-stop: %s\n", esp_err_to_name(air_quality_service_stop()));
    } else if (strcmp(argv[1], "config-reset") == 0) {
        config_reset_command(argc, argv);
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
        .help = "MQ baseline-ratio diagnostics and configuration",
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
