#include "mq_calibration_nvs.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"

#define MQ_CAL_NVS_NAMESPACE "mq_cal"

static const char *TAG = "mq_cal_nvs";
static bool s_initialized;

static void make_key(uint8_t sensor_id, char *key, size_t key_size)
{
    snprintf(key, key_size, "s%u", (unsigned)sensor_id);
}

esp_err_t mq_calibration_nvs_init(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MQ_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to open NVS namespace %s: %s", MQ_CAL_NVS_NAMESPACE, esp_err_to_name(err));
        return err;
    }
    nvs_close(handle);
    s_initialized = true;
    ESP_LOGI(TAG, "calibration NVS initialized");
    return ESP_OK;
}

esp_err_t mq_calibration_nvs_load(uint8_t sensor_id, mq_calibration_record_t *out)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "calibration NVS is not initialized");
    ESP_RETURN_ON_FALSE(out != NULL, ESP_ERR_INVALID_ARG, TAG, "calibration output is required");

    memset(out, 0, sizeof(*out));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(MQ_CAL_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to open NVS namespace");

    char key[8];
    make_key(sensor_id, key, sizeof(key));

    size_t required_size = sizeof(*out);
    err = nvs_get_blob(handle, key, out, &required_size);
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to load calibration for sensor %u", (unsigned)sensor_id);
    ESP_RETURN_ON_FALSE(required_size == sizeof(*out), ESP_ERR_INVALID_SIZE, TAG,
                        "stored calibration size mismatch for sensor %u", (unsigned)sensor_id);
    return ESP_OK;
}

esp_err_t mq_calibration_nvs_save(uint8_t sensor_id, const mq_calibration_record_t *record)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "calibration NVS is not initialized");
    ESP_RETURN_ON_FALSE(record != NULL, ESP_ERR_INVALID_ARG, TAG, "calibration record is required");

    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(MQ_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle), TAG,
                        "failed to open NVS namespace");

    char key[8];
    make_key(sensor_id, key, sizeof(key));

    esp_err_t err = nvs_set_blob(handle, key, record, sizeof(*record));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to save calibration for sensor %u", (unsigned)sensor_id);
    return ESP_OK;
}

esp_err_t mq_calibration_nvs_erase(uint8_t sensor_id)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "calibration NVS is not initialized");

    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(MQ_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle), TAG,
                        "failed to open NVS namespace");

    char key[8];
    make_key(sensor_id, key, sizeof(key));

    esp_err_t err = nvs_erase_key(handle, key);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    ESP_RETURN_ON_ERROR(err, TAG, "failed to erase calibration for sensor %u", (unsigned)sensor_id);
    return ESP_OK;
}

esp_err_t mq_calibration_nvs_erase_all(void)
{
    ESP_RETURN_ON_FALSE(s_initialized, ESP_ERR_INVALID_STATE, TAG, "calibration NVS is not initialized");

    nvs_handle_t handle;
    ESP_RETURN_ON_ERROR(nvs_open(MQ_CAL_NVS_NAMESPACE, NVS_READWRITE, &handle), TAG,
                        "failed to open NVS namespace");

    esp_err_t err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    ESP_RETURN_ON_ERROR(err, TAG, "failed to erase all calibrations");
    return ESP_OK;
}
