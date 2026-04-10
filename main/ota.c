#include "ota.h"

static const char *TAG = "OTA";

esp_err_t ota_perform(const char *url) {
    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return ESP_FAIL;

    if (esp_http_client_open(client, 0) != ESP_OK) {
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);

    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle;
    ESP_ERROR_CHECK(esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle));

    char buffer[1024];
    int read_bytes;

    while ((read_bytes = esp_http_client_read(client, buffer, sizeof(buffer))) > 0) {
        ESP_ERROR_CHECK(esp_ota_write(ota_handle, buffer, read_bytes));
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    ESP_ERROR_CHECK(esp_ota_end(ota_handle));
    ESP_ERROR_CHECK(esp_ota_set_boot_partition(update_partition));

    ESP_LOGI(TAG, "OTA done, rebooting...");
    esp_restart();

    return ESP_OK;
}

void ota_mark_app_valid(void) {
    esp_ota_img_states_t state;
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
        if (state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI("OTA", "Marking app as valid");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
}

esp_err_t ota_upload_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "OTA upload started, content_len=%d", req->content_len);
    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);

    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    static char buf[4096];
    int received;

    while ((received = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        if (esp_ota_write(ota_handle, buf, received) != ESP_OK) {
            ESP_LOGE(TAG, "OTA write failed");
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return ESP_FAIL;
        }
    }

    if (received < 0) {
        ESP_LOGE(TAG, "OTA receive error: %d", received);
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
        return ESP_FAIL;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Rebooting now...");
    esp_restart();

    return ESP_OK;
}

void start_ota_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 16384;
    config.task_priority = 5;
    httpd_handle_t server = NULL;

    httpd_start(&server, &config);

    httpd_uri_t ota_uri = {
        .uri = "/upload",
        .method = HTTP_POST,
        .handler = ota_upload_handler,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(server, &ota_uri);
    ESP_LOGI(TAG, "OTA server started, POST /upload to flash");
}