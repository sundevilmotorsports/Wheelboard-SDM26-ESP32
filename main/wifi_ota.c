#include "wifi_ota.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "string.h"
#include "sys/param.h"
#include "driver/gpio.h"
#include "led_util.h"

static const char *TAG = "WIFI_OTA";

// Wi-Fi AP settings
#define WIFI_AP_SSID "ESP32_S3_Dev"
#define WIFI_AP_PASS "Rahil12345"
#define WIFI_AP_CHANNEL 1
#define MAX_STA_CONN 4

// Handler for OTA update GET request (serves HTML page)
static esp_err_t ota_get_handler(httpd_req_t *req) {
  const char *html =
      "<!DOCTYPE html><html>"
      "<head><title>ESP32 Firmware Update</title></head>"
      "<body>"
      "<h1>ESP32 OTA Firmware Update</h1>"
      "<input type='file' id='update' name='update' required>"
      "<button onclick='uploadFirmware()'>Update Firmware</button>"
      "<div id='status' style='margin-top: 20px; font-weight: bold;'></div>"
      "<script>"
      "function uploadFirmware() {"
      "  var fileInput = document.getElementById('update').files;"
      "  if (fileInput.length === 0) { alert('Select a file!'); return; }"
      "  var file = fileInput[0];"
      "  var xhr = new XMLHttpRequest();"
      "  xhr.onload = function() {"
      "    if (xhr.status === 200) { document.getElementById('status').innerHTML = 'Update successful! Rebooting...'; }"
      "    else { document.getElementById('status').innerHTML = 'Update failed: ' + xhr.statusText; }"
      "  };"
      "  xhr.upload.onprogress = function(e) {"
      "    if (e.lengthComputable) {"
      "      var pct = (e.loaded / e.total) * 100;"
      "      document.getElementById('status').innerHTML = 'Uploading: ' + pct.toFixed(1) + '%';"
      "    }"
      "  };"
      "  xhr.open('POST', '/update', true);"
      "  xhr.send(file);"
      "}"
      "</script>"
      "</body></html>";
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

// Handler for OTA update POST request (receives binary)
static esp_err_t ota_post_handler(httpd_req_t *req) {
  esp_err_t err;
  esp_ota_handle_t update_handle = 0;
  const esp_partition_t *update_partition = NULL;

  ESP_LOGI(TAG, "Starting OTA update...");

  int led_state = 0;

  update_partition = esp_ota_get_next_update_partition(NULL);
  if (update_partition == NULL) {
    ESP_LOGE(TAG, "Passive OTA partition not found");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES,
                      &update_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  char buf[1024];
  int received = 0;
  int remaining = req->content_len;

  while (remaining > 0) {
    led_state = !led_state;
    if (led_state) {
      led_util_set_color(255, 165, 0); // Orange/Yellowish for progress
    } else {
      led_util_clear();
    }

    if ((received = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <=
        0) {
      if (received == HTTPD_SOCK_ERR_TIMEOUT) {
        continue;
      }
      ESP_LOGE(TAG, "File receive failed");
      esp_ota_end(update_handle);
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }

    err = esp_ota_write(update_handle, (const void *)buf, received);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
      esp_ota_end(update_handle);
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    remaining -= received;
  }

  err = esp_ota_end(update_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_end failed (%s)", esp_err_to_name(err));
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  err = esp_ota_set_boot_partition(update_partition);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)",
             esp_err_to_name(err));
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "OTA Success! Rebooting...");
  httpd_resp_sendstr(req, "Firmware updated successfully! Rebooting...");

  led_util_set_color(0, 255, 0); // Green for success
  // Slight delay to allow HTTP response to be sent before restarting
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  esp_restart();

  return ESP_OK;
}

// Start HTTP Server
static httpd_handle_t start_webserver(void) {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 8;
  // We increase stack size for OTA processing.
  config.stack_size = 8192;

  ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t uri_get = {.uri = "/",
                           .method = HTTP_GET,
                           .handler = ota_get_handler,
                           .user_ctx = NULL};
    httpd_register_uri_handler(server, &uri_get);

    httpd_uri_t uri_update_get = {.uri = "/update",
                                  .method = HTTP_GET,
                                  .handler = ota_get_handler,
                                  .user_ctx = NULL};
    httpd_register_uri_handler(server, &uri_update_get);

    httpd_uri_t uri_post = {.uri = "/update",
                            .method = HTTP_POST,
                            .handler = ota_post_handler,
                            .user_ctx = NULL};
    httpd_register_uri_handler(server, &uri_post);

    return server;
  }

  ESP_LOGE(TAG, "Error starting server!");
  return NULL;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_id == WIFI_EVENT_AP_STACONNECTED) {
    wifi_event_ap_staconnected_t *event =
        (wifi_event_ap_staconnected_t *)event_data;
    ESP_LOGI(TAG, "station " MACSTR " join, AID=%d", MAC2STR(event->mac),
             event->aid);
  } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
    wifi_event_ap_stadisconnected_t *event =
        (wifi_event_ap_stadisconnected_t *)event_data;
    ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac),
             event->aid);
  }
}

void wifi_ota_init(void) {
  // Initialize NVS is often done in main.c, but we do it here if not done yet
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

  wifi_config_t wifi_config = {
      .ap =
          {
              .ssid = WIFI_AP_SSID,
              .ssid_len = strlen(WIFI_AP_SSID),
              .channel = WIFI_AP_CHANNEL,
              .password = WIFI_AP_PASS,
              .max_connection = MAX_STA_CONN,
              .authmode = WIFI_AUTH_WPA2_PSK,
          },
  };
  if (strlen(WIFI_AP_PASS) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s", WIFI_AP_SSID,
           WIFI_AP_PASS);

  start_webserver();
}
