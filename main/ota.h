#ifndef WHEELBOARD_SDM26_ESP32_OTA_H
#define WHEELBOARD_SDM26_ESP32_OTA_H

#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_log.h"

esp_err_t ota_perform(const char *url);
void ota_mark_app_valid(void);
void start_ota_server(void);
esp_err_t ota_upload_handler(httpd_req_t *req);

#endif //WHEELBOARD_SDM26_ESP32_OTA_H