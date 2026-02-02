#ifndef WHEELBOARD_SDM26_ESP32_MCP2518FD_H
#define WHEELBOARD_SDM26_ESP32_MCP2518FD_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/twai.h"
#include "esp_err.h"

esp_err_t mcp_init(spi_device_handle_t spi);

esp_err_t mcp_send(spi_device_handle_t spi, const twai_message_t *msg);

esp_err_t mcp_receive(spi_device_handle_t spi, twai_message_t *msg);

bool mcp_available(spi_device_handle_t spi);

#endif //WHEELBOARD_SDM26_ESP32_MCP2518FD_H