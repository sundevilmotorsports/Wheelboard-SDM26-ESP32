#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"
#include "sdkconfig.h"
#include "driver/i2c_master.h"

static const char *TAG = "Wheelboard";

twai_node_handle_t CAN1 = NULL;
#define CAN1_TX GPIO_NUM_18
#define CAN1_RX GPIO_NUM_17

i2c_master_bus_handle_t I2C = NULL;
i2c_master_dev_handle_t MLX90642 = NULL;
#define I2C_SCL 47
#define I2C_SDA 48
#define MLX_ADDR 0x33
#define MLX_DATA_SIZE 769

void initializeCan() {
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = CAN1_TX,
        .io_cfg.rx = CAN1_RX,
        .bit_timing.bitrate = 200000,
        .tx_queue_depth = 5,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &CAN1));
    ESP_ERROR_CHECK(twai_node_enable(CAN1));
}

void initializeI2c() {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &I2C));
}


int MLX_Read(uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *rData) {
    uint8_t cmd[2];
    cmd[0] = startAddress >> 8;
    cmd[1] = startAddress & 0xFF;

    uint8_t *data_bytes = (uint8_t *)rData;
    esp_err_t ret = i2c_master_transmit_receive(MLX90642, cmd, 2, data_bytes, nMemAddressRead * 2, 1000);

    for (int i = 0; i < nMemAddressRead; i++) {
        uint16_t temp = (data_bytes[i * 2] << 8) | data_bytes[i * 2 + 1];
        rData[i] = temp;
    }

    return (ret == ESP_OK) ? 0 : -1;
}

int MLX90642_Write(uint8_t slaveAddr, uint8_t *buffer, uint8_t bytesNum) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = slaveAddr,
        .scl_speed_hz = 400000,
    };

    i2c_master_dev_handle_t dev;
    if (i2c_master_bus_add_device(I2C, &dev_cfg, &dev) != ESP_OK) {
        return -1;
    }

    esp_err_t ret = i2c_master_transmit(dev, buffer, bytesNum, 1000);

    i2c_master_bus_rm_device(dev);
    return (ret == ESP_OK) ? 0 : -1;
}

int initializeMLX90642() {
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MLX_ADDR,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(I2C, &cfg, &MLX90642);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MLX90642 device: %s", esp_err_to_name(ret));
        return -1;
    }

    uint16_t flags;
    if (MLX_Read(0x3C14, 1, &flags) < 0) {
        ESP_LOGE(TAG, "Failed to read MLX90642 flags register");
        return -1;
    }

    ESP_LOGI(TAG, "MLX90642 initialized, flags: 0x%04X", flags);
    return 0;
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Wheelboard");

    initializeCan();
    ESP_LOGI(TAG, "Can init");

    initializeI2c();
    ESP_LOGI(TAG, "I2C init");

    initializeMLX90642();
    ESP_LOGI(TAG, "MLX init");

    for (;;) {
        uint16_t data[MLX_DATA_SIZE];
        MLX_Read(0x342C, MLX_DATA_SIZE, data);

        size_t len = sizeof data / sizeof data[0];
        ESP_LOGI(TAG, "Data length: %d", len);

        uint16_t total = 0;
        for (size_t i = 0; i < len; i++) {
            total += data[i];
        }

        ESP_LOGI(TAG, "Average temp: %d", total / len);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
