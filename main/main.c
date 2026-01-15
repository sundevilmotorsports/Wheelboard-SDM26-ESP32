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

#define I2C_SCL 47
#define I2C_SDA 48

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
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 1,
        .scl_io_num = I2C_SCL,
        .sda_io_num = I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x58,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Wheelboard");

    initializeCan();
    ESP_LOGI(TAG, "Can init");
}
