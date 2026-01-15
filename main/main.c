#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"
#include "sdkconfig.h"

static const char *TAG = "Wheelboard";

twai_node_handle_t CAN1 = NULL;
#define CAN1_TX GPIO_NUM_18
#define CAN1_RX GPIO_NUM_17

void initialize_can() {
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = CAN1_TX,
        .io_cfg.rx = CAN1_RX,
        .bit_timing.bitrate = 200000,
        .tx_queue_depth = 5,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &CAN1));
    ESP_ERROR_CHECK(twai_node_enable(CAN1));
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Wheelboard");

    initialize_can();
    ESP_LOGI(TAG, "Can init");
}
