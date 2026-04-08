#include "MLX90642.h"
#include "MLX90642_depends.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mcp2518fd.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include "mlx.h"
#include "MLX90614.h"
#include "MLX90642.h"
#include "MLX90642_depends.h"
static const char *TAG = "Wheelboard";

twai_node_handle_t CAN1 = NULL;
#define CAN1_TX GPIO_NUM_17
#define CAN1_RX GPIO_NUM_18
#define CAN1_BAUD 1000000 //bits
#define CAN1_TIMEOUT 0 //in milliseconds

spi_device_handle_t CAN2;

#define CAN2_MISO GPIO_NUM_20
#define CAN2_MOSI GPIO_NUM_9
#define CAN2_SCK GPIO_NUM_10
#define CAN2_CS GPIO_NUM_19
#define CAN2_INT GPIO_NUM_11
#define CAN2_INT0 GPIO_NUM_12
#define CAN2_INT1 GPIO_NUM_13

#define I2C_SCL GPIO_NUM_5
#define I2C_SDA GPIO_NUM_4

#define HE_SENSOR_GPIO GPIO_NUM_6
#define HE_MAGNET_COUNT 20U
#define HE_MIN_INTERVAL_MS 1U
#define HE_DEBOUNCE_PERCENT 50U
#define HE_STOP_TIMEOUT_MS 500U
#define HE_STOP_TIMEOUT_MULTIPLIER 3U
#define HE_QUEUE_POLL_MS 25U

#define FLW_ID 0x370
#define FRW_ID 0x380
#define RRW_ID 0x390
#define RLW_ID 0x3a0

typedef enum { BOARD_FL, BOARD_FR, BOARD_RR, BOARD_RL } board_pos_t;
#define BOARD_POSITION BOARD_RR

#define CAN_DEBUG false

static i2c_master_dev_handle_t mlx_camera_dev = NULL;
static i2c_master_dev_handle_t mlx_pix_dev = NULL;
static i2c_master_bus_handle_t i2c_bus = NULL;

static QueueHandle_t he_evt_queue = NULL;

static uint32_t g_wheel_speed = 0;
static float g_obj_temp = 0;
static float g_amb_temp = 0;

static uint32_t WORKING_ID;

static uint16_t data[MLX_TOTAL_PIXELS];
static uint8_t tx_buffer[8];
static uint8_t wt1_buffer[8];
static uint8_t wt2_buffer[8]; 

esp_err_t initializeSPI() {
    // TODO: Interrupts
    spi_bus_config_t cfg = {
        .mosi_io_num = CAN2_MOSI,
        .miso_io_num = CAN2_MISO,
        .sclk_io_num = CAN2_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &cfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) return ret;

    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 10000000,
        .mode = 0,
        .spics_io_num = CAN2_CS,
        .queue_size = 5,
    };

    return spi_bus_add_device(SPI2_HOST, &dev_config, &CAN2);
}

void initializeCan() {
    const char* LOCAL_TAG = "CAN INIT";
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = CAN1_TX,
        .io_cfg.rx = CAN1_RX,
        .bit_timing.bitrate = CAN1_BAUD,
        .tx_queue_depth = 10,
    };
    esp_err_t ret = twai_new_node_onchip(&node_config, &CAN1);
    if (ret != ESP_OK) {
        ESP_LOGE(LOCAL_TAG, "Failed to create CAN node: %s", esp_err_to_name(ret));
        return;
    }
    ret = twai_node_enable(CAN1);
    if (ret != ESP_OK) {
        ESP_LOGE(LOCAL_TAG, "Failed to enable CAN node: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(LOCAL_TAG, "CAN initialized on TX=%d, RX=%d @ %dkbps", CAN1_TX, CAN1_RX, CAN1_BAUD);
}

void sendCan1Message() {
    const char* LOCAL_TAG = "CAN 1 TX";

    tx_buffer[0] = (g_wheel_speed >> 8) & 0xFF;
    tx_buffer[1] = g_wheel_speed & 0xFF;

    uint16_t obj_temp = (uint16_t) g_obj_temp; 
    uint16_t amb_temp = (uint16_t) g_amb_temp; 

    tx_buffer[2] = (obj_temp >> 8) & 0xFF;
    tx_buffer[3] = obj_temp & 0xFF;

    tx_buffer[4] = (amb_temp >> 8) & 0xFF;
    tx_buffer[5] = amb_temp & 0xFF;

    twai_frame_t tx_msg = {
        .header.id = WORKING_ID,
        .header.ide = false,
        .header.rtr = false,
        .header.dlc = 6,
        .buffer = tx_buffer,
        .buffer_len = 6,
    };

    if (CAN_DEBUG){
        esp_err_t ret = twai_node_transmit(CAN1, &tx_msg, pdMS_TO_TICKS(CAN1_TIMEOUT));
        if (ret == ESP_OK) ESP_LOGI(LOCAL_TAG, "RPM=%u, Obj=%d, Amb=%d", g_wheel_speed, g_obj_temp,g_amb_temp);
        else ESP_LOGW(TAG, "Failed to send CAN message: %s", esp_err_to_name(ret));
    } else twai_node_transmit(CAN1, &tx_msg, pdMS_TO_TICKS(CAN1_TIMEOUT));
}

void sendCan1Message2() {
    const char* LOCAL_TAG = "CAN 2 TX";
    twai_frame_t tx1_msg = {
        .header.id = WORKING_ID + 1,
        .header.ide = false,
        .header.rtr = false,
        .header.dlc = 8,
        .buffer = wt1_buffer,
        .buffer_len = 8,
    };
    twai_frame_t tx2_msg = {
        .header.id = WORKING_ID + 2,
        .header.ide = false,
        .header.rtr = false,
        .header.dlc = 8,
        .buffer = wt2_buffer,
        .buffer_len = 8,
    };
    if (CAN_DEBUG){
        esp_err_t ret = twai_node_transmit(CAN1, &tx1_msg, pdMS_TO_TICKS(CAN1_TIMEOUT));
        if (ret == ESP_OK) ESP_LOGI(LOCAL_TAG, "Sent Tire Temp 1");
        else ESP_LOGW(LOCAL_TAG, "Failed to send CAN message: %s", esp_err_to_name(ret));

        ret = twai_node_transmit(CAN1, &tx2_msg, pdMS_TO_TICKS(CAN1_TIMEOUT));
        if (ret == ESP_OK) ESP_LOGI(LOCAL_TAG, "Sent Tire Temp 2");
        else ESP_LOGW(LOCAL_TAG, "Failed to send CAN message: %s", esp_err_to_name(ret));
    } else {
        twai_node_transmit(CAN1, &tx1_msg, pdMS_TO_TICKS(CAN1_TIMEOUT));
        vTaskDelay(pdMS_TO_TICKS(5)); // needed to not crash don't ask
        twai_node_transmit(CAN1, &tx2_msg, pdMS_TO_TICKS(CAN1_TIMEOUT));
    }
}

esp_err_t initializeI2C() {
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL,
        .sda_io_num = I2C_SDA,
        .glitch_ignore_cnt = 17,
        .flags.enable_internal_pullup = false
    };
    return i2c_new_master_bus(&i2c_mst_config, &i2c_bus);
}



static void gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(he_evt_queue, &gpio_num, NULL);
}

static void he_task(void *arg) {
    uint32_t io_num;
    uint32_t last_valid_time = 0;
    uint32_t last_interval_ms = 0;

    for (;;) {
        if (xQueueReceive(he_evt_queue, &io_num, pdMS_TO_TICKS(HE_QUEUE_POLL_MS))) {
            uint32_t current_time = esp_timer_get_time() / 1000;

            if (last_valid_time > 0) {
                uint32_t time_diff = current_time - last_valid_time;
                uint32_t min_valid_interval_ms = HE_MIN_INTERVAL_MS;

                if (last_interval_ms > 0) {
                    uint32_t dynamic_debounce_ms = (last_interval_ms * HE_DEBOUNCE_PERCENT) / 100U;
                    if (dynamic_debounce_ms > min_valid_interval_ms) {
                        min_valid_interval_ms = dynamic_debounce_ms;
                    }
                }

                if (time_diff >= min_valid_interval_ms) {
                    // RPM = (60000 ms/min) / (time_diff_ms * num_magnets)
                    uint32_t rpm = 60000U / (time_diff * HE_MAGNET_COUNT);
                    last_interval_ms = time_diff;
                    last_valid_time = current_time;
                    g_wheel_speed = rpm;
                    ESP_LOGI(TAG, "Magnet pass - RPM: %lu (interval: %lu ms)", rpm, time_diff);
                }
            } else {
                last_valid_time = current_time;
            }

            vTaskDelay(pdMS_TO_TICKS(5));

            while (xQueueReceive(he_evt_queue, &io_num, 0)) {
            }
        } else if (last_valid_time > 0) {
            uint32_t current_time = esp_timer_get_time() / 1000;
            uint32_t elapsed_ms = current_time - last_valid_time;
            uint32_t stop_timeout_ms = HE_STOP_TIMEOUT_MS;

            // Allow a few expected pulse periods before declaring the wheel stopped.
            if (last_interval_ms > 0) {
                uint32_t interval_timeout_ms = last_interval_ms * HE_STOP_TIMEOUT_MULTIPLIER;
                if (interval_timeout_ms > stop_timeout_ms) {
                    stop_timeout_ms = interval_timeout_ms;
                }
            }

            if (elapsed_ms >= stop_timeout_ms && g_wheel_speed != 0) {
                g_wheel_speed = 0;
                last_valid_time = 0;
                last_interval_ms = 0;
                ESP_LOGI(TAG, "No hall pulses for %lu ms, setting RPM to 0", elapsed_ms);
            }
        }
    }
}

static void mlx_pix_task(void *arg){
    const char* LOCAL_TAG = "MLX PIX TASK";
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t refresh_rate_hz = (uint32_t)arg;
    esp_err_t ret = MLX90614_init(i2c_bus, &mlx_pix_dev, MLX90614_DEFAULT_ADDRESS, MLX90614_DEFAULT_SPEED);
    if(ret != ESP_OK){
        ESP_LOGE(LOCAL_TAG, "FAILD PIXEL INIT");
        return;
    }
    for(;;){
        ret = MLX90614_GetAmbObjTemp(mlx_pix_dev, MLX90614_DEFAULT_ADDRESS, &g_amb_temp, &g_obj_temp);
        if(ret != ESP_OK)ESP_LOGE(LOCAL_TAG, "Failed to get object temp %s", esp_err_to_name(ret));
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / refresh_rate_hz));
    }
}



static void can1_tx_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t refresh_rate_hz = (uint32_t)arg;
    for (;;) {
        sendCan1Message();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / refresh_rate_hz));
    }
}

static void can2_tx_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t refresh_rate_hz = (uint32_t)arg;
    for (;;) {
        sendCan1Message2();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / refresh_rate_hz));
    }
}

void app_main(void) {
    static const uint32_t board_ids[]       = {FLW_ID,  FRW_ID,  RRW_ID,  RLW_ID};
    static const char * const board_names[] = {"FL",    "FR",    "RR",    "RL"};
    WORKING_ID = board_ids[BOARD_POSITION];
    ESP_LOGI(TAG, "Wheelboard: %s, Working ID: 0x%lx", board_names[BOARD_POSITION], WORKING_ID);

    vTaskDelay(pdMS_TO_TICKS(5));

    initializeCan();

    bool can2_ok = false;
    esp_err_t ret = initializeSPI();
    if (ret == ESP_OK) {
        can2_ok = (mcp_init(CAN2) == ESP_OK);
        if (!can2_ok) ESP_LOGE(TAG, "CAN2 unavailable");
    } else {
        ESP_LOGE(TAG, "SPI init failed (%s), CAN2 disabled", esp_err_to_name(ret));
    }

    bool mlx_ok = false;
    ret = initializeI2C();
    if (ret == ESP_OK) {
        ret = MLX90642_Initialize(i2c_bus, &mlx_camera_dev, data);
        mlx_ok = (ret == ESP_OK);
        if (!mlx_ok) ESP_LOGW(TAG, "MLX90642 unavailable (%s), thermal data disabled", esp_err_to_name(ret));
    } else {
        ESP_LOGW(TAG, "I2C init failed (%s), MLX disabled", esp_err_to_name(ret));
    }

    xTaskCreate(can1_tx_task, "can1_tx_task", 4096, (void*)200, 9, NULL);

    if (mlx_ok) {
        xTaskCreate(mlx_pix_task, "mlx_pix_task", 4096, (void*)50, 10, NULL);
        xTaskCreate(mlx_camera_task, "mlx_camera_task", 4096, NULL, 10, NULL);
        xTaskCreate(can2_tx_task, "can2_tx_task", 4096, (void*)2, 9, NULL);
    }

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << HE_SENSOR_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);

    he_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(he_task, "he_task", 4096, NULL, 10, NULL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(HE_SENSOR_GPIO, gpio_isr_handler, (void *) HE_SENSOR_GPIO);

    ESP_LOGI(TAG, "Everything initialized\n");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
