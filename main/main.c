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

static const char *TAG = "Wheelboard";

twai_node_handle_t CAN1 = NULL;
#define CAN1_TX GPIO_NUM_18
#define CAN1_RX GPIO_NUM_17

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

#define MLX_TOTAL_PIXELS 768
#define MLX_WIDTH 24
#define MLX_HEIGHT 32

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t mlx_dev = NULL;

static QueueHandle_t he_evt_queue = NULL;

static uint32_t g_wheel_speed = 0;
static int16_t g_obj_temp = 0;
static int16_t g_amb_temp = 0;

int MLX90642_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                     uint16_t nMemAddressRead, uint16_t *rData) {
    if (mlx_dev == NULL)
        return -1;

    uint8_t addr_buf[2];
    addr_buf[0] = (startAddress >> 8) & 0xFF;
    addr_buf[1] = startAddress & 0xFF;

    size_t read_size = nMemAddressRead * 2;
    uint8_t *read_buf = malloc(read_size);
    if (read_buf == NULL)
        return -1;

    esp_err_t ret = i2c_master_transmit_receive(
        mlx_dev, addr_buf, sizeof(addr_buf), read_buf, read_size, 1000);
    if (ret != ESP_OK) {
        free(read_buf);
        return -1;
    }

    // Convert bytes to 16-bit words (big-endian)
    for (int i = 0; i < nMemAddressRead; i++) {
        rData[i] = (read_buf[i * 2] << 8) | read_buf[i * 2 + 1];
    }

    free(read_buf);
    return 0;
}

int MLX90642_I2CWrite(uint8_t slaveAddr, uint8_t *buffer, uint8_t bytesNum) {
    if (mlx_dev == NULL)
        return -1;

    esp_err_t ret = i2c_master_transmit(mlx_dev, buffer, bytesNum, 1000);
    return (ret == ESP_OK) ? 0 : -1;
}

void MLX90642_Wait_ms(uint16_t time_ms) { vTaskDelay(pdMS_TO_TICKS(time_ms)); }

int MLX90642_Config(uint8_t slaveAddr, uint16_t writeAddress, uint16_t wData) {
    uint8_t buffer[6];
    buffer[0] = (writeAddress >> 8) & 0xFF; // Address MSB
    buffer[1] = writeAddress & 0xFF; // Address LSB
    buffer[2] = 0x00; // Padding MSB
    buffer[3] = 0x00; // Padding LSB
    buffer[4] = (wData >> 8) & 0xFF; // Data MSB
    buffer[5] = wData & 0xFF; // Data LSB

    return MLX90642_I2CWrite(slaveAddr, buffer, 6);
}

int MLX90642_I2CCmd(uint8_t slaveAddr, uint16_t i2c_cmd) {
    uint8_t buffer[4];
    buffer[0] = 0x01; // Command address MSB
    buffer[1] = 0x80; // Command address LSB (0x0180)
    buffer[2] = (i2c_cmd >> 8) & 0xFF; // Command MSB
    buffer[3] = i2c_cmd & 0xFF; // Command LSB

    return MLX90642_I2CWrite(slaveAddr, buffer, 4);
}

void initializeSPI() {
    // TODO: Interrupts
    spi_bus_config_t cfg = {
        .mosi_io_num = CAN2_MOSI,
        .miso_io_num = CAN2_MISO,
        .sclk_io_num = CAN2_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &cfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 10000000,
        .mode = 0,
        .spics_io_num = CAN2_CS,
        .queue_size = 5,
    };

    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_config, &CAN2));
}

void initializeCan() {
    twai_onchip_node_config_t node_config = {
        .io_cfg.tx = CAN1_TX,
        .io_cfg.rx = CAN1_RX,
        .bit_timing.bitrate = 200000,
        .tx_queue_depth = 5,
    };

    esp_err_t ret = twai_new_node_onchip(&node_config, &CAN1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create CAN node: %s", esp_err_to_name(ret));
        return;
    }

    ret = twai_node_enable(CAN1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable CAN node: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "CAN initialized on TX=%d, RX=%d @ 200kbps", CAN1_TX, CAN1_RX);
}

void sendCan1Message() {
    uint8_t tx_buffer[8];

    tx_buffer[0] = (g_wheel_speed >> 8) & 0xFF;
    tx_buffer[1] = g_wheel_speed & 0xFF;

    tx_buffer[2] = (g_obj_temp >> 8) & 0xFF;
    tx_buffer[3] = g_obj_temp & 0xFF;

    tx_buffer[4] = (g_amb_temp >> 8) & 0xFF;
    tx_buffer[5] = g_amb_temp & 0xFF;

    twai_frame_t tx_msg = {
        .header.id = 0x365,
        .header.ide = false,
        .header.rtr = false,
        .header.dlc = 6,
        .buffer = tx_buffer,
        .buffer_len = 6,
    };

    esp_err_t ret = twai_node_transmit(CAN1, &tx_msg, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "CAN TX: RPM=%u, Obj=%d, Amb=%d", g_wheel_speed, g_obj_temp,
                 g_amb_temp);
    } else {
        ESP_LOGW(TAG, "Failed to send CAN message: %s", esp_err_to_name(ret));
    }
}

void initializeI2C() {
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL,
        .sda_io_num = I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus));
}

void MLX90642_Initialize() {
    i2c_device_config_t mlx_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SA_90642_DEFAULT,
        .scl_speed_hz = 100000,
    };
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &mlx_cfg, &mlx_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MLX90642 device: %s", esp_err_to_name(ret));
        return;
    }

    // Wake up sensor
    uint8_t wake_cmd = 0x57;
    i2c_master_transmit(mlx_dev, &wake_cmd, 1, 100);
    vTaskDelay(pdMS_TO_TICKS(50));

    uint16_t id[4];
    if (MLX90642_I2CRead(SA_90642_DEFAULT, 0x1230, 4, id) != 0) {
        ESP_LOGE(TAG, "MLX90642 not responding");
        return;
    }
    ESP_LOGI(TAG, "MLX90642 ID: %04X %04X %04X %04X", id[0], id[1], id[2], id[3]);

    // Ensure sensor is in continuous mode
    if (MLX90642_GetMeasMode(SA_90642_DEFAULT) == 0x0800) {
        MLX90642_SetMeasMode(SA_90642_DEFAULT, 0x0000);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    int refresh_rate = MLX90642_GetRefreshRate(SA_90642_DEFAULT);
    if (refresh_rate < 0) {
        ESP_LOGE(TAG, "Failed to get refresh rate");
        return;
    }
    int refresh_time = 2000 >> refresh_rate;

    MLX90642_ClearDataReady(SA_90642_DEFAULT);
    MLX90642_StartSync(SA_90642_DEFAULT);
    vTaskDelay(pdMS_TO_TICKS(refresh_time));

    // Wait for first measurement
    int ready = MLX90642_NO;
    for (int i = 0; i < 100 && ready == MLX90642_NO; i++) {
        ready = MLX90642_IsDataReady(SA_90642_DEFAULT);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (ready != MLX90642_YES) {
        ESP_LOGE(TAG, "Sensor initialization timeout");
        return;
    }

    ESP_LOGI(TAG, "MLX90642 initialized (refresh: %dms)", refresh_time);
}

static void gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(he_evt_queue, &gpio_num, NULL);
}

static void he_task(void *arg) {
    uint32_t io_num;
    uint32_t last_time = 0;
    uint32_t time_diff = 0;
    const uint32_t MIN_INTERVAL_MS = 10; // Ignore triggers faster than 10ms

    for (;;) {
        if (xQueueReceive(he_evt_queue, &io_num, portMAX_DELAY)) {
            uint32_t current_time = esp_timer_get_time() / 1000;

            if (last_time > 0) {
                time_diff = current_time - last_time;

                if (time_diff > MIN_INTERVAL_MS) {
                    // RPM = (60000 ms/min) / (time_diff_ms * num_magnets)
                    uint32_t rpm = (60000) / (time_diff * 20);
                    g_wheel_speed = rpm;
                    ESP_LOGI(TAG, "Magnet pass - RPM: %lu (interval: %lu ms)", rpm,
                             time_diff);
                }
            }

            last_time = current_time;

            vTaskDelay(pdMS_TO_TICKS(5));

            while (xQueueReceive(he_evt_queue, &io_num, 0)) {
            }
        }
    }
}

static void mlx_task(void *arg) {
    uint16_t *data = malloc(MLX_TOTAL_PIXELS * sizeof(uint16_t));
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return;
    }

    for (;;) {
        int ready = MLX90642_NO;
        for (int timeout = 0; timeout < 100 && ready == MLX90642_NO; timeout++) {
            ready = MLX90642_IsDataReady(SA_90642_DEFAULT);
            if (ready < 0)
                break;
            if (ready == MLX90642_NO)
                vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (ready == MLX90642_YES) {
            if (MLX90642_GetImage(SA_90642_DEFAULT, data) == 0) {
                g_amb_temp = 1;
                g_obj_temp = 1;
            }

            MLX90642_ClearDataReady(SA_90642_DEFAULT);
            MLX90642_StartSync(SA_90642_DEFAULT);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    free(data);
}

static void can_tx_task(void *arg) {
    for (;;) {
        sendCan1Message();
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Wheelboard");
    vTaskDelay(pdMS_TO_TICKS(100));

    initializeCan();
    initializeSPI();
    if (mcp_init(CAN2) != ESP_OK) {
        ESP_LOGE(TAG, "CAN init failed!");
    }
    initializeI2C();

    MLX90642_Initialize();

    xTaskCreate(mlx_task, "mlx_task", 4096, NULL, 10, NULL);
    xTaskCreate(can_tx_task, "can_tx_task", 4096, NULL, 9, NULL);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_6),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);

    he_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(he_task, "he_task", 4096, NULL, 10, NULL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_6, gpio_isr_handler, (void *) GPIO_NUM_6);

    ESP_LOGI(TAG, "Everything initialized\n");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
