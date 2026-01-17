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
#include "MLX90642.h"
#include "MLX90642_depends.h"

static const char *TAG = "Wheelboard";

twai_node_handle_t CAN1 = NULL;
#define CAN1_TX GPIO_NUM_18
#define CAN1_RX GPIO_NUM_17

#define I2C_SCL GPIO_NUM_22
#define I2C_SDA GPIO_NUM_21

#define MLX_TOTAL_PIXELS 768
#define MLX_WIDTH 24
#define MLX_HEIGHT 32

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t mlx_dev = NULL;

int MLX90642_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *rData) {
    if (mlx_dev == NULL) return -1;

    uint8_t addr_buf[2];
    addr_buf[0] = (startAddress >> 8) & 0xFF;
    addr_buf[1] = startAddress & 0xFF;

    size_t read_size = nMemAddressRead * 2;
    uint8_t *read_buf = malloc(read_size);
    if (read_buf == NULL) return -1;

    esp_err_t ret = i2c_master_transmit_receive(mlx_dev, addr_buf, sizeof(addr_buf),
                                                 read_buf, read_size, 1000);
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
    if (mlx_dev == NULL) return -1;

    esp_err_t ret = i2c_master_transmit(mlx_dev, buffer, bytesNum, 1000);
    return (ret == ESP_OK) ? 0 : -1;
}

void MLX90642_Wait_ms(uint16_t time_ms) {
    vTaskDelay(pdMS_TO_TICKS(time_ms));
}

int MLX90642_Config(uint8_t slaveAddr, uint16_t writeAddress, uint16_t wData) {
    uint8_t buffer[6];
    buffer[0] = (writeAddress >> 8) & 0xFF;   // Address MSB
    buffer[1] = writeAddress & 0xFF;          // Address LSB
    buffer[2] = 0x00;                         // Padding MSB
    buffer[3] = 0x00;                         // Padding LSB
    buffer[4] = (wData >> 8) & 0xFF;          // Data MSB
    buffer[5] = wData & 0xFF;                 // Data LSB

    return MLX90642_I2CWrite(slaveAddr, buffer, 6);
}

int MLX90642_I2CCmd(uint8_t slaveAddr, uint16_t i2c_cmd) {
    uint8_t buffer[4];
    buffer[0] = 0x01;                         // Command address MSB
    buffer[1] = 0x80;                         // Command address LSB (0x0180)
    buffer[2] = (i2c_cmd >> 8) & 0xFF;        // Command MSB
    buffer[3] = i2c_cmd & 0xFF;               // Command LSB

    return MLX90642_I2CWrite(slaveAddr, buffer, 4);
}

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

static void print_thermal_image(uint16_t *image_data) {
    uint16_t min_val = 0xFFFF;
    uint16_t max_val = 0;

    for (int i = 0; i < MLX_TOTAL_PIXELS; i++) {
        if (image_data[i] < min_val) min_val = image_data[i];
        if (image_data[i] > max_val) max_val = image_data[i];
    }

    // Gradient characters (coldest to hottest)
    const char gradient[] = " .-:=+*#%@";
    int gradient_len = sizeof(gradient) - 1;

    printf("\n╔");
    for (int x = 0; x < MLX_WIDTH; x++) printf("═");
    printf("╗\n");

    for (int y = 0; y < MLX_HEIGHT; y++) {
        printf("║");
        for (int x = 0; x < MLX_WIDTH; x++) {
            int idx = y * MLX_WIDTH + x;

            int scaled = 0;
            if (max_val > min_val) {
                scaled = ((image_data[idx] - min_val) * gradient_len) / (max_val - min_val);
                if (scaled >= gradient_len) scaled = gradient_len - 1;
            }

            printf("%c", gradient[scaled]);
        }
        printf("║\n");
    }

    printf("╚");
    for (int x = 0; x < MLX_WIDTH; x++) printf("═");
    printf("╝\n");

    ESP_LOGI(TAG, "Temp range: %u - %u (raw values)", min_val, max_val);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting Wheelboard");

    initializeCan();

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_SCL,
        .sda_io_num = I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus));

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

    // Manual initialization (more reliable than MLX90642_Init)
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

    uint16_t *data = malloc(MLX_TOTAL_PIXELS * sizeof(uint16_t));
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return;
    }

    while (1) {
        int ready = MLX90642_NO;
        for (int timeout = 0; timeout < 100 && ready == MLX90642_NO; timeout++) {
            ready = MLX90642_IsDataReady(SA_90642_DEFAULT);
            if (ready < 0) break;
            if (ready == MLX90642_NO) vTaskDelay(pdMS_TO_TICKS(10));
        }

        if (ready == MLX90642_YES) {
            if (MLX90642_GetImage(SA_90642_DEFAULT, data) == 0) {
                print_thermal_image(data);
            }

            MLX90642_ClearDataReady(SA_90642_DEFAULT);
            MLX90642_StartSync(SA_90642_DEFAULT);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    free(data);
}
