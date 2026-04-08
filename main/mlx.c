#include "mlx.h"
static i2c_master_dev_handle_t *dev;
static bool mlx_ready = false;
uint16_t data[MLX_TOTAL_PIXELS];

esp_err_t MLX90642_Initialize(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *mlx_dev, uint16_t *data) {
    const char* LOCAL_TAG = "MLX CAMERA INIT";
    dev = mlx_dev;
    i2c_device_config_t mlx_camera_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SA_90642_DEFAULT,
        .scl_speed_hz = 100000,
    };
     esp_err_t ret = i2c_master_bus_add_device(bus, &mlx_camera_cfg, dev);
    if (ret != ESP_OK) return ret;
    uint8_t wake_cmd = 0x57;
    ret = i2c_master_transmit(*dev, &wake_cmd, 1, 100);
    if(ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));
    uint16_t id[4];
    ret = MLX90642_I2CRead(SA_90642_DEFAULT, 0x1230, 4, id);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(LOCAL_TAG, "MLX90642 ID: %04X %04X %04X %04X", id[0], id[1], id[2], id[3]);

    if (MLX90642_GetMeasMode(SA_90642_DEFAULT) == 0x0800) {
        MLX90642_SetMeasMode(SA_90642_DEFAULT, 0x0000);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    int refresh_rate = MLX90642_GetRefreshRate(SA_90642_DEFAULT);
    if (refresh_rate < 0) {
        ESP_LOGE(LOCAL_TAG, "Failed to get refresh rate");
        return ESP_FAIL;
    }
    int refresh_time = 2000 >> refresh_rate;

    MLX90642_ClearDataReady(SA_90642_DEFAULT);
    MLX90642_StartSync(SA_90642_DEFAULT);
    vTaskDelay(pdMS_TO_TICKS(refresh_time));

    // Wait for first measurement
    int ready = MLX90642_NO;
    for (int i = 0; i < 100 && ready == MLX90642_NO; i++) {
        if(MLX90642_IsDataReady(SA_90642_DEFAULT) == MLX90642_YES){
            ESP_LOGI(LOCAL_TAG, "MLX90642 initialized (refresh: %dms)", refresh_time);
            mlx_ready = true;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(LOCAL_TAG, "Sensor initialization timeout");
    return ESP_ERR_TIMEOUT;    
}

// MLX 90642 Thermal Array 
int MLX90642_I2CWrite(uint8_t slaveAddr, uint8_t *buffer, uint8_t bytesNum) {
    return (i2c_master_transmit(*dev, buffer, bytesNum, 1000) == ESP_OK) ? 0 : -1;
}

int MLX90642_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *rData){
    if (dev == NULL) return -1;

    uint8_t addr_buf[2];
    addr_buf[0] = (startAddress >> 8) & 0xFF;
    addr_buf[1] = startAddress & 0xFF;

    size_t read_size = nMemAddressRead * 2;
    uint8_t *read_buf = malloc(read_size);
    if (read_buf == NULL)return -1;

    esp_err_t ret = i2c_master_transmit_receive(*dev, addr_buf, sizeof(addr_buf), read_buf, read_size, 1000);
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

int MLX90642_I2CCmd(uint8_t slaveAddr, uint16_t i2c_cmd){
    uint8_t buffer[4];
    buffer[0] = 0x01; // Command address MSB
    buffer[1] = 0x80; // Command address LSB (0x0180)
    buffer[2] = (i2c_cmd >> 8) & 0xFF; // Command MSB
    buffer[3] = i2c_cmd & 0xFF; // Command LSB

    return MLX90642_I2CWrite(slaveAddr, buffer, 4);
}


void mlx_camera_task(void* args) {
    const char* LOCAL_TAG = "MLX CAMERA TASK";
    if (!mlx_ready) {
        ESP_LOGE(LOCAL_TAG, "MLX not initialized, task exiting");
        vTaskDelete(NULL);
        return;
    }
    for (;;) {
        if(MLX90642_IsDataReady(SA_90642_DEFAULT) == MLX90642_YES){
            if (MLX90642_GetImage(SA_90642_DEFAULT, data) == 0) {
                if (MLX_DEBUG) ESP_LOGI(LOCAL_TAG,"MLX READ");
            } else ESP_LOGE(LOCAL_TAG, "Failed to get image");
            MLX90642_ClearDataReady(SA_90642_DEFAULT);
            MLX90642_StartSync(SA_90642_DEFAULT);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else if (MLX_DEBUG) ESP_LOGW(LOCAL_TAG, "MLX Not Ready");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}