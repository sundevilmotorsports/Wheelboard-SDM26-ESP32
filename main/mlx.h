#ifndef WHEELBOARD_SDM26_ESP32_MLX_H
#define WHEELBOARD_SDM26_ESP32_MLX_H

#include "driver/i2c_master.h"
#include "MLX90642.h"
#include "MLX90642_depends.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define MLX_WIDTH 24
#define MLX_HEIGHT 32
#define MLX_TOTAL_PIXELS MLX_WIDTH * MLX_HEIGHT
#define MLX_DEBUG false

void mlx_camera_task(void* args);
esp_err_t MLX90642_Initialize(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *mlx_dev, uint16_t *data);
void MLX90642_Wait_ms(uint16_t time_ms);
int MLX90642_I2CWrite(uint8_t slaveAddr, uint8_t *buffer, uint8_t bytesNum);

#endif