#ifndef _MLX90614_H
#define _MLX90614_H

#include <stdint.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#define ESP_ERR_MLX_BASE                         0xE000

#define ESP_ERR_MLX90614_INIT_TIMEOUT            ESP_ERR_MLX_BASE
#define ESP_ERR_MLX90614_WAKE_TIMEOUT           (ESP_ERR_MLX_BASE + 1)
#define ESP_ERR_MLX90614_PROTECTED_MEM          (ESP_ERR_MLX_BASE + 2)
#define ESP_ERR_MLX90614_BUS_UNDEFINED          (ESP_ERR_MLX_BASE + 3)
#define ESP_ERR_MLX90614_DEV_UNDEFINED          (ESP_ERR_MLX_BASE + 4)
#define ESP_ERR_MLX90614_INVALID_ADDR           (ESP_ERR_MLX_BASE + 5)
#define ESP_ERR_MLX90614_UNSUPPORTED_FEATURE    (ESP_ERR_MLX_BASE + 6)
#define ESP_ERR_MLX90614_WRITE_FAIL             (ESP_ERR_MLX_BASE + 7)

#define MLX90614_DEFAULT_ADDRESS 0x5A
#define MLX90614_DEFAULT_SPEED 100000

// #define IR_COUNT_SHIFT      6
// #define KS_SIGN_SHIFT       7
#define FIR_SHIFT           8
#define GAIN_SHIFT          11
// #define KT2_SIGN_SHIFT      14

#define IIR_MASK            0x0007
#define IR_COUNT_MASK       0x0040
#define KS_SIGN_MASK        0x0080
#define FIR_MASK            0x0700
#define GAIN_MASK           0x3800
#define KT2_SIGN_MASK       0x4000

#define POR_FLAG_MASK       0x10
#define EEPROM_DEAD_MASK    0x20
#define EEPROM_BUSY_MASK    0x80

typedef enum {
    MLX90614_FLAG_POR,
    MLX90614_FLAG_EEPROM_DEAD,
    MLX90614_FLAG_EEPROM_BUSY
} MLX90614_Flag_t;

/** MLX90614 initialize, passes back the pointer for the dev handler
 *
 * @param[in] bus i2c bus handler
 * @param[out] dev returned i2c device handler
 * @param[in] alaveAddr MLX90614 slave address, default 0x5A
 * @param[in] sclSpeed i2c bus speed in Hz, @note must be 10 kHz <= sclSpeed <= 100 kHz
 *
 */
esp_err_t MLX90614_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev, uint8_t slaveAddr, uint32_t sclSpeed);

/** MLX90614 block read I2C command
 *
 * @param[in] dev i2c device handler
 * @param[in] slaveAddr I2C slave address of the device
 * @param[in] startAddress Start address for the block read
 * @param[in] nAddrRead Number of words to read
 * @param[out] rData Pointer to where the read data will be stored
 *
 *
 */
esp_err_t MLX90614_I2CRead(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t startAddress, uint16_t nAddrRead, uint16_t *rData);

/** MLX90614 configuration I2C command
 * @note For more information refer to the MLX90614 datasheet
 *
 * @param[in] dev i2c device handler
 * @param[in] slaveAddr I2C slave address of the device
 * @param[in] startAddress Configuration address to write to
 * @param[in] wData Data to write
 *
 */
esp_err_t MLX90614_I2CWrite(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t startAddress, uint16_t wData);

/** MLX90614 I2C commands send
 * @note The addressed reset, start/sync measurement and sleep commands ahsre the same I2C format. For more information refer to the MLX90614 datasheet
 *
 * @param[in] dev i2c device handler
 * @param[in] slaveAddr I2C slave address of the device
 * @param[in] i2c_cmd MLX90614 I2C command to send
 *
 */
esp_err_t MLX90614_I2CCmd(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t i2c_cmd);

/** MLX90614 dump EEPROM Memory command
 *
 * @param[in] dev i2c device handler
 * @param[in] slaveAddr I2C slave address of the device
 * @param[out] eeData Pointer to where the EEPROM data will be stored
 *
 */
esp_err_t MLX90614_DumpEE(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *eeData);

/** MLX90614 dump RAM Memory command
 *
 * @param[in] dev i2c device handler
 * @param[in] slaveAddr I2C slave address of the device
 * @param[out] ramData Pointer to where the RAM data will be stored
 *
 */
esp_err_t MLX90614_DumpRAM(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *ramData);

/** Get the object max temperature of the MLX90614 device for scaling */
esp_err_t MLX90614_GetObjectMaxTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *objMaxTemp);

/** Set the object max temperature of the MLX90614 device for scaling */
esp_err_t MLX90614_SetObjectMaxTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t wObjMaxTemp);

/** Get the object min temperature of the MLX90614 device for scaling */
esp_err_t MLX90614_GetObjectMinTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *objMinTemp);

/** Set the object max temperature of the MLX90614 device for scaling */
esp_err_t MLX90614_SetObjectMinTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t wObjMinTemp);

/** Get the ambiant temperature range of the MLX90614 device for scaling */
esp_err_t MLX90614_GetAmbiantRange( i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *ambRange);

/** Set the ambiant temperature range of the MLX90614 device for scaling */
esp_err_t MLX90614_SetAmbiantRange(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t wAmbRange);

/** Get the emissivity used by the MLX90614 device to compensate the data */
esp_err_t MLX90614_GetEmissivity(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float *emissivity);

/** Set the emissivity used by the MLX90642 device to compensate the data */
esp_err_t MLX90614_SetEmissivity(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float wEmissivity);

/** Get the IIR used by the MLX90614 device to filter the data */
esp_err_t MLX90614_GetIIR(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t *iir);

/** Set the IIR used by the MLX90614 device to filter the data */
esp_err_t MLX90614_SetIIR(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t wIir);

/** Set the number of sensors included in th MLX90614 device */
esp_err_t MLX90614_GetSensorCount(i2c_master_dev_handle_t dev, uint8_t slaveAddr, bool *num);

/** Get the Ks sign used by the MLX90614 device */
esp_err_t MLX90614_GetKsSign(i2c_master_dev_handle_t dev, uint8_t slaveAddr, bool *num);

/** Get the FIR used by the MLX90614 device to filter the data */
esp_err_t MLX90614_GetFIR(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t *fir);

/** Set the FIR used by the MLX90614 device to filter the data */
esp_err_t MLX90614_SetFIR(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t wFir);

/** Get the gain used by the MLX90614 device to amplify the data */
esp_err_t MLX90614_GetGain(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t *gain);

/** Set the gain used by the MLX90614 device to amplify the data */
esp_err_t MLX90614_SetGain(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t wGain);

/** Get the Kt2 sign used by the MLX90614 device */
esp_err_t MLX90614_GetKt2Sign(i2c_master_dev_handle_t dev, uint8_t slaveAddr, bool *num);

/** Get the i2c address used by the MLX90614
 * @note technically redundant since you need to pass it's address to 
 * check it's address, but used for wakeup function and could verify the device exists*/
esp_err_t MLX90614_GetI2CAddr(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t *addr);

/** Set the i2c address used by the MLX90614 device */
esp_err_t MLX90614_SetI2CAddr(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t wAddr);

/** Get the identifier unique to each individual sensor */
esp_err_t MLX90614_GetID(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint64_t *id);

/** Get the IR data from the main sensor on the device */
esp_err_t MLX90614_GetIRdata(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *ir);

/** Get the IR data from the second sensor on the device */
esp_err_t MLX90614_GetIRdata2(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *ir2);

/** Get the ambiant temperature from the device */
esp_err_t MLX90614_GetAmbTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float *amb);

/** Sequential read of the ambiant and object temperature from the device */
esp_err_t MLX90614_GetAmbObjTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float *amb, float *obj);

/** Get the object temperature from the main sensor on the device */
esp_err_t MLX90614_GetObjTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float *obj);

/** Get the object temperature from the second sensor on the device */
esp_err_t MLX90614_GetObjTemp2(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float *obj2);

/** Get the status flags from MLX90614 */
esp_err_t MLX90614_GetFlags(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *flags);

/** Get the individual status flags from MLX90614 */
esp_err_t MLX90614_CheckFlag(i2c_master_dev_handle_t dev, uint8_t slaveAddr, bool *flagStat, MLX90614_Flag_t flag);

/** Have the MLX90614 Enter sleep mode */
esp_err_t MLX90614_EnterSleep(i2c_master_dev_handle_t dev, uint8_t slaveAddr);

/** Have the MLX90614 wake up from sleep mode */
esp_err_t MLX90614_Wake(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint32_t patience);

/** Helper function for freedom units */
float MLX90614_TemperatureInFahrenheit(float temperature);

/** Helper function for converting from IR to C */
int16_t MLX90614_ConvertIRdata(uint16_t ir);

/**Error checking helper function */
uint8_t MLX90614_CalcPEC(const uint8_t *data, size_t len);

#endif