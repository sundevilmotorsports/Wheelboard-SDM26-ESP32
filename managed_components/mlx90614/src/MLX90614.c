#include "MLX90614.h"


esp_err_t MLX90614_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev, uint8_t slaveAddr, uint32_t sclSpeed){
    if (!bus || !dev || slaveAddr < 0x08 || slaveAddr > 0x77) return ESP_ERR_INVALID_ARG;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = slaveAddr,
        .scl_speed_hz = sclSpeed
    };
    esp_err_t ret = i2c_master_bus_add_device(bus, &cfg, dev);
    if(ret != ESP_OK) return ret;
    if(*dev == NULL) return ESP_FAIL;
    for(int i = 0; i < 100; i++){
        bool POR_Status = 0;
        ret = MLX90614_CheckFlag(*dev,slaveAddr,&POR_Status,MLX90614_FLAG_POR);
        if(ret != ESP_OK) return ret;
        if(POR_Status) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_MLX90614_INIT_TIMEOUT;
}

esp_err_t MLX90614_I2CRead(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t startAddress, uint16_t nAddrRead,  uint16_t *rData){
    if (!rData || nAddrRead == 0) return ESP_ERR_INVALID_ARG;
    uint8_t cmd;
    uint8_t buf[3];
    uint8_t addrW = (slaveAddr << 1) | 0;
    uint8_t addrR = (slaveAddr << 1) | 1;

    for (uint16_t i = 0; i < nAddrRead; i++) {
        cmd = (uint8_t)(startAddress + i);
        esp_err_t ret = i2c_master_transmit_receive(dev, &cmd, 1, buf, 3, 1000 );
        if (ret != ESP_OK) return ret;
        uint8_t pec_data[5] = {addrW, cmd, addrR, buf[0], buf[1]};
        uint8_t pec = MLX90614_CalcPEC(pec_data, 5);
        if (pec != buf[2]) return ESP_ERR_INVALID_CRC;
        rData[i] = ((uint16_t)buf[1] << 8) | buf[0];
    }
    return ESP_OK;
}

esp_err_t MLX90614_I2CWrite(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t startAddress, uint16_t wData){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint8_t buf[3];
    buf[0] = (uint8_t)startAddress;
    buf[1] = (uint8_t)(wData & 0xFF);
    buf[2] = (uint8_t)(wData >> 8);
    esp_err_t ret = i2c_master_transmit(dev, buf, sizeof(buf), 1000 );
    if(ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));
    uint16_t temp = 0;
    bool eeprom_busy = true;
    uint32_t start_time = esp_timer_get_time() / 1000;
    while(eeprom_busy && (esp_timer_get_time() / 1000) - start_time < 25){    
        MLX90614_CheckFlag(dev,slaveAddr, &eeprom_busy, MLX90614_FLAG_EEPROM_BUSY);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if(eeprom_busy) return ESP_ERR_MLX90614_WRITE_FAIL;
    ret = MLX90614_I2CRead(dev, slaveAddr, startAddress, 1, &temp);
    if(ret != ESP_OK) return ret;
    return (temp == wData)?ESP_OK:ESP_ERR_MLX90614_WRITE_FAIL;
}

esp_err_t MLX90614_I2CCmd(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t i2c_cmd){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint8_t cmd = (uint8_t)i2c_cmd;
    return i2c_master_transmit(dev, &cmd, 1, 1000);
}

esp_err_t MLX90614_DumpEE(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *eeData){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    return MLX90614_I2CRead(dev, slaveAddr, 0x20, 32, eeData);
}

esp_err_t MLX90614_DumpRAM(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *ramData){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    return MLX90614_I2CRead(dev, slaveAddr, 0x00, 32, ramData);
}

esp_err_t MLX90614_GetObjectMaxTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *objMaxTemp){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    return MLX90614_I2CRead(dev, slaveAddr, 0x20, 1, objMaxTemp);
}

esp_err_t MLX90614_SetObjectMaxTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t wObjMaxTemp){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    return MLX90614_I2CWrite(dev, slaveAddr, 0x20, wObjMaxTemp);
}

esp_err_t MLX90614_GetObjectMinTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *objMinTemp){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    return MLX90614_I2CRead(dev, slaveAddr, 0x21, 1, objMinTemp);
}

esp_err_t MLX90614_SetObjectMinTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t wObjMinTemp){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    return MLX90614_I2CWrite(dev, slaveAddr, 0x21, wObjMinTemp);
}

esp_err_t MLX90614_GetAmbiantRange(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *ambRange){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    return MLX90614_I2CRead(dev, slaveAddr, 0x23, 1, ambRange);
}

esp_err_t MLX90614_SetAmbiantRange(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t wAmbRange){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    return MLX90614_I2CWrite(dev, slaveAddr, 0x23, wAmbRange);
}

esp_err_t MLX90614_GetEmissivity(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float *emissivity){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t raw;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x24, 1, &raw);
    if (ret != ESP_OK) return ret;
    *emissivity = raw / 65535.0f;
    return ESP_OK;
}

esp_err_t MLX90614_SetEmissivity(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float wEmissivity){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t raw = (uint16_t)(wEmissivity * 65535.0f);
    return MLX90614_I2CWrite(dev, slaveAddr, 0x24, raw);
}

esp_err_t MLX90614_GetIIR(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t *iir){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t tmp;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x25, 1, &tmp);
    if (ret != ESP_OK) return ret;
    *iir = (tmp & IIR_MASK);
    return ret;
}

esp_err_t MLX90614_SetIIR(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t wIir){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t reg;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x25, 1, &reg);
    if (ret != ESP_OK) return ret;
    reg = (reg & ~IIR_MASK) | (wIir & IIR_MASK);
    return MLX90614_I2CWrite(dev, slaveAddr, 0x25, reg);
}

esp_err_t MLX90614_GetSensorCount(i2c_master_dev_handle_t dev, uint8_t slaveAddr, bool *num){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t tmp;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x25, 1, &tmp);
    if (ret != ESP_OK) return ret;
    *num = (tmp & IR_COUNT_MASK) != 0;
    return ret;
}

esp_err_t MLX90614_GetKsSign(i2c_master_dev_handle_t dev, uint8_t slaveAddr, bool *num){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t tmp;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x25, 1, &tmp);
    *num = (tmp & KS_SIGN_MASK) != 0;
    return ret;
}

esp_err_t MLX90614_GetFIR(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t *fir){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t tmp;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x25, 1, &tmp);
    *fir = (tmp & FIR_MASK) >> FIR_SHIFT;
    return ret;
}

esp_err_t MLX90614_SetFIR(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t wFir){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    if (wFir <= 3) ESP_LOGW("Set FIR", "Melexis recommends to use higher than 64 (2^(wFir)) filter");
    uint16_t reg;
    esp_err_t ret;
    ret = MLX90614_I2CRead(dev, slaveAddr, 0x25, 1, &reg);
    if (ret != ESP_OK) return ret;
    reg = (reg & ~FIR_MASK) | ((wFir << FIR_SHIFT) & FIR_MASK);
    return MLX90614_I2CWrite(dev, slaveAddr, 0x25, reg);
}

esp_err_t MLX90614_GetGain(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t *gain){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t tmp;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x25, 1, &tmp);
    if (ret != ESP_OK) return ret;
    *gain = (uint8_t)((tmp & GAIN_MASK) >> GAIN_SHIFT);
    return ret;
}

esp_err_t MLX90614_SetGain(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t wGain){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t reg;
    esp_err_t ret;
    ret = MLX90614_I2CRead(dev, slaveAddr, 0x25, 1, &reg);
    if (ret != ESP_OK) return ret;
    reg = (reg & ~GAIN_MASK) | ((wGain << GAIN_SHIFT) & GAIN_MASK);
    return MLX90614_I2CWrite(dev, slaveAddr, 0x25, reg);
}

esp_err_t MLX90614_GetKt2Sign(i2c_master_dev_handle_t dev, uint8_t slaveAddr, bool *num){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t tmp;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x25, 1, &tmp);
    if (ret != ESP_OK) return ret;
    *num = (tmp & KT2_SIGN_MASK) != 0;
    return ret;
}

esp_err_t MLX90614_GetI2CAddr(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t *addr){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t tmp;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x2E, 1, &tmp);
    if(ret != ESP_OK) return ret;
    *addr = (uint8_t) tmp;
    return ESP_OK;
}

esp_err_t MLX90614_SetI2CAddr(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint8_t wAddr){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    if (wAddr < 0x08 || wAddr > 0x77) return ESP_ERR_MLX90614_INVALID_ADDR;
    return MLX90614_I2CWrite(dev, slaveAddr, 0x2E, wAddr);
}

esp_err_t MLX90614_GetID(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint64_t *id){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t buf[4];
    esp_err_t err = MLX90614_I2CRead(dev, slaveAddr, 0x3C, 4, buf);
    if (err != ESP_OK) return err;
    *id = ((uint64_t)buf[3] << 48) | ((uint64_t)buf[2] << 32) | ((uint64_t)buf[1] << 16) | (uint64_t)buf[0];
    return ESP_OK;
}

esp_err_t MLX90614_GetIRdata(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *ir){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    return MLX90614_I2CRead(dev, slaveAddr, 0x04, 1, ir);
}

esp_err_t MLX90614_GetIRdata2(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *ir2){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    bool sensor_check;
    esp_err_t ret = MLX90614_GetSensorCount(dev, slaveAddr, &sensor_check);
    if(ret != ESP_OK) return ret;
    if(!sensor_check) return ESP_ERR_MLX90614_UNSUPPORTED_FEATURE;
    return MLX90614_I2CRead(dev, slaveAddr, 0x05, 1, ir2);
}

esp_err_t MLX90614_GetAmbTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float *amb){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t raw;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x06, 1, &raw);
    if (ret != ESP_OK) return ret;
    *amb = (raw * 0.02f) - 273.15f;
    return ESP_OK;
}

esp_err_t MLX90614_GetAmbObjTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float *amb, float *obj){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t buf[2];
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x06, 2, buf);
    if (ret != ESP_OK) return ret;
    *amb = (buf[0] * 0.02f) - 273.15f;
    *obj = (buf[1] * 0.02f) - 273.15f;
    return ESP_OK;
}

esp_err_t MLX90614_GetObjTemp(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float *obj){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t raw;
    esp_err_t ret = MLX90614_I2CRead(dev, slaveAddr, 0x07, 1, &raw);
    if (ret != ESP_OK) return ret;
    *obj = (raw * 0.02f) - 273.15f;
    return ESP_OK;
}

esp_err_t MLX90614_GetObjTemp2(i2c_master_dev_handle_t dev, uint8_t slaveAddr, float *obj2){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    bool sensor_check;
    esp_err_t ret = MLX90614_GetSensorCount(dev, slaveAddr, &sensor_check);
    if(ret != ESP_OK) return ret;
    if(!sensor_check) return ESP_ERR_MLX90614_UNSUPPORTED_FEATURE;
    uint16_t raw;
    ret = MLX90614_I2CRead(dev, slaveAddr, 0x08, 1, &raw);
    if (ret != ESP_OK) return ret;
    *obj2 = (raw * 0.02f) - 273.15f;
    return ESP_OK;
}

esp_err_t MLX90614_GetFlags(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint16_t *flags){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    return MLX90614_I2CRead(dev, slaveAddr, 0xF0, 1, flags);
}

esp_err_t MLX90614_CheckFlag(i2c_master_dev_handle_t dev, uint8_t slaveAddr, bool *flagStat, MLX90614_Flag_t flag){
    if (!dev) return ESP_ERR_MLX90614_DEV_UNDEFINED;
    uint16_t flags = 0;
    esp_err_t ret = MLX90614_GetFlags(dev, slaveAddr, &flags);
    if(ret != ESP_OK) return ret;
    switch (flag) {
        case MLX90614_FLAG_POR:
            *flagStat = (flags & POR_FLAG_MASK) != 0;
            break;
        case MLX90614_FLAG_EEPROM_DEAD:
            *flagStat = (flags & EEPROM_DEAD_MASK) != 0;
            break;
        case MLX90614_FLAG_EEPROM_BUSY:
            *flagStat = (flags & EEPROM_BUSY_MASK) != 0;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    } 
    return ESP_OK;
}

esp_err_t MLX90614_EnterSleep(i2c_master_dev_handle_t dev, uint8_t slaveAddr){
    uint8_t cmd = 0xFF;
    return i2c_master_transmit(dev, &cmd, 1, 1000 );
}

esp_err_t MLX90614_Wake(i2c_master_dev_handle_t dev, uint8_t slaveAddr, uint32_t patience){
    uint8_t addr = 0;
    for(int i = 0; i < patience; i++){
        esp_err_t ret = MLX90614_GetI2CAddr(dev, slaveAddr, &addr);
        if(ret == ESP_OK) return ESP_OK;
       vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_MLX90614_WAKE_TIMEOUT;
}

float MLX90614_TemperatureInFahrenheit(float temperature){
    return (temperature * 9.0f / 5.0f) + 32.0f;
}

int16_t MLX90614_ConvertIRdata(uint16_t ir){
    return (int16_t)ir;
}

uint8_t MLX90614_CalcPEC(const uint8_t *data, size_t len){
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}