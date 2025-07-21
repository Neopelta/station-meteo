#include "sensor_platform.h"
#include "lps22hh_reg.h"
#include "hts221_reg.h"

int32_t platform_write_lps22(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    HAL_I2C_Mem_Write(handle, LPS22HH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
    return 0;
}

int32_t platform_read_lps22(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    HAL_I2C_Mem_Read(handle, LPS22HH_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    return 0;
}

int32_t platform_write_hts221(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    reg |= 0x80;
    HAL_I2C_Mem_Write(handle, HTS221_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)bufp, len, 1000);
    return 0;
}

int32_t platform_read_hts221(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    reg |= 0x80;
    HAL_I2C_Mem_Read(handle, HTS221_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    return 0;
}

void platform_delay(uint32_t ms)
{
    HAL_Delay(ms);
}
