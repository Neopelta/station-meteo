#ifndef SENSOR_PLATFORM_H
#define SENSOR_PLATFORM_H

#include "stm32l4xx_hal.h"  // Pour HAL_Delay, HAL_I2C_Mem_Write, etc.
#include "i2c.h"            // Pour I2C_HandleTypeDef, hi2c1

// Assurez-vous que hi2c1 est déclaré en externe
extern I2C_HandleTypeDef hi2c1;
#define SENSOR_BUS hi2c1

// Pour le capteur LPS22HH (pression)
int32_t platform_write_lps22(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read_lps22(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

// Pour le capteur HTS221 (température/humidité)
int32_t platform_write_hts221(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read_hts221(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

// Fonction de délai
void platform_delay(uint32_t ms);

#endif /* SENSOR_PLATFORM_H */
