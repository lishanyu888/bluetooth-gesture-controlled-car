#ifndef CAR_I2C_SOFT_H
#define CAR_I2C_SOFT_H

#include <stdint.h>

#include "main.h"

#define SOFT_I2C_SCL_PORT GPIOC
#define SOFT_I2C_SCL_PIN  GPIO_PIN_15
#define SOFT_I2C_SDA_PORT GPIOC
#define SOFT_I2C_SDA_PIN  GPIO_PIN_14

void SoftI2C_Init(void);
void SoftI2C_Recover(void);
uint8_t SoftI2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
uint8_t SoftI2C_ReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
uint8_t SoftI2C_ReadBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

#endif /* CAR_I2C_SOFT_H */
