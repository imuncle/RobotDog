#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "stm32f4xx.h"

#define IIC_GPIO GPIOB
#define SDA GPIO_PIN_11
#define SCL GPIO_PIN_10

int IIC_ReadData(uint8_t dev_addr, uint8_t reg_addr, uint8_t * pdata, uint8_t count);
int IIC_WriteData(uint8_t dev_addr,uint8_t reg_addr,uint8_t data);

#endif
