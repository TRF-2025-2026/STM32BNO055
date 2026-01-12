/*
 * i2c.h
 *
 *  Created on: Jan 4, 2026
 *      Author: chait
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_
#include "stm32f4xx_hal.h"
#include "bno055.h"

//#define BNO055_I2C_ADDR_LO     (0x28 << 1)   // AD0 = LOW
//#define BNO055_I2C_ADDR_HI     (0x29 << 1)   // AD0 = HIGH

extern I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef BNO055_WriteReg(uint8_t reg, uint8_t data);
HAL_StatusTypeDef BNO055_WriteMulti(uint8_t reg, uint8_t *data, uint8_t len);

HAL_StatusTypeDef BNO055_ReadReg(uint8_t reg, uint8_t *data);
HAL_StatusTypeDef BNO055_ReadMulti(uint8_t reg, uint8_t *buffer, uint8_t len);
void MX_I2C1_Init(void);


#endif /* INC_I2C_H_ */
