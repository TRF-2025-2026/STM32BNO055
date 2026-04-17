#ifndef EEPROM_H_
#define EEPROM_H_

#include "stm32f4xx_hal.h"
#define NB_OF_VAR ((uint16_t)11)

// Virtual Address Mapping for BNO055 Calibration
#define EE_ADDR_ACC_X ((uint16_t)0x0001)
#define EE_ADDR_ACC_Y ((uint16_t)0x0002)
#define EE_ADDR_ACC_Z ((uint16_t)0x0003)
#define EE_ADDR_MAG_X ((uint16_t)0x0004)
#define EE_ADDR_MAG_Y ((uint16_t)0x0005)
#define EE_ADDR_MAG_Z ((uint16_t)0x0006)
#define EE_ADDR_GYR_X ((uint16_t)0x0007)
#define EE_ADDR_GYR_Y ((uint16_t)0x0008)
#define EE_ADDR_GYR_Z ((uint16_t)0x0009)
#define EE_ADDR_ACC_RADIUS ((uint16_t)0x000A)
#define EE_ADDR_MAG_RADIUS ((uint16_t)0x000B)

uint16_t EE_Init(void);
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);
#endif /* EEPROM_H_ */
