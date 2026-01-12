/*
 * accel.h
 *
 *  Created on: Jan 3, 2026a
 *      Author: chait
 */

#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_
#include <stdint.h>
#include "i2c.h"

#define ACC_MEASURE_PERIOD 91
#define	IMU_NUMBER_OF_BYTES 18

extern uint8_t	imu_readings[IMU_NUMBER_OF_BYTES];
extern HAL_StatusTypeDef status;

void BNO055_Init(void);
HAL_StatusTypeDef BNO055_Init_I2C(I2C_HandleTypeDef* hi2c_device);
void readAccelData(int16_t * destination);
uint8_t GetAccelData(I2C_HandleTypeDef* hi2c_device, uint8_t* str);
uint8_t GetAccelChipId(I2C_HandleTypeDef* hi2c_device, uint8_t *chip_id);
uint8_t GetAccelTemp(I2C_HandleTypeDef* hi2c_device);
uint8_t BNO055_Get_Calibration(I2C_HandleTypeDef* hi2c_device);
void BNO055_Calc_Calibration(uint8_t calibration, uint8_t *cal_system, uint8_t *cal_gyro, uint8_t *cal_acc, uint8_t *cal_mag);

#endif
