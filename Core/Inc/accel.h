/*
 * accel.h
 *
 * Created on: Jan 3, 2026
 * Author: chait
 */

#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

#define ACC_MEASURE_PERIOD 91
#define IMU_NUMBER_OF_BYTES 6 // Matches the acceleration-only read in main.c

/* Global Variables for Synchronization and Data */
// volatile ensures the compiler doesn't optimize away the flag modified in an ISR
extern volatile uint8_t dma_rx_complete;
extern uint8_t imu_reading[IMU_NUMBER_OF_BYTES];
extern HAL_StatusTypeDef status;

/* Initialization and Communication Functions */
// Rewritten to support blocking initialization for stability
HAL_StatusTypeDef BNO055_Init_I2C(I2C_HandleTypeDef* hi2c_device);

// DMA-based data acquisition
uint8_t GetAccelData(I2C_HandleTypeDef* hi2c_device, uint8_t* buf);
uint8_t GetAccelChipId(I2C_HandleTypeDef* hi2c_device, uint8_t *chip_id);
uint8_t GetAccelTemp(I2C_HandleTypeDef* hi2c_device);

/* Calibration Functions */
// Updated to match implementation in accel.c
uint8_t GetCalibration(I2C_HandleTypeDef* hi2c_device);
uint8_t BNO055_Calib_Calc(uint8_t calibration, uint8_t *cal_system, uint8_t *cal_gyro, uint8_t *cal_acc, uint8_t *cal_mag, bool *fully_calibrated);

#endif
