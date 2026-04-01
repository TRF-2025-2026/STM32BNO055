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
#include "main.h" // Includes HAL and I2C handles

/* --- Constants --- */
#define ACC_MEASURE_PERIOD 91
#define IMU_NUMBER_OF_BYTES 6

/* --- Global Variables for Synchronization and Data --- */
// Using 'extern' so these can be accessed in main.c without re-declaring them
extern volatile uint8_t dma_rx_complete;
extern volatile uint8_t imu_reading[IMU_NUMBER_OF_BYTES];

/* --- Initialization and Communication Functions --- */

/**
 * @brief Standardized to return HAL_StatusTypeDef for better error tracking
 */
HAL_StatusTypeDef BNO055_Init_I2C(I2C_HandleTypeDef* hi2c_device);

/** * @brief DMA-based data acquisition
 * Note: These now return HAL_StatusTypeDef instead of uint8_t
 * to allow for HAL_BUSY or HAL_ERROR detection in the main loop.
 */
HAL_StatusTypeDef GetAccelData(I2C_HandleTypeDef* hi2c_device, uint8_t* buf);
HAL_StatusTypeDef GetAccelChipId(I2C_HandleTypeDef* hi2c_device, uint8_t *chip_id_dest);

/**
 * @brief Corrected to use a pointer destination to prevent DMA memory corruption.
 */
HAL_StatusTypeDef GetAccelTemp(I2C_HandleTypeDef* hi2c_device, uint8_t *temp_dest);
HAL_StatusTypeDef GetCalibration(I2C_HandleTypeDef* hi2c_device, uint8_t *cal_dest);

/* --- Calibration Logic Functions --- */

/**
 * @brief Parses a single calibration byte into sub-sensor components.
 */
uint8_t BNO055_Calib_Calc(uint8_t calibration, uint8_t *cal_system, uint8_t *cal_gyro,
                         uint8_t *cal_acc, uint8_t *cal_mag, bool *fully_calibrated);

#endif /* INC_ACCEL_H_ */
