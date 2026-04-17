#include "bno055.h"
#include "accel.h"
#include "i2c.h"
#include <stdbool.h>

/* --- Configuration Constants --- */
uint8_t OPRMode   = NDOF;
uint8_t Ascale    = AFS_16G;
uint8_t APwrMode  = NormalA;
uint8_t Abw       = ABW_125Hz;
uint8_t Gscale    = GFS_2000DPS;
uint8_t GPwrMode  = NormalG;
uint8_t Gbw       = GBW_230Hz;
uint8_t MPwrMode  = Normal;
uint8_t MOpMode   = EnhancedRegular;
uint8_t Modr      = MODR_30Hz;
HAL_StatusTypeDef res;

/**
 * @brief Initializes BNO055 using Blocking I2C.
 */
HAL_StatusTypeDef BNO055_Init_I2C(I2C_HandleTypeDef *hi2c_dev) {
    uint8_t dev_addr = (BNO055_I2C_ADDR_LO << 1);
    uint8_t data[2];

    // 1. HARD RESET
    data[0] = BNO055_SYS_TRIGGER;
    data[1] = 0x20;
    res = HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);
    HAL_Delay(700);

    // 2. CONFIGMODE on PAGE 0
    data[0] = BNO055_PAGE_ID;
    data[1] = 0x00;
    if((res = HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000)) != HAL_OK) return res;

    data[0] = BNO055_OPR_MODE;
    data[1] = 0x00;
    if((res = HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000)) != HAL_OK) return res;
    HAL_Delay(25);

    // 3. Switch to PAGE 1 for Config
    data[0] = BNO055_PAGE_ID;
    data[1] = 0x01;
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    data[0] = BNO055_ACC_CONFIG;
    data[1] = (APwrMode << 5 | Abw << 2 | Ascale);
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    data[0] = BNO055_GYRO_CONFIG_0;
    data[1] = (Gbw << 3 | Gscale);
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    data[0] = BNO055_MAG_CONFIG;
    data[1] = (MPwrMode << 5 | MOpMode << 3 | Modr);
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    // 4. Return to PAGE 0 and enter NDOF
    data[0] = BNO055_PAGE_ID;
    data[1] = 0x00;
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    data[0] = BNO055_OPR_MODE;
    data[1] = 0x0C;
    res = HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    HAL_Delay(800);
    return res;
}

/**
 * @brief Reads Accelerometer Data using Polling (Blocking).
 * Cleans up the Error 16 DMA interference.
 */
HAL_StatusTypeDef GetAccelData(I2C_HandleTypeDef* hi2c_dev, uint8_t* buf) {
    // Replaced _DMA with standard Mem_Read
    return HAL_I2C_Mem_Read(hi2c_dev, BNO055_I2C_ADDR_LO << 1,
                            BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT,
                            buf, 6, 100); // 100ms timeout
}

/**
 * @brief Reads Calibration Status using Polling.
 */
HAL_StatusTypeDef GetCalibration(I2C_HandleTypeDef* hi2c_dev, uint8_t *cal_dest) {
    return HAL_I2C_Mem_Read(hi2c_dev, BNO055_I2C_ADDR_LO << 1,
                                BNO055_CALIB_STAT, I2C_MEMADD_SIZE_8BIT,
                                cal_dest, 1, 100);
}

/* --- The rest of your ID and Temp functions should also be switched to HAL_I2C_Mem_Read --- */

uint8_t BNO055_Calib_Calc(uint8_t calibration, uint8_t *cal_system, uint8_t *cal_gyro,
                         uint8_t *cal_acc, uint8_t *cal_mag, bool *fully_calibrated) {
    *cal_system = (calibration >> 6) & 0x03;
    *cal_gyro   = (calibration >> 4) & 0x03;
    *cal_acc    = (calibration >> 2) & 0x03;
    *cal_mag    = (calibration >> 0) & 0x03;

    if (*cal_system == 3 && *cal_gyro == 3 && *cal_acc == 3 && *cal_mag == 3) {
        *fully_calibrated = true;
    } else {
        *fully_calibrated = false;
    }
    return (*cal_system + *cal_gyro + *cal_acc + *cal_mag);
}

void BNO055_Get_Offsets(I2C_HandleTypeDef *hi2c, uint16_t *offsets) {
    uint8_t raw_data[22];
    if (HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR_LO << 1, 0x55, I2C_MEMADD_SIZE_8BIT, raw_data, 22, 200) == HAL_OK) {
        for (int i = 0; i < 11; i++) {
            offsets[i] = (uint16_t)((raw_data[i*2+1] << 8) | raw_data[i*2]);
        }
    }
}

void BNO055_Set_Offsets(I2C_HandleTypeDef *hi2c, uint16_t *offsets) {
    uint8_t raw_data[22];
    for (int i = 0; i < 11; i++) {
        raw_data[i*2] = (uint8_t)(offsets[i] & 0xFF);
        raw_data[i*2+1] = (uint8_t)((offsets[i] >> 8) & 0xFF);
    }
    HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR_LO << 1, 0x55, I2C_MEMADD_SIZE_8BIT, raw_data, 22, 200);
}
