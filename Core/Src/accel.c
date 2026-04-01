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
 * @brief Initializes BNO055 using Blocking I2C for stability during boot.
 */
HAL_StatusTypeDef BNO055_Init_I2C(I2C_HandleTypeDef *hi2c_dev) {
//    HAL_StatusTypeDef res;
    uint8_t dev_addr = (BNO055_I2C_ADDR_LO << 1);
    uint8_t data[2];

    // 1. HARD RESET: Give the sensor a clean slate
    data[0] = BNO055_SYS_TRIGGER;
    data[1] = 0x20; // Self-reset bit
    res = HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, HAL_MAX_DELAY);
    HAL_Delay(700); // BNO055 needs time to reboot

    // 2. Ensure we are in CONFIGMODE on PAGE 0
    data[0] = BNO055_PAGE_ID;
    data[1] = 0x00;
    if((res = HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, HAL_MAX_DELAY)) != HAL_OK) return res;

    data[0] = BNO055_OPR_MODE;
    data[1] = 0x00; // CONFIGMODE
    if((res = HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, HAL_MAX_DELAY)) != HAL_OK) return res;
    HAL_Delay(25);

    // 3. Switch to PAGE 1 for Sensor Configuration (Ranges/Bandwidths)
    data[0] = BNO055_PAGE_ID;
    data[1] = 0x01;
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    // Configure Accel
    data[0] = BNO055_ACC_CONFIG;
    data[1] = (APwrMode << 5 | Abw << 2 | Ascale);
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    // Configure Gyro
    data[0] = BNO055_GYRO_CONFIG_0;
    data[1] = (Gbw << 3 | Gscale);
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    // Configure Mag
    data[0] = BNO055_MAG_CONFIG;
    data[1] = (MPwrMode << 5 | MOpMode << 3 | Modr);
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    // 4. Return to PAGE 0 and enter NDOF Fusion Mode
    data[0] = BNO055_PAGE_ID;
    data[1] = 0x00;
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    data[0] = BNO055_OPR_MODE;
    data[1] = 0x0C; // NDOF
    res = HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    HAL_Delay(800); // CRITICAL: Wait for Fusion engine to stabilize
    return res;
}

/**
 * @brief Starts non-blocking DMA read for Accelerometer Data.
 */
HAL_StatusTypeDef GetAccelData(I2C_HandleTypeDef* hi2c_dev, uint8_t* buf) {
    if (hi2c_dev->State != HAL_I2C_STATE_READY) {
        return HAL_BUSY;
    }
    return HAL_I2C_Mem_Read_DMA(hi2c_dev, BNO055_I2C_ADDR_LO << 1,
                                BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT,
                                buf, 6);
}

/**
 * @brief Starts non-blocking DMA read for Chip ID.
 */
HAL_StatusTypeDef GetAccelChipId(I2C_HandleTypeDef* hi2c_dev, uint8_t *chip_id_dest) {
    return HAL_I2C_Mem_Read_DMA(hi2c_dev, BNO055_I2C_ADDR_LO << 1,
                                BNO055_CHIP_ID, I2C_MEMADD_SIZE_8BIT,
                                chip_id_dest, 1);
}

/**
 * @brief Starts non-blocking DMA read for Temperature.
 */
HAL_StatusTypeDef GetAccelTemp(I2C_HandleTypeDef* hi2c_dev, uint8_t *temp_dest) {
    return HAL_I2C_Mem_Read_DMA(hi2c_dev, BNO055_I2C_ADDR_LO << 1,
                                BNO055_TEMP, I2C_MEMADD_SIZE_8BIT,
                                temp_dest, 1);
}

/**
 * @brief Starts non-blocking DMA read for Calibration Status.
 */
HAL_StatusTypeDef GetCalibration(I2C_HandleTypeDef* hi2c_dev, uint8_t *cal_dest) {
    return HAL_I2C_Mem_Read_DMA(hi2c_dev, BNO055_I2C_ADDR_LO << 1,
                                BNO055_CALIB_STAT, I2C_MEMADD_SIZE_8BIT,
                                cal_dest, 1);
}

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
