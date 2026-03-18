#include "bno055.h"
#include "accel.h"
#include "i2c.h"
#include <stdbool.h>

uint8_t OPRMode = NDOF;//Operation modes like NDOF,ACCONLY,IMU,etc/

uint8_t Ascale= AFS_16G;// Accelerometer full scale
uint8_t APwrMode= NormalA;// Accelerometer power mode
uint8_t Abw= ABW_125Hz;//Accelerometer Bandwidth
uint8_t Gscale= GFS_2000DPS;// Accelerometer full scale
uint8_t GPwrMode= NormalG;// Accelerometer power mode
uint8_t Gbw= GBW_230Hz;
uint8_t MPwrMode=Normal;
uint8_t MOpMode=EnhancedRegular;
uint8_t Modr= MODR_30Hz;// Magnetometer output data rate
uint8_t PWRMode=Normal;//Power Mode
uint8_t OpMode= NDOF;

//uint8_t status;
float aRes,mRes,gRes;//Resolution
uint8_t cal_sys=0;//Calibration of system
uint8_t cal_gyro=0;
uint8_t cal_imu=0;
uint8_t cal_acc=0;
uint8_t cal_mag=0;//Calibration constants are from 0 to 4, where 4 is most calibrated and 0 is least
const uint8_t num_of_bytes_read = 38;
const char read_devid[]= {START_BYTE, REG_READ, BNO055_CHIP_ID, 0x01};//Contains Device/Chip ID
const char read_calib[2]= {REG_READ, BNO055_CALIB_STAT};//Reads Calibration Status of Sub sensors
const char reset_sensor[3]= {REG_READ, BNO055_SYS_TRIGGER,0x01};//Used to Reset sub sensors
uint8_t get_readings[1] = {BNO055_ACC_DATA_X_LSB};
HAL_StatusTypeDef BNO055_Init_I2C(I2C_HandleTypeDef *hi2c_dev) {
    HAL_StatusTypeDef res;
    uint8_t dev_addr = (BNO055_I2C_ADDR_LO << 1); // 0x28 shifted for HAL
    uint8_t data[2];

    // 1. Force Page 0 and enter CONFIGMODE
    // The BNO055 must be in CONFIGMODE to change sensor settings
    data[0] = BNO055_PAGE_ID;
    data[1] = 0x00;
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    data[0] = BNO055_OPR_MODE;
    data[1] = CONFIGMODE;
    res = HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);
    if(res != HAL_OK) return res;
    HAL_Delay(50); // Allow time for mode switch

    // 2. Setup Page 1 for Configuration
    // Page 1 contains the configuration registers for Acc, Mag, and Gyro
    data[0] = BNO055_PAGE_ID;
    data[1] = 0x01;
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    // 3. Configure Accelerometer
    data[0] = BNO055_ACC_CONFIG;
    data[1] = (APwrMode << 5 | Abw << 2 | Ascale); // Uses settings defined in accel.c
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    // 4. Configure Gyroscope
    // Fixed: Using Gyro-specific variables Gbw and Gscale
    data[0] = BNO055_GYRO_CONFIG_0;
    data[1] = (Gbw << 3 | Gscale);
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    data[0] = BNO055_GYRO_CONFIG_1;
    data[1] = GPwrMode;
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    // 5. Configure Magnetometer
    data[0] = BNO055_MAG_CONFIG;
    data[1] = (MPwrMode << 5 | MOpMode << 3 | Modr);
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);
    HAL_Delay(10);

    // 6. Return to Page 0 and set Operation Mode
    data[0] = BNO055_PAGE_ID;
    data[1] = 0x00;
    HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    // Enter NDOF (9-Degrees of Freedom Sensor Fusion mode)
    data[0] = BNO055_OPR_MODE;
    data[1] = NDOF;
    res = HAL_I2C_Master_Transmit(hi2c_dev, dev_addr, data, 2, 1000);

    // Critical: The BNO055 requires up to 800ms to start fusion in NDOF mode
    HAL_Delay(800);

    return res;
}
uint8_t GetAccelData(I2C_HandleTypeDef* hi2c_dev, uint8_t* buf) {
	if (hi2c_dev->State != HAL_I2C_STATE_READY) {
	        return HAL_BUSY;
	    }
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read_DMA(hi2c_dev,BNO055_I2C_ADDR_LO << 1,BNO055_ACC_DATA_X_LSB,I2C_MEMADD_SIZE_8BIT,buf,6);
    return (uint8_t)status;
}//Read data via I2C communication

uint8_t GetAccelChipId(I2C_HandleTypeDef* hi2c_dev, uint8_t *chip_id) {
	return HAL_I2C_Mem_Read_DMA(hi2c_dev, BNO055_I2C_ADDR_LO<<1, BNO055_CHIP_ID, I2C_MEMADD_SIZE_8BIT, chip_id, 1);
}//Read Chip ID


uint8_t GetAccelTemp(I2C_HandleTypeDef* hi2c_dev){
	uint8_t temp;
	HAL_I2C_Mem_Read_DMA(hi2c_dev, BNO055_I2C_ADDR_LO<<1, BNO055_TEMP,I2C_MEMADD_SIZE_8BIT, &temp,1);
	return temp;
	}

uint8_t GetCalibration(I2C_HandleTypeDef* hi2c_dev){
	uint8_t cal;
	HAL_I2C_Mem_Read_DMA(hi2c_dev, BNO055_I2C_ADDR_LO<<1, BNO055_CALIB_STAT,I2C_MEMADD_SIZE_8BIT, &cal,1);
	return cal;
}//Read Calibration constants

uint8_t BNO055_Calib_Calc(uint8_t calibration,uint8_t *cal_system,uint8_t *cal_gyro,uint8_t *cal_acc,uint8_t *cal_mag,bool *fully_calibrated){
	*cal_system = (calibration >> 6) & 0x03;
	*cal_gyro   = (calibration >> 4) & 0x03;
	*cal_acc    = (calibration >> 2) & 0x03;
	*cal_mag    = (calibration >> 0) & 0x03;
	uint8_t total = *cal_system + *cal_gyro + *cal_acc + *cal_mag;//Total Calibration status of all sub sensors and collective
	if (*cal_system == 3 && *cal_gyro == 3 && *cal_acc == 3 && *cal_mag == 3) {
	        *fully_calibrated = true;//Alternative is if *total==12
	    } else {
	        *fully_calibrated = false;
	    }
	    return total;
}//Determine calibration status


