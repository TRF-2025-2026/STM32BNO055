#include "bno055.h"
#include "accel.h"
#include "i2c.h"
#include <stdbool.h>

uint8_t OPRMode = NDOF;

uint8_t Ascale= AFS_16G;      // Accel full scale
uint8_t APwrMode= NormalA;   		// Accel power mode
uint8_t Abw= ABW_125Hz;
uint8_t Gscale= GFS_2000DPS;      // Accel full scale
uint8_t GPwrMode= NormalG;   		// Accel power mode
uint8_t Gbw= GBW_230Hz;
uint8_t MPwrMode=Normal;
uint8_t MOpMode=EnhancedRegular;
uint8_t Modr= MODR_30Hz;
uint8_t PWRMode=Normal;
uint8_t OpMode= NDOF;

uint8_t status;
float aRes,mRes,gRes;
uint8_t cal_sys=0;
uint8_t cal_gyro=0;
uint8_t cal_imu=0;
uint8_t cal_acc=0;
uint8_t cal_mag=0;
const uint8_t num_of_bytes_read = 38;
const char read_devid[]= {START_BYTE, REG_READ, BNO055_CHIP_ID, 0x01};
const char read_calib[2]= {REG_READ, BNO055_CALIB_STAT};
const char reset_sensor[3]= {REG_READ, BNO055_SYS_TRIGGER,0x01};
uint8_t get_readings[1] = {BNO055_ACC_DATA_X_LSB};

void BNO055_InitI2C(I2C_HandleTypeDef *hi2c_dev){
	uint8_t opr_conf_mode[2]={BNO055_OPR_MODE,CONFIGMODE};
	HAL_I2C_Master_Transmit(hi2c_dev, BNO055_I2C_ADDR_LO<<1, opr_conf_mode, sizeof(opr_conf_mode), 10);
	HAL_Delay(10);
	uint8_t conf_page1[2]={BNO055_PAGE_ID,0x01};
	HAL_I2C_Master_Transmit(hi2c_dev, BNO055_I2C_ADDR_LO<<1, conf_page1, sizeof(conf_page1), 10);
	HAL_Delay(10);
	uint8_t conf_acc[2]={BNO055_ACC_CONFIG,APwrMode << 5 |Abw<< 2| Ascale};
	HAL_I2C_Master_Transmit(hi2c_dev, BNO055_I2C_ADDR_LO<<1, conf_acc, sizeof(conf_acc), 10);
	HAL_Delay(10);
//	uint8_t conf_mag[2]={BNO055_MAG_CONFIG,MPwrMode << 5 |Mbw<< 2| Mscale};
//	HAL_I2C_Master_Transmit(hi2c_dev, BNO055_I2C_ADDR_LO<<1, conf_mag, sizeof(conf_mag), 10);
//	HAL_Delay(10);
	uint8_t conf_gyro[2] = {BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale};
	HAL_I2C_Master_Transmit(hi2c_dev, BNO055_I2C_ADDR_LO<<1, conf_gyro, sizeof(conf_gyro), 10);
	HAL_Delay(10);
	uint8_t conf_gyro_pwr[2]={BNO055_GYRO_CONFIG_1,GPwrMode << 5 |Abw<< 2|Gscale};
	HAL_I2C_Master_Transmit(hi2c_dev, BNO055_I2C_ADDR_LO<<1, conf_gyro, sizeof(conf_gyro_pwr), 10);
	HAL_Delay(10);
	uint8_t conf_mag_pwr[4] = {REG_WRITE, BNO055_MAG_CONFIG, 0x01, MPwrMode << 5 | MOpMode << 3 | Modr};
	HAL_I2C_Master_Transmit(hi2c_dev, BNO055_I2C_ADDR_LO<<1, conf_mag_pwr, sizeof(conf_mag_pwr), 10);
	HAL_Delay(10);
	uint8_t conf_page0[2]={BNO055_PAGE_ID,0x00};
	HAL_I2C_Master_Transmit(hi2c_dev, BNO055_I2C_ADDR_LO<<1, conf_page0, sizeof(conf_page0), 10);
	HAL_Delay(10);
	uint8_t pwr_pwrmode[2] = {BNO055_PWR_MODE, PWRMode};
	HAL_I2C_Master_Transmit(hi2c_dev, BNO055_I2C_ADDR_LO<<1, pwr_pwrmode, sizeof(pwr_pwrmode), 10);
	HAL_Delay(10);
	uint8_t opr_oprmode[2] = {BNO055_OPR_MODE, OPRMode};
	HAL_I2C_Master_Transmit(hi2c_dev, BNO055_I2C_ADDR_LO<<1, opr_oprmode, sizeof(opr_oprmode), 10);
	HAL_Delay(50);

}

uint8_t GetAccelData(I2C_HandleTypeDef* hi2c_dev, uint8_t* buf) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(hi2c_dev,BNO055_I2C_ADDR_LO << 1,BNO055_ACC_DATA_X_LSB,I2C_MEMADD_SIZE_8BIT,buf,6,100);
    return (uint8_t)status;
}

uint8_t GetAccelChipId(I2C_HandleTypeDef* hi2c_dev, uint8_t *chip_id) {
	return HAL_I2C_Mem_Read(hi2c_dev, BNO055_I2C_ADDR_LO<<1, BNO055_CHIP_ID, I2C_MEMADD_SIZE_8BIT, chip_id, 1, 100);
}


uint8_t GetAccelTemp(I2C_HandleTypeDef* hi2c_dev){
	uint8_t temp;
	HAL_I2C_Mem_Read_DMA(hi2c_dev, BNO055_I2C_ADDR_LO<<1, BNO055_TEMP,I2C_MEMADD_SIZE_8BIT, &temp,1);
	return temp;
	}

uint8_t GetCalibration(I2C_HandleTypeDef* hi2c_dev){
	uint8_t cal;
	HAL_I2C_Mem_Read_DMA(hi2c_dev, BNO055_I2C_ADDR_LO<<1, BNO055_CALIB_STAT,I2C_MEMADD_SIZE_8BIT, &cal,1);
	return cal;
}

uint8_t BNO055_Calib_Calc(uint8_t calibration,uint8_t *cal_system,uint8_t *cal_gyro,uint8_t *cal_acc,uint8_t *cal_mag,bool *fully_calibrated){
	*cal_system = (calibration >> 6) & 0x03;
	*cal_gyro   = (calibration >> 4) & 0x03;
	*cal_acc    = (calibration >> 2) & 0x03;
	*cal_mag    = (calibration >> 0) & 0x03;
	uint8_t total = *cal_system + *cal_gyro + *cal_acc + *cal_mag;
	if (*cal_system == 3 && *cal_gyro == 3 && *cal_acc == 3 && *cal_mag == 3) {
	        *fully_calibrated = true;
	    } else {
	        *fully_calibrated = false;
	    }
	    return total;
}


