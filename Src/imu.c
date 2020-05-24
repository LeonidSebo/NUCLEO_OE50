/*
 * imu.c
 *
 *  Created on: May 19, 2020
 *      Author:Leonid Sorkin
 */
#include  "main.h"

extern uint8_t sw_error;
extern I2C_HandleTypeDef hi2c3;

uint8_t imu_data[IMU_DATA_LEN];
device_state_t	imu_state = {0};

#ifdef ICM_42605

void IMU_Read_Measurements(uint8_t* pData){

	uint8_t Bank = 0;

	sw_error = HAL_I2C_Mem_Write(&hi2c3,IMU_I2C_ADDR<<1,IMU_REG_BANK_SEL,1,&Bank,1,I2C_TX_TIMEOUT);  		// select bank 0
	if(sw_error == HAL_OK){
		sw_error = HAL_I2C_Mem_Read(&hi2c3,IMU_I2C_ADDR<<1,IMU_DATA_START_ADDR,1,pData,IMU_DATA_LEN,I2C_TX_TIMEOUT);
	}
}

void IMU_PowerOff(void){
	uint8_t Bank = 0;
	uint8_t Data = IMU_POWER_OFF;

	sw_error = HAL_I2C_Mem_Write(&hi2c3,IMU_I2C_ADDR<<1,IMU_REG_BANK_SEL,1,&Bank,1,I2C_TX_TIMEOUT);  		// select bank 0
	if(sw_error == HAL_OK){
		sw_error = HAL_I2C_Mem_Read(&hi2c3,IMU_I2C_ADDR<<1,IMU_REG_PWR_MGMT0,1,&Data,IMU_DATA_LEN,I2C_TX_TIMEOUT);
	}

}

uint8_t IMU_Read_ID(void){

	uint8_t Bank = 0;
	uint8_t id;

	sw_error = HAL_I2C_Mem_Write(&hi2c3,IMU_I2C_ADDR<<1,IMU_REG_BANK_SEL,1,&Bank,1,I2C_TX_TIMEOUT);  		// select bank 0
	if(sw_error == HAL_OK){
		sw_error = HAL_I2C_Mem_Read(&hi2c3,IMU_I2C_ADDR<<1,IMU_DATA_START_ADDR,1,&id,1,I2C_TX_TIMEOUT);
		if(sw_error == HAL_OK){
			return id;
		}
	}
	return 0xFF;
}

#endif //ICM_42605
