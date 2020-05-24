/*
 * imu.h
 *
 *  Created on: May 10, 2020
 *      Author: Leonid Sorkin
 */
#ifndef IMU_H_
#define IMU_H_

#define ICM_42605

#define IMU_CMD_READ  		0
#define IMU_CMD_WRITE  		1

#define IMU_REG_BANK_SEL		0x76			/*all banks*/
#define IMU_REG_WHO_AM_I		0x75			/*BANK 0*/
#define IMU_REG_PWR_MGMT0		0x4E			/*BANK 0*/
#define IMU_REG_TEMP_DATA1		0x1D			/*BANK 0*/
#define IMU_REG_ACCEL_DATA_X1	0x1F			/*BANK 0*/
#define IMU_REG_GYRO_DATA_X1	0x25			/*BANK 0*/

#define IMU_POWER_OFF			0
#define IMU_ID					0x42

#define IMU_DATA_START_ADDR		IMU_REG_TEMP_DATA1
#define IMU_TEMP_DATA_OFFSET 	IMU_REG_TEMP_DATA1 - IMU_DATA_START_ADDR
#define IMU_ACCEL_DATA_OFFSET 	IMU_REG_ACCEL_DATA_X1 - IMU_DATA_START_ADDR
#define IMU_GYRO_DATA_OFFSET	IMU_REG_GYRO_DATA_X1 - IMU_DATA_START_ADDR



//#define IMU_ACCEL_XOUT_REG		0x3B
#define IMU_TEMP_DATA_LEN		2
#define IMU_ACCSEL_DATA_LEN		6
#define IMU_GIRO_DATA_LEN		6

#define IMU_DATA_LEN		IMU_TEMP_DATA_LEN + IMU_ACCSEL_DATA_LEN + IMU_GIRO_DATA_LEN

/*
 * Temperature data value from the sensor data registers can be converted to degrees centigrade by using the following formula:
 * Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
 */

typedef struct{
	uint16_t Temp;
	uint16_t Gyro_X;
	uint16_t Gyro_Y;
	uint16_t Gyro_Z;
	uint16_t Accelerator_X;
	uint16_t Accelerator_Y;
	uint16_t Accelerator_Z;
}imu_data_t;

void IMU_Read_Measurements(uint8_t* pData);
void IMU_PowerOff(void);
uint8_t IMU_Read_ID(void);
#endif /* IMU_H_ */
