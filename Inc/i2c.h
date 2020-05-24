/*
 * i2c.h
 *
 *  Created on: May 5, 2020
 *      Author: Leonid Sorkin
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#define IMU_I2C_ADDR			0x68
#define ASIC_I2C_ADDR			0x3C
#define LCOS_I2C_ADDR			0x49
#define VGA_TO_RGB_I2C_ADDR		0x5C
#define LED_DRIVER_I2C_ADDR		0x28
#define MCU_I2C_ADDR			0x78



#define I2C_TX_TIMEOUT			1000
#define I2C_RX_TIMEOUT			1000

#endif /* INC_I2C_H_ */
