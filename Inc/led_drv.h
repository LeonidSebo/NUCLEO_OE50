/*
 * adc.h
 *
 *  Created on: May 2, 2020
 *      Author: Leonid Sorkin
 */



#ifndef FLASH_H
#define FLASH_H
#include "stm32l4xx_hal.h"

#define LED_DRV_LEDLO_REG		0x00
#define LED_DRV_FAULT_REG		0x07
#define LED_R_ON				(1<<0)
#define LED_G_ON				(1<<1)
#define LED_B_ON				(1<<2)


typedef struct{
	uint8_t i2c_status;		// 0 - i2c ok, 1 - fault
	uint8_t fault_reg;		// request read led driver FAULT register
}led_drv_state_t;


void setBrightnesas(void);
void LedDrv_GetFaultReg(void);


#endif  //FLASH_H
