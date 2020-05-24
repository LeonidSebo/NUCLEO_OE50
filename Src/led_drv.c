/*
 * led_drv.c
 *
 *  Created on: May 5, 2020
 *      Author: Leonid Sorkin
 */

#include "main.h"
#include "led_drv.h"
#include "flash.h"
#include "i2c.h"


extern config_t* pConfigStruct;
extern uint8_t sw_error;
extern I2C_HandleTypeDef hi2c1;
extern power_state_t CurrentPowerMode;
extern irq_flg_t  irq_flg;
extern uint8_t FlashCopyBrightnessTab0[FLASH_BRIGHTNESS_TAB_SIZE];
extern uint8_t FlashCopyBrightnessTab1[FLASH_BRIGHTNESS_TAB_SIZE];

led_drv_state_t led_drv_state = {0};
uint8_t led_RGB = LED_R_ON|LED_G_ON|LED_B_ON;


void setBrightnesas(void){
	uint8_t ld_buff[4];
	uint16_t color;
	uint32_t color_addr;
	uint8_t* BrightnessBaseAddr;
	sw_error = HAL_OK;
	if(CurrentPowerMode == PWR_ON){ //use boot brightness table
		if(pConfigStruct->bootBrightnesTabNo==1){
			BrightnessBaseAddr = FlashCopyBrightnessTab1;
			HAL_GPIO_WritePin(LED_DAY_GPIO_Port, LED_DAY_Pin, GPIO_PIN_SET);
		}else if(pConfigStruct->bootBrightnesTabNo==0){
			BrightnessBaseAddr = FlashCopyBrightnessTab0;
			HAL_GPIO_WritePin(LED_DAY_GPIO_Port, LED_DAY_Pin, GPIO_PIN_RESET);
		}else{
			sw_error = PARAM_ERROR;
			return;
		}
	}else if(CurrentPowerMode == PWR_OPERATION){ //use operation brightness table
		if(pConfigStruct->brightnessTabNo==1){
			BrightnessBaseAddr = FlashCopyBrightnessTab1;
			HAL_GPIO_WritePin(LED_DAY_GPIO_Port, LED_DAY_Pin, GPIO_PIN_SET);

		}else if(pConfigStruct->brightnessTabNo==0){
			BrightnessBaseAddr = FlashCopyBrightnessTab0;
			HAL_GPIO_WritePin(LED_DAY_GPIO_Port, LED_DAY_Pin, GPIO_PIN_RESET);
		}else{
			sw_error = PARAM_ERROR;
			return;
		}

	}else{
		HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_RESET);
		return;
	}
	color_addr = pConfigStruct->brightness*sizeof(brightnees_t);
	color = (led_RGB & LED_R_ON)? *(uint32_t*)(BrightnessBaseAddr+color_addr) : 0;  //red
	ld_buff[0] = (uint8_t)((color&0x03)<<4);	// low 2 bits to LED_DRIVER_LEDLO_REG
	ld_buff[3] = (uint8_t)((color>>2)&0xff);	// hight 8 bits to LED_DRIVER_RLEDH_REG
	color = (led_RGB & LED_G_ON)? *(uint32_t*)(BrightnessBaseAddr+color_addr+2) : 0;  //green
	ld_buff[0] = (uint8_t)((color&0x03)<<0);	// low 2 bits to LED_DRIVER_LEDLO_REG
	ld_buff[1] = (uint8_t)((color>>2)&0xff);	// hight 8 bits to LED_DRIVER_RLEDH_REG
	color = (led_RGB & LED_B_ON)?  *(uint32_t*)(BrightnessBaseAddr+color_addr+4) : 0;  //blue
	ld_buff[0] = (uint8_t)((color&0x03)<<2);	// low 2 bits to LED_DRIVER_LEDLO_REG
	ld_buff[2] = (uint8_t)((color>>2)&0xff);	// hight 8 bits to LED_DRIVER_RLEDH_REG

	sw_error = HAL_I2C_Mem_Write(&hi2c1, LED_DRIVER_I2C_ADDR<<1, LED_DRV_LEDLO_REG, sizeof(uint8_t), ld_buff, sizeof(ld_buff), I2C_TX_TIMEOUT);
	if(sw_error == HAL_OK){
		led_drv_state.i2c_status = 0;
	} else{
		led_drv_state.i2c_status = 1;
		sw_error = HAL_ERROR;
	}
	led_drv_state.fault_reg = 0;
	if(ld_buff[0]||ld_buff[1]||ld_buff[2]||ld_buff[3]){
		HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_RESET);
	}
}

void LedDrv_GetFaultReg(void){
	sw_error = HAL_I2C_Mem_Read(&hi2c1, LED_DRIVER_I2C_ADDR<<1, LED_DRV_FAULT_REG, sizeof(uint8_t), &(led_drv_state.fault_reg), sizeof(uint8_t),I2C_RX_TIMEOUT);
	if(sw_error == HAL_OK){
		led_drv_state.i2c_status = 0;
	} else{
		led_drv_state.i2c_status = 1;
//		irq_flg.irq_LED_DRIVER = 1;
	}
}

void LedDrv_Off(void){
	HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_RESET);
}
