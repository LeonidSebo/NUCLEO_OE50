/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "flash.h"
#include "adc.h"
#include "led_drv.h"
#include "i2c.h"
#include "lcos.h"
#include "vga2rgb.h"
#include "imu.h"
#include "usart.h"
#include "stdio.h"		//printf function

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define _LPM_EN

#define CHECK_ERROR				  if(sw_error!=HAL_OK) Error_Handler()

typedef struct{
	unsigned ADC_dataReady			:1;
	unsigned led_drv_fault			:1;
	unsigned pwrModeChange_req		:1;
	unsigned spare					:26;
}main_status_t;

typedef enum {
	FLASH_NOT_INITIALISED = HAL_TIMEOUT+1,
	FLASH_CMD_ERROR,
	PARAM_ERROR
}sw_error_t;

typedef struct{
	uint8_t us_countEn;
	uint16_t ms_counter;
}timer_ctrl_t;
typedef struct{
	uint16_t version_msb;
	uint16_t version_lsb;
	uint16_t  release;
	uint16_t  build;
}version_t;

typedef enum{
	DEV_INITIALISED,
	DEV_INITIALISATIN_ERROR
}ext_dev_status_t;

typedef struct{
	unsigned ASIC_init_status		: 1;	//HX7816_init_error
	unsigned LCOS_init_status		: 1;	//HX7318_init_error
	unsigned VGA2RGB_init_status	: 1;	//TVP70025_init_error
	unsigned IMU_init_status		: 1;	//ICM_20600_init_error
	unsigned LED_DRIVER_init_status	: 1;	//LM3435_init_error
	unsigned DeviceID_error			: 1;    //DeviceID_error - flash is not initialized
	unsigned ADC_Error				: 1;
	unsigned FlashConfigTabError	: 1;
	unsigned Voltage_err			: 1;
	unsigned Temp_err				: 1;
	unsigned unused					: 22;
}BIT_status_t;

#define BIT_STATUS_MASK		0x000003FF

typedef enum{
	INT_NONE,
	INT_IMU,
	INT_TEMP,
	INT_POWER,
	INT_LCOS,
	INT_ASIC,
	INT_VGA2RGB,
	INT_LED_DRIVER,
	INT_INIT_FINISH,
	INT_IMU_ERR
}interrupt_type_t;

typedef struct{
	unsigned irq_IMU 		: 1;
	unsigned irq_TEMP 		: 1;
	unsigned irq_POWER 		: 1;
	unsigned irq_LCOS 		: 1;
	unsigned irq_ASIC 		: 1;
	unsigned irq_VGA2RGB 	: 1;
	unsigned irq_LED_DRIVER : 1;
	unsigned irq_InitFinish	: 1;
	unsigned irq_IMU_err	: 1;
	unsigned irq_unused		:23;
}irq_flg_t;


typedef enum{
	PWR_OFF,
	PWR_ON,
	PWR_OPERATION,
	PWR_IMU,
	PWR_MODE_MAX = PWR_IMU
}power_state_t;

typedef struct{
	uint8_t i2c_error	:1;		// 0 - i2c ok, 1 - faul
	uint8_t init_error	:1;
}device_state_t;


#define LED_DRIVER_LEDLO_REG	0
#define LED_DRIVER_GLEDH_REG	1
#define LED_DRIVER_BLEDH_REG	2
#define LED_DRIVER_RLEDH_REG	3
#define LED_DRIVER_FLT_RPT_REG	5
#define LED_DRIVER_DELAY_REG	6
#define LED_DRIVER_FAULT_REG	7

#define SCRIPT_RECORD_SIZE		3
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void runScript(uint8_t* BaseAddr, uint16_t recordNum);
void ReinitAnalogWDT(void);
void ADC_Calc(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MON_6V_Pin GPIO_PIN_1
#define MON_6V_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define IMU_IRQ1_Pin GPIO_PIN_3
#define IMU_IRQ1_GPIO_Port GPIOA
#define IMU_IRQ1_EXTI_IRQn EXTI3_IRQn
#define LED_DAY_Pin GPIO_PIN_4
#define LED_DAY_GPIO_Port GPIOA
#define LED_FAULT_Pin GPIO_PIN_5
#define LED_FAULT_GPIO_Port GPIOA
#define LCOS_nRST_Pin GPIO_PIN_6
#define LCOS_nRST_GPIO_Port GPIOA
#define I2COUT_EN_Pin GPIO_PIN_0
#define I2COUT_EN_GPIO_Port GPIOB
#define mon_1V8_Pin GPIO_PIN_1
#define mon_1V8_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_8
#define INT_GPIO_Port GPIOA
#define IMU_IRQ2_Pin GPIO_PIN_12
#define IMU_IRQ2_GPIO_Port GPIOA
#define IMU_IRQ2_EXTI_IRQn EXTI15_10_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LED_EN_Pin GPIO_PIN_3
#define LED_EN_GPIO_Port GPIOB
#define VGA_nRST_Pin GPIO_PIN_5
#define VGA_nRST_GPIO_Port GPIOB
#define HX7816_nRST_Pin GPIO_PIN_3
#define HX7816_nRST_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
