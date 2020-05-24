/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */
#define  NUCLEO_M412KB

#define TIME_BETWEN_MESSAGES		49  // Time between two messages > 50msec
/* USER CODE END Private defines */
#ifndef NUCLEO_M412KB
void MX_USART1_UART_Init(void);
#endif
/* USER CODE BEGIN Prototypes */
#define UART_REQ_OPCODE_OFFSET		3
#define UART_REQ_SEQ_NUM_OFFSET		5
#define UART_REQ_MSG_LEN_OFFSET		7
#define UART_REQ_DATA_OFFSET		9
#define UART_HEADER_LEN				UART_REQ_DATA_OFFSET
#define UART_CRC_LEN				2
#define UART_OPCODE_LEN				2
#define UART_EMPTY_RESP_LEN			UART_REQ_DATA_OFFSET+UART_CRC_LEN /*+UART_OPCODE_LEN*/
#define UART_FLASH_WR_HEADER_SIZE	5

#define UART_REQ_SYNC0				'E'
#define UART_REQ_SYNC1				'U'
#define UART_REQ_SYNC2				'M'

#define UART_RESP_SYNC0				'H'
#define UART_RESP_SYNC1				'U'
#define UART_RESP_SYNC2				'M'

#define UART_RESPONSE						0
#define UART_ACK_CMD						1
#define UART_NACK_CMD						2
#define UART_ERROR_CMD						3

#define UART_ACK_OPCODE						0x9999
#define UART_NACK_OPCODE					0x8888
#define UART_ERROR_OPCODE					0x7777

#define CRC_INIT_VAL				0x00

#define UART_REQ_MAX_DATA_SIZE		1024+UART_FLASH_WR_HEADER_SIZE
#define UART_REQ_MAX_SIZE			UART_REQ_MAX_DATA_SIZE+UART_REQ_DATA_OFFSET+UART_CRC_LEN
#define UART_RESP_MAX_SIZE			UART_REQ_MAX_SIZE

//OPCODE definitions
#define GET_VER_CMD					0x0001
#define GET_BIT_CMD					0x0002
#define SET_TEMP_MIN_MAX			0x0003
#define GET_INTR_DATA				0x0004
#define SEND_IMU_CMD				0x0005
#define SET_BRIGHTNESS				0x0006
#define GET_CURRENT_BRIGHTNESS		0x0007
#define SET_RGB						0x0008
#define GET_CURRENT_RGB				0x0009
#define SWITCH_BRIGHTNESS_TABLE		0x000A
#define FLIP_DISPLAY_ON_X			0x000B
#define FLIP_DISPLAY_ON_Y			0x000C
#define MOVE_OFFSET_X				0x000D
#define MOVE_OFFSET_Y				0x000E
#define POWER_CMD					0x000F
#define GET_A2D						0x0010
#define GET_TEMP_CMD				0x0011
#define SAVE_REGION_2_FLASH			0x0012
#define LOAD_REGION_2_RAM			0x0013
#define GET_DATA_FROM_REGION		0x0014
#define SET_REGION					0x0015
#define I2C_WRITE					0x0016
#define I2C_READ					0x0017
#define RUN_SCRIPT					0x0018

#define SAVE_IN_STORAGE				0x0040
#define GET_FROM_STORAGE			0x0041
#define ERASE_STORAGE_PAGE			0x0042


typedef struct{
	unsigned uart_req_ready_flg		:1;
//	unsigned uart_req_crc_err		:1;
}uart_req_flags_t;
typedef struct{
	uint8_t request[UART_REQ_MAX_SIZE];
	uint8_t response[UART_RESP_MAX_SIZE];
	uint16_t length;
	uart_req_flags_t flag;
}uart_msg_t;

typedef enum {
	UART_SYNC0_ST,
	UART_SYNC1_ST,
	UART_SYNC2_ST,
	UART_REQ_HEADER_ST,
	UART_REQ_DATA_ST
//	UART_REQ_CRC_ST
}uart_req_state_t;

typedef enum {
	UART_NACK_UNKNOWN,
	UART_NACK_BAD_CRC,
	UART_NACK_BAD_LENGTH,
	UART_NACK_BAD_OPCODE,
	UART_NACK_BAD_SYNC,
}uart_nack_reson_t;

typedef enum{
	UART_ANSW_NO_ERR,
	UART_ANSW_FLASH_ERASE_ERR,
	UART_ANSW_FLASH_PROGRAM_ERR,
	UART_ANSW_FLASH_DATA_OFFSET_ERR,
	UART_ANSW_PARAM_ERROR,
	UART_ANSW_BAD_I2C_BUSS_ID_ERR,
	UART_ANSW_I2C_ERROR,
	UART_ANS_BAD_CMD_PARAM_ERR,
	UART_ANS_UNKNOWN_ERR
}uart_answ_err_t;


void UART_Rx_Interrupt(UART_HandleTypeDef *huart);
void UART_Rx_Byte(void);

HAL_StatusTypeDef UART_Tx(uint8_t* pData, uint16_t DataLen, uint8_t RespType);
HAL_StatusTypeDef uartReqHandler(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
