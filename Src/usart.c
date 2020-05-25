/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "main.h"
/* USER CODE BEGIN 0 */
#include "led_drv.h"
extern config_t* pConfigStruct;
extern const version_t sw_revision;
extern BIT_status_t boot_BIT_status;    // boot error status
extern irq_flg_t  irq_flg;
extern main_status_t main_status;
extern uint8_t sw_error;
extern led_drv_state_t led_drv_state;
extern uint8_t led_RGB;
extern power_state_t NextPowerMode;
extern uint16_t adc_raw_data[4];
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern uint16_t VDDA_mv;
extern device_state_t lcos_state;
extern device_state_t asic_state;
extern device_state_t vga2rgb_state;
extern device_state_t	imu_state;
extern power_state_t CurrentPowerMode;
extern uint8_t FlashCopyDevInfo[FLASH_DEVICE_INFO_SIZE];
extern uint8_t FlashCopyConfigStruct[FLASH_CONFIG_SIZE];
extern uint8_t FlashCopyBrightnessTab0[FLASH_BRIGHTNESS_TAB_SIZE];
extern uint8_t FlashCopyBrightnessTab1[FLASH_BRIGHTNESS_TAB_SIZE];
extern uint8_t FlashCopyInitData[FLASH_INIT_DATA_SIZE];
extern uint8_t imu_data[IMU_DATA_LEN];
extern UART_HandleTypeDef huart1;
extern uint16_t mon_6V_mv;
extern uint16_t mon_1V8_mv;
extern uint32_t TemperatureHighThreshold;
extern uint32_t TemperatureLowThreshold;

uint8_t RdByteReady = 0;
uint8_t RxBuf;
uint8_t PageBuf[FLASH_PAGE_SIZE];
uint16_t Req_CRC16 = 0;
uint16_t RxCRC16 = 0;
uint32_t* puint32;

int16_t debug_lastCntr;   //for debug
static FLASH_EraseInitTypeDef EraseInitStruct;
static const uint8_t CRC_table_1[] =
{
  0x00,0x00,0x10,0x21,0x20,0x42,0x30,0x63,0x40,0x84,0x50,0xA5,0x60,0xC6,0x70,0xE7,
  0x81,0x08,0x91,0x29,0xA1,0x4A,0xB1,0x6B,0xC1,0x8C,0xD1,0xAD,0xE1,0xCE,0xF1,0xEF,
  0x12,0x31,0x02,0x10,0x32,0x73,0x22,0x52,0x52,0xB5,0x42,0x94,0x72,0xF7,0x62,0xD6,
  0x93,0x39,0x83,0x18,0xB3,0x7B,0xA3,0x5A,0xD3,0xBD,0xC3,0x9C,0xF3,0xFF,0xE3,0xDE,
  0x24,0x62,0x34,0x43,0x04,0x20,0x14,0x01,0x64,0xE6,0x74,0xC7,0x44,0xA4,0x54,0x85,
  0xA5,0x6A,0xB5,0x4B,0x85,0x28,0x95,0x09,0xE5,0xEE,0xF5,0xCF,0xC5,0xAC,0xD5,0x8D,
  0x36,0x53,0x26,0x72,0x16,0x11,0x06,0x30,0x76,0xD7,0x66,0xF6,0x56,0x95,0x46,0xB4,
  0xB7,0x5B,0xA7,0x7A,0x97,0x19,0x87,0x38,0xF7,0xDF,0xE7,0xFE,0xD7,0x9D,0xC7,0xBC,
  0x48,0xC4,0x58,0xE5,0x68,0x86,0x78,0xA7,0x08,0x40,0x18,0x61,0x28,0x02,0x38,0x23,
  0xC9,0xCC,0xD9,0xED,0xE9,0x8E,0xF9,0xAF,0x89,0x48,0x99,0x69,0xA9,0x0A,0xB9,0x2B,
  0x5A,0xF5,0x4A,0xD4,0x7A,0xB7,0x6A,0x96,0x1A,0x71,0x0A,0x50,0x3A,0x33,0x2A,0x12,
  0xDB,0xFD,0xCB,0xDC,0xFB,0xBF,0xEB,0x9E,0x9B,0x79,0x8B,0x58,0xBB,0x3B,0xAB,0x1A,
  0x6C,0xA6,0x7C,0x87,0x4C,0xE4,0x5C,0xC5,0x2C,0x22,0x3C,0x03,0x0C,0x60,0x1C,0x41,
  0xED,0xAE,0xFD,0x8F,0xCD,0xEC,0xDD,0xCD,0xAD,0x2A,0xBD,0x0B,0x8D,0x68,0x9D,0x49,
  0x7E,0x97,0x6E,0xB6,0x5E,0xD5,0x4E,0xF4,0x3E,0x13,0x2E,0x32,0x1E,0x51,0x0E,0x70,
  0xFF,0x9F,0xEF,0xBE,0xDF,0xDD,0xCF,0xFC,0xBF,0x1B,0xAF,0x3A,0x9F,0x59,0x8F,0x78
};

static const uint8_t CRC_table_2[] =
{
  0x91,0x88,0x81,0xA9,0xB1,0xCA,0xA1,0xEB,0xD1,0x0C,0xC1,0x2D,0xF1,0x4E,0xE1,0x6F,
  0x10,0x80,0x00,0xA1,0x30,0xC2,0x20,0xE3,0x50,0x04,0x40,0x25,0x70,0x46,0x60,0x67,
  0x83,0xB9,0x93,0x98,0xA3,0xFB,0xB3,0xDA,0xC3,0x3D,0xD3,0x1C,0xE3,0x7F,0xF3,0x5E,
  0x02,0xB1,0x12,0x90,0x22,0xF3,0x32,0xD2,0x42,0x35,0x52,0x14,0x62,0x77,0x72,0x56,
  0xB5,0xEA,0xA5,0xCB,0x95,0xA8,0x85,0x89,0xF5,0x6E,0xE5,0x4F,0xD5,0x2C,0xC5,0x0D,
  0x34,0xE2,0x24,0xC3,0x14,0xA0,0x04,0x81,0x74,0x66,0x64,0x47,0x54,0x24,0x44,0x05,
  0xA7,0xDB,0xB7,0xFA,0x87,0x99,0x97,0xB8,0xE7,0x5F,0xF7,0x7E,0xC7,0x1D,0xD7,0x3C,
  0x26,0xD3,0x36,0xF2,0x06,0x91,0x16,0xB0,0x66,0x57,0x76,0x76,0x46,0x15,0x56,0x34,
  0xD9,0x4C,0xC9,0x6D,0xF9,0x0E,0xE9,0x2F,0x99,0xC8,0x89,0xE9,0xB9,0x8A,0xA9,0xAB,
  0x58,0x44,0x48,0x65,0x78,0x06,0x68,0x27,0x18,0xC0,0x08,0xE1,0x38,0x82,0x28,0xA3,
  0xCB,0x7D,0xDB,0x5C,0xEB,0x3F,0xFB,0x1E,0x8B,0xF9,0x9B,0xD8,0xAB,0xBB,0xBB,0x9A,
  0x4A,0x75,0x5A,0x54,0x6A,0x37,0x7A,0x16,0x0A,0xF1,0x1A,0xD0,0x2A,0xB3,0x3A,0x92,
  0xFD,0x2E,0xED,0x0F,0xDD,0x6C,0xCD,0x4D,0xBD,0xAA,0xAD,0x8B,0x9D,0xE8,0x8D,0xC9,
  0x7C,0x26,0x6C,0x07,0x5C,0x64,0x4C,0x45,0x3C,0xA2,0x2C,0x83,0x1C,0xE0,0x0C,0xC1,
  0xEF,0x1F,0xFF,0x3E,0xCF,0x5D,0xDF,0x7C,0xAF,0x9B,0xBF,0xBA,0x8F,0xD9,0x9F,0xF8,
  0x6E,0x17,0x7E,0x36,0x4E,0x55,0x5E,0x74,0x2E,0x93,0x3E,0xB2,0x0E,0xD1,0x1E,0xF0
};

uart_msg_t uart_msg;

#ifndef NUCLEO_M412KB

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */
    __HAL_UART_ENABLE_IT(uartHandle, UART_IT_RXNE);
  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#else
extern UART_HandleTypeDef huart1;
#endif
static uint32_t GetPage(uint32_t Addr);
static void get_flash_region(uint8_t RegId,uint32_t* pRegAddr, uint8_t* pRespType);
static void erase_page(uint8_t RegID,uint8_t NbPages,uint8_t* pRespType);
static uint16_t GetIntrData(void);
static void imu_cmd(uint8_t* pCmd,uint16_t* DataLen,uint8_t* RespType);
static uint8_t* getRamRegion(uint8_t regID, uint16_t* Len);
static void run_BIT(BIT_status_t* pBIT_status);

 static uint16_t CRC16_CRCCalc(uint8_t *buf, uint16_t dataLen, uint8_t headerLen, uint16_t crcInitVal)
{
  uint8_t  constant_index, CRC_Hi, CRC_Lo;
  uint16_t data_index;
  const uint8_t       *CRC_table;

  CRC_Lo = ( crcInitVal & 0x00FF);
  CRC_Hi = ( crcInitVal >> 8);

  for (data_index = (headerLen); data_index < dataLen; data_index ++)
  {
    CRC_table = (CRC_Lo & 0x80) ? CRC_table_2 : CRC_table_1;
    constant_index = CRC_Lo << 1;
    CRC_Lo = CRC_Hi ^ CRC_table[constant_index ++];
   CRC_Hi = buf[data_index] ^ CRC_table[constant_index];
  }

  return (CRC_Lo + (CRC_Hi << 8));
}

void UART_Rx_Interrupt(UART_HandleTypeDef *huart){
	uint32_t isrflags   = READ_REG(huart->Instance->ISR);
    if ((isrflags & USART_ISR_RXNE) != 0U){
    	RxBuf = huart->Instance->RDR;
    	RdByteReady = 1;
    }
}

void UART_Rx_Byte(void)
{
	static uint32_t CurrentTime = 0;  //Current interrupt time (mS)
	static uint32_t LastTime = 0;				//Last interrupt time (mS)
	//Current state of uart request state machine
	static uart_req_state_t uart_req_current_state = UART_REQ_HEADER_ST;
	static uint32_t uart_req_count = 0;
	// Data or CRC length counter
	static uint16_t cntr = 0;
	static uint16_t DataLen = 0;
	CurrentTime = HAL_GetTick();
	if((CurrentTime - LastTime)>TIME_BETWEN_MESSAGES){		//the time between interruptions is large enough to start a new transfer
		uart_req_current_state = UART_SYNC0_ST;
		uart_req_count = 0;
	}
	switch(uart_req_current_state){
	case UART_SYNC0_ST:
		if(RxBuf == UART_REQ_SYNC0){
			uart_msg.request[uart_req_count++]=RxBuf;
			uart_req_current_state = UART_SYNC1_ST;

		}else{
			PageBuf[0] = UART_NACK_BAD_SYNC;
			UART_Tx(PageBuf,1,UART_NACK_CMD);
			HAL_Delay(1000);
		}
		break;
	case UART_SYNC1_ST:
		if(RxBuf == UART_REQ_SYNC1){
			uart_msg.request[uart_req_count++]=RxBuf;
			uart_req_current_state = UART_SYNC2_ST;

		}else{
			uart_req_current_state = UART_SYNC0_ST;
			uart_req_count = 0;
			PageBuf[0] = UART_NACK_BAD_SYNC;
			UART_Tx(PageBuf,1,UART_NACK_CMD);
			HAL_Delay(1000);
		}
		break;
	case UART_SYNC2_ST:
		if(RxBuf == UART_REQ_SYNC2){
			uart_msg.request[uart_req_count++]=RxBuf;
			uart_req_current_state = UART_REQ_HEADER_ST;

		}else{
			uart_req_current_state = UART_SYNC0_ST;
			uart_req_count = 0;
			PageBuf[0] = UART_NACK_BAD_SYNC;
			UART_Tx(PageBuf,1,UART_NACK_CMD);
			HAL_Delay(1000);
		}
		break;
	case UART_REQ_HEADER_ST:
		if((uart_req_count>2)&&(uart_req_count<UART_REQ_MSG_LEN_OFFSET)){// sequence number
			uart_msg.request[uart_req_count++]=RxBuf;
		}else if(uart_req_count==UART_REQ_MSG_LEN_OFFSET){
			uart_msg.request[uart_req_count++]=RxBuf;
			DataLen = RxBuf;
			cntr = RxBuf;
		}else if(uart_req_count==(UART_REQ_MSG_LEN_OFFSET+1)){
			uart_msg.request[uart_req_count++]=RxBuf;
			DataLen |= RxBuf<<8;
			cntr |= RxBuf<<8;
			cntr -= UART_HEADER_LEN; //+UART_CRC_LEN);
			uart_req_current_state = UART_REQ_DATA_ST;
			if((cntr > UART_REQ_MAX_DATA_SIZE+UART_CRC_LEN)||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == GET_VER_CMD)				&&(cntr != UART_CRC_LEN))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == GET_BIT_CMD)				&&(cntr != UART_CRC_LEN+2))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == SET_TEMP_MIN_MAX)			&&(cntr != UART_CRC_LEN+8))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == GET_INTR_DATA)			&&(cntr != UART_CRC_LEN))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == SEND_IMU_CMD)				&&(cntr >UART_REQ_MAX_DATA_SIZE+UART_CRC_LEN))		||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == SET_BRIGHTNESS)			&&(cntr != UART_CRC_LEN+2))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == GET_CURRENT_BRIGHTNESS)	&&(cntr != UART_CRC_LEN))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == SET_RGB)					&&(cntr != UART_CRC_LEN+3))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == GET_CURRENT_RGB)			&&(cntr != UART_CRC_LEN))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == SWITCH_BRIGHTNESS_TABLE)	&&(cntr != UART_CRC_LEN))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == FLIP_DISPLAY_ON_X)		&&(cntr != UART_CRC_LEN))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == FLIP_DISPLAY_ON_Y)		&&(cntr != UART_CRC_LEN))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == MOVE_OFFSET_X)			&&(cntr != UART_CRC_LEN+1))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == MOVE_OFFSET_X)			&&(cntr != UART_CRC_LEN+1))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == POWER_CMD)				&&(cntr != UART_CRC_LEN+1))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == GET_A2D)					&&(cntr != UART_CRC_LEN))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == GET_TEMP_CMD)				&&(cntr != UART_CRC_LEN))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == SAVE_REGION_2_FLASH)		&&(cntr != UART_CRC_LEN+1))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == LOAD_REGION_2_RAM)		&&(cntr != UART_CRC_LEN+1))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == GET_DATA_FROM_REGION)		&&(cntr != UART_CRC_LEN+5))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == SET_REGION)				&&(cntr >UART_REQ_MAX_DATA_SIZE+UART_CRC_LEN))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == SAVE_IN_STORAGE)			&&(cntr >UART_REQ_MAX_DATA_SIZE+UART_CRC_LEN))		||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == GET_FROM_STORAGE)			&&(cntr != UART_CRC_LEN+5))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == ERASE_STORAGE_PAGE)		&&(cntr != UART_CRC_LEN+2))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == I2C_WRITE)				&&(cntr >UART_REQ_MAX_DATA_SIZE+UART_CRC_LEN))		||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == I2C_READ)					&&(cntr != UART_CRC_LEN+4))							||
				((uart_msg.request[UART_REQ_OPCODE_OFFSET] == RUN_SCRIPT)&&(cntr > UART_REQ_MAX_DATA_SIZE+UART_CRC_LEN))	)
			{
				uart_req_current_state = UART_SYNC0_ST;
				uart_req_count = 0;
				PageBuf[0] = UART_NACK_BAD_LENGTH;
				UART_Tx(PageBuf,1,UART_NACK_CMD);
				HAL_Delay(1000);
			}
		}else{//Sync error
			uart_req_count = 0;
		}
		break;
	case UART_REQ_DATA_ST:
		uart_msg.request[uart_req_count++]=RxBuf;
		if(--cntr == 0){
			Req_CRC16 = uart_msg.request[uart_req_count-2];
			Req_CRC16 |= uart_msg.request[uart_req_count-1]<<8;
			uart_req_current_state = UART_SYNC0_ST;
			uart_msg.length = uart_req_count;
			uart_req_count = 0;
			RxCRC16 = CRC16_CRCCalc(uart_msg.request, DataLen-UART_CRC_LEN-UART_REQ_OPCODE_OFFSET, UART_REQ_OPCODE_OFFSET, CRC_INIT_VAL);
			if(RxCRC16==Req_CRC16){
				uart_msg.flag.uart_req_ready_flg = 1;
			}else{
				PageBuf[0] = UART_NACK_BAD_CRC;
				UART_Tx(PageBuf,1,UART_NACK_CMD);
				HAL_Delay(1000);
			}
			break;
		}
	}
	debug_lastCntr = cntr;
 	LastTime = CurrentTime;
	__NOP();
}

HAL_StatusTypeDef UART_Tx(uint8_t* pData, uint16_t DataLen, uint8_t RespType){
	uint16_t crc;
	uint16_t responseLen;
	uart_msg.response[UART_REQ_SEQ_NUM_OFFSET] = uart_msg.request[UART_REQ_SEQ_NUM_OFFSET];
	uart_msg.response[UART_REQ_SEQ_NUM_OFFSET+1] = uart_msg.request[UART_REQ_SEQ_NUM_OFFSET+1];
	if(RespType == UART_RESPONSE){
		uart_msg.response[UART_REQ_OPCODE_OFFSET] = uart_msg.request[UART_REQ_OPCODE_OFFSET];
		uart_msg.response[UART_REQ_OPCODE_OFFSET+1] = 0xF0;
		if(DataLen){
			memcpy(uart_msg.response+UART_REQ_DATA_OFFSET,pData,DataLen);
		}

	}else{
		if (RespType == UART_ACK_CMD){
			uart_msg.response[UART_REQ_OPCODE_OFFSET] = (uint8_t)(UART_ACK_OPCODE&0xFF);
			uart_msg.response[UART_REQ_OPCODE_OFFSET+1] = (uint8_t)(UART_ACK_OPCODE>>8);
			DataLen = 0;
		}else if(RespType==UART_NACK_CMD){
			uart_msg.response[UART_REQ_OPCODE_OFFSET] = (uint8_t)(UART_NACK_OPCODE&0xFF);
			uart_msg.response[UART_REQ_OPCODE_OFFSET+1] = (uint8_t)(UART_NACK_OPCODE>>8);
			if(uart_msg.response[UART_REQ_DATA_OFFSET] == UART_NACK_BAD_SYNC){
				uart_msg.response[UART_REQ_SEQ_NUM_OFFSET] = 0;
				uart_msg.response[UART_REQ_SEQ_NUM_OFFSET+1] = 0;
			}
			//			DataLen = 0;	//must be 1
		}else if(RespType==UART_ERROR_CMD) {
			uart_msg.response[UART_REQ_OPCODE_OFFSET] = (uint8_t)(UART_ERROR_OPCODE&0xFF);
			uart_msg.response[UART_REQ_OPCODE_OFFSET+1] = (uint8_t)(UART_ERROR_OPCODE>>8);
//			DataLen = 0;	//must be 1
		}
		uart_msg.response[UART_REQ_DATA_OFFSET] = *pData;
	}
	responseLen = UART_HEADER_LEN+DataLen+UART_CRC_LEN;
	uart_msg.response[UART_REQ_MSG_LEN_OFFSET] = responseLen&0xff;
	uart_msg.response[UART_REQ_MSG_LEN_OFFSET+1] = responseLen>>8;
	crc = CRC16_CRCCalc(uart_msg.response, responseLen -UART_CRC_LEN-UART_REQ_OPCODE_OFFSET,UART_REQ_OPCODE_OFFSET, CRC_INIT_VAL);
	uart_msg.response[responseLen-2] = (uint8_t)crc&0xff;
	uart_msg.response[responseLen-1] = (uint8_t)(crc>>8);
	uart_msg.length = responseLen;
	return HAL_UART_Transmit_IT (&huart1, uart_msg.response, uart_msg.length);
//	 HAL_Delay(100);
}
/**
  * @brief  Handler of UART requests
  * @param  none
  * @retval Operation status.
  */

HAL_StatusTypeDef uartReqHandler(void){
	uint8_t RespType = UART_ACK_CMD;
	uint16_t DataLen = 1;
	uint8_t* pData = NULL;
	uint16_t offset;
	uint16_t len;
	uint8_t AddrCorrection;
	uint32_t i;
	uint32_t Address;
	uint16_t DataStartPos;
    uint64_t uint64_buf;
    float	float_buf;
	static uint16_t seq_num = 0;
	uint16_t Opcode = uart_msg.request[UART_REQ_OPCODE_OFFSET];
	I2C_HandleTypeDef* pi2c_bus;
	uint16_t voltage_3V3;

	Opcode |= uart_msg.request[UART_REQ_OPCODE_OFFSET+1];
	seq_num++;
	switch(Opcode){
	case GET_VER_CMD:
		RespType = UART_RESPONSE;
		DataLen = sizeof(version_t);
		pData = (uint8_t*)&sw_revision;
		break;
	case GET_BIT_CMD:
		RespType = UART_RESPONSE;
		DataLen = sizeof(BIT_status_t)+2;
		PageBuf[0] = uart_msg.request[UART_REQ_DATA_OFFSET];
		PageBuf[1] = uart_msg.request[UART_REQ_DATA_OFFSET+1];
		pData = PageBuf;
		if(uart_msg.request[UART_REQ_DATA_OFFSET] == 0){
			memcpy(PageBuf+2,(uint8_t*)&boot_BIT_status,sizeof(BIT_status_t));
		}else if(uart_msg.request[UART_REQ_DATA_OFFSET] == 1){
			run_BIT((BIT_status_t*)(PageBuf+2));
		}else{
			PageBuf[0] = UART_NACK_BAD_PARAM;
			DataLen = 1;
			RespType = UART_NACK_CMD;
		}
		break;
	case SET_TEMP_MIN_MAX:
		memcpy((uint8_t*)&(pConfigStruct->temperature_min),uart_msg.request+UART_REQ_DATA_OFFSET,TEMP_MINMAX_SIZE);
		ADC_UpdateThresholds();
		break;
	case GET_INTR_DATA:
		RespType = UART_RESPONSE;
		HAL_GPIO_WritePin(INT_GPIO_Port, INT_Pin, GPIO_PIN_RESET);
		DataLen = GetIntrData();
		pData = PageBuf;
		break;
	case SEND_IMU_CMD:
		imu_cmd((uint8_t*)(uart_msg.request+UART_REQ_DATA_OFFSET),&DataLen,&RespType);
		pData = PageBuf;
		break;
	case SET_BRIGHTNESS:
		if(CurrentPowerMode == PWR_ON){ //use boot brightness table
			pConfigStruct->bootBrightness = uart_msg.request[UART_REQ_DATA_OFFSET];
			pConfigStruct->bootBrightness += (uart_msg.request[UART_REQ_DATA_OFFSET+1]<<8);
		}else if(CurrentPowerMode == PWR_OPERATION){
			pConfigStruct->brightness = uart_msg.request[UART_REQ_DATA_OFFSET];
			pConfigStruct->brightness += (uart_msg.request[UART_REQ_DATA_OFFSET+1]<<8);
		}
		setBrightnesas();
		if(sw_error != HAL_OK){
			PageBuf[0] = UART_ANS_UNKNOWN_ERR;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_ERROR_CMD;
		}
		break;
	case GET_CURRENT_BRIGHTNESS:
		RespType = UART_RESPONSE;
		if(CurrentPowerMode == PWR_ON){ //use boot brightness table
			PageBuf[0] = pConfigStruct->bootBrightness & 0xFF;
			PageBuf[1] = pConfigStruct->bootBrightness >> 8;
		}else if(CurrentPowerMode == PWR_OPERATION){
			PageBuf[0] = pConfigStruct->brightness & 0xFF;
			PageBuf[1] = pConfigStruct->brightness >> 8;
		}
		else{
			PageBuf[0] = 0;
			PageBuf[1] = 0;
		}
		DataLen = sizeof(pConfigStruct->brightness);
		pData = PageBuf;
		break;
	case SET_RGB:
		led_RGB = 0;
		led_RGB |= (uart_msg.request[UART_REQ_DATA_OFFSET]==1)? LED_R_ON : 0;
		led_RGB |= (uart_msg.request[UART_REQ_DATA_OFFSET+1]==1)? LED_G_ON : 0;
		led_RGB |= (uart_msg.request[UART_REQ_DATA_OFFSET+2]==1)? LED_B_ON : 0;
		setBrightnesas();
		if(sw_error != HAL_OK){
			PageBuf[0] = UART_ANS_UNKNOWN_ERR;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_ERROR_CMD;
		}
		break;
	case GET_CURRENT_RGB:
		RespType = UART_RESPONSE;
		pData = PageBuf;
		DataLen = 6;
		if(CurrentPowerMode == PWR_ON){ //use boot brightness table
			pData  = (pConfigStruct->bootBrightnesTabNo)? FlashCopyBrightnessTab1 : FlashCopyBrightnessTab0; //BrightnessBaseAddr
			Address = pConfigStruct->bootBrightness*sizeof(brightnees_t);
			memcpy(PageBuf,pData + Address,6);
		}else if(CurrentPowerMode == PWR_OPERATION){  //use operation brightness table
			pData  = (pConfigStruct->brightnessTabNo)? FlashCopyBrightnessTab1 : FlashCopyBrightnessTab0; //BrightnessBaseAddr
			Address = pConfigStruct->brightness*sizeof(brightnees_t);
			memcpy(PageBuf,pData + Address,sizeof(brightnees_t));
		}else{
			memset(PageBuf,0,sizeof(brightnees_t));
		}
		pData = PageBuf;

		break;
	case SWITCH_BRIGHTNESS_TABLE:
		if(CurrentPowerMode == PWR_ON){ //use boot brightness table
			pConfigStruct->bootBrightnesTabNo &= ~1;
		}else if(CurrentPowerMode == PWR_OPERATION){  //use operation brightness table
			pConfigStruct->brightnessTabNo &= ~1;
		}
		setBrightnesas();
		if(sw_error != HAL_OK){
			PageBuf[0] = UART_ANS_UNKNOWN_ERR;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_ERROR_CMD;
		}
		break;
	case FLIP_DISPLAY_ON_X:
		pConfigStruct->flipScreenHor &= ~1;
		lcos_flip();
		if(sw_error != HAL_OK){
			PageBuf[0] = UART_ANSW_I2C_ERROR;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_ERROR_CMD;
		}
		break;
	case FLIP_DISPLAY_ON_Y:
		pConfigStruct->flipScreenVer &= ~1;
		lcos_flip();
		if(sw_error != HAL_OK){
			PageBuf[0] = UART_ANSW_I2C_ERROR;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_ERROR_CMD;
		}
		break;
	case MOVE_OFFSET_X:
		pConfigStruct->screenOffsetHor = uart_msg.request[UART_REQ_DATA_OFFSET];
		asic_ScreenOffset_Hor();
		if(sw_error != HAL_OK){
			PageBuf[0] = UART_ANSW_I2C_ERROR;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_ERROR_CMD;
		}
		break;
	case MOVE_OFFSET_Y:
		pConfigStruct->screenOffsetHor = uart_msg.request[UART_REQ_DATA_OFFSET];
		asic_ScreenOffset_Ver();
		if(sw_error != HAL_OK){
			PageBuf[0] = UART_ANSW_I2C_ERROR;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_ERROR_CMD;
		}
		break;
	case POWER_CMD:
		NextPowerMode = uart_msg.request[UART_REQ_DATA_OFFSET];
		if(NextPowerMode<= PWR_MODE_MAX){
			main_status.pwrModeChange_req = 1;
		}else{
			PageBuf[0] = UART_NACK_BAD_PARAM;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_NACK_CMD;
		}
		break;
	case GET_A2D:
		RespType = UART_RESPONSE;
		pData = PageBuf;
		DataLen = 3*sizeof(float);
		voltage_3V3 =  __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_raw_data[RAW_ADC_VDDA_OFFSET], ADC_RESOLUTION_12B);	//3.3 V
		float_buf = ((float)__HAL_ADC_CALC_DATA_TO_VOLTAGE(voltage_3V3,adc_raw_data[RAW_ADC_1V8_OFFSET], ADC_RESOLUTION_12B)/1000); //1.8 V
		memcpy(PageBuf,&float_buf,sizeof(float));
		float_buf = ((float)voltage_3V3/1000);	//3.3 V
		memcpy(PageBuf+sizeof(float),&float_buf,sizeof(float));
		float_buf = ((float)__HAL_ADC_CALC_DATA_TO_VOLTAGE(voltage_3V3,adc_raw_data[RAW_ADC_6V_OFFSET], ADC_RESOLUTION_12B)/1000);	//6.0 V
		memcpy(PageBuf+2*sizeof(float),&float_buf,sizeof(float));
		break;
	case GET_TEMP_CMD:
		RespType = UART_RESPONSE;
		pData = PageBuf;
		DataLen = sizeof(float);
		if(adc_raw_data[RAW_ADC_VDDA_OFFSET]!= 0){
			VDDA_mv =  __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_raw_data[RAW_ADC_VDDA_OFFSET],ADC_RESOLUTION_12B);
		}else{
			VDDA_mv = VDDA;
		}
#if 0
		float_buf = (float)__HAL_ADC_CALC_TEMPERATURE(VDDA_mv, adc_raw_data[RAW_ADC_TEMP_OFFSET], ADC_RESOLUTION_12B);
#else
		float_buf = ((( ((float)((adc_raw_data[RAW_ADC_TEMP_OFFSET]* VDDA_mv) / TEMPSENSOR_CAL_VREFANALOG) - (int32_t) *TEMPSENSOR_CAL1_ADDR)) * (int32_t)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)) / (int32_t)((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR)) + TEMPSENSOR_CAL1_TEMP;
#endif
		memcpy(PageBuf,&float_buf,sizeof(float));
		break;
	case SAVE_REGION_2_FLASH:
		get_flash_region(uart_msg.request[UART_REQ_DATA_OFFSET],&Address, &RespType);
		if(RespType == UART_ERROR_CMD){
			PageBuf[0] = UART_NACK_BAD_PARAM;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			Error_Handler();
			break;
		}
		pData = getRamRegion(uart_msg.request[UART_REQ_DATA_OFFSET], (uint16_t*)&DataLen);
		erase_page(uart_msg.request[UART_REQ_DATA_OFFSET],DataLen/FLASH_PAGE_SIZE,&RespType);
		if(RespType == UART_ERROR_CMD){
			PageBuf[0] = UART_ANSW_FLASH_ERASE_ERR;
			pData = PageBuf;
			DataLen = 1;
			Error_Handler();
		}else{
			HAL_FLASH_Unlock();
			for(i=0;i<FLASH_PAGE_SIZE/sizeof(uint64_t);i++){
				memcpy(&uint64_buf, pData,sizeof(uint64_t));
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address,uint64_buf) != HAL_OK){
					RespType = UART_ERROR_CMD;
					PageBuf[0] = UART_ANSW_FLASH_PROGRAM_ERR;
					pData = PageBuf;
					DataLen = 1;
					Error_Handler();
					break;
				}else{
					Address += sizeof(uint64_t);
					pData   += sizeof(uint64_t);
				}
			}
			HAL_FLASH_Lock();
			RespType = UART_ACK_CMD;
			DataLen = 1;
		}
		break;
	case LOAD_REGION_2_RAM:
		get_flash_region(uart_msg.request[UART_REQ_DATA_OFFSET],&Address, &RespType);
		if(RespType == UART_ERROR_CMD){
			PageBuf[0] = UART_NACK_BAD_PARAM;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			Error_Handler();
			break;
		}
		pData = getRamRegion(uart_msg.request[UART_REQ_DATA_OFFSET],&DataLen);
		memcpy(pData,(uint8_t*)Address,DataLen);
		RespType = UART_ACK_CMD;
		DataLen = 1;
		break;
	case GET_DATA_FROM_REGION:
		pData = getRamRegion(uart_msg.request[UART_REQ_DATA_OFFSET],&DataLen);
		if(DataLen == 0){
			PageBuf[0] = UART_NACK_BAD_PARAM;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			Error_Handler();
			break;
		}
		offset = uart_msg.request[UART_REQ_DATA_OFFSET+1];
		offset += uart_msg.request[UART_REQ_DATA_OFFSET+2]<<8;
		len = uart_msg.request[UART_REQ_DATA_OFFSET+3];
		len += uart_msg.request[UART_REQ_DATA_OFFSET+4]<<8;
		if((offset + len)>DataLen){
			PageBuf[0] = UART_NACK_BAD_PARAM;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			Error_Handler();
			break;
		}
		pData += offset;
		DataLen = len;
		RespType = UART_RESPONSE;
		break;
	case SET_REGION:
		pData = getRamRegion(uart_msg.request[UART_REQ_DATA_OFFSET],&DataLen); //region size
//		pData = FlashCopyDevInfo;
		if(DataLen == 0){
			PageBuf[0] = UART_NACK_BAD_PARAM;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			Error_Handler();
			break;
		}
		offset = uart_msg.request[UART_REQ_DATA_OFFSET+1];
		offset += uart_msg.request[UART_REQ_DATA_OFFSET+2]<<8;
		len = uart_msg.request[UART_REQ_DATA_OFFSET+3];
		len += uart_msg.request[UART_REQ_DATA_OFFSET+4]<<8;
		if((offset + len)>DataLen){
			PageBuf[0] = UART_NACK_BAD_PARAM;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			Error_Handler();
			break;
		}
		memcpy(pData + offset,uart_msg.request+UART_REQ_DATA_OFFSET + 5,len);
		RespType = UART_ACK_CMD;
		DataLen = 1;
		break;
	case SAVE_IN_STORAGE:
		HAL_FLASH_Unlock();
		get_flash_region(uart_msg.request[UART_REQ_DATA_OFFSET],&Address, &RespType);
		if(RespType == UART_ERROR_CMD){
			PageBuf[0] = UART_NACK_BAD_PARAM;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			Error_Handler();
			break;
		}
		offset = uart_msg.request[UART_REQ_DATA_OFFSET+1];
		offset += uart_msg.request[UART_REQ_DATA_OFFSET+2]<<8;
		Address += offset;
		DataLen = uart_msg.request[UART_REQ_DATA_OFFSET+3];
		DataLen += uart_msg.request[UART_REQ_DATA_OFFSET+4]<<8;
		if(DataLen > FLASH_PAGE_SIZE/2){
			PageBuf[0] = UART_NACK_BAD_LENGTH;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			pData = PageBuf;
			break;
		}
		DataStartPos = UART_REQ_DATA_OFFSET+5;
		AddrCorrection = Address & 0x07;
		if(AddrCorrection){
			RespType = UART_ERROR_CMD;
			PageBuf[0] = UART_ANSW_FLASH_DATA_OFFSET_ERR;
			pData = PageBuf;
			DataLen = 1;
			Error_Handler();
			break;
		}
		DataLen >>= 3;
		for(i=0;i<DataLen;i++){
			memcpy(&uint64_buf, (uint64_t*)(uart_msg.request+DataStartPos),sizeof(uint64_t));
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, uint64_buf) != HAL_OK){
				RespType = UART_ERROR_CMD;
				PageBuf[0] = UART_ANSW_FLASH_PROGRAM_ERR;
				pData = PageBuf;
				DataLen = 1;
				Error_Handler();
				break;
			}else{
				Address += sizeof(uint64_t);
				DataStartPos += sizeof(uint64_t);
			}
		}
		if(RespType == UART_ERROR_CMD){
			break;
		}
		/* Lock the Flash to disable the flash control register access (recommended
		 to protect the FLASH memory against possible unwanted operation) *********/
		HAL_FLASH_Lock();
		DataLen = 0;
		break;
	case GET_FROM_STORAGE:
		RespType = UART_RESPONSE;
		get_flash_region(uart_msg.request[UART_REQ_DATA_OFFSET],&Address, &RespType);
		if(RespType == UART_ERROR_CMD){
			PageBuf[0] = UART_NACK_BAD_PARAM;
			pData = PageBuf;
			RespType = UART_NACK_CMD;
			DataLen = 1;
			Error_Handler();
		}else{
			offset = uart_msg.request[UART_REQ_DATA_OFFSET+1];
			offset += uart_msg.request[UART_REQ_DATA_OFFSET+2]<<8;
			Address += offset;
			pData = (uint8_t*)Address;
			DataLen = uart_msg.request[UART_REQ_DATA_OFFSET+3];
			DataLen += uart_msg.request[UART_REQ_DATA_OFFSET+4]<<8;
			if(DataLen > FLASH_PAGE_SIZE/2){
				PageBuf[0] = UART_NACK_BAD_LENGTH;
				DataLen = 1;
				RespType = UART_NACK_CMD;
				pData = PageBuf;
			}
		}
		break;
	case ERASE_STORAGE_PAGE:
		erase_page(uart_msg.request[UART_REQ_DATA_OFFSET],uart_msg.request[UART_REQ_DATA_OFFSET+1],&RespType);
		if(RespType == UART_ERROR_CMD){
			PageBuf[0] = UART_NACK_BAD_PARAM;
			pData = PageBuf;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			Error_Handler();
		}
		break;
	case I2C_WRITE:
		if(uart_msg.request[UART_REQ_DATA_OFFSET]==0){
			pi2c_bus = &hi2c1;
		}else if(uart_msg.request[UART_REQ_DATA_OFFSET]==1){
			pi2c_bus = &hi2c3;
		}else{
			PageBuf[0] = UART_NACK_BAD_PARAM;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			pData = PageBuf;
			break;
		}
		PageBuf[0] = uart_msg.request[UART_REQ_DATA_OFFSET+1]<<1; //device address
		PageBuf[1] = uart_msg.request[UART_REQ_DATA_OFFSET+2]; //register address
		PageBuf[2] = uart_msg.request[UART_REQ_DATA_OFFSET+3];	// data count
		for(i=0;i<PageBuf[2];i++){
			PageBuf[3+i] = uart_msg.request[UART_REQ_DATA_OFFSET+4+i];	// data
		}
		sw_error = HAL_I2C_Mem_Write(pi2c_bus,PageBuf[0], PageBuf[1], 1, PageBuf+3, PageBuf[2], I2C_TX_TIMEOUT);
		if(sw_error != HAL_OK){
			PageBuf[0] = UART_ANSW_I2C_ERROR;
			DataLen = 1;
			RespType = UART_ERROR_CMD;
			pData = PageBuf;
		}
		break;
	case I2C_READ:
		pData = PageBuf;
		if(uart_msg.request[UART_REQ_DATA_OFFSET]==0){
			pi2c_bus = &hi2c1;
		}else if(uart_msg.request[UART_REQ_DATA_OFFSET]==1){
			pi2c_bus = &hi2c3;
		}else{
			PageBuf[0] = UART_NACK_BAD_PARAM;
			DataLen = 1;
			RespType = UART_NACK_CMD;
			break;
		}
		PageBuf[0] = uart_msg.request[UART_REQ_DATA_OFFSET];  		//Number of I2C bus:
		PageBuf[1] = uart_msg.request[UART_REQ_DATA_OFFSET+1]<<1; 	//device address
		PageBuf[2] = uart_msg.request[UART_REQ_DATA_OFFSET+2]; 		//register address
		len = uart_msg.request[UART_REQ_DATA_OFFSET+3];				// data count
		sw_error = HAL_I2C_Mem_Read(pi2c_bus, PageBuf[1], PageBuf[2], 1, PageBuf+3, len, I2C_RX_TIMEOUT);
		if(sw_error != HAL_OK){
			PageBuf[0] = UART_ANSW_I2C_ERROR;
			RespType = UART_ERROR_CMD;
			DataLen = 1;
			pData = PageBuf;
		}else{
			RespType = UART_RESPONSE;
			DataLen = len+3;
		}
		break;
	case RUN_SCRIPT:
		len = (uart_msg.request[UART_REQ_MSG_LEN_OFFSET]+(uint16_t)(uart_msg.request[UART_REQ_MSG_LEN_OFFSET+1]<<8) - UART_EMPTY_RESP_LEN)/SCRIPT_RECORD_SIZE;
		runScript(uart_msg.request+UART_REQ_DATA_OFFSET,len);
		if(sw_error != HAL_OK){
			PageBuf[0] = UART_NACK_UNKNOWN;
			RespType = UART_NACK_CMD;
			DataLen = 1;
			pData = PageBuf;
		}
		break;
	default:
		PageBuf[0] = UART_NACK_BAD_OPCODE;
		DataLen = 1;
		RespType = UART_NACK_CMD;
		pData = PageBuf;
		break;
	}
	UART_Tx(pData, DataLen, RespType);
	return HAL_OK;
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

static void get_flash_region(uint8_t RegId,uint32_t* pRegAddr, uint8_t* pRespType)
{
	switch(RegId){   //look  flash region
	case FLASH_DEVICE_INFO_ID:
		*pRegAddr = FLASH_DEVICE_INFO_BASE_ADDR;
		break;
	case FLASH_CONFIG_ID:
		*pRegAddr = FLASH_CONFIG_BASE_ADDR;
		break;
	case FLASH_BRIGHTNESS_TABLE_0_ID:
		*pRegAddr = FLASH_BRIGHTNESS_0_BASE_ADDR;
		break;
	case FLASH_BRIGHTNESS_TABLE_1_ID:
		*pRegAddr = FLASH_BRIGHTNESS_1_BASE_ADDR;
		break;
	case FLASH_INIT_DATA_ID:
		*pRegAddr = FLASH_INIT_BASE_ADDR;
		break;
	default:
		*pRespType = UART_ERROR_CMD;
		break;
	}

}
static void erase_page(uint8_t RegID,uint8_t NbPages,uint8_t* pRespType){
	uint32_t FirstPage;	//first flash page for operation
	uint32_t PageError;
	uint32_t Addr;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();
	get_flash_region(RegID,&Addr, pRespType);
	if(*pRespType != UART_ERROR_CMD){
		/* Get the 1st page to erase */
		FirstPage = GetPage(Addr);

		/* Fill EraseInit structure*/
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Banks       = FLASH_BANK_1;
		EraseInitStruct.Page        = FirstPage;
		EraseInitStruct.NbPages     = NbPages;

		/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
		 you have to make sure that these data are rewritten before they are accessed during code
		 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
		 DCRST and ICRST bits in the FLASH_CR register. */
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK){
//			FlshOpError = HAL_FLASH_GetError();
			Error_Handler();
			*pRespType = UART_ERROR_CMD;
		}
	}
	HAL_FLASH_Lock();
}

static uint16_t GetIntrData(void){
	uint16_t i = 0;
	float floatVal;
	uint8_t NumOfInterrupts = 0;
	HAL_GPIO_WritePin(INT_GPIO_Port, INT_Pin, GPIO_PIN_RESET);

	i++;		// for NumOfInterrupts;

	if(irq_flg.irq_IMU){
		NumOfInterrupts++;
		PageBuf[i++] = INT_IMU;
		PageBuf[i++] = IMU_GIRO_DATA_LEN+IMU_ACCSEL_DATA_LEN+IMU_TEMP_DATA_LEN;
		PageBuf[i++] = 0;
		memcpy((uint8_t*)(PageBuf+i),imu_data,IMU_ACCSEL_DATA_LEN);		//ACCSEL_DATA
		i+=IMU_ACCSEL_DATA_LEN;
		memcpy((uint8_t*)(PageBuf+i),(uint8_t*)(imu_data+IMU_GYRO_DATA_OFFSET),IMU_GIRO_DATA_LEN); //GIRO_DATA
		i+=IMU_GIRO_DATA_LEN;
		//T_c - (TEMP_DATA / 132.48) + 25
		floatVal = imu_data[IMU_TEMP_DATA_OFFSET];
		floatVal += imu_data[IMU_TEMP_DATA_OFFSET+1]<<8;
		floatVal /= 132.48;
		floatVal += 25;
		memcpy((uint8_t*)(PageBuf+i),&floatVal,sizeof(float));		//temperature
		i += sizeof(float);
	}
	if(irq_flg.irq_TEMP){
		NumOfInterrupts++;
		PageBuf[i++] = INT_TEMP;
		PageBuf[i++] = sizeof(float);
		PageBuf[i++] = 0;
		floatVal  = (float)__HAL_ADC_CALC_TEMPERATURE(VDDA,adc_raw_data[RAW_ADC_TEMP_OFFSET],ADC_RESOLUTION_12B);
		memcpy((uint8_t*)(PageBuf+i),(uint8_t*)&floatVal,sizeof(float));
		i+=sizeof(float);
	}
	if(irq_flg.irq_POWER){
		NumOfInterrupts++;
		PageBuf[i++] = INT_POWER;
		PageBuf[i++] = 3*sizeof(float);
		PageBuf[i++] = 0;

		floatVal = (float)__HAL_ADC_CALC_DATA_TO_VOLTAGE(VDDA,adc_raw_data[RAW_ADC_6V_OFFSET],ADC_RESOLUTION_12B); // 6V
		memcpy((uint8_t*)(PageBuf+i),(uint8_t*)&floatVal,sizeof(float));
		i+=sizeof(float);
		floatVal = (float)__HAL_ADC_CALC_DATA_TO_VOLTAGE(VDDA,adc_raw_data[RAW_ADC_1V8_OFFSET],ADC_RESOLUTION_12B);
		memcpy((uint8_t*)(PageBuf+i),(uint8_t*)&floatVal,sizeof(float));
		i+=sizeof(float);
		floatVal =  (float)__HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_raw_data[RAW_ADC_VDDA_OFFSET],ADC_RESOLUTION_12B);
		memcpy((uint8_t*)(PageBuf+i),(uint8_t*)&floatVal,sizeof(float));
		i+=sizeof(float);
	}
	if(irq_flg.irq_LCOS){
		NumOfInterrupts++;
		PageBuf[i++] = INT_LCOS;
		PageBuf[i++] = sizeof(device_state_t);
		PageBuf[i++] = 0;
		memcpy((uint8_t*)(PageBuf+i),(uint8_t*)&lcos_state,sizeof(device_state_t));
		*(uint16_t*)&lcos_state = 0;
		i+=sizeof(device_state_t);
	}
	if(irq_flg.irq_ASIC){
		NumOfInterrupts++;
		PageBuf[i++] = INT_ASIC;
		PageBuf[i++] = sizeof(device_state_t);
		PageBuf[i++] = 0;
		memcpy((uint8_t*)(PageBuf+i),(uint8_t*)&asic_state,sizeof(device_state_t));
		*(uint16_t*)&asic_state = 0;
		i+=sizeof(device_state_t);
	}
	if(irq_flg.irq_VGA2RGB){
		NumOfInterrupts++;
		PageBuf[i++] = INT_ASIC;
		PageBuf[i++] = sizeof(device_state_t);
		PageBuf[i++] = 0;
		memcpy((uint8_t*)(PageBuf+i),(uint8_t*)&vga2rgb_state,sizeof(device_state_t));
		*(uint16_t*)&vga2rgb_state = 0;
		i+=sizeof(device_state_t);
	}
	if(irq_flg.irq_LED_DRIVER){
		NumOfInterrupts++;
		PageBuf[i++] = INT_ASIC;
		PageBuf[i++] = sizeof(led_drv_state_t);
		PageBuf[i++] = 0;
		memcpy((uint8_t*)(PageBuf+i),(uint8_t*)&led_drv_state,sizeof(led_drv_state_t));
		*(uint16_t*)&led_drv_state = 0;
		i+=sizeof(led_drv_state_t);
	}
	if(irq_flg.irq_InitFinish){
		NumOfInterrupts++;
		PageBuf[i++] = INT_INIT_FINISH;
		PageBuf[i++] = 0;
		PageBuf[i++] = 0;
	}
	if(irq_flg.irq_IMU_err){
		NumOfInterrupts++;
		PageBuf[i++] = INT_IMU_ERR;
		PageBuf[i++] = sizeof(device_state_t);
		PageBuf[i++] = 0;
		memcpy((uint8_t*)(PageBuf+i),(uint8_t*)&imu_state,sizeof(device_state_t));
		*(uint16_t*)&imu_state = 0;
		i+=sizeof(led_drv_state_t);
	}
	*(uint32_t*)&irq_flg = 0;	// clear interrupt reason
	PageBuf[0] = NumOfInterrupts;
	return i;
}


static void imu_cmd(uint8_t* pCmd,uint16_t* DataLen,uint8_t* RespType){
	// pCmd[0] - wr/rd
	// pCmd[1] - register address
	// pCmd[2] -  length
	// pCmd[3 ..] - data
	PageBuf[0] = pCmd[1];			//register address
	*DataLen = pCmd[2];				// data length
 	if(*pCmd == IMU_CMD_READ){
 		sw_error = HAL_I2C_Mem_Read(&hi2c3, IMU_I2C_ADDR<<1, PageBuf[0], sizeof(uint8_t), PageBuf+1, *DataLen,I2C_RX_TIMEOUT);
		if(sw_error == HAL_OK){
			*RespType = UART_RESPONSE;
		}else{
			*RespType = UART_ERROR_CMD;
			*DataLen = 1;
			PageBuf[0] = UART_ANSW_I2C_ERROR;
		}
	}else if(*pCmd == IMU_CMD_WRITE){
		sw_error = HAL_I2C_Mem_Write(&hi2c3, IMU_I2C_ADDR<<1, PageBuf[0], sizeof(uint8_t), pCmd+3, *DataLen, I2C_TX_TIMEOUT);
		if(sw_error == HAL_OK){
			*RespType = UART_ACK_CMD;
			*DataLen = 1;
		}else{
			*RespType = UART_ERROR_CMD;
			*DataLen = 1;
			PageBuf[0] = UART_ANSW_I2C_ERROR;
		}
	}else{
		PageBuf[0] = UART_NACK_BAD_PARAM;
		*DataLen = 1;
		*RespType = UART_NACK_CMD;
	}
}
static uint8_t* getRamRegion(uint8_t regID, uint16_t* DataLen){
	uint8_t* pData;
	switch(regID){
	case FLASH_DEVICE_INFO_ID:
		pData = FlashCopyDevInfo;
		*DataLen = sizeof(FlashCopyDevInfo);
		break;
	case FLASH_CONFIG_ID:
		pData = FlashCopyConfigStruct;
		*DataLen = sizeof(FlashCopyConfigStruct);
		break;
	case FLASH_BRIGHTNESS_TABLE_0_ID:
		pData = FlashCopyBrightnessTab0;
		*DataLen = sizeof(FlashCopyBrightnessTab0);
		break;
	case FLASH_BRIGHTNESS_TABLE_1_ID:
		pData = FlashCopyBrightnessTab1;
		*DataLen = sizeof(FlashCopyBrightnessTab1);
		break;
	case FLASH_INIT_DATA_ID:
		pData = FlashCopyInitData;
		*DataLen = sizeof(FlashCopyInitData);
		break;
	default:
		pData = NULL;
		*DataLen = 0;
	}
	return pData;
}

static void run_BIT(BIT_status_t* pBIT_status){
	uint8_t buff;
	pBIT_status->ASIC_init_status = 1;
	if(HAL_I2C_Mem_Read(&hi2c1, ASIC_I2C_ADDR<<1, HX7816_PASSWORD_ADDR, 1,&buff, 1, I2C_RX_TIMEOUT)==HAL_OK){
		if(buff == HX7816_PASSWORD_ADDR){
			pBIT_status->ASIC_init_status = 0;
		}
	}
	pBIT_status->ADC_Error = boot_BIT_status.ADC_Error;
	if(pBIT_status->ADC_Error == 0){
		pBIT_status->Voltage_err = 0;
		if( (VDDA_mv < ADC_3V3_MIN) 	||
			(VDDA_mv > ADC_3V3_MAX) 	||
			(mon_6V_mv   < ADC_6V_MIN) 	||
			(mon_6V_mv   > ADC_6V_MAX) 	||
			(mon_1V8_mv  < ADC_1V8_MIN) ||
			(mon_1V8_mv > ADC_1V8_MAX)){
			pBIT_status->Voltage_err = 1;
		}
		pBIT_status->Temp_err = 0;
		if((adc_raw_data[RAW_ADC_TEMP_OFFSET]>TemperatureHighThreshold)||
				(adc_raw_data[RAW_ADC_TEMP_OFFSET]<TemperatureLowThreshold)){
			pBIT_status->Temp_err = 1;
		}
	}else{ // if ADC is wrong measurements are wrong
		pBIT_status->Voltage_err = 1;
		pBIT_status->Temp_err = 1;
	}
	pBIT_status->DeviceID_error = ((*(uint32_t*)FlashCopyDevInfo == 0)||(*(uint32_t*)FlashCopyDevInfo == 1))? 0:1;

	if((pConfigStruct->temperature_min <-40)	||
		(pConfigStruct->temperature_max>125)	||
		(pConfigStruct->brightnessTabNo>1)		||
		(pConfigStruct->flipScreenHor>1)		||
		(pConfigStruct->flipScreenVer>1)		||
		(pConfigStruct->screenOffsetHor<-50)	||
		(pConfigStruct->screenOffsetHor>50)		||
		(pConfigStruct->screenOffsetVer<-30)	||
		(pConfigStruct->screenOffsetVer>30)		||
		(pConfigStruct->bootBrightness>0x03FF)	||
		(pConfigStruct->brightness>0x03FF)){
		pBIT_status->FlashConfigTabError = 1;
	}else{
		pBIT_status->FlashConfigTabError = 0;
	}
	buff = IMU_Read_ID();
	pBIT_status->IMU_init_status = (buff == IMU_ID)? 0:1;
	pBIT_status->LCOS_init_status = (HAL_I2C_Mem_Read(&hi2c1, LCOS_I2C_ADDR<<1, LCOS_CTRL_REG, sizeof(uint8_t), &buff, 1, I2C_TX_TIMEOUT)==HAL_OK)? 0:1;
	pBIT_status->LED_DRIVER_init_status = 1;
	if(HAL_I2C_Mem_Read(&hi2c1, LED_DRIVER_I2C_ADDR<<1, LED_DRV_FAULT_REG, 1, &buff, 1,I2C_RX_TIMEOUT)==HAL_OK){
		if(buff == 0){
			pBIT_status->LED_DRIVER_init_status = 0;
		}
	}
	pBIT_status->VGA2RGB_init_status = 1;
	if(HAL_I2C_Mem_Read(&hi2c1, VGA_TO_RGB_I2C_ADDR<<1, VGA2RGB_REG_CHIP_REV, 1, &buff, 1,I2C_RX_TIMEOUT)==HAL_OK){
		if(buff == VGA2RGB_CHIP_REV){
			pBIT_status->VGA2RGB_init_status = 0;
		}
	}

}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
