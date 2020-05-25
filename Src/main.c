/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usart.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

LPTIM_HandleTypeDef hlptim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t debug_cntr = 0;


const version_t sw_revision = {1,0,2,3};

uint8_t FlashCopyDevInfo[FLASH_DEVICE_INFO_SIZE];
uint8_t FlashCopyConfigStruct[FLASH_CONFIG_SIZE];
uint8_t FlashCopyBrightnessTab0[FLASH_BRIGHTNESS_TAB_SIZE];
uint8_t FlashCopyBrightnessTab1[FLASH_BRIGHTNESS_TAB_SIZE];
uint8_t FlashCopyInitData[FLASH_INIT_DATA_SIZE];

config_t* pConfigStruct;
uint8_t sw_error = HAL_OK;
uint8_t LPTIM1_us_countEn;
BIT_status_t boot_BIT_status = {0};    // boot error status
irq_flg_t  irq_flg = {0};
main_status_t main_status = {0};
power_state_t CurrentPowerMode = PWR_OFF;
power_state_t NextPowerMode;

// ADC variables
uint32_t TemperatureHighThreshold;
uint32_t TemperatureLowThreshold;
uint16_t adc_raw_data[ADC_CONV_NUM];
uint16_t VDDA_mv;
uint16_t mon_6V_mv;
uint16_t mon_1V8_mv;

device_state_t	vga2rgb_state = {0};

extern uart_msg_t uart_msg;
extern uint8_t RdByteReady;
extern led_drv_state_t led_drv_state;
extern uint16_t asic_BasePos_Hor;
extern uint16_t asic_BasePos_Ver;
extern device_state_t lcos_state;
extern device_state_t asic_state;
extern device_state_t	imu_state;
extern uint8_t imu_data[IMU_DATA_LEN];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
//void runScript(uint8_t* BaseAddr, uint16_t recordNum);
static void init_periferial(uint8_t boot);
static void SetPowerMode(uint8_t NewPowerMode);
static void ADC_start(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  memcpy(FlashCopyDevInfo,(uint8_t*)FLASH_DEVICE_INFO_BASE_ADDR,sizeof(FlashCopyDevInfo));
  memcpy(FlashCopyConfigStruct,(uint8_t*)FLASH_CONFIG_BASE_ADDR,sizeof(FlashCopyConfigStruct));
  memcpy(FlashCopyBrightnessTab0,(uint8_t*)FLASH_BRIGHTNESS_0_BASE_ADDR,sizeof(FlashCopyBrightnessTab0));
  memcpy(FlashCopyBrightnessTab1,(uint8_t*)FLASH_BRIGHTNESS_1_BASE_ADDR,sizeof(FlashCopyBrightnessTab1));
  memcpy(FlashCopyInitData,(uint8_t*)FLASH_INIT_BASE_ADDR,sizeof(FlashCopyInitData));
  pConfigStruct = (config_t*)FlashCopyConfigStruct;
  *(unsigned*)&uart_msg.flag = 0;
  uart_msg.response[0] = 'H';
  uart_msg.response[1] = 'U';
  uart_msg.response[2] = 'M';

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LPTIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SetPowerMode(PWR_OFF);
  SetPowerMode(PWR_ON);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef LPM_EN
	  /*Suspend Tick increment to prevent wakeup by Systick interrupt.
		Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
	  HAL_SuspendTick();

		/* Enter Sleep Mode , wake up is done once jumper is put between PA.12 (Arduino D2) and GND */
	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

		/* Resume Tick interrupt if disabled prior to SLEEP mode entry */
	  HAL_ResumeTick();
#endif

	  if(RdByteReady){
		  RdByteReady = 0;
		  UART_Rx_Byte();		//receive 1 byte by UART
	  }
	  if(uart_msg.flag.uart_req_ready_flg){
		  uart_msg.flag.uart_req_ready_flg = 0;
		  uartReqHandler();
	  }
	  if(*(uint32_t*)&irq_flg){
		  HAL_GPIO_WritePin(INT_GPIO_Port, INT_Pin, GPIO_PIN_SET);
	  }

	  if (main_status.led_drv_fault){

	  }
	  if(main_status.pwrModeChange_req){
		  main_status.pwrModeChange_req = 0;
		  SetPowerMode(NextPowerMode);
	  }
	  if( main_status.ADC_dataReady == 1){
		  ADC_Calc();
		  main_status.ADC_dataReady = 0;
		  if(debug_cntr++>1000){
			  printf("mon_1V8_mv = %d\n\r ",mon_1V8_mv);
			  debug_cntr = 0;
		  }
	  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_LPTIM1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analog WatchDog 1 
  */
  AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 4000;
  AnalogWDGConfig.LowThreshold = 10;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00100413;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Fast mode Plus enable 
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C3);
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV32;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  hlptim1.Init.RepetitionCounter = 0;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_DAY_Pin|LCOS_nRST_Pin|INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, I2COUT_EN_Pin|LED_EN_Pin|VGA_nRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HX7816_nRST_GPIO_Port, HX7816_nRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IMU_IRQ1_Pin LED_FAULT_Pin */
  GPIO_InitStruct.Pin = IMU_IRQ1_Pin|LED_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_DAY_Pin LCOS_nRST_Pin INT_Pin */
  GPIO_InitStruct.Pin = LED_DAY_Pin|LCOS_nRST_Pin|INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : I2COUT_EN_Pin LED_EN_Pin VGA_nRST_Pin */
  GPIO_InitStruct.Pin = I2COUT_EN_Pin|LED_EN_Pin|VGA_nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_IRQ2_Pin */
  GPIO_InitStruct.Pin = IMU_IRQ2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IMU_IRQ2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HX7816_nRST_Pin */
  GPIO_InitStruct.Pin = HX7816_nRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HX7816_nRST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

static void init_periferial(uint8_t boot)
{
	uint32_t card_id = *(uint32_t*)FlashCopyDevInfo;
	uint8_t* pDevBaseAddr;
	uint8_t i = 0;
	flash_header_record_t flash_header_record[5];
	memcpy((uint8_t*)flash_header_record,FlashCopyInitData,5*sizeof(flash_header_record_t));
	*(uint32_t*)&boot_BIT_status = BIT_STATUS_MASK;
	if((card_id == CARD_ID_0)||(card_id == CARD_ID_1)){ 			// check card_ID
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
			boot_BIT_status.FlashConfigTabError = 1;
			return;
		}
		boot_BIT_status.FlashConfigTabError = 0;
		if(boot){
			pDevBaseAddr = (uint8_t*)(FlashCopyInitData+flash_header_record[i++].offset);
		}else{
			pDevBaseAddr = (uint8_t*)(FlashCopyInitData+flash_header_record[4].offset);
			i++;
		}
		runScript(pDevBaseAddr,flash_header_record[i++].cmd_num); //init HX7816_BOOT
		if(sw_error==HAL_OK){
			asic_SetPattern(ASIC_PATTERN_GRAY_SCASLE,0);
			if(sw_error==HAL_OK){
				asic_GetScreenPos(&asic_BasePos_Hor, &asic_BasePos_Ver);
				asic_ScreenOffset_Hor();
				asic_ScreenOffset_Ver();
				boot_BIT_status.ASIC_init_status = DEV_INITIALISED;
			}
		}else{
			boot_BIT_status.ASIC_init_status = DEV_INITIALISATIN_ERROR;
			asic_state.init_error = 1;
			irq_flg.irq_ASIC = 1;
			Error_Handler();
		}
		pDevBaseAddr = (uint8_t*)(FlashCopyInitData+flash_header_record[i++].offset);
		runScript(pDevBaseAddr,flash_header_record[i++].cmd_num); //init LCOS HX7318
		if(sw_error==HAL_OK){
			lcos_flip();
			if(lcos_state.i2c_error){
				irq_flg.irq_LCOS = 1;
			}else{
				boot_BIT_status.LCOS_init_status = DEV_INITIALISED;
//				lcos_GetScreenPos(&asic_BasePoz_Hor, &asic_BasePoz_Ver);
			}

		}else{
			boot_BIT_status.LCOS_init_status = DEV_INITIALISATIN_ERROR;
			lcos_state.init_error = 1;
			irq_flg.irq_LCOS = 1;
			Error_Handler();
		}
		pDevBaseAddr = (uint8_t*)(FlashCopyInitData+flash_header_record[i++].offset);
		runScript(pDevBaseAddr,flash_header_record[i++].cmd_num); //init TVP70025
		if(sw_error==HAL_OK){
			boot_BIT_status.VGA2RGB_init_status = DEV_INITIALISED;
		}else{
			boot_BIT_status.VGA2RGB_init_status = DEV_INITIALISATIN_ERROR;
			vga2rgb_state.init_error = 1;
			irq_flg.irq_VGA2RGB = 1;
			Error_Handler();
		}
		pDevBaseAddr = (uint8_t*)(FlashCopyInitData+flash_header_record[i++].offset);
		runScript(pDevBaseAddr,flash_header_record[i++].cmd_num); //init IMU
		if(sw_error==HAL_OK){
			boot_BIT_status.IMU_init_status = DEV_INITIALISED;
		}else{
			boot_BIT_status.IMU_init_status = DEV_INITIALISATIN_ERROR;
			imu_state.init_error = 1;
			irq_flg.irq_IMU = 1;

			Error_Handler();
		}
		setBrightnesas();
		if(sw_error==HAL_OK){
			boot_BIT_status.LED_DRIVER_init_status = DEV_INITIALISED;
		}else{
			boot_BIT_status.LED_DRIVER_init_status = DEV_INITIALISATIN_ERROR;
			Error_Handler();
		}
	//	  adc_set_watchdog_high_threshold();
	  /* Run the ADC calibration in single-ended mode */
//	  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
//	  {
//		/* Calibration Error */
//		Error_Handler();
//	  }

	  /*## Start ADC conversions #################################################*/

	  /* Start ADC conversion on regular group with transfer by DMA */
//	  if (HAL_ADC_Start_DMA(&hadc1, adc_convert_data,ADC_CONVERTED_DATA_BUFFER_SIZE) != HAL_OK)
//	  {
//		Error_Handler();
//	  }
	}else{ //if((card_id == CARD_ID_0)||(card_id == CARD_ID_1))
		boot_BIT_status.DeviceID_error = 1;
		sw_error = FLASH_NOT_INITIALISED;
		Error_Handler();
	}
	irq_flg.irq_InitFinish = 1;
}
void runScript(uint8_t* pData, uint16_t recordNum){
	uint32_t period = 0;
	uint8_t i2c_addr;
	I2C_HandleTypeDef* pi2c_bus;
	uint16_t i;
	uint8_t command;
	uint8_t cmdData[2];
	sw_error=HAL_OK;
	for(i=0;i<recordNum;i++){
		command = *pData++;
		cmdData[0] = *pData++;
		cmdData[1] = *pData++;
		switch(command){
		case FLASH_CMD_DEVICE_DESTINATION:
			if(cmdData[0]==0){
				pi2c_bus = &hi2c1;//use I2C0
			}else if(cmdData[0]){
				pi2c_bus = &hi2c3;//use I2C3
			}else{
				sw_error = PARAM_ERROR;
				return;
			}
			i2c_addr = cmdData[1]<<1;
			break;
		case FLASH_CMD_I2C_WRITE_8x8:
			sw_error = HAL_I2C_Master_Transmit(pi2c_bus, i2c_addr, cmdData, sizeof(cmdData), I2C_TX_TIMEOUT);
			break;
		case FLASH_CMD_DELAY_MS:
			HAL_Delay(cmdData[0]);
			break;
		case FLASH_CMD_DELAY_US:
			period = cmdData[0];
			period += cmdData[1]<<8;
			LPTIM1_us_countEn = 1;
			sw_error = HAL_LPTIM_Counter_Start_IT(&hlptim1,period);
			if(sw_error==HAL_OK){
				while (LPTIM1_us_countEn);
			}
			break;
		default:
			sw_error = PARAM_ERROR;
			return ;
		}//switch(command)
		if(sw_error!=HAL_OK){
			return ;
		}
	}//for(i=0;i<recordNum;i++)

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	  UNUSED(hadc);
//	  main_status.ADC_dataReady = 1;
}

static void SetPowerMode(uint8_t NewPowerMode) {
	switch(NewPowerMode){
	case PWR_OFF:
		CurrentPowerMode = NewPowerMode;
		HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCOS_nRST_GPIO_Port, LCOS_nRST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VGA_nRST_GPIO_Port, VGA_nRST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VGA_nRST_GPIO_Port, HX7816_nRST_Pin, GPIO_PIN_RESET);
		IMU_PowerOff();
		HAL_ADC_Stop_DMA(&hadc1);
// IMU power off - ??
		break;
	case PWR_ON:
		if(CurrentPowerMode != PWR_ON){
			CurrentPowerMode = PWR_ON;
//			HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LCOS_nRST_GPIO_Port, LCOS_nRST_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(VGA_nRST_GPIO_Port, VGA_nRST_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(VGA_nRST_GPIO_Port, HX7816_nRST_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			init_periferial(1);
			ADC_start();
		}
		break;
	case PWR_OPERATION:
		if((CurrentPowerMode == PWR_OFF)||(CurrentPowerMode == PWR_IMU)){
			// init PWR_ON mode
			CurrentPowerMode = PWR_ON;
			HAL_GPIO_WritePin(LCOS_nRST_GPIO_Port, LCOS_nRST_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(VGA_nRST_GPIO_Port, VGA_nRST_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(VGA_nRST_GPIO_Port, HX7816_nRST_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
			init_periferial(0);
		}
		if(CurrentPowerMode == PWR_ON){
			CurrentPowerMode = PWR_OPERATION;
			asic_SetPattern(ASIC_PATTERN_GRAY_SCASLE,0);
			setBrightnesas();
			if(sw_error != HAL_OK){
				led_drv_state.i2c_status = 1;
				irq_flg.irq_LED_DRIVER = 1;
			}
		}
		break;
	case PWR_IMU:
		CurrentPowerMode = NewPowerMode;
		HAL_GPIO_WritePin(LED_EN_GPIO_Port, LED_EN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LCOS_nRST_GPIO_Port, LCOS_nRST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VGA_nRST_GPIO_Port, VGA_nRST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VGA_nRST_GPIO_Port, HX7816_nRST_Pin, GPIO_PIN_RESET);

		break;
	default:

		break;
	}
}
void ADC_UpdateThresholds(void){
	uint32_t delta_CAL_TEMP = (int32_t)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP);
	uint32_t ts_delta_cal = (int32_t)((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR);

	if(adc_raw_data[RAW_ADC_VDDA_OFFSET]!= 0){
		VDDA_mv =  __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_raw_data[RAW_ADC_VDDA_OFFSET],ADC_RESOLUTION_12B);
	}else{
		VDDA_mv = VDDA;
	}

	TemperatureLowThreshold =  ((pConfigStruct->temperature_min - TEMPSENSOR_CAL1_TEMP)*ts_delta_cal/delta_CAL_TEMP + (int32_t) *TEMPSENSOR_CAL1_ADDR)*TEMPSENSOR_CAL_VREFANALOG / VDDA_mv;
	TemperatureHighThreshold =  ((pConfigStruct->temperature_max - TEMPSENSOR_CAL1_TEMP)*ts_delta_cal/delta_CAL_TEMP + (int32_t) *TEMPSENSOR_CAL1_ADDR)*TEMPSENSOR_CAL_VREFANALOG / VDDA_mv;

	/** Configure Analog WatchDog 1
	*/

//	HAL_ADC_Stop(&hadc1);
//	LL_ADC_ConfigAnalogWDThresholds	(ADC1, LL_ADC_AWD1,	TemperatureHighThreshold,TemperatureLowThreshold);
//	HAL_ADC_Start(&hadc1);


//	AnalogWDGConfig.WatchdogNumber = ADC_ANALOGWATCHDOG_1;
//	AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
//	AnalogWDGConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
//	AnalogWDGConfig.ITMode = DISABLE;
//	AnalogWDGConfig.HighThreshold = HighThreshold;
//	AnalogWDGConfig.LowThreshold = LowThreshold;
//	if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	else{
//		HAL_ADC_Start(&hadc1);
//	}

}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  /* Set variable to report analog watchdog out of window status to main      */
  /* program.                                                                 */
	irq_flg.irq_TEMP = SET;
}
void ADC_Calc(void){

	if(adc_raw_data[RAW_ADC_VDDA_OFFSET]!= 0){
		VDDA_mv =  __HAL_ADC_CALC_VREFANALOG_VOLTAGE(adc_raw_data[RAW_ADC_VDDA_OFFSET],ADC_RESOLUTION_12B);
	}else{
		VDDA_mv = VDDA;
	}
	mon_6V_mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_mv,adc_raw_data[RAW_ADC_6V_OFFSET],ADC_RESOLUTION_12B);
	mon_1V8_mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_mv,adc_raw_data[RAW_ADC_1V8_OFFSET],ADC_RESOLUTION_12B);

	if( (VDDA_mv < ADC_3V3_MIN) 	||
		(VDDA_mv > ADC_3V3_MAX) 	||
		(mon_6V_mv   < ADC_6V_MIN) 	||
		(mon_6V_mv   > ADC_6V_MAX) 	||
		(mon_1V8_mv  < ADC_1V8_MIN) ||
		(mon_1V8_mv > ADC_1V8_MAX)){
		irq_flg.irq_POWER = 1;
	}

//	temperature = (float)__HAL_ADC_CALC_TEMPERATURE(VDDA_mv,adc_raw_data[RAW_ADC_TEMP_OFFSET],ADC_RESOLUTION_12B);
//	if( (adc_raw_data[RAW_ADC_VDDA_OFFSET] > ADC_3V3_MIN) 	||
//		(adc_raw_data[RAW_ADC_VDDA_OFFSET] < ADC_3V3_MAX) 	||
//		(adc_raw_data[RAW_ADC_6V_OFFSET]   < ADC_6V_MIN) 	||
//		(adc_raw_data[RAW_ADC_6V_OFFSET]   > ADC_6V_MAX) 	||
//		(adc_raw_data[RAW_ADC_1V8_OFFSET]  < ADC_1V8_MIN) 	||
//		(adc_raw_data[RAW_ADC_1V8_OFFSET]  > ADC_1V8_MAX)){
//		irq_flg.irq_POWER = 1;
//	}
	if((adc_raw_data[RAW_ADC_TEMP_OFFSET]>TemperatureHighThreshold)||
			(adc_raw_data[RAW_ADC_TEMP_OFFSET]<TemperatureLowThreshold)){
		irq_flg.irq_TEMP = 1;
	}


#if 0
	uint32_t raw_vdda_min;
	uint32_t raw_vdda_max;
	raw_vdda_min = ((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF)/3000;
	raw_vdda_max = ((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF)/3600;

#endif


}
static void ADC_start(void){
#define VDDA_RAW_AVERAGE_CNT		10
	uint32_t i;
	float VDDA_aver = 0;
	uint32_t ts_delta_cal = (int32_t)((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR);
	uint32_t delta_CAL_TEMP = (int32_t)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP);

	if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK){
		boot_BIT_status.ADC_Error = 1;
		Error_Handler();
	}
	HAL_Delay(1);
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_data,4) != HAL_OK){
		boot_BIT_status.ADC_Error = 1;
		Error_Handler();
	}

	main_status.ADC_dataReady = 0;
	while(main_status.ADC_dataReady == 0){};

	for(i=0; i<VDDA_RAW_AVERAGE_CNT; i++){
		main_status.ADC_dataReady = 0;
		while(main_status.ADC_dataReady == 0){};
		VDDA_aver += adc_raw_data[RAW_ADC_VDDA_OFFSET];
		__NOP();
	}
	VDDA_aver /= i;
	VDDA_aver += 0.5;
	VDDA_mv =  __HAL_ADC_CALC_VREFANALOG_VOLTAGE((uint32_t)VDDA_aver,ADC_RESOLUTION_12B);
	TemperatureLowThreshold =  ((pConfigStruct->temperature_min - TEMPSENSOR_CAL1_TEMP)*ts_delta_cal/delta_CAL_TEMP + (int32_t) *TEMPSENSOR_CAL1_ADDR)*TEMPSENSOR_CAL_VREFANALOG / VDDA_mv;
	TemperatureHighThreshold =  ((pConfigStruct->temperature_max - TEMPSENSOR_CAL1_TEMP)*ts_delta_cal/delta_CAL_TEMP + (int32_t) *TEMPSENSOR_CAL1_ADDR)*TEMPSENSOR_CAL_VREFANALOG / VDDA_mv;
//	HAL_ADC_Stop(&hadc1);
//	LL_ADC_ConfigAnalogWDThresholds	(ADC1, LL_ADC_AWD1,	HighThreshold,LowThreshold); // after this command ADC work incorrectly
//	HAL_ADC_Start(&hadc1);

	//check voltage
	mon_6V_mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_mv,adc_raw_data[RAW_ADC_6V_OFFSET],ADC_RESOLUTION_12B);
	mon_1V8_mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_mv,adc_raw_data[RAW_ADC_1V8_OFFSET],ADC_RESOLUTION_12B);
	boot_BIT_status.Voltage_err = 0;
	if( (VDDA_mv < ADC_3V3_MIN) 	||
		(VDDA_mv > ADC_3V3_MAX) 	||
		(mon_6V_mv   < ADC_6V_MIN) 	||
		(mon_6V_mv   > ADC_6V_MAX) 	||
		(mon_1V8_mv  < ADC_1V8_MIN) ||
		(mon_1V8_mv > ADC_1V8_MAX)){
		boot_BIT_status.Voltage_err = 1;
	}
	boot_BIT_status.Temp_err = 0;
	if((adc_raw_data[RAW_ADC_TEMP_OFFSET]>TemperatureHighThreshold)||
			(adc_raw_data[RAW_ADC_TEMP_OFFSET]<TemperatureLowThreshold)){
		boot_BIT_status.Temp_err = 1;
	}
}
/*
 * Implement write code, this is used by puts and printf.
 */
int _write(int file, char* ptr, int len){
	int i = 0;
	for(i = 0; i<len; i++){
		ITM_SendChar((*ptr++));
	}
	return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
