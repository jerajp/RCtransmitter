/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "nokia5110_LCD.h"
#include "nrf24.h"
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

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint32_t test1;
uint32_t test2;
uint32_t test3;
uint32_t test4;
uint32_t wifiOK;

uint32_t watch1;
uint32_t watch2;
uint32_t watch3;
uint32_t watch4;
uint32_t watch5;
uint32_t watch6;


char stringlcdBuffer[20];


extern uint32_t T1statusdebounce;
extern uint32_t T2statusdebounce;
extern uint32_t T3statusdebounce;
extern uint32_t T4statusdebounce;
extern uint32_t TDstatusdebounce;
extern uint32_t TLstatusdebounce;
extern uint32_t TOGGLstatusdebounce;
extern uint32_t TOGGDstatusdebounce;

uint16_t adcDataArray[7];

extern uint32_t Batt1cellAVG;


extern uint32_t LjoyUPDOWN;
extern uint32_t LjoyLEFTRIGHT;
extern uint32_t DjoyUPDOWN;
extern uint32_t DjoyLEFTRIGHT;

extern uint32_t offsetLjoyUPDOWN;
extern uint32_t offsetLjoyLEFTRIGHT;
extern uint32_t offsetDjoyUPDOWN;
extern uint32_t offsetDjoyLEFTRIGHT;

extern uint32_t potenc1;
extern uint32_t potenc2;

uint32_t LCDMenu=0;
uint32_t LCDMenuHist=0;

//NRF24
uint8_t nRF24_payloadTX[32]; //TX buffer
uint8_t nRF24_payloadRX[32]; //RX buffer
const uint8_t nRF24_ADDR[3] = {5, 3, 5 }; //Address
uint8_t RXstpaketov=0;

extern uint32_t TXdelay;
extern uint8_t Buttons;
extern uint32_t DroneBattUpperByte;
extern uint32_t DroneBattLowerByte;
extern uint32_t DroneBattmV;

extern uint32_t TotalMSGsend;
extern uint32_t TotalMSGrecv;
extern uint32_t MSGprerSecond;
extern uint32_t MSGLowCount;

uint32_t MainInitDoneFlag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

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

	//test timings DWT counter
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= 1;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_ADC_Start_DMA(&hadc1,adcDataArray, 7);

  //lcd init pins
  LCD_setRST(LCD_RST_GPIO_Port, LCD_RST_Pin);
  LCD_setCE(LCD_CE_GPIO_Port, LCD_CE_Pin);
  LCD_setDC(LCD_COMM_GPIO_Port, LCD_COMM_Pin);
  LCD_setDIN(LCD_DATA_GPIO_Port, LCD_DATA_Pin);
  LCD_setCLK(LCD_CLK_GPIO_Port, LCD_CLK_Pin);

  LCD_init(); //6 lines 15 characters max per line (0,0), (0,1), (0,2), (0,3), (0,4), (0,5) line starts

  //NRF24INIT
  SPI1->CR1|=SPI_CR1_SPE; //enable SPI


  nRF24_CE_L(); // RX/TX disabled

  wifiOK=nRF24_Check();

  nRF24_Init(); //Default init

  // Disable ShockBurst for all RX pipes
  nRF24_DisableAA(0xFF);

  // Set RF channel
  nRF24_SetRFChannel(15); //2400Mhz + 15Mhz

  // Set data rate
  nRF24_SetDataRate(nRF24_DR_250kbps);

  // Set CRC scheme
  nRF24_SetCRCScheme(nRF24_CRC_1byte);

  // Set address width, its common for all pipes (RX and TX)
  nRF24_SetAddrWidth(3);

  nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program address for RX pipe #1
  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 2); // Auto-ACK: disabled, payload length: 2 bytes

  nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); // program TX address

  // Set TX power
  nRF24_SetTXPower(nRF24_TXPWR_6dBm);

  // Set operational mode (PTX == transmitter)
  nRF24_SetOperationalMode(nRF24_MODE_TX);

  // Clear any pending IRQ flags
  nRF24_ClearIRQFlags();

  // Wake the transceiver
  nRF24_SetPowerMode(nRF24_PWR_UP);

  MainInitDoneFlag=1;

  //Zeroing Joysticks on startup------------------------------------------------
  offsetLjoyUPDOWN=2047-adcDataArray[0];
  offsetLjoyLEFTRIGHT=2047-adcDataArray[1];
  offsetDjoyUPDOWN=2047-adcDataArray[2];
  offsetDjoyLEFTRIGHT=2047-adcDataArray[3];


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(LCDMenuHist!=LCDMenu)LCD_clrScr();
	  LCDMenuHist=LCDMenu;

	  switch(LCDMenu)
	  {
	  	  case 0:{	  //Main menu
		  	  	  	  sprintf(stringlcdBuffer,"Battery");
		  	  	  	  LCD_print(stringlcdBuffer,0,0);

	  		  	  	  sprintf(stringlcdBuffer,"RC: %u mV",Batt1cellAVG);
	  		  	  	  LCD_print(stringlcdBuffer,0,1);

	  		  	  	  sprintf(stringlcdBuffer,"DR: %u mV",DroneBattmV);
	  		  	  	  LCD_print(stringlcdBuffer,0,2);

	  	  	  	 }break;

	  	  case 1:{
	  		  	  	  //Diagnostics Connection
	  		  	  	  sprintf(stringlcdBuffer,"Total MSG");
	  	  	  	  	  LCD_print(stringlcdBuffer,0,0);

	  	  	  	  	  sprintf(stringlcdBuffer,"Send %u",TotalMSGsend);
	  	  	  	  	  LCD_print(stringlcdBuffer,0,1);

	  	  	  	  	  sprintf(stringlcdBuffer,"Recv %u",TotalMSGrecv);
	  	  	  	  	  LCD_print(stringlcdBuffer,0,2);

	  	  	  	  	  sprintf(stringlcdBuffer,"MSG/s: %u",MSGprerSecond);
	  	  	  	  	  LCD_print(stringlcdBuffer,0,3);

	  		  	  	  sprintf(stringlcdBuffer,"RClow[s]: %u",MSGLowCount);
	  		  	  	  LCD_print(stringlcdBuffer,0,4);

	  	  	  	 }break;


	  	  case 2:{
	  		  	  	  //Diagnostics Inputs
	  		  	  	  sprintf(stringlcdBuffer,"INPUTS");
	  			  	  LCD_print(stringlcdBuffer,0,0);

	  		  	  	  sprintf(stringlcdBuffer,"Buttons:");
		  			  LCD_print(stringlcdBuffer,0,1);

		  			  sprintf(stringlcdBuffer,"%u%u%u%u %u%u %u%u",T1statusdebounce,T2statusdebounce,T3statusdebounce,T4statusdebounce,TLstatusdebounce,TDstatusdebounce,TOGGLstatusdebounce,TOGGDstatusdebounce);
		  			  LCD_print(stringlcdBuffer,0,2);

	  		  	  	  sprintf(stringlcdBuffer,"Pot: %u %u",potenc1,potenc2);
		  			  LCD_print(stringlcdBuffer,0,3);

		  			  sprintf(stringlcdBuffer,"%u %u %u %u  ",LjoyUPDOWN,LjoyLEFTRIGHT,DjoyUPDOWN,DjoyLEFTRIGHT);
		  			  LCD_print(stringlcdBuffer,0,4);

		  			  sprintf(stringlcdBuffer,"%d %d %d %d  ",offsetLjoyUPDOWN,offsetLjoyLEFTRIGHT,offsetDjoyUPDOWN,offsetDjoyLEFTRIGHT);
		  			  LCD_print(stringlcdBuffer,0,5);

	  	  	  	 }break;

	  }

	  //test1=DWT->CYCCNT;
	  //test2=DWT->CYCCNT-test1;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_7;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_NR24_CE_Pin|CPI_NR24_CSN_Pin|LCD_RST_Pin|LCD_CLK_Pin 
                          |LCD_CE_Pin|LCD_DATA_Pin|LED3_Pin|LED4_Pin 
                          |LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_COMM_GPIO_Port, LCD_COMM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DT_Pin LT_Pin */
  GPIO_InitStruct.Pin = DT_Pin|LT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_NR24_CE_Pin CPI_NR24_CSN_Pin LCD_RST_Pin LED3_Pin 
                           LED4_Pin LED1_Pin */
  GPIO_InitStruct.Pin = SPI_NR24_CE_Pin|CPI_NR24_CSN_Pin|LCD_RST_Pin|LED3_Pin 
                          |LED4_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TOGGD_Pin TOGGL_Pin */
  GPIO_InitStruct.Pin = TOGGD_Pin|TOGGL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CLK_Pin LCD_CE_Pin LCD_DATA_Pin */
  GPIO_InitStruct.Pin = LCD_CLK_Pin|LCD_CE_Pin|LCD_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_COMM_Pin */
  GPIO_InitStruct.Pin = LCD_COMM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_COMM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : T1_Pin T2_Pin T3_Pin T4_Pin */
  GPIO_InitStruct.Pin = T1_Pin|T2_Pin|T3_Pin|T4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF24_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF24_IRQ_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
