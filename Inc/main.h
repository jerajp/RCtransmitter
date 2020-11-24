/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define BUTTONTHRESHOLD 10
#define BATTAVERAGETIME 50 //50 msec average
#define TXPERIOD 20 	  //20ms -50 MSG/sec
#define MINMSGPERSEC 10   //min 10 msg per second

#define MINREMOTEBATT 3000
#define MINDRONEBATT  10500

//JOYSTICK END POINTS
#define LJOYXMINDEF 440
#define LJOYXMAXDEF 4020
#define LJOYYMINDEF 15
#define LJOYYMAXDEF 3780

#define RJOYXMINDEF 70
#define RJOYXMAXDEF 3790
#define RJOYYMINDEF 300
#define RJOYYMAXDEF 4080

//FLASH constants
#define FLASHCONSTADDR 0x800FC00
#define CONTROLWORD 7 //control word to check if Flash constants are present

#define TRIMJOYSTEP 5

//Menu data
typedef enum {LVL0,LVL1}MenuLvL;
typedef enum {MainScreen, MenuScreen1,MSGScreen1,ButtonScreen1,ButtonScreen2,ButtonScreen3,TestScreen1,FlashDataScreenRd,FlashDataScreenWr, CommandScreen1 }LCDScreen;
typedef enum {Line0,Line1,Line2,Line3,Line4,Line5}CursorPositions;


struct FlashDatastruct
{
	uint32_t controlData;
	uint8_t LCD_contrast;
};

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE BEGIN 4 */

void WriteFlashData(uint32_t flashstartaddr, struct FlashDatastruct *p);
void ReadFlashData(uint32_t flashstartaddr, struct FlashDatastruct *p);
void EraseFlashData(uint32_t StartAddr);
uint32_t CheckFlashData(uint32_t StartAddr);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define AD1_0_LX_Pin GPIO_PIN_0
#define AD1_0_LX_GPIO_Port GPIOA
#define AD1_1_LY_Pin GPIO_PIN_1
#define AD1_1_LY_GPIO_Port GPIOA
#define AD1_2_DX_Pin GPIO_PIN_2
#define AD1_2_DX_GPIO_Port GPIOA
#define AD1_3_DY_Pin GPIO_PIN_3
#define AD1_3_DY_GPIO_Port GPIOA
#define AD1_4_P1_Pin GPIO_PIN_4
#define AD1_4_P1_GPIO_Port GPIOA
#define AD1_8_P2_Pin GPIO_PIN_0
#define AD1_8_P2_GPIO_Port GPIOB
#define AD1_9_BATT_Pin GPIO_PIN_1
#define AD1_9_BATT_GPIO_Port GPIOB
#define SPI_NR24_IRQ_Pin GPIO_PIN_2
#define SPI_NR24_IRQ_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_12
#define LCD_RST_GPIO_Port GPIOB
#define LCD_CLK_Pin GPIO_PIN_13
#define LCD_CLK_GPIO_Port GPIOB
#define LCD_CE_Pin GPIO_PIN_14
#define LCD_CE_GPIO_Port GPIOB
#define LCD_DATA_Pin GPIO_PIN_15
#define LCD_DATA_GPIO_Port GPIOB
#define LCD_COMM_Pin GPIO_PIN_8
#define LCD_COMM_GPIO_Port GPIOA
#define TOGG1_Pin GPIO_PIN_9
#define TOGG1_GPIO_Port GPIOA
#define TOGG2_Pin GPIO_PIN_10
#define TOGG2_GPIO_Port GPIOA
#define TOGG3_Pin GPIO_PIN_11
#define TOGG3_GPIO_Port GPIOA
#define TOGG4_Pin GPIO_PIN_12
#define TOGG4_GPIO_Port GPIOA
#define SPI_NR24_CE_Pin GPIO_PIN_15
#define SPI_NR24_CE_GPIO_Port GPIOA
#define TOGG5_Pin GPIO_PIN_3
#define TOGG5_GPIO_Port GPIOB
#define TOGG6_Pin GPIO_PIN_4
#define TOGG6_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOB
#define SPI_NR24_CSN_Pin GPIO_PIN_6
#define SPI_NR24_CSN_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define LED1ON HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET)
#define LED1OFF HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET)

#define LED2ON HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET)
#define LED2OFF HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET)

#define LED3ON HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_SET)
#define LED3OFF HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,GPIO_PIN_RESET)

#define LED4ON HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET)
#define LED4OFF HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_RESET)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
