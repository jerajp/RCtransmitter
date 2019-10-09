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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define DT_Pin GPIO_PIN_14
#define DT_GPIO_Port GPIOC
#define LT_Pin GPIO_PIN_15
#define LT_GPIO_Port GPIOC
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
#define SPI_NR24_CE_Pin GPIO_PIN_2
#define SPI_NR24_CE_GPIO_Port GPIOB
#define CPI_NR24_CSN_Pin GPIO_PIN_10
#define CPI_NR24_CSN_GPIO_Port GPIOB
#define TOGG2_Pin GPIO_PIN_11
#define TOGG2_GPIO_Port GPIOB
#define TOGG1_Pin GPIO_PIN_12
#define TOGG1_GPIO_Port GPIOB
#define SPI_CE_LCD_Pin GPIO_PIN_14
#define SPI_CE_LCD_GPIO_Port GPIOB
#define LCDCOMAND_Pin GPIO_PIN_8
#define LCDCOMAND_GPIO_Port GPIOA
#define T1_Pin GPIO_PIN_9
#define T1_GPIO_Port GPIOA
#define T2_Pin GPIO_PIN_10
#define T2_GPIO_Port GPIOA
#define T3_Pin GPIO_PIN_11
#define T3_GPIO_Port GPIOA
#define T4_Pin GPIO_PIN_12
#define T4_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOB
#define LED_4_Pin GPIO_PIN_5
#define LED_4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
