/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POTEN_1_Pin GPIO_PIN_0
#define POTEN_1_GPIO_Port GPIOA
#define POTEN_2_Pin GPIO_PIN_1
#define POTEN_2_GPIO_Port GPIOA
#define TM1637_CLK_Pin GPIO_PIN_5
#define TM1637_CLK_GPIO_Port GPIOA
#define TM1637_DIO_Pin GPIO_PIN_7
#define TM1637_DIO_GPIO_Port GPIOA
#define M_A_SW_Pin GPIO_PIN_0
#define M_A_SW_GPIO_Port GPIOB
#define M_A_SW_EXTI_IRQn EXTI0_IRQn
#define Retract_SW_Pin GPIO_PIN_1
#define Retract_SW_GPIO_Port GPIOB
#define Retract_SW_EXTI_IRQn EXTI1_IRQn
#define Rotary_SW_1_Pin GPIO_PIN_2
#define Rotary_SW_1_GPIO_Port GPIOB
#define Rotary_SW_1_EXTI_IRQn EXTI2_IRQn
#define Rotary_SW_2_Pin GPIO_PIN_10
#define Rotary_SW_2_GPIO_Port GPIOB
#define Rotary_SW_2_EXTI_IRQn EXTI15_10_IRQn
#define Rotary_SW_3_Pin GPIO_PIN_11
#define Rotary_SW_3_GPIO_Port GPIOB
#define Rotary_SW_3_EXTI_IRQn EXTI15_10_IRQn
#define Foot_SW_Pin GPIO_PIN_12
#define Foot_SW_GPIO_Port GPIOB
#define Foot_SW_EXTI_IRQn EXTI15_10_IRQn
#define BTN_Side_Pin GPIO_PIN_13
#define BTN_Side_GPIO_Port GPIOB
#define BTN_Side_EXTI_IRQn EXTI15_10_IRQn
#define nSLEEP_Pin GPIO_PIN_12
#define nSLEEP_GPIO_Port GPIOA
#define nFAULT_Pin GPIO_PIN_15
#define nFAULT_GPIO_Port GPIOA
#define nFAULT_EXTI_IRQn EXTI15_10_IRQn
#define M_PWM1_Pin GPIO_PIN_3
#define M_PWM1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
