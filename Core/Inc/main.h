/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
typedef enum {
  false,
  true
} bool;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define Servo_Set_X(PWM) __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, PWM)
#define Servo_Set_Y(PWM) __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, PWM)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_RST_Pin GPIO_PIN_1
#define BTN_RST_GPIO_Port GPIOA
#define BTN_RST_EXTI_IRQn EXTI1_IRQn
#define BTN_PAUSE_Pin GPIO_PIN_2
#define BTN_PAUSE_GPIO_Port GPIOA
#define BTN_PAUSE_EXTI_IRQn EXTI2_IRQn
#define SERVO_X_Pin GPIO_PIN_15
#define SERVO_X_GPIO_Port GPIOA
#define SERVO_Y_Pin GPIO_PIN_3
#define SERVO_Y_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
