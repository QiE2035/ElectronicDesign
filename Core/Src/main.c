/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "shell.h"
// #include <stdio.h>
// #include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_START 0x2035
#define UART_END   0x5302
#define UART_SIZE  2

#define OFFSET_MAX 2

// #define PWM_X TIM_CHANNEL_1
// #define PWM_Y TIM_CHANNEL_2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// SHELL_TypeDef shell;
uint16_t uart_data;

union {
#pragma pack(1)
  struct {
    uint16_t channel;
    uint16_t pwm;
  };
  uint16_t data[3];
} frame;

uint8_t offset = 0;

bool recive = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(huart, &uart_data, UART_SIZE);
  if (&huart1 == huart) {
    // shellInput(&shell, uart_data);
    if (offset > OFFSET_MAX) {
      offset = 0;
      return;
    }

    if (uart_data == UART_START && !recive) {
      recive = true;
      offset = 0;
      return;
    } else if (uart_data == UART_END) {
      if (offset != OFFSET_MAX) {
        recive = false;
        offset = 0;
        return;
      }
      recive = false;
      offset = 0;
      // char tmp[50];
      // sprintf(tmp, "%d, %d\r\n", frame.channel, frame.pwm);

      // HAL_UART_Transmit(huart, tmp, strlen(tmp), UINT32_MAX);
      __HAL_TIM_SetCompare(&htim2, frame.channel, frame.pwm);
    } else if (recive) {
      frame.data[offset++] = uart_data;
    }
  }
}

// void Shell_Write(const char data)
// {
//   HAL_UART_Transmit(&huart1, &data, 1, UINT32_MAX);
// }

// void PWM_X(uint16_t pwm)
// {
//   // shellPrint(&shell,"设置X轴PWM: %d\r\n", pwm);
//   __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm);
// }
// SHELL_EXPORT_CMD(pwm_x, PWM_X, "设置X轴PWM");

// void PWM_Y(uint16_t pwm)
// {
//   // shellPrint(&shell,"设置Y轴PWM: %d\r\n", pwm);
//   __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm);
// }
// SHELL_EXPORT_CMD(pwm_y, PWM_Y, "设置Y轴PWM");

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1565);
  // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1565);
  // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1565);
  // __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1565);

  // shell.write = Shell_Write;
  // shellInit(&shell);

  HAL_UART_Receive_IT(&huart1, &uart_data, UART_SIZE);
  // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 2200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // HAL_UART_Transmit(&huart1, "Hello World!\r\n", 15, UINT32_MAX);
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
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
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
