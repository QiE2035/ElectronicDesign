/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "mpu6050.h"
// #include "shell.h"

// #include <stdio.h>
// #include <string.h>

#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MPU6050_INT_Pin) {
    shellDisplay(shellGetCurrent(), "MPU6050 EXIT\r\n");
  }
} */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
// extern MPU6050_t mpu6050;
// extern SHELL_TypeDef shell;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1) {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line1 interrupt.
 */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BTN_RST_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
 * @brief This function handles EXTI line2 interrupt.
 */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BTN_PAUSE_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
 * @brief This function handles TIM1 update interrupt.
 */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
#define TIM_IRQ_TIME 10.0

#define TMP_OFFSET (-30)

// TODO: MIN MAX
#define SERVO_MIN_X 1300
#define SERVO_MID_X (1550 + TMP_OFFSET - 9)
#define SERVO_MAX_X 1800

#define SERVO_MIN_Y 1300
#define SERVO_MID_Y (1570 + TMP_OFFSET - 14)
#define SERVO_MAX_Y 1800

// #define Max(a, b) (((a) > (b)) ? (a) : (b))
#define Min(a, b) (((a) < (b)) ? (a) : (b))

int Clamp(int x, int min, int max)
{
  if (x <= min) {
    return min;
  } else if (x >= max) {
    return max;
  }
  return x;
}

int Lerp(int start, int end, float percent)
{
  return start + (end - start) * Min(percent, 1);
}

int servo_x = SERVO_MID_X,
    servo_y = SERVO_MID_Y;

int current_x = SERVO_MID_X,
    current_y = SERVO_MID_Y;

int last_x = SERVO_MID_X,
    last_y = SERVO_MID_Y;

int count = 0;

#define RESET_TIME  1500
#define SQUARE_TIME 1500

uint16_t move_time = 3000;

bool servo_finish = true;

#define SERVO_FINISH_PERCENT 1.0

#define Servo_Set_X(PWM)                            \
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, PWM); \
  current_x = PWM

#define Servo_Set_Y(PWM)                            \
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, PWM); \
  current_y = PWM

void Servo_Move()
{
  static float percent = 0;

  percent = ++count / (move_time / TIM_IRQ_TIME);

  if (percent >= SERVO_FINISH_PERCENT) {
    servo_finish = true;
  }

  Servo_Set_X(Clamp(Lerp(last_x, servo_x, percent),
                    SERVO_MIN_X, SERVO_MAX_X));
  Servo_Set_Y(Clamp(Lerp(last_y, servo_y, percent),
                    SERVO_MIN_Y, SERVO_MAX_Y));
}

enum {
  // STATE_IDLE,
  STATE_RESET,
  STATE_SQUARE,
  STATE_A4_RECT,
} state = STATE_RESET;

void Servo_Update(uint16_t x, uint16_t y, uint16_t time)
{
  last_x = current_x;
  last_y = current_y;

  servo_x = x;
  servo_y = y;

  move_time = time;

  count = 0;

  servo_finish = false;
}

#define SQUARE_COUNT 8

enum square_stage_enum {
  SQUARE_RESET,
  SQUARE_RST_LT,
  SQUARE_LT_RT,
  SQUARE_RT_RB,
  SQUARE_RB_LB,
  SQUARE_LB_LT,
  SQUARE_LT_LC,
  SQUARE_LT_RST,
} square_stage = SQUARE_RESET;
// bool square_next = false;

// #define TOP_OFFSET    (-30)
// #define BOTTOM_OFFSET (-30)
// #define LEFT_OFFSET   (-30)

const uint16_t SQUARE_PWM[][2] = {
    [SQUARE_RESET]  = {SERVO_MID_X, SERVO_MID_Y},
    [SQUARE_RST_LT] = {1670 + TMP_OFFSET - 1, 1420 + TMP_OFFSET - 13},
    [SQUARE_LT_RT]  = {1440 + TMP_OFFSET - 4, 1420 + TMP_OFFSET - 13},
    [SQUARE_RT_RB]  = {1440 + TMP_OFFSET - 4, 1710 + TMP_OFFSET - 13},
    [SQUARE_RB_LB]  = {1660 + TMP_OFFSET, 1710 + TMP_OFFSET - 13},
    [SQUARE_LB_LT]  = {1670 + TMP_OFFSET - 1, 1420 + TMP_OFFSET - 18},
    [SQUARE_LT_LC]  = {SERVO_MID_X, 1420 + TMP_OFFSET - 18},
    [SQUARE_LT_RST] = {SERVO_MID_X, SERVO_MID_Y},
};

#define TURN_WAIT 1000

#define X 0
#define Y 1

#define SQUARE_PWM_CUR SQUARE_PWM[square_stage]

void Servo_Reset()
{
  uint16_t servo_mid_x = SERVO_MID_X;

  square_stage = SQUARE_RESET;

  state = STATE_RESET;

  if (current_x < SERVO_MID_X) {
    servo_mid_x += 18;
  }
  Servo_Update(servo_mid_x, SERVO_MID_Y, RESET_TIME);
}

void Servo_Square()
{
  if (servo_finish) {
    // square_next = false;
    // TODO: TURN_WAIT
    // HAL_Delay(TURN_WAIT);
    if (++square_stage >= SQUARE_COUNT) {
      square_stage = SQUARE_RESET;

      // count = 0;
      Servo_Reset();
      return;
    }
    Servo_Update(SQUARE_PWM_CUR[X],
                 SQUARE_PWM_CUR[Y],
                 SQUARE_TIME);
    // square_stage++;
  }

  Servo_Move();
}

void Servo_A4()
{
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // static bool is_reset = false;
  if (htim == &htim1) {
    switch (state) {
      case STATE_RESET:
        // state = STATE_SQUARE;
        Servo_Move();
        break;
      case STATE_SQUARE:
        // is_reset = false;
        Servo_Square();
        break;
      default:
        Servo_Reset();
        break;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // HAL_Delay(TIM_IRQ_TIME);
  switch (GPIO_Pin) {
    case BTN_PAUSE_Pin:
      // if(state==STATE_RESET)
      state = STATE_SQUARE;
      break;
    case BTN_RST_Pin:
      Servo_Reset();
      break;
  }
}

/* USER CODE END 1 */
