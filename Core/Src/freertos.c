/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// FreeRTOS Hook Functions for debugging
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  // Stack overflow detected - blink RED LED rapidly
  (void)xTask;
  (void)pcTaskName;
  
  __disable_irq();
  while(1) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);  // RED LED
    for(volatile uint32_t i=0; i<200000; i++);
  }
}

void vApplicationMallocFailedHook(void)
{
  // Memory allocation failed - blink RED LED very rapidly
  __disable_irq();
  while(1) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);  // RED LED
    for(volatile uint32_t i=0; i<100000; i++);
  }
}

void vApplicationIdleHook(void)
{
  // Idle task is running - everything OK
  // Can add power saving code here later
}

/* USER CODE END Application */

