/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_ll_dac.h"
#include "stm32f407xx.h"
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
extern float *wave;
extern int out1, out2;
extern int wav_sin[600];
extern int wav_square[600];
extern float wav_AKWF_0001[600];
extern float wav_AKWF_saw_0001[600];

int wavsq[600] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216, 16777216};
int TIM3_i;
float out;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
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
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
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
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
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

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  ///////////////////////__disable_irq();////////////////////////////////////////////////////////////////
  //__enable_irq();
                
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  
  //wave = wav_square;
  
  wave = wav_AKWF_saw_0001;
  out = wave[TIM3_i];
  if (out > 0){
    out1 = out*4096;
    out2 = 0;
  }
  else{
    out2 = (0-out)*4096;
    out1 = 0;
  }
  
  
  //out1 = 2100;
  //out2 = 3000;
  
    LL_DAC_ConvertDualData12RightAligned(DAC, out1, out2);
  
  TIM3_i++;
  if(TIM3_i >599){
    TIM3_i = 0;
  }
  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
