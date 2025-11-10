/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f2xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

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
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
//void HardFault_Handler(void)
//{
//  /* USER CODE BEGIN HardFault_IRQn 0 */
//
//  /* USER CODE END HardFault_IRQn 0 */
//  while (1)
//  {
//    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
//    /* USER CODE END W1_HardFault_IRQn 0 */
//  }
//}
void HardFault_Handler(void)
{
    __asm volatile
    (
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "b HardFault_HandlerC\n"
    );
}

void HardFault_HandlerC(uint32_t *stack)
{    volatile uint32_t r0  = stack[0];
volatile uint32_t r1  = stack[1];
volatile uint32_t r2  = stack[2];
volatile uint32_t r3  = stack[3];
volatile uint32_t r12 = stack[4];
volatile uint32_t lr  = stack[5];
volatile uint32_t pc  = stack[6];
volatile uint32_t psr = stack[7];

// inspect these in debugger
for(;;);
    // inspect these values
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

//__attribute__((naked)) void PendSV_Handler(void)
//{
//	__asm volatile(
//	        "cpsid   i                        \n" // Disable interrupts
//
//	        // if (OS_Curr != 0)
//	        "ldr     r0, =OS_Curr             \n" // r0 = &OS_Curr
//	        "ldr     r1, [r0]                 \n" // r1 = OS_Curr
//	        "cbz     r1, switch_restore       \n" // if null, skip save
//
//	        // Save r4–r11 on current task's stack
//	        "push    {r4-r11}                 \n"
//
//	        // OS_Curr->sp = sp
//	        "str     sp, [r1]                 \n"
//
//	        // label: switch_restore
//	        "switch_restore:                  \n"
//
//	        // sp = OS_Next->sp
//	        "ldr     r0, =OS_Next             \n"
//	        "ldr     r1, [r0]                 \n" // r1 = OS_Next
//	        "ldr     sp, [r1]                 \n" // SP = OS_Next->sp
//
//	        // OS_Curr = OS_Next
//	        "ldr     r0, =OS_Curr             \n"
//	        "str     r1, [r0]                 \n"
//
//	        // Restore r4–r11 from new task’s stack
//	        "pop     {r4-r11}                 \n"
//
//	        "cpsie   i                        \n" // Enable interrupts
//	        "bx      lr                       \n" // Return from exception
//	    );
//}
//
//
///**
//  * @brief This function handles System tick timer.
//  */
//void SysTick_Handler(void)
//{
//	OS_ticks();
//  /* USER CODE BEGIN SysTick_IRQn 0 */
//	__disable_irq();
//	OS_sched();
//	__enable_irq();
//  /* USER CODE END SysTick_IRQn 0 */
//   HAL_IncTick();
//  /* USER CODE BEGIN SysTick_IRQn 1 */
//
//  /* USER CODE END SysTick_IRQn 1 */
//}

/******************************************************************************/
/* STM32F2xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f2xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
