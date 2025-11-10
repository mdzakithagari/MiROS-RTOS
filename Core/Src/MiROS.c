/*
 * MiROS.c
 *
 *  Created on: Nov 3, 2025
 *      Author: mdzak
 */
#include "main.h"
#include "MiROS.h"
OSThread * volatile OS_Curr = 0;
OSThread * volatile OS_Next = 0;
OSThread *OS_Threads [32 + 1];
uint32_t OS_readySet;
uint32_t OS_delayedSet;
OSThread Idle_handler;


void OSThread_start (
			OSThread *me,
			uint8_t prio,
			OSThredhandler threadhandler,
			void *stcsto,uint32_t Stack_size)
{
	uint32_t *sp = (uint32_t *)(((uint32_t)&stcsto[Stack_size]) & ~0x7U);
	uint32_t *stk_limit;

    *(--sp) = 0x01000000;              // xPSR  (Thumb bit)
    *(--sp) = (uint32_t)threadhandler;          // PC    (entry point)
    *(--sp) = 0xFFFFFFFD;              // LR    (return to Thread mode using PSP)
    *(--sp) = 0x12121212;              // R12
    *(--sp) = 0x03030303;              // R3
    *(--sp) = 0x02020202;              // R2
    *(--sp) = 0x01010101;              // R1
    *(--sp) = 0xDEADBEEF;         // R0

    // then space for R4–R11 (callee-saved)
    *(--sp) = 0x11111111;              // R11
    *(--sp) = 0x10101010;              // R10
    *(--sp) = 0x09090909;              // R9
    *(--sp) = 0x08080808;              // R8
    *(--sp) = 0x07070707;              // R7
    *(--sp) = 0x06060606;              // R6
    *(--sp) = 0x05050505;              // R5
    *(--sp) = 0x04040404;              // R4              /* R4 */


	me->sp = sp;

//	/* round up the bottom of the stack to the 8-byte boundary */
//	stk_limit = (uint32_t *)(((((uint32_t)stcsto - 1U) / 8U) + 1U) * 8U);
//
//	/* pre-fill the unused part of the stack with 0xDEADBEEF */
//
//    for (uint32_t *p = sp - 1U; p >= stk_limit; --p) {
//        *p = 0xDEADBEEFU;
//    }


	 OS_Threads[prio] = me;

  me->prio = prio;
	if(prio > 0)
	{
		OS_readySet |= (1 << (prio - 1));
	}
}


void OS_init(void *stcsto,uint32_t Stack_size)
{
	    /* set the PendSV interrupt priority to the lowest level */
	    *(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16);
	    OSThread_start(&Idle_handler, 0 , &Idle_Task, stcsto, Stack_size);
}

void OS_sched(void) {

	if(OS_readySet == 0)
	{
		OS_Next = OS_Threads[0];
	}
	else
	{
		OS_Next = OS_Threads[LOG(OS_readySet)];
	}


    /* OS_next = ... */
    if (OS_Next != OS_Curr)
    {
    	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;   // Set the PendSV pending bit
    }
}

void OS_run()
{
	HAL_InitTick(TICK_INT_PRIORITY);

	__disable_irq();
	OS_sched();
	__enable_irq();

}

void Idle_Task(void)
{
    while (1)
    {
    	__WFI();
    }
}


void OS_delay(uint32_t ticks)
{
	__disable_irq();
	OS_Curr->timeout = ticks;
	OS_readySet &= ~(1 << (OS_Curr->prio - 1 ));
	OS_delayedSet |= (1 << (OS_Curr->prio - 1 ));
	OS_sched();
	__enable_irq();
}

void OS_ticks()
{
	  uint32_t workingSet = OS_delayedSet;

	    while (workingSet != 0U) {
	        OSThread *t = OS_Threads[LOG(workingSet)];
	        uint32_t bit;

	        bit = (1U << (t->prio - 1U));
	        --t->timeout;

	        if (t->timeout == 0U) {
	            OS_readySet  |= bit;   /* insert to ready set */
	            OS_delayedSet &= ~bit; /* remove from delayed set */
	        }

	        workingSet &= ~bit;        /* remove from working set */
	    }
}
__attribute__((naked)) void PendSV_Handler(void)
{
	__asm volatile(
	        "cpsid   i                        \n" // Disable interrupts

	        // if (OS_Curr != 0)
	        "ldr     r0, =OS_Curr             \n" // r0 = &OS_Curr
	        "ldr     r1, [r0]                 \n" // r1 = OS_Curr
	        "cbz     r1, switch_restore       \n" // if null, skip save

	        // Save r4–r11 on current task's stack
	        "push    {r4-r11}                 \n"

	        // OS_Curr->sp = sp
	        "str     sp, [r1]                 \n"

	        // label: switch_restore
	        "switch_restore:                  \n"

	        // sp = OS_Next->sp
	        "ldr     r0, =OS_Next             \n"
	        "ldr     r1, [r0]                 \n" // r1 = OS_Next
	        "ldr     sp, [r1]                 \n" // SP = OS_Next->sp

	        // OS_Curr = OS_Next
	        "ldr     r0, =OS_Curr             \n"
	        "str     r1, [r0]                 \n"

	        // Restore r4–r11 from new task’s stack
	        "pop     {r4-r11}                 \n"

	        "cpsie   i                        \n" // Enable interrupts
	        "bx      lr                       \n" // Return from exception
	    );
}


/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
	OS_ticks();
  /* USER CODE BEGIN SysTick_IRQn 0 */
	__disable_irq();
	OS_sched();
	__enable_irq();
  /* USER CODE END SysTick_IRQn 0 */
   HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}
