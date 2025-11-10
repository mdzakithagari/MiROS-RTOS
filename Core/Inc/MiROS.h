/*
 * MiROS.h
 *
 *  Created on: Nov 3, 2025
 *      Author: mdzak
 */

#ifndef INC_MIROS_H_
#define INC_MIROS_H_

#define LOG(x) (32 - __CLZ(x))

/* Thread Control Block TCB */
typedef struct{
	void *sp;
	uint32_t timeout;
	uint8_t prio;
}OSThread;

typedef void (*OSThredhandler)();

void OSThread_start (
			OSThread *me,
			uint8_t prio,
			OSThredhandler threadhandler,
			void *stcsto,uint32_t Stack_size);

void OS_ticks();
void OS_delay(uint32_t ticks);
void Idle_Task();
void OS_run();
void OS_sched(void);
void OS_init(void *stcsto,uint32_t Stack_size);

extern OSThread * volatile OS_Curr;
extern OSThread * volatile OS_Next;



#endif /* INC_MIROS_H_ */
