/*
 * EERTOS.h
 *
 *  Created on: Jan 23, 2020
 *      Author: u
 */

#ifndef INC_EERTOS_H_
#define INC_EERTOS_H_

#include "stm32f3xx_hal.h"


//частота проца
#define F_CPU 		72000000
//частота вызова таймера операционки
#define F_RTOS		1000
#define TimerTick	F_CPU/F_RTOS-1

//максимальное количество задач
#define	TaskQueueSize		12

//максимальное количество таймеров
#define MainTimerQueueSize	15


extern void InitRTOS(void);
extern void Idle(void);

typedef void (*TPTR)(void);

extern void SetTask(TPTR TS);
extern void SetTimerTask(TPTR TS,uint32_t NewTime);
extern void StopTask(TPTR TS);

extern void TaskManager(void);
extern void TimerService(void);
//void delay_ms (u32 time);

#endif /* INC_EERTOS_H_ */
