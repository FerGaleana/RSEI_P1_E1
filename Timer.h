/*
 * Timer.h
 *
 *  Created on: 17 nov. 2021
 *      Author: ferna
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "TimersManager.h"
#include "FunctionLib.h"
#include "LED.h"
/* KSDK */
#include "fsl_common.h"
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"


#define ASCII_NUMBERS_OFFSET	48
/* Define the available Task's Events */
#define gEvent1StartTimer_c (1 << 0)
#define gEvent2CallbackTimer_c (1 << 1)
#define gEvent3StopTimer_c (1 << 2)

#define gMyTaskPriority_c 3
#define gMyTaskStackSize_c 400

void My_Task(osaTaskParam_t argument);
void MyTaskTimer_Start(tmrTimeInMilliseconds_t time);
void MyTaskTimer_Stop(void);
void MyTask_Init(void (*pFunc)(void));

#endif /* TIMER_H_ */
