/*
 * Timer.c
 *
 *  Created on: 17 nov. 2021
 *      Author: Fernanda Galeana
 */

#include "Timer.h"

#define MAX_COUNTER_VALUE		200

osaEventId_t          mMyEvents;
/* Global Variable to store our TimerID */
tmrTimerID_t myTimerID = gTmrInvalidTimerID_c;

/* Handler ID for task */
osaTaskId_t gTimerHandler_ID;

/* OSA Task Definition*/
OSA_TASK_DEFINE(My_Task, gMyTaskPriority_c, 1, gMyTaskStackSize_c, FALSE);

/* Total time for the timer in ms */
tmrTimeInMilliseconds_t g_time;

uint8_t g_counter = 1;

/* Callback declaration */
static void myTaskTimerCallback(void *param);

/* Main custom task */
void My_Task(osaTaskParam_t argument)
{
 osaEventFlags_t customEvent;
 myTimerID = TMR_AllocateTimer();

 while(1)
 {
	 OSA_EventWait(mMyEvents, osaEventFlagsAll_c, FALSE, osaWaitForever_c,&customEvent);
	  if( !gUseRtos_c && !customEvent)
	  {
	   break;
	  }
	  /* Depending on the received event */
	  switch(customEvent){
	  case gEvent1StartTimer_c:
		  TMR_StartIntervalTimer(myTimerID, g_time, myTaskTimerCallback,NULL);
		  break;
	  case gEvent2CallbackTimer_c: /* Event called from myTaskTimerCallback */
		  if(MAX_COUNTER_VALUE > g_counter)
		  {
			  g_counter++;
		  }
		  else
		  {
			  g_counter = 1;
		  }
		  break;
	  case gEvent3StopTimer_c: 	/* Event to stop the timer */
		  TMR_StopTimer(myTimerID);
		  break;
	  default:
		  break;
  }
 }
}
/* Function to init the task */
void MyTask_Init(void)
{
	mMyEvents = OSA_EventCreate(TRUE);
	/* The instance of the MAC is passed at task creaton */
	gTimerHandler_ID = OSA_TaskCreate(OSA_TASK(My_Task), NULL);
}
/* This is the function called by the Timer each time it expires */
static void myTaskTimerCallback(void *param)
{
	OSA_EventSet(mMyEvents, gEvent2CallbackTimer_c);
}
/* Public function to send an event to stop the timer */
void MyTaskTimer_Stop(void)
{
	OSA_EventSet(mMyEvents, gEvent3StopTimer_c);
}
void MyTaskTimer_Start(tmrTimeInMilliseconds_t time)
{
	g_time = time;
	OSA_EventSet(mMyEvents, gEvent1StartTimer_c);
}
uint8_t GetCounter(void)
{
	return(g_counter);
}
void RestartCounter(void)
{
	g_counter = 0;
}
