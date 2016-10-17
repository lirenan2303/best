#include "stm32f4xx.h"
#include "time_plan.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stdio.h"
#include "sys_debug.h"

#define UPLOAD_TASK_STACK_SIZE			     (configMINIMAL_STACK_SIZE + 2*1024)



static void vTimePlanTask(void *parameter)
{
	for(;;)
	{
   
	}
}

void TimePlanInit(void)
{
  xTaskCreate(vTimePlanTask, "TimePlanTask", UPLOAD_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 10, NULL);
}
