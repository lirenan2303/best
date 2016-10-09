#include "time_plan.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define UPLOAD_TASK_STACK_SIZE			     (configMINIMAL_STACK_SIZE + 1024 * 10)

static void vTimePlanTask(void *parameter)
{
	
}

void TimePlanInit(void)
{
  xTaskCreate(vTimePlanTask, "TimePlanTask", UPLOAD_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
}
