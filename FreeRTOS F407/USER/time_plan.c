#include "stm32f4xx.h"
#include "time_plan.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stdio.h"
#include "sys_debug.h"

#define UPLOAD_TASK_STACK_SIZE			     (configMINIMAL_STACK_SIZE + 1024)



static void vTimePlanTask(void *parameter)
{
	char tbuf[40];
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;
	
	for(;;)
	{
		RTC_GetTime(RTC_Format_BIN,&RTC_TimeStruct);
		sprintf((char*)tbuf,"Time:%02d:%02d:%02d",RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds); 
		printf_str(tbuf);
		
		RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);
    sprintf((char*)tbuf,"Date:20%02d-%02d-%02d",RTC_DateStruct.RTC_Year,RTC_DateStruct.RTC_Month,RTC_DateStruct.RTC_Date); 
		printf_str(tbuf);
		
		vTaskDelay(configTICK_RATE_HZ);
	}
	
}

void TimePlanInit(void)
{
  xTaskCreate(vTimePlanTask, "TimePlanTask", UPLOAD_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY + 10, NULL);
}
