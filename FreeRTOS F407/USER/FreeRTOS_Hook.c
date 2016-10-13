#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "sys_debug.h"

void vApplicationTickHook(void)
{
	
}

void vApplicationStackOverflowHook( xTaskHandle xTask, signed char *pcTaskName )
{
	char buf[30];
	sprintf(buf, "vApplicationStackOverflowHook: %s\n" , pcTaskGetTaskName(xTaskGetCurrentTaskHandle()));
	printf_str(buf);
	while(1);
}
