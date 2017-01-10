#include "FreeRTOS.h"
#include "stdio.h"
#include "task.h"
#include "misc.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_rcc.h"
#include "electric.h"
#include "time_plan.h"
#include "sys_debug.h"
#include "gsm.h"
#include "ballast_Comm.h"
#include "gateway_protocol.h"
#include "ballast_protocol.h"
#include "table_process.h"
#include "rtc.h"
#include "delay.h"
#include "km_ctrl.h"


extern void WatchdogInit(void);
extern void NorFlashInit(void);
	
const char Sofeware_Version[] = "Software Version: V1.01  \r\n";
const char Complie_Time[] = "Compile Date: " __DATE__ "  " __TIME__;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
//  WatchdogInit();
	NorFlashInit();
	RtcInit();
	UartDebugInit();
	GSMInit();
	BallastCommInit();
	ElectricInit();
	TimePlanInit();
	KM_CtrlInit();
	AllParaInit();
	AllTableInit();

//  printf_str("\r\n/********************************************************/\r\n"); 
//  printf_str((char*)Sofeware_Version);
//	printf_str((char*)Complie_Time);
//  printf_str("\r\n/****************  COPYRIGHT  2016  DMKJ  ***************/\r\n");
  vTaskStartScheduler();
	
	return 0;
}

