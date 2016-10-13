#include "FreeRTOS.h"
#include "task.h"
#include "misc.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_rcc.h"
#include "electric.h"
#include "time_plan.h"
#include "sys_debug.h"
#include "gsm.h"
#include "ballast_Comm.h"
#include "rtc.h"
#include "delay.h"


extern void WatchdogInit(void);
extern void NorFlashInit(void);
//extern void RtcInit(void);

int main(void)
{
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组2
 //  WatchdogInit();
    NorFlashInit();
    RtcInit(); 
    UartDebugInit();
  	GSMInit();
    BallastCommInit();
    ElectricInit();
    TimePlanInit();

  //printf("\n==============================\n");
  //printf("%s", Version());
  //printf("\n==============================\n");
  vTaskStartScheduler();
	return 0;
}

